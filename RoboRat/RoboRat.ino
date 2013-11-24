#include <AFMotor.h>
#include <Servo.h>
#include "RingBuffer.h"
#include "ControlLoop.h"
#include "LineSensor.h"
#include "States.h"
#include "Chains.h"
#include <cmath>
#include <cstdio>


// Pin assignments
const int rightInterruptPin = 2;
const int leftInterruptPin = 3;
const int bumpSensorPin = A0;
const int rightLineSensorPin = A1;
const int leftLineSensorPin = A2;
const int rightDistanceSensorPin = A3;
const int leftDistanceSensorPin = A4;
const int lightSensorPin = A5;
const int elevationServoPin = 9;
const int clawServoPin = 10;
const int selectButtonPin = 11;
const int scrollButtonPin = 13;

// Initialize motors
AF_DCMotor rightMotor(4, MOTOR34_1KHZ);
AF_DCMotor leftMotor(3, MOTOR34_1KHZ);

// Servos
Servo elevationServo;
Servo clawServo;
const int clawOpen = 180;
const int clawClosed = 0;
const int elevationRaised = 0;
const int elevationLowered = 180;

// Encoder variables
struct EncoderData
{
    EncoderData() { count = 0; filteredVelocity = 0.f; }
    void reset() { count = 0; filteredVelocity = 0.f; pulseTimes.clear(); }
    volatile long count;
    volatile RingBuffer<unsigned long, 6> pulseTimes;
    bool forward;
    float filteredVelocity;
    static const float filterConstant = 0.15f;
};
EncoderData rightData;
EncoderData leftData;
const unsigned long debounceMicros = 250;

// Physical parameters
const float wheelCircumference = 0.216f; // meters
const long pulsesPerRev = 6 * 3 * 3 * 5;
const float ticksPerSecond = 1000000.f;
const float wheelOffset = 0.092f; // meters

// Loop control
unsigned long lastMillis = 0;
unsigned int loopPeriodMs = 20;
const float dt = 0.020f; // Should be kept in sync with loopPeriodMs
unsigned int cycleCount = 0;
const unsigned int displayUpdateCycles = 10;

// Motor PID data
float rightVelocityCommand = 0.f;
float leftVelocityCommand = 0.f;
ControlLoop rightVelocityLoop(dt);
ControlLoop leftVelocityLoop(dt);
float rightControlFilter = 0.f;
float leftControlFilter = 0.f;
const float controlFilterConstant = 0.3f;
float rightControlFilter2 = 0.f;
float leftControlFilter2 = 0.f;
const float controlFilterConstant2 = 0.1f;

// Wall follower PID data
ControlLoop wallFollowLoop(dt);
float angleFilter = 0.f;
const float angleFilterConstant = 0.2f;

// Line detection
LineSensor rightLineSensor(rightLineSensorPin);
LineSensor leftLineSensor(leftLineSensorPin);
bool lastEdgeRight;

// Bump detection
bool bumpSensorPressed = false;
bool bumpDetected = false;

// Buttons
bool selectPressed = false;
bool scrollPressed = false;
bool selectPressDetected = false;
bool scrollPressDetected = false;

// Light sensor
const float lightSensorThreshold = 600.f;
float lightFilter = 0.f;
const float lightFilterConstant = 0.2f;

// Actions
struct Action
{   
    enum Type
    {
        rampCheese,
        sideCheese,
        floorCheese,
        opponentWall,
        opponentWallOuter,
        dropOff,
        
        last
    };
    
    enum Side
    {
        right,
        left
    };
    
    enum ReturnMode
    {
        sameSide,
        center,
        oppositeSide,
        hitWall,
    };
    
    enum CheesePlacement
    {
        farLeftSide,
        leftSide,
        rightSide,
        farRightSide,
    };
    
    Type type;
    Side side;
    ReturnMode returnMode;
    CheesePlacement placement;
};
const int numActions = 4;
Action actions[numActions];
int actionEntryIndex = 0;

// State machine
State state = state_enterActions;
float rightRelativePositionBase = 0.f;
float leftRelativePositionBase = 0.f;
unsigned long relativeTimeBase = 0ul;
int currentAction = 0;
const int numChains = 5;
const State* currentActionChains[numChains]; // start, action, transition, move back, place
int currentChain = 0;
int chainPosition = 0;

// State parameters
enum WallFollowDistance
{
    wallFollow_close,
    wallFollow_side,
    wallFollow_normal,
};
WallFollowDistance wallFollowDistance = wallFollow_normal;
enum StopCondition
{
    stopCondition_line,
    stopCondition_bump,
    stopCondition_halfRampDistance,
    stopCondition_shortDistance,
};
StopCondition stopCondition = stopCondition_line;

// Function prototypes
void rightPulse();
void leftPulse();
float getPosition(EncoderData& data);
void updateVelocity(EncoderData& data);
float getVelocity(EncoderData& data);
void updateVelocityLoop();
void updateLineDetection();
void driveStraight(float speed);
void driveAndTurn(float speed, float radius);
void turnInPlace(float angularVelocity);
void doStateAction(State st);
State stateTransition(State oldState);
float getRelativeDistance();
float getRelativeAngle();
float getRelativeTime();
void resetRelativeBase();
float feedForward(float velocity);
float getWallFollowRadius(bool movingOutward);
void assembleActionChains();


void setup()
{
    // Set up pins
    pinMode(rightInterruptPin, INPUT);
    pinMode(leftInterruptPin, INPUT);
    pinMode(bumpSensorPin, INPUT);
    pinMode(lightSensorPin, INPUT);
    pinMode(selectButtonPin, INPUT);
    pinMode(scrollButtonPin, INPUT);
    digitalWrite(rightInterruptPin, HIGH);
    digitalWrite(leftInterruptPin, HIGH);
    digitalWrite(bumpSensorPin, HIGH);
    digitalWrite(lightSensorPin, HIGH);
    digitalWrite(selectButtonPin, HIGH);
    digitalWrite(scrollButtonPin, HIGH);
    
    attachInterrupt(0, &rightPulse, RISING);
    attachInterrupt(1, &leftPulse, RISING);
    
    rightVelocityLoop.setTuning(400.f, 1500.f, 100.f);
    leftVelocityLoop.setTuning(400.f, 1500.f, 100.f);
    rightVelocityLoop.setOutputLimits(-255.f, 255.f);
    leftVelocityLoop.setOutputLimits(-255.f, 255.f);
    
    wallFollowLoop.setTuning(60.f, 0.f, 25.f);
    wallFollowLoop.setOutputLimits(-8.f, 8.f);
    
    elevationServo.attach(elevationServoPin);
    clawServo.attach(clawServoPin);
    
    Serial.begin(9600);
    
    resetRelativeBase();
}


void loop()
{   
    // Buttons
    bool temp = !digitalRead(bumpSensorPin);
    bumpDetected = (!bumpSensorPressed && temp);
    bumpSensorPressed = temp;
    
    temp = !digitalRead(selectButtonPin);
    selectPressDetected = (!selectPressed && temp);
    selectPressed = temp;
    
    temp = !digitalRead(scrollButtonPin);
    scrollPressDetected = (!scrollPressed && temp);
    scrollPressed = temp;
    
    // Update line sensors
    rightLineSensor.update();
    leftLineSensor.update();
    
    // Update state machine
    doStateAction(state);
    state = stateTransition(state);
    
    // Update motion
    updateVelocityLoop();
    
    // Update display
    if (cycleCount % displayUpdateCycles == 0)
    {
        Serial.print("?f");
        Serial.print("?x00?y0");
        
        for (int i = 0; i < numActions; ++i)
        {
            switch (actions[i].type)
            {
            case Action::rampCheese:
                Serial.print(actionEntryIndex == 4 * i ? "R" : "r");
                break;
            case Action::sideCheese:
                Serial.print(actionEntryIndex == 4 * i ? "S" : "s");
                break;
            case Action::floorCheese:
                Serial.print(actionEntryIndex == 4 * i ? "F" : "f");
                break;
            case Action::opponentWall:
                Serial.print(actionEntryIndex == 4 * i ? "W" : "W");
                break;
            case Action::opponentWallOuter:
                Serial.print(actionEntryIndex == 4 * i ? "O" : "o");
                break;
            case Action::dropOff:
                Serial.print(actionEntryIndex == 4 * i ? "D" : "d");
                break;
            case Action::last:
                Serial.print(actionEntryIndex == 4 * i ? "X" : "x");
                break;
            }
            
            switch (actions[i].side)
            {
            case Action::right:
                Serial.print(actionEntryIndex == 4 * i + 1 ? "R" : "r");
                break;
            case Action::left:
                Serial.print(actionEntryIndex == 4 * i + 1 ? "L" : "l");
                break;
            }
            
            switch (actions[i].returnMode)
            {
            case Action::sameSide:
                Serial.print(actionEntryIndex == 4 * i + 2 ? "S" : "s");
                break;
            case Action::center:
                Serial.print(actionEntryIndex == 4 * i + 2 ? "C" : "c");
                break;
            case Action::oppositeSide:
                Serial.print(actionEntryIndex == 4 * i + 2 ? "O" : "o");
                break;
            case Action::hitWall:
                Serial.print(actionEntryIndex == 4 * i + 2 ? "W" : "w");
                break;
            }
            
            switch (actions[i].placement)
            {
            case Action::farLeftSide:
                Serial.print(actionEntryIndex == 4 * i + 3 ? "LF" : "lf");
                break;
            case Action::leftSide:
                Serial.print(actionEntryIndex == 4 * i + 3 ? "L" : "l");
                break;
            case Action::rightSide:
                Serial.print(actionEntryIndex == 4 * i + 3 ? "R" : "r");
                break;
            case Action::farRightSide:
                Serial.print(actionEntryIndex == 4 * i + 3 ? "RF" : "rf");
                break;
            }
            
            Serial.print(" ");
            
            switch (i)
            {
            case 0:
                Serial.print("?x08?y0");
                break;
            case 1:
                Serial.print("?x00?y1");
                break;
            case 2:
                Serial.print("?x08?y1");
                break;
            }
        }
    }
    
    // Limit loop speed to a consistent value to make timing and integration simpler
    while (millis() - lastMillis < loopPeriodMs);
    lastMillis = millis();
    ++cycleCount;
}


void rightPulse()
{
    unsigned long time = micros();
    
    if (time - rightData.pulseTimes[0] > debounceMicros)
    {
        // Update encoder data
        rightData.pulseTimes.push(time);
        if (rightData.forward)
            rightData.count++;
        else
            rightData.count--;
    }
}


void leftPulse()
{
    unsigned long time = micros();
    
    if (time - leftData.pulseTimes[0] > debounceMicros)
    {
        // Update encoder data
        leftData.pulseTimes.push(time);
        if (leftData.forward)
            leftData.count++;
        else
            leftData.count--;
    }
}


float getPosition(EncoderData& data)
{
    noInterrupts();
    long count = data.count;
    interrupts();
    return count * wheelCircumference / pulsesPerRev;
}


void updateVelocity(EncoderData& data)
{
    // Assume zero velocity before enough data is gathered to estimate
    if (data.pulseTimes.size() != data.pulseTimes.capacity())
        return;
    
    noInterrupts();
    unsigned long lastDiff = data.pulseTimes[0] - data.pulseTimes[4];
    unsigned long newDiff = micros() - data.pulseTimes[0];
    interrupts();
    
    /**
     * If the current pulse diff is taking much longer than the average of the previous diffs, 
     * use it for the velocity calculation instead. This causes the velocity to go to zero 
     * when the robot is stationary (no pulses are generated).
     */
    float velocity;
    if (newDiff > lastDiff / 4ul * 2ul)
        velocity = (data.forward ? 1.0f : -1.0f) * wheelCircumference / ((newDiff * pulsesPerRev) / ticksPerSecond);
    else
        velocity = (data.forward ? 1.0f : -1.0f) * wheelCircumference / (((lastDiff * pulsesPerRev) / 4ul) / ticksPerSecond);
        
    data.filteredVelocity = data.filteredVelocity * (1.f - EncoderData::filterConstant) + 
                            velocity * EncoderData::filterConstant;
}


float getVelocity(EncoderData& data)
{
    // Assume zero velocity before enough data is gathered to estimate
    if (data.pulseTimes.size() != data.pulseTimes.capacity())
        return 0.f;
        
    return data.filteredVelocity;
}


void driveStraight(float speed)
{
    rightVelocityCommand = speed;
    leftVelocityCommand = speed;
}


void driveAndTurn(float speed, float radius)
{
    rightVelocityCommand = speed * (radius + wheelOffset) / radius;
    leftVelocityCommand = speed * (radius - wheelOffset) / radius;
}


void turnInPlace(float angularVelocity)
{
    rightVelocityCommand = wheelOffset * angularVelocity;
    leftVelocityCommand = -wheelOffset * angularVelocity;
}


void updateVelocityLoop()
{
    updateVelocity(rightData);
    updateVelocity(leftData);

    // Velocity control
    float rightControl = rightVelocityLoop.update(rightVelocityCommand - getVelocity(rightData), feedForward(rightVelocityCommand));
    float leftControl = leftVelocityLoop.update(leftVelocityCommand - getVelocity(leftData), feedForward(leftVelocityCommand));
    
    rightControlFilter = rightControlFilter * (1.f - controlFilterConstant) +
                         rightControl * controlFilterConstant;
    leftControlFilter = leftControlFilter * (1.f - controlFilterConstant) +
                        leftControl * controlFilterConstant;
    
    rightMotor.run((rightControlFilter > 0.f) ? FORWARD : BACKWARD);
    rightMotor.setSpeed( (int)fabs(rightControlFilter) );
    leftMotor.run((leftControlFilter > 0.f) ? FORWARD : BACKWARD);
    leftMotor.setSpeed( (int)fabs(leftControlFilter) );
    
    rightControlFilter2 = rightControlFilter2 * (1.f - controlFilterConstant2) +
                          rightControl * controlFilterConstant2;
    leftControlFilter2 = leftControlFilter2 * (1.f - controlFilterConstant2) +
                         leftControl * controlFilterConstant2;
    
    rightData.forward = rightControlFilter2 > 0.f;
    leftData.forward = leftControlFilter2 > 0.f;
}


void doStateAction(State st)
{
    const float speed = 0.25f;
    const float slowSpeed = 0.15f;

    switch (st)
    {
    // End of chain marker
    case state_endOfChain:
        break;
    
    // Program entry
    case state_enterActions:
		lightFilter = lightFilter * (1.f - lightFilterConstant) + analogRead(lightSensorPin) * lightFilterConstant;
        
        if (selectPressDetected)
        {
            ++actionEntryIndex;
            actionEntryIndex %= 4 * numActions;
        }
        
        if (scrollPressDetected)
        {   
            int index = actionEntryIndex / 4;
            int field = actionEntryIndex % 4;
            switch (field)
            {
            case 0: // type
                actions[index].type = Action::Type(int(actions[index].type) + 1);
                actions[index].type = Action::Type(int(actions[index].type) % int(Action::last));
                break;
            case 1: // side
                actions[index].side = Action::Side(int(actions[index].side) + 1);
                actions[index].side = Action::Side(int(actions[index].side) % 2);
                break;
            case 2: // return side
                actions[index].returnMode = Action::ReturnMode(int(actions[index].returnMode) + 1);
                actions[index].returnMode = Action::ReturnMode(int(actions[index].returnMode) % 4);
                break;
            }
        }
		break;
    
    // Alignment
    case state_alignOutgoing:
        
        break;
    case state_alignOutgoingCenter:
        
        break;
    case state_alignIncoming:
        
        break;
    case state_alignIncomingCenter:
        
        break;
    case state_alignToDropOff:
        
        break;
    
    // Line following
    case state_lineFollowBack:
        
        break;
    case state_lineFollowSlow:
        
        break;
    
    // Movement with settable parameters
    case state_wallFollowOut:
        
        break;
    case state_wallFollowBack:
        
        break;
    case state_forward:
        driveStraight(speed);
        break;
    
    // Movement options
    case state_setCloseFollowDistance:
        wallFollowDistance = wallFollow_close;
        break;
    case state_setSideFollowDistance:
        wallFollowDistance = wallFollow_side;
        break;
    case state_setNormalFollowDistance:
        wallFollowDistance = wallFollow_normal;
        break;
    case state_setStopAtLine:
        stopCondition = stopCondition_line;
        break;
    case state_setStopAtBump:
        stopCondition = stopCondition_bump;
        break;
    case state_setStopAtHalfRampDistance:
        stopCondition = stopCondition_halfRampDistance;
        break;
    case state_setStopAtShortDistance:
        stopCondition = stopCondition_shortDistance;
        break;
    
    // Dropping off and picking up cheese
    case state_straighten:
        
        break;
    case state_pickUpCheeseFromWall:
        
        break;
    case state_pickUpCheeseFromRamp:
        
        break;
    case state_pickUpCheeseFromFloor:
        
        break;
    case state_dropOffCheese:
        
        break;
    
    // Claw movements
    case state_openClaw:
        
        break;
    case state_closeClaw:
        
        break;
    case state_clawMaxHeight:
        
        break;
    case state_clawWallLevel:
        
        break;
    case state_clawRampLevel:
        
        break;
    case state_clawSideLevel:
        
        break;
    case state_clawFloorLevel:
        
        break;
    
    // Miscellaneous movement
    case state_shortPause:
        
        break;
    case state_backUpToTurn:
        
        break;
    case state_backUpToPlaceCheese:
        
        break;
    case state_turnInwards90:
        
        break;
    case state_turn3PointInwards90:
        
        break;
    case state_turnOutwards90:
        
        break;
    case state_turnInwards180:
        
        break;
    case state_turnOutwards180:
        
        break;
    }
}


State stateTransition(State oldState)
{
    bool nextState = false;
    State newState = oldState;
    
    switch (oldState)
    {
    // End of chain marker
    case state_endOfChain:
        newState = state_enterActions;
        break;
    
    // Program entry
    case state_enterActions:
		if (getRelativeTime() > 1.0f && lightFilter < lightSensorThreshold)
        {
            nextState = true;
            currentAction = 0;
            currentChain = 0;
            chainPosition = 0;
            assembleActionChains();
        }
		break;
    
    // Alignment
    case state_alignOutgoing:
        
        break;
    case state_alignOutgoingCenter:
        
        break;
    case state_alignIncoming:
        
        break;
    case state_alignIncomingCenter:
        
        break;
    case state_alignToDropOff:
        
        break;
    
    // Line following
    case state_lineFollowBack:
        
        break;
    case state_lineFollowSlow:
        nextState = bumpDetected;
        break;
    
    // Movement with settable parameters
    case state_wallFollowOut:
    case state_wallFollowBack:
    case state_forward:
        switch (stopCondition)
        {
        case stopCondition_line:
            nextState = (rightLineSensor.detected() || leftLineSensor.detected());
            break;
        case stopCondition_bump:
            nextState = bumpDetected;
            break;
        case stopCondition_halfRampDistance:
            nextState = (getRelativeDistance() > 0.3f);
            break;
        case stopCondition_shortDistance:
            nextState = (getRelativeDistance() > 0.1f);
            break;
        }
        break;
    
    // Movement options
    case state_setCloseFollowDistance:
    case state_setSideFollowDistance:
    case state_setNormalFollowDistance:
    case state_setStopAtLine:
    case state_setStopAtBump:
    case state_setStopAtHalfRampDistance:
    case state_setStopAtShortDistance:
        nextState = true;
        break;
    
    // Dropping off and picking up cheese
    case state_straighten:
        
        break;
    case state_pickUpCheeseFromWall:
        
        break;
    case state_pickUpCheeseFromRamp:
        
        break;
    case state_pickUpCheeseFromFloor:
        
        break;
    case state_dropOffCheese:
        
        break;
    
    // Claw movements
    case state_openClaw:
    case state_closeClaw:
    case state_clawMaxHeight:
    case state_clawWallLevel:
    case state_clawRampLevel:
    case state_clawSideLevel:
    case state_clawFloorLevel:
        nextState = true;
        break;
    
    // Miscellaneous movement
    case state_shortPause:
        nextState = (getRelativeTime() > 1.0f);
        break;
    case state_backUpToTurn:
        nextState = (getRelativeDistance() < 0.1f);
        break;
    case state_backUpToPlaceCheese:
        nextState = (getRelativeDistance() < 0.05f);
        break;
    case state_turnInwards90:
    case state_turn3PointInwards90:
    case state_turnOutwards90:
        nextState = (fabs(getRelativeAngle()) < 75.f);
        break;
    case state_turnInwards180:
    case state_turnOutwards180:
        nextState = (fabs(getRelativeAngle()) < 160.f);
        break;
    }
    
    if (nextState)
    {
        resetRelativeBase();
        ++chainPosition;
        
        if (currentActionChains[currentChain][chainPosition] == state_endOfChain)
        {
            ++currentChain;
            chainPosition = 0;
            
            if (currentChain >= numChains)
            {
                ++currentAction;
                currentAction %= numActions;
                currentChain = 0;
                assembleActionChains();
            }
        }
        
        newState = currentActionChains[currentChain][chainPosition];
    }
        
    if (oldState != state_enterActions && (selectPressDetected || scrollPressDetected))
    {
        newState = state_enterActions;
        actionEntryIndex = 0;
    }
    
    return newState;
}


float getRelativeDistance()
{
    return (getPosition(rightData) - rightRelativePositionBase + 
            getPosition(leftData) - leftRelativePositionBase) * 0.5f ;
}


float getRelativeAngle()
{
    const float rad2deg = 57.2958f;
    return ((getPosition(rightData) - rightRelativePositionBase) - 
            (getPosition(leftData) - leftRelativePositionBase)) / (2.f * wheelOffset) * rad2deg;
}


float getRelativeTime()
{
    return (millis() - relativeTimeBase) / 1000.f;
}


void resetRelativeBase()
{
    rightRelativePositionBase = getPosition(rightData);
    leftRelativePositionBase = getPosition(leftData);
    relativeTimeBase = millis();
}


float feedForward(float velocity)
{
    const float epsilon = 0.02f;
    if (fabs(velocity) < epsilon)
        return 0.f;
        
    return 54.45f * exp(3.237f * velocity);
}


float getWallFollowRadius(bool movingOutward)
{
    bool usingRightSensor = (actions[currentAction].side == Action::right) == movingOutward;
    
    float targetDist;
    switch (wallFollowDistance)
    {
    case wallFollow_close:
        targetDist = 0.05f;
        break;
    case wallFollow_side:
        targetDist = 0.10f;
        break;
    case wallFollow_normal:
        targetDist = 0.14f;
        break;
    }
    
    float dist;
    if (usingRightSensor)
        dist = 24.3f / (1 + analogRead(rightDistanceSensorPin)) - 0.03f;
    else
        dist = 21.7f / (1 + analogRead(leftDistanceSensorPin)) - 0.03f;
    
    float angle = wallFollowLoop.update(targetDist - dist);
    
    angleFilter = angleFilter * (1.f - angleFilterConstant) + angle * angleFilterConstant;
    
    if (fabs(angleFilter) < 0.0001)
        angle = 0.0001f;
        
    return usingRightSensor ? 1.f / angleFilter : -1.f / angleFilter;
}


void assembleActionChains()
{
    currentActionChains[0] = startingChain;

    switch (actions[currentAction].type)
    {
    case Action::rampCheese:
        currentActionChains[1] = getRampCheese;
        switch (actions[currentAction].returnMode)
        {
        case Action::sameSide:
            currentActionChains[2] = rampToSameTransition;
            currentActionChains[3] = returnSameSide;
            break;
        case Action::center:
            currentActionChains[2] = rampToCenterTransition;
            currentActionChains[3] = returnCenter;
            break;
        case Action::oppositeSide:
            currentActionChains[2] = rampToOppositeTransition;
            currentActionChains[3] = returnOpposite;
            break;
        case Action::hitWall:
            currentActionChains[2] = rampToHitWallTransition;
            currentActionChains[3] = returnOpposite;
            break;
        }
        break;
    case Action::sideCheese:
        currentActionChains[1] = getSideCheese;
        switch (actions[currentAction].returnMode)
        {
        case Action::sameSide:
            currentActionChains[2] = sideToSameTransition;
            currentActionChains[3] = returnSameSide;
            break;
        case Action::center:
            currentActionChains[2] = sideToCenterTransition;
            currentActionChains[3] = returnCenter;
            break;
        case Action::oppositeSide:
            currentActionChains[2] = sideToOppositeTransition;
            currentActionChains[3] = returnOpposite;
            break;
        case Action::hitWall:
            currentActionChains[2] = sideToHitWallTransition;
            currentActionChains[3] = returnOpposite;
            break;
        }
        break;
    }
    
    currentActionChains[4] = dropOff;
}
