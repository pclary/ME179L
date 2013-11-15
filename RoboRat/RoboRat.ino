#include <AFMotor.h>
#include <Servo.h>
#include "RingBuffer.h"
#include "ControlLoop.h"
#include "LineSensor.h"
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

// Light sensor
const float lightSensorThreshold = 600.f;
float lightFilter = 0.f;
const float lightFilterConstant = 0.2f;

// State machine
enum State
{   
    state_waitForLight,
    state_turnToSide,
    state_driveFromStart,
    state_wallFollowOut,
    state_shortForward,
    state_turnTowardsRamp,
    state_lineFollow,
    state_straighten,
    state_pickUpCheese,
    state_backUpFromRamp,
    state_turnBack,
    state_wallFollowBack,
    state_straightenAtWall,
    state_dropOffCheese,
    state_backUpFromWall,
    state_turnAround,
    state_forward,
    state_stop,
};
State state = state_waitForLight;
float rightRelativePositionBase = 0.f;
float leftRelativePositionBase = 0.f;
unsigned long relativeTimeBase = 0ul;
bool rightSide = false;
int trips = 0;

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
float getWallFollowRadius();


void setup()
{
    // Set up pins
    pinMode(rightInterruptPin, INPUT);
    pinMode(leftInterruptPin, INPUT);
    pinMode(bumpSensorPin, INPUT);
    pinMode(lightSensorPin, INPUT);
    digitalWrite(rightInterruptPin, HIGH);
    digitalWrite(leftInterruptPin, HIGH);
    digitalWrite(bumpSensorPin, HIGH);
    digitalWrite(lightSensorPin, HIGH);
    
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
    bool temp = !digitalRead(bumpSensorPin);
    bumpDetected = (!bumpSensorPressed && temp);
    bumpSensorPressed = temp;

    rightLineSensor.update();
    leftLineSensor.update();
    
    doStateAction(state);
    state = stateTransition(state);
    
    updateVelocityLoop();
    
    // Update display
    if (cycleCount % displayUpdateCycles == 0)
    {
        Serial.print("?f?x00?y0");
        Serial.print(state);
        Serial.print("?x08?y0");
        Serial.print(wallFollowLoop.derivativeFilter);
        Serial.print("?x00?y1");
        Serial.print(analogRead(leftDistanceSensorPin));
        Serial.print("?x08?y1");
        Serial.print(analogRead(rightDistanceSensorPin));
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
    case state_waitForLight:
		lightFilter = lightFilter * (1.f - lightFilterConstant) + analogRead(lightSensorPin) * lightFilterConstant;
		break;
    case state_turnToSide:
        clawServo.write(clawOpen);
        elevationServo.write(elevationRaised);
        turnInPlace(rightSide ? -2.f : 2.f);
        break;
    case state_driveFromStart:
        if (trips > 0)
            driveAndTurn(speed, rightSide ? 0.35f : -0.35f);
        else
            driveAndTurn(speed, rightSide ? 0.25f : -0.25f);
		break;
    case state_wallFollowOut:
		driveAndTurn(speed, getWallFollowRadius((trips == 0) ? 0.13f : 0.12f));
		break;
    case state_shortForward:
		driveStraight(slowSpeed);
		break;
    case state_turnTowardsRamp:
		turnInPlace(rightSide ? 1.5f : -1.5f);
		break;
    case state_lineFollow:
		if (rightLineSensor.detected() || leftLineSensor.detected())
        {
            driveStraight(slowSpeed);
        }
        else if (rightLineSensor.detected())
        {
            driveAndTurn(slowSpeed, 0.3f);
            lastEdgeRight = false;
        }
        else if (leftLineSensor.detected())
        {
            driveAndTurn(slowSpeed, -0.3f);
            lastEdgeRight = true;
        }
        else
        {
            driveAndTurn(slowSpeed, lastEdgeRight ? 0.2f : -0.2f);
        }
		break;
    case state_straighten:
        if (int(getRelativeTime()) % 2 == 0)
            driveAndTurn(slowSpeed, 0.05f);
        else
            driveAndTurn(slowSpeed, -0.05f);
        break;
    case state_pickUpCheese:
		driveStraight(0.f);
        if (getRelativeTime() < 0.5f)
        {
            clawServo.write(clawOpen);
            elevationServo.write(elevationLowered);
        }
        else if (getRelativeTime() < 1.5f)
        {
            clawServo.write(clawClosed);
            elevationServo.write(elevationLowered);
        }
        else
        {
            clawServo.write(clawClosed);
            elevationServo.write(elevationRaised);
        }
		break;
    case state_backUpFromRamp:
		driveStraight(-speed);
		break;
    case state_turnBack:
		turnInPlace(rightSide ? 1.5f : -1.5f);
		break;
    case state_wallFollowBack:
		if (getRelativeDistance() < 0.4f)
        {
            driveAndTurn(speed, getWallFollowRadius(0.2f));
        }
        else
        {
            driveAndTurn(speed, getWallFollowRadius(0.25f));
        }
		break;
    case state_straightenAtWall:
        if (int(getRelativeTime()) % 2 == 0)
            driveAndTurn(slowSpeed, 0.05f);
        else
            driveAndTurn(slowSpeed, -0.05f);
        break;
    case state_dropOffCheese:
		driveStraight(0.f);
        if (getRelativeTime() < 1.f)
        {
            clawServo.write(clawOpen);
            elevationServo.write(elevationLowered);
        }
        else
        {
            clawServo.write(clawOpen);
            elevationServo.write(elevationRaised);
        }
		break;
    case state_backUpFromWall:
		driveStraight(-speed);
		break;
    case state_turnAround:
		turnInPlace(rightSide ? -2.f : 2.f);
		break;
    case state_forward:
        driveStraight(speed);
        break;
    case state_stop:
        driveStraight(0.f);
        break;
    }
}


State stateTransition(State oldState)
{
    State newState = oldState;
    
    switch (oldState)
    {
    case state_waitForLight:
		if (getRelativeTime() > 1.0f && lightFilter < lightSensorThreshold)
        {
            newState = state_turnToSide;
        }
		break;
    case state_turnToSide:
        if (fabs(getRelativeAngle()) > 50.f)
        {
            newState = state_driveFromStart;
        }
        break;
    case state_driveFromStart:
		if (getRelativeDistance() > 0.6f)
        {
            newState = state_wallFollowOut;
        }
		break;
    case state_wallFollowOut:
        if (trips < 2)
        {
            if (getRelativeDistance() > 0.7f && (rightLineSensor.detected() || leftLineSensor.detected()))
            {
                newState = state_shortForward;
            }
        }
        else
        {
            if (bumpSensorPressed)
            {
                newState = state_straighten;
            }
        }
		break;
    case state_shortForward:
		if (getRelativeDistance() > 0.03f)
        {
            newState = state_turnTowardsRamp;
        }
		break;
    case state_turnTowardsRamp:
		if (fabs(getRelativeAngle()) > 65.f)
        {
            newState = state_lineFollow;
        }
		break;
    case state_lineFollow:
		if (bumpSensorPressed)
        {
            newState = state_straighten;
        }
		break;
    case state_straighten:
        if (getRelativeTime() > 2.f)
        {
            newState = state_pickUpCheese;
        }
        break;
    case state_pickUpCheese:
        if (getRelativeTime() > 2.f)
        {
            newState = (trips < 2) ? state_backUpFromRamp : state_stop;
        }
		break;
    case state_backUpFromRamp:
		if (getRelativeDistance() < -0.1f)
        {
            newState = state_turnBack;
        }
		break;
    case state_turnBack:
		if (fabs(getRelativeAngle()) > 60.f)
        {
            newState = state_wallFollowBack;
        }
		break;
    case state_wallFollowBack:
		if (bumpSensorPressed)
        {
            newState = state_straightenAtWall;
        }
		break;
    case state_straightenAtWall:
		if (getRelativeTime() > 2.f)
        {
            newState = state_dropOffCheese;
        }
		break;
    case state_dropOffCheese:
		if (getRelativeTime() > 2.f)
        {
            newState = state_backUpFromWall;
        }
		break;
    case state_backUpFromWall:
		if (getRelativeDistance() < -0.1f)
        {
            newState = state_turnAround;
        }
		break;
    case state_turnAround:
		if (fabs(getRelativeAngle()) > 60.f)
        {
            newState = state_forward;
            rightSide = !rightSide;
            ++trips;
        }
		break;
    case state_forward:
        if (getRelativeDistance() > 0.3f)
        {
            newState = state_driveFromStart;
        }
        break;
    case state_stop:
        break;
    }
    
    if (newState != oldState)
        resetRelativeBase();
    
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


float getWallFollowRadius(float targetDist)
{
    bool usingRightSensor = (state == state_wallFollowOut) == rightSide;
    
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
