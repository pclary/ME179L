#include <AFMotor.h>
#include "RingBuffer.h"
#include "ControlLoop.h"
#include "LineSensor.h"
#include <cmath>
#include <cstdio>


// Pin assignments
const int rightInterruptPin = 2;
const int leftInterruptPin = 3;
const int potentiometerPin = A0;
const int rightLineSensorPin = A1;
const int leftLineSensorPin = A4;
const int distanceSensorPin = A2;
const int bumpSensorPin = A3;

// Initialize motors
AF_DCMotor rightMotor(4, MOTOR34_1KHZ);
AF_DCMotor leftMotor(3, MOTOR34_1KHZ);

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

// PID data
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

// Line detection
LineSensor rightLineSensor(rightLineSensorPin);
LineSensor leftLineSensor(leftLineSensorPin);

// State machine
enum State
{   
    state_0,
	state_1,
};
State state = state_0;
float rightRelativePositionBase = 0.f;
float leftRelativePositionBase = 0.f;
int tripCount = 0;
bool movingRight;
bool startRight;
int measuredDist;

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
void resetRelativeBase();
float feedForward(float velocity);


void setup()
{
    // Set up pins
    pinMode(rightInterruptPin, INPUT);
    pinMode(leftInterruptPin, INPUT);
    pinMode(bumpSensorPin, INPUT);
    digitalWrite(rightInterruptPin, HIGH);
    digitalWrite(leftInterruptPin, HIGH);
    digitalWrite(bumpSensorPin, HIGH);
    
    attachInterrupt(0, &rightPulse, RISING);
    attachInterrupt(1, &leftPulse, RISING);
	
    rightVelocityLoop.setTuning(400.f, 1500.f, 100.f);
    leftVelocityLoop.setTuning(400.f, 1500.f, 100.f);
    rightVelocityLoop.setOutputLimits(-255.f, 255.f);
    leftVelocityLoop.setOutputLimits(-255.f, 255.f);
    
    Serial.begin(9600);
    
    // Detect whether on the right or left side of the board
    unsigned long dist = 0ul;
    for (int i = 0; i < 100; ++i)
    {
        dist += analogRead(distanceSensorPin);
        delay(1);
    }
    
	measuredDist = dist / 100ul;
    movingRight = measuredDist < 150;
	startRight = !movingRight;
}


void loop()
{
	rightLineSensor.update();
	leftLineSensor.update();
	
	doStateAction(state);
	state = stateTransition(state);
	
    updateVelocityLoop();
    
    // Update display
    if (cycleCount % displayUpdateCycles == 0)
    {
		Serial.print("?f");
		Serial.print("?x00?y0");
		
		if (state == state_0)
		{
			if (movingRight)
				Serial.write("Starting left");
			else
				Serial.write("Starting right");
			Serial.print("?x00?y1");
			Serial.print(measuredDist);
		}
		else
		{
			Serial.print(leftLineSensor.lineFilter);
			Serial.print("?x08?y0");
			Serial.print(rightLineSensor.lineFilter);
			Serial.print("?x00?y1");
			Serial.print(getRelativeDistance());
			Serial.print("?x08?y1");
			Serial.print(state);
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

	switch (st)
	{
	case state_0:
        driveStraight(0.f);
        break;
	case state_1a:
        if (rightLineSensor.detected() && leftLineSensor.detected())
            driveStraight(0.3f);
        else if (rightLineSensor.detected())
            driveAndTurn(0.3f, -0.4f);
        else if (leftLineSensor.detected())
            driveAndTurn(0.3f, 0.4f);
        else
            driveAndTurn(0.3f, movingRight? -0.1f : 0.1f);
        break;
	}
}


State stateTransition(State oldState)
{
	State newState = oldState;
	
	switch (oldState)
	{
	case state_0:
        if (!digitalRead(bumpSensorPin))
		{
            newState = state_1a;
			resetRelativeBase();
		}
        break;
	case state_1a:
        
        break;
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


void resetRelativeBase()
{
	rightRelativePositionBase = getPosition(rightData);
	leftRelativePositionBase = getPosition(leftData);
}


float feedForward(float velocity)
{
	const float epsilon = 0.02f;
	if (fabs(velocity) < epsilon)
		return 0.f;
		
	return 54.45f * exp(3.237f * velocity);
}
