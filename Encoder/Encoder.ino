#include <AFMotor.h>
#include <SoftwareSerial.h>
#include "RingBuffer.h"
#include <cmath>

// Pin assignments
const int LCDTx = 13;
const int LCDRx = 13;

// Initialize motors and LCD screen
AF_DCMotor rightMotor(3, MOTOR34_1KHZ);
AF_DCMotor leftMotor(4, MOTOR34_1KHZ);
SoftwareSerial screen = SoftwareSerial(LCDRx, LCDTx);

// Encoder variables
struct EncoderData
{
    volatile unsigned long count = 0;
    volatile RingBuffer<unsigned long, 2> pulseTimes;
};

EncoderData rightData;
EncoderData leftData;

// Interrupt routines
const int debounceMicros = 250;

void rightPulse()
{
    unsigned long time = micros();
    
    if (time - rightData.pulseTimes[0] > debounceMicros)
    {
        rightData.pulseTimes.push(time);
        rightData.count++;
    }
}

void leftPulse()
{
    unsigned long time = micros();
    
    if (time - leftData.pulseTimes[0] > debounceMicros)
    {
        leftData.pulseTimes.push(time);
        leftData.count++;
    }
}

// Physical parameters
const float wheelDiameter = 0.1f; // meters
const unsigned long pulsesPerRev = 6 * 25;
const float ticksPerSecond = 1000000.f;

// Function prototypes
float getPosition(EncoderData& data);
float getVelocity(EncoderData& data);

// Command variables
float rightPositionCommand;
float rightVelocityCommand;
float leftPositionCommand;
float leftVelocityCommand;
enum CommandMode
{
    positionMode,
    velocityMode
};
CommandMode commandMode;

// Integral terms
const float integralThreshold = 255.f;
float rightVelInt = 0.f;
float leftVelInt = 0.f;

// Tuning parameters
float pl_kp = 1.f;
float pl_ki = 0.f;
float pl_kd = 0.f;
float vl_kp = 1.f;
float vl_ki = 1.f;
float vl_kd = 0.f;

// Loop control
unsigned long lastMillis = 0;
unsigned long loopPeriodMs = 50;


void setup()
{
    // Set up pins
    pinMode(LCDTx, OUTPUT);
    
    rightData.pulseTimes.push(0ul);
    leftData.pulseTimes.push(0ul);
    
    commandMode = positionMode;
    rightPositionCommand = 0.f;
    leftPositionCommand = 0.f;
    
    attachInterrupt(0, &rightPulse, RISING);
    attachInterrupt(1, &leftPulse, RISING);
}


void loop()
{
    if (commandMode == positionMode)
    {   
        // Position loop
        rightVelocityCommand = pl_kp * (rightPositionCommand - getPosition(rightData));
        leftVelocityCommand = pl_kp * (leftPositionCommand - getPosition(leftData));
    }
    
    // Velocity loop
    float rightVelErr = rightVelocityCommand - getVelocity(rightData);
    float leftVelErr = leftVelocityCommand - getVelocity(leftData);
    
    rightVelInt += rightVelErr * (loopPeriodMs / 1000.f);
    leftVelInt += leftVelErr * (loopPeriodMs / 1000.f);
    
    // Limit integral terms to prevent windup
    rightVelInt = constrain(rightVelInt, -integralThreshold, integralThreshold);
    leftVelInt = constrain(leftVelInt, -integralThreshold, integralThreshold);
    
    float rightAccel = vl_kp * rightVelErr + vl_ki * rightVelInt;
    float leftAccel = vl_kp * leftVelErr + vl_ki * leftVelInt;
    
    rightMotor.run(rightAccel > 0.f ? FORWARD : BACKWARD);
    rightMotor.setSpeed( (unsigned char)fabs(rightAccel) );
    leftMotor.run(leftAccel > 0.f ? FORWARD : BACKWARD);
    leftMotor.setSpeed( (unsigned char)fabs(leftAccel) );
    
    // Limit loop speed to a consistent value to make integration easier
    while (millis() - lastMillis < loopPeriodMs)
    {
    }
    
    lastMillis = millis();
}


float getPosition(EncoderData& data)
{
    noInterrupts();
    unsigned long count = data.count;
    interrupts();
    return count * wheelDiameter / pulsesPerRev;
}


float getVelocity(EncoderData& data)
{
    // Assume zero velocity before enough data is gathered to estimate
    if (data.pulseTimes.size() != data.pulseTimes.capacity())
        return 0.f;
    
    noInterrupts();
    unsigned long lastDiff = data.pulseTimes[0] - data.pulseTimes[1];
    unsigned long newDiff = micros() - data.pulseTimes[0];
    interrupts();
    
    /**
     * If the current pulse diff is taking longer than the previous pulse diff, 
     * use it for the velocity calculation instead. This causes the velocity to 
     * go to zero when the robot is stationary (no pulses are generated).
     */
    if (newDiff > lastDiff)
        return wheelDiameter / (newDiff * pulsesPerRev) * ticksPerSecond;
    else
        return wheelDiameter / (lastDiff * pulsesPerRev) * ticksPerSecond;;
}
