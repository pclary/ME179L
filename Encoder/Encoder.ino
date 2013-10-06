#include <AFMotor.h>
#include "RingBuffer.h"
#include <cmath>
#include <cstdio>

// Pin assignments
const int rightInterruptPin = 2;
const int leftInterruptPin = 3;

// Initialize motors and LCD screen
AF_DCMotor rightMotor(3, MOTOR34_1KHZ);
AF_DCMotor leftMotor(4, MOTOR34_1KHZ);

// Encoder variables
struct EncoderData
{
    EncoderData() { count = 0; }
    volatile unsigned long count;
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
unsigned int loopPeriodMs = 50;
unsigned int cycleCount = 0;
const unsigned int displayUpdateCycles = 5;

// Data display
enum DisplayMode
{
    displayPosition,
    displayVelocity,
    displayPositionCommand,
    displayVelocityCommand
};

DisplayMode displayMode;


void setup()
{
    // Set up pins
    pinMode(rightInterruptPin, INPUT);
    pinMode(leftInterruptPin, INPUT);
    digitalWrite(leftInterruptPin, HIGH);
    digitalWrite(leftInterruptPin, HIGH);
    
    rightData.pulseTimes.push(0ul);
    leftData.pulseTimes.push(0ul);
    
    commandMode = positionMode;
    rightPositionCommand = 0.f;
    leftPositionCommand = 0.f;
    
    attachInterrupt(0, &rightPulse, RISING);
    attachInterrupt(1, &leftPulse, RISING);
    
    Serial.begin(9600);
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
    
    
    // Update display once every few cycles 
    if (cycleCount % displayUpdateCycles)
    {
        char buffer[17];
        Serial.print("?f?x00?y0");
    
        switch (displayMode)
        {
        case displayPosition:
            snprintf(buffer, 17, "RPos: %*.3f m/s", 16, getPosition(rightData));
            Serial.print(buffer);
            snprintf(buffer, 17, "LPos: %*.3f m/s", 16, getPosition(leftData));
            Serial.print(buffer);
            break;
        case displayVelocity:
            snprintf(buffer, 17, "RVel: %*.3f m/s", 16, getVelocity(rightData));
            Serial.print(buffer);
            snprintf(buffer, 17, "LVel: %*.3f m/s", 16, getVelocity(leftData));
            Serial.print(buffer);
            break;
        case displayPositionCommand:
            snprintf(buffer, 17, "RPCmd: %*.3f m/s", 16, rightPositionCommand);
            Serial.print(buffer);
            snprintf(buffer, 17, "LPCmd: %*.3f m/s", 16, leftPositionCommand);
            Serial.print(buffer);
            break;
        case displayVelocityCommand:
            snprintf(buffer, 17, "RVCmd: %*.3f m/s", 16, rightVelocityCommand);
            Serial.print(buffer);
            snprintf(buffer, 17, "LVCmd: %*.3f m/s", 16, leftVelocityCommand);
            Serial.print(buffer);
            break;
        }
    }
    
    
    // Limit loop speed to a consistent value to make integration easier
    while (millis() - lastMillis < loopPeriodMs)
    {
    }
    
    lastMillis = millis();
    ++cycleCount;
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
