#include <AFMotor.h>
#include "RingBuffer.h"
#include <cmath>
#include <cstdio>


// Pin assignments
const int rightInterruptPin = 2;
const int leftInterruptPin = 3;
const int analogPin = 0;

// Initialize motors and LCD screen
AF_DCMotor rightMotor(3, MOTOR34_1KHZ);
AF_DCMotor leftMotor(4, MOTOR34_1KHZ);

// Encoder variables
struct EncoderData
{
    EncoderData() { count = 0; }
    void reset() { count = 0; pulseTimes.clear(); pulseTimes.push(0ul); }
    volatile long count;
    volatile RingBuffer<unsigned long, 66> pulseTimes;
    bool forward;
};

EncoderData rightData;
EncoderData leftData;
long startButtonCount = 0;
long modeButtonCount = 0;
const long buttonCountThreshold = 3;
unsigned long stoppedTime = 0;
const unsigned long allowButtonsTime = 500000; // microseconds
const unsigned long debounceMicros = 250;

// Physical parameters
const float wheelCircumference = 0.21f; // meters
const long pulsesPerRev = 6 * 25 * 3;
const float ticksPerSecond = 1000000.f;

// Function prototypes
void rightPulse();
void leftPulse();
float getPosition(EncoderData& data);
float getVelocity(EncoderData& data);
void startMoving();
void stopMoving();

// Command variables
float rightPositionCommand;
float leftPositionCommand;
float rightVelocityCommand;
float leftVelocityCommand;
float rightPercentage;
float leftPercentage;
enum CommandMode
{
    positionMode,
    velocityMode
};
CommandMode commandMode;

// Integral terms
float rightVelInt = 0.f;
float leftVelInt = 0.f;
const float velIntMax = 0.240f;

// Tuning parameters
const float pl_kp = 1.f;
const float vl_kp = 1000.f;
const float vl_ki = 1000.f;

// Loop control
unsigned long lastMillis = 0;
unsigned int loopPeriodMs = 20;
unsigned int cycleCount = 0;
const unsigned int displayUpdateCycles = 10;

// Movement control
bool started = false;
bool showResults = false;
unsigned long startedMillis;
const float positionTolerance = 0.02; // meters
const float velocityTolerance = 0.02; // m/s
const unsigned long positionTimeout = 120000; // milliseconds
const unsigned long velocityRunTime = 10000; // milliseconds
const float positionMin = -3.f; // meters
const float positionMax = 3.f; // meters
const float velocityMin = -0.15f; // m/s
const float velocityMax = 0.15f; // m/s
const float percentageMin = 0.f;
const float percentageMax = 100.f;


void setup()
{
    // Set up pins
    pinMode(rightInterruptPin, INPUT);
    pinMode(leftInterruptPin, INPUT);
    digitalWrite(rightInterruptPin, HIGH);
    digitalWrite(leftInterruptPin, HIGH);
    
    rightData.pulseTimes.push(0ul);
    leftData.pulseTimes.push(0ul);
    
    commandMode = positionMode;
    
    attachInterrupt(0, &rightPulse, RISING);
    attachInterrupt(1, &leftPulse, RISING);
    
    Serial.begin(9600);
}


void loop()
{
    if (started)
    {
        if (commandMode == positionMode)
        {
            // Use a position feedback loop to control the setpoint of a velocity feedback loop
        
            // Position loop
            float rightPosErr = rightPositionCommand - getPosition(rightData);
            float leftPosErr = leftPositionCommand - getPosition(leftData);
        
            rightVelocityCommand = pl_kp * rightPosErr;
            leftVelocityCommand = pl_kp * leftPosErr;
            
            rightVelocityCommand = constrain(rightVelocityCommand, velocityMin, velocityMax);
            leftVelocityCommand = constrain(leftVelocityCommand, velocityMin, velocityMax);
            
            // Velocity loop
            float rightVelErr = rightVelocityCommand - getVelocity(rightData);
            float leftVelErr = leftVelocityCommand - getVelocity(leftData);
            
            rightVelInt += rightVelErr * (loopPeriodMs / 1000.f);
            leftVelInt += leftVelErr * (loopPeriodMs / 1000.f);
            
            rightVelInt = constrain(rightVelInt, -velIntMax, velIntMax);
            leftVelInt = constrain(leftVelInt, -velIntMax, velIntMax);
            
            float rightAccel = vl_kp * rightVelErr + vl_ki * rightVelInt;
            float leftAccel = vl_kp * leftVelErr + vl_ki * leftVelInt;
            
            rightMotor.run((rightAccel > 0.f) ? FORWARD : BACKWARD);
            rightMotor.setSpeed( constrain((int)fabs(rightAccel), 0, 255) );
            leftMotor.run((leftAccel > 0.f) ? FORWARD : BACKWARD);
            leftMotor.setSpeed( constrain((int)fabs(leftAccel), 0, 255) );
            
            rightData.forward = rightAccel > 0.f;
            leftData.forward = leftAccel > 0.f;
        
            // Stop when close to the target distance with a low speed or after a timeout
            if ( (fabs(rightPosErr) < positionTolerance && fabs(leftPosErr) < positionTolerance &&
                  fabs(rightVelErr) < velocityTolerance && fabs(leftVelErr) < velocityTolerance) ||
                millis() - startedMillis >= positionTimeout )
            {
                stopMoving();
            }
        }
        else // velocityMode
        {
            // Give each motor a set PWM speed equal to a percentage of the maximum
            rightMotor.run(FORWARD);
            leftMotor.run(FORWARD);
            
            rightMotor.setSpeed( (int)(rightPercentage * 2.55f) );
            leftMotor.setSpeed( (int)(leftPercentage * 2.55f) );
            
            rightData.forward = true;
            leftData.forward = true;
        
            // Stop after a certain time
            if (millis() - startedMillis >= velocityRunTime)
                stopMoving();
        }
    }
    else
    {
        rightMotor.run(RELEASE);
        leftMotor.run(RELEASE);
        
        // Update command value
        float value = analogRead(analogPin) / 1023.f;
        
        if (commandMode == positionMode)
        {
            rightPositionCommand = positionMin + value * (positionMax - positionMin);
            leftPositionCommand = positionMin + value * (positionMax - positionMin);
        }
        else // velocityMode
        {
            rightPercentage = percentageMin + value * (percentageMax - percentageMin);
            leftPercentage = percentageMin + value * (percentageMax - percentageMin);
        }
    }
    
    
    // Update display once every few cycles 
    if (cycleCount % displayUpdateCycles == 0)
    {
        Serial.print("?f");
    
        if (commandMode == positionMode)
        {
            if (started || showResults)
            {
                Serial.print("?x00?y0RPos: ");
                Serial.print(getPosition(rightData));
                Serial.print(" m");
                Serial.print("?x00?y1LPos: ");
                Serial.print(getPosition(leftData));
                Serial.print(" m");
            }
            else
            {
                Serial.print("?x00?y0RPCmd: ");
                Serial.print(rightPositionCommand);
                Serial.print(" m");
                Serial.print("?x00?y1LPCmd: ");
                Serial.print(leftPositionCommand);
                Serial.print(" m");
            }
        }
        else // velocityMode
        {
            if (started)
            {
                Serial.print("?x00?y0RVel: ");
                Serial.print(getVelocity(rightData) * 100.f);
                Serial.print(" cm/s");
                Serial.print("?x00?y1LVel: ");
                Serial.print(getVelocity(leftData) * 100.f);
                Serial.print(" cm/s");
            }
            else if (showResults)
            {
                Serial.print("?x00?y0RCnt: ");
                Serial.print(rightData.count);
                Serial.print("?x00?y1LCnt: ");
                Serial.print(leftData.count);
            }
            else
            {
                Serial.print("?x00?y0RVCmd: ");
                Serial.print(rightPercentage);
                Serial.print("%");
                Serial.print("?x00?y1LVCmd: ");
                Serial.print(leftPercentage);
                Serial.print("%");
            }
        }
    }
    
    
    // Limit loop speed to a consistent value to make timing and integration simpler
    while (millis() - lastMillis < loopPeriodMs)
    {
    }
    
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
        
        // Update information input state
        if (time - stoppedTime < allowButtonsTime)
        {
            startButtonCount = rightData.count;
        }
        else if (!started)
        {
            if ( abs(rightData.count - startButtonCount) >= buttonCountThreshold )
                startMoving();
        }
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
        
        // Update information input state
        if (time - stoppedTime < allowButtonsTime)
        {
            modeButtonCount = leftData.count;
        }
        else if (!started)
        {
            if ( abs(leftData.count - modeButtonCount) >= buttonCountThreshold )
            {
                modeButtonCount = leftData.count;
                
                if (showResults)
                    showResults = false;
                else
                    commandMode = (commandMode == positionMode) ? velocityMode : positionMode;
            }
        }
    }
}


float getPosition(EncoderData& data)
{
    noInterrupts();
    long count = data.count;
    interrupts();
    return count * wheelCircumference / pulsesPerRev;
}


float getVelocity(EncoderData& data)
{
    // Assume zero velocity before enough data is gathered to estimate
    if (data.pulseTimes.size() != data.pulseTimes.capacity())
        return 0.f;
    
    noInterrupts();
    unsigned long lastDiff = data.pulseTimes[0] - data.pulseTimes[64];
    unsigned long newDiff = micros() - data.pulseTimes[0];
    interrupts();
    
    /**
     * If the current pulse diff is taking much longer than the average of the previous diffs, 
     * use it for the velocity calculation instead. This causes the velocity to go to zero 
     * when the robot is stationary (no pulses are generated).
     */
    if (newDiff > lastDiff / 64ul * 2ul)
        return (data.forward ? 1.0f : -1.0f) * wheelCircumference / ((newDiff * pulsesPerRev) / ticksPerSecond);
    else
        return (data.forward ? 1.0f : -1.0f) * wheelCircumference / (((lastDiff * pulsesPerRev) / 64ul) / ticksPerSecond);
}

void startMoving()
{
    started = true;
    showResults = false;
    startedMillis = millis();
    noInterrupts();
    rightData.reset();
    leftData.reset();
    interrupts();
    rightVelInt = 0.f;
    leftVelInt = 0.f;
}

void stopMoving()
{
    started = false;
    showResults = true;
    stoppedTime = micros();
    startButtonCount = rightData.count;
    modeButtonCount = leftData.count;
}
