//#include <AFMotor.h>
//#include <SoftwareSerial.h>
#include "RingBuffer.h"

// Pin assignments
const int LCDTx = 13;
const int LCDRx = 13;

// Initialize motors and LCD screen
//AF_DCMotor rightMotor(3, MOTOR34_1KHZ);
//AF_DCMotor leftMotor(4, MOTOR34_1KHZ);
//SoftwareSerial screen = SoftwareSerial(LCDRx, LCDTx);

// Encoder variables
struct EncoderData
{
    volatile unsigned long count = 0;
    volatile RingBuffer<unsigned long, 2> pulseTimes;
};

EncoderData rightData;
EncoderData leftData;

// Interrupt routines
void rightPulse()
{
    rightData.pulseTimes.push(micros());
    rightData.count++;
}

void leftPulse()
{
    leftData.pulseTimes.push(micros());
    leftData.count++;
}

// Parameters
const float wheelDiameter = 0.1f; // meters
const unsigned long pulsesPerRev = 6 * 25;
const float ticksPerSecond = 1000000.f;

// Function prototypes
float getPosition(EncoderData& data);
float getVelocity(EncoderData& data);


void setup()
{
    // Set up pins
    pinMode(LCDTx, OUTPUT);
}


void loop()
{
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
