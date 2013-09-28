#include <AFMotor.h>
#include <SoftwareSerial.h>
#include "RingBuffer.h"

// Pin assignments
const int leftBumper = 11;
const int rightBumper = 10;
const int LCDTx = 13;
const int LCDRx = 13;

// Initialize motors and LCD screen
AF_DCMotor rightMotor(1, MOTOR12_1KHZ);
AF_DCMotor leftMotor(2, MOTOR12_1KHZ);
SoftwareSerial screen = SoftwareSerial(LCDRx, LCDTx);

// Ring buffer is used to store recent bump states
enum Side
{
    rightSide,
    leftSide
};

RingBuffer<Side, 6> bumpHistory;


// Function prototypes
void clearScreen();
void driveForward(uint8_t speed = 255);
void driveBackward(uint8_t speed = 255);
void brake();
void coast();
void turnDegrees(int degrees, uint8_t speed = 255);
bool checkCorner();


void setup()
{
    // Set up pins
    pinMode(leftBumper, INPUT);
    pinMode(rightBumper, INPUT);
    pinMode(LCDTx, OUTPUT);
    
    // Bump switches are pulled low when pressed
    digitalWrite(leftBumper, HIGH);
    digitalWrite(rightBumper, HIGH);
    
    screen.begin(9600);
    clearScreen();
    screen.print("Bump to begin.");
    
    // Motors are disabled at start
    coast();
    
    // Wait for one of the bumpers to be pressed
    while (digitalRead(leftBumper) && digitalRead(rightBumper))
    {
    }
    
    // Three second countdown before the robot starts moving
    for (int count = 3; count > 0; --count)
    {
        clearScreen();
        screen.print(count);
        delay(1000);
    }
}


void loop()
{
    clearScreen();
    screen.print("Rolling");
    
    driveForward();
    
    // Drive forward until either switch is pressed
    bool leftHit, rightHit;
    
    do
    {
        rightHit = !digitalRead(rightBumper);
        leftHit = !digitalRead(leftBumper);
    }
    while (!rightHit && !leftHit);
    
    // Determine which switch was pressed
    if (rightHit)
    {
        clearScreen();
        screen.print("RIGHT");
        bumpHistory.push(rightSide);
    }
    else // leftHit
    {
        clearScreen();
        screen.print("LEFT");
        bumpHistory.push(leftSide);
    }
    
    // Check to see if we're caught in a corner
    if (checkCorner())
    {
        // Back up and turn around 180 degrees
        driveBackward();
        delay(1000);
        turnDegrees(180);
    }
    else // not in a corner
    {
        // Back up and turn away from the obstacle
        driveBackward();
        delay(1000);
        turnDegrees(rightHit ? 30 : -30);
    }
}


void clearScreen()
{
    // Clear screen and reset cursor position
    screen.print("?f?x00?y0");
}


void driveForward(uint8_t speed)
{
    rightMotor.setSpeed(speed);
    leftMotor.setSpeed(speed);
    
    rightMotor.run(FORWARD);
    leftMotor.run(FORWARD);
}


void driveBackward(uint8_t speed)
{
    rightMotor.setSpeed(speed);
    leftMotor.setSpeed(speed);
    
    rightMotor.run(BACKWARD);
    leftMotor.run(BACKWARD);
}


void brake()
{
    rightMotor.setSpeed(0);
    leftMotor.setSpeed(0);
}

void coast()
{
    rightMotor.run(RELEASE);
    leftMotor.run(RELEASE);
}

void turnDegrees(int degrees, uint8_t speed)
{
    // Spin in place
    // CCW is positive
    
    brake();
	delay(50);

    rightMotor.setSpeed(speed);
    leftMotor.setSpeed(speed);
    
    if (degrees > 0)
    {
        rightMotor.run(BACKWARD);
        leftMotor.run(FORWARD);
    }
    else
    {
        rightMotor.run(FORWARD);
        leftMotor.run(BACKWARD);
    }
    
    delay(10 * degrees);
    brake();
    delay(50);
}


bool checkCorner()
{
    // Returns whether or not the robot thinks that it's stuck in a corner
    
    // No corner if the history buffer is not full yet
    if (bumpHistory.size() != bumpHistory.capacity())
        return false;
    
    // Check for an alternating pattern of right and left bumper presses
    for (int i = 1; i < bumpHistory.size(); ++i)
    {
        // If two bumps in a row are the same, it is not stuck in a corner
        if (bumpHistory[i] == bumpHistory[i + 1])
            return false;
    }
    
    // Control reaches here if the bump history alternates left and right
    return true;
}
