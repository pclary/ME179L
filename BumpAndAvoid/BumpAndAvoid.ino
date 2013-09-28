#include <AFMotor.h>
#include <SoftwareSerial.h>

// Pin assignments
const int leftBumper = 11;
const int rightBumper = 10;
const int LCDTx = 13;
const int LCDRx = 13;

// Initialize motors and LCD screen
AF_DCMotor rightMotor(1, MOTOR12_1KHZ);
AF_DCMotor leftMotor(2, MOTOR12_1KHZ);
SoftwareSerial screen = SoftwareSerial(LCDRx, LCDTx);


void clearScreen()
{
    // Clear screen and reset cursor position
    screen.print("?f?x00?y0");
}


void driveForward(uint8_t speed = 255)
{
    rightMotor.setSpeed(speed);
    leftMotor.setSpeed(speed);
    
    rightMotor.run(FORWARD);
    leftMotor.run(FORWARD);
}


void driveBackward(uint8_t speed = 255)
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

void turnDegrees(int degrees, uint8_t speed = 255)
{
    // Spin in place
    // CCW is positive
    
    brake(50);

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
    while (!rightHit && !leftHit)
    
    // Determine which switch was pressed, then back up and turn to avoid the obstacle
    if (leftHit)
    {
        clearScreen();
        screen.print("LEFT");
        driveBackward();
        delay(1500);
        turnDegrees(-30);
    }
    else // rightHit
    {
        clearScreen();
        screen.print("RIGHT");
        driveBackward();
        delay(1500);
        turnDegrees(30);
    }
}
