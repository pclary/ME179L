#include <AFMotor.h>
#include <SoftwareSerial.h>

const int lBump = 11;  //define pins and motor speed. LCDRx is a dummy pin for setup purposes
const int rBump = 10;
const int LCDTx = 13;
const int LCDRx = 13;
int motorSpeed = 255;

AF_DCMotor rMotor(1, MOTOR12_1KHZ);  //right motor is on port 1, left on port 2
AF_DCMotor lMotor(2, MOTOR12_1KHZ);
SoftwareSerial screen = SoftwareSerial(LCDRx, LCDTx);  //got screen?


void driveForward(uint8_t speed = 255)
{
	rMotor.setSpeed(speed);
    lMotor.setSpeed(speed);
	
	rMotor.run(FORWARD);
    lMotor.run(FORWARD);
}


void driveBackward(uint8_t speed = 255)
{
	rMotor.setSpeed(speed);
    lMotor.setSpeed(speed);
	
	rMotor.run(BACKWARD);
    lMotor.run(BACKWARD);
}


void brake()
{
	rMotor.setSpeed(0);
    lMotor.setSpeed(0);
}


void coast()
{
	rMotor.run(RELEASE);
    lMotor.run(RELEASE);
}


void turnDegrees(int degrees, uint8_t speed = 255) // CCW is positive
{
	rMotor.setSpeed(speed);
    lMotor.setSpeed(speed);
	
	if (degrees > 0)
	{
		rMotor.run(BACKWARD);
        lMotor.run(FORWARD);
	}
	else
	{
		rMotor.run(FORWARD);
        lMotor.run(BACKWARD);
	}
	
	delay(10 * degrees);
	
	brake();
	
	delay(50);
}


void setup()
{
  pinMode(lBump, INPUT);  //switches are inputs, LCD is an output
  pinMode(rBump, INPUT);
  pinMode(LCDTx, OUTPUT);
  
  digitalWrite(lBump, HIGH);  //sets pin high such that the switches can pull them low
  digitalWrite(rBump, HIGH);  // or drain them to ground, however you want to think
  
  screen.begin(9600);
  screen.print("?f");  //clears screen
  screen.print("?x00?y0");  //sets cursor to x=00 and y=0
  
  rMotor.setSpeed(motorSpeed);
  lMotor.setSpeed(motorSpeed);
  
  screen.print("Bump to begin.     ");
  
  while ( digitalRead(lBump) && digitalRead(rBump) )  //wait for a bump
  {
  }
  
  for (int i = 3; i > 0; i--)  //countdown for fun. maybe add racing beeps later
  {
    screen.print("?x00?y0");
    screen.print("                 ");
    screen.print("?x00?y0");
    screen.print(i);
    delay(1000);
  }
}

void loop()
{
  screen.print("?f?x00?y0");
  screen.print("Rolling         ");
  
  rMotor.run(FORWARD);
  lMotor.run(FORWARD);
  
  bool leftHit, rightHit;
  
  while ( !(leftHit = !digitalRead(lBump)) && !(rightHit = !digitalRead(rBump)) )
  {
  }
  
  if (leftHit)  //if the left switch is hit
  {
    screen.print("?x00?y0LEFT");
    lMotor.run(BACKWARD);|
    rMotor.run(BACKWARD);  //backup for 300ms
    delay(1500);
    lMotor.run(RELEASE);  //stahp
    rMotor.run(RELEASE);
    lMotor.run(FORWARD);  //spin CW about center for 100ms
    rMotor.run(BACKWARD);
    delay(300);
  }
  else
  {
    screen.print(?"?x00?y0");
    screen.print("RIGHT            ");
    lMotor.run(BACKWARD);
    rMotor.run(BACKWARD);
    delay(1500);
    lMotor.run(RELEASE);  //stahp
    rMotor.run(RELEASE);
    rMotor.run(FORWARD);  //spin CCW about center for 100ms
    lMotor.run(BACKWARD);
    delay(300);
  }
  
  /* logic of the above is if bot hits dead on (both switches) or left switch,
   * the first if statement will read a hit on the left switch, reverse, and spin CW
   * otherwise (so if only a right hit, or momentary tap of left switch/both switches
   * reverse and spin CCW. the while prefacing the if's should garentee a hit did
   * infact happen.
   */
}
