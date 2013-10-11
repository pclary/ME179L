#include <AFMotor.h>

#define IRSensor 14
#define toggleSwitch 11
#define potMeterPin 15
#define baudrate 9600

int IRvalue = -1;
int Kp = -1;
int Ki = -1;
int potMeter = -1;
void setup(){
	Serial.begin(baudrate);
	pinMode(toggleSwitch, INPUT);
	digitalWrite(toggleSwitch, HIGH);
}

void loop(){
	if(switchPressed){
		IRvalue = analogRead(IRSensor);
		printValue("IR:", IRValue);
		delay(100);
	} else {
		potMeter = analogRead(potMeterPin);
		printValue("Pot:", potMeter);
		delay(100);
		
	}
}

void clearScreen(){
	Serial.print("?f");
}

void putCursor(int x, int y){
	Serial.print("?x" + x + "?y" + y);
}

void printValue(String label, int data){
	clearScreen();
	putCursor(0,0);
	Serial.print(label);
	putCursor(label.length(),0);//intentional +1 for spacing
	Serial.print(data);
}

boolean switchPressed(int button){
	if !digitalRead(button)
		return true;
	else
		return false;
}
