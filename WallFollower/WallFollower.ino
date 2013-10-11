#include <AFMotor.h>

#define IRSensor 14
#define toggleSwitch 11
#define potMeter 15
#define baudrate 9600

int IRvalue = -1;
int Kp = -1;
int Ki = -1;

void setup(){
	Serial.begin(baudrate);
	pinMode(toggleSwitch, INPUT);
	digitalWrite(toggleSwitch, HIGH);
}

void loop(){
	if(switchPressed(toggleSwitch)){
		while(switchPressed(toggleSwitch)){
			clearScreen();
			Serial.print("?x00?y0");
			Serial.print("Release to tune");
			delay(100);
		}
		clearScreen();
		tuneParameters();
	} else {
		clearScreen();
		Serial.print("?x00?y0");
		Serial.print("Kp = ");
		Serial.print("?x05?y0");
		Serial.print(Kp);
		Serial.print("?x00?y1");
		Serial.print("Ki = ");
		Serial.print("?x05?y1");
		Serial.print(Ki);
	}
	delay(100);
}

void clearScreen(){
	Serial.print("?f");
}

void putCursor(int x, int y){
	String line = "?x" + String(x) + "?y" + String(y);
	Serial.print(line);
}

void printValue(String label, int data){
	clearScreen();
	putCursor(0,0);
	Serial.print(label);
	putCursor(label.length(),0);//intentional +1 for spacing
	Serial.print(data);
}

boolean switchPressed(int button){
	if(!digitalRead(button))
		return true;
	else
		return false;
}

void tuneParameters(){
	while(!switchPressed(toggleSwitch)){
		Kp = tune("Kp = ");
		delay(100);
	}
	clearScreen();
	while(switchPressed(toggleSwitch)){
		clearScreen();
		Serial.print("?x00?y0");
		Serial.print("Release to tune next");
	}
	clearScreen();
	while(!switchPressed(toggleSwitch)){
		Ki = tune("Ki = ");
		delay(100);
	}
	while(switchPressed(toggleSwitch)){
		clearScreen();
		Serial.print("?x00?y0");
		Serial.print("Release to exit");
	}
}
int tune(String label){
		clearScreen();
		Serial.print("?x00?y0");
		Serial.print("Turn potmeter");
		Serial.print("?x00?y1");
		Serial.print(label);
		Serial.print("?x05?y1");
		int parameter = analogRead(potMeter);
		Serial.print(parameter);
		return parameter;
}