#include <AFMotor.h>

#define IRSensor 14
#define baudrate 9600
#define LCDTxPin 1

int IRvalue = -1;

void setup(){
	Serial.begin(baudrate);
}

void loop(){
	IRvalue = analogRead(IRSensor);
	printValue(IRSensor);
}

void printValue(int data){
	Serial.print("?f");
	Serial.print("?x00?y0");
	Serial.print(data);
}
