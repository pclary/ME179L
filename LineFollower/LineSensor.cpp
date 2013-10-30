#include "LineSensor.h"
#include "Arduino.h"


LineSensor::LineSensor(int pin) : pin(pin), lineFilter(0.f), lineDetected(false)
{
	pinMode(pin, INPUT);
	digitalWrite(pin, HIGH);
}


void LineSensor::update()
{
	lineFilter = lineFilter * (1.f - lineFilterConstant) + 
	             analogRead(pin) * lineFilterConstant;
	
	if (!lineDetected && lineFilter > lineRisingThreshold)
		lineDetected = true;
		
	if (lineDetected && lineFilter < lineFallingThreshold)
		lineDetected = false;
}