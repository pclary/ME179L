#include "LineSensor.h"
#include "Arduino.h"


const float LineSensor::lineFilterConstant = 0.3f;

LineSensor::LineSensor(int pin) : pin(pin), 
                                  lineFilter(0.f), 
                                  lineDetected(false), 
                                  lineRisingThreshold(300.f),
                                  lineFallingThreshold(200.f)
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


void LineSensor::calibrate(float risingThreshold, float fallingThreshold)
{
    lineRisingThreshold = risingThreshold;
    lineFallingThreshold = fallingThreshold;
}