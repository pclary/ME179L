#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "RingBuffer.h"


class LineSensor
{
public:
    LineSensor(int pin);
	void update();
	bool detected() { return lineDetected; }
    
//private:
	static const float lineRisingThreshold = 300.f;
	static const float lineFallingThreshold = 200.f;
	static const float lineFilterConstant = 0.3f;
	
	int pin;
	float lineFilter;
	bool lineDetected;
};

#endif // LINESENSOR_H