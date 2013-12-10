#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "RingBuffer.h"


class LineSensor
{
public:
    LineSensor(int pin);
        void update();
        bool detected() { return lineDetected; }
    void calibrate(float risingThreshold, float fallingThreshold);
    
private:
        static const float lineFilterConstant;
        
    float lineRisingThreshold;
        float lineFallingThreshold;
        int pin;
        float lineFilter;
        bool lineDetected;
};

#endif // LINESENSOR_H