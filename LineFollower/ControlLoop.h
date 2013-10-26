#ifndef CONTROLLOOP_H
#define CONTROLLOOP_H

#include "RingBuffer.h"

class ControlLoop
{
public:
    ControlLoop(float dt);
    void setTuning(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    float update(float error);
    
private:
    RingBuffer<float, 5> errorValues;
    float errorIntegral;
    const float dt;
    float kp;
    float ki;
    float kd;
    
    float derivative(RingBuffer<float, 5>& x, float dt);
}

#endif // CONTROLLOOP_H