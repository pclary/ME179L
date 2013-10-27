#include "ControlLoop.h"


ControlLoop::ControlLoop(float dt) : dt(dt), kp(0.f), ki(0.f), kd(0.f), errorIntegral(0.f), outputMax(1.f), outputMin(-1.f)
{
    for (int i = 0; i < errorValues.capacity(); ++i)
        errorValues.push(0.f);
}


void ControlLoop::setTuning(float kp, float ki, float kd)
{
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
}


void ControlLoop::setOutputLimits(float min, float max)
{
    outputMin = min;
    outputMax = max;
}


float ControlLoop::update(float error)
{
    float tempErrorIntegral = errorIntegral + error * dt;
    
    errorValues.push(error);
    float errorDerivative = derivative(errorValues);
    
    float control = kp * error + ki * tempErrorIntegral + kd * errorDerivative;
    
    // Don't integrate if the output is pinned at the limits
    if ( !(control > outputMax && error > 0.f) &&
         !(control < outputMin && error < 0.f) )
    {
        errorIntegral = tempErrorIntegral;
    }
	
	// Limit the integral to the amount needed to saturate the output
	if (errorIntegral * ki > outputMax)
		errorIntegral = outputMax / ki;
	if (errorIntegral * ki < outputMin)
		errorIntegral = outputMin / ki;
    
    return control > outputMax ? outputMax : (control < outputMin ? outputMin : control);
}


float ControlLoop::derivative(RingBuffer<float, 5>& x)
{
    // Uses a 4th order backward finite difference method the approximate the derivative of a set of data
    return ( 2.083333f * x[0] - 4.f * x[1] + 3.f * x[2] - 1.333333f * x[3] + 0.25f * x[4] ) / dt;
}