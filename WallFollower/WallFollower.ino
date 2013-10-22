#include <AFMotor.h>
#include <Servo.h>
#include <cmath>
#include "RingBuffer.h"

#define paramSwitch 11
#define exponentSwitch 2
#define mantissaSwitch 3
#define IRSensor A0
#define potMeter A1
#define photoResistor A2
#define lineSensor1 A3
#define lineSensor2 A4
#define ledPin A5
#define baudrate 9600

// Motors and servos
Servo servo;
AF_DCMotor rightMotor(3, MOTOR34_1KHZ);
AF_DCMotor leftMotor(4, MOTOR34_1KHZ);

// Store previous values
RingBuffer<float, 5> distValues;
RingBuffer<float, 5> distErrorValues;
RingBuffer<float, 5> velErrorValues;

// PID tuning constants
float dist_Kp = 6.f;
float dist_Ki = 0.f;
float dist_Kd = 1.f;
float vel_Kp = 60.f;
float vel_Ki = 0.f;
float vel_Kd = 5.f;

// Controller limits
float distIntLimit = 1000.f;
float velIntLimit = 1000.f;
float velSetpointLimit = 1.f; // m/s

// Integral accumulators
float distErrorInt = 0.f;
float velErrorInt = 0.f;

// Output filter
float filteredOutput = 0.f;
float filterConstant = 0.05f;

// Setpoints
float distSetpoint = 0.3f; // meters

// Loop control
const unsigned int loopPeriodMs = 20;
float dt = loopPeriodMs / 1000.f;
unsigned long lastMillis = 0;
unsigned int cycleCount = 0;
const unsigned int displayUpdateCycles = 10;

// Steering parameters
int servoCenter = 65;
int servoRight = 100;
int servoLeft = 30;

// Function prototypes
float getDistance();
float derivative(RingBuffer<float, 5>& x, float dt);

// Light sensor
int photoResistorThreshold = 150;

// Line sensor
float filteredLineSignal1 = 0.f;
float filteredLineSignal2 = 0.f;
float lineFilterConstant = 0.3f;
float lineRisingThreshold = 625.f;
float lineFallingThreshold = 500.f;
int lineCount = 0;
bool onLine1 = false;
bool onLine2 = false;
int lineCountTarget = 14;
unsigned long lastLineMillisBoth = 0;
unsigned long lastLineMillis1 = 0;
unsigned long lastLineMillis2 = 0;
unsigned long lineWindow = 1000; // ms
unsigned long lineTimeout = 1500; // ms

// Debug info
float distance;

// Tuning data structures
struct Parameter {
	float* var;
	const char* name;
};
Parameter params[] = {
	{ &dist_Kp, "dist_Kp" },
	{ &dist_Ki, "dist_Ki" },
	{ &dist_Kd, "dist_Kd" },
	{ &vel_Kp, "vel_Kp" },
	{ &vel_Ki, "vel_Ki" },
	{ &vel_Kd, "vel_Kd" },
	{ &distIntLimit, "distIntLim" },
	{ &velIntLimit, "velIntLim" },
	{ &velSetpointLimit, "velSPLim" },
	{ &distSetpoint, "distSP" },
	{ &filterConstant, "filter_C" },
	{ &distance, "dist" },
};
int currentParam = 0;
bool switchParamsPressed = false;
bool setExponentPressed = false;
bool setMantissaPressed = false;
enum InterfaceMode {
    viewMode,
    exponentMode,
    mantissaMode,
};
InterfaceMode interfaceMode;
int currentExponent;
float currentMantissa;
const int maxExponent = 6;
const int minExponent = -3;


void setup() {
	Serial.begin(baudrate);
	
	pinMode(paramSwitch, INPUT);
	pinMode(exponentSwitch, INPUT);
	pinMode(mantissaSwitch, INPUT);
	pinMode(photoResistor, INPUT);
	pinMode(lineSensor1, INPUT);
	pinMode(lineSensor2, INPUT);
	pinMode(ledPin, OUTPUT);
	
	digitalWrite(paramSwitch, HIGH);
	digitalWrite(exponentSwitch, HIGH);
	digitalWrite(mantissaSwitch, HIGH);
	digitalWrite(photoResistor, HIGH);
	digitalWrite(lineSensor1, HIGH);
	digitalWrite(lineSensor2, HIGH);
	digitalWrite(ledPin, LOW);
	
	servo.attach(10);
	
	// Wait for start light
	while(analogRead(photoResistor) > photoResistorThreshold) {}
	
	rightMotor.setSpeed(255);
	leftMotor.setSpeed(255);
	rightMotor.run(FORWARD);
	leftMotor.run(FORWARD);
}

void loop() {
    /* Use a position feedback loop to control the setpoint of a velocity feedback loop */    
    // Position loop
    distValues.push(getDistance());
    float distError = distSetpoint - distValues[0];
    distErrorValues.push(distError);
	distance = distValues[0];
    
    distErrorInt += distError * dt;
    distErrorInt = constrain(distErrorInt, -distIntLimit, distIntLimit);

    float velSetpoint = dist_Kp * distError + dist_Ki * distErrorInt + dist_Ki * derivative(distErrorValues, dt);
    velSetpoint = constrain(velSetpoint, -velSetpointLimit, velSetpointLimit);
    
    // Velocity loop
    float velError = velSetpoint - derivative(distValues, dt);
    velErrorValues.push(velError);
    
    velErrorInt += velError * dt;
    velErrorInt = constrain(velErrorInt, -velIntLimit, velIntLimit);
    
    float controlValue = vel_Kp * velError + vel_Ki * velErrorInt + vel_Ki * derivative(velErrorValues, dt);
    
	filteredOutput = filteredOutput * (1.f - filterConstant) + controlValue * filterConstant;
	
    // Use velocity loop output to control the steering angle
	int servoValue = servoCenter + (int)filteredOutput;
	servoValue = constrain(servoValue, servoLeft, servoRight);
	servo.write(servoValue);
    
    
    // Handle buttons
    bool temp = switchPressed(paramSwitch);
    
    if (temp && !switchParamsPressed) {
        interfaceMode = viewMode;
        ++currentParam;
        currentParam %= sizeof params / sizeof (Parameter);
    }
    switchParamsPressed = temp;
	
	temp = switchPressed(exponentSwitch);
    if (temp && !setExponentPressed) {
        if (interfaceMode == exponentMode) {
            interfaceMode = viewMode;
            *params[currentParam].var = currentMantissa * pow(10.f, (float)currentExponent);
        } else {
            interfaceMode = exponentMode;
            currentMantissa = *params[currentParam].var / pow(10.f, (float)(int)log10(*params[currentParam].var));
        }
    }
    setExponentPressed = temp;
    
	temp = switchPressed(mantissaSwitch);
    if (temp && !setMantissaPressed) {
        if (interfaceMode == mantissaMode) {
            interfaceMode = viewMode;
            *params[currentParam].var = currentMantissa * pow(10.f, (float)currentExponent);
        } else {
            interfaceMode = mantissaMode;
            currentExponent = (int)log10(*params[currentParam].var);
        }
    }
    setMantissaPressed = temp;
    
    // Draw interface
    if (cycleCount % displayUpdateCycles == 0) {
        clearScreen();
		
		Serial.print("?x00?y0");
        Serial.print(params[currentParam].name);
        
        switch (interfaceMode) {
        case viewMode:
			Serial.print("?x00?y1");
			Serial.print(*params[currentParam].var);
            break;
        case exponentMode:
            currentExponent = minExponent + (int)((long)analogRead(potMeter) * (maxExponent - minExponent) / 1024);
            Serial.print("?x15?y0");
            Serial.print("E");
			Serial.print("?x00?y1");
			Serial.print(currentMantissa * pow(10.f, (float)currentExponent));
            break;
        case mantissaMode:
            currentMantissa = analogRead(potMeter) / 1023.f;
            if (currentMantissa < 0.1f) {
                currentMantissa = 0.f;
            }
            Serial.print("?x15?y0");
            Serial.print("M");
			Serial.print("?x00?y1");
			Serial.print(currentMantissa * pow(10.f, (float)currentExponent));
            break;
        }
    }
	
	// Sense lines
	filteredLineSignal1 = filteredLineSignal1 * (1.f - lineFilterConstant) + 
						 analogRead(lineSensor1) * lineFilterConstant;
	filteredLineSignal2 = filteredLineSignal2 * (1.f - lineFilterConstant) + 
						 analogRead(lineSensor2) * lineFilterConstant;
	
	bool risingEdge = false;
	
	if (filteredLineSignal1 > lineRisingThreshold && !onLine1) {
		onLine1 = true;
		lastLineMillis1 = millis();
		if (millis() - lastLineMillis2 < lineWindow && 
			millis() - lastLineMillisBoth > lineTimeout) {
			lineCount++;
			lastLineMillisBoth = millis();
		}
	}
	if (filteredLineSignal2 > lineRisingThreshold && !onLine2) {
		onLine2 = true;
		lastLineMillis2 = millis();
		if (millis() - lastLineMillis1 < lineWindow && 
			millis() - lastLineMillisBoth > lineTimeout) {
			lineCount++;
			lastLineMillisBoth = millis();
		}
	}
	
	if (filteredLineSignal1 < lineFallingThreshold && onLine1) {
		onLine1 = false;
		lastLineMillis1 = millis();
	}
	if (filteredLineSignal2 < lineFallingThreshold && onLine2) {
		onLine2 = false;
		lastLineMillis2 = millis();
	}
	
	if (lineCount >= lineCountTarget) {
		// Stop moving
		rightMotor.run(RELEASE);
		leftMotor.run(RELEASE);
	}
	
	digitalWrite(ledPin, millis() - lastLineMillisBoth < 500);
	
    
    // Limit loop speed to a consistent value to make timing and integration simpler
    while (millis() - lastMillis < loopPeriodMs) {}
    
    lastMillis = millis();
    ++cycleCount;
}


float getDistance() {
    const float c1 = 0.0291f;
	const float c2 = 0.00076f;
	
	float value = analogRead(IRSensor);
	
	return 1.f / (value * c1 + c2);
}


float derivative(RingBuffer<float, 5>& x, float dt) {
    // Uses a 4th order backward finite difference method the approximate the derivative of a set of data
    return ( 2.083333f * x[0] - 4.f * x[1] + 3.f * x[2] - 1.333333f * x[3] + 0.25f * x[4] ) / dt;
}


void clearScreen() {
	Serial.print("?f");
}

bool switchPressed(int button) {
	return !digitalRead(button);
}
