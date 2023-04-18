// Include the AccelStepper Library
#include <AccelStepper.h>

// Define pin connections
const int dirPin = 54;
const int stepPin = 55;

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
	// set the maximum speed, acceleration factor,
	// initial speed and the target position
	myStepper.setMaxSpeed(1000);
	myStepper.setAcceleration(50);
	myStepper.setSpeed(200);
	myStepper.moveTo(200);
}

void loop() {
	// Change direction once the motor reaches target position
	if (myStepper.distanceToGo() == 0)
		myStepper.moveTo(-myStepper.currentPosition());

	// Move the motor one step
	myStepper.run();
}