#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <IBusBM.h>
#include <EEPROM.h>
#include <AccelStepper.h>

#define USE_PCA9685_SERVO_EXPANDER
#include <ServoEasing.hpp>
#include "PinDefinitions.h"

#define BASIC_CONTROL false

// R/C
IBusBM IBus;
IBusBM IBusSensor;
int ch1, ch2, ch3, ch4, ch5, ch6 = 0;

// Servos
ServoEasing servoW1(PCA9685_DEFAULT_ADDRESS);
ServoEasing servoW3(PCA9685_DEFAULT_ADDRESS);
ServoEasing servoW4(PCA9685_DEFAULT_ADDRESS);
ServoEasing servoW6(PCA9685_DEFAULT_ADDRESS);

//Motors
int RR_EL = 39, RR_ZF = 37, RR_VR = 7;
int LR_EL = 28, LR_ZF = 30, LR_VR = 4;
int RM_EL = 36, RM_ZF = 34, RM_VR = 6;
int LF_EL = 22, LF_ZF = 24, LF_VR = 2;
int RF_EL = 33, RF_ZF = 31, RF_VR = 5;
int LM_EL = 25, LM_ZF = 27, LM_VR = 3;

int pwr = 0;

// Camera stepper
#define motorInterfaceType 1
const int cameraStepEnable = 56;
const int cameraDir = 54;
const int cameraStep = 55;
AccelStepper cameraStepper(motorInterfaceType, cameraStep, cameraDir);

// Geometry
float d1 = 380; // Horizontal distance between middle of rover and corner wheels
float d2 = 350; // Vertical distance between middle of rover and back corner wheels
float d3 = 430; // Vertical distance between middle of rover and front corner wheels
float d4 = 300; // Horizontal distance between middle of rover and centre wheels
float rMin = 850; // Min turning radius (d1 + (d3/tan(45deg))  tan(45) = 1  380 + 430  (add 40mm for safety)
float rMax = 2500; // Max turning radius (~straight)

int angle = 0;   // servo position in degrees
int servoTrimsAddress = 0;
int servoTrims[4] = {90,90,90,90};
int currentWheel = 1;

int s = 0; // rover speed
int r = 0; // turning radius
int m1, m2, m3, m4, m5, m6;
float speed1, speed2, speed3 = 0;
float speed1PWM, speed2PWM, speed3PWM = 0;
float thetaInnerFront, thetaInnerBack, thetaOuterFront, thetaOuterBack = 0;

void setup()
{
	// Start serial monitor
	Serial.begin(115200);
	Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));

	// R/C: Attach iBus object to serial port
	IBus.begin(Serial1);

	readIntArrayFromEEPROM(servoTrimsAddress, servoTrims, 4);

	if (servoTrims[0] <= 0) {
		servoTrims[0] = 90;
		servoTrims[1] = 90;
		servoTrims[2] = 90;
		servoTrims[3] = 90;
	}

	servoW1.attach(1, servoTrims[0]);
	servoW3.attach(3, servoTrims[1]);
	servoW4.attach(0, servoTrims[2]);
	servoW6.attach(2, servoTrims[3]);

	servoW1.setSpeed(90);
	servoW3.setSpeed(90);
	servoW4.setSpeed(90);
	servoW6.setSpeed(90);

	// Motors
	pinMode(RR_EL, OUTPUT);
	pinMode(RR_ZF, OUTPUT);
	pinMode(RR_VR, OUTPUT);
	pinMode(LR_EL, OUTPUT);
	pinMode(LR_ZF, OUTPUT);
	pinMode(LR_VR, OUTPUT);
	pinMode(LF_EL, OUTPUT);
	pinMode(LF_ZF, OUTPUT);
	pinMode(LF_VR, OUTPUT);
	pinMode(LM_EL, OUTPUT);
	pinMode(LM_ZF, OUTPUT);
	pinMode(LM_VR, OUTPUT);
	pinMode(RF_EL, OUTPUT);
	pinMode(RF_ZF, OUTPUT);
	pinMode(RF_VR, OUTPUT);	
	pinMode(RM_EL, OUTPUT);
	pinMode(RM_ZF, OUTPUT);
	pinMode(RM_VR, OUTPUT);

	// Turn motors on
	digitalWrite(RF_EL, HIGH);
	digitalWrite(RM_EL, HIGH);
	digitalWrite(RR_EL, HIGH);
	digitalWrite(LF_EL, HIGH);
	digitalWrite(LM_EL, HIGH);
	digitalWrite(LR_EL, HIGH);

	// Camera stepper
	pinMode(cameraStepEnable, OUTPUT);
	digitalWrite(cameraStepEnable, LOW);

	cameraStepper.setMaxSpeed(1000);
	cameraStepper.setAcceleration(1000);
	cameraStepper.setSpeed(1000);
	
	cameraStepper.moveTo(90 * 1.8 * 8);
	//cameraStepper.moveTo(1200);
}

void loop()
{
	//IBus.loop();
	ch1 = readChannel(0);
	ch2 = readChannel(1);
	ch3 = readChannel(2);
	ch4 = readChannel(3);
	ch5 = readChannel(4);
	ch6 = readChannel(5);

	printChannels();
	
	//if (readSwitch(5, false))  // Channel 6
	//{ 		
	//	setTrims();
	//}
	//else 
	//{
	//	if (BASIC_CONTROL) {
	//		basicControl();
	//	}
	//	else
	//	{
	//		advancedControl();
	//	}
	//}

	if (cameraStepper.distanceToGo() == 0)
		cameraStepper.moveTo(-cameraStepper.currentPosition());
	//	cameraStepper.move(ch4 * 28.8);// 2 * 1.8 * 8);

	cameraStepper.run();
}

void setTrims()
{
	// Channel 5 adjusts trim
	// Channel 3 going to high saves
	// Channel 4 moves to next `/ prev wheel

	int sDeg = map(ch5, 100, -100, 0, 180);
	
	if (currentWheel == 1) {
		servoW1.easeTo(sDeg);
	}
	else if (currentWheel == 2) {
		servoW3.easeTo(sDeg);
	}
	else if (currentWheel == 3) {
		servoW4.easeTo(sDeg);
	}
	else {
		servoW6.easeTo(sDeg);
	}

	if (ch3 > 90) {
		Serial.println("Saving...");
		servoTrims[currentWheel-1] = sDeg;
		delay(2000);
		// Do save
		writeIntArrayIntoEEPROM(servoTrimsAddress, servoTrims, 4);
	}

	if (ch4 > 90) {
		currentWheel++;
		if (currentWheel == 5) { currentWheel = 1; }
		delay(1000);
	}
	else if (ch4 < -90) {
		currentWheel--;
		if (currentWheel == 0) { currentWheel = 4; }
		delay(2000);
	}
}

void advancedControl()
{
	//Serial.print("Ch1:");
	//Serial.print(ch1);

	int sDeg = map(ch1, 100, -100, 0, 180);
	
	if (ch1 > 0) {
		r = map(ch1, 100, 0, rMin, rMax);
	}
	else {
		r = map(ch1, -100, 0, rMin, rMax);
	}
	
	//Serial.print(" r:");
	//Serial.print(r);

	if (ch2 > 0) {
		s = ch2;
	}
	else {
		s = ch2 * -1;
	}
	s = map(s, 0, 100, 0, 255);

	//Serial.print(" s:");
	//Serial.print(s);

	calculateMotorsSpeed();
	/*Serial.print(" SP1:");
	Serial.print(speed1);
	Serial.print(" SP2:");
	Serial.print(speed2);
	Serial.print(" SP3:");
	Serial.print(speed2);*/

	calculateServoAngle();

	// Set motor directions
	if (ch2 < 0) {
		digitalWrite(RR_ZF, HIGH);
		digitalWrite(RM_ZF, HIGH);
		digitalWrite(RF_ZF, HIGH);
		digitalWrite(LR_ZF, LOW);
		digitalWrite(LM_ZF, LOW);
		digitalWrite(LF_ZF, LOW);
	}
	else {
		digitalWrite(RR_ZF, LOW);
		digitalWrite(RF_ZF, LOW);
		digitalWrite(RM_ZF, LOW);
		digitalWrite(LR_ZF, HIGH);
		digitalWrite(LF_ZF, HIGH);
		digitalWrite(LM_ZF, HIGH);
	}

	if (ch1 > 10) {
		// Right
		// Outer wheels
		servoW1.startEaseTo(servoTrims[0] + thetaInnerFront); // front wheel steer right
		servoW3.startEaseTo(servoTrims[1] - thetaInnerBack); // back wheel steer left for overall steering to the right of the rover
		// Inner wheels
		servoW4.startEaseTo(servoTrims[2] + thetaOuterFront);
		servoW6.startEaseTo(servoTrims[3] - thetaOuterBack);

		//Outer wheels run at speed1 (outer wheels are L)
		analogWrite(LR_VR, speed1);
		analogWrite(LF_VR, speed1);
		analogWrite(LM_VR, speed1);
		// Inner front and rear at speed2
		analogWrite(RR_VR, speed2);
		analogWrite(RF_VR, speed2);
		// Inner middle ar speed3
		analogWrite(RM_VR, speed3);
		
	}
	else if (ch1 < -10) {
		servoW1.startEaseTo(servoTrims[0] - thetaOuterFront);
		servoW3.startEaseTo(servoTrims[1] + thetaOuterBack);
		servoW4.startEaseTo(servoTrims[2] - thetaInnerFront);
		servoW6.startEaseTo(servoTrims[3] + thetaInnerBack);
		
		//Outer wheels run at speed1 (outer wheels are R)
		analogWrite(RR_VR, speed1);
		analogWrite(RF_VR, speed1);
		analogWrite(RM_VR, speed1);
		// Inner front and rear at speed2
		analogWrite(LR_VR, speed2);
		analogWrite(LF_VR, speed2);
		// Inner middle ar speed3
		analogWrite(LM_VR, speed3);
	}
	else
	{
		servoW1.startEaseTo(servoTrims[0]);
		servoW3.startEaseTo(servoTrims[1]);
		servoW4.startEaseTo(servoTrims[2]);
		servoW6.startEaseTo(servoTrims[3]);

		//All wheels run at speed1
		analogWrite(RR_VR, speed1);
		analogWrite(RF_VR, speed1);
		analogWrite(RM_VR, speed1);
		analogWrite(LR_VR, speed1);
		analogWrite(LF_VR, speed1);
		analogWrite(LM_VR, speed1);
	}
	//r = map(sDeg, 0, 180, -53, 53);

	delay(100);
}

void basicControl() {

	int sDeg = map(ch1, 100, -100, 0, 180);
	Serial.print("CH:");
	Serial.print(ch1);
	Serial.print("   Degrees:");
	Serial.println(sDeg);

	// 90 === 53

	servoW1.startEaseToD(sDeg, 100);
	servoW3.startEaseToD(sDeg, 100);
	servoW4.startEaseToD(sDeg, 100);
	servoW6.startEaseToD(sDeg, 100);

	/*pwm.setPWM(0, 0, map(ch1, 100, -100, SERVOMIN, SERVOMAX));
	pwm.setPWM(1, 0, map(ch1, 100, -100, SERVOMIN, SERVOMAX));
	pwm.setPWM(2, 0, map(ch1, 100, -100, SERVOMIN, SERVOMAX));
	pwm.setPWM(3, 0, map(ch1, 100, -100, SERVOMIN, SERVOMAX));*/


	//analogWrite(RR_VR, 50);
	//delay(2000);
	//analogWrite(RR_VR, 0);
	//delay(2000);

	if (ch2 > 0) {
		digitalWrite(RR_ZF, HIGH);
		digitalWrite(RM_ZF, HIGH);
		digitalWrite(RF_ZF, HIGH);
		digitalWrite(LR_ZF, LOW);
		digitalWrite(LM_ZF, LOW);
		digitalWrite(LF_ZF, LOW);
		pwr = ch2;
	}
	else {
		digitalWrite(RR_ZF, LOW);
		digitalWrite(RF_ZF, LOW);
		digitalWrite(RM_ZF, LOW);
		digitalWrite(LR_ZF, HIGH);
		digitalWrite(LF_ZF, HIGH);
		digitalWrite(LM_ZF, HIGH);
		pwr = ch2 * -1;
	}

	long val = map(pwr, 0, 100, 0, 255);
	analogWrite(RR_VR, val);
	analogWrite(RF_VR, val);
	analogWrite(RM_VR, val);
	analogWrite(LR_VR, val);
	analogWrite(LF_VR, val);
	analogWrite(LM_VR, val);

	//Serial.println(map(pwr, 0, 100, 0, 255));

	delay(100);
}

void calculateMotorsSpeed() {
	// if no steering, all wheels speed is the same - straight move
	if (ch1 > -20 && ch1 < 20) {
		speed1 = speed2 = speed3 = s;
	}
	// when steering, wheels speed depend on the turning radius value
	else {
		// Outer wheels, furthest wheels from turning point, have max speed
		// Due to the rover geometry, all three outer wheels should rotate almost with the same speed. They differe only 1% so we asume they are the same.
		speed1 = s;
		// Inner front and back wheels are closer to the turing point and have lower speeds compared to the outer speeds
		speed2 = s * sqrt(pow(d3, 2) + pow((r - d1), 2)) / (r + d4);
		// Inner middle wheel is closest to the turning point, has the lowest speed
		speed3 = s * (r - d4) / (r + d4);
	}

	// speed value from 0 to 100% to PWM value from 0 to 255
	speed1PWM = map(round(speed1), 0, 100, 0, 255);
	speed2PWM = map(round(speed2), 0, 100, 0, 255);
	speed3PWM = map(round(speed3), 0, 100, 0, 255);
}

void calculateServoAngle() {
	// Calculate the angle for each servo for the input turning radius "r"
	thetaInnerFront = map(round((atan((d3 / (r + d1)))) * 180 / PI), 0, 53, 0, 90);
	thetaInnerBack = map(round((atan((d2 / (r + d1)))) * 180 / PI), 0, 53, 0, 90);
	thetaOuterFront = map(round((atan((d3 / (r - d1)))) * 180 / PI), 0, 53, 0, 90);
	thetaOuterBack = map(round((atan((d2 / (r - d1)))) * 180 / PI), 0, 53, 0, 90);
}

void printChannels() {

	Serial.print("Ch1: ");
	Serial.print(ch1);
	Serial.print(" | ");

	Serial.print("Ch2: ");
	Serial.print(ch2);
	Serial.print(" | ");

	Serial.print("Ch3: ");
	Serial.print(ch3);
	Serial.print(" | ");

	Serial.print("Ch4: ");
	Serial.print(ch4);
	Serial.print(" | ");

	Serial.print("Ch5: ");
	Serial.print(ch5);
	Serial.print(" | ");

	Serial.print("Ch6: ");
	Serial.print(ch6);
	Serial.print(" | ");

	Serial.println();
}

int readChannel(byte channel) {
	return readChannel(channel, -100, 100, 0);
}

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
	uint16_t ch = IBus.readChannel(channelInput);
	if (ch < 100) return defaultValue;
	return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
	int intDefaultValue = (defaultValue) ? 100 : 0;
	int ch = readChannel(channelInput, 0, 100, intDefaultValue);
	return (ch > 50);
}

void writeIntArrayIntoEEPROM(int address, int numbers[], int arraySize)
{
	int addressIndex = address;
	for (int i = 0; i < arraySize; i++)
	{
		EEPROM.update(addressIndex, numbers[i] >> 8);
		EEPROM.update(addressIndex + 1, numbers[i] & 0xFF);
		addressIndex += 2;
	}
}

void readIntArrayFromEEPROM(int address, int numbers[], int arraySize)
{
	int addressIndex = address;
	for (int i = 0; i < arraySize; i++)
	{
		numbers[i] = (EEPROM.read(addressIndex) << 8) + EEPROM.read(addressIndex + 1);
		addressIndex += 2;
	}
}