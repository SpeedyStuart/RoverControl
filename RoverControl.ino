#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <IBusBM.h>

// R/C
IBusBM IBus;
IBusBM IBusSensor;
int ch1, ch2, ch3, ch4, ch5, ch6 = 0;

//Servos
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  350 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60//330 // Analog servos run at ~50 Hz updates

void setup()
{
	// Start serial monitor
	Serial.begin(115200);

	// R/C: Attach iBus object to serial port
	IBus.begin(Serial1);

	// Servos:
	pwm.begin();
	/*
	   In theory the internal oscillator (clock) is 25MHz but it really isn't
	   that precise. You can 'calibrate' this by tweaking this number until
	   you get the PWM update frequency you're expecting!
	   The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
	   is used for calculating things like writeMicroseconds()
	   Analog servos run at ~50 Hz updates, It is importaint to use an
	   oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
	   1) Attach the oscilloscope to one of the PWM signal pins and ground on
		  the I2C PCA9685 chip you are setting the value for.
	   2) Adjust setOscillatorFrequency() until the PWM update frequency is the
		  expected value (50Hz for most ESCs)
	   Setting the value here is specific to each individual I2C PCA9685 chip and
	   affects the calculations for the PWM update frequency.
	   Failure to correctly set the int.osc value will cause unexpected PWM results
	*/
	//pwm.setOscillatorFrequency(27000000);
	pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

// Add the main program code into the continuous loop() function
void loop()
{
	ch1 = readChannel(0);
	ch2 = readChannel(1);
	ch3 = readChannel(2);
	ch4 = readChannel(3);
	ch5 = readChannel(4);
	ch6 = readChannel(5);

	//printChannels();
	/*.setPWM(0, 0, SERVOMIN);

	delay(2000);
	pwm.setPWM(0, 0, SERVOMAX);
	delay(2000);*/

	
	// Drive each servo one at a time using setPWM()
	pwm.setPWM(0, 0, map(ch1, 100, -100, SERVOMIN, SERVOMAX));
	pwm.setPWM(1, 0, map(ch1, 100, -100, SERVOMIN, SERVOMAX));
	
	
	pwm.setPWM(2, 0, map(ch3, 100, -100, SERVOMIN, SERVOMAX));
	pwm.setPWM(3, 0, map(ch4, 100, -100, SERVOMIN, SERVOMAX));
	pwm.setPWM(4, 0, map(ch5, 100, -100, SERVOMIN, SERVOMAX));
	pwm.setPWM(5, 0, map(ch6, 100, -100, SERVOMIN, SERVOMAX));

	delay(100);
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