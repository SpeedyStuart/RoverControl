#include <IBusBM.h>

IBusBM IBus;
IBusBM IBusSensor;
int ch1, ch2, ch3, ch4, ch5, ch6 = 0;


void setup()
{
	// Start serial monitor
	Serial.begin(115200);

	// Attach iBus object to serial port
	IBus.begin(Serial1);

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

	printChannels();

	delay(10);
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