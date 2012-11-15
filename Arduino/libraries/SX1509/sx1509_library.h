#include "Arduino.h"

#ifndef sx1509_library_H
#define sx1509_library_H

#define RECEIVE_TIMEOUT_VALUE 1000	// Timeout for I2C receive

// These are used for setting LED driver to linear or log mode:
#define LINEAR		0	
#define LOGARITHMIC	1

class sx1509Class
{
private:
	// I2C Address
	byte deviceAddress;
	// Pin definitions:
	byte pinInterrupt;
	byte pinOscillator;
	byte pinReset;
	// Private variables:
	bool polling;
	// Functions:
	static void interruptHandler(void);
	byte readByte(byte registerAddress);
	unsigned int readWord(byte registerAddress);
	void readBytes(byte firstRegisterAddress, byte * destination, byte length);
	void writeByte(byte registerAddress, byte writeValue);
	void writeWord(byte registerAddress, unsigned int writeValue);
	void writeBytes(byte firstRegisterAddress, byte * writeArray, byte length);

public:
	sx1509Class(byte address, byte resetPin = 255, byte interruptPin = 255, byte oscillatorPin = 255);	// Constructor
	byte init(void);		// begin Wire, initializes pins, resets the SX1509
	void reset(bool hardware);	// Resets the SX1509
	void pinDir(byte pin, byte inOut);
	void writePin(byte pin, byte highLow);
	byte readPin(byte pin);
	void ledDriverInit(byte pin, byte freq = 1, bool log = 0);
	void pwm(byte pin, byte iOn);
	void sync(void);
	void blink(byte pin, byte tOn, byte toff, 
			   byte offIntensity = 0, byte onIntensity = 255,
			   byte tRise = 0, byte tFall = 0);
	// something with oscio, clock managment
	// void debouncePin(byte pin);
	// void debounceTime(byte time);
	// void keypad(byte rows, byte columns);
	// void levelShifter();
	// void oscio input/output, frequency
};

#endif
