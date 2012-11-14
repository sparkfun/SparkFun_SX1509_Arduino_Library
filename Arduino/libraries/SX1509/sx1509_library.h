#include "Arduino.h"

#ifndef sx1509_library_H
#define sx1509_library_H

#define RECEIVE_TIMEOUT_VALUE 1000	// Timeout for I2C receive

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
	void nReset(void);	// Resets the SX1509
	void pinDir(byte pin, byte inOut);
	void writePin(byte pin, byte highLow);
	byte readPin(byte pin);
};

#endif
