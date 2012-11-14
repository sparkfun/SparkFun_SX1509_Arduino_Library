#include <Wire.h>
#include "Arduino.h"
#include "sx1509_library.h"
#include "sx1509_includes/sx1509_registers.h"

sx1509Class::sx1509Class(byte address, byte resetPin, byte interruptPin, byte oscillatorPin)
{
	// Store the received parameters into member variables
	deviceAddress =  address;
	pinInterrupt = interruptPin;
	pinOscillator = oscillatorPin;
	pinReset = resetPin;
	
	if ((pinInterrupt == 255) || (pinInterrupt != 2) || (pinInterrupt != 3))
		polling = 1;
	else
		polling = 0;
}

byte sx1509Class::init(void)
{
	// If we're not in polling mode, setup the interrupt pin as an input
	if (!polling)
	{
		pinMode(pinInterrupt, INPUT_PULLUP);
		if (pinInterrupt == 2)
		{
			attachInterrupt(0, &sx1509Class::interruptHandler, FALLING);
		}
		else if (pinInterrupt == 3)
		{
			attachInterrupt(1, &sx1509Class::interruptHandler, FALLING);			
		}
		interrupts();
	}
	
	// If the reset pin is connected
	if (pinReset != 255)
	{
		nReset();
	}
	
	// Begin I2C
	Wire.begin();
	
	// Communication test. We'll read from two registers with different
	// default values to verify communication.
	unsigned int testRegisters = 0;
	testRegisters = readWord(REG_INTERRUPT_MASK_A);	// This should return 0xFF00
	// Then read a byte that should be 0x00
	if (testRegisters == 0xFF00)
		return 1;
	else
		return 0;
}

// pinDir(byte pin, byte inOut)
//	This function sets one of the SX1509's 16 outputs to either an INPUT or OUTPUT
//	- pin should be a value between 0 and 15
//	- The Arduino INPUT and OUTPUT constants should be used for the inOut parameter.
//		- Even though the ATmega and SX1509 binary values for input and output are not equivalent
//		  We'll handle the 1/0 switching in this function
//	- No return value
void sx1509Class::pinDir(byte pin, byte inOut)
{
	unsigned int tempRegDir = readWord(REG_DIR_B);
	// The SX1509 RegDir registers: REG_DIR_B, REG_DIR_A
	//	0: IO is configured as an output
	//	1: IO is configured as an input
	inOut = !inOut;  // Flip inOut, in arduino.h INPUT = 0, OUTPUT = 1
	if (inOut)	tempRegDir |= (1<<pin);
	else		tempRegDir &= ~(1<<pin);
	
	writeWord(REG_DIR_B, tempRegDir);
}

// writePin(byte pin, byte highLow)
//	This function writes a pin to either high or low if it's configured as an output
//	If the pin is configured as an input, this method will activate either the pull-up
//	or pull-down resistor (HIGH or LOW respectively).
//	- pin should be a value between 0 and 15
//	- highLow should be Arduino's defined HIGH or LOW constants
void sx1509Class::writePin(byte pin, byte highLow)
{
	unsigned int tempRegDir = readWord(REG_DIR_B);
	
	if ((0xFFFF^tempRegDir)&(1<<pin))	// If the pin is an output, write high/low
	{
		unsigned int tempRegData = readWord(REG_DATA_B);
		if (highLow)	tempRegData |= (1<<pin);
		else			tempRegData &= ~(1<<pin);
		writeWord(REG_DATA_B, tempRegData);
	}
	else	// Otherwise the pin is an input, pull-up/down
	{
		unsigned int tempPullUp = readWord(REG_PULL_UP_B);
		unsigned int tempPullDown = readWord(REG_PULL_DOWN_B);
		
		if (highLow)	// if HIGH, do pull-up, disable pull-down
		{
			tempPullUp |= (1<<pin);
			tempPullDown &= ~(1<<pin);
			writeWord(REG_PULL_UP_B, tempPullUp);
			writeWord(REG_PULL_DOWN_B, tempPullDown);
		}
		else	// If LOW do pull-down, disable pull-up
		{
			tempPullDown |= (1<<pin);
			tempPullUp &= ~(1<<pin);
			writeWord(REG_PULL_UP_B, tempPullUp);
			writeWord(REG_PULL_DOWN_B, tempPullDown);
		}
	}
}

// readPin(byte pin, byte highLow)
//	This function writes a pin to either high or low.
//	- pin should be a value between 0 and 15
//	- This function returns a 1 if HIGH, 0 if LOW
byte sx1509Class::readPin(byte pin)
{
	unsigned int tempRegDir = readWord(REG_DIR_B);
	
	if (tempRegDir&(1<<pin))	// If the pin is an input
	{
		unsigned int tempRegData = readWord(REG_DATA_B);
		if (tempRegData&(1<<pin))
			return 1;
	}
	
	return 0;
}

// nReset(void)
//	This function resets the SX1509 - pulling the reset line low, pausing, then pulling the reset line high.
//	No return value.
void sx1509Class::nReset(void)
{
	// Reset the SX1509, the pin is active low
	pinMode(pinReset, OUTPUT);	// set reset pin as output
	digitalWrite(pinReset, LOW);	// pull reset pin low
	delay(1);	// Wait for the pin to settle
	digitalWrite(pinReset, HIGH);	// pull reset pin back high
}

// readByte(byte registerAddress)
//	This function reads a single byte located at the registerAddress register.
//	- deviceAddress should already be set by the constructor.
//	- Return value is the byte read from registerAddress
//		- Currently returns 0 if communication has timed out
byte sx1509Class::readByte(byte registerAddress)
{
	byte readValue;
	unsigned int timeout = RECEIVE_TIMEOUT_VALUE;

	Wire.beginTransmission(deviceAddress);
	Wire.write(registerAddress);
	Wire.endTransmission();
	Wire.requestFrom(deviceAddress, (byte) 1);

	while ((Wire.available() < 1) && (timeout != 0))
		timeout--;
		
	if (timeout == 0)
		return 0;

	readValue = Wire.read();

	return readValue;
}

// readWord(byte registerAddress)
//	This function will read a two-byte word beginning at registerAddress
//	- A 16-bit unsigned int will be returned.
//		- The msb of the return value will contain the value read from registerAddress
//		- The lsb of the return value will contain the value read from registerAddress + 1
unsigned int sx1509Class::readWord(byte registerAddress)
{
	unsigned int readValue;
	unsigned int msb, lsb;
	unsigned int timeout = RECEIVE_TIMEOUT_VALUE * 2;

	Wire.beginTransmission(deviceAddress);
	Wire.write(registerAddress);
	Wire.endTransmission();
	Wire.requestFrom(deviceAddress, (byte) 2);

	while ((Wire.available() < 2) && (timeout != 0))
		timeout--;
		
	if (timeout == 0)
		return 0;
	
	msb = (Wire.read() & 0x00FF) << 8;
	lsb = (Wire.read() & 0x00FF);
	readValue = msb | lsb;

	return readValue;
}

// readBytes(byte firstRegisterAddress, byte * destination, byte length)
//	This function reads a series of bytes incrementing from a given address
//	- firstRegsiterAddress is the first address to be read
//	- destination is an array of bytes where the read values will be stored into
//	- length is the number of bytes to be read
//	- No return value.
void sx1509Class::readBytes(byte firstRegisterAddress, byte * destination, byte length)
{
	byte readValue;

	Wire.beginTransmission(deviceAddress);
	Wire.write(firstRegisterAddress);
	Wire.endTransmission();
	Wire.requestFrom(deviceAddress, length);
	
	while (Wire.available() < length)
		;
	
	for (int i=0; i<length; i++)
	{
		destination[i] = Wire.read();
	}
}

// writeByte(byte registerAddress, byte writeValue)
//	This function writes a single byte to a single register on the SX509.
//	- writeValue is written to registerAddress
//	- deviceAddres should already be set from the constructor
//	- No return value.
void sx1509Class::writeByte(byte registerAddress, byte writeValue)
{
	Wire.beginTransmission(deviceAddress);
	Wire.write(registerAddress);
	Wire.write(writeValue);
	Wire.endTransmission();
}

// writeWord(byte registerAddress, ungisnged int writeValue)
//	This function writes a two-byte word to registerAddress and registerAddress + 1
//	- the upper byte of writeValue is written to registerAddress
//		- the lower byte of writeValue is written to registerAddress + 1
//	- No return value.
void sx1509Class::writeWord(byte registerAddress, unsigned int writeValue)
{
	byte msb, lsb;
	msb = ((writeValue & 0xFF00) >> 8);
	lsb = (writeValue & 0x00FF);
	Wire.beginTransmission(deviceAddress);
	Wire.write(registerAddress);
	Wire.write(msb);
	Wire.write(lsb);
	Wire.endTransmission();	
}

// writeBytes(byte firstRegisterAddress, byte * writeArray, byte length)
//	This function writes an array of bytes, beggining at a specific adddress
//	- firstRegisterAddress is the initial register to be written.
//		- All writes following will be at incremental register addresses.
//	- writeArray should be an array of byte values to be written.
//	- length should be the number of bytes to be written.
//	- no return value.
void sx1509Class::writeBytes(byte firstRegisterAddress, byte * writeArray, byte length)
{
	Wire.beginTransmission(deviceAddress);
	Wire.write(firstRegisterAddress);
	for (int i=0; i<length; i++)
	{
		Wire.write(writeArray[i]);
	}
	Wire.endTransmission();
}

void sx1509Class::interruptHandler(void)
{
}