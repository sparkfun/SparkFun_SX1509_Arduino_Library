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
}

byte sx1509Class::init(void)
{
	if (pinInterrupt != 255)
	{
		pinMode(pinInterrupt, INPUT_PULLUP);
	}
	
	// Begin I2C
	Wire.begin();
	
	// If the reset pin is connected
	if (pinReset != 255)
		reset(1);
	else
		reset(0);
	
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

// pwm(byte pin, byte iOn)
//	This function can be used to control the intensity of an output pin.
//	- pin is the pin to be blinked. Should be 0-15.
//	- iOn should be a 0-255 value setting the intensity of the LED
//		- 0 is completely off, 255 is 100% on.
// ledDriverInit should be called on the pin before calling this function.
void sx1509Class::pwm(byte pin, byte iOn)
{
	// Write the on intensity of pin
	// Linear mode: Ion = iOn
	// Log mode: Ion = f(iOn)
	writeByte(REG_I_ON[pin], iOn);
}

// blink(byte pin, byte tOn, byte tOff, byte offIntensity, byte tRise, byte tFall)
//	blink performs both the blink and breath functions.
//  - pin: the pin (0-15) you want to set blinking/breathing.
//	- tOn: the amount of time the pin is HIGH
//		- This value should be between 1 and 31. 0 is off.
//	- tOff: the amount of time the pin is at offIntensity
//		- This value should be between 1 and 31. 0 is off.
//	- offIntensity: How dim the LED is during the off period.
//		- This value should be between 0 and 7. 0 is completely off.
//	- onIntensity: How bright the LED will be when completely on.
//		- This value can be between 0 (0%) and 255 (100%).
//	- tRise: This sets the time the LED takes to fade in.
//		- This value should be between 1 and 31. 0 is off.
//		- This value is used with tFall to make the LED breath.
//	- tFall: This sets the time the LED takes to fade out.
//		- This value should be between 1 and 31. 0 is off.
//  Note: The breathable pins are 4, 5, 6, 7, 12, 13, 14, 15 only. If tRise and tFall
//		are set on 0-3 or 8-11 those pins will still only blink.
// 	ledDriverInit should be called on the pin to be blinked before this.
void sx1509Class::blink(byte pin, byte tOn, byte tOff, 
						byte offIntensity, byte onIntensity,
						byte tRise, byte tFall)
{
	// Keep parameters within their limits:
	tOn &= 0x1F;	// tOn should be a 5-bit value
	tOff &= 0x1F;	// tOff should be a 5-bit value
	offIntensity &= 0x07;
	// Write the time on
	// 1-15:  TON = 64 * tOn * (255/ClkX)
	// 16-31: TON = 512 * tOn * (255/ClkX)
	writeByte(REG_T_ON[pin], tOn);
	
	
	// Write the time/intensity off register
	// 1-15:  TOFF = 64 * tOff * (255/ClkX)
	// 16-31: TOFF = 512 * tOff * (255/ClkX)
	// linear Mode - IOff = 4 * offIntensity
	// log mode - Ioff = f(4 * offIntensity)
	writeByte(REG_OFF[pin], (tOff<<3) | offIntensity);
	
	// Write the on intensity:
	writeByte(REG_I_ON[pin], onIntensity);
	
	// Prepare tRise and tFall
	tRise &= 0x1F;	// tRise is a 5-bit value
	tFall &= 0x1F;	// tFall is a 5-bit value
	
	
	// Write regTRise
	// 0: Off
	// 1-15:  TRise =      (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
	// 16-31: TRise = 16 * (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
	if (REG_T_RISE[pin] != 0xFF)
		writeByte(REG_T_RISE[pin], tRise);
	// Write regTFall
	// 0: off
	// 1-15:  TFall =      (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
	// 16-31: TFall = 16 * (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
	if (REG_T_FALL[pin] != 0xFF)
		writeByte(REG_T_FALL[pin], tFall);
}

// ledDriverInit(byte pin, byte freq, bool log)
//	This function initializes LED driving on a pin. It must be called
//	if you want to use the pwm or blink functions on that pin.
//	- pin should be a pin number between 0 and 15
// 	- freq decides ClkX, and should be a value between 1-7
//		- ClkX = 2MHz / (2^(freq - 1)
//		- freq defaults to 1, which makes ClkX = 2MHz
//	- log selects either linear or logarithmic mode on the LED drivers
//		- log defaults to 0, linear mode
//		- currently log sets both bank A and B to the same mode
//	Note: this function automatically decides to use the internal 2MHz
//		oscillator.
void sx1509Class::ledDriverInit(byte pin, byte freq, bool log)
{
	unsigned int tempWord;
	byte tempByte;
	// Disable input buffer
	// Writing a 1 to the pin bit will disable that pins input buffer
	tempWord = readWord(REG_INPUT_DISABLE_B);
	tempWord |= (1<<pin);
	writeWord(REG_INPUT_DISABLE_B, tempWord);
	
	// Disable pull-up
	// Writing a 0 to the pin bit will disable that pull-up resistor
	tempWord = readWord(REG_PULL_UP_B);
	tempWord &= ~(1<<pin);
	writeWord(REG_PULL_UP_B, tempWord);
	
	// Enable open-drain
	// Writing a 1 to the pin bit will enable open drain on that pin
	tempWord = readWord(REG_OPEN_DRAIN_B);
	tempWord |= (1<<pin);
	writeWord(REG_OPEN_DRAIN_B, tempWord);
	
	// Set direction to output (REG_DIR_B)
	pinDir(pin, OUTPUT);
	
	// Enable oscillator (REG_CLOCK)
	tempByte = readByte(REG_CLOCK);
	tempByte |= (1<<6);	// Internal 2MHz oscillator part 1 (set bit 6)
	tempByte &= ~(1<<5);	// Internal 2MHz oscillator part 2 (clear bit 5)
	writeByte(REG_CLOCK, tempByte);
	
	// Configure LED driver clock and mode (REG_MISC)
	tempByte = readByte(REG_MISC);
	if (log)
	{
		tempByte |= (1<<7);	// set logarithmic mode bank B
		tempByte |= (1<<3);	// set logarithmic mode bank A
	}
	else
	{
		tempByte &= ~(1<<7);	// set linear mode bank B
		tempByte &= ~(1<<3);	// set linear mode bank A
	}
	if (freq == 0)	// don't want it to be 0, that'll disable all led drivers
		freq = 1;
	freq = (freq & 0x07) << 4;	// freq should only be 3 bits from 6:4
	tempByte |= freq;
	writeByte(REG_MISC, tempByte);
	
	// Enable LED driver operation (REG_LED_DRIVER_ENABLE)
	tempWord = readWord(REG_LED_DRIVER_ENABLE_B);
	tempWord |= (1<<pin);
	writeWord(REG_LED_DRIVER_ENABLE_B, tempWord);
	
	// Set REG_DATA bit low ~ LED driver started
	tempWord = readWord(REG_DATA_B);
	tempWord &= ~(1<<pin);
	writeWord(REG_DATA_B, tempWord);
}

// readKeyData()
//	This function returns a 16-bit value containing the values of
//	RegKeyData1 and RegKeyData2. However the bit-values are all
//	complemented, so there should only be a 1 where the active columns
//	and rows are represented.
unsigned int sx1509Class::readKeyData()
{
	return (0xFFFF ^ readWord(REG_KEY_DATA_1));
}

// keypad(byte rows, byte columns, byte sleepTime, byte scanTime)
//	This function will initialize the keypad function on the SX1509.
//	a rows x columns matrix of buttons will be created.
//	- rows is the number of rows in the button matrix.
//		- This value must be between 1 and 7. 0 will turn it off.
//		- 1 = 2 rows, 7 = 8 rows, etc.
//	- columns is the number of columns in the button matrix
//		- This value should be between 0 and 7.
//		- 0 = 1 column, 7 = 8 columns, etc.
//	- sleepTime sets the Auto-sleep time of the keypad engine.
//		- This value should be between 0 and 7. See the comments in the 
//		  function for their values
//	- scanTime sets the scan time per row.
//		- This value should be betwee 0 and 7. See the comments for what 
//        effect this value will have.
void sx1509Class::keypad(byte rows, byte columns, byte sleepTime, byte scanTime)
{
	unsigned int tempWord;
	byte tempByte;
	
	// Set regDir 0:7 outputs, 8:15 inputs:
	tempWord = readWord(REG_DIR_B);
	for (int i=0; i<rows; i++)
		tempWord &= ~(1<<i);
	for (int i=8; i<(columns * 2); i++)
		tempWord |= (1<<i);
	writeWord(REG_DIR_B, tempWord);
	
	// Set regOpenDrain on 0:7:
	tempByte = readByte(REG_OPEN_DRAIN_A);
	for (int i=0; i<rows; i++)
		tempByte |= (1<<i);
	writeByte(REG_OPEN_DRAIN_A, tempByte);
	
	// Set regPullUp on 8:15:
	tempByte = readByte(REG_PULL_UP_B);
	for (int i=0; i<columns; i++)
		tempByte |= (1<<i);
	writeByte(REG_PULL_UP_B, tempByte);
	
	// Enable and configure debouncing on 8:15:
	tempByte = readByte(REG_DEBOUNCE_ENABLE_B);
	for (int i=0; i<columns; i++)
		tempByte |= (1<<i);
	writeByte(REG_DEBOUNCE_ENABLE_B, tempByte);
	writeByte(REG_DEBOUNCE_CONFIG, (scanTime & 0b111));	// Debounce must be less than scan time
	
	// RegKeyConfig1 sets the auto sleep time and scan time per row
	// Auto sleep time: 3-bit value ~
	//		000 : OFF
	//		001 : 128ms x 2MHz/fOSC
	//		010 : 256ms x 2MHz/fOSC
	//		011 : 512ms x 2MHz/fOSC
	//		100 : 1sec x 2MHz/fOSC
	//		101 : 2sec x 2MHz/fOSC
	//		110 : 4sec x 2MHz/fOSC
	//		111 : 8sec x 2MHz/fOSC
	// Scan time per row: 3-bit value, must be set above debounce time ~
	//		000 : 1ms x 2MHz/fOSC
	//		001 : 2ms x 2MHz/fOSC
	//		010 : 4ms x 2MHz/fOSC
	//		011 : 8ms x 2MHz/fOSC
	//		100 : 16ms x 2MHz/fOSC
	//		101 : 32ms x 2MHz/fOSC
	//		110 : 64ms x 2MHz/fOSC
	//		111 : 128ms x 2MHz/fOSC
	sleepTime = (sleepTime & 0b111)<<4;	
	scanTime &= 0b111;	// Scan time is bits 2:0
	tempByte = sleepTime | scanTime;
	writeByte(REG_KEY_CONFIG_1, tempByte);
	
	// RegKeyConfig2 tells the SX1509 how many rows and columns we've got going
	rows = (rows - 1) & 0b111;	// 0 = off, 0b001 = 2 rows, 0b111 = 8 rows, etc.
	columns = (columns - 1) & 0b111;	// 0b000 = 1 column, ob111 = 8 columns, etc.
	writeByte(REG_KEY_CONFIG_2, (rows << 3) | columns);
}

// sync(void)
//	This function resets the PWM/Blink/Fade counters, syncing any blinking LEDs
// 	Two functions are performed:
//		1) Bit 2 of REG_MISC is set, which alters the functionality of the nReset pin
//		2) The nReset pin is toggled low->high, which should reset all LED counters
//		3) Bit 2 of REG_MISC is again cleared, returning nReset to POR functionality
void sx1509Class::sync(void)
{
	// First check if nReset functionality is set
	byte regMisc = readByte(REG_MISC);
	if (!(regMisc & 0x04))
	{
		regMisc |= (1<<2);
		writeByte(REG_MISC, regMisc);
	}
	
	// Toggle nReset pin to sync LED timers
	pinMode(pinReset, OUTPUT);	// set reset pin as output
	digitalWrite(pinReset, LOW);	// pull reset pin low
	delay(1);	// Wait for the pin to settle
	digitalWrite(pinReset, HIGH);	// pull reset pin back high	
	
	// Return nReset to POR functionality
	writeByte(REG_MISC, (regMisc & ~(1<<2)));
}

// nReset(bool hardware)
//	This function resets the SX1509
// 	Two reset methods are available: hardware and software.
//	A hardware reset (hardware parameter = 1) pulls the reset line low, 
//		pausing, then pulling the reset line high.
//	A software reset writes a 0x12 then 0x34 to the REG_RESET as outlined
// 		in the datasheet.
//	No return value.
void sx1509Class::reset(bool hardware)
{
	// if hardware bool is set
	if (hardware)
	{
		// Check if bit 2 of REG_MISC is set
		// if so nReset will not issue a POR, we'll need to clear that bit first
		byte regMisc = readByte(REG_MISC);
		if (regMisc & (1<<2))
		{
			regMisc &= ~(1<<2);
			writeByte(REG_MISC, regMisc);
		}
		// Reset the SX1509, the pin is active low
		pinMode(pinReset, OUTPUT);	// set reset pin as output
		digitalWrite(pinReset, LOW);	// pull reset pin low
		delay(1);	// Wait for the pin to settle
		digitalWrite(pinReset, HIGH);	// pull reset pin back high
	}
	else
	{
		// Software reset command sequence:
		writeByte(REG_RESET, 0x12);
		writeByte(REG_RESET, 0x34);
	}
}

// debounceConfig(byte configValue)
// 	This method configures the debounce time of every input.
//	configValue should be a 3-bit value:
//		000: 0.5ms * 2MHz/fOSC
//		001: 1ms * 2MHz/fOSC
//		010: 2ms * 2MHz/fOSC
//		011: 4ms * 2MHz/fOSC
//		100: 8ms * 2MHz/fOSC
//		101: 16ms * 2MHz/fOSC
//		110: 32ms * 2MHz/fOSC
//		111: 64ms * 2MHz/fOSC
void sx1509Class::debounceConfig(byte configValue)
{
	// First make sure clock is configured
	byte tempByte = readByte(REG_MISC);
	if ((tempByte & 0x70) == 0)
	{
		tempByte |= (1<<4);	// Just default to no divider if not set
		writeByte(REG_MISC, tempByte);
	}
	tempByte = readByte(REG_CLOCK);
	if ((tempByte & 0x60) == 0)
	{
		tempByte |= (1<<6);	// default to internal osc.
		writeByte(REG_CLOCK, tempByte);
	}
	
	configValue &= 0b111;	// 3-bit value
	writeByte(REG_DEBOUNCE_CONFIG, configValue);
}

// debounceEnable(byte pin)
//	This method enables debounce on a input pin on the SX1509.
//	pin should be a value between 0 and 15.
void sx1509Class::debounceEnable(byte pin)
{
	unsigned int debounceEnable = readWord(REG_DEBOUNCE_ENABLE_B);
	debounceEnable |= (1<<pin);
	writeWord(REG_DEBOUNCE_ENABLE_B, debounceEnable);
}
// interruptSource(void)
//	Returns an unsigned int representing which pin caused an interrupt.
//	Pins are represented by their bit position. So a return value of
//		0x0104 would mean pins 8 and 3 (bits 8 and 3) have generated an interrupt
//	This function also clears all interrupts
unsigned int sx1509Class::interruptSource(void)
{
	unsigned int intSource = readWord(REG_INTERRUPT_SOURCE_B);
	writeWord(REG_INTERRUPT_SOURCE_B, 0xFFFF);	// Clear interrupts
	return intSource;
}

// enableInterrupt(byte pin, byte riseFall)
//	This function sets up an interrupt on a pin. Parameters are:
//	-pin: a value between 0 and 15. The pin you want an interrupt enabled on
//	-riseFall: Configures if you want an interrupt generated on rise fall or both.
// 		For this param, send the pin-change values previously defined by Arduino:
//		#define CHANGE 1	<-Both
//		#define FALLING 2	<- Falling
//		#define RISING 3	<- Rising
//	Note: This function does not set up a pin as an input, or configure
//		its pull-up/down resistors! Do that before (or after).
void sx1509Class::enableInterrupt(byte pin, byte riseFall)
{
	// Set REG_INTERRUPT_MASK
	unsigned int tempWord = readWord(REG_INTERRUPT_MASK_B);
	tempWord &= ~(1<<pin);	// 0 = event on IO will trigger interrupt
	writeWord(REG_INTERRUPT_MASK_B, tempWord);
	
	byte sensitivity = 0;
	switch (riseFall)
	{
	case CHANGE:
		sensitivity = 0b11;
		break;
	case FALLING:
		sensitivity = 0b10;
		break;
	case RISING:
		sensitivity = 0b01;
		break;
	}
	
	// Set REG_SENSE_XXX
	// Sensitivity is set as follows:
	// 00: None
	// 01: Rising
	// 10: Falling
	// 11: Both
	byte pinMask = (pin & 0x07) * 2;
	byte senseRegister;
	
	// Need to select between two words. One for bank A, one for B.
	if (pin >= 8)	senseRegister = REG_SENSE_HIGH_B;
	else			senseRegister = REG_SENSE_HIGH_A;
	
	tempWord = readWord(senseRegister);
	tempWord &= ~(0b11<<pinMask);	// Mask out the bits we want to write
	tempWord |= (sensitivity<<pinMask);	// Add our new bits
	writeWord(senseRegister, tempWord);
}

// configClock(byte oscSource, byte oscPinFunction, byte oscFreqOut, byte oscDivider)
// 	This function configures the oscillator source/speed and the 
//		clock, which is used to drive LEDs and time debounces.
//	Parameters to be sent are:
//	- oscSource: Choose either internal 2MHz oscillator or an external signal applied to the OSCIO pin
//		This value defaults to internal.
//		INTERNAL_CLOCK and EXTERNAL_CLOCK are defined in the header file. Use those.
//	- oscPinFunction: Allows you to set OSCIO as an input or output.
//		This value defaults to input
//		You can use Arduino's INPUT, OUTPUT defines for this value
//	- oscFreqOut: If oscio is configured as an output, this will set the output frequency
//		This value defaults to 0.
//		This should be a 4-bit value. 0=0%, 0xF=100%, else fOSCOut = FOSC / (2^(RegClock[3:0]-1))
//	- oscDivider: Sets the clock divider in REG_MISC.
//		This value defaults to 1.
//		ClkX = fOSC / (2^(RegMisc[6:4] -1))
void sx1509Class::configClock(byte oscSource, byte oscPinFunction, byte oscFreqOut, byte oscDivider)
{
	// RegClock constructed as follows:
	//	6:5 - Oscillator frequency souce
	//		00: off, 01: external input, 10: internal 2MHz, 1: reserved
	//	4 - OSCIO pin function
	//		0: input, 1 ouptut
	//	3:0 - Frequency of oscout pin
	//		0: LOW, 0xF: high, else fOSCOUT = FoSC/(2^(RegClock[3:0]-1))
	oscSource = (oscSource & 0b11)<<5;		// 2-bit value, bits 6:5
	oscPinFunction = (oscPinFunction & 1)<<4;	// 1-bit value bit 4
	oscFreqOut = (oscFreqOut & 0b1111);	// 4-bit value, bits 3:0
	byte regClock = oscSource | oscPinFunction | oscFreqOut;
	writeByte(REG_CLOCK, regClock);
	
	// Config RegMisc[6:4] with oscDivider
	// 0: off, else ClkX = fOSC / (2^(RegMisc[6:4] -1))
	oscDivider = (oscDivider & 0b111)<<4;	// 3-bit value, bits 6:4
	byte regMisc = readByte(REG_MISC);
	regMisc &= ~(0b111<<4);
	regMisc |= oscDivider;
	writeByte(REG_MISC, regMisc);
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