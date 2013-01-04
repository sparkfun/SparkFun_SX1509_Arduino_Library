# SX1509 Arduino Library

# What's here?

* *sx1509_library.cpp*: The core code for the SX1509 library. This file contains the definition of the **sx1509Class** and all of its member functions.
<br>
Depends on the *Wire.h* library.
* *sx1509_library.h*: The header files for the SX1509 library. Includes the declaration of the **sx1509Class** class and some of the constants usable by sketches and the cpp file.
* *sx1509_includes* folder: This folder includes just one file - *sx1509_registers.h* - a list of SX1509 register defines.
* *examples* folder: All of the SX1509 library examples live in this directory. Check these out if you're trying to find out how to use the library. Speaking of which...

# How to use the library

## Install the library
This SX1509 library (the *SX1509* folder) should live in a *libraries* directory within your Arduino sketchbook.

If you're not sure where your sketchbook is, open up the IDE, go to File>Preferences and look at the *sketchbook location* parameter.

## Include the library
Of course, you'll need to include the **SX1509_library** if you plan on using it. You'll also need to **include Wire.h**, Arduino's I<sup>2</sup>C library.

These lines of code, near the top of your sketch, should suffice:<br>
``#include <Wire.h>  // Wire.h library is required to use SX1509 lib``
``#include <sx1509_library.h>  // Include the SX1509 library``


## Constructor
To initialize the SX1509 library, you need to create an instance of it by calling the constructor. This is like creating a global variable of type **sx1509Class**, but with parameters included with the variable name.

In order, the **constructor parameters** are:

* *address*: The 7-bit I<sup>2</sup>C address of your SX1509. There are four possible options (0x3E, 0x3F, 0x70, 0x71), configurable with the ADDR[0:1] pins. (This parameter is **required**).
* *resetPin*: The Arduino pin connected to the SX1509 *nRST* pin. This will reset the chip, and is also used for the *sync* function. (This parameter is **optional**, if note used software resets are enacted.)
* *interruptPin*: This is the Arduino pin connected to the *nINT* pin on the SX1509. The interrupt output is useful if you've got the SX1509 configured as an input device. (This parameter is **optional**.)
* *oscillatorPin*: This is the Arduino pin connected to the *OSCIO* pin on the SX1509. Useful if you plan to supply an external clock, or would like to set the pin as an output. (This parameter is **optional**).

As you can see, the only requred parameter is the I<sup>2</sup>C address. Some examples of a call to the constructor might be:<br>
`` sx1509Class sx1509(0x3E); // Just the I2C address, both ADDR pins are 0 ``<br>
`` sx1509Class sx1509(0x3F, 8); // I2C address (addr[0:1] = 01), and reset pin connected to Arduino pin 8``<br>
`` sx1509Class sx1509(0x70, 8, 3); // I2C address (addr[0:1] = 10), reset pin on D8, interrupt pin on D3.``<br>
`` sx1509Class sx1509(0x71, 8, 3, 5); // I2C address (addr[0:1] = 11), reset on D8, interrupt on D3, clock on D5``<br>

The reset, interrupt, and oscillator pins can be connected to any of the Arduino's digital pins. Interrupt might be well placed on the Arduino's external interrupt pins.

You don't have to call your SX1509 instance *sx1509*, remember you'll be referencing this instance name throughtout the rest of the sketch.

## Initialize the SX1509 instance
Before you can use the functions below, you need to initialize the SX1509 instance you created with the constructor. *init* will set up the Wire library, reset and test communication with the SX1509. Returns a 1 if communication is successful.

The *init* function does **not have any parameters**.

Usually you'll stick it in the Arduino's *setup()* function, just like this:<br>
``sx1509.init();  // Initialize the SX1509, does Wire.begin()``


## Digital I/O Functions

These functions work much like the Arduino *digitalWrite* and *pinMode* functions. You can configure the 16 SX1509 pins as INPUT/OUTPUT, HIGH/LOW, PULL-UP/PULL-DOWN with the following sx1509Class functions.

For an example sketch using these functions, check out the [sx1509_digitalReadWrite example](https://github.com/sparkfun/SX1509_IO-Expander/tree/master/Arduino/libraries/SX1509/examples/sx1509_digitalReadWrite).

### pinDir(byte pin, byte inOut)
Pin direction (INPUT/OUTPUT) control

Input:

* *pin*: should be a value between 0 and 15
* *inOut*: The Arduino INPUT and OUTPUT constants should be used for the inOut parameter.

Examples:<br>
`` sx1509.pinDir(1, INPUT);  // Set SX1509 pin 1 as an input ``<br>
`` sx1509.pinDir(15, OUTPUT); // Set SX1509 pin 15 as an output``

### writePin(byte pin, byte highLow)
Writes a pin to either high or low if it's configured as an OUTPUT. If the pin is configured as an INPUT, this method will activate either the PULL-UP	or PULL-DOWN resistor (HIGH or LOW respectively).

Input:

* *pin*: The SX1509 pin number. Should be a value between 0 and 15.
* *highLow*: should be Arduino's defined HIGH or LOW constants.

Examples:<br>
``sx1509.writePin(1, HIGH);  // If pin 1 is an INPUT, this activates a pull-up resistor ``<br>
``sx1509.writePin(15, LOW);  // If pin 15 is an OUTPUT, this writes the pin LOW``

### readPin(byte pin)
Reads the digital value (HIGH/LOW) of a pin. The pin should be configured as an INPUT.

Input:

* *pin*: The SX1509 pin to be read. should be a value between 0 and 15.

Output:

Returns a 1 if the pin is HIGH, 0 if LOW.

Examples:<br>
``int buttonValue = sx1509.readPin(1);	// Read pin 1's value and store it into a variable``

## LED Driver Functions

The SX1509 allows you to configure any pin as an LED driver with PWM intensity control. In addition to that, each pin can be configured to blink at specific on/off rates. A few select pins can breathe (blinking with fade ins/outs) - pins 4-7 and 12-15.

For an example sketch using these functions, check out the [sx1509_ledDriver example](https://github.com/sparkfun/SX1509_IO-Expander/tree/master/Arduino/libraries/SX1509/examples/sx1509_ledDriver).

Here's the functions available to do some LED driving:

### ledDriverInit(byte pin, byte freq, bool log)
Initializes LED driving on a pin. This function must be called if you want to use the pwm or blink functions on that pin.

Inputs:

* *pin*: The SX1509 pin connected to an LED. Should be 0-15.
* *freq*: decides ClkX, and should be a value between 1-7
	* ClkX = 2MHz / (2^(freq - 1)
	* freq defaults to 1, which makes ClkX = 2MHz
* *log*: selects either linear or logarithmic mode on the LED drivers
	* log defaults to 0, linear mode
	* currently log sets both bank A and B to the same mode

*freq* and *log* are **optional** parameters.

Examples:<br>
``sx1509.ledDriverInit(0, 1, LOGARITHMIC); // Sets SX1509 pin 0 as a ledDriver with a clock divider of 1 and logarithmic brightness control. ``<br>
``sx1509.ledDriverInit(14); // Enables LED driving on pin 14, defaults to no clock divider and linear intensity.``


### pwm(byte pin, byte iOn)
This function can be used to control the intensity of an output pin connected to an LED.

Inputs:

* *pin*: The SX1509 pin connecte to an LED.Should be 0-15.
* *iOn*: should be a 0-255 value setting the intensity of the LED
	* 0 is completely off, 255 is 100% on.

Example:<br>
``sx1509.pwm(0, 127); // Sets LED on pin 0 to medium intensity ``<br>
``sx1509.pwm(14, 255); // Sets LED on pin 14 to 100% on``

### blink(byte pin, byte tOn, byte tOff, byte offIntensity, byte tRise, byte tFall)
Controls both the **blink and breathe** functionality on LED pins.

Input:

* *pin*: the SX1509 pin (0-15) you want to set blinking/breathing.
* *tOn*: the amount of time the pin is HIGH
	* This value should be between 1 and 31. 0 is off.
* *tOff*: the amount of time the pin is at offIntensity
	* This value should be between 1 and 31. 0 is off.
* *offIntensity*: How dim the LED is during the off period.
	* This value should be between 0 and 7. 0 is completely off.
* *onIntensity*: How bright the LED will be when completely on.
	* This value can be between 0 (0%) and 255 (100%).
* *tRise*: This sets the time the LED takes to fade in.
	* This value should be between 1 and 31. 0 is off.
	* This value is used with tFall to make the LED breath.
* *tFall*: This sets the time the LED takes to fade out.
	* This value should be between 1 and 31. 0 is off.

*offIntensity*, *onIntensity*, *tRise*, and *tFall* all are optional parameters. If left undeclared, they'll be set to 0, 255, 0 and 0 respectively.

The breathable pins are 4, 5, 6, 7, 12, 13, 14, 15 only. If *tRise* and *tFall* are set on 0-3 or 8-11 those pins will still only blink.

Examples:<br>
`` sx1509.blink(0, 31, 1); // Blinks an LED on pin 0, on as long as possible, off as short as possible``<br>
`` sx1509.blink(14, 16, 16, 7, 0, 16, 16); // breathes an LED on pin 14, rising and falling for an equal amount of time``

## Keypad Functions

You can matrix up to 64 keys (an an 8x8 matrix) to the SX1509. The keypad scanning engine takes care of all the row scanning and scanning period. Pretty nifty.

For an example sketch using the keypad functions check out the [sx1509_keypad](https://github.com/sparkfun/SX1509_IO-Expander/tree/master/Arduino/libraries/SX1509/examples/sx1509_keypad) example.

There are two functions used by the keypad engine - one to initialize the keypad, and one to read it.

### keypad(byte rows, byte columns, byte sleepTime, byte scanTime)
Initializes the keypad function on the SX1509.

Inputs:

* rows: The number of rows in the button matrix.
	* This value must be between 1 and 7. 0 will turn it off.
	* eg: 1 = 2 rows, 2 = 3 rows, 7 = 8 rows, etc.
* columns: The number of columns in the button matrix
	* This value should be between 0 and 7.
	* 0 = 1 column, 7 = 8 columns, etc.
* sleepTime: Sets the auto-sleep time of the keypad engine. 3-bit value:
	* 0 : OFF
	* 1 : 128ms x 2MHz/fOSC
	* 2 : 256ms x 2MHz/fOSC
	* 3 : 512ms x 2MHz/fOSC
	* 4 : 1sec x 2MHz/fOSC
	* 5 : 2sec x 2MHz/fOSC
	* 6 : 4sec x 2MHz/fOSC
	* 7 : 8sec x 2MHz/fOSC
*scanTime: Sets the scan time per row. Must be set above debounce time. 3-bit value:
	* 0 : 1ms x 2MHz/fOSC
	* 1 : 2ms x 2MHz/fOSC
	* 2 : 4ms x 2MHz/fOSC
	* 3 : 8ms x 2MHz/fOSC
	* 4 : 16ms x 2MHz/fOSC
	* 5 : 32ms x 2MHz/fOSC
	* 6 : 64ms x 2MHz/fOSC
	* 7 : 128ms x 2MHz/fOSC

*sleepTime* and *scanTime* are optional parameters. If left undeclared they'll both be set to 0 (no auto-sleep and fastest scan time, not exactly power-efficient :).

Examples:<br>
``sx1509.keypad(7, 7); // Creates an 8x8 keypad. 64 keys! Fastest scan time.``<br>
``sx1509.keypad(3, 2, 3, 2); // 4 row, 3 column keypad keypad - 12 keys. Sleep about half a second, 4ms scan time.``

### readKeyData()
Returns a 16-bit value containing the status of the keypad engine.

Output: A 16-bit value is returned. The lower 8 bits represent the up-to 8 rows, while the MSB represents the up-to 8 columns. Bit-values of 1 indicate a button in that row or column is being pressed. As such, at least two bits should be set.

Example usage:<br>
``keyData = sx1509.readKeyData(); // read the keypad data into keyData variable``<br>
``row = keyData & 0xFF;  // The row is the lower 8-bits``<br>
``column = keyData >> 8;  // column is the upper 8-bits``<br>
`` Then extract which bit(s) are set in each of the row/column variables to find which button was pressed``

For a more detailed example check out the keypad example sketch!

## Interrupt functions

Want to make use of the SX1509's interrupt output? Want an interrupt generated when a input goes HIGH/LOW/Changes? Check out these functions: *enableInterrupt* and *interruptSource*.

For an example sketch using the SX1509's interrupt functionality check out the [sx1509_interruptDebounce example](https://github.com/sparkfun/SX1509_IO-Expander/tree/master/Arduino/libraries/SX1509/examples/sx1509_interruptDebounce).

### enableInterrupt(byte pin, byte riseFall)
This function sets up an interrupt on a pin. Interrupts can occur on all SX1509 pins, and can be generated on rising, falling, or both.

Inputs:

* *pin*: SX1509 input pin that will generate an input. Should be 0-15.
* *riseFall*: Configures if you want an interrupt generated on rise fall orboth. For this param, send the pin-change values previously defined by Arduino:
	* #define CHANGE 1	<-Both
	* #define FALLING 2	<- Falling
	* #define RISING 3	<- Rising

This function does not set up a pin as an input, or configure its pull-up/down resistors!

### interruptSource(void)
Returns an unsigned int representing which pin caused an interrupt.

Output: 16-bit value, with a single bit set representing the pin(s) that generated an interrupt. E.g. a return value of	0x0104 would mean pins 8 and 3 (bits 8 and 3) have generated an interrupt.

This function also clears all interrupts.

## Debounce functions

Getting some jitter in the SX1509 inputs? Need to add debounce to your keypad scanning engine? Check out these functions: *debounceConfig* and *debounceEnable*.

For an example sketch using the SX1509's debounce functionality check out the [sx1509_interruptDebounce example](https://github.com/sparkfun/SX1509_IO-Expander/tree/master/Arduino/libraries/SX1509/examples/sx1509_interruptDebounce).

### debounceConfig(byte configValue)
This method configures the debounce time of every input.

Input: 

* *configValue*: A 3-bit value configuring the debounce time.
	* 000: 0.5ms * 2MHz/fOSC
	* 001: 1ms * 2MHz/fOSC
	* 010: 2ms * 2MHz/fOSC
	* 011: 4ms * 2MHz/fOSC
	* 100: 8ms * 2MHz/fOSC
	* 101: 16ms * 2MHz/fOSC
	* 110: 32ms * 2MHz/fOSC
	* 111: 64ms * 2MHz/fOSC

fOSC is set with the configClock function. It defaults to 2MHz.

### debounceEnable(byte pin)
This method enables debounce on SX1509 input pin.

Input: 

* *pin*: The SX1509 pin to be debounced. Should be between 0 and 15.

*debounceConfig* should be called before this, to configure the clock and other debounce parameters.

## Utility functions

Need to **reset** the SX1509 (hardware or software)? Want to **configure the clock**, use an external clock, our set that OSCIO pin as an output? Need to **sync up blinking LEDs**? Check out some of these utility functions.

These functions are all interspersed through many of the library examples sketches.

### reset(bool hardware)
This function resets the SX1509 - either a hardware reset or software. A hardware reset (hardware parameter = 1) pulls the reset line low, pausing, then pulling the reset line high. A software reset writes a 0x12 then 0x34 to the REG_RESET as outlined in the datasheet.

Input:

* *hardware*: 0 executes a software reset, 1 executes a hardware reset.

### configClock(byte oscSource, byte oscPinFunction, byte oscFreqOut, byte oscDivider)
This function configures the oscillator source/speed and the clock, which is used to drive LEDs and time debounces.

Inputs:

* *oscSource*: Choose either internal 2MHz oscillator or an external signal applied to the OSCIO pin.
	* INTERNAL_CLOCK and EXTERNAL_CLOCK are defined in the header file. Use those.
	* This value defaults to internal.
* *oscPinFunction*: Allows you to set OSCIO as an input or output.
	* You can use Arduino's INPUT, OUTPUT defines for this value
	* This value defaults to input
* *oscFreqOut*: If oscio is configured as an output, this will set the output frequency
	* This should be a 4-bit value. 0=0%, 0xF=100%, else fOSCOut = FOSC / (2^(RegClock[3:0]-1))
	* This value defaults to 0.
* *oscDivider*: Sets the clock divider in REG_MISC.
	* ClkX = fOSC / (2^(RegMisc[6:4] -1))
	* This value defaults to 1.

### sync()
Resets the PWM/Blink/Fade counters, syncing any blinking LEDs. Bit 2 of REG_MISC is set, which alters the functionality of the nReset pin. The nReset pin is toggled low->high, which should reset all LED counters. Bit 2 of REG_MISC is again cleared, returning nReset pin to POR functionality.

No inputs or outputs. Just does.

# License

All of the library code and example code is written by Jim Lindblom @ SparkFun Electronics and release under the **Beerware** license! Please use, reuse, share, and modify this code. I'd ask that you maintain attribution and open-source. If you find it useful, you can buy me a beer when we meet some day.

If you do make use of the library, or if you find places for improvement, I'd love to hear about it!

Thanks for checking this out.

Snooch.
