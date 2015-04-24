/* SX1509 Library Example 01
   by: Jim Lindblom
   SparkFun Electronics
   license: Beerware. Please use, reuse, share, and modify this
   code. I'd ask that you maintain attribution and open-source.
   If you find it useful, you can buy me a beer when we meet
   some day.
  
   This is the simplest example for the SX1509 Arduino library.
   This sketch shows how you can use the following SX1509 library   
   methods:
     constructor - initialize an instance of the SX1509 library.
       The minimum required parameter is the I2C address of the
       SX1509. Other parameters (resetPin, interruptPin, oscPin)
       are optional (as shown).
     init() - initializes the SX1509. Will perform a reset, start
       up the Wire library and read some registers to make sure
       the SX1509 is operational.
     pinDir(pin, INPUT/OUTPUT) - This method functions much like
       the Arduino pinMode() function. pin should be an SX1509
       pin between 0 and 15. INPUT and OUTPUT work like usual.
     writePin(pin, HIGH/LOW) - Similar to the Arduino
       digitalWrite() function. pin should be between 0 and 15,
       HIGH and LOW work as usual. This function will also
       activate the pull-up and down resistors if the pin is
       configured as an input.
     readPin(pin) - This function works like the digitalRead()
       function of the Arduino. Give it a pin between 0 and 15
       and a 0 or 1 will be returned representing whether the pin
       is LOW or HIGH.

    Hardware: The SX1509 should be hooked up like so:
    SX1509 Pin      Arduino Pin
       3.3V ---------- 3.3V
       GND ----------- GND
       SDA ----------- A4 (or SDA on newer boards)
       SCL ----------- A5 (or SCL on newer boards)
       
       nRST, OSCIO, and nINT are not used in this example. Though
       they can be connected to any digital pin.
   
   See the SX1509_ADDRESS defines to decide which address you need
   to send to the constructor. By default the SX1509 Breakout
   sets both ADDR pins to 0 (so 0x3E I2C address).

   In addition two SX1509 i/o pins are used:
     14 - Output, could be connected to an LED
     1 - Input, could be connected to a button.
*/

#include <Wire.h>  // Wire.h library is required to use SX1509 lib
#include <sx1509_library.h>  // Include the SX1509 library

// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

// Pin definitions, not actually used in this example
const byte interruptPin = 2;
const byte resetPin = 8;

// Create a new sx1509Class object. You can make it with all
// of the above pins:
//sx1509Class sx1509(SX1509_ADDRESS, resetPin, interruptPin);
// Or make an sx1509 object with just the SX1509 I2C address:
sx1509Class sx1509(SX1509_ADDRESS);

// SX1509 pin defintions:
const byte buttonPin = 1;
const byte ledPin = 14;

void setup()
{
  Serial.begin(9600);
  sx1509.init();  // Initialize the SX1509, does Wire.begin()
  sx1509.pinDir(buttonPin, INPUT);  // Set SX1509 pin 1 as an input
  sx1509.writePin(buttonPin, HIGH);  // Activate pull-up
  sx1509.pinDir(ledPin, OUTPUT);  // Set SX1509 pin 14 as an output
  
  // Blink pin 14 a few times
  for (int i=0; i<5; i++)
  {
    sx1509.writePin(ledPin, LOW);  // Write pin LOW
    delay(100);
    sx1509.writePin(ledPin, HIGH);  // Write pin HIGH
    delay(100);
  }
}

void loop()
{
  int buttonValue = sx1509.readPin(buttonPin);  // read pin 1 status
  
  if (buttonValue == 1)  // by default pin 14 should be high
    sx1509.writePin(ledPin, LOW);  // turn pin off
  else
    sx1509.writePin(ledPin, HIGH);  // turn pin on
}