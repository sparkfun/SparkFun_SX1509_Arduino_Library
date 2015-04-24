/* SX1509 Library Example 02
   Driving LEDs: PWM, Blink, Breath
   by: Jim Lindblom
   SparkFun Electronics
   license: Beerware. Please use, reuse, share, and modify this
   code. I'd ask that you maintain attribution and open-source.
   If you find it useful, you can buy me a beer when we meet
   some day.
  
   This example shows how you can use the following SX1509 
   library methods:
     constructor - initialize an instance of the SX1509 library.
       The minimum required parameter is the I2C address of the
       SX1509. Other parameters (resetPin, interruptPin, oscPin)
       are optional (as shown).
     init() - initializes the SX1509. Will perform a reset, start
       up the Wire library and read some registers to make sure
       the SX1509 is operational.
     ledDriverInit
     pwm
     blink
     sync

    Hardware: The SX1509 should be hooked up like so:
    SX1509 Pin      Arduino Pin
       3.3V ---------- 3.3V
       GND ----------- GND
       SDA ----------- A4 (or SDA on newer boards)
       SCL ----------- A5 (or SCL on newer boards)
       nRST ---------- 8 (could be any digital pin)
       OSCIO, and nINT are not used in this example.
   
   See the SX1509_ADDRESS defines to decide which address you need
   to send to the constructor. By default the SX1509 Breakout
   sets both ADDR pins to 0 (so 0x3E I2C address).

   In addition SX1509 i/o pins are used to drive LEDs. These
   pins should be sinking current to an LED. So the LED should be
   pulled (through a current-limiting resistor) to 3.3V.
     14 - Breathing output (pins 4-7 and 12-15 can breath)
     3  - Blinking output (any i/o pin can blink)
     0 - PWM output (any i/o pin can pwm)
*/

#include <Wire.h>  // Wire.h library is required to use SX1509 lib
#include <sx1509_library.h>  // Include the SX1509 library

// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
//const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

// Arduino pin definitions
const byte resetPin = 8;

// SX1509 I/O pin definitions
int breatheLED = 14;
int blinkLED = 3;
int pwmLED = 0;

// Create a new sx1509Class object. You can make it with all
// of the above pins:
//sx1509Class sx1509(SX1509_ADDRESS, resetPin, interruptPin);
// Or make an sx1509 object with just the SX1509 I2C address and
// reset pin. Reset is used to demo sync:
sx1509Class sx1509(SX1509_ADDRESS, resetPin);

void setup()
{
  // Must first initialize the sx1509:
  sx1509.init();
  // Now init ledDriver-ness on the LED driver pins:
  // By default these will set the led driver clock (ClkX) to 2MHz
  //  and linear ramp mode.
  sx1509.ledDriverInit(breatheLED);
  sx1509.ledDriverInit(blinkLED);
  sx1509.ledDriverInit(pwmLED);
  // There are two more parameters you can send the ledDriverInit
  // function. One sets log vs. linear ramp mode. The other
  // sets the ClkX divider.
  // clockDivider is a 3-bit value (value between 1 and 7)
  // A bigger clockDivider value will lead to a slower clock
  //     ClkX = 2MHz/(2^(clockDivider-1))
  byte clockDivider = 1;
  sx1509.ledDriverInit(pwmLED, clockDivider, LOGARITHMIC);
  // Note that the log/linear and ClkX divider parameters affect
  // the LED drivers on all pins. The last setting you make will
  // apply to all led driver pins.
  
  // Demo breathe:
  // Play around with some of these variables.
  // onTime and offTime set Ton and Toff as follows:
  //  0: off
  //  1-15: Ton = 64 * onTime * (255 / ClkX)
  //  16-31: Ton = 512 * onTime * (255 / ClkX)
  byte onTime = 16;  // should be between 1 and 31. 0 = off.
  byte offTime = 16;  // big diff between 15 and 16
  byte offIntensity = 0;  // 0=100% off, 7=highest (still fairly low)
  byte onIntensity = 255;  // 0 = 0%, 255 = 100%
  // riseTime and fallTime set TRise and TFall as follows:
  //  0: off
  //  1-15: TRise = (onIntensity) - (4 * offIntensity) * riseTime * (255 / ClkX)
  //  16-31: TRise = 16 * (the 1-15 above equation)
  byte riseTime = 10;  // 31 = longest, 1 = shortest
  byte fallTime = 10;  // big difference between 15 and 16
  sx1509.blink(breatheLED, onTime, offTime, 
               offIntensity, onIntensity,
               riseTime, fallTime);
      
  // Demo blink:
  // To blink an LED, you can send just three parameters to the
  // blink function:  
  sx1509.blink(blinkLED, onTime, offTime);
  // Or you can send two more to indicate the on and off intensity
  sx1509.blink(blinkLED, onTime, offTime, 
               offIntensity, onIntensity);
  
  // If you want to sync up each of the I/O's led driver
  // counters. Try sending a sync command. This requires nReset
  // to be connected:
  sx1509.sync();
}

void loop()
{
  // In the loop, we'll manually cycle this pin. What a pain! 
  // You could just breath it and leave it.
  for (byte intensity=0; intensity<=254; intensity++)
  {
    // The pwm function requires two paramaters:
    // a pin and an intensity between 0 and 255.
    // It works a lot like analogWrite()!
    sx1509.pwm(pwmLED, intensity);
    delay(5);
  }
  delay(500);
  for (byte intensity=255; intensity>0; intensity--)
  {
    sx1509.pwm(pwmLED, intensity);
    delay(5);
  }
  delay(500);
}
