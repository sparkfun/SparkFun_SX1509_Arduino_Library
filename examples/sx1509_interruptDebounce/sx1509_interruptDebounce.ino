/* SX1509 Library Example 03
   Buttons with interrupts, and debouncing
   by: Jim Lindblom
   SparkFun Electronics
   license: Beerware. Please use, reuse, share, and modify this
   code. I'd ask that you maintain attribution and open-source.
   If you find it useful, you can buy me a beer when we meet
   some day.
  
   This example shows how you can use the following SX1509 
   library methods:
     - constructor
     - init()
     - pinDir
     - writePin
     - readPin
     - configClock
     - enableInterrupt
     - debounceConfig
     - debounceEnable
     - pwm

    Hardware: The SX1509 should be hooked up like so:
    SX1509 Pin      Arduino Pin
       3.3V ---------- 3.3V
       GND ----------- GND
       SDA ----------- A4 (or SDA on newer boards)
       SCL ----------- A5 (or SCL on newer boards)
       nRST ---------- 8 (could be any digital pin)
       nINT ---------- 3 (could be any digital pin)
       OSCIO is not used in this example.
   
   See the SX1509_ADDRESS defines to decide which address you need
   to send to the constructor. By default the SX1509 Breakout
   sets both ADDR pins to 0 (so 0x3E I2C address).

   In addition SX1509 i/o pins are used. The pwmPin should be 
   sinking current to an LED. So the LED should be pulled 
   (through a current-limiting resistor) to 3.3V. One button pin
   is active high, the other is active low to demo both functions.
     14 - PWM'ing LED
     1 and 5 - Input buttons. One tied high, other tied low.
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
const byte interruptPin = 3;

// SX1509 I/O pin definitions
const byte upPin = 1;
const byte downPin = 5;
int pwmLED = 14;

// Create a new sx1509Class object
sx1509Class sx1509(SX1509_ADDRESS, resetPin, interruptPin);

// global variables
byte intensity = 0;
const byte increment = 1;

void setup()
{
  // Must first initialize the sx1509:
  sx1509.init();
  
  // Configure the PWM'ing LED. For more info see example 2.
  sx1509.ledDriverInit(pwmLED);
  sx1509.pwm(pwmLED, intensity);
        
  // Setup the button pins. The upPin will be pulled low, while
  //  the downPin will be pulled high. The upPin button will cause
  //  that pin to go high. The downPin button will cause that pin
  //  to go low.
  sx1509.pinDir(upPin, INPUT);  // Configures the pin as an input
  sx1509.writePin(upPin, LOW);  // Activates pull-down resistor
  // Now we enable the interrupt on upPin. A signal going from 0V
  //  to 3.3V (rising) will mean the button is being pressed.
  //  That's when we want the interrupt to trigger.
  sx1509.enableInterrupt(upPin, RISING);
  
  sx1509.pinDir(downPin, INPUT);  // Configures the pin as an input
  sx1509.writePin(downPin, HIGH);  // Activates pull-up resistor
  // Now we enable the interrupt on downPin. A signal going from 3.3V
  //  to 0 (falling) will mean the button is being pressed.
  //  That's when we want the interrupt to trigger.
  sx1509.enableInterrupt(downPin, FALLING);
  
  // Stuff to configure button debouncing:
  
  // The clock must be configured for debouncing to work:
  // The configClock command takes 4 parameters (all optional).
  // First is whether to use an external clock signal or external.
  //  Internal is easier, requires less connections to SX1509.
  // Next two parameters can be used to configure OSCIO as an
  //  output or input. And sets the output frequency (if config'd
  //  as an output.
  // Last parameter sets the clock divider. This should be a value
  //  between 1 and 7. clkX = fOSC / (2^(thisValue - 1)
  sx1509.configClock(INTERNAL_CLOCK, OUTPUT, 0xE, 1);
  // Sending this command with no parameters is also an option.
  //  This will automatically select the internal oscillator.
  //  Divider of 1 on the clock (so it'll go at 2MHz)
  //  And leave the OSCIO pin unused:
  //sx1509.configClock();
  
  // the debounceConfig function sets the debounce time. This
  //  function's parameter should be a 3-bit value. 
  // 0: 0.5ms * 2MHz/fOSC
  // 1: 1ms * 2MHz/fOSC
  // 2: 2ms * 2MHz/fOSC
  // 3: 4ms * 2MHz/fOSC
  // 4: 8ms * 2MHz/fOSC
  // 5: 16ms * 2MHz/fOSC
  // 6: 32ms * 2MHz/fOSC
  // 7: 64ms * 2MHz/fOSC
  sx1509.debounceConfig(7);  // maximum debuonce time
  
  // Now we call debounceEnable on the pins we want to debounce:
  sx1509.debounceEnable(upPin);
  sx1509.debounceEnable(downPin);
}

// The loop will demo the interruptSource() function as well
//  as readPin and pwm.
void loop()
{
  // The interrupt pin will go low when an interrupt has occured
  if (digitalRead(interruptPin) == 0)
  {
    // The interruptSource function will return a 16-bit value.
    //   where each bit represents on of the pins. A 1-value
    //   bit means that pin was the source of an interrupt.
    unsigned int interruptSource = sx1509.interruptSource();
    
    // When need to check a particular bit against the
    //  interruptSource return value. This will check if upPin
    //  created the interrupt:
    if (interruptSource & (1<<upPin))
    {
      // Let's stay here until the upPin is released. upPin is
      //  pulled low, when the button is pressed it goes high.
      //  So do this loop while the upPin remains high.
      while (sx1509.readPin(upPin) == 1) 
      {
        // Increment intensity, and output that to the pwm'ing LED
        intensity+=increment;
        sx1509.pwm(pwmLED, intensity);
      }
    }
    
    // Now we'll check if downPin created the interrupt.
    if (interruptSource & (1<<downPin))
    {
      // If so, do the same thing while this pin reads low.
      while (sx1509.readPin(downPin) == 0) 
      {
        // Decrement intensity, and send that new value to the pwm
        intensity-=increment;
        sx1509.pwm(pwmLED, intensity);
      }
    }
  }
}
