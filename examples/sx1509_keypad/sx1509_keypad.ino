/* SX1509 Library Example 03
   Keypad interfacing
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
     - configClock()
     - keypad(numRows, numCols, sleepTime, scanTime)
     - readKeyData()
     - debounceConfig(debounceTime)
     
    Hardware: The SX1509 should be hooked up like so:
    SX1509 Pin      Arduino Pin
       3.3V ---------- 3.3V
       GND ----------- GND
       SDA ----------- A4 (or SDA on newer boards)
       SCL ----------- A5 (or SCL on newer boards)
       nRST ---------- 8 (could be any unused digital pin)
       nINT ---------- 7 (could be any unused digital pin)
       OSCIO is not used in this example.
   
   See the SX1509_ADDRESS defines to decide which address you need
   to send to the constructor. By default the SX1509 Breakout
   sets both ADDR pins to 0 (so 0x3E I2C address).

   In addition SX1509 is connected to a 4x4 matrix of 16 buttons.
   I used the SparkFun 4x4 button pad breakout:
                       https://www.sparkfun.com/products/8033
   You could tie 4 of those together and interface this chip
   with the 8x8 array of 64(!) buttons.
   
   The four rows of the button matrix should be connected to
   the SX1509's 0:3 pins. The columns of the button matrix should
   be connected to the SX1509's 8:11 pins.
*/

#include <Wire.h>  // Wire.h library is required to use SX1509 lib
#include <sx1509_library.h>  // Include the SX1509 library

// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

// Arduino pin definitions
const byte resetPin = 8;
const byte interruptPin = 7;

//////////////////////////////////
//// Global Variables  ///////////
//////////////////////////////////
// Here we'll define the number of rows and columns our keypad
//  matrix has. Each of these values can be between 1 and 8.
//  4x4 = 16 buttons
const byte numRows = 4;
const byte numCols = 4;
// This key map will be used to define what keypress is sent to
//  the computer when a key on the keypad is pressed.
char keyMap[numRows][numCols] = {
  {'1','2','3','4'},
  {'q','w','e','r'},
  {'a','s','d','f'},
  {'z','x','c','v'}};

// Create a new sx1509Class object
sx1509Class sx1509(SX1509_ADDRESS, resetPin, interruptPin);

void setup()
{
  // This example will use a Leonardo to send USB keypresses to
  //  the computer its connected to. The 'Keyboard.' statements
  //  can easily be replaced by Serial.print's, etc.
  #ifdef HID_ENABLED
  Keyboard.begin();
  #else
  Serial.begin(9600);
  #endif
  
  // Must first initialize the sx1509:
  sx1509.init();
  // In order to use the keypad, the clock must first be
  // configured. We can call configureClock() with the default
  // parameters (2MHz internal oscillator, no clock in/out).
  sx1509.configClock();
  // Next call the keypad function with the number of rows
  //  and columns.
  //sx1509.keypad(numRows, numCols);  // Basic keypad init
  // There are two optional parameters in the keypad method:
  //  sleepTime and scanTime. Each of these values can be between
  //  0 and 7. If not set, these values default to 0 (sleep off
  //  scan time set to 1ms).
  byte sleepTime = 7;
  byte scanTime = 2;  // Scan time per row
  sx1509.keypad(numRows, numCols, sleepTime, scanTime);  // Advanced keypad init
  // We can also debounce the keypad inputs. The debounceConfig
  //  method takes one parameter. Similar to the scanTime it's a
  //  3-bit value (0-7). This value must be <= scanTime!
  byte debounceTime = 1;   // The debounce config value
  sx1509.debounceConfig(debounceTime);
}

//////////////////////////////////
//// Loop Variables  /////////////
//////////////////////////////////
unsigned int keyData;  // The raw data from the key press register
unsigned int previousKeyData = 0;  // previously read raw data
byte activeRow;  // The row of the button being pressed
byte activeColumn;  // The column of the button being pressed
// These variables are used to emulate a key-hold. While the key
//  is held down, there's a long delay before the second character
//  is printed. Then a shorter delay between the remaining key presses
unsigned int holdCount = 0;
// This behavior is highly dependent on scanTime and debounceConfig
//  which are set in the setup.
const byte holdCountMax = 25;
// These releaseCount variables 
//  The keypad engin on the SX1509 doesn't generate an interrupt
//  when a key is relased. So we'll use this counter to generate
//  releases.
unsigned int releaseCount = 0;  // Our counter
unsigned int releaseCountMax = 100;  // Top, in about milliseconds

// The loop will poll the interrupt pin. If the pin
//  is pulled low by the SX1509, we'll read the keypad data and
//  sort it into row and column, and send the corresponding key
//  press out to the computer.
void loop()
{
  // The interrupt is active low, and pulled-up otherwise.
  //  The interrupt will be activated whenever a key press is read
  if (!digitalRead(interruptPin))
  {
    // readKeyData() returns a 16-bit word of data. The lower 8-bits 
    //  represent each of the up-to 8 rows. The upper 8-bits
    //  correspond to the columns. A 1 in a bit position means
    //  that a button in that row or column is being pressed.
    keyData = sx1509.readKeyData();
    
    // Next, we'll sort out which row and column are being pressed.
    //  And we'll send out a keypress over USB HID  
    activeRow = keyData & 0xFF;  // The row is the lower 8-bits
    activeColumn = keyData >> 8;  // column is the upper 8-bits
    // The getBitPosition functio will return which bit is our 1
    activeRow = getBitPosition(activeRow);
    activeColumn = getBitPosition(activeColumn);
    
    // If it's a new button press spit it out, reset hold delay
    if (keyData != previousKeyData)
    {
      holdCount = 0;
      // Keyboard.write is a Leonardo-specific Arduino function.
      //  It'll perform a key press and release just like any
      //  keyboard connected to your computer. For testing, this
      //  could easily be replaced by
      #ifdef HID_ENABLED      
      Keyboard.write(keyMap[activeRow][activeColumn]);
      #else
      Serial.print(keyMap[activeRow][activeColumn]);
      #endif
    }
    else
    {
      holdCount++;  // Increment holdCount
      // This works as something of a key-press delay. Hold
      //  down a key on your computer to see what I'm talking
      //  about. After the initial delay, all characters following
      //  will stream out quickly.
      if (holdCount > holdCountMax)
      {
        #ifdef HID_ENABLED      
        Keyboard.write(keyMap[activeRow][activeColumn]);
        #else
        Serial.print(keyMap[activeRow][activeColumn]);
        #endif
      }
    }
    // Reset release count since there's been a key-press
    releaseCount = 0;
    // Set keyData as previousKeyData
    previousKeyData = keyData;
  }
  
  // If no keys have been pressed we'll continuously increment
  //  releaseCount. Eventually creating a release, once the count
  //  hits the max.
  releaseCount++;
  if (releaseCount == releaseCountMax)
  {
    releaseCount = 0;
    previousKeyData = 0;
  }
  delay(1);  // This gives releaseCountMax a more intuitive unit
}

// This function scours a byte and returns the position of the 
//  first byte it sees a 1. Great if our data bytes only have
//  a single 1 in them! Should return 0-7 if it sees a 1, 255 otherwise
byte getBitPosition(byte dataByte)
{
  for (int i=0; i<8; i++)
  {
    if (dataByte & (1<<i))
    {
      return i;
    }
  }
  return 255;  // Otherwise return an error
}
