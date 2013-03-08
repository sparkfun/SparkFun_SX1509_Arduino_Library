SX1509 16 Output I/O Expander Breakout
======================================

[![SX1509 16 Output I/O Expander Breakout](https://dlnmh9ip6v2uc.cloudfront.net/images/products/1/1/5/0/2/11502-01_medium.jpg)  
*SX1509 16 Output I/O Expander Breakout (BOB-11502)*](https://www.sparkfun.com/products/11502)

The SX1509 16 Output I/O Expander Breakout Board makes it easy to prototype using the SX1509 so you can add more I/O onto your Arduino or I/O limited controller. It can leverage your I2C interface for 16 extra channels of GPIO and contains a fully programmable LED driver and a keypad scanning engine which enables continuous keypad monitoring of up to 64 keys. Since the I/O banks can operate between 1.2V and 3.6V (5.5V tolerant) independent of both the core and each other, this device can also work as a level-shifter.

Features include:

* 16 channels of bi-directional I/O - Pull-up/down resistors, push/pull or open-drain outputs, programmable polarity.
* 5.5V tolerant I/O's (1.2-3.6V operating voltage)
* LED Driver: PWM and blink control on all I/O's, breathe capability on a select few.
* Level shifting I/O's: Independent I/O rails (VCC1, VCC2) allow for up/down level shifting.
* Keypad scanning engine: Supports up to 64 keys (in an 8x8 matrix configuration).

Repository Contents
-------------------

* **/Arduino** - This directory contains the Arduino library. This directory's structure follows that of your standard Arduino sketchbook installation. A *libraries* folder within contains the **SX1509 library**.

* **/hardware** - This directory contains the Eagle design files - the breakout PCB and schematic design. These files were created with version 6.2.0 of Eagle Cad - a lite version is available, for free, at [www.cadsoftusa.com](http://www.cadsoftusa.com).

License Information
-------------------

The hardware is released under [Creative Commons Share-alike 3.0](http://creativecommons.org/licenses/by-sa/3.0/).  
All other code is open source hardware so please feel free to do anything you want with it; you buy me a beer if you use this and we meet someday ([Beerware license](http://en.wikipedia.org/wiki/Beerware)).

Author
------

Jim Lindblom, SparkFun Electronics
