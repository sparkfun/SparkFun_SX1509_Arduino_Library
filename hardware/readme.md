# What Are these Files?
The .sch and .brd files hare are Eagle CAD schematic and PCB design files.

These files were created with Eagle 6.2.0, you'll need Eagle 6.0 or later to open them up. There is a free, lite, version of Eagle available from [cadsoftusa.com](http://www.cadsoftusa.com).

# Board Features

It's just a breakout board for the SX1509, but it does have its fair share of shortcuts.

## Jumpers

There are five surface mount jumpers located on the back side of the board: two to select the I<sup>2</sup>C address and two to enable the VCC1 and VCC2 rails for I/O's 0-7 and 8-15 respectively. And one jumper to disable/enable 10k pull-up resistors.

<div align="center"><a href="https://dlnmh9ip6v2uc.cloudfront.net/tutorialimages/SX1509/back_annotated.png"><img src="https://dlnmh9ip6v2uc.cloudfront.net/tutorialimages/SX1509/back_annotated_400.png"></a></div>

All jumpers are set to a default value with a small, exposed trace between the pads. With a semi-sharp exacto blade, these traces should be relatively easy to cut.

In the case of the address jumper, if you cut the trace, you'll need to apply a bubble of solder jumping the middle pad to **one** of the other two edge pads to set the ADDR pins.

### Address Jumpers

These two jumper select the I<sup>2</sup>C address of the SX1509. These jumpers allow you to select the value of the **SX1509's ADDR0 and ADDR1** pins - either VCC (1) or GND (0).

These jumpers both default to GND (0).

A table of possible values is printed on the PCB, but I'll stick it here too.

<table border="1" align="center">
<tr align="center"><td><b>ADDR1</b></td><td><b>ADDR0</b></td><td><b>SX1509 7-bit Address</b></td></b></tr>
<tr align="center"><td>0</td><td>0</td><td>0x3E</td></tr>
<tr align="center"><td>0</td><td>1</td><td>0x3F</td></tr>
<tr align="center"><td>1</td><td>0</td><td>0x70</td></tr>
<tr align="center"><td>1</td><td>1</td><td>0x71</td></tr>
</table>

### I/O Rail Jumpers

If you're looking to take advantage of the SX1509's level-shifting functionality, you may want to use the VCC1 and VCC2 pins. These pins can supply the voltage for I/O pins 0-7 and 8-15 respectively.

**Closed**, these jumpers will **short VCC1 and VCC2 to VCC** (the supply voltage 1.2-3.6V). Open the jumpers and you'll have to connect your own voltage supply. If no supply is tied to VCC1 and VCC2, the I/O pins will not work.

These jumpers **default to closed**, which means both I/O rails are tied to the supply voltage (usually 3.3V).

### Pull-up Resistor Jumpers

The SX1509 Breakout comes with 10k resistors to pull-up the I<sup>2</sup>C lines - SDA and SCL. This jumper ties both of those resistors to VCC.

The middle pad of this three-way jumper is connected to VCC. On either side is a connection to a resistor, which connects on the other end to SDA or SCL. Shorting both jumpers to VCC connects both resistors to VCC. Cutting the jumper creates an open connection on the resistor, basically removing them from the circuit.

By default, both jumpers are shorted, turning the pull-up resistors on. Should your setup require the pull-up resistors not be connected, take an exacto blade to both wire snippets.

## 2x5 Pin Headers

All I/O, power and GND pins are broken out to either side of the breakout in a breadboard-friendly manner. But 16+ wires could create quite a mess upon exiting a breadboard and entering a project. So we also made all of those 16 I/O's ribbon cable compatible.

The two 2x5 headers should mate with our [2x5 Pin Shrouded Headers](https://www.sparkfun.com/products/8506), which could mate with [Ribbon Crimp Connectors](https://www.sparkfun.com/products/10650), which could mate with [10-Wire Ribbon Cable](https://www.sparkfun.com/products/10647)!

Each of the 2x5 headers breakout out 8 I/O pins, a GND pin, and the VCC rail for that set of pins. The pinout of each connector is on the bottom of the board, but check out this picture if the bottom isn't easily visible.

<div align="center"><a href="https://dlnmh9ip6v2uc.cloudfront.net/tutorialimages/SX1509/top_annotated.png"><img src="https://dlnmh9ip6v2uc.cloudfront.net/tutorialimages/SX1509/top_annotated_400.png"></a></div>

# License
This project is released as **beerware**. Feel free to use it, with or without attribution, in your own projects. If you find it helpful, buy me a beer (alcoholic or root variety are both fine :) next time we meet up.
