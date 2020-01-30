Overview
========
Example of reading temperature from the MLX90614 temperature sensor via I2C.

The temperature is calculated and printed to the debug console.

Toolchain supported
===================
- IAR embedded Workbench  8.40.2
- Keil MDK  5.29
- GCC ARM Embedded  8.3.1
- MCUXpresso  11.1.0

Hardware requirements
=====================
- Mini USB cable
- FRDM-K64F board
- Personal Computer
- MLX90614 IR Temperature sensor


Board settings
==============
I2C0 on board: 
Used to communicate with the MLX90614 IR temperature sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FRDM-K64F(I2C0)     CONNECTS TO         MLX90614(I2C)
Pin Name   Board Location     Pin Name  Board Location
SCL        J6 pin 20           SCL      pin 1
SDA        J6 pin 18           SDA      pin 2
5V         J3 pin 10		   5V		pin 3 
GND		   J3 pin 12 		   GND	    pin 4
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Prepare the Demo
================
1.  Connect a mini USB cable between the PC host and the OpenSDA USB port on the board.
2.  Open a serial terminal on PC for OpenSDA serial device with these settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Reset the SoC and run the project.


Running the demo
================
The program will print the current temperature measured by the sensor to the console every few seconds.
Customization options
=====================

