Overview
========
The Pixy2 basic demo shows an example of initializing the Pixy2 camera using SPI and getting the position of detected blocks. It connects to the camera and prints the width, height, location and signature of the detected blocks.

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
- Pixy2 Camera

Board settings
==============
SPI0 on board: 
Used to communicate with the Pixy2 Camera 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FRDM-K64F(SPI0)     CONNECTS TO         Pixy2 Cam(SPI)
Pin Name   Board Location     Pin Name  Board Location
MISO       J2 pin 10           MISO      J2 pin 1
MOSI       J2 pin 8            MOSI      J2 pin 4
SCK        J2 pin 12           SCK       J2 pin 3
PCS0       J2 pin 6            PCS0      J2 pin 7
5V         J3 pin 10           5V        J2 pin 2 
GND        J3 pin 12           GND       J2 pin 6
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
The program should print information about the detected objects such as width, height and location.




Customization options
=====================

