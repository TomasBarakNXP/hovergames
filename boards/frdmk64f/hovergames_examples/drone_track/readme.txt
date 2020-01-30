Overview
========
This demo uses the Pixy2 camera to track a learned signature and commands the drone to follow it. It also reads the temperature using an IR temperature sensor and sends this information via a debug console to a base station.

Toolchain supported
===================
- IAR embedded Workbench  8.40.2
- Keil MDK  5.29
- GCC ARM Embedded  8.3.1
- MCUXpresso  11.1.0

Hardware requirements
=====================
- Assembled Hovergames drone kit
- Mini USB cable
- FRDM-K64F board
- Pixy2 Camera
- HC-05 Bluetooth Module (optional)
- Personal Computer
- JST connector cables for linking the FRDM-K64F board to the RDDRONE-FMUK66 FMU



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
5V         J3 pin 10		   5V		 J2 pin 2 
GND		   J3 pin 12 		   GND	     J2 pin 6
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

UART3 on board: 
Used to communicate with the RDDRONE-FMUK66 FMU 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FRDM-K64F(UART3)     CONNECTS TO         RDDRONE-FMUK66(UART)
Pin Name   Board Location     Pin Name  Board Location
TX         J1 pin 4            RX       Telemetry RX
RX         J1 pin 2            TX       Telemetry TX
GND		   J3 pin 12 		   GND	    Telemetry GND
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

UART1 on board: 
Connected to the Bluetooth transparent serial interface used for sending debugging information remotely
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FRDM-K64F(UART1)     CONNECTS TO        HC-05(UART)
Pin Name   Board Location     Pin Name  Board Location
TX         J2 pin 4            RX       RX
RX         J1 pin 16           TX       TX
5V         J3 pin 10		   5V		5V
GND		   J3 pin 12 		   GND	    GND
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


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
1.  Connect a mini USB cable to the Pixy2 camera and using the PixyMon v2 software set the first signature to the color of the desired target.
2.  Connect a mini USB cable between the PC host and the OpenSDA USB port on the board.
3.  Open a serial terminal on PC for OpenSDA serial device with these settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Download the program to the target board.
5.  Reset the SoC and run the project.
6.  Start QGroundControl on your groundstation and wait for connection to drone
7. 	Arm drone and set flight mode to Acro (make sure camera is not tracking anything when arming drone)

Running the demo
================
When the demo runs successfully, the drone propeller should start up in a idle mode. When an object is tracked, the drone should run at 50% throttle and attempt to rotate in order to track the object. At the same time, temperature data form the sensor is transmitted via Bluetooth to a serial console.

Customization options
=====================

