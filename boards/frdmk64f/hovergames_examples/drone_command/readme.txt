Overview
========
The drone command example demonstrates a concept of controlling the PX4 FMU using MAVlink commands via UART. This example shows a simple program that sends manual commands to the flight controller and demonstrates the external board can steer the drone.

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
- Personal Computer
- JST connector cables for linking the FRDM-K64F board to the RDDRONE-FMUK66 FMU



Board settings
==============
UART3 on board: 
Used to communicate with the RDDRONE-FMUK66 FMU 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FRDM-K64F(UART3)     CONNECTS TO         RDDRONE-FMUK66(UART)
Pin Name   Board Location     Pin Name  Board Location
TX         J1 pin 4            RX       Telemetry RX
RX         J1 pin 2            TX       Telemetry TX
GND		   J3 pin 12 		   GND	    Telemetry GND
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
5.  Start QGroundControl on your groundstation and wait for connection to drone
6. 	Arm drone and set flight mode to Acro (make sure camera is not tracking anything when arming drone)

Running the demo
================
When the demo runs successfully, the drone propeller should start up in a idle mode. Then the drone should cycle from idel, to left rotation to right rotation and back to idel. On the debug console the current state is being printed.

Customization options
=====================

