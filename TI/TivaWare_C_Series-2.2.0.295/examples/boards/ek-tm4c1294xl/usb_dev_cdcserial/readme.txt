USB CDC Serial Device example

This example application turns the EK-TM4C1294XL LaunchPad into a single
port USB CDC device when connected to a USB host system. The application
supports the USB Communication Device Class, Abstract Control Model to
by interacting with the USB host via a virtual COM port. For this example,
the evaluation kit will enumerate as a CDC device with one virtual
serial port that redirects UART0 traffic to and from the USB host system.

The virtual serial port will echo data to the physical UART0 port on the
device which is connected to the virtual serial port on the ICDI device
on this board. The physical UART0 will also echo onto the virtual serial
device provided by the TM4C controller.

Assuming you installed TivaWare in the default directory, a driver
information (INF) file for use with Windows XP, Windows Vista, Windows 7,
and Windows 10 can be found in C:/TivaWare_C_Series-x.x/windows_drivers.
For Windows 2000, the required INF file is in
C:/TivaWare_C_Series-x.x/windows_drivers/win2K.

-------------------------------------------------------------------------------

Copyright (c) 2019-2020 Texas Instruments Incorporated.  All rights reserved.
Software License Agreement

Texas Instruments (TI) is supplying this software for use solely and
exclusively on TI's microcontroller products. The software is owned by
TI and/or its suppliers, and is protected under applicable copyright
laws. You may not combine this software with "viral" open-source
software in order to form a larger program.

THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, FOR ANY REASON WHATSOEVER.

This is part of revision 2.2.0.295 of the EK-TM4C1294XL Firmware Package.
