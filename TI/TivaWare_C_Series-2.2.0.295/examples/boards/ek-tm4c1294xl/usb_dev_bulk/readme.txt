USB Generic Bulk Device

This example provides a generic USB device offering simple bulk data
transfer to and from the host.  The device uses a vendor-specific class ID
and supports a single bulk IN endpoint and a single bulk OUT endpoint.
Data received from the host is assumed to be ASCII text and it is
echoed back with the case of all alphabetic characters swapped.

Assuming you installed TivaWare in the default directory, a driver
information (INF) file for use with Windows XP, Windows Vista, Windows 7,
and Windows 10 can be found in C:/TivaWare_C_Series-x.x/windows_drivers.
For Windows 2000, the required INF file is in
C:/TivaWare_C_Series-x.x/windows_drivers/win2K.

A sample Windows command-line application, usb_bulk_example, illustrating
how to connect to and communicate with the bulk device is also provided.
The application binary is installed as part of the ``TivaWare for C Series
PC Companion Utilities'' package (SW-TM4C-USB-WIN) on the installation CD
or via download from http://www.ti.com/tivaware .  Project files are
included to allow the examples to be built using
Microsoft Visual Studio 2008.  Source code for this application can be
found in directory ti/TivaWare_C_Series-x.x/tools/usb_bulk_example.

-------------------------------------------------------------------------------

Copyright (c) 2013-2020 Texas Instruments Incorporated.  All rights reserved.
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
