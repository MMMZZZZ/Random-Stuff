Humidity Measurement with the SHT21

This example demonstrates the usage of the I2C interface to obtain the
temperature and relative humidity of the environment using the Sensirion
SHT21 sensor.

The I2C7 on the EK-TM4C1294XL launchPad is used to interface with the
SHT21 sensor.  The SHT21 sensor is on the BOOSTXL_SENSHUB boosterPack
expansion board that can be directly plugged into the Booster pack 1
connector of the EK-TM4C1294XL launchPad board.  Please make sure proper
pull-up resistors are on the I2C SCL and SDA buses.

This example uses the following peripherals and I/O signals.  You must
review these and change as needed for your own board:
- I2C7 peripheral
- GPIO Port D peripheral
- I2C7_SCL - PD0
- I2C7_SDA - PD1

UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
is used to display messages from this application.

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
