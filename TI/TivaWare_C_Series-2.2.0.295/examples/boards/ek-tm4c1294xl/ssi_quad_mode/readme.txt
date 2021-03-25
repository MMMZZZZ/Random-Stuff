Quad-SSI Master

This example shows how to configure SSI0 as a Quad-SSI Master and SSI1 as a
Quad-SSI slave.  The master device will send four characters to the slave
device using the advanced Quad mode.  In Quad-SSI mode, four bits are sent
on each SSI Clock pulse.  Once the Quad-SSI slave receives the four
characters in its receive FIFO it will generate an interrupt.

This example uses the following peripherals and I/O signals.  You must
review these and change as needed for your own board:
- SSI0 peripheral
- GPIO Port A peripheral (for SSI0 pins)
- SSI0Clk    - PA2
- SSI0Fss    - PA3
- SSI0XDAT0  - PA4
- SSI0XDAT1  - PA5
- SSI0XDAT2  - PA6
- SSI0XDAT3  - PA7

- SSI1 peripheral
- GPIO Port B, D, E peripheral (for SSI1 pins)
- SSI1Clk    - PB5
- SSI1Fss    - PB4
- SSI1XDAT0  - PE4
- SSI1XDAT1  - PE5
- SSI1XDAT2  - PD4
- SSI1XDAT3  - PD5

This example requires board level connection between SSI0 and SSI1.

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
