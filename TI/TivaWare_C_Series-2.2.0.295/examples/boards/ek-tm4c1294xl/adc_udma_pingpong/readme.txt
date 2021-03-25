ADC with uDMA Demo

This example demonstrates how to use the ADC peripheral with both the uDMA
and Timer peripherals to optimize the ADC sampling process.  The uDMA is
configured for Ping-Pong mode and used to transfer ADC measurement results
into a buffer in the background to minimize CPU usage and then indicate
when the buffer is ready for processing by the application.  The Timer is
used to trigger the ADC measurements at a set sampling frequency of 16 kHz.

This example uses the following peripherals and I/O signals.  You must
review these and change as needed for your own board:
- ADC0 peripheral
- GPIO Port E peripheral (for AIN0 pin)
- AIN0 - PE3

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
