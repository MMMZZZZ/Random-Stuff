PWM Reload Interrupt Demo

This example shows how to configure PWM2 for a load interrupt.  The PWM
interrupt will trigger every time the PWM2 counter gets reloaded.  In the
interrupt, 0.1% will be added to the current duty cycle.  This will
continue until a duty cycle of 75% is received, then the duty cycle will
get reset to 0.1%.

This example uses the following peripherals and I/O signals.
- GPIO Port F peripheral (for PWM2 pin)
- PWM2 - PF2

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
