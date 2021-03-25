Timer Edge Capture Demo

This example demonstrates how to use two timers to determine the duration
of the high period of an input signal.  The same logic can be applied to
also measure the low period.  The example uses Timer 0 in Split Mode and
the 8-bit prescaler to create two 24-bit wide timers.  There are some
limitations with this method to be aware of.  The limitation on the lower
end is if a signal is too fast to sample with the timers due to the time to
execute code.  The limitation on the upper end is if the signal period is
longer than 139.8 milliseconds at which point the 24-bit timer would
overflow.  That issue can be worked around by using two 32-bit timers which
would allow measuring a signal up to 35.79 seconds long.

To test this example either input a square wave to PL4 and PL5 with a
signal generator or leverage a second LaunchPad running a TivaWare PWM
project such as pwm_interrupt.

This example uses the following peripherals and I/O signals.
- GPIO Port L peripheral (for TIMER0 pins)
- T0CCP0 - PL4
- T0CCP1 - PL5

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
