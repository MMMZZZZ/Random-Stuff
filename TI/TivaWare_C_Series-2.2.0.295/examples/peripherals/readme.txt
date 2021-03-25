The examples in this directory demonstrate how to program various peripherals
available on Tiva microcontrollers.  These examples are meant to be used
as guides to follow for your own application.  They have only been tested for
the EK-TM4C123GXL and EK-TM4C1294XL.  They may not directly run on all other
Tiva development kits (DK) without modification.  These are not ready-to-run
projects but rather code that you add to your own project.  The easiest way
to test out a single example is to use Project0 for your evaluation kit as
a base project to copy and paste the code into.

For examples that are meant to run directly on kit boards immediately,
please see the examples/boards subdirectory.

Note for TivaWare 2.2.0 and newer:

The PWM examples have been moved into the boards folder for each specific
LaunchPad.  This was done due to the high amount of variance on pin
configurations which added unneeded complexity for each example.

The examples are:
 - pwm_dead_band
 - pwm_interrupt
 - pwm_invert

These examples can be found in the following TivaWare folders:
 - examples\boards\ek-tm4c123gxl
 - examples\boards\ek-tm4c1294xl
 - examples\boards\ek-tm4c129exl

-------------------------------------------------------------------------------

Copyright (c) 2013-2020 Texas Instruments Incorporated.  All rights reserved.
Software License Agreement

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the  
  distribution.

  Neither the name of Texas Instruments Incorporated nor the names of
  its contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This is part of revision 2.2.0.295 of the Tiva Firmware Development Package.
