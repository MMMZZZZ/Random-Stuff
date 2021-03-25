//*****************************************************************************
//
// timer_edge_capture.c - Example for how to measure a pulsed signal with the
//                        timer peripheral.
//
// Copyright (c) 2019-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.2.0.295 of the EK-TM4C129EXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Timer Edge Capture Demo (timer_edge_capture)</h1>
//!
//! This example demonstrates how to use two timers to determine the duration
//! of the high period of an input signal.  The same logic can be applied to
//! also measure the low period.  The example uses Timer 0 in Split Mode and
//! the 8-bit prescaler to create two 24-bit wide timers.  There are some
//! limitations with this method to be aware of.  The limitation on the lower
//! end is if a signal is too fast to sample with the timers due to the time to
//! execute code.  The limitation on the upper end is if the signal period is
//! longer than 139.8 milliseconds at which point the 24-bit timer would
//! overflow.  That issue can be worked around by using two 32-bit timers which
//! would allow measuring a signal up to 35.79 seconds long.
//!
//! To test this example either input a square wave to PL4 and PL5 with a
//! signal generator or leverage a second LaunchPad running a TivaWare PWM
//! project such as pwm_interrupt.
//!
//! This example uses the following peripherals and I/O signals.
//! - GPIO Port L peripheral (for TIMER0 pins)
//! - T0CCP0 - PL4
//! - T0CCP1 - PL5
//!
//! UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
//! is used to display messages from this application.
//
//*****************************************************************************

//****************************************************************************
//
// The variable g_ui32SysClock contains the system clock frequency in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;

//****************************************************************************
//
// Variables to store Timer values from the ISR, must be volatile.
//
//****************************************************************************
volatile uint32_t g_ui32HighStartCount;
volatile uint32_t g_ui32HighEndCount;

//****************************************************************************
//
// Flag to indicate if an interrupt occurred, must be volatile.
//
//****************************************************************************
volatile bool g_bIntFlag = false;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The interrupt handler for the Timer 0A interrupt.
//
//*****************************************************************************
void
Timer0AIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT);

    //
    // Store the end time.  In Edge Time Mode, the prescaler is used for the
    // the most significant bits.  Therefore, it must be shifted by 16 before
    // being added onto the final value.
    //
    g_ui32HighStartCount = (MAP_TimerValueGet(TIMER0_BASE, TIMER_A)) +
                           (MAP_TimerPrescaleGet(TIMER0_BASE, TIMER_A) << 16);
}

//*****************************************************************************
//
// The interrupt handler for the Timer 0B interrupt.
//
//*****************************************************************************
void
Timer0BIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER0_BASE, TIMER_CAPB_EVENT);

    //
    // Store the end time.  In Edge Time Mode, the prescaler is used for the
    // the most significant bits.  Therefore, it must be shifted by 16 before
    // being added onto the final value.
    //
    g_ui32HighEndCount = (MAP_TimerValueGet(TIMER0_BASE, TIMER_B)) +
                         (MAP_TimerPrescaleGet(TIMER0_BASE, TIMER_B) << 16);

    //
    // Set the global flag to indicate that new timer values have been stored.
    // This is only done on Timer B as the measurement is tracking the high
    // period of the signal, so Timer A handles the rising edge at the start
    // of the measurement and Timer B handles the falling edge at the end of
    // the measurement.  Therefore, once Timer B is triggered, the application
    // can now calculate the high period of the signal.
    //
    g_bIntFlag = true;
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

//*****************************************************************************
//
// Configure the system clock, UART, and setup Timer0 to be a Split Timer
// which uses the 8-bit prescaler to create two 24-bit timers to measure signal
// high periods based on rising and falling edge events on the Timer pins PL4
// and PL5.  The measurement results will be output on UART0 to the Virtual
// Serial Port.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32HighPeriod;

    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);

    //
    // Enable the peripherals used by this example.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

    //
    // Configure the GPIO to be CCP pins for the Timer peripheral.
    //
    MAP_GPIOPinConfigure(GPIO_PL4_T0CCP0);
    MAP_GPIOPinConfigure(GPIO_PL5_T0CCP1);

    //
    // Configure the GPIO for the Timer peripheral.
    //
    MAP_GPIOPinTypeTimer(GPIO_PORTL_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //
    // Initialize the UART and write initial status.
    //
    ConfigureUART();
    UARTprintf("Edge Capture Example for measuring the high period of an"
               "input signal.\n");
    UARTprintf("Input a square wave onto pins PL4 and PL5 of the "
               "EK-TM4C1294XL.\n");

    //
    // Initialize Timers A and B to both run as periodic up-count edge capture
    // timers.  This will split the 32-bit timer into two 16-bit timers.
    //
    MAP_TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR |
                                     TIMER_CFG_A_PERIODIC |
				     TIMER_CFG_A_CAP_TIME_UP |
                                     TIMER_CFG_B_PERIODIC |
				     TIMER_CFG_B_CAP_TIME_UP));

    //
    // To use the timer in Edge Time mode, it must be preloaded with initial
    // values.  If the prescaler is used, then it must be preloaded as well.
    // Since we want to use all 24-bits for both timers it will be loaded with
    // the maximum of 0xFFFF for the 16-bit wide split timers, and 0xFF to add
    // the additional 8-bits to the split timers with the prescaler.
    //
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_BOTH, 0xFFFF);
    MAP_TimerPrescaleSet(TIMER0_BASE, TIMER_BOTH, 0xFF);

    //
    // Configure Timer A to trigger on a Positive Edge and configure Timer B
    // to trigger on a Negative Edge.
    //
    MAP_TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    MAP_TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we enable it.
    //
    MAP_TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);

    //
    // Enable the Timer A and B interrupts for Capture Events.
    //
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);

    //
    // Enable the interrupts for Timer A and Timer B on the processor (NVIC).
    //
    MAP_IntEnable(INT_TIMER0A);
    MAP_IntEnable(INT_TIMER0B);

    //
    // Enable processor interrupts.
    //
    MAP_IntMasterEnable();

    //
    // Enable both Timer A and Timer B to begin the application.
    //
    MAP_TimerEnable(TIMER0_BASE, TIMER_BOTH);

    while(1)
    {
        //
        // Wait until the Timer B interrupt has occurred.
        //
        if (g_bIntFlag)
        {
            //
            // Simple check to avoid overflow cases.  The End Count is the
            // second measurement taken and therefore should never be smaller
            // than the Start Count unless the timer has overflowed.  If that
            // occurs, then add 2^24-1 to the End Count before subtraction.
            //
            if (g_ui32HighEndCount > g_ui32HighStartCount)
            {
                ui32HighPeriod = g_ui32HighEndCount - g_ui32HighStartCount;
            }
            else
            {
                ui32HighPeriod =
                        (g_ui32HighEndCount + 16777215) - g_ui32HighStartCount;
            }

            //
            // Now that the results have been processed, toggle the interrupt
            // flag back to false to wait for the next measurement pair to be
            // done.  This is done before the UART output in case the timers
            // are triggered while the application is still running.
            //
            g_bIntFlag = false;

            //
            // The System Clock speed is needed to calculate the period length
            // of the input signal.  Dividing by the system clock gives the
            // period in seconds.  Multiplying by 10^6 then shifts the result
            // to microseconds to simplify the UART output of the result.
            //
            ui32HighPeriod /= (g_ui32SysClock / 1000000);

            //
            // Output the result of the measured input signal high period.
            //
            UARTprintf("Input Signal Period = %d microseconds\n",
                       ui32HighPeriod);
        }
    }
}
