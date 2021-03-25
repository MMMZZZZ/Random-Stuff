//*****************************************************************************
//
// pwm_interrupt.c - Example demonstrating the PWM interrupt.
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

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>PWM Reload Interrupt Demo (pwm_interrupt)</h1>
//!
//! This example shows how to configure PWM2 for a load interrupt.  The PWM
//! interrupt will trigger every time the PWM2 counter gets reloaded.  In the
//! interrupt, 0.1% will be added to the current duty cycle.  This will
//! continue until a duty cycle of 75% is received, then the duty cycle will
//! get reset to 0.1%.
//!
//! This example uses the following peripherals and I/O signals.
//! - GPIO Port F peripheral (for PWM2 pin)
//! - PWM2 - PF2
//!
//! UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
//! is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// The variable g_ui32SysClock contains the system clock frequency in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The variable g_ui32PWMIncrement contains the value needed to increment the
// PWM duty cycle by 0.1% cycles based on the System Clock speed.
//
//*****************************************************************************
uint32_t g_ui32PWMIncrement;

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
// The interrupt handler for the for PWM2 interrupts.
//
//*****************************************************************************
void
PWM0Gen1IntHandler(void)
{
    //
    // Clear the PWM2 LOAD interrupt flag.  This flag gets set when the PWM
    // counter gets reloaded.
    //
    MAP_PWMGenIntClear(PWM0_BASE, PWM_GEN_1, PWM_INT_CNT_LOAD);

    //
    // If the duty cycle is less or equal to 75% then add 0.1% to the duty
    // cycle.  Else, reset the duty cycle to 0.1% cycles.  Note that 64 is
    // 0.1% of the period (64000 cycles).
    //
    if((MAP_PWMPulseWidthGet(PWM0_BASE, PWM_OUT_2) + g_ui32PWMIncrement) <=
       ((MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) * 3) / 4))
    {
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                             MAP_PWMPulseWidthGet(PWM0_BASE, PWM_GEN_1) +
                             g_ui32PWMIncrement);
    }
    else
    {
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, g_ui32PWMIncrement);
    }
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
// Configure PWM for a load interrupt.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32PWMClockRate;

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
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Display the setup on the console.
    //
    UARTprintf("PWM ->\n");
    UARTprintf("  Module: PWM2\n");
    UARTprintf("  Pin: PF2\n");
    UARTprintf("  Duty Cycle: Variable -> ");
    UARTprintf("0.1%% to 75%% in 0.1%% increments.\n");
    UARTprintf("  Features: ");
    UARTprintf("Variable pulse-width done using a reload interrupt.\n\n");

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //
    // Enable the GPIO pin for the LED (PN0) as an output.
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    //
    // The PWM peripheral must be enabled for use.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //
    // Enable the GPIO port that is used for the PWM output.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Configure the PWM function for this pin.
    //
    MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Set the PWM clock to be SysClk / 8.
    //
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);

    //
    // Use a local variable to store the PWM clock rate which will be
    // 120 MHz / 8 = 15 MHz. This variable will be used to set the
    // PWM generator period.
    //
    ui32PWMClockRate = g_ui32SysClock / 8;

    //
    // Configure PWM2 to count down without synchronization.
    //
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //
    // Set the PWM period to 250Hz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * PWMClk.  Where N is the
    // function parameter, f is the desired frequency, and PWMClk is the
    // PWM clock frequency based on the system clock.
    //
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, (ui32PWMClockRate / 250));

    //
    // For this example the PWM2 duty cycle will be variable.  The duty cycle
    // will start at 0.1% and will increase to 75%. After a duty cycle of 75%
    // is reached, it is reset back to 0.1%.  This dynamic adjustment of the
    // pulse width is done in the PWM2 load interrupt, which increases the
    // duty cycle by 0.1% every time the reload interrupt is received.
    //

    //
    // Set the PWM increment variable based on the System Clock. Since this
    // is a 250 Hz PWM, continue to use the equation N = (1 / f) * SysClk.
    // Then to set the initial period to 0.1% by dividing (N / 1000).
    // This variable will be used to increment PWM2 by 0.1% on each
    // interrupt.
    //
    g_ui32PWMIncrement = ((ui32PWMClockRate / 250) / 1000);

    //
    // Set the initial PWM2 Pulse Width with the calculated increment variable
    // to start at 0.1% duty cycle.
    //
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, g_ui32PWMIncrement);

    //
    // Enable processor interrupts.
    //
    MAP_IntMasterEnable();

    //
    // Allow PWM2 generated interrupts.  This configuration is done to
    // differentiate fault interrupts from other PWM2 related interrupts.
    //
    MAP_PWMIntEnable(PWM0_BASE, PWM_INT_GEN_1);

    //
    // Enable the PWM0 LOAD interrupt on PWM Gen 1.
    //
    MAP_PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_1, PWM_INT_CNT_LOAD);

    //
    // Enable the interrupt for PWM Gen 1 on the processor (NVIC).
    //
    MAP_IntEnable(INT_PWM0_1);

    //
    // Enable the PWM Out Bit 2 (PF2) output signal.
    //
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);

    //
    // Enable the PWM generator block.
    //
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    //
    // Loop forever blinking an LED while the PWM signals are generated and
    // PWM2 interrupts are occurring.
    //
    while(1)
    {
        //
        // Turn on the LED.
        //
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

        //
        // This function provides a means of generating a constant length
        // delay.  The function delay (in cycles) = 3 * parameter.  Delay
        // 0.5 seconds arbitrarily.
        //
        MAP_SysCtlDelay((g_ui32SysClock / 2) / 3);

        //
        // Turn off the LED.
        //
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x00);

        //
        // This function provides a means of generating a constant length
        // delay.  The function delay (in cycles) = 3 * parameter.  Delay
        // 0.5 seconds arbitrarily.
        //
        MAP_SysCtlDelay((g_ui32SysClock / 2) / 3);
    }
}
