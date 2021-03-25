//*****************************************************************************
//
// hibernate.c - Simple hibernate example with RTC.
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
// This is part of revision 2.2.0.295 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "time.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_hibernate.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/hibernate.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Hiberate Demo (hibernate)</h1>
//!
//! This example demonstrates the use of the Hibernation module.  The module
//! will be configured for Real-Time Counter (RTC) mode using the 32.768kHz
//! crystal that is installed on the EK-TM4C123GXL LaunchPad.  With RTC mode, a
//! match time will be loaded to trigger an interrupt that will wake the device
//! from hibernation mode.
//!
//! This example also uses a UART configured for 115200 baud, 8-N-1 mode to
//! send the current RTC count on each interrupt.
//
//*****************************************************************************

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
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// This function is the interrupt handler for the Hibernation Module.  When
// entered, it checks which interrupt occurred, clears all pending Hibernation
// interrupts, and then toggles an LED based on whether the device was woken
// by a button press or the RTC timer reaching the count match.
//
//*****************************************************************************
void
HibernateISRHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrupt status and clear any pending interrupts.
    //
    ui32Status = MAP_HibernateIntStatus(true);
    MAP_HibernateIntClear(ui32Status);

    //
    // Process the RTC match 0 interrupt.
    //
    if(ui32Status & HIBERNATE_INT_RTC_MATCH_0)
    {
        uint32_t ui32RTCMatch;

        //
        // Reads from the Hibernation RTC Counter (HIBRTCC) and Hibernation RTC
        // Sub Seconds (HIBRTCSS) registers may not be correct per Errata Item
        // HIB#02.
        //
        // The following code block is the workaround to get the correct
        // HIBRTCC value.  To get the value of both HIBRTCC and HIBRTCSS,
        // refer to the errata document for a variant of the workaround.
        //

        //
        // Disable Interrupts.
        //
        MAP_IntMasterDisable();

        //
        // To get the individual value for the HIBRTCC register, use the
        // following method.
        //
        do
        {
            ui32RTCMatch = HWREG(HIB_RTCC);
        } while (ui32RTCMatch != HWREG(HIB_RTCC));

        //
        // Re-enable interrupts.
        //
        MAP_IntMasterEnable();
	
        UARTprintf("Waking from Hiberation, current RTC count %d seconds.\n",
                   ui32RTCMatch);

        //
        // Set next interrupt for 5 seconds in future.
        //
        MAP_HibernateRTCMatchSet(0, ui32RTCMatch + 5);

        //
        // Toggle the current state of the Blue LED.
        //
        if(MAP_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
        {
            MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
        }
        else
        {
            MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        }
        //
        // Request Hibernation.
        //
        MAP_HibernateRequest();
    }
}

//*****************************************************************************
//
// This example demonstrates how to use the RTC mode of the Hibernation module.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32Status;

    //
    // Set the clocking to run directly from the external crystal/oscillator.
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Enable the hibernate module.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);

    //
    // Read and clear any status bits that might have been set since
    // last clearing them.
    //
    if(MAP_HibernateIsActive())
    {
        //
        // Read the status bits to see what caused the wake.
        //
        ui32Status = MAP_HibernateIntStatus(0);
        MAP_HibernateIntClear(ui32Status);
    }

    //
    // Set up the serial console to use for displaying messages.
    //
    ConfigureUART();
    UARTprintf("Hibernation with RTC Wake Example.\n");

    //
    // Enable the GPIO peripheral for the on-board LED.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Configure the GPIOs for the RGB LED.
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 |
                                               GPIO_PIN_1);

    //
    // Enable the Hibernation module for operation.
    //
    MAP_HibernateEnableExpClk(0);

    //
    // Wait for 32.768kHz clock to stabilize.
    //
    while(!(HWREG(HIB_RIS) & HIB_RIS_WC));

    //
    // Configure the drive strength for the crystal.
    //
    MAP_HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE);

    //
    // Enable the Hibernate module RTC mode.
    //
    MAP_HibernateRTCEnable();

    //
    // Load initial RTC value.
    //
    MAP_HibernateRTCSet(0);

    //
    // Set initial match value to trigger RTC after 5 seconds.
    //
    MAP_HibernateRTCMatchSet(0, 5);

    //
    // Enable RTC match interrupt.
    //
    MAP_HibernateIntEnable(HIBERNATE_INT_RTC_MATCH_0);

    //
    // Re-intializes the Trim which gets modified due to a known issue of
    // Hibernate register initialization from HIB#01 errata.
    //
    MAP_HibernateRTCTrimSet(0x7FFF);

    //
    // Configure MCU interrupts.
    //
    MAP_IntEnable(INT_HIBERNATE_TM4C123);

    //
    // Enable MCU interrupts.
    //
    MAP_IntMasterEnable();

    //
    // Turn on the Blue LED.
    //
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

    //
    // Enter Hibernation Mode.
    //
    UARTprintf("Entering Hiberation, waking in 5 seconds.\n");
    MAP_HibernateRequest();

    while(1)
    {
        //
        // Do nothing, Hibernate interrupt routine will handle the rest.
        //
    }
}
