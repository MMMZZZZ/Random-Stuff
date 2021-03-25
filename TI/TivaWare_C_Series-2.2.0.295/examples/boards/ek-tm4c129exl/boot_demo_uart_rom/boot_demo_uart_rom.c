//*****************************************************************************
//
// boot_demo_uart_rom.c - Example demonstrating the use of the ROM UART boot 
//                        loader.
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
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "drivers/buttons.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>ROM UART Boot Loader Demo (boot_demo_uart_rom)</h1>
//!
//! An example to demonstrate the use of a ROM-based boot loader.  At
//! startup, the application will configure the UART peripheral,
//! and then branch to the ROM boot loader to await the start of an
//! update.  The UART will always be configured at 115,200 baud and does
//! not require the use of auto-bauding.
//!
//! Note: The newly loaded application should start from APP_BASE 0x00000000
//! and will override this boot loader example. This is the intended use of
//! this example. For a persistent Flash-based boot loader, see the boot_serial
//! example.
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
// Initialize UART0 and set the appropriate communication parameters.
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // We need to make sure that UART0 and its associated GPIO port are
    // enabled before we pass control to the boot loader.  The serial boot
    // loader does not enable or configure these peripherals for us if we
    // enter it via its SVC vector.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Set GPIO A0 and A1 as UART.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115200, 8-N-1.
    //
    MAP_UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
                            (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_WLEN_8));

    //
    // Enable the UART operation.
    //
    MAP_UARTEnable(UART0_BASE);
}

//*****************************************************************************
//
// Passes control to the boot loader and initiates a remote software update.
//
// This function passes control to the boot loader and initiates an update of
// the main application firmware image via UART0
//
// \return Never returns.
//
//*****************************************************************************
void
JumpToBootLoader(void)
{
    //
    // Disable all processor interrupts.  Instead of disabling them
    // one at a time, a direct write to NVIC is done to disable all
    // peripheral interrupts.
    //
    HWREG(NVIC_DIS0) = 0xffffffff;
    HWREG(NVIC_DIS1) = 0xffffffff;

    //
    // Call the ROM UART boot loader.
    //
    ROM_UpdateUART();
}

//*****************************************************************************
//
// A simple application demonstrating the use of the ROM UART boot loader.
//
//*****************************************************************************
int
main(void)
{
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
    // Initialize the peripherals for the serial UART boot loader.
    //
    ConfigureUART();

    //
    // Initialize the buttons driver.
    //
    ButtonsInit();

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    
    //
    // Enable the GPIO pin for the LED (PN0) as an output.
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    
    //
    // If the Switch SW1 is not pressed then blink the LED.  Press
    // and hold the SW1 for the debouncer to recognize the pin pressed.
    // On switch SW1 press detection, exit the blinking program and jump
    // to the ROM boot loader.
    //
    while ((ButtonsPoll(0, 0) & LEFT_BUTTON) == 0)
    {
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
        MAP_SysCtlDelay(g_ui32SysClock / 50);
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x00);
        MAP_SysCtlDelay(g_ui32SysClock / 50);
    }

    //
    // Before passing control make sure that the LED is turned OFF.
    //
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x00);

    //
    // Pass control to whichever flavor of boot loader the board is configured
    // with.
    //
    JumpToBootLoader();

    //
    // The boot loader should not return.  In the off chance that it does,
    // enter a dead loop.
    //
    while(1)
    {
    }
}
