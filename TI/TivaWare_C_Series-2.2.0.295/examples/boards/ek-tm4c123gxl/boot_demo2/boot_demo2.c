//*****************************************************************************
//
// boot_demo2.c - Second boot loader example.
//
// Copyright (c) 2008-2020 Texas Instruments Incorporated.  All rights reserved.
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

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "drivers/buttons.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Boot Loader Demo 2 (boot_demo2)</h1>
//!
//! An example to demonstrate the use of a flash-based boot loader.  At
//! startup, the application will configure the UART and USB peripherals,
//! and then branch to the boot loader to await the start of an
//! update.  If using the serial boot loader (boot_serial), the UART will
//! always be configured at 115,200 baud and does not require the use of
//! auto-bauding.
//!
//! This application is intended for use with any of the two flash-based boot
//! loader flavors (boot_serial or boot_usb) included in the software
//! release.  To accommodate the largest of these (boot_usb), the link address
//! is set to 0x2800.  If you are using serial, you may change this
//! address to a 1KB boundary higher than the last address occupied
//! by the boot loader binary as long as you also rebuild the boot
//! loader itself after modifying its bl_config.h file to set APP_START_ADDRESS
//! to the same value.
//!
//! The boot_demo1 application can be used along with this application to 
//! easily demonstrate that the boot loader is actually updating the on-chip
//! flash.
//!
//! Note that the TM4C123G and other Blizzard-class devices also the
//! support serial and USB boot loaders in ROM.  To make use of this
//! function, link your application to run at address 0x0000 in flash and enter
//! the bootloader using the ROM_UpdateSerial and ROM_UpdateUSB functions 
//! (defined in rom.h).  This mechanism is used in the utils/swupdate.c 
//! module when built specifically targeting a suitable Blizzard-class device.
//
//*****************************************************************************

//*****************************************************************************
//
// Pin Definitions for EK-TM4C123GXL LaunchPad RGB LED.
//
//*****************************************************************************
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

//*****************************************************************************
//
// A global we use to keep track of when the user presses the "Update now"
// button.
//
//*****************************************************************************
volatile bool g_bFirmwareUpdate = false; 

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
// Passes control to the boot loader and initiates a remote software update.
//
// This function passes control to the boot loader and initiates an update of
// the main application firmware image via UART0, Ethernet or USB depending
// upon the specific boot loader binary in use.
//
//*****************************************************************************
void
JumpToBootLoader(void)
{
    //
    // We must make sure we turn off SysTick and its interrupt before entering 
    // the boot loader!
    //
    ROM_SysTickIntDisable(); 
    ROM_SysTickDisable(); 

    //
    // Disable all processor interrupts.  Instead of disabling them
    // one at a time, a direct write to NVIC is done to disable all
    // peripheral interrupts.
    //
    HWREG(NVIC_DIS0) = 0xffffffff;
    HWREG(NVIC_DIS1) = 0xffffffff;

    //
    // Return control to the boot loader.  This is a call to the SVC
    // handler in the boot loader.
    //
    (*((void (*)(void))(*(uint32_t *)0x2c)))(); 
}

//*****************************************************************************
//
// Initialize UART0 and set the appropriate communication parameters.
//
//*****************************************************************************
void
SetupForUART(void)
{
    //
    // We need to make sure that UART0 and its associated GPIO port are
    // enabled before we pass control to the boot loader.  The serial boot
    // loader does not enable or configure these peripherals for us if we
    // enter it via its SVC vector.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Set GPIO A0 and A1 as UART.
    //
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115200, n, 8, 1.
    //
    ROM_UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_WLEN_8));

    //
    // Enable the UART operation.
    //
    ROM_UARTEnable(UART0_BASE);
}

//*****************************************************************************
//
// Enable the USB controller
//
//*****************************************************************************
void
SetupForUSB(void)
{
    //
    // The USB boot loader takes care of all required USB initialization so,
    // if the application itself doesn't need to use the USB controller, we
    // don't actually need to enable it here.  The only requirement imposed by
    // the USB boot loader is that the system clock is running from the PLL
    // when the boot loader is entered.
    //
}

//*****************************************************************************
//
// A simple application demonstrating use of the boot loader.
//
//*****************************************************************************
int
main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the system clock to run at 50MHz from the PLL.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Initialize the peripherals that each of the boot loader flavors
    // supports.  Since this example is intended for use with any of the
    // boot loaders and we don't know which is actually in use, we cover all
    // bases and initialize for serial and USB use here.
    //
    SetupForUART();
    SetupForUSB();

    //
    // Initialize the buttons driver.
    //
    ButtonsInit(); 

    //
    // Enable and wait for the port to be ready for access.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    //
    // Configure the GPIO port for the LED operation.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED | BLUE_LED);

    //
    // If the Switch SW1 is not pressed then blink the BLUE LED.
    // Press and hold the SW1 for the debouncer to recognize the pin pressed.
    // On switch SW1 press detection, exit the blinking program and jump to
    // the flash boot loader.
    // 
    while ((ButtonsPoll(0, 0) & LEFT_BUTTON) == 0)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0x0);
        SysCtlDelay(ROM_SysCtlClockGet() / 100);
        GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, BLUE_LED);
        SysCtlDelay(ROM_SysCtlClockGet() / 100);
    }

    //
    // Before passing control make sure that the BLUE LED is turned OFF.
    // But turn on the RED LED.
    //
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED, RED_LED);

    //
    // Transfer control to the boot loader.
    //
    JumpToBootLoader();

    //
    // The previous function never returns but we need to stick in a return
    // code here to keep the compiler from generating a warning.
    //
    return(0);
}
