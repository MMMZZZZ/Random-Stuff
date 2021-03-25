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
// This is part of revision 2.2.0.295 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/flash.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Boot Loader Demo 2 (boot_demo2)</h1>
//!
//! An example to demonstrate the use of a flash-based boot loader.  At startup,
//! the application will configure the UART peripheral, wait for SW1 to be
//! pressed and blink LED D2.  When the SW1 is pressed, LED D2 is turned off,
//! and then the application branches to the boot loader to await the start of
//! an update.  The UART will always be configured at 115,200 baud and does not
//! require the use of auto-bauding.
//!
//! This application is intended for use with the boot_serial flash-based boot
//! loader included in the software release. Since the sector size is 16KB, the
//! link address is set to 0x4000.  If you are using USB or Ethernet boot 
//! loader, you may change this address to a 16KB boundary higher than the last
//! address occupied by the boot loader binary as long as you also rebuild the
//! boot loader itself after modifying its bl_config.h file to set 
//! APP_START_ADDRESS to the same value.
//!
//! The boot_demo1 application can be used along with this application to 
//! easily demonstrate that the boot loader is actually updating the on-chip
//! flash.
//!
//! Note that the TM4C129x-class device also support serial, Ethernet and USB
//! boot loaders in ROM.  To make use of this function, link your application
//! to run at address 0x0000 in flash and enter the bootloader using either the
//! ROM_UpdateSerial, ROM_UpdateEMAC or ROM_UpdateUSB functions (defined in
//! rom.h).  This mechanism is used in the utils/swupdate.c module when built
//! specifically targeting a suitable TM4C129x-class device.
//
//*****************************************************************************

//*****************************************************************************
//
// A global we use to update the system clock frequency
//
//*****************************************************************************
volatile uint32_t g_ui32SysClock;

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
// Passes control to the bootloader and initiates a remote software update.
//
// This function passes control to the bootloader and initiates an update of
// the main application firmware image via UART0 or USB depending
// upon the specific boot loader binary in use.
//
// \return Never returns.
//
//*****************************************************************************
void
JumpToBootLoader(void)
{
    //
    // We must make sure we turn off SysTick and its interrupt before entering 
    // the boot loader!
    //
    MAP_SysTickIntDisable(); 
    MAP_SysTickDisable(); 

    //
    // Disable all processor interrupts.  Instead of disabling them
    // one at a time, a direct write to NVIC is done to disable all
    // peripheral interrupts.
    //
    HWREG(NVIC_DIS0) = 0xffffffff;
    HWREG(NVIC_DIS1) = 0xffffffff;
    HWREG(NVIC_DIS2) = 0xffffffff;
    HWREG(NVIC_DIS3) = 0xffffffff;

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
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Set GPIO PA0 and PA1 as UART.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115200, n, 8, 1
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
// A simple application demonstrating use of the boot loader,
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
    MAP_FPULazyStackingEnable();

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
    // Initialize the peripherals that each of the boot loader flavours
    // supports.  Since this example is intended for use with any of the
    // boot loaders and we don't know which is actually in use, we cover all
    // bases and initialize for serial, Ethernet and USB use here.
    //
    SetupForUART();
    SetupForUSB();

    //
    // Enable Port J Pin 0 for exit to UART Boot loader when Pressed Low.
    // On the EK-TM4C1294 the weak pull up is enabled to detect button
    // press.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)));
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
    HWREG(GPIO_PORTJ_BASE + GPIO_O_PUR) = GPIO_PIN_0;

    //
    // Configure Port N pin 0 as output.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)));
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    //
    // If the Switch SW1 is not pressed then blink the LED D2 at 1 Hz rate
    // On switch SW press detection exit the blinking program and jump to
    // the flash boot loader.
    //
    while((MAP_GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) & GPIO_PIN_0) != 0x0)
    {
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x0);
        MAP_SysCtlDelay(g_ui32SysClock / 6);
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
        MAP_SysCtlDelay(g_ui32SysClock / 6);
    }

    //
    // Before passing control make sure that the LED is turned OFF.
    //
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x0);
    
    //
    // Pass control to whichever flavour of boot loader the board is configured
    // with.
    //
    JumpToBootLoader();

    //
    // The previous function never returns but we need to stick in a return
    // code here to keep the compiler from generating a warning.
    //
    return(0);
}
