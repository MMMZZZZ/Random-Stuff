//*****************************************************************************
//
// boot_demo_emac_rom.c - Example demonstrating the use of the ROM Ethernet
//                        boot loader.
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
// This is part of revision 2.2.0.295 of the EK-TM4C129EXL Firmware Package.
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
#include "driverlib/flash.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/lwiplib.h"
#include "utils/swupdate.h"
#include "utils/ustdlib.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>ROM Boot Loader Ethernet Demo (boot_demo_emac_rom)</h1>
//!
//! An example to demonstrate the use of remote update signaling with the
//! ROM-based Ethernet boot loader.  This application configures the Ethernet
//! controller and acquires an IP address.  It then listens for a
//! ``magic packet'' telling it that a firmware upgrade request is being made
//! and, when this packet is received, transfers control to the ROM boot loader
//! to perform the upgrade.
//
//*****************************************************************************

//*****************************************************************************
//
// The number of SysTick ticks per second.
//
//*****************************************************************************
#define TICKS_PER_SECOND 100

//****************************************************************************
//
// The variable g_ui32SysClock contains the system clock frequency in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// A global flag used to indicate if a remote firmware update request has been
// received.
//
//*****************************************************************************
static volatile bool g_bFirmwareUpdate = false;

//*****************************************************************************
//
// Buffers used to hold the Ethernet MAC and IP addresses for the board.
//
//*****************************************************************************
#define SIZE_MAC_ADDR_BUFFER 32
#define SIZE_IP_ADDR_BUFFER 32
char g_pcMACAddr[SIZE_MAC_ADDR_BUFFER];
char g_pcIPAddr[SIZE_IP_ADDR_BUFFER];

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
// This is the handler for this SysTick interrupt.  We use this to provide the
// required timer call to the lwIP stack.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Call the lwIP timer.
    //
    lwIPTimer(1000 / TICKS_PER_SECOND);
}

//*****************************************************************************
//
// This function is called by the software update module whenever a remote
// host requests to update the firmware on this board.  We set a flag that
// will cause the main loop to exit and transfer control to the boot loader.
//
// IMPORTANT:
// Note that this callback is made in interrupt context and, since it is not
// permitted to transfer control to the boot loader from within an interrupt,
// we can't just call SoftwareUpdateBegin() here.
//
//*****************************************************************************
void
SoftwareUpdateRequestCallback(void)
{
    g_bFirmwareUpdate = true;
}

//*****************************************************************************
//
// Perform the initialization steps required to start up the Ethernet controller
// and lwIP stack.
//
//*****************************************************************************
void
SetupForEthernet(void)
{
    uint32_t ui32User0, ui32User1;
    uint8_t pui8MACAddr[6];

    //
    // Configure SysTick for a 100Hz interrupt.
    //
    MAP_SysTickPeriodSet(g_ui32SysClock / TICKS_PER_SECOND);
    MAP_SysTickEnable();
    MAP_SysTickIntEnable();

    //
    // Get the MAC address from the UART0 and UART1 registers in NV ram.
    //
    MAP_FlashUserGet(&ui32User0, &ui32User1);

    //
    // Convert the 24/24 split MAC address from NV ram into a MAC address
    // array.
    //
    pui8MACAddr[0] = ui32User0 & 0xff;
    pui8MACAddr[1] = (ui32User0 >> 8) & 0xff;
    pui8MACAddr[2] = (ui32User0 >> 16) & 0xff;
    pui8MACAddr[3] = ui32User1 & 0xff;
    pui8MACAddr[4] = (ui32User1 >> 8) & 0xff;
    pui8MACAddr[5] = (ui32User1 >> 16) & 0xff;

    //
    // Format this address into the string used by the relevant widget.
    //
    usnprintf(g_pcMACAddr, SIZE_MAC_ADDR_BUFFER,
              "MAC: %02X-%02X-%02X-%02X-%02X-%02X",
              pui8MACAddr[0], pui8MACAddr[1], pui8MACAddr[2], pui8MACAddr[3],
              pui8MACAddr[4], pui8MACAddr[5]);

    //
    // Remember that we don't have an IP address yet.
    //
    usnprintf(g_pcIPAddr, SIZE_IP_ADDR_BUFFER, "IP: Not assigned");

    //
    // Initialize the lwIP TCP/IP stack.
    //
    lwIPInit(g_ui32SysClock, pui8MACAddr, 0, 0, 0, IPADDR_USE_DHCP);

    //
    // Start the remote software update module.
    //
    SoftwareUpdateInit(SoftwareUpdateRequestCallback);
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
    // Initialize the peripherals for the Ethernet boot loader.
    //
    SetupForEthernet();

    //
    // Configure Port N pin 1 as output.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)));
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

    //
    // If the Switch SW1 is not pressed then blink the LED D1 at 1 Hz rate.
    // On switch SW1 press detection exit the blinking program and jump to
    // the flash boot loader.
    //
    while(!g_bFirmwareUpdate)
    {
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0x0);
        MAP_SysCtlDelay(g_ui32SysClock / 6);
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
        MAP_SysCtlDelay(g_ui32SysClock / 6);
    }

    //
    // Before passing control make sure that the LED is turned OFF.
    //
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0x0);
    
    //
    // Pass control to whichever flavor of boot loader the board is configured
    // with.
    //
    SoftwareUpdateBegin(g_ui32SysClock);

    //
    // The previous function never returns but we need to stick in a return
    // code here to keep the compiler from generating a warning.
    //
    return(0);
}
