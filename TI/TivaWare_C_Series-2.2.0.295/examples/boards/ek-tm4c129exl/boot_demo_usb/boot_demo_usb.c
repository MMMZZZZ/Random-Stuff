//*****************************************************************************
//
// boot_demo_usb.c - USB ROM Boot Loader leveraging an USB HID/DFU composite
//                   device example.
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
#include "driverlib/systick.h"
#include "driverlib/usb.h"
#include "drivers/buttons.h"
#include "drivers/pinout.h"
#include "utils/uartstdio.h"
#include "usblib/usblib.h"
#include "usblib/usbhid.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdhid.h"
#include "usblib/device/usbdhidmouse.h"
#include "usblib/device/usbddfu-rt.h"
#include "usblib/device/usbdcomp.h"
#include "usb_hiddfu_structs.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Boot Loader USB Demo (boot_demo_usb)</h1>
//!
//! This example application is used in conjunction with the USB boot loader in
//! ROM and turns the development board into a composite device supporting a
//! mouse via the Human Interface Device class and also publishing runtime
//! Device Firmware Upgrade (DFU) capability.  Pressing USR_SW1 will trigger
//! the USB mouse interface to move the cursor in a square pattern once.  This
//! is a basic example of how HID reports are sent to the USB host in order to
//! demonstrate controlling the mouse pointer on the host system.
//!
//! Since the device also publishes a DFU interface, host software such as the
//! dfuprog tool can determine that the device is capable of receiving software
//! updates over USB.  The runtime DFU protocol allows such tools to signal the
//! device to switch into DFU mode and prepare to receive a new software image.
//!
//! Runtime DFU functionality requires only that the device listen for a
//! particular request (DETACH) from the host and, when this is received,
//! transfer control to the USB boot loader via the normal means to re-enumerate
//! as a pure DFU device capable of uploading and downloading firmware images.
//!
//! Windows device drivers for both the runtime and DFU mode of operation can
//! be found in <tt>C:/TI/TivaWare_C_Series-x.x/windows_drivers</tt> assuming
//! you installed TivaWare in the default directory.
//!
//! To illustrate runtime DFU capability, use the <tt>dfuprog</tt> tool which
//! is part of the Tiva Windows USB Examples package (SW-USB-win-xxxx.msi)
//! Assuming this package is installed in the default location, the
//! <tt>dfuprog</tt> executable can be found in the
//! <tt>C:/Program Files/Texas Instruments/Tiva/usb_examples</tt> or 
//! <tt>C:/Program Files (x86)/Texas Instruments/Tiva/usb_examples</tt>
//! directory.
//!
//! With the device connected to your PC and the device driver installed, enter
//! the following command to enumerate DFU devices:
//!
//! <tt>dfuprog -e</tt>
//!
//! This will list all DFU-capable devices found and you should see that you
//! have one or two devices available which are in ``Runtime'' mode.
//!
//! *** IMPORTANT - PLEASE READ ***
//! If you see two devices, it is strongly recommended that you disconnect
//! ICDI debug port from the PC, and change the POWER_SELECT jumper (JP1) 
//! from 'ICDI' to 'OTG' in order to power the LaunchPad from the USB
//! OTG port. The reason for this is that the ICDI chip on the board is
//! a DFU-capable TM4C129x device, and if not careful, the firmware on the
//! ICDI chip could be accidently erased which can not restored easily.
//! As a result, debug capabilities would be lost!
//! *** IMPORTANT - PLEASE READ ***
//!
//! If IDCI debug port is disconnected from your PC, you should see only one
//! device from above command, and its index should be 0, and should be named
//! as ``Mouse with Device Firmware Upgrade''. 
//! If for any reason you need to keep the ICDI port connected, the above
//! command should show two devices. The second device is probably named as
//! ``In-Circuit Debug interface'', and we need to be careful not to update
//! the firmware on that device. So please take careful note of the index for
//! the device ``Mouse with Device Firmware Upgrade'', it could be 0 or 1, we
//! will need this index number for the following command. 
//! Entering the following command will switch this device into DFU mode and
//! leave it ready to receive a new firmware image:
//!
//! <tt>dfuprog -i index -m</tt>
//!
//! After entering this command, you should notice that the device disconnects
//! from the USB bus and reconnects again.  Running ``<tt>dfuprog -e</tt>'' a
//! second time will show that the device is now in DFU mode and ready to
//! receive downloads.  At this point, either LM Flash Programmer or dfuprog
//! may be used to send a new application binary to the device.
//
//*****************************************************************************

//*****************************************************************************
//
// Scalars for mouse X and Y delta movements.
//
//*****************************************************************************
#define DELTAX 10
#define DELTAY 10

//*****************************************************************************
//
// The system tick timer rate.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     50

//*****************************************************************************
//
// A flag used to indicate whether or not we are currently connected to the USB
// host.
//
//*****************************************************************************
volatile bool g_bConnected;

//*****************************************************************************
//
// Global system tick counter.
//
//*****************************************************************************
volatile uint32_t g_ui32SysTickCount;

//*****************************************************************************
//
// This enumeration holds the various states that the mouse can be in during
// normal operation.
//
//*****************************************************************************
volatile enum
{
    //
    // Unconfigured.
    //
    MOUSE_STATE_UNCONFIGURED,

    //
    // No keys to send and not waiting on data.
    //
    MOUSE_STATE_IDLE,

    //
    // Waiting on data to be sent out.
    //
    MOUSE_STATE_SENDING
}
g_eMouseState = MOUSE_STATE_UNCONFIGURED;

//*****************************************************************************
//
// Flag used to tell the main loop that it's time to pass control back to the
// boot loader for an update.
//
//*****************************************************************************
volatile bool g_bUpdateSignalled;

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
// This is the callback from the USB DFU runtime interface driver.
//
// \param pvCBData is ignored by this function.
// \param ui32Event is one of the valid events for a DFU device.
// \param ui32MsgParam is defined by the event that occurs.
// \param pvMsgData is a pointer to data that is defined by the event that
// occurs.
//
// This function will be called to inform the application when a change occurs
// during operation as a DFU device.  Currently, the only event passed to this
// callback is USBD_DFU_EVENT_DETACH which tells the recipient that they should
// pass control to the boot loader at the earliest, non-interrupt context
// point.
//
// \return This function will return 0.
//
//*****************************************************************************
uint32_t
DFUDetachCallback(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgData,
                  void *pvMsgData)
{
    if(ui32Event == USBD_DFU_EVENT_DETACH)
    {
        //
        // Set the flag that the main loop uses to determine when it is time
        // to transfer control back to the boot loader.  Note that we
        // absolutely DO NOT call USBDDFUUpdateBegin() here since we are
        // currently in interrupt context and this would cause critical issues
        // to occur and would not allow the boot loader to work.
        //
        g_bUpdateSignalled = true;
    }

    return(0);
}

//*****************************************************************************
//
// This is the callback from the USB composite device class driver.
//
// \param pvCBData is ignored by this function.
// \param ui32Event is one of the valid events for a mouse device.
// \param ui32MsgParam is defined by the event that occurs.
// \param pvMsgData is a pointer to data that is defined by the event that
// occurs.
//
// This function will be called to inform the application when a change occurs
// during operation as a HID class USB mouse device.
//
// \return This function will return 0.
//
//*****************************************************************************
uint32_t
MouseHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgData,
             void *pvMsgData)
{
    switch(ui32Event)
    {
        //
        // The USB host has connected to and configured the device.
        //
        case USB_EVENT_CONNECTED:
        {
            g_eMouseState = MOUSE_STATE_IDLE;
            g_bConnected = true;
            break;
        }
        //
        // The USB host has disconnected from the device.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bConnected = false;
            g_eMouseState = MOUSE_STATE_UNCONFIGURED;
            break;
        }
        //
        // A report was sent to the host. We are not free to send another.
        //
        case USB_EVENT_TX_COMPLETE:
        {
            g_eMouseState = MOUSE_STATE_IDLE;
            break;
        }
    }

    return(0);
}

//*****************************************************************************
//
// This is the interrupt handler for the SysTick interrupt.  It is called
// periodically and updates a global tick counter then sets a flag to tell the
// main loop to check the button state.
//
//*****************************************************************************
void
SysTickHandler(void)
{
    g_ui32SysTickCount++;
}

//*****************************************************************************
//
// Configure the UART and its pins. This must be called before UARTprintf().
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
// This is the main loop that runs the application.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32PLLRate;
    int32_t i32toggle=1;

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
    // Configure the device pins for USB.
    //
    PinoutSet(0,1);

    //
    // Initialize the buttons driver.
    //
    ButtonsInit();

    //
    // Initialize UART0 and write initial status.
    //
    ConfigureUART();
    UARTprintf("USB HID Mouse/DFU Composite Device Example.\n");

    //
    // Set the system tick to fire 100 times per second.
    //
    MAP_SysTickPeriodSet(g_ui32SysClock / SYSTICKS_PER_SECOND);
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();

    //
    // Tell the USB library the CPU clock and the PLL frequency.  This is a
    // new requirement for TM4C129 devices.
    //
    SysCtlVCOGet(SYSCTL_XTAL_25MHZ, &ui32PLLRate);
    USBDCDFeatureSet(0, USBLIB_FEATURE_CPUCLK, &g_ui32SysClock);
    USBDCDFeatureSet(0, USBLIB_FEATURE_USBPLL, &ui32PLLRate);

    //
    // Initialize each of the device instances that will form our composite
    // USB device.
    //
    USBDHIDMouseCompositeInit(0, &g_sMouseDevice,
                              &(g_sCompDevice.psDevices[0]));
    USBDDFUCompositeInit(0, &g_sDFUDevice, &(g_sCompDevice.psDevices[1]));

    //
    // Pass the USB library our device information, initialize the USB
    // controller and connect the device to the bus.
    //
    USBDCompositeInit(0, &g_sCompDevice, DESCRIPTOR_BUFFER_SIZE,
                      g_pui8DescriptorBuffer);

    //
    // Tell the user that the device is waiting for a USB host to be connected.
    //
    UARTprintf("Waiting for host...\n");

    //
    // Wait for USB configuration to complete.
    //
    while(!g_bConnected)
    {
    }

    //
    // Update the status.
    //
    UARTprintf("\nHost connected...\n");
    UARTprintf("Reminder: Look for 'Mouse with Device Firmware Upgrade'.\n");

    //
    // The DFU interface is waiting for the host to place the device in
    // DFU mode from Runtime mode via the DETACH request.
    //
    while(!g_bUpdateSignalled)
    {
        //
        // While waiting for the host to send the DETACH request, the
        // device will simulate a mouse movement in a square pattern  
        // if the USR_SW1 (LEFT_BUTTON) on the LaunchPad is pressed.
        //
        if ((ButtonsPoll(0, 0) & LEFT_BUTTON) == 1)
        {
            USBDHIDMouseStateChange((void *)&g_sMouseDevice,
                                    (char)DELTAX*i32toggle, (char)0, 0);
            SysCtlDelay(g_ui32SysClock / 10);

            USBDHIDMouseStateChange((void *)&g_sMouseDevice,
                                    (char)0, (char)DELTAY*i32toggle, 0);
            SysCtlDelay(g_ui32SysClock / 10);

            i32toggle *= -1;
        }
    }

    //
    // Tell the user that DFU mode has been triggered.
    //
    UARTprintf("\nSwitching to DFU mode.\n");

    //
    // Terminate the USB device and detach from the bus.
    //
    USBDCDTerm(0);

    //
    // Disable all interrupts.
    //
    MAP_IntMasterDisable();

    //
    // Disable SysTick and its interrupt.
    //
    MAP_SysTickIntDisable();
    MAP_SysTickDisable();

    //
    // Disable all processor interrupts.  Instead of disabling them one at a
    // time, a direct write to NVIC is done to disable all peripheral
    // interrupts.
    //
    HWREG(NVIC_DIS0) = 0xffffffff;
    HWREG(NVIC_DIS1) = 0xffffffff;
    HWREG(NVIC_DIS2) = 0xffffffff;
    HWREG(NVIC_DIS3) = 0xffffffff;
    HWREG(NVIC_DIS4) = 0xffffffff;

    //
    // Enable and reset the USB peripheral.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_USB0);
    MAP_USBClockEnable(USB0_BASE, 4, USB_CLOCK_INTERNAL);

    //
    // Wait for about a second.
    //
    MAP_SysCtlDelay(g_ui32SysClock / 3);

    //
    // Re-enable interrupts at the NVIC level.
    //
    MAP_IntMasterEnable();

    //
    // Call the USB ROM boot loader.
    //
    ROM_UpdateUSB(0);

    //
    // Should never get here, but trap just in case.
    //
    while(1)
    {
    }
}
