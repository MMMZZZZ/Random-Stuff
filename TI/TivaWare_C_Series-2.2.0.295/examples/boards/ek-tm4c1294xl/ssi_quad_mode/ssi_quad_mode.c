//*****************************************************************************
//
// ssi_quad_mode.c - Demonstrates configuring SSI0 and SSI1 in Quad-SSI mode.
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
// This is part of revision 2.2.0.295 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Quad-SSI Master (ssi_quad_mode)</h1>
//!
//! This example shows how to configure SSI0 as a Quad-SSI Master and SSI1 as a
//! Quad-SSI slave.  The master device will send four characters to the slave
//! device using the advanced Quad mode.  In Quad-SSI mode, four bits are sent
//! on each SSI Clock pulse.  Once the Quad-SSI slave receives the four
//! characters in its receive FIFO it will generate an interrupt.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - SSI0 peripheral
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0Clk    - PA2
//! - SSI0Fss    - PA3
//! - SSI0XDAT0  - PA4
//! - SSI0XDAT1  - PA5
//! - SSI0XDAT2  - PA6
//! - SSI0XDAT3  - PA7
//!
//! - SSI1 peripheral
//! - GPIO Port B, D, E peripheral (for SSI1 pins)
//! - SSI1Clk    - PB5
//! - SSI1Fss    - PB4
//! - SSI1XDAT0  - PE4
//! - SSI1XDAT1  - PE5
//! - SSI1XDAT2  - PD4
//! - SSI1XDAT3  - PD5
//!
//! This example requires board level connection between SSI0 and SSI1.
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

//*****************************************************************************
//
// Global flag to indicate data was received
//
//*****************************************************************************
volatile uint32_t g_bReceiveFlag = 0;

//*****************************************************************************
//
// Number of bytes to send and receive.
//
//*****************************************************************************
#define NUM_SSI_DATA            4

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
// Configure the UART and its pins. This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

//*****************************************************************************
//
// When the received FIFO is half-full, an interrupt will be generated.
//
//*****************************************************************************
void
SSI1IntHandler(void)
{
    uint32_t ui32Status;

    //
    // Read the SSI Masked Interrupt Status.
    //
    ui32Status = MAP_SSIIntStatus(SSI1_BASE, true);

    //
    // Clear the SSI interrupt.
    //
    MAP_SSIIntClear(SSI1_BASE, ui32Status);

    //
    // Turn off the RX FIFO interrupt.
    //
    MAP_SSIIntDisable(SSI1_BASE, SSI_RXFF);

    //
    // Toggle flag to indicate that data has been received.
    //
    g_bReceiveFlag = 1;
}


//*****************************************************************************
//
// Configure SSI0 in Quad-SSI master Freescale (SPI) mode and SSI1 in Quad-SSI
// slave mode.  The SSI0 will send out 4 bytes of data in advanced Quad mode
// and the SSI1 slave will receive the 4 bytes of data also in Quad mode. The
// slave will generate interrupt upon receiving the 4 bytes of data.
//
//*****************************************************************************
int
main(void)
{
    uint32_t pui32DataTx[NUM_SSI_DATA];
    uint32_t pui32DataRx[NUM_SSI_DATA];
    uint32_t ui32Index;

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
    // Set up the serial console to use for displaying messages.
    //
    ConfigureUART();

    //
    // Display the setup on the console.
    //
    UARTprintf("SSI ->\n");
    UARTprintf("Mode: Advanced Quad-SPI\n");
    UARTprintf("Data: 8-bit\n\n");

    //
    // The SSI0 and SSI1 peripheral must be enabled for use.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    //
    // For this example SSI0 is used with PortA[7:2].  The SSI1 uses
    // PortB, PortD and PortE for the SSICLK, SSIFss and the 4 data pins.
    // GPIO ports need to be enabled so those pins can be used.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Configure the pin muxing for SSI0 functions on PA[7:2].
    // Configure the pin muxing for SSI1 functions on PB, PD and PE.
    //
    MAP_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    MAP_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    MAP_GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
    MAP_GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);
    MAP_GPIOPinConfigure(GPIO_PA6_SSI0XDAT2);
    MAP_GPIOPinConfigure(GPIO_PA7_SSI0XDAT3);

    MAP_GPIOPinConfigure(GPIO_PB5_SSI1CLK);
    MAP_GPIOPinConfigure(GPIO_PB4_SSI1FSS);
    MAP_GPIOPinConfigure(GPIO_PE4_SSI1XDAT0);
    MAP_GPIOPinConfigure(GPIO_PE5_SSI1XDAT1);
    MAP_GPIOPinConfigure(GPIO_PD4_SSI1XDAT2);
    MAP_GPIOPinConfigure(GPIO_PD5_SSI1XDAT3);

    //
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      SSI0
    //      PA7 - SSI0XDAT3
    //      PA6 - SSI0XDAT2
    //      PA5 - SSI0XDAT1
    //      PA4 - SSI0XDAT0
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
    //      SSI1
    //      PD5 - SSI1XDAT3
    //      PD4 - SSI1XDAT2
    //      PE5 - SSI1XDAT1
    //      PE4 - SSI1XDAT0
    //      PB4 - SSI1Fss
    //      PB5 - SSI1CLK
    //
    MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 |
                       GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
    MAP_GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_5 | GPIO_PIN_4 );
    MAP_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4 );
    MAP_GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_5 | GPIO_PIN_4 );

    //
    // Configure and enable the SSI0 port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 2MHz SSI frequency, and 8-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the device datasheet for more
    // information on the different SPI modes.
    //
    MAP_SSIConfigSetExpClk(SSI0_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, 2000000, 8);

    //
    // Configure and enable the SSI1 port for SPI slave mode.
    //
    MAP_SSIConfigSetExpClk(SSI1_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_SLAVE, 2000000, 8);

    //
    // Enable Quad-SSI mode for both SSI0 and SSI1.
    //
    MAP_SSIAdvModeSet(SSI0_BASE, SSI_ADV_MODE_QUAD_WRITE);
    MAP_SSIAdvModeSet(SSI1_BASE, SSI_ADV_MODE_QUAD_READ);

    //
    // Hold the Fss pin low during transfers.  The Fss will be controlled
    // directly by the SSIAdvDataPutFrameEnd().  If calling
    // SSIAdvDataPutFrameEnd to write data to the FIFO, the Fss is de-asserted
    // for the corresponding data.
    //
    MAP_SSIAdvFrameHoldEnable(SSI0_BASE);

    //
    // Enable processor interrupts.
    //
    MAP_IntMasterEnable();

    //
    // Enable SSI1 interrupt on RX FIFO full.
    //
    MAP_SSIIntEnable(SSI1_BASE, SSI_RXFF);

    //
    // Enable the SSI1 interrupts on the processor (NVIC).
    //
    MAP_IntEnable(INT_SSI1);

    //
    // Enable the SSI0 and SSI1 modules.
    //
    MAP_SSIEnable(SSI0_BASE);
    MAP_SSIEnable(SSI1_BASE);

    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    while(MAP_SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[0]))
    {
    }
    while(MAP_SSIDataGetNonBlocking(SSI1_BASE, &pui32DataRx[0]))
    {
    }

    //
    // Initialize the data to send.
    //
    pui32DataTx[0] = 'Q';
    pui32DataTx[1] = 'S';
    pui32DataTx[2] = 'S';
    pui32DataTx[3] = 'I';

    //
    // Display indication that the SSI0 is transmitting data.
    //
    UARTprintf("SSI0 Sent:\n  ");
    UARTprintf("'Q' 'S' 'S' 'I' ");

    //
    // Send 4 bytes of data.
    //
    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
    {

        //
        // Dummy write to start slave which is required for Quad-SSI
        // mode operation.
        //
        MAP_SSIDataPut(SSI1_BASE, 0);

        //
        // Check if the last data byte is queued up to be sent.
        //
        if (ui32Index == (NUM_SSI_DATA - 1))
        {
            //
            // Calling SSIAdvDataPutFrameEnd on the last data will put out the
            // data and de-assert the the Fss pin.
            //
            MAP_SSIAdvDataPutFrameEnd(SSI0_BASE, pui32DataTx[ui32Index]);
        }
        else
        {
            //
            // Send the data using the "blocking" put function.  This function
            // will wait until there is room in the send FIFO before returning.
            // This allows you to assure that all the data you send makes it
            // into the send FIFO.
            //
            MAP_SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
        }
    }


    //
    // Wait until SSI1 receives the half-full interrupt on the RX FIFO.
    //
    while (g_bReceiveFlag == 0);

    //
    // Display indication that the SSI is receiving data.
    //
    UARTprintf("\nSSI1 Received:\n  ");

    //
    // Receive 4 bytes of data.
    //
    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
    {
        //
        // Receive the data using the "blocking" Get function.  This function
        // will wait until there is data in the receive FIFO before returning.
        //
        MAP_SSIDataGet(SSI1_BASE, &pui32DataRx[ui32Index]);

        //
        // Since we are using 8-bit data, mask off the MSB.
        //
        pui32DataRx[ui32Index] &= 0x00FF;

        //
        // Display the data that SSI1 received.
        //
        UARTprintf("'%c' ", pui32DataRx[ui32Index]);
    }

    //
    // Return no errors.
    //
    while (1);
}
