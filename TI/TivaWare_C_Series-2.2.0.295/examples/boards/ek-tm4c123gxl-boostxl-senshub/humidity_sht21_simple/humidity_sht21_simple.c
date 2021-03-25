//*****************************************************************************
//
// humidity_sht21.c - Example to demonstrate quick humidity measurements with
//                    the SHT21 and BOOSTXL-SENSHUB BoosterPack.
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
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
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
//! <h1>Humidity Measurement with the SHT21 (humidity_sht21_simple)</h1>
//!
//! This example demonstrates the usage of the I2C interface to obtain the
//! temperature and relative humidity of the environment using the Sensirion
//! SHT21 sensor.
//!
//! The I2C3 on the EK-TM4C123GXL launchPad is used to interface with the
//! SHT21 sensor.  The SHT21 sensor is on the BOOSTXL_SENSHUB boosterPack
//! expansion board that can be directly plugged into the Booster Pack
//! connector on the EK-TM4C123GXL launchPad board.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - I2C3 peripheral
//! - GPIO Port D peripheral
//! - I2C3_SCL - PD0
//! - I2C3_SDA - PD1
//!
//! UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
//! is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// Define SHT21 I2C Address.
//
//*****************************************************************************
#define SHT21_I2C_ADDRESS  0x40

//*****************************************************************************
//
// Define SHT21 Commands.  Please refer to the SHT21 datasheet for
// all available commands.
//
//*****************************************************************************
#define SW_RESET                0xFE
#define TRIGGER_RH_MEASUREMENT  0xF5
#define TRIGGER_T_MEASUREMENT   0xF3

//*****************************************************************************
//
// Application function to capture ASSERT failures and other debug conditions.
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
    // Enable UART0
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
// This function sends the specified command to the I2C slave device.
//
//*****************************************************************************
void
I2CWriteCommand(uint32_t ui32Command)
{
    //
    // Set up the slave address with write transaction.
    //
    MAP_I2CMasterSlaveAddrSet(I2C3_BASE, SHT21_I2C_ADDRESS, false);

    //
    // Store the command data in I2C data register.
    //
    MAP_I2CMasterDataPut(I2C3_BASE, ui32Command);

    //
    // Start the I2C transaction.
    //
    MAP_I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    //
    // Wait until the I2C transaction is complete.
    //
    while(MAP_I2CMasterBusy(I2C3_BASE))
    {
    }
}

//*****************************************************************************
//
// This function will read three 8-bit data from the I2C slave.  The first
// two 8-bit data forms the humidity data while the last 8-bit data is the
// checksum. This function illustrates three different I2C burst mode
// commands to read the I2C slave device.
//
//*****************************************************************************
void
I2CReadCommand(uint32_t * pui32DataRx)
{
    //
    // Modify the data direction to true, so that seeing the address will
    // indicate that the I2C Master is initiating a read from the slave.
    //
    MAP_I2CMasterSlaveAddrSet(I2C3_BASE, SHT21_I2C_ADDRESS, true);

    //
    // Setup for first read.  Use I2C_MASTER_CMD_BURST_RECEIVE_START
    // to start a burst mode read.  The I2C master continues to own
    // the bus at the end of this transaction.
    //
    MAP_I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

    //
    // Wait until master module is done transferring.
    //
    while(MAP_I2CMasterBusy(I2C3_BASE))
    {
    }

    //
    // Read the first byte data from the slave.
    //
    pui32DataRx[0] = MAP_I2CMasterDataGet(I2C3_BASE);

    //
    // Setup for the second read.  Use I2C_MASTER_CMD_BURST_RECEIVE_CONT
    // to continue the burst mode read.  The I2C master continues to own
    // the bus at the end of this transaction.
    //
    MAP_I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

    //
    // Wait until master module is done transferring.
    //
    while(MAP_I2CMasterBusy(I2C3_BASE))
    {
    }

    //
    // Read the second byte data from the slave.
    //
    pui32DataRx[1] = MAP_I2CMasterDataGet(I2C3_BASE);

    //
    // Setup for the third read.  Use I2C_MASTER_CMD_BURST_RECEIVE_FINISH
    // to terminate the I2C transaction.  At the end of this transaction,
    // the STOP bit will be issued and the I2C bus is returned to the
    // Idle state.
    //
    MAP_I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    //
    // Wait until master module is done transferring.
    //
    while(MAP_I2CMasterBusy(I2C3_BASE))
    {
    }
    
    //
    // Note the third 8-bit data is the checksum byte.  It will be
    // left to the users as an exercise if they want to verify if the
    // checksum is correct.
    pui32DataRx[2] = MAP_I2CMasterDataGet(I2C3_BASE);
}

//*****************************************************************************
//
// Main 'C' Language entry point.
//
//*****************************************************************************
int
main(void)
{
    float fTemperature, fHumidity;
    int32_t i32IntegerPart;
    int32_t i32FractionPart;
    uint32_t pui32DataRx[3] = {0};

    //
    // Setup the system clock to run at 40 MHz from PLL with crystal reference.
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Print the welcome message to the terminal.
    //
    UARTprintf("SHT21 Example\n");

    //
    // The I2C3 peripheral must be enabled before use.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    // This step is not necessary if your part does not support pin muxing.
    //
    MAP_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    MAP_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    MAP_GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Enable interrupts to the processor.
    //
    MAP_IntMasterEnable();

    //
    // Enable and initialize the I2C3master module.  Use the system clock for
    // the I2C3 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.  For this example we will use a data rate of 100kbps.
    //
    MAP_I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), false);

    //
    // Initialize the SHT21 Humidity and Temperature sensors.
    //
    I2CWriteCommand(SW_RESET);

    //
    // Per SHT21 sensor datasheet, wait for at least 15ms before the
    // software reset is complete.  Here we will wait for 20ms.
    //
    MAP_SysCtlDelay(MAP_SysCtlClockGet() / (50 * 3));

    while(1)
    {
        //
        // Write the command to start a humidity measurement.
        //
        I2CWriteCommand(TRIGGER_RH_MEASUREMENT);

        //
        // Per SHT21 sensor datasheet, the humidity measurement
        // can take a maximum of 29ms to complete at 12-bit
        // resolution.  Here we will wait for 33ms.
        //
        MAP_SysCtlDelay(MAP_SysCtlClockGet() / (30 * 3));

        //
        // Read the humidity measurements.
        //
        I2CReadCommand(&pui32DataRx[0]);

        //
        // Process the raw measurement and convert it to physical
        // humidity percentage.  Refer to the SHT21 datasheet for the
        // conversion formula.
        //
        pui32DataRx[0] = ((pui32DataRx[0] << 8) + (pui32DataRx[1] & 0xFC));
        fHumidity = (float)(-6 + 125 * (float)pui32DataRx[0] / 65536);

        //
        // Convert the floats to an integer part and fraction part for easy
        // print.  Humidity is returned as 0.0 to 1.0 so multiply by 100 to get
        // percent humidity.
        //
        i32IntegerPart = (int32_t) fHumidity;
        i32FractionPart = (int32_t) (fHumidity * 1000.0f);
        i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
        if(i32FractionPart < 0)
        {
            i32FractionPart *= -1;
        }

        //
        // Print the humidity value using the integers we just converted.
        //
        UARTprintf("Humidity %3d.%03d%%\t", i32IntegerPart, i32FractionPart);

        //
        // Write the command to start a temperature measurement.
        //
        I2CWriteCommand(TRIGGER_T_MEASUREMENT);

        //
        // Per SHT21 sensor datasheet, the temperature measurement
        // can take a maximum of 85ms to complete at 14-bit resolution.
        // Here we will wait for approximately 90ms.
        //
        MAP_SysCtlDelay(MAP_SysCtlClockGet() / (11 * 3));

        //
        // Read the temperature measurements.
        //
        I2CReadCommand(&pui32DataRx[0]);

        //
        // Process the raw measurement and convert it to physical
        // temperature.  Refer to the SHT21 datasheet for the
        // conversion formula.
        //
        pui32DataRx[0] = ((pui32DataRx[0] << 8) + (pui32DataRx[1] & 0xFC));
        fTemperature = (float)(-46.85 + 175.72 * (float)pui32DataRx[0]/65536);

        //
        // Convert the floats to an integer part and fraction part for easy
        // print.
        //
        i32IntegerPart = (int32_t) fTemperature;
        i32FractionPart = (int32_t) (fTemperature * 1000.0f);
        i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
        if(i32FractionPart < 0)
        {
            i32FractionPart *= -1;
        }

        //
        // Print the temperature as integer and fraction parts.
        //
        UARTprintf("Temperature %3d.%03d\n", i32IntegerPart, i32FractionPart);

        //
        // Wait for one second before taking the measurements again.
        //
        MAP_SysCtlDelay(MAP_SysCtlClockGet() / 3);
    }
}
