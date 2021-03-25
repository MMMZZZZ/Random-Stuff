//*****************************************************************************
//
// adc_udma.c - ADC Sampling with a Timer trigger and using uDMA Ping-Pong Mode
//              for transfers to memory buffers.
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
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_adc.h"
#include "inc/hw_types.h"
#include "inc/hw_udma.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>ADC with uDMA Demo (adc_udma)</h1>
//!
//! This example demonstrates how to use the ADC peripheral with both the uDMA
//! and Timer peripherals to optimize the ADC sampling process.  The uDMA is
//! configured for Ping-Pong mode and used to transfer ADC measurement results
//! into a buffer in the background to minimize CPU usage and then indicate
//! when the buffer is ready for processing by the application.  The Timer is
//! used to trigger the ADC measurements at a set sampling frequency of 16 kHz.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - ADC0 peripheral
//! - GPIO Port E peripheral (for AIN0 pin)
//! - AIN0 - PE3
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
// Definition for ADC buffer size.
//
//*****************************************************************************
#define ADC_SAMPLE_BUF_SIZE     64

//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************
//
// Global buffers to store ADC sample data.
//
//*****************************************************************************
static uint16_t pui16ADCBuffer1[ADC_SAMPLE_BUF_SIZE];
static uint16_t pui16ADCBuffer2[ADC_SAMPLE_BUF_SIZE];

//*****************************************************************************
//
// Each possible state of the fill status for an ADC buffer.
//
//*****************************************************************************
enum BUFFER_STATUS
{
    EMPTY,
    FILLING,
    FULL
};

//*****************************************************************************
//
// Global variable to keep track of the fill status of each ADC buffer.
//
//*****************************************************************************
static enum BUFFER_STATUS pui32BufferStatus[2];

//*****************************************************************************
//
// The count of uDMA errors.  This value is incremented by the uDMA error
// handler.
//
//*****************************************************************************
static uint32_t g_ui32DMAErrCount = 0u;

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
// The interrupt handler for uDMA errors.  This interrupt will occur if the
// uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//
//*****************************************************************************
void
uDMAErrorHandler(void)
{
    uint32_t ui32Status;

    //
    // Check for uDMA error bit.
    //
    ui32Status = uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ui32Status)
    {
        uDMAErrorStatusClear();
        g_ui32DMAErrCount++;
    }
}

//*****************************************************************************
//
// Interrupt handler for ADC0 Sequence Zero.
//
//*****************************************************************************
void
ADCSeq0Handler(void)
{
    //
    // Clear the Interrupt Flag.
    //
    ADCIntClear(ADC0_BASE, 0);

    //
    // Determine which buffer as been filled based on the UDMA_MODE_STOP flag
    // and update the buffer status.
    //
    if ((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT) ==
                            UDMA_MODE_STOP) &&
                           (pui32BufferStatus[0] == FILLING))
    {
        pui32BufferStatus[0] = FULL;
        pui32BufferStatus[1] = FILLING;
    }
    else if ((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT) ==
                                 UDMA_MODE_STOP) &&
                                (pui32BufferStatus[1] == FILLING))
    {
        pui32BufferStatus[0] = FILLING;
        pui32BufferStatus[1] = FULL;
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
// This example demonstrates how to use the uDMA controller to transfer data
// between the ADC peripheral and memory buffers.  It also uses a timer to
// trigger ADC measurements at a 16 kHz sampling frequency.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32Count, ui32AverageResult1, ui32AverageResult2;
    uint32_t ui32SamplesTaken = 0;

    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                         SYSCTL_OSC_MAIN |
                                         SYSCTL_USE_PLL |
                                         SYSCTL_CFG_VCO_240), 120000000);

    //
    // Initialize the buffer status.
    //
    pui32BufferStatus[0] = FILLING;
    pui32BufferStatus[1] = EMPTY;

    //
    // Enable the peripherals used by this application.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Enable the GPIO pin for ADC0 Channel 0 (PE3) which configures it for
    // analog functionality.
    //
    MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    //
    // Initialize UART0 and write initial status.
    //
    ConfigureUART();
    UARTprintf("Timer->ADC->uDMA demo!\n\n");
    UARTprintf("ui32AverageResult1\tui32AverageResult2\tTotal Samples\n");

    //
    // Enable the uDMA controller.
    //
    uDMAEnable();

    //
    // Point at the control table to use for channel control structures.
    //
    uDMAControlBaseSet(pui8ControlTable);

    //
    // Put the attributes in a known state for the uDMA ADC0 channel.  These
    // should already be disabled by default.
    //
    uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC0,
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY |
                                UDMA_ATTR_REQMASK);

    //
    // Configure the control parameters for the primary control structure for
    // the ADC0 channel.  The primary control structure is used for the "A"
    // part of the ping-pong receive.  The transfer data size is 16 bits, the
    // source address does not increment since it will be reading from a
    // register.  The destination address increment is 16-bits.  The
    // arbitration size is set to one byte transfers.
    //
    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_SIZE_16 |
                          UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

    //
    // Configure the control parameters for the alternate control structure for
    // the ADC0 channel.  The alternate control structure is used for the
    // "B" part of the ping-pong receive.  The configuration is identical to
    // the primary/A control structure.
    //
    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_SIZE_16 |
                          UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

    //
    // Set up the transfer parameters for the ADC0 primary control structure
    // The mode is set to ping-pong, the transfer source is the ADC Sample
    // Sequence Result FIFO 0 register, and the destination is the receive
    // "A" buffer.  The transfer size is set to match the size of the buffer.
    //
    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                           UDMA_MODE_PINGPONG,
                           (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                           &pui16ADCBuffer1, ADC_SAMPLE_BUF_SIZE);

    //
    // Set up the transfer parameters for the ADC0 primary control structure
    // The mode is set to ping-pong, the transfer source is the ADC Sample
    // Sequence Result FIFO 0 register, and the destination is the receive
    // "B" buffer.  The transfer size is set to match the size of the buffer.
    //
    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
                           UDMA_MODE_PINGPONG,
                           (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                           &pui16ADCBuffer2, ADC_SAMPLE_BUF_SIZE);

    //
    // Set the USEBURST attribute for the uDMA ADC0 channel.  This will force
    // the controller to always use a burst when transferring data from the
    // TX buffer to the UART.  This is somewhat more efficient bus usage than
    // the default which allows single or burst transfers.
    //
    uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC0, UDMA_ATTR_USEBURST);


    // Enables DMA channel so it can perform transfers.  As soon as the
    // channels are enabled, the peripheral will issue a transfer request and
    // the data transfers will begin.
    //
    uDMAChannelEnable(UDMA_CHANNEL_ADC0);

    //
    // Use ADC0 sequence 0 to sample channel 0 once for each timer period.
    //
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_HALF, 1);

    //
    // Wait for the clock configuration to set.
    //
    SysCtlDelay(10);

    //
    // Disable the ADC0 sequence 0 interrupt on the processor (NVIC).
    //
    IntDisable(INT_ADC0SS0);

    //
    // Disable interrupts for ADC0 sample sequence 0 to configure it.
    //
    ADCIntDisable(ADC0_BASE, 0);

    //
    // Disable ADC0 sample sequence 0.  With the sequence disabled, it is now
    // safe to load the new configuration parameters.
    //
    ADCSequenceDisable(ADC0_BASE, 0);

    //
    // Enable sample sequence 0 with a processor signal trigger.  Sequence 0
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    //
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);

    //
    // Configure step 0 on sequence 0.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 0 (ADC_CTL_END).  Sequence
    // 0 has 8 programmable steps.  Since we are only doing a single conversion
    // using sequence 0 we will only configure step 0.  For more information
    // on the ADC sequences and steps, reference the datasheet.
    //
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 | ADC_CTL_END |
                             ADC_CTL_IE);

    //
    // Since sample sequence 0 is now configured, it must be enabled.
    //
    ADCSequenceEnable(ADC0_BASE, 0);

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    ADCIntClear(ADC0_BASE, 0);

    //
    // Enables the DMA channel for the ADC0 sample sequence 0.
    //
    ADCSequenceDMAEnable(ADC0_BASE, 0);

    //
    // Enable the ADC 0 sample sequence 0 interrupt.
    //
    ADCIntEnable(ADC0_BASE, 0);

    //
    // Enable the interrupt for ADC0 sequence 0 on the processor (NVIC).
    //
    IntEnable(INT_ADC0SS0);

    //
    // Configure a 16-bit periodic timer.
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

    //
    // Set ADC sampling frequency to be 16KHz i.e. every 62.5uS.
    //
    TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock/16000) - 1);

    //
    // Enable the ADC trigger output for Timer A.
    //
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Enable Timer 0 which will start the whole application process.
    //
    TimerEnable(TIMER0_BASE, TIMER_A);

    while(1)
    {
        //
        // Check if the first buffer is full, if so process data.
        //
        if(pui32BufferStatus[0] == FULL)
        {
            //
            // Process the data in pui16ADCBuffer1 and clear buffer entries.
            //
            ui32AverageResult1 = 0;

            for(ui32Count = 0; ui32Count < ADC_SAMPLE_BUF_SIZE; ui32Count++)
            {
                ui32AverageResult1 += pui16ADCBuffer1[ui32Count];
                pui16ADCBuffer1[ui32Count] = 0;
            }

            pui32BufferStatus[0] = EMPTY;

            //
            // Enable for another uDMA block transfer.
            //
            uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                                   &pui16ADCBuffer1, ADC_SAMPLE_BUF_SIZE);
            //
            // Enable DMA channel so it can perform transfers.
            //
            uDMAChannelEnable(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT);

            //
            // Track the number of samples taken and update the average.
            //
            ui32SamplesTaken += ADC_SAMPLE_BUF_SIZE;
            ui32AverageResult1 = ((ui32AverageResult1 +
                                  (ADC_SAMPLE_BUF_SIZE / 2)) /
                                   ADC_SAMPLE_BUF_SIZE);
        }

        //
        // Check if the second buffer is full, if so process data.
        //
        if(pui32BufferStatus[1] == FULL)
        {
            //
            // Process the data in pui16ADCBuffer2 and clear buffer entries.
            //
            ui32AverageResult2 = 0;

            for(ui32Count =0; ui32Count < ADC_SAMPLE_BUF_SIZE; ui32Count++)
            {
                ui32AverageResult2 += pui16ADCBuffer2[ui32Count];
                pui16ADCBuffer2[ui32Count] = 0;
            }

            //
            // Indicate the Buffer data as been processed so new data can
            // be stored.
            //
            pui32BufferStatus[1] = EMPTY;

            //
            // Enable for another uDMA block transfer.
            //
            uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                                   &pui16ADCBuffer2, ADC_SAMPLE_BUF_SIZE);

            //
            // Enable DMA channel so it can perform transfers.
            //
            uDMAChannelEnable(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT);

            //
            // Track the number of samples taken and update the average.
            //
            ui32SamplesTaken += ADC_SAMPLE_BUF_SIZE;
            ui32AverageResult2 = ((ui32AverageResult2 +
                                  (ADC_SAMPLE_BUF_SIZE / 2)) /
                                   ADC_SAMPLE_BUF_SIZE);

            //
            // Both buffers have been filled by now, so print out the two
            // average results and the number of samples taken so far.
            //
            UARTprintf("\t%4d\t\t\t%4d\t\t%d\r", ui32AverageResult1,
                       ui32AverageResult2, ui32SamplesTaken);
        }
    }
}

