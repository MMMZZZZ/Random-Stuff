//*****************************************************************************
//
// udma_scatter_gather.c - uDMA Scatter Gather example.
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

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "utils/cpu_usage.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>uDMA Scatter Gather Demo (udma_scatter_gather)</h1>
//!
//! This example application demonstrates the use of the uDMA controller to
//! transfer data between memory buffers, and to transfer data to and from a
//! UART channel using the scatter-gather mode.  The test runs for 10 seconds
//! before exiting.
//!
//! UART0, connected to the ICDI virtual COM port and running at 115,200,
//! 8-N-1, is used to display messages from this application.
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
// The number of SysTick ticks per second used for the SysTick interrupt.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     100

//*****************************************************************************
//
// The size of the memory transfer source and destination buffers (in words).
//
//*****************************************************************************
#define MEM_BUFFER_SIZE         16

//*****************************************************************************
//
// The size of the UART transmit and receive buffers.  They do not need to be
// the same size.
//
//*****************************************************************************
#define UART_TXBUF_SIZE         16
#define UART_RXBUF_SIZE         48

//*****************************************************************************
//
// The source and destination buffers used for memory transfers.  This example
// illustrates using memory scatter mode to transfer from three different
// memory buffers to one destination memory buffer.  The destination memory
// buffer size is the sum of the three source memory buffers.
//
//*****************************************************************************
static uint32_t g_ui32SrcSGBuf1[MEM_BUFFER_SIZE] = {0};
static uint32_t g_ui32SrcSGBuf2[MEM_BUFFER_SIZE] = {0};
static uint32_t g_ui32SrcSGBuf3[MEM_BUFFER_SIZE] = {0};
static uint32_t g_ui32DstSGBuf[3 * MEM_BUFFER_SIZE] = {0};

//*****************************************************************************
//
// The transmit and receive buffers used for the UART transfers.  This example
// illustrates using peripheral scatter mode to transfer from three different
// sources of buffer to the UART.  The receive buffer size is the sum of of
// of the three transmit buffers.
//
//*****************************************************************************
static uint8_t g_ui8TxBuf1[UART_TXBUF_SIZE] = "uDMA Peripheral " ;
static uint8_t g_ui8TxBuf2[UART_TXBUF_SIZE] = "Scatter-Gather  ";
static uint8_t g_ui8TxBuf3[UART_TXBUF_SIZE] = "Mode example.   ";
static uint8_t g_ui8RxBuf[UART_RXBUF_SIZE] = {0};

//*****************************************************************************
//
// The count of uDMA errors.  This value is incremented by the uDMA error
// handler.
//
//*****************************************************************************
static uint32_t g_ui32uDMAErrCount = 0;

//*****************************************************************************
//
// The count of times the uDMA interrupt occurred but the uDMA transfer was not
// complete.  This should remain 0.
//
//*****************************************************************************
static uint32_t g_ui32BadISR = 0;

//*****************************************************************************
//
// The count of UART receive buffers filled.
//
//*****************************************************************************
static uint32_t g_ui32RxBufCount = 0;

//*****************************************************************************
//
// The count of memory uDMA transfer blocks.  This value is incremented by the
// uDMA interrupt handler whenever a memory block transfer is completed.
//
//*****************************************************************************
static uint32_t g_ui32MemXferCount = 0;

//*****************************************************************************
//
// The CPU usage in percent, in 16.16 fixed point format.
//
//*****************************************************************************
static uint32_t g_ui32CPUUsage;

//*****************************************************************************
//
// The number of seconds elapsed since the start of the program.  This value is
// maintained by the SysTick interrupt handler.
//
//*****************************************************************************
static uint32_t g_ui32Seconds = 0;

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
// Task list declaration for scatter-gather mode.  There are two task lists,
// one for the memory and another for peripheral
//
//*****************************************************************************
static tDMAControlTable pvMemTaskList[3];
static tDMAControlTable pvUARTTxTaskList[3];

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1)
    {
    }
}
#endif

//*****************************************************************************
//
// The memory task list contains three tasks of transfers.
//
//*****************************************************************************
void
MemTaskListConfigure()
{
    tDMAControlTable Task1 = uDMATaskStructEntry(UART_TXBUF_SIZE, UDMA_SIZE_32,
                                    UDMA_SRC_INC_32, &g_ui32SrcSGBuf3,
                                    UDMA_DST_INC_32, &g_ui32DstSGBuf,
                                    UDMA_ARB_8, UDMA_MODE_MEM_SCATTER_GATHER);
    tDMAControlTable Task2 = uDMATaskStructEntry(UART_TXBUF_SIZE, UDMA_SIZE_32,
                                    UDMA_SRC_INC_32, &g_ui32SrcSGBuf2,
                                    UDMA_DST_INC_32,
                                    &g_ui32DstSGBuf[0] + MEM_BUFFER_SIZE,
                                    UDMA_ARB_8, UDMA_MODE_MEM_SCATTER_GATHER);
    tDMAControlTable Task3 = uDMATaskStructEntry(UART_TXBUF_SIZE, UDMA_SIZE_32,
                                    UDMA_SRC_INC_32, &g_ui32SrcSGBuf1,
                                    UDMA_DST_INC_32,
                                    &g_ui32DstSGBuf[0] + 2 * MEM_BUFFER_SIZE,
                                    UDMA_ARB_8, UDMA_MODE_AUTO);

    pvMemTaskList[0] = Task1;
    pvMemTaskList[1] = Task2;
    pvMemTaskList[2] = Task3;
}

//*****************************************************************************
//
// The peripheral task list contains three tasks of transfers.
//
//*****************************************************************************
void
UARTTxTaskListConfigure()
{

    tDMAControlTable Task1 = uDMATaskStructEntry(UART_TXBUF_SIZE, UDMA_SIZE_8,
                                        UDMA_SRC_INC_8, &g_ui8TxBuf1,
                                        UDMA_DST_INC_NONE,
                                        (void *)(UART1_BASE + UART_O_DR),
                                        UDMA_ARB_4,
                                        UDMA_MODE_PER_SCATTER_GATHER);
    tDMAControlTable Task2 = uDMATaskStructEntry(UART_TXBUF_SIZE, UDMA_SIZE_8,
                                        UDMA_SRC_INC_8, &g_ui8TxBuf2,
                                        UDMA_DST_INC_NONE,
                                        (void *)(UART1_BASE + UART_O_DR),
                                        UDMA_ARB_4,
                                        UDMA_MODE_PER_SCATTER_GATHER);
    tDMAControlTable Task3 = uDMATaskStructEntry(UART_TXBUF_SIZE, UDMA_SIZE_8,
                                        UDMA_SRC_INC_8, &g_ui8TxBuf3,
                                        UDMA_DST_INC_NONE,
                                        (void *)(UART1_BASE + UART_O_DR),
                                        UDMA_ARB_4, UDMA_MODE_BASIC);

    pvUARTTxTaskList[0] = Task1;
    pvUARTTxTaskList[1] = Task2;
    pvUARTTxTaskList[2] = Task3;
}

//*****************************************************************************
//
// The interrupt handler for the SysTick timer.  This handler will increment a
// seconds counter whenever the appropriate number of ticks has occurred.  It
// will also call the CPU usage tick function to find the CPU usage percent.
//
//*****************************************************************************
void
SysTickHandler(void)
{
    static uint32_t ui32TickCount = 0;

    //
    // Increment the tick counter.
    //
    ui32TickCount++;

    //
    // If the number of ticks per second has occurred, then increment the
    // seconds counter.
    //
    if(!(ui32TickCount % SYSTICKS_PER_SECOND))
    {
        g_ui32Seconds++;
    }

    //
    // Call the CPU usage tick function.  This function will compute the amount
    // of cycles used by the CPU since the last call and return the result in
    // percent in fixed point 16.16 format.
    //
    g_ui32CPUUsage = CPUUsageTick();
}

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
    ui32Status = MAP_uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ui32Status)
    {
        MAP_uDMAErrorStatusClear();
        g_ui32uDMAErrCount++;
    }
}

//*****************************************************************************
//
// The interrupt handler for uDMA interrupts from the memory channel.  This
// interrupt will increment a counter, and then restart another memory
// transfer.
//
//*****************************************************************************
void
uDMAIntHandler(void)
{
    uint32_t ui32Mode;

    //
    // Check for the primary control structure to indicate complete.
    //
    ui32Mode = MAP_uDMAChannelModeGet(UDMA_CHANNEL_SW);

    if(ui32Mode == UDMA_MODE_STOP)
    {
        //
        // Increment the count of completed transfers.
        //
        g_ui32MemXferCount++;

        //
        // Configure it for another transfer.
        //
        MAP_uDMAChannelScatterGatherSet(UDMA_CHANNEL_SW, 3, pvMemTaskList,
                                        false);
        //
        // Initiate another transfer.
        //
        MAP_uDMAChannelEnable(UDMA_CHANNEL_SW);
        MAP_uDMAChannelRequest(UDMA_CHANNEL_SW);
    }

    //
    // If the channel is not stopped, then something is wrong.
    //
    else
    {
        g_ui32BadISR++;
    }
}

//*****************************************************************************
//
// The interrupt handler for UART1.  This interrupt will occur when a DMA
// transfer is complete using the UART1 uDMA channel.  It will also be
// triggered if the peripheral signals an error as well as restart a TX
// uDMA transfer if the prior transfer is complete.  This will keep the UART
// running continuously (looping TX data back to RX).
//
//*****************************************************************************
void
UART1IntHandler(void)
{
    uint32_t ui32Status;
    uint32_t ui32Mode;

    //
    // Read the interrupt status of the UART.
    //
    ui32Status = MAP_UARTIntStatus(UART1_BASE, 1);

    //
    // Clear any pending status, even though there should be none since no UART
    // interrupts were enabled.  If UART error interrupts were enabled, then
    // those interrupts could occur here and should be handled.  Since uDMA is
    // used for both the RX and TX, then neither of those interrupts should be
    // enabled.
    //
    MAP_UARTIntClear(UART1_BASE, ui32Status);

    //
    // Check the DMA control table to see if the UART RX transfer is
    // complete.
    //
    ui32Mode = MAP_uDMAChannelModeGet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT);

    //
    // If the primary control structure indicates stop, that means the
    // receive buffer is done.
    //
    if(ui32Mode == UDMA_MODE_STOP)
    {
        //
        // Increment a counter to indicate data was received into the RX
        // buffer.  In a real application this would be used to signal the
        // main thread that data was received so the main thread can process
        // the data.
        //
        g_ui32RxBufCount++;

        //
        // Re-enable the UART1 RX channel to start the next transfer.
        //
        MAP_uDMAChannelEnable(UDMA_CHANNEL_UART1RX);
    }

    //
    // If the UART1 DMA TX channel is disabled, that means the TX DMA transfer
    // is done.
    //
    if(!MAP_uDMAChannelIsEnabled(UDMA_CHANNEL_UART1TX))
    {
        //
        // Start another DMA transfer to UART1 TX. Repeat the same peripheral
        // task list.
        //
        MAP_uDMAChannelScatterGatherSet(UDMA_CHANNEL_UART1TX, 3,
                                        pvUARTTxTaskList, true);

        //
        // The uDMA TX channel must be re-enabled.
        //
        MAP_uDMAChannelEnable(UDMA_CHANNEL_UART1TX);
    }
}

//*****************************************************************************
//
// Initializes the UART1 peripheral and sets up the TX and RX uDMA channels.
// The UART is configured for loopback mode so that any data sent on TX will be
// received on RX.  The uDMA channels are configured so that the TX channel
// will copy data from three source buffers to the UART TX output using
// peripheral scatter gather mode.  And the uDMA RX channel will receive any
// incoming data into the receive buffer using BASIC mode.
//
//*****************************************************************************
void
InitUART1Transfer(void)
{
    uint_fast16_t ui16Idx;

    //
    // Fill the three TX buffers with a simple data pattern.
    //
    for(ui16Idx = 0; ui16Idx < UART_TXBUF_SIZE; ui16Idx++)
    {
        g_ui8TxBuf1[ui16Idx] = ui16Idx;
        g_ui8TxBuf2[ui16Idx] = ui16Idx + UART_TXBUF_SIZE;
        g_ui8TxBuf3[ui16Idx] = ui16Idx + 2 * UART_TXBUF_SIZE;
    }

    //
    // Enable the UART peripheral, and configure it to operate even if the CPU
    // is in sleep.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART1);

    //
    // Configure the UART communication parameters.
    //
    MAP_UARTConfigSetExpClk(UART1_BASE, g_ui32SysClock, 115200,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE);

    //
    // Set both the TX and RX trigger thresholds to 4.  This will be used by
    // the uDMA controller to signal when more data should be transferred.  The
    // uDMA TX and RX channels will be configured so that it can transfer 4
    // bytes in a burst when the UART is ready to transfer more data.
    //
    MAP_UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

    //
    // Enable the UART for operation, and enable the uDMA interface for both TX
    // and RX channels.
    //
    MAP_UARTEnable(UART1_BASE);
    MAP_UARTDMAEnable(UART1_BASE, UART_DMA_RX | UART_DMA_TX);

    //
    // This register write will set the UART to operate in loopback mode.  Any
    // data sent on the TX output will be received on the RX input.
    //
    HWREG(UART1_BASE + UART_O_CTL) |= UART_CTL_LBE;

    //
    // Put the attributes in a known state for the uDMA UART1RX channel.  These
    // should already be disabled by default.
    //
    MAP_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART1RX,
                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);

    //
    // Configure the control parameters for the primary control structure for
    // the UART RX channel.  The transfer data size is 8 bits, the source
    // address does not increment since it will be reading from a register.
    // The destination address increment is byte 8-bit bytes.  The arbitration
    // size is set to 4 to match the RX FIFO trigger threshold.  The uDMA
    // controller will use a 4 byte burst transfer if possible.  This will be
    // somewhat more efficient that single byte transfers.
    //
    MAP_uDMAChannelControlSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE |
                              UDMA_DST_INC_8 | UDMA_ARB_4);

    //
    // Set up the transfer parameters for the UART RX primary control
    // structure.  The transfer size is set to match the size of the buffer.
    //
    MAP_uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *)(UART1_BASE + UART_O_DR),
                               g_ui8RxBuf, sizeof(g_ui8RxBuf));

    //
    // The uDMA UART TX channel is used to transfer three blocks of data to
    // the UART.  The transfer is based on the three tasks configured in the
    // peripheral list. Each task is set up to transfer from one source buffer
    // to the UART.
    //
    UARTTxTaskListConfigure();

    MAP_uDMAChannelScatterGatherSet(UDMA_CHANNEL_UART1TX, 3, pvUARTTxTaskList,
                                true);

    //
    // Enable the UART DMA TX/RX interrupts.
    //
    MAP_UARTIntEnable(UART1_BASE, UART_INT_DMATX | UART_INT_DMATX);

    //
    // Enable the UART peripheral interrupts.
    //
    MAP_IntEnable(INT_UART1);

    //
    // Now both the uDMA UART TX and RX channels are primed to start a
    // transfer.  As soon as the channels are enabled, the peripheral will
    // issue a transfer request and the data transfers will begin.
    //
    MAP_uDMAChannelEnable(UDMA_CHANNEL_UART1RX);
    MAP_uDMAChannelEnable(UDMA_CHANNEL_UART1TX);
}

//*****************************************************************************
//
// Initializes the uDMA software channel to perform a memory to memory uDMA
// transfer using scatter-gather mode.  Three discontinuous source buffers are
// transferred to the destination buffer using the scatter-gather mode.
//
//*****************************************************************************
void
InitSGTransfer(void)
{
    uint_fast16_t ui16Idx;

    //
    // Fill the three memory source buffers with simple pattern.
    //
    for(ui16Idx = 0; ui16Idx < MEM_BUFFER_SIZE; ui16Idx++)
    {
        g_ui32SrcSGBuf3[ui16Idx] = ui16Idx;
        g_ui32SrcSGBuf2[ui16Idx] = ui16Idx + MEM_BUFFER_SIZE;
        g_ui32SrcSGBuf1[ui16Idx] = ui16Idx + 2 * MEM_BUFFER_SIZE;
    }

    //
    // Enable interrupts from the uDMA software channel.
    //
    MAP_IntEnable(INT_UDMA);

    //
    // Configure the Software DMA channel (UDMA_CHANNEL_SW) for
    // scatter-gather mode by associating the channel with the task list
    // and the number of the tasks in the list.
    //
    MAP_uDMAChannelScatterGatherSet(UDMA_CHANNEL_SW, 3, pvMemTaskList , false);

    //
    // Now the software channel is primed to start a transfer.  The channel
    // must be enabled.  For software based transfers, a request must be
    // issued.  After this, the uDMA memory transfer begins.
    //
    MAP_uDMAChannelEnable(UDMA_CHANNEL_SW);
    MAP_uDMAChannelRequest(UDMA_CHANNEL_SW);
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
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);

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
// between memory buffers and to and from a peripheral, in this case a UART.
// The uDMA controller is configured to repeatedly transfer data from three
// memory source buffers to one destination memory buffer.  It is also set up
// to repeatedly copy three blocks of data buffers to the UART output.  The
// UART data is looped back so the same data is received, and the uDMA controlled
// is configured to continuously receive the UART data into one receive buffer
// using BASIC mode.
//
// The processor is put to sleep when it is not doing anything, and this allows
// collection of CPU usage data to see how much CPU is being used while the
// data transfers are ongoing.
//
//*****************************************************************************
int
main(void)
{
    static uint32_t ui32PrevSeconds;
    static uint32_t ui32PrevXferCount;
    static uint32_t ui32PrevUARTCount = 0;
    uint32_t ui32XfersCompleted;
    uint32_t ui32BytesTransferred;

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
    // Enable peripherals to operate when CPU is in sleep.
    //
    MAP_SysCtlPeripheralClockGating(true);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //
    // Enable the GPIO pins for the LED (PN0).
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    //
    // Initialize the UART and display initial message.
    //
    ConfigureUART();
    UARTprintf("\033[2J\033[H");
    UARTprintf("uDMA Example using Memory and Peripheral Scatter-Gather mode\n");

    //
    // Show the clock frequency on the display.
    //
    UARTprintf("Tiva C Series @ %u MHz\n\n", g_ui32SysClock / 1000000);

    //
    // Show statistics headings.
    //
    UARTprintf("CPU\tRemaining\tMemory\t\t\t   UART\n");
    UARTprintf("Usage\tTime\t\tTransfers\t\t   Transfers\n");

    //
    // Configure SysTick to occur 100 times per second, to use as a time
    // reference.  Enable SysTick to generate interrupts.
    //
    MAP_SysTickPeriodSet(g_ui32SysClock / SYSTICKS_PER_SECOND);
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();

    //
    // Initialize the CPU usage measurement routine.
    //
    CPUUsageInit(g_ui32SysClock, SYSTICKS_PER_SECOND, 2);

    //
    // Enable the uDMA controller at the system level.  Enable it to continue
    // to run while the processor is in sleep.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);

    //
    // Enable the uDMA controller error interrupt.  This interrupt will occur
    // if there is a bus error during a transfer.
    //
    MAP_IntEnable(INT_UDMAERR);

    //
    // Enable the uDMA controller.
    //
    MAP_uDMAEnable();

    //
    // Point at the control table to use for channel control structures.
    //
    MAP_uDMAControlBaseSet(pui8ControlTable);

    //
    // Initialize the uDMA memory to memory transfers using memory
    // scatter-gather mode.
    //
    MemTaskListConfigure();
    InitSGTransfer();

    //
    // Initialize the uDMA UART transfers.
    //
    InitUART1Transfer();

    //
    // Remember the current SysTick seconds count.
    //
    ui32PrevSeconds = g_ui32Seconds;

    //
    // Remember the current count of memory buffer transfers.
    //
    ui32PrevXferCount = g_ui32MemXferCount;

    //
    // Loop until the button is pressed.  The processor is put to sleep
    // in this loop so that CPU utilization can be measured.
    //
    while(1)
    {
        //
        // Check to see if one second has elapsed.  If so, the make some
        // updates.
        //
        if(g_ui32Seconds != ui32PrevSeconds)
        {
            //
            // Turn on the LED as a heartbeat.
            //
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

            //
            // Print a message showing the CPU usage percent.
            // The fractional part of the percent value is ignored.
            //
            UARTprintf("%2u%%", g_ui32CPUUsage >> 16);

            //
            // Tell the user how many seconds we have to go before ending.
            //
            UARTprintf("\t%d seconds", 10 - g_ui32Seconds);

            //
            // Remember the new seconds count.
            //
            ui32PrevSeconds = g_ui32Seconds;

            //
            // Calculate how many memory transfers have occurred since the last
            // second.
            //
            ui32XfersCompleted = g_ui32MemXferCount - ui32PrevXferCount;

            //
            // Remember the new transfer count.
            //
            ui32PrevXferCount = g_ui32MemXferCount;

            //
            // Compute how many bytes were transferred in the memory transfer
            // since the last second.
            //
            ui32BytesTransferred = ui32XfersCompleted * MEM_BUFFER_SIZE * 4;
																 
            //
            // Print a message showing the transfer rate.
            //
            UARTprintf("\t%8u Bytes/Sec", ui32BytesTransferred);

            //
            // Calculate how many UART transfers have occurred since the last
            // second.  This expression is split to prevent a compiler warning
            // about undefined order of volatile accesses.
            //
            ui32XfersCompleted = g_ui32RxBufCount;
            ui32XfersCompleted -= ui32PrevUARTCount;

            //
            // Remember the new UART transfer count.  This expression is split
            // to prevent a compiler warning about undefined order of volatile
            // accesses.
            //
            ui32PrevUARTCount = g_ui32RxBufCount;

            //
            // Compute how many bytes were transferred by the UART.  The number
            // of bytes received is multiplied by 2 so that the TX bytes
            // transferred are also accounted for.
            //
            ui32BytesTransferred = ui32XfersCompleted * UART_RXBUF_SIZE * 2;

            //
            // Print a message showing the transfer rate.
            //
            UARTprintf("\t%8u Bytes/Sec\n", ui32BytesTransferred);
						
            //
            // Turn off the LED.
            //
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
        }

        //
        // Put the processor to sleep if there is nothing to do.  This allows
        // the CPU usage routine to measure the number of free CPU cycles.
        // If the processor is sleeping a lot, it can be hard to connect to
        // the target with the debugger.
        //
        MAP_SysCtlSleep();

        //
        // See if we have run long enough and exit the loop if so.
        //
        if(g_ui32Seconds >= 10)
        {
            break;
        }
    }

    //
    // Indicate on the display that the example is stopped.
    //
    UARTprintf("\nStopped.\n");

    //
    // Disable uDMA and UART interrupts now that the test is complete.
    //
    MAP_IntDisable(INT_UART1);
    MAP_IntDisable(INT_UDMA);

    //
    // Loop forever with the CPU not sleeping, so the debugger can connect.
    //
    while(1)
    {
        //
        // Turn on the LED.
        //
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

        //
        // Delay for a bit.
        //
        MAP_SysCtlDelay(g_ui32SysClock / 20 / 3);

        //
        // Turn off the LED.
        //
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);

        //
        // Delay for a bit.
        //
        MAP_SysCtlDelay(g_ui32SysClock / 20 / 3);
    }
}
