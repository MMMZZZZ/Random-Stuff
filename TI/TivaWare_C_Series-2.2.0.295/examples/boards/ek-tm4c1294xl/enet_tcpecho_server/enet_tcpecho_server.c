//*****************************************************************************
//
// enet_tcp_echo_server.c - Sample Echo Server Application using lwIP.
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
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/flash.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "drivers/pinout.h"
#include "lwip/tcp.h"
#include "utils/locator.h"
#include "utils/lwiplib.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Ethernet TCP Echo Server (enet_tcp_echo_server)</h1>
//!
//! This example application demonstrates the operation of the TM4C129x
//! Ethernet controller using the lwIP TCP/IP Stack.  DHCP is used to obtain
//! an Ethernet address.  If DHCP times out without obtaining an address,
//! AutoIP will be used to obtain a link-local address.  The address that is
//! selected will be shown on the UART.  The application echoes back the data
//! received from the client.
//!
//! UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
//! is used to display messages from this application.
//!
//! For additional details on lwIP, refer to the lwIP web page at:
//! http://savannah.nongnu.org/projects/lwip/
//
//*****************************************************************************

//*****************************************************************************
//
// Defines for setting up the system clock.
//
//*****************************************************************************
#define SYSTICKHZ               100
#define SYSTICKMS               (1000 / SYSTICKHZ)

//*****************************************************************************
//
// Interrupt priority definitions.  The top 3 bits of these values are
// significant with lower values indicating higher priority interrupts.
//
//*****************************************************************************
#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0

//*****************************************************************************
//
// The variable g_ui32SysClock contains the system clock frequency in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The current IP address.
//
//*****************************************************************************
uint32_t g_ui32IPAddress;

//*****************************************************************************
//
// Global counter to keep track the duration the connection has been idle.
//
//*****************************************************************************
uint32_t g_ui32tcpPollTick = 0;

//*****************************************************************************
//
// Volatile global flag to manage LED blinking, since it is used in interrupt
// and main application.  The LED blinks at the rate of SYSTICKHZ.
//
//*****************************************************************************
volatile bool g_bLED;

//*****************************************************************************
//
// This is the buffer to receive the TCP payload.  Data in this buffer is then
// processed before echoing back to the host.
//
//*****************************************************************************
char g_pctcpBuffer[4096] = {0};

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
// Display an lwIP type IP Address.
//
//*****************************************************************************
void
DisplayIPAddress(uint32_t ui32Addr)
{
    char pcBuf[16];

    //
    // Convert the IP Address into a string.
    //
    usprintf(pcBuf, "%d.%d.%d.%d", ui32Addr & 0xff, (ui32Addr >> 8) & 0xff,
            (ui32Addr >> 16) & 0xff, (ui32Addr >> 24) & 0xff);

    //
    // Display the string.
    //
    UARTprintf(pcBuf);
}

//*****************************************************************************
//
// Required by lwIP library to support any host-related timer functions.
//
//*****************************************************************************
void
lwIPHostTimerHandler(void)
{
    uint32_t ui32NewIPAddress;

    //
    // Get the current IP address.
    //
    ui32NewIPAddress = lwIPLocalIPAddrGet();

    //
    // See if the IP address has changed.
    //
    if(ui32NewIPAddress != g_ui32IPAddress)
    {
        //
        // See if there is an IP address assigned.
        //
        if(ui32NewIPAddress == 0xffffffff)
        {
            //
            // Indicate that there is no link.
            //
            UARTprintf("Waiting for link.\n");
        }
        else if(ui32NewIPAddress == 0)
        {
            //
            // There is no IP address, so indicate that the DHCP process is
            // running.
            //
            UARTprintf("Waiting for IP address.\n");
        }
        else
        {
            //
            // Display the new IP address.
            //
            UARTprintf("IP Address: ");
            DisplayIPAddress(ui32NewIPAddress);
            UARTprintf("\nEcho Server is ready.\n");
        }

        //
        // Save the new IP address.
        //
        g_ui32IPAddress = ui32NewIPAddress;
    }

    //
    // If there is not an IP address.
    //
    if((ui32NewIPAddress == 0) || (ui32NewIPAddress == 0xffffffff))
    {
        //
        // Do nothing and keep waiting.
        //
    }
}

//*****************************************************************************
//
// The interrupt handler for the SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Call the lwIP timer handler.
    //
    lwIPTimer(SYSTICKMS);

    //
    // Tell the application to change the state of the LED (in other words
    // blink).
    //
    g_bLED = true;
}

//*****************************************************************************
//
// Callback function to close the connection.
//
//*****************************************************************************
static void
close_conn (struct tcp_pcb *pcb)
{
    //
    // NULL will be passed to all other callback functions.
    //
    tcp_arg(pcb, NULL);
    //
    // NULL callback for the PCB when data is sent.
    //
    tcp_sent(pcb, NULL);
    //
    // NULL callback for the PCB when data is received.
    //
    tcp_recv(pcb, NULL);
    //
    // Closes the connection held by the PCB.
    //
    tcp_close(pcb);
}

//*****************************************************************************
//
// Callback function to indicate the number of bytes that were acknowledged by
// last acknowledgment.
//
//*****************************************************************************
static err_t
echo_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  LWIP_UNUSED_ARG(arg);

  //
  // Print a message indicating the number of bytes that has been acknowledged
  // by the remote host.
  //
  UARTprintf("\rBytes acknowledged by the remote host:%4d", len);

  return ERR_OK;
}

//*****************************************************************************
//
// Callback function to process the received data from the client.
//
//*****************************************************************************
static err_t
echo_recv( void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    uint32_t ui32index;
    uint32_t ui32len;
    char *pcpayLoad;
    char pcheaderMsg[45];
    uint32_t ui32statuslen;

    //
    // Clear the Idle time counter.
    //
    g_ui32tcpPollTick = 0;

    if (err == ERR_OK && p != NULL)
    {
        //
        // tcp_recved must be called when the application has processed the
        // data and is prepared to receive more.
        //
        tcp_recved(pcb, p->tot_len);

        //
        // Obtain the payload length and the pointer to the payload.
        //
        pcpayLoad = (char *)p->payload;
        ui32len = p->tot_len;

        usprintf(pcheaderMsg, "Server received %d bytes. "
                                    "Converting character case.\n", ui32len);

        //
        // What will be echoed back to the host will contain a header message
        // and the received payload from the TCP.  The received payload will
        // be processed to convert its case from upper to lower case and vice
        // versa.  Both the header message and the received TCP data will
        // first be stored in the buffer before processing.
        //

        //
        // First, find out the length of the header message that will be
        // prepended before echoing back the received data from the
        // host.
        //
        ui32statuslen = strlen(pcheaderMsg);

        //
        // Store the TCP payload after the header message to the buffer.
        //
        for( ui32index = 0; ui32index < ui32statuslen; ui32index++ )
        {
            g_pctcpBuffer[ui32index] = pcheaderMsg[ui32index];
        }

        //
        // Copy the received payload to a buffer for processing.
        //
        for(ui32index = ui32statuslen;
                    ui32index < (ui32len + ui32statuslen);
                                                    ui32index++)
        {
            //
            // Is this a lower case character?
            //
            if((pcpayLoad[ui32index-ui32statuslen] >= 'a') &&
               (pcpayLoad[ui32index-ui32statuslen] <= 'z'))
            {
                //
                // Convert to upper case.
                //
                g_pctcpBuffer[ui32index] =
                    (pcpayLoad[ui32index-ui32statuslen] - 'a') + 'A';
            }
            else
            {
                //
                // Is this an upper case character?
                //
                if((pcpayLoad[ui32index-ui32statuslen] >= 'A') &&
                   (pcpayLoad[ui32index-ui32statuslen] <= 'Z'))
                {
                    //
                    // Convert to lower case.
                    //
                    g_pctcpBuffer[ui32index] =
                        (pcpayLoad[ui32index-ui32statuslen] - 'Z') + 'z';
                }
                else
                {
                    //
                    // Copy the received character to the transmit buffer.
                    //
                    g_pctcpBuffer[ui32index] =
                        pcpayLoad[ui32index-ui32statuslen];
                }
            }

        }

        //
        // Dereference a pbuf chain.
        //
        pbuf_free(p);

        //
        // Call tcp_sndbuf() to find the maximum amount of data that can be
        // sent.
        //
        if(ui32len > tcp_sndbuf(pcb))
            ui32len = tcp_sndbuf(pcb);

        //
        // Enqueues the data stored in g_pctcpBuffer.  The length of the data
        // is passed in ui32len.  The argument copy may be either 0 or 1 and
        // indicates whether the new memory should be allocated for the data to
        // be copied into.  If the argument is 0, no new memory should be
        // allocated and the data should only be referenced by pointer.
        //
        tcp_write(pcb, g_pctcpBuffer, ui32len+ui32statuslen, 0);

        //
        // Specifies the callback function echo_sent be called when data has
        // successfully been received (i.e. acknowledged) by the remote host.
        // The len argument passed to the sent callback function gives the
        // number of bytes that were acknowledged by the last acknowledgment.
        //
        tcp_sent(pcb, echo_sent);
    }
    else
    {
        //
        // If there is an error during pbuf allocation then free the pbuf.
        //
        pbuf_free(p);
    }

    //
    // If the remote host requests a connection close with p == NULL then
    // close the connection held by the PCB.
    //
    if(err == ERR_OK && p == NULL)
    {
        close_conn(pcb);
    }

    return ERR_OK;
}

//*****************************************************************************
//
// Callback function for tcp_poll when the connection is idle.
//
//*****************************************************************************
static err_t
echo_poll(void *arg, struct tcp_pcb *tpcb)
{
    LWIP_UNUSED_ARG(arg);

    //
    // Each time the echo_poll is entered it means the connection
    // has been idle for 5 seconds.
    //
    g_ui32tcpPollTick++;

    //
    // Print a message indicating the duration the connection has been idle.
    //
    UARTprintf("\rConnection has been idle for %4d seconds.",
                                                       g_ui32tcpPollTick*5);

    return ERR_OK;
}

//*****************************************************************************
//
// Callback function for tcp_accept when a new connection arrives.
//
//*****************************************************************************
static err_t
echo_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(err);

    tcp_setprio(pcb, TCP_PRIO_MIN);

    //
    // Sets the callback function - echo_recv that will be called when new
    // data arrives on the connection associated with PCB.  The callback
    // function will be passed a NULL pbuf to indicate that the remote host
    // has closed the connection.
    //
    tcp_recv(pcb, echo_recv);

    //
    // Error callback function currently not implemented.
    //
    tcp_err(pcb, NULL);

    //
    // Specifies the polling interval and the callback function that should be
    // called to poll the application.  The interval is specified in number of
    // TCP coarse grained timer shots, which occurs twice a second. An interval
    // of 10 means that the application would be polled every 5 seconds.
    //
    tcp_poll(pcb, echo_poll, 10);

    return ERR_OK;
}

//*****************************************************************************
//
// Create a new TCP connection on telenet port 23 and bind it any IP addressed
// acquired by the DHCP server.
//
//*****************************************************************************
void
echo_init(void)
{
    struct tcp_pcb *tcp_pcb;

    //
    // Creates a new TCP connection identifier (PCB).
    //
    tcp_pcb = tcp_new();

    //
    // Bind the PCB to all local IP addresses at port 23.
    //
    tcp_bind(tcp_pcb, IP_ADDR_ANY, 23);

    //
    // Start listening for incoming connections.  tcp_listen()
    // returns a new connection identifier and the one passed as
    // an argument to the function will be deallocated.
    //
    tcp_pcb = tcp_listen(tcp_pcb);

    //
    // Specify the callback function - echo_accept that should be
    // called when a new connection arrives for a listening PCB.
    //
    tcp_accept(tcp_pcb, echo_accept);
}

//*****************************************************************************
//
// This example demonstrates the use of the Ethernet Controller.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32User0, ui32User1;
    uint8_t pui8MACArray[8];

    //
    // Make sure the main oscillator is enabled because this is required by
    // the PHY.  The system must have a 25MHz crystal attached to the OSC
    // pins. The SYSCTL_MOSC_HIGHFREQ parameter is used when the crystal
    // frequency is 10MHz or higher.
    //
    MAP_SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

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
    // Configure the device pins.
    //
    PinoutSet(true, false);

    //
    // Initialize the UART and write initial status.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
    UARTprintf("Ethernet lwIP TCP echo example.\n\n");

    //
    // Configure Port N1 for as an output for the animation LED.
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

    //
    // Initialize LED to OFF (0).
    //
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, ~GPIO_PIN_1);

    //
    // Configure SysTick for a periodic interrupt.
    //
    MAP_SysTickPeriodSet(g_ui32SysClock / SYSTICKHZ);
    MAP_SysTickEnable();
    MAP_SysTickIntEnable();

    //
    // Configure the hardware MAC address for Ethernet Controller filtering of
    // incoming packets.  The MAC address will be stored in the non-volatile
    // USER0 and USER1 registers.
    //
    MAP_FlashUserGet(&ui32User0, &ui32User1);
    if((ui32User0 == 0xffffffff) || (ui32User1 == 0xffffffff))
    {
        //
        // We should never get here.  This is an error if the MAC address has
        // not been programmed into the device.  Exit the program.
        // Let the user know there is no MAC address.
        //
        UARTprintf("No MAC programmed!\n");
        while(1)
        {
        }
    }

    //
    // Tell the user what we are waiting for.
    //
    UARTprintf("Waiting for IP.\n");

    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
    // address needed to program the hardware registers, then program the MAC
    // address into the Ethernet Controller registers.
    //
    pui8MACArray[0] = ((ui32User0 >>  0) & 0xff);
    pui8MACArray[1] = ((ui32User0 >>  8) & 0xff);
    pui8MACArray[2] = ((ui32User0 >> 16) & 0xff);
    pui8MACArray[3] = ((ui32User1 >>  0) & 0xff);
    pui8MACArray[4] = ((ui32User1 >>  8) & 0xff);
    pui8MACArray[5] = ((ui32User1 >> 16) & 0xff);

    //
    // Initialize the lwIP library, using DHCP.
    //
    lwIPInit(g_ui32SysClock, pui8MACArray, 0, 0, 0, IPADDR_USE_DHCP);

    //
    // Initialize the Echo Server.
    //
    echo_init();

    //
    // Set the interrupt priorities.  We set the SysTick interrupt to a higher
    // priority than the Ethernet interrupt to ensure that the file system
    // tick is processed if SysTick occurs while the Ethernet handler is being
    // processed.  This is very likely since all the TCP/IP and HTTP work is
    // done in the context of the Ethernet interrupt.
    //
    MAP_IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
    MAP_IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);

    //
    // Loop forever, processing the LED blinking.  All the work is done in
    // interrupt handlers.
    //
    while(1)
    {
        //
        // Wait till the SysTick Interrupt indicates to change the state of the
        // LED.
        //
        while(g_bLED == false)
        {
        }

        //
        // Clear the flag.
        //
        g_bLED = false;

        //
        // Toggle the LED.
        //
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,
                         (MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_1) ^
                          GPIO_PIN_1));
    }
}
