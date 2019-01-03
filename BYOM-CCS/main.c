//*************************************************************************************************
//
// main.c - controls a BLDC motor and provides a rudimentary serial user interface
//
// Begun: 12/13/18
//
// Copyright (c) 2010-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//************************************************************************************************/

#include <bldc.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <driverlib/fpu.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <inc/hw_memmap.h>
#include <utils/uartstdio.h>

// Set up UART0 to be used for a console to display info as the programs runs.
void InitConsole(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART0 so that we can configure the clock.
    GPIOPinConfigure(GPIO_PA0_U0RX); // pin muxing
    GPIOPinConfigure(GPIO_PA1_U0TX); // pin muxing
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Select the alternate (UART) function$
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC); // Use the internal 16MHz oscillator as the UART$
    UARTStdioConfig(0, 115200, SysCtlClockGet()); // Initialize the UART for console I/O.
    UARTprintf("\n\nConsole on UART0 is up.\n");
}

// Main function:
int main(void) {
    // Enable processor interrupts.
    IntMasterEnable();

    // Allow interrupt handlers to use floating-point instructions,
    // at the expense of extra stack usage:
    FPULazyStackingEnable();

    // Set the clocking to run directly from the external crystal/oscillator:
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Set up the serial console to use for displaying messages:
    InitConsole();

    // Set up the motor drive
    bldc_setup();

    UARTprintf("Setup complete!\n");

	while(1);
}
