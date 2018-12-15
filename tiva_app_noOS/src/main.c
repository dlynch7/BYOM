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

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"

#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#include "bldc.h"
#include "circ_buffer.h"
#include "drv8323rs.h"
#include "setup.h"

// Macros
#define MOTOR_CONTROL_FREQ 1000 // frequency of timer A0 interrupt (in Hz)
#define MENU_BUF_LEN 2 // length of serial I/O buffer for receiving stuff from client PC program

// Global variables
uint8_t gSensorBufferWritePermission; // permits SensorPollHandler() to write to circ buffer
uint8_t gSensorBufferReadPermission; // permits main() to read from circ buffer

//*************************************************************************************************
//
// This function sets up UART0 to be used for a console to display info as the programs runs.
//
//************************************************************************************************/
void InitConsole(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART0 so that we can configure the clock.
    GPIOPinConfigure(GPIO_PA0_U0RX); // pin muxing
    GPIOPinConfigure(GPIO_PA1_U0TX); // pin muxing
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Select the alternate (UART) function$
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC); // Use the internal 16MHz oscillator as the UART$
    UARTStdioConfig(0, 115200, SysCtlClockGet()); // Initialize the UART for console I/O.
    UARTprintf("\nConsole on UART0 is up.\n");
}

//*********************************************************************************************
//
// This function sets up timer A0 to generate interrupts
// at frequency specified by MOTOR_CONTROL_FREQ (#define'd above)
//
//*********************************************************************************************
void TimerBegin(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // Enable the peripherals used by this example.
    IntMasterEnable(); // Enable processor interrupts.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // Configure a 32-bit periodic timer.
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / MOTOR_CONTROL_FREQ);
    IntEnable(INT_TIMER0A); // Setup the interrupts for the timer timeouts.
    IntPrioritySet(INT_TIMER0A, 0x00); // set the Timer 0A interrupt priority to be "HIGH"
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A); // Enable the timer.
    UARTprintf("Timer A0 initialized at %d Hz!\n",MOTOR_CONTROL_FREQ);
}

//********************************************************************************************
//
// The interrupt handler for the periodic interrupt generated by Timer A0
//
//********************************************************************************************
void MotorControlInterruptHandler(void)
{
    // declare static local variables:
    static uint8_t  tic = 0, toc = 0, dec_count = 0;

    // clear the timer interrupt:
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // If the permission flag is set, get ready to write everything to the circular buffer:
    //if (gSensorBufferWritePermission==1)
    //{
        //GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE, LED_BLUE); // BLUE LED ON
        // if the DECIMATION counter is up, go ahead and write to the buffer:
        if (dec_count==DECIMATION) // DECIMATION is defined in circ_buffer.h
        {
            //buffer_write(...TODO: fill in args...);
            //set_timer_dc(toc);
            dec_count = 0;
            ++toc;
        }
        ++dec_count;

        if (toc >= 100) toc = 0;
    //}

    
}


// Main function:
int main(void) {
    //*****************************************************************************************
    //
    // Local variable declarations
    //
    //*****************************************************************************************
    //uint16_t i = 0; // simple counter
    //buf_out_t buf_line; // typedef struct that stores output from buffer_read()
    //buf_out_t *buf_line_ptr = &buf_line;   // store address of buf_line in buf_line_prt
    //char menu_buf[MENU_BUF_LEN]; // buffer for receiving junk from client program

    //*****************************************************************************************
    //
    // Startup
    //
    //*****************************************************************************************

    // Allow interrupt handlers to use floating-point instructions,
    // at the expense of extra stack usage:
    FPULazyStackingEnable();

    // Set the clocking to run directly from the external crystal/oscillator:
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Set up the serial console to use for displaying messages:
    InitConsole();

    // TODO: set up the interface to the DRV8323 motor driver
    bldc_setup();
    //gSensorBufferWritePermission = 0; // MotorControlInterruptHandler() not permitted to write to buffer
    //gSensorBufferReadPermission = 0;  // main() not permitted to read from buffer
    //buffer_reset();
    TimerBegin();

    UARTprintf("Setup complete!\n");

    //*****************************************************************************************
    //
    // Superloop:
    //
    //*****************************************************************************************
	while(1)
	{
        UARTprintf(".");
        UARTprintf(".");
        UARTprintf(".");
        UARTprintf("\b\b\b");
        //UARTgets(menu_buf,MENU_BUF_LEN); // wait until receive a command from client
        //buffer_reset();
        //switch (menu_buf[0])
        //{
            //case 'q': // quit (just returns to beginning of superloop)
            //    break;
            //case 'r': // send circular buffer contents to client via batch transmission
            //{
                // prompt user for PWM duty cycle
                //****int16_t newpwm = bldc_get_pwm();
                // disable interrupts
                //****pwm = newpwm;
                // enable interrupts
                //****bldc_commutate(pwm, state());
                // permit SensorPollHandler() to write to circular buffer:
                //gSensorBufferWritePermission = 1;
                // wait for read permission from SensorPollHandler():
                //while(!buffer_full());
                //gSensorBufferWritePermission = 0;
                //for (i=0; i<NSAMPLES; i++) // NSAMPLES is #defined in circ_buffer.h
                //{
                    //buffer_read(buf_line_ptr);
                    //TODO: print contents of buf_line to UART
                //}

                //gSensorBufferWritePermission = 0; // disable write permission when done (safeguard)
                //gSensorBufferReadPermission = 0;  // disable read permission when done
                //break;
            //} // end 'r' case
            //default:
            //{
                //UARTprintf("error: %c is unknown menu option\n",menu_buf[0]);
                //break;
            //}
        //} // end switch()
	} // end while(1)
    return 0; // something, somewhere, went horribly wrong.
}
