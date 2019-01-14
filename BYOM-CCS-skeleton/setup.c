
#include "setup.h"

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

//Setup LED on port F
void LED_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // enable the GPIO port used for the on-board LED
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); // check if peripheral access enabled
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED); // enable GPIO pin for red LED (PF1) as ouput
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_BLUE); // enable GPIO pin for blue LED (PF2) as ouput
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_GREEN); // enable GPIO pin for green LED (PF3) as ouput
    GPIOPinWrite(GPIO_PORTF_BASE, LED_RED, 0x0); // RED LED OFF
    GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE, 0x0); // BLUE LED OFF
    GPIOPinWrite(GPIO_PORTF_BASE, LED_GREEN, 0x0); // GREEN LED OFF
}

// Set up PB0 (unused by DRV8323RS) as a digital output, for visualizing ISR timing:
void timer_viz_pin_init(void){
    // - enable GPIO port B
    // - check if peripheral access enabled
    // - enable pin PB0
    // - initialize PB0 to LOW
    UARTprintf("\t\t...PB0 initialized as a digital output.\n");
}
