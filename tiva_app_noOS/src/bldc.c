// Port of bldc.c in NU Mechatronics textbook from PIC32 to Tivaware.
// Uses TI DRV8323RS gate driver instead of STM L6234 driver.
#include "bldc.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
// #include all the tiva things
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"

#include "inc/hw_pwm.h"
#include "driverlib/pwm.h"

#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "drv8323rs.h" // API for BOOSTXL-DRV8323RS motor driver devboard

// Set up peripherals required to interface with DRV8323RS driver and Hall sensors
// and configure DRV8323RS driver in 3x PWM mode.
// - 3 PWM modules
// - 3+1 GPIO pins, configured as outputs (Tiva --> DRV8323RS)
// - ## GPIO pins, configured as inputs (DRV8323RS --> Tiva) with interrupts (for Hall sensors)
// - ## analog inputs for current sensing
// - REQUIRES InitConsole() to be called prior, because this function uses UARTprintf()
//   to display notifications on the serial console as each module is set up.
void bldc_setup(void)
{
    UARTprintf("Beginning BLDC setup...\n");

    // Set up 1 digital output pin to enable/disable the DRV8323RS and default to 0 (disabled)
    init_drv8323rs_enable();

    // Set up 3 digital output pins as half-bridge enable pins
    // - these 3 pins go to the DRV8323RS INLx pins (PF3-->INLA, PC4-->INLB, PC6-->INLC)
    // - each pin enables one half-bridge of the DRV8323RS
    // - default to 0 (disabled)
    // TODO: abstraction layer

    // Set up 3 PWM modules
    // - each output pin corresponds to INHx pins on DRV8323 (PF2-->INHA,PB3-->INHB,PC5-->INHC)
    // - PWM1 module, output 6 (PF2) --> INHA
    // - Timer 3 CCP 1 (PB3) --> INHB (we must use a general purpose timer as a third PWM module)
    // - PWM0 module, output 7 (PC5) --> INHC
    // TODO: abstraction layer
    init_all_PWMs();

    // set up digital input pins - input capture interrupts for Hall sensors

    // set up 3 analog inputs - current sense

    // set up SPI module - required to configure DRV8323RS
    init_drv8323rs_SPI();

    // Configure DRV8323RS in 3x PWM mode
    // - INHx pins receive PWM signals
    // - INLx pins function as 'enable' pins
    config_drv8323rs_pwm(PWM_3X_MODE);

    UARTprintf("...BLDC setup is complete.\n");
}

// A convenient new type to use the mnemonic names PHASE_A, PHASE_B, PHASE_C, PHASE_NONE
// - PHASE_NONE is not needed, but there for potential error handling.
//typedef enum {PHASE_A = 0, PHASE_B = 1, PHASE_C = 2, PHASE_NONE = 3} phase;
typedef enum {PHASE_A = 0, PHASE_B = 1, PHASE_C = 2, PHASE_NONE = 3} phase;

// A 3-element array that maps 'phase' (above) to PWM_OUT_x
uint32_t phase_to_PWMOut[3] = {PWM_OUT_0, PWM_OUT_1, PWM_OUT_2};

// A 3-element array that maps 'enable' to GPIO pins
//uint32_t enable_to_GPIO[3] = {,,};

// Performs the actual commutation: one phase is PWMed, one is grounded, and the third floats.
static void phases_set(int16_t pwm, phase p1, phase p2)
{
    // an array of the addresses of the PWM duty cycle-controlling SFRs
    //static volatile unsigned int * ocr[] = {&OC1RS, &OC2RS, &OC3RS, &OC4RS, &OC5RS, &OC6RS};

    // given 2 energized phases, find the floating phase via lookup table 'floating'
    // - if p1 and p2 are the energized phases, then floating[p1][p2] gives the floating phase.
    // - NOTE: p1 should not equal p2 (that would be an error - use assert()?),
    //   so the diagonal entries in the 2d matrix are the bogus PHASE_NONE
    static phase floating[3][3] =   {{PHASE_NONE,   PHASE_C,    PHASE_B},
                                     {PHASE_C,      PHASE_NONE, PHASE_A},
                                     {PHASE_B,      PHASE_A,    PHASE_NONE}};

    // elow_bits[pfloat] takes the floating phase 'pfloat' (e.g., pfloat could be PHASE_A)
    // and returns a 3-bit value with a zero in the column corresponding to the
    // floating phase (0th column = A, 1st column = B, 2nd column = C).
    // 0b110 = 0x6, 0b101 = 0x5, 0b011 = 0x3
    static uint8_t elow_bits[3] = {0x6, 0x5, 0x3};

    phase pfloat = floating[p1][p2];    // the floating phase
    phase phigh, plow;                  // phigh is the PWMed phase, plow is the grounded phase
    int apwm;                           // magnitude ofthe pwm count

    // choose the appripriate direction
    if (pwm > 0)
    {
        phigh = p1;
        plow = p2;
        apwm = pwm;
    }
    else
    {
        phigh = p2;
        plow = p1;
        apwm = -pwm;
    }

    // Set/clear the three ENABLE pins (INLx) according to elow_bits[pfloat]
    // TODO: create & use an abstraction layer so this function can use bitwise operations
    // to set/clear the INLx pins - drv8323rs.{c,h}
    //LATE = (LATE & ~0x7) | elow_bits[pfloat]; // TODO: convert from PIC32 to Tiva

    // Set the PWMs appropriately
    // TODO: create & use an abstraction layer - drv8323rs.{c,h}
    // so this function doesn't need to know which PWM modules
    // are actually PWM modules and which are timer-based PWM.
    //set_pulse_width(pfloat,0);
    //set_pulse_width(plow,0);
    //set_pulse_width(phigh,apwm);
    PWMPulseWidthSet(PWM0_BASE,pfloat,0);   // floating pin has 0% duty cycle
    PWMPulseWidthSet(PWM0_BASE,plow,0);     // low pin also has 0% duty cycle
    PWMPulseWidthSet(PWM0_BASE,phigh,apwm); // the high phase gets the actual duty cycle
}

// Perform commutation, given the PWM percentage and the present Hall state
void bldc_commutate(int16_t pwm, uint8_t state)
{
    // convert pwm to ticks
    pwm = ((int16_t)PWM_PERIOD*pwm )/100; // TODO: #define PWM_PERIOD in drv8323rs.h

    switch(state)
    {
        case 0x4: // 0x4, 0b100
        {
            phases_set(pwm, PHASE_B, PHASE_A);  // if pwm > 0, phase A = GND and B is PWMed
            break;                              // if pwm < 0, phase B = GND and A is PWMed
        }
        case 0x6: // 0x6, 0b110
        {
            phases_set(pwm, PHASE_C, PHASE_A);
            break;
        }
        case 0x2: // 0x2, 0b010
        {
            phases_set(pwm, PHASE_C, PHASE_B);
            break;
        }
        case 0x3: // 0x3, 0b011
        {
            phases_set(pwm, PHASE_A, PHASE_B);
            break;
        }
        case 0x1: // 0x1, 0b001
        {
            phases_set(pwm, PHASE_A, PHASE_C);
            break;
        }
        case 0x5: // 0x5, 0b101
        {
            phases_set(pwm, PHASE_B, PHASE_C);
            break;
        }
        default:
        {
            // print an error msg to the serial console
            break;
        }
    }
}

// Prompt the user for a signed PWM percentage
int16_t bldc_get_pwm(void);
