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
    // Assumes InitConsole() has already been called, i.e. UARTprintf() can be called
    UARTprintf("Beginning BLDC setup...\n");

    // 1) Set up 3 digital output pins as half-bridge enable pins
    // - these 3 pins go to the DRV8323RS INLx pins (PF3-->INLA, PC4-->INLB, PC6-->INLC)
    // - each pin enables one half-bridge of the DRV8323RS
    // - default to 0 (disabled)

    // 2) Set up 3 PWM modules
    // - each output pin corresponds to INHx pins on DRV8323 (PF2-->INHA,PB3-->INHB,PC5-->INHC)
    // - PWM1 module, output 6 (PF2) --> INHA
    // - Timer 3 CCP 1 (PB3) --> INHB (we must use a general purpose timer as a third PWM module)
    // - PWM0 module, output 7 (PC5) --> INHC

    // 3) set up 3 analog inputs - current sense

    // 4) Set up 1 digital output pin to enable/disable the DRV8323RS and default to 1 (enabled)

    // 5) set up SPI module - required to configure DRV8323RS

    // 6) Configure DRV8323RS in 3x PWM mode
    // - INHx pins receive PWM signals
    // - INLx pins function as 'enable' pins
    // - Use an SPI read to make sure the setup worked (log to the console)

    // 7) set up digital input pins - input capture interrupts for Hall sensors

    UARTprintf("...BLDC setup is complete.\n");
}


// set/clear the DRV8323RS phase EN pins - in 3x PWM mode, these are INL{A:C}
static void set_enable_phases(uint8_t floating_phase)
{
    switch(floating_phase)
    {
        case 0: // phase A
        {
            // disable phase A, enable phase B, and enable phase C
            break;
        }
        case 1: // phase B
        {
            // enable phase A, disable phase B, and enable phase C
            break;
        }
        case 2: // phase C
        {
            // enable phase A, enable phase B, and disable phase C
            break;
        }
        default: // received a bad 'floating_phase' input
        {
            UARTprintf("Error: 0x%02X is an unknown floating_phase.\n",floating_phase);
            break;
        }
    }
}

// set a particular PWM module to a particular duty cycle:
// I left this code in place because it is a very convenient abstraction layer
// that allows the calling function to not care how each PWM signal is generated
static void set_pulse_width(uint8_t pwm_module, uint8_t duty_cycle)
{
    switch (pwm_module)
    {
        case 0: // phase A?
        {
            set_pwm1_dc(duty_cycle);
            break;
        }
        case 1: // phase B?
        {
            set_timer_pwm_dc(duty_cycle);
            break;
        }
        case 2: // phase C?
        {
            set_pwm0_dc(duty_cycle);
            break;
        }
        default:
        {
            UARTprintf("Error: 0x%02X is an unknown pwm_module value.\n",pwm_module);
            break;
        }
    }
}

// Performs the actual commutation: one phase is PWMed, one is grounded, and the third floats.
// Partly commented out because you can find it in Ch. 29 of the NU Mechatronics textbook.
static void phases_set(int16_t pwm, phase p1, phase p2)
{
    // given 2 energized phases, find the floating phase via lookup table 'floating'
    // - if p1 and p2 are the energized phases, then floating[p1][p2] gives the floating phase.
    // - NOTE: p1 should not equal p2 (that would be an error - use assert()?),
    //   so the diagonal entries in the 2d matrix are the bogus PHASE_NONE
    static phase floating[3][3] =   {{PHASE_NONE,   PHASE_C,    PHASE_B},
                                     {PHASE_C,      PHASE_NONE, PHASE_A},
                                     {PHASE_B,      PHASE_A,    PHASE_NONE}};

    phase pfloat = floating[p1][p2];    // the floating phase
    phase phigh, plow;                  // phigh is the PWMed phase, plow is the grounded phase
    int apwm;                           // magnitude ofthe pwm count

    // choose the appropriate direction
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

    // Record the PWM'd phase for current demuxing
    HighPhase = phigh;

    // Set/clear the three ENABLE pins (INLx) according to elow_bits[pfloat]
    // - uses an abstraction layer defined in drv8323rs.{c,h}
    set_enable_phases(pfloat); // definined in bldc.{c,h}

    // Set the PWMs appropriately
    // 1) floating phase has 0% duty cycle
    // 2) low phase also has 0% duty cycle
    // 3) high phase gets the actual duty cycle
}

// Perform commutation, given the PWM percentage and the present Hall state
// Mostly commented out because you can find it in Ch. 29 of the NU Mechatronics textbook.
void bldc_commutate(int16_t pwm, uint8_t state)
{
    // implement the commutation table:
    switch(state)
    {
        case 0x4: // 0x4, 0b100
        {
            // phases_set(pwm, ??, ??);
            break;
        }
        case 0x6: // 0x6, 0b110
        {
            // phases_set(pwm, ??, ??);
            break;
        }
        case 0x2: // 0x2, 0b010
        {
            // phases_set(pwm, ??, ??);
            break;
        }
        case 0x3: // 0x3, 0b011
        {
            // phases_set(pwm, ??, ??);
            break;
        }
        case 0x1: // 0x1, 0b001
        {
            // phases_set(pwm, ??, ??);
            break;
        }
        case 0x5: // 0x5, 0b101
        {
            // phases_set(pwm, ??, ??);
            break;
        }
        default:
        {
            // print an error msg to the serial console
            UARTprintf("Error: 0x%02X is an unknown commutation state.\n",state);
            break;
        }
    }
}


// print the phase currents to the serial console:
void print_phase_currents(void)
{
    uint32_t phase_curr_arr[4] = {0};
    read_ISEN_ABC(phase_curr_arr);
    UARTprintf("[ADC] A: %d, B: %d, C: %d.\n",
        phase_curr_arr[2],phase_curr_arr[1],phase_curr_arr[0]);
}



// Gets the current from the phase that is being PWM'd
uint16_t get_current(void)
{
    uint32_t phase_curr_arr[4] = {0};
    read_ISEN_ABC(phase_curr_arr);

    switch(HighPhase) {
        case PHASE_A:
            return phase_curr_arr[2] & 0x0FFF;
        case PHASE_B:
            return phase_curr_arr[1] & 0x0FFF;
        case PHASE_C:
            return phase_curr_arr[0] & 0x0FFF;
        default:
            return 0;
    }
}


// print the hall state to the serial console:
void print_hall_state(void)
{
    uint8_t temp_hall = HallState; // HallState is a global variable declared in bldc.h

    UARTprintf("H[A]: %01X, H[B]: %01X, H[C]: %01X.\n",
        temp_hall & 0x01, (temp_hall & 0x02) >> 1, (temp_hall & 0x04) >> 2);
}

// Hall sensor input capture interrupt handlers:
void HallAIntHandler(void)
{
    // clear interrupt flag

    // update Hall states

    // commutate
}

void HallBIntHandler(void)
{
    // clear interrupt flag

    // update Hall states

    // commutate
}

void HallCIntHandler(void)
{
    // clear interrupt flag

    // update Hall states

    // commutate
}
