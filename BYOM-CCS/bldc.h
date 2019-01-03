// Port of bldc.h in NU Mechatronics textbook from PIC32 to Tivaware.
// Uses TI DRV8323RS gate driver instead of STM L6234 driver.

#ifndef BLDC_H
#define BLDC_H

#include <stdint.h>

volatile uint8_t HallState;

// Set up peripherals required to interface with DRV8323RS driver:
// - 3 PWM modules
// - ## GPIO pins, configured as outputs (Tiva --> DRV8323RS)
// - 3 GPIO pins, configured as inputs (DRV8323RS --> Tiva) with interrupts (for Hall sensors)
void bldc_setup(void);

// Perform commutation, given the PWM percentage and the present Hall state:
void bldc_commutate(int16_t pwm, uint8_t state);

// Prompt the user for a signed PWM percentage:
int16_t bldc_get_pwm(void);

// print the phase currents to the serial console:
void print_phase_currents(void);

// print the hall state to the serial console:
void print_hall_state(void);

// start the motor
void start_motor(void);

// Hall sensor input capture interrupt handlers:
void HallAIntHandler(void);
void HallBIntHandler(void);
void HallCIntHandler(void);

#endif
