// Port of bldc.h in NU Mechatronics textbook from PIC32 to Tivaware.
// Uses TI DRV8323RS gate driver instead of STM L6234 driver.

#ifndef BLDC_H
#define BLDC_H

#include <stdint.h>

// A convenient new type to use the mnemonic names PHASE_A, PHASE_B, PHASE_C, PHASE_NONE
// - PHASE_NONE is not needed, but there for potential error handling.
typedef enum {PHASE_A = 0, PHASE_B = 1, PHASE_C = 2, PHASE_NONE = 3} phase;

volatile uint8_t HallState;
volatile phase HighPhase;

// Set up peripherals required to interface with DRV8323RS driver:
// - 3 PWM modules
// - ## GPIO pins, configured as outputs (Tiva --> DRV8323RS)
// - 3 GPIO pins, configured as inputs (DRV8323RS --> Tiva) with interrupts (for Hall sensors)
void bldc_setup(void);

// Perform commutation, given the PWM percentage and the present Hall state:
void bldc_commutate(int16_t pwm, uint8_t state);

// print the phase currents to the serial console:
void print_phase_currents(void);

// print the hall state to the serial console:
void print_hall_state(void);

// Get's the current on the actively PWM'd phase
uint16_t get_current(void);

// Hall sensor input capture interrupt handlers:
void HallAIntHandler(void);
void HallBIntHandler(void);
void HallCIntHandler(void);

#endif
