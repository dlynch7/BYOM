// Functions for commutating a brushless motor.

#ifndef BLDC_H
#define BLDC_H

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#include <bldc.h>
#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_uart.h>
#include <driverlib/adc.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/pwm.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <driverlib/pin_map.h>
#include <drv8323rs.h>
#include <utils/uartstdio.h>


 // PWM Frequency = fsys/PWM_PERIOD = 16M/PWM_PERIOD. f_pwm < 200 kHz
#define PWM_PERIOD 400

  // The duty cycle to use, as a percentage
#define DUTY_CYCLE 10


// Mnemonic phase names PHASE_A, PHASE_B, PHASE_C, PHASE_NONE
// - PHASE_NONE is not needed, but there for potential error handling.
typedef enum {PHASE_A = 0, PHASE_B = 1, PHASE_C = 2, PHASE_NONE = 3} phase;


// Globals
volatile uint8_t HallState;
volatile phase HighPhase;


// Set up peripherals required to interface with DRV8323RS driver:
// - 3 PWM modules
// - ## GPIO pins, configured as outputs (Tiva --> DRV8323RS)
// - 3 GPIO pins, configured as inputs (DRV8323RS --> Tiva) with interrupts (for Hall sensors)
void MotorSetup(void);

// Perform commutation
void MotorCommutate();

// Get's the current on the actively PWM'd phase
uint16_t GetCurrent(void);

// Hall sensor input capture interrupt handlers:
void HallAIntHandler(void);
void HallBIntHandler(void);
void HallCIntHandler(void);

#endif
