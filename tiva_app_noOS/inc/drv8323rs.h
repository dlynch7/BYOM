#ifndef DRV8323RS_H
#define DRV8323RS_H

#include <stdint.h>

#define PWM_PERIOD 400 // f_pwm = fsys/PWM_PERIOD = 16M/PWM_PERIOD. f_pwm < 200 kHz

// create a 16-bit PWM module using Timer 3 CCP 1:
void init_timer_pwm(void); // based on tivaware/examples/peripherals/timer/pwm
void set_timer_dc(uint8_t); // accepts a duty cycle from 0 to 100 and sets the timer MATCH
void init_PWM0(void); // initialize PWM0 module
void init_PWM1(void); // initialize PWM1 module

#endif
