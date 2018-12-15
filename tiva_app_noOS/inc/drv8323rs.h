#ifndef DRV8323RS_H
#define DRV8323RS_H

#include <stdint.h>

#define DRV8323RS_ENABLE_PIN    GPIO_PIN_4 // DRV8323RS ENABLE is PE4 on Tiva

// SPI-related macros
#define DRV8323RS_NSCS_PIN      GPIO_PIN_3  // nSCS is PA3 on Tiva
#define DRV8323RS_SPI_CLK_FREQ  1000000     // 1 MHz (max 10 MHz)
#define DRV8323RS_SPI_WORD_LEN  16          // words are 16 bits long


#define PWM_PERIOD 400 // f_pwm = fsys/PWM_PERIOD = 16M/PWM_PERIOD. f_pwm < 200 kHz

// initialize a digital output pin to drive the DRV8323RS ENABLE input:
void init_drv8323rs_enable(void);

// initialize a SPI module to communicate with the DRV8323RS:
void init_drv8323rs_SPI(void);

// initialize all 3 PWM modules:
void init_all_PWMs(void);

// set PWM duty cycles:
void set_timer_pwm_dc(uint8_t); // accepts duty cycle from 0 to 100 and sets timer 'match' register
void set_pwm0_dc(uint8_t); // accepts duty cycle from 0 to 100 and sets PWM0 'width' register
void set_pwm1_dc(uint8_t); // accepts duty cycle from 0 to 100 and sets PWM1 'width' register

#endif
