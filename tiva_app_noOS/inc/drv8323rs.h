#ifndef DRV8323RS_H
#define DRV8323RS_H

#include <stdint.h>

#define DRV8323RS_ENABLE_PIN    GPIO_PIN_4 // DRV8323RS ENABLE is PE4 on Tiva

// Phase enable pin macros - assumes DRV8323RS is configured in 3x PWM mode
// - INLA is on PF3
#define INLA_PERIPH     SYSCTL_PERIPH_GPIOF
#define INLA_PORT       GPIO_PORTA_BASE
#define INLA_PIN        GPIO_PIN_3

// - INLB is on PC4
#define INLB_PERIPH     SYSCTL_PERIPH_GPIOC
#define INLB_PORT       GPIO_PORTC_BASE
#define INLB_PIN        GPIO_PIN_4

// - INLC is on PC6
#define INLC_PERIPH     SYSCTL_PERIPH_GPIOC
#define INLC_PORT       GPIO_PORTC_BASE
#define INLC_PIN        GPIO_PIN_6

// Hall sensor macros
#define HALLA_PERIPH    SYSCTL_PERIPH_GPIOB
#define HALLA_PORT      GPIO_PORTB_BASE
#define HALLA_PIN       GPIO_PIN_2
#define HALLA_INT       INT_GPIOB

#define HALLB_PERIPH    SYSCTL_PERIPH_GPIOE
#define HALLB_PORT      GPIO_PORTE_BASE
#define HALLB_PIN       GPIO_PIN_0
#define HALLB_INT       INT_GPIOE

#define HALLC_PERIPH    SYSCTL_PERIPH_GPIOA
#define HALLC_PORT      GPIO_PORTA_BASE
#define HALLC_PIN       GPIO_PIN_4
#define HALLC_INT       INT_GPIOA

// ADC macros
#define ISENA_ADC 0x01
#define ISENB_ADC 0x02
#define ISENC_ADC 0x03

// SPI-related macros
#define DRV8323RS_NSCS_PIN      GPIO_PIN_3  // nSCS is PA3 on Tiva
#define DRV8323RS_SPI_CLK_FREQ  1000000     // 1 MHz (max 10 MHz)
#define DRV8323RS_SPI_WORD_LEN  16          // words are 16 bits long

// Driver Control Fields (Driver Control Register has address = 0x02)
#define DRIVER_CONTROL_REG      0x02

//#define CLR_FLT                 0x01
//#define BRAKE                   0x02
//#define COAST                   0x04
//#define ONE_PWM_DIR             0x08
//#define ONE_PWM_COM             0x10

#define PWM_MODE_FLD_LSB        0x05
#define PWM_MODE_FLD_MSB        0x06
#define PWM_INDEPENDENT_MODE    0x60
#define PWM_1X_MODE             0x40
#define PWM_3X_MODE             0x20
#define PWM_6X_MODE             0x00
#define PWM_PERIOD 400 // f_pwm = fsys/PWM_PERIOD = 16M/PWM_PERIOD. f_pwm < 200 kHz

// initialize a digital output pin to drive the DRV8323RS ENABLE input:
void init_drv8323rs_enable(void);

// initialize a SPI module to communicate with the DRV8323RS:
void init_drv8323rs_SPI(void);

// initialize 3 GPIOs with input capture interrutps, to read DRV8323RS Hall sensors:
// - Tiva pins PB2, PE0, and PA4 map to DRV8323RS HALLA, HALLB, and HALLC, respectively.
void init_halls(void);

// initialize 3 ADC pins to read DRV8323RS ISEN{A:C} (current sense for each phase)
void init_isense_ADCs(void);

// initialize phase enable (INLx) pins
void init_phase_enable_pins(void);

// initialize all 3 PWM modules:
// - PWM output on pins PF2 (INHA), PB3 (INHB), and PC5 (INHC)
void init_all_PWMs(void);

// set PWM duty cycles:
void set_timer_pwm_dc(uint8_t); // accepts duty cycle from 0 to 100 and sets timer 'match' register
void set_pwm0_dc(uint8_t); // accepts duty cycle from 0 to 100 and sets PWM0 'width' register
void set_pwm1_dc(uint8_t); // accepts duty cycle from 0 to 100 and sets PWM1 'width' register

// Configure DRV8323RS PWM mode:
void config_drv8323rs_pwm(uint16_t pwm_mode);

// SPI write to DRV8323RS:
void drv8323rs_spi_write(uint8_t address, uint16_t data);

// SPI read from DRV8323RS:
uint16_t drv8323rs_spi_read(uint8_t address);

// ISENSE ADC read functions:
void read_ISEN_ABC(uint32_t *phase_curr_arr); // phase_curr_arr must be length 4

// ISENSE ADC --> currents

// read all 3 Hall sensor states:
uint8_t read_halls(void);

// print Hall states:
extern void print_hall_state();

// Hall sensor input capture interrupt handlers, defined in bldc.{c,h}:
extern void HallAIntHandler(void);
extern void HallBIntHandler(void);
extern void HallCIntHandler(void);

#endif
