// Functions and pin definitions to communicate with the DRV8323RS motor driver on the TI BOOSTXL dev board.

#ifndef DRV8323RS_H
#define DRV8323RS_H

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

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
#include <utils/uartstdio.h>

// Device enable constants
#define DRV8323RS_ENABLE_PERIPH SYSCTL_PERIPH_GPIOE
#define DRV8323RS_ENABLE_PORT   GPIO_PORTE_BASE
#define DRV8323RS_ENABLE_PIN    GPIO_PIN_4

// SPI communication constants
#define DRV8323RS_SSI_PERIPH            SYSCTL_PERIPH_SSI2
#define DRV8323RS_SSI_BASE              SSI2_BASE
#define DRV8323RS_SSI_GPIO_PERIPH       SYSCTL_PERIPH_GPIOB
#define DRV8323RS_SSI_PORT              GPIO_PORTB_BASE
#define DRV8323RS_SSI_SCLK_PIN          GPIO_PIN_4
#define DRV8323RS_SSI_RX_PIN            GPIO_PIN_6
#define DRV8323RS_SSI_TX_PIN            GPIO_PIN_7

#define DRV8323RS_SSI_SCLK_PIN_CONFIG   GPIO_PB4_SSI2CLK
#define DRV8323RS_SSI_RX_PIN_CONFIG     GPIO_PB6_SSI2RX
#define DRV8323RS_SSI_TX_PIN_CONFIG     GPIO_PB7_SSI2TX

#define DRV8323RS_NSCS_PERIPH   SYSCTL_PERIPH_GPIOA
#define DRV8323RS_NSCS_PORT     GPIO_PORTA_BASE
#define DRV8323RS_NSCS_PIN      GPIO_PIN_3
#define DRV8323RS_SPI_CLK_FREQ  1000000
#define DRV8323RS_SPI_WORD_LEN  16

// Device control pin constants
#define DRV8323RS_INLA_PERIPH     SYSCTL_PERIPH_GPIOF
#define DRV8323RS_INLA_PORT       GPIO_PORTF_BASE
#define DRV8323RS_INLA_PIN        GPIO_PIN_3

#define DRV8323RS_INLB_PERIPH     SYSCTL_PERIPH_GPIOC
#define DRV8323RS_INLB_PORT       GPIO_PORTC_BASE
#define DRV8323RS_INLB_PIN        GPIO_PIN_4

#define DRV8323RS_INLC_PERIPH     SYSCTL_PERIPH_GPIOC
#define DRV8323RS_INLC_PORT       GPIO_PORTC_BASE
#define DRV8323RS_INLC_PIN        GPIO_PIN_6

// Hall sensor pin constants
#define DRV8323RS_HALLA_PERIPH    SYSCTL_PERIPH_GPIOB
#define DRV8323RS_HALLA_PORT      GPIO_PORTB_BASE
#define DRV8323RS_HALLA_PIN       GPIO_PIN_2
#define DRV8323RS_HALLA_INT       INT_GPIOB

#define DRV8323RS_HALLB_PERIPH    SYSCTL_PERIPH_GPIOE
#define DRV8323RS_HALLB_PORT      GPIO_PORTE_BASE
#define DRV8323RS_HALLB_PIN       GPIO_PIN_0
#define DRV8323RS_HALLB_INT       INT_GPIOE

#define DRV8323RS_HALLC_PERIPH    SYSCTL_PERIPH_GPIOA
#define DRV8323RS_HALLC_PORT      GPIO_PORTA_BASE
#define DRV8323RS_HALLC_PIN       GPIO_PIN_4
#define DRV8323RS_HALLC_INT       INT_GPIOA

// PWM pin constants
#define DRV8323RS_PWMA_GPIO_PERIPH    SYSCTL_PERIPH_GPIOF
#define DRV8323RS_PWMA_GPIO_PORT      GPIO_PORTF_BASE
#define DRV8323RS_PWMA_GPIO_PIN       GPIO_PIN_2

#define DRV8323RS_PWMA_PERIPH         SYSCTL_PERIPH_PWM1
#define DRV8323RS_PWMA_BASE           PWM1_BASE
#define DRV8323RS_PWMA_GEN            PWM_GEN_3
#define DRV8323RS_PWMA_OUT            PWM_OUT_6
#define DRV8323RS_PWMA_OUT_BIT        PWM_OUT_6_BIT
#define DRV8323RS_PWMA_PIN_CONFIG     GPIO_PF2_M1PWM6

#define DRV8323RS_PWMB_GPIO_PERIPH    SYSCTL_PERIPH_GPIOB
#define DRV8323RS_PWMB_GPIO_PORT      GPIO_PORTB_BASE
#define DRV8323RS_PWMB_GPIO_PIN       GPIO_PIN_3

#define DRV8323RS_PWMB_PERIPH         SYSCTL_PERIPH_TIMER3
#define DRV8323RS_PWMB_BASE           TIMER3_BASE
#define DRV8323RS_PWMB_TIMER          TIMER_B
#define DRV8323RS_PWMB_PIN_CONFIG     GPIO_PB3_T3CCP1

#define DRV8323RS_PWMC_GPIO_PERIPH    SYSCTL_PERIPH_GPIOC
#define DRV8323RS_PWMC_GPIO_PORT      GPIO_PORTC_BASE
#define DRV8323RS_PWMC_GPIO_PIN       GPIO_PIN_5

#define DRV8323RS_PWMC_PERIPH         SYSCTL_PERIPH_PWM0
#define DRV8323RS_PWMC_BASE           PWM0_BASE
#define DRV8323RS_PWMC_GEN            PWM_GEN_3
#define DRV8323RS_PWMC_OUT            PWM_OUT_7
#define DRV8323RS_PWMC_OUT_BIT        PWM_OUT_7_BIT
#define DRV8323RS_PWMC_PIN_CONFIG     GPIO_PC5_M0PWM7

// Current sense pin constants
#define DRV8323RS_ISENSE_ADC_PERIPH     SYSCTL_PERIPH_ADC0
#define DRV8323RS_ISENSE_ADC_BASE       ADC0_BASE

#define DRV8323RS_ISENSEA_GPIO_PERIPH   SYSCTL_PERIPH_GPIOE
#define DRV8323RS_ISENSEA_GPIO_PORT     GPIO_PORTE_BASE
#define DRV8323RS_ISENSEA_GPIO_PIN      GPIO_PIN_2
#define DRV8323RS_ISENSEA_ADC_CTL_CH    ADC_CTL_CH1

#define DRV8323RS_ISENSEB_GPIO_PERIPH   SYSCTL_PERIPH_GPIOE
#define DRV8323RS_ISENSEB_GPIO_PORT     GPIO_PORTE_BASE
#define DRV8323RS_ISENSEB_GPIO_PIN      GPIO_PIN_1
#define DRV8323RS_ISENSEB_ADC_CTL_CH    ADC_CTL_CH2

#define DRV8323RS_ISENSEC_GPIO_PERIPH   SYSCTL_PERIPH_GPIOD
#define DRV8323RS_ISENSEC_GPIO_PORT     GPIO_PORTD_BASE
#define DRV8323RS_ISENSEC_GPIO_PIN      GPIO_PIN_3
#define DRV8323RS_ISENSEC_ADC_CTL_CH    ADC_CTL_CH4

// Register addresses
#define DRV8323RS_FAULT_STATUS_REG_1      0x00
#define DRV8323RS_FAULT_STATUS_REG_2      0x01
#define DRV8323RS_DRIVER_CONTROL_REG      0x02
#define DRV8323RS_GATE_DRIVE_HS_REG       0x03
#define DRV8323RS_GATE_DRIVE_LS_REG       0x04
#define DRV8323RS_OCP_CONTROL_REG         0x05
#define DRV8323RS_CSA_CONTROL_REG         0x06

// Initializes DRV8323RS SPI communication.
void InitDRV8323RS(void);

// Writes a word to an address on the DRV8323RS.
// @param address: The address to write to
// @param data: The data to write
void SPIWriteDRV8323(uint8_t address, uint16_t data);

// Reads a word from an address on the DRV8323RS.
// @param address: The address to read from
// @return: The data at the specified address
uint16_t SPIReadDRV8323(uint8_t address);

#endif
