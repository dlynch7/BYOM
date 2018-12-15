#include "drv8323rs.h"

#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_pwm.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#define EQUIV_TIMER_PERIOD (PWM_PERIOD) // timer load value that results in PWM_PERIOD

// create a 16-bit PWM module using Timer 3 CCP 1:
void init_timer_pwm(void) // based on tivaware/examples/peripherals/timer/pwm.c
{
    // Enable the Timer3 peripheral.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    // Use T3CCP1 with port B pin 3. Start by enabling port B.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure GPIO pin muxing for the Timer/CCP function.
    GPIOPinConfigure(GPIO_PB3_T3CCP1);

    // Assume InitConsole() has already been called by main().

    // Configure the ccp settings for CCP pin.
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Configure Timer3B as a 16-bit periodic timer.
    TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM);

    // Set the Timer3B load value to EQUIV_TIMER_PERIOD
    TimerLoadSet(TIMER3_BASE, TIMER_B, EQUIV_TIMER_PERIOD);

    // Set the Timer3B match value to load value / 3.
    TimerMatchSet(TIMER3_BASE, TIMER_B, TimerLoadGet(TIMER3_BASE, TIMER_B) / 3);

    // Enable Timer3B
    TimerEnable(TIMER3_BASE, TIMER_B);

    UARTprintf("Timer 3B PWM enabled.\n");
}

// update duty cycle of the timer-generated PWM module:
void set_timer_dc(uint8_t duty_cycle_pc)
{
    uint32_t load = TimerLoadGet(TIMER3_BASE, TIMER_B);
    uint32_t match = ((uint32_t) load - (load*duty_cycle_pc)/100);
    TimerMatchSet(TIMER3_BASE, TIMER_B, match);
}

// initialize PWM0 module:
void init_PWM0(void)
{
    // Configure PWM clock to match system.
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    // Configure pin C5 as PWM output pin.
    GPIOPinConfigure(GPIO_PC5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);

    // Configure PWM options.
    // - PWM_GEN_3 covers M0PWM6 and M0PWM7.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the period (expressed in clock ticks).
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWM_PERIOD); // PWM_PERIOD is #defined in drv8323rs.h

    // Set the PWM duty cycle to 33%.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, (PWM_PERIOD/3));

    // Enable the PWM generator.
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    // Turn on the PWM output pin.
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
}

// initialize PWM1 module:
void init_PWM1(void)
{
    // Configure PWM clock to match system.
    //SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    // Configure pin F2 as PWM output pin.
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    // Configure PWM options.
    // - PWM_GEN_3 covers M1PWM6 and M1PWM7.
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the period (expressed in clock ticks).
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PWM_PERIOD); // PWM_PERIOD is #defined in drv8323rs.h

    // Set the PWM duty cycle to 33%.
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, (PWM_PERIOD/3));

    // Enable the PWM generator.
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    // Turn on the PWM output pin.
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
}