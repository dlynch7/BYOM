#include "drv8323rs.h"

#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_pwm.h"
#include "inc/hw_ssi.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//#define EQUIV_TIMER_PERIOD (PWM_PERIOD) // timer load value that results in PWM_PERIOD

//********************
//                   *
// Private functions *
//                   *
//********************
// create a 16-bit PWM module using Timer 3 CCP1 on PB3 (drives DRV8323RS - INHB):
static void init_timer_pwm(void) // based on tivaware/examples/peripherals/timer/pwm.c
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

    // Set the Timer3B load value to PWM_PERIOD
    TimerLoadSet(TIMER3_BASE, TIMER_B, PWM_PERIOD-1); // don't forget to -1 in the 'load' register!

    // Set the Timer3B match value to load value / 3.
    TimerMatchSet(TIMER3_BASE, TIMER_B, TimerLoadGet(TIMER3_BASE, TIMER_B) / 3);

    // Enable Timer3B
    TimerEnable(TIMER3_BASE, TIMER_B);

    // Display startup notification on serial console
    UARTprintf("PWM (via Timer3B) initialized.\n");
}

// initialize PWM0 module on PC5 (drives DRV8323RS - INHC):
static void init_PWM0(void)
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

    // Display startup notification on serial console
    UARTprintf("PWM0 module initialized.\n");
}

// initialize PWM1 module on PF2 (drives DRV8323RS - INHA):
static void init_PWM1(void)
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

    // Display startup notification on serial console
    UARTprintf("PWM1 module initialized.\n");
}

//*******************
//                  *
// Public functions *
//                  *
//*******************

// initialize PE4 as a digital output pin to drive the DRV8323RS ENABLE input:
void init_drv8323rs_enable(void)
{
    UARTprintf("Initializing DRV8323RS Enable pin...\n");

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // enable the GPIO port used for DRV8323RS-enable
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)); // check if peripheral access enabled
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, DRV8323RS_ENABLE_PIN); // enable pin PE4
    GPIOPinWrite(GPIO_PORTE_BASE, DRV8323RS_ENABLE_PIN, 0); // initialize PE4 to LOW

    // Display startup notification on serial console
    UARTprintf("...DRV8323RS Enable pin initialized.\n");
}

// initialize a SPI module to communicate with the DRV8323RS:
void init_drv8323rs_SPI(void)
{
    // pin map (Tiva --> BOOSTXL-DRV8323RS)
    // - PA3 --> nSCS
    // - PB4 --> SCLK
    // - PB7 --> SDI (MOSI)
    // - PB6 --> SDO (MISO)
    // We will use SSI2 module (PB[7:4]) for SCLK, SDI, and SDO and use PA3 as a manual CS pin

    UARTprintf("Initializing DRV8323RS SPI communication...\n");

    // Set up PA3 as a manual CS pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // enable the GPIO port used for DRV8323RS-nSCS
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)); // check if peripheral access enabled
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, DRV8323RS_NSCS_PIN); // enable pin PA3
    GPIOPinWrite(GPIO_PORTA_BASE, DRV8323RS_NSCS_PIN, DRV8323RS_NSCS_PIN); // initialize PA3 to HIGH
    UARTprintf("\t...SPI: nSCS pin enabled.\n");

    // Set up SSI2 in master Freescale (SPI) mode.
    // - we do not want to use SSI2Fss, we want to use our manual nSCS on PA3 instead.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // use SSI2 on PB[7:4]
    GPIOPinConfigure(GPIO_PB4_SSI2CLK); // SCLK
    GPIOPinConfigure(GPIO_PB6_SSI2RX);  // MISO/SDO
    GPIOPinConfigure(GPIO_PB7_SSI2TX);  // MOSI/SDI
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);

    // Configure SSI2 for Freescale SPI - see DRV8323.pdf in documentation/ for details
    SSIConfigSetExpClk(SSI2_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_1,
                       SSI_MODE_MASTER,DRV8323RS_SPI_CLK_FREQ,DRV8323RS_SPI_WORD_LEN);
    SSIEnable(SSI2_BASE); // enable SSI2 module. Make sure nSCS has already been set.
    UARTprintf("\t...SPI: SSI2 base enabled.\n");

    // Display startup notification on serial console:
    UARTprintf("...DRV8323RS SPI communication initialized.\n");
}

// initialize all 3 PWM modules
void init_all_PWMs(void)
{
    // Display startup notification on serial console
    UARTprintf("Initializing PWM modules...\n");
    init_timer_pwm();
    init_PWM0();
    init_PWM1();

    set_timer_pwm_dc(20);
    set_pwm0_dc(20);
    set_pwm1_dc(20);
    UARTprintf("...All PWM modules have been initialized.\n");
}

// update duty cycle of the timer-generated PWM module:
void set_timer_pwm_dc(uint8_t duty_cycle_pc) // 0 <= duty_cycle_pc <= 100
{
    uint32_t load = TimerLoadGet(TIMER3_BASE, TIMER_B);
    uint32_t match = ((uint32_t) load - (load*duty_cycle_pc)/100);
    TimerMatchSet(TIMER3_BASE, TIMER_B, match);
}

// update duty cycle of the timer-generated PWM module:
void set_pwm0_dc(uint8_t duty_cycle_pc) // 0 <= duty_cycle_pc <= 100
{
    uint32_t width = ((uint32_t) (PWM_PERIOD*duty_cycle_pc)/100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, width);
}

// update duty cycle of the timer-generated PWM module:
void set_pwm1_dc(uint8_t duty_cycle_pc) // 0 <= duty_cycle_pc <= 100
{
    uint32_t width = ((uint32_t) (PWM_PERIOD*duty_cycle_pc)/100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, width);
}

// SPI write to DRV8323RS:
void drv8323rs_spi_write(uint8_t address, uint16_t data)
{
    uint32_t ui32RxData = 0; // initialization
    uint32_t *pui32RxData = &ui32RxData;

    uint32_t ui32TxData = (((1 << 15) | (address << 11)) | (data & 0x7FF));
    GPIOPinWrite(GPIO_PORTA_BASE, DRV8323RS_NSCS_PIN, 0);   // pull CS LOW
    SysCtlDelay(50);                                        // delay before sending data
    SSIDataPut(SSI2_BASE, ui32TxData);                      // send data
    while(SSIBusy(SSI2_BASE)){;}                            // wait until data sent
    SysCtlDelay(50);                                        // delay after sending data
    GPIOPinWrite(GPIO_PORTA_BASE, DRV8323RS_NSCS_PIN, DRV8323RS_NSCS_PIN);   // pull CS HIGH
    SSIDataGet(SSI2_BASE, pui32RxData);                     // empty RX buffer
    SysCtlDelay(50);                                        // delay before next SPI operation?
}

// SPI read from DRV8323RS:
uint16_t drv8323rs_spi_read(uint8_t address)
{
    // Check that address is valid:
    if(address > 0x06) return(0); // valid register addresses are [0x06:0x00]

    uint32_t ui32RxData = 0; // initialization
    uint32_t *pui32RxData = &ui32RxData;

    uint32_t ui32TxData = (((0 << 15) | (address << 11)));
    GPIOPinWrite(GPIO_PORTA_BASE, DRV8323RS_NSCS_PIN, 0);   // pull CS LOW
    SysCtlDelay(50);                                        // delay before sending data
    SSIDataPut(SSI2_BASE, ui32TxData);                      // send data
    while(SSIBusy(SSI2_BASE)){;}                            // wait until data sent
    SysCtlDelay(50);                                        // delay after sending data
    GPIOPinWrite(GPIO_PORTA_BASE, DRV8323RS_NSCS_PIN, DRV8323RS_NSCS_PIN);   // pull CS HIGH
    SSIDataGet(SSI2_BASE, pui32RxData);                     // empty RX buffer
    SysCtlDelay(50);                                        // delay before next SPI operation?

    return ((uint16_t) (ui32RxData & 0x7FF));
}

// Configure DRV8323RS PWM mode:
void config_drv8323rs_pwm(uint16_t pwm_mode)
{
    // Check that pwm_mode is valid:
    if((pwm_mode == PWM_INDEPENDENT_MODE) || (pwm_mode == PWM_1X_MODE) ||
           (pwm_mode == PWM_3X_MODE) || (pwm_mode == PWM_6X_MODE))
    {
        drv8323rs_spi_write(DRIVER_CONTROL_REG,pwm_mode);
        UARTprintf("Configured DRV8323RS to PWM mode %d.\n",pwm_mode >> PWM_MODE_FLD_LSB);
    }
    else
    {
        UARTprintf("Error: invalid PWM mode %d.\n",pwm_mode >> PWM_MODE_FLD_LSB);
    }
}
