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
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

// Global variables declared in this file: (none)

// Global variables declared in other files:
extern uint8_t HallState;

//*************************************************************************************************
//
// Private functions
//
//*************************************************************************************************
// create a 16-bit PWM module using Timer 3 CCP1 on PB3 (drives DRV8323RS - INHB):
static void init_timer_pwm(void) // based on tivaware/examples/peripherals/timer/pwm.c
{
    // Assume InitConsole() has already been called by main(), i.e. safe to call UARTprintf()
    UARTprintf("\t\tInitializing PWM (via Timer3B)...\n");

    // Enable the Timer3 peripheral.

    // Use T3CCP1 with port B pin 3. Start by enabling port B.

    // Configure GPIO pin muxing for the Timer/CCP function.

    // Configure the ccp settings for CCP pin.

    // Configure Timer3B as a 16-bit periodic timer.

    // Set the Timer3B load value to PWM_PERIOD
    // - don't forget to -1 in the 'load' register (only for this timer-based PWM; the PWM modules do this for you)!

    // Set the Timer3B match value to load value / 3.

    // Enable Timer3B

    // Display startup notification on serial console
    UARTprintf("\t\t...PWM (via Timer3B) initialized.\n");
}

// initialize PWM0 module on PC5 (drives DRV8323RS - INHC):
static void init_PWM0(void)
{
    // Assume InitConsole() has already been called by main(), i.e. safe to call UARTprintf()
    UARTprintf("\t\tInitializing PWM0 module...\n");

    // Configure PWM clock to match system, i.e. divisor = 1

    // Enable the peripherals used by this PWM module and output pin (GPIOC, PWM0)

    // Configure pin C5 as PWM output pin for M0PWM7

    // Configure PWM options.
    // - PWM_GEN_3 covers M0PWM6 and M0PWM7, you will use M0PWM7
    // - count down
    // - no sync

    // Set the period (expressed in clock ticks) using PWM_PERIOD (#define'd in drv8323rs.h

    // Set the pulse width to 0.

    // Enable the PWM generator.

    // Turn on the PWM output pin.

    // Display startup notification on serial console
    UARTprintf("\t\t...PWM0 module initialized.\n");
}

// initialize PWM1 module on PF2 (drives DRV8323RS - INHA):
static void init_PWM1(void)
{
    // Assume InitConsole() has already been called by main(), i.e. safe to call UARTprintf()
     UARTprintf("\t\tInitializing PWM1 module...\n");

    // Configure PWM clock to match system.

    // Enable the peripherals used by this PWM module and output pin (GPIOF and PWM1)

    // Configure pin F2 as PWM output pin for M1PWM6

    // Configure PWM options.
    // - PWM_GEN_3 covers M1PWM6 and M1PWM7, you will use M1PWM6
    // - count down
    // - no sync

    // Set the period (expressed in clock ticks) using PWM_PERIOD (#define'd in drv8323rs.h)

    // Set the pulse width to 0

    // Enable the PWM generator

    // Turn on the PWM output pin.

    // Display startup notification on serial console
    UARTprintf("\t\t...PWM1 module initialized.\n");
}

//*************************************************************************************************
//
// Public functions
//
//*************************************************************************************************

// initialize PE4 as a digital output pin to drive the DRV8323RS ENABLE input:
void init_drv8323rs_enable(void)
{
    // Assume InitConsole() has already been called by main(), i.e. safe to call UARTprintf()
    UARTprintf("\tInitializing DRV8323RS Enable pin...\n");

    // 1) enable GPIO port E used for DRV8323RS-enable
    // 2) wait until peripheral is ready
    // 3) enable pin DRV8323RS_ENABLE_PIN (see drv8323rs.h for #define)
    // 4) initialize DRV8323RS_ENABLE_PIN to "on"

    // Display startup notification on serial console
    UARTprintf("\t...DRV8323RS Enable pin initialized.\n");
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

    // Assume InitConsole() has already been called by main(), i.e. safe to call UARTprintf()
    UARTprintf("\tInitializing DRV8323RS SPI communication...\n");

    // Set up PA3 as a manual CS pin
    // 1) enable the GPIO port used for DRV8323RS-nSCS
    // 2) wait until the peripheral is ready
    // 3) enable pin DRV8323RS_NSCS_PIN
    // 4) initialize DRV8323RS_NSCS_PIN to "on"
    UARTprintf("\t\t...SPI: nSCS pin enabled.\n");

    // Set up SSI2 in master Freescale (SPI) mode.
    // - we do not want to use SSI2Fss, we want to use our manual nSCS on PA3 instead.
    // 1) enable SSI2 peripheral
    // 2) wait until the SSI2 peripheral is ready
    // 3) use SSI2 on PB[7:4] - GPIOB
    // 4) wait until the GPIOB peripheral is ready
    // 5) configure GPIO PB4 as SCLK
    // 6) configure GPIO PB6 as MISO/SDO
    // 7) configure GPIO PB7 as MOSI/SDI
    // 8) use a bitmask to config PB4, PB6, and PB7 as SSI pins
    // 9) Configure SSI2 for Freescale SPI - see DRV8323.pdf in documentation/ for details
    // 10) enable SSI2 module. Make sure nSCS has already been set.
    UARTprintf("\t\t...SPI: SSI2 base enabled.\n");

    // Display startup notification on serial console:
    UARTprintf("\t...DRV8323RS SPI communication initialized.\n");
}

// initialize 3 GPIOs with input capture interrutps, to read DRV8323RS Hall sensors:
// - Tiva pins PB2, PE0, and PA4 map to DRV8323RS HALLA, HALLB, and HALLC, respectively.
void init_halls(void)
{
    // Assume InitConsole() has already been called by main(), i.e. safe to call UARTprintf()
    UARTprintf("\tInitializing interface with Hall sensors...\n");

    // 1) Enable the #define'd peripherals for Hall A, B, and C (see drv8323rs.h)

    // 2) Regitster the interrupt handlers for Hall A, B, and C

    // 3) Set HALLx_PIN as a GPIO input

    // 4) Trigger interrupts on both rising and falling edges

    // 5) Set interrupt priorities to something "high"

    // 6) Enable interrupts on HALLx_PIN

    // 7) Initialize HallState by calling read_halls();

    // Display startup notification on serial console:
    UARTprintf("\t...Interface with Hall sensors initialized.\n");
}

// initialize 3 ADC pins to read DRV8323RS ISEN{A:C} (current sense for each phase)
void init_isense_ADCs(void)
{
    // Assume InitConsole() has already been called by main(), i.e. safe to call UARTprintf()
    UARTprintf("\tInitializing DRV8323RS current sense ADCs...\n");

    // 1) enable ADC0 peripheral
    // 2) wait until ADC0 peripheral is ready

    // 3) need PE1/AIN2 (ISENB) and PE2/AIN1 (ISENA), so enable GPIOE peripheral
    // 4) need PD3/AIN4 (ISENC), so enable GPIOD peripheral

    // 5) configure GPIO PE1 (ISENB) as an ADC pin
    // 6) configure GPIO PE2 (ISENA) as an ADC pin
    // 7) configure GPIO PD3 (ISENC) as an ADC pin
    // 8) Use ADC sequence 1 (SS1) - 4 samples
    // 9) Configure each step of ADC sequence 1:
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH4);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
    // 10) Enable ADC sequence 1 on ADC0
    // 11) Clear ADC0 sequence 1 interrupt

    UARTprintf("\t...DRV8323RS current sense ADCs initialized.\n");
}

// initialize phase enable (INLx) pins - pin definitions are in drv8323rs.h
void init_phase_enable_pins(void)
{
    // Assume InitConsole() has already been called by main(), i.e. safe to call UARTprintf()
    UARTprintf("\tInitializing DRV8323RS phase enable pins...\n");

    // 1) Enable INLx_PERIPH (see drv8323rs.h for #define's)

    // 2a) wait until peripheral is ready
    // 2b) enable pin INLA
    // 2c) initialize INLA to LOW
    UARTprintf("\t\t...Phase enable A (INLA) initialized and disabled.\n");

    // 3a) wait until peripheral is ready
    // 3b) enable pin INLB
    // 3c) initialize INLB to LOW
    UARTprintf("\t\t...Phase enable B (INLB) initialized and disabled.\n");

    // 4a) wait until peripheral is ready
    // 4b) enable pin INLC
    // 4c) initialize INLC to HIGH
    UARTprintf("\t\t...Phase enable C (INLC) initialized and enabled.\n");

    UARTprintf("\t...DRV8323RS phase enable pins initialized.\n");
}

// initialize all 3 PWM modules
void init_all_PWMs(void)
{
    // Display startup notification on serial console
    UARTprintf("\tInitializing PWM modules...\n");
    // 1) Initialize the timer-based pwm
    // 2) initialize PWM0
    // 3) initialize PWM1

    // 4a-c) Initialize all 3 pwm duty cycles to a low % (i.e., 1%)

    UARTprintf("\t...All PWM modules have been initialized.\n");
}

// update duty cycle of the timer-generated PWM module:
// I left this code in place because it isn't crucial to understanding BLDC commutation,
// but it is crucial to allowing the commutation code to be agnostic w.r.t. the PWM generators
void set_timer_pwm_dc(uint8_t duty_cycle_pc) // 0 <= duty_cycle_pc <= 100
{
    uint32_t load = TimerLoadGet(TIMER3_BASE, TIMER_B);
    uint32_t match = ((uint32_t) load - (load*duty_cycle_pc)/100);
    if (match >= load) match = load - 1;
    TimerMatchSet(TIMER3_BASE, TIMER_B, match);
}

// update duty cycle of the timer-generated PWM module:
void set_pwm0_dc(uint8_t duty_cycle_pc) // 0 <= duty_cycle_pc <= 100
{
    uint32_t width = ((uint32_t) (PWM_PERIOD*duty_cycle_pc)/100);
    if(width < 1) width = 1;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, width);
}

// update duty cycle of the timer-generated PWM module:
void set_pwm1_dc(uint8_t duty_cycle_pc) // 0 <= duty_cycle_pc <= 100
{
    uint32_t width = ((uint32_t) (PWM_PERIOD*duty_cycle_pc)/100);
    if(width < 1) width = 1;
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, width);
}

// SPI write to DRV8323RS:
void drv8323rs_spi_write(uint8_t address, uint16_t data)
{
    uint32_t ui32RxData = 0; // initialization
    uint32_t *pui32RxData = &ui32RxData;

    uint32_t ui32TxData = (((0 << 15) | (address << 11)) | (data & 0x7FF));
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
// I left this code in place because getting comms to work takes a long time
// and isn't crucial to understanding BLDC commutation
uint16_t drv8323rs_spi_read(uint8_t address)
{
    // Check that address is valid:
    if(address > 0x06) return(0); // valid register addresses are [0x06:0x00]

    uint32_t ui32RxData = 0; // initialization
    uint32_t *pui32RxData = &ui32RxData;

    uint32_t ui32TxData = (((1 << 15) | (address << 11)));
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
        UARTprintf("\tConfigured DRV8323RS to PWM mode %d.\n",pwm_mode >> PWM_MODE_FLD_LSB);
    }
    else
    {
        UARTprintf("\tError: invalid PWM mode %d.\n",pwm_mode >> PWM_MODE_FLD_LSB);
    }
}

// ISENSE ADC read functions:
void read_ISEN_ABC(uint32_t *phase_curr_arr) // read current sense for all 3 phases
{
    // Trigger the ADC conversion
    ADCProcessorTrigger(ADC0_BASE, 1);

    // Wait for conversion to be completed
    while(!ADCIntStatus(ADC0_BASE, 1, false)) {;}

    // Clear the ADC interrupt flag
    ADCIntClear(ADC0_BASE, 1);

    // phase_curr_arr must have 4 elements in it! No error checking here.
    ADCSequenceDataGet(ADC0_BASE, 1, phase_curr_arr);
}

// Read all 3 Hall sensor states:
uint8_t read_halls(void)
{
    return ((uint8_t) (((GPIOPinRead(HALLC_PORT, HALLC_PIN) & HALLC_PIN) >> 2) \
                | ((GPIOPinRead(HALLB_PORT, HALLB_PIN) & HALLB_PIN) << 1) \
                | ((GPIOPinRead(HALLA_PORT, HALLA_PIN) & HALLA_PIN) >> 2)));
}
