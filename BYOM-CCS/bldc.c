// Functions for commutating a brushless motor.

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


// Initialize PWM1 module on PF2 (drives DRV8323RS - INHA)
static void InitPWMPhaseA(void) {
    // Configure PWM clock to match system.
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

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
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 0);

    // Enable the PWM generator.
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    // Turn on the PWM output pin.
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
}


// Create a 16-bit PWM module using Timer 3 CCP1 on PB3 (drives DRV8323RS - INHB)
static void InitPWMPhaseB(void) {
    // Enable the Timer3 peripheral.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    // Use T3CCP1 with port B pin 3. Start by enabling port B.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure GPIO pin muxing for the Timer/CCP function.
    GPIOPinConfigure(GPIO_PB3_T3CCP1);

    // Configure the ccp settings for CCP pin.
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Configure Timer3B as a 16-bit periodic timer.
    TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM);

    // Set the Timer3B load value to PWM_PERIOD
    TimerLoadSet(TIMER3_BASE, TIMER_B, PWM_PERIOD-1); // don't forget to -1 in the 'load' register!

    // Set the Timer3B match value to load value / 3.
    TimerMatchSet(TIMER3_BASE, TIMER_B, TimerLoadGet(TIMER3_BASE, TIMER_B));

    // Enable Timer3B
    TimerEnable(TIMER3_BASE, TIMER_B);
}


// Initialize PWM0 module on PC5 (drives DRV8323RS - INHC)
static void InitPWMPhaseC(void) {
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

    // Set the PWM duty cycle to 0.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 0);

    // Enable the PWM generator.
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    // Turn on the PWM output pin.
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
}

// Set/clear the DRV8323RS phase EN pins - in 3x PWM mode, these are INL{A:C}
// @param floatingPhase: The phase that should be disabled (floated)
static void SetEnablePhases(phase floatingPhase) {
    switch(floatingPhase)
    {
        case PHASE_A: {
            // Disable phase A, enable phase B, and enable phase C
            GPIOPinWrite(DRV8323RS_INLA_PORT, DRV8323RS_INLA_PIN, 0); // Disable phase A
            GPIOPinWrite(DRV8323RS_INLB_PORT, DRV8323RS_INLB_PIN, DRV8323RS_INLB_PIN); // Enable phase B
            GPIOPinWrite(DRV8323RS_INLC_PORT, DRV8323RS_INLC_PIN, DRV8323RS_INLC_PIN); // Enable phase C
            break;
        }
        case PHASE_B: {
            // Enable phase A, disable phase B, and enable phase C
            GPIOPinWrite(DRV8323RS_INLA_PORT, DRV8323RS_INLA_PIN, DRV8323RS_INLA_PIN);    // Enable phase A
            GPIOPinWrite(DRV8323RS_INLB_PORT, DRV8323RS_INLB_PIN, 0);           // Disable phase B
            GPIOPinWrite(DRV8323RS_INLC_PORT, DRV8323RS_INLC_PIN, DRV8323RS_INLC_PIN);    // Enable phase C
            break;
        }
        case PHASE_C: {
            // Enable phase A, enable phase B, and disable phase C
            GPIOPinWrite(DRV8323RS_INLA_PORT, DRV8323RS_INLA_PIN, DRV8323RS_INLA_PIN);    // Enable phase A
            GPIOPinWrite(DRV8323RS_INLB_PORT, DRV8323RS_INLB_PIN, DRV8323RS_INLB_PIN);    // Enable phase B
            GPIOPinWrite(DRV8323RS_INLC_PORT, DRV8323RS_INLC_PIN, 0);           // Disable phase C
            break;
        }
        default: // received a bad 'floating_phase' input
        {
            UARTprintf("Error: 0x%02X is an unknown floating_phase.\n",floatingPhase);
            break;
        }
    }
}


// Initialize hall sensor inputs and set up interrupts for those inputs
static void InitHalls(void) {
    SysCtlPeripheralEnable(DRV8323RS_HALLA_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_HALLB_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_HALLC_PERIPH);

    GPIOIntRegister(DRV8323RS_HALLA_PORT, HallAIntHandler);
    GPIOIntRegister(DRV8323RS_HALLB_PORT, HallBIntHandler);
    GPIOIntRegister(DRV8323RS_HALLC_PORT, HallCIntHandler);

    GPIOPinTypeGPIOInput(DRV8323RS_HALLA_PORT, DRV8323RS_HALLA_PIN);
    GPIOPinTypeGPIOInput(DRV8323RS_HALLB_PORT, DRV8323RS_HALLB_PIN);
    GPIOPinTypeGPIOInput(DRV8323RS_HALLC_PORT, DRV8323RS_HALLC_PIN);

    GPIOIntTypeSet(DRV8323RS_HALLA_PORT, DRV8323RS_HALLA_PIN, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(DRV8323RS_HALLB_PORT, DRV8323RS_HALLB_PIN, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(DRV8323RS_HALLC_PORT, DRV8323RS_HALLC_PIN, GPIO_BOTH_EDGES);

    IntPrioritySet(DRV8323RS_HALLA_INT, 0x00);
    IntPrioritySet(DRV8323RS_HALLB_INT, 0x00);
    IntPrioritySet(DRV8323RS_HALLC_INT, 0x00);

    GPIOIntEnable(DRV8323RS_HALLA_PORT, DRV8323RS_HALLA_PIN);
    GPIOIntEnable(DRV8323RS_HALLB_PORT, DRV8323RS_HALLB_PIN);
    GPIOIntEnable(DRV8323RS_HALLC_PORT, DRV8323RS_HALLC_PIN);

}


// Initialize current sense inputs
static void InitADC(void) {
    // Enable the ADC peripheral
    SysCtlPeripheralEnable(DRV8323RS_ISENSE_ADC_PERIPH);
    while(!SysCtlPeripheralReady(DRV8323RS_ISENSE_ADC_PERIPH)) {;}

    // Enable the GPIO port peripherals required by the ADC
    SysCtlPeripheralEnable(DRV8323RS_ISENSEA_GPIO_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_ISENSEB_GPIO_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_ISENSEC_GPIO_PERIPH);


    GPIOPinTypeADC(DRV8323RS_ISENSEA_GPIO_PORT, DRV8323RS_ISENSEA_GPIO_PIN);
    GPIOPinTypeADC(DRV8323RS_ISENSEB_GPIO_PORT, DRV8323RS_ISENSEB_GPIO_PIN);
    GPIOPinTypeADC(DRV8323RS_ISENSEB_GPIO_PORT, DRV8323RS_ISENSEC_GPIO_PIN);
    ADCSequenceConfigure(DRV8323RS_ISENSE_ADC_BASE, 1, ADC_TRIGGER_PROCESSOR, 0); // sequence 1 (SS1) - 4 samples
    ADCSequenceStepConfigure(DRV8323RS_ISENSE_ADC_BASE, 1, 0, DRV8323RS_ISENSEC_ADC_CTL_CH);
    ADCSequenceStepConfigure(DRV8323RS_ISENSE_ADC_BASE, 1, 1, DRV8323RS_ISENSEB_ADC_CTL_CH);
    ADCSequenceStepConfigure(DRV8323RS_ISENSE_ADC_BASE, 1, 2, DRV8323RS_ISENSEA_ADC_CTL_CH | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(DRV8323RS_ISENSE_ADC_BASE, 1);
    ADCIntClear(DRV8323RS_ISENSE_ADC_BASE, 1);
}


// Initialize the phase enable output pins
static void InitPhaseEnable(void) {
    SysCtlPeripheralEnable(DRV8323RS_INLA_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_INLB_PERIPH);
    SysCtlPeripheralEnable(DRV8323RS_INLC_PERIPH);

    while(!SysCtlPeripheralReady(DRV8323RS_INLA_PERIPH)); // check if peripheral access enabled
    GPIOPinTypeGPIOOutput(DRV8323RS_INLA_PORT, DRV8323RS_INLA_PIN); // enable pin INLA
    GPIOPinWrite(DRV8323RS_INLA_PORT, DRV8323RS_INLA_PIN, 0); // initialize INLA to LOW

    while(!SysCtlPeripheralReady(DRV8323RS_INLB_PERIPH)); // check if peripheral access enabled
    GPIOPinTypeGPIOOutput(DRV8323RS_INLB_PORT, DRV8323RS_INLB_PIN); // enable pin INLB
    GPIOPinWrite(DRV8323RS_INLB_PORT, DRV8323RS_INLB_PIN, 0); // initialize INLB to LOW

    while(!SysCtlPeripheralReady(DRV8323RS_INLC_PERIPH)); // check if peripheral access enabled
    GPIOPinTypeGPIOOutput(DRV8323RS_INLC_PORT, DRV8323RS_INLC_PIN); // enable pin INLC
    GPIOPinWrite(DRV8323RS_INLC_PORT, DRV8323RS_INLC_PIN, DRV8323RS_INLC_PIN); // initialize INLC to LOW
}


// Initialize the PWM outputs
static void InitPWM(void){
    InitPWMPhaseA();
    InitPWMPhaseB();
    InitPWMPhaseC();
}


// Updates the hall sensor state global.
static void UpdateHalls(void) {
    HallState = ((uint8_t) (((GPIOPinRead(DRV8323RS_HALLC_PORT, DRV8323RS_HALLC_PIN) & DRV8323RS_HALLC_PIN) >> 2) \
                    | ((GPIOPinRead(DRV8323RS_HALLB_PORT, DRV8323RS_HALLB_PIN) & DRV8323RS_HALLB_PIN) << 1) \
                    | ((GPIOPinRead(DRV8323RS_HALLA_PORT, DRV8323RS_HALLA_PIN) & DRV8323RS_HALLA_PIN) >> 2)));
}


// Sets the duty cycle of phase A (the PWM output associated with PWM1)
// @param dutyCycle: The duty cycle as a percentage
static void SetPhaseADutyCycle(uint8_t dutyCycle) {
    uint32_t width = ((uint32_t) (PWM_PERIOD*dutyCycle)/100);
    if(width < 1) width = 1;
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, width);
}

// Sets the duty cycle of phase B (the PWM output associated with Timer3)
// @param dutyCycle: The duty cycle as a percentage
static void SetPhaseBDutyCycle(uint8_t dutyCycle) {
    uint32_t load = TimerLoadGet(TIMER3_BASE, TIMER_B);
    uint32_t match = ((uint32_t) load - (load*dutyCycle)/100);
    if (match >= load) match = load - 1;
    TimerMatchSet(TIMER3_BASE, TIMER_B, match);
}


// Sets the duty cycle of phase C (the PWM output associated with PWM0)
// @param dutyCycle: The duty cycle as a percentage
static void SetPhaseCDutyCycle(uint8_t dutyCycle) {
    uint32_t width = ((uint32_t) (PWM_PERIOD*dutyCycle)/100);
    if(width < 1) width = 1;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, width);
}


// Set a particular phase's PWM output to a particular duty cycle.
// @param phase: The phase to set
// @param dutyCycle: The duty cycle as a percentage
static void SetPulseWidth(phase phase, uint8_t dutyCycle) {
    switch (phase) {
        case PHASE_A: {
            SetPhaseADutyCycle(dutyCycle);
            break;
        }
        case PHASE_B: {
            SetPhaseBDutyCycle(dutyCycle);
            break;
        }
        case PHASE_C: {
            SetPhaseCDutyCycle(dutyCycle);
            break;
        }
        default: {
            UARTprintf("Error: 0x%02X is an unknown phase value.\n",phase);
            break;
        }
    }
}


// Sets one phase to "ground" (zero PWM), one to PWM, and the third floating.
// @param pwm: The signed pwm percentage from -100 to 100
// @param p1: The first enabled phase
// @param p2: The second enabled phase
static void SetPhases(int16_t pwm, phase p1, phase p2) {
    // Given 2 energized phases, find the floating phase via lookup table 'floating'
    // If p1 and p2 are the energized phases, then floating[p1][p2] gives the floating phase
    static phase floating[3][3] =   {{PHASE_NONE,   PHASE_C,    PHASE_B},
                                     {PHASE_C,      PHASE_NONE, PHASE_A},
                                     {PHASE_B,      PHASE_A,    PHASE_NONE}};

    // The floating phase
    phase pfloat = floating[p1][p2];

    // phigh is the PWMed phase, plow is the grounded phase
    phase phigh, plow;

    // Will store the magnitude of the pwm count
    int apwm;

    // Choose the direction
    if (pwm > 0) {
        phigh = p1;
        plow = p2;
        apwm = pwm;
    }
    else {
        phigh = p2;
        plow = p1;
        apwm = -pwm;
    }

    // Record the PWM'd phase for current demuxing
    HighPhase = phigh;

    // Sets the phase enable outputs so the appropriate phase is left floating
    SetEnablePhases(pfloat); // definined in bldc.{c,h}

    // Set the PWMs appropriately
    SetPulseWidth(pfloat,0);      // floating phase has 0% duty cycle (technically doesn't matter)
    SetPulseWidth(plow,0);        // low phase also has 0% duty cycle
    SetPulseWidth(phigh,apwm);    // high phase gets the actual duty cycle
}


// Sets up peripherals required to interface with DRV8323RS driver and hall sensors,
// configures DRV8323RS driver in 3X PWM mode, and initializes current sense ADCs
void MotorSetup(void) {
    // Initialize phase enable pins
    InitPhaseEnable();

    // Initialize PWM outputs
    InitPWM();

    // Initialize ADCs
    InitADC();

    // Initialize serial communication with DRV8323RS, and set to 3X PWM mode
    InitDRV8323RS();
    SPIWriteDRV8323(DRV8323RS_DRIVER_CONTROL_REG, 0x320);
    UARTprintf("%x",SPIReadDRV8323(DRV8323RS_DRIVER_CONTROL_REG));

    // Initialize hall sensor interrupts
    InitHalls();
}


// Perform commutation
void MotorCommutate()
{
    UpdateHalls();

    switch(HallState)
    {
        case 0x4: // 0x4, 0b100
        {
            SetPhases(DUTY_CYCLE, PHASE_B, PHASE_A);  // if pwm > 0, phase A = GND and B is PWMed
            break;                              // if pwm < 0, phase B = GND and A is PWMed
        }
        case 0x6: // 0x6, 0b110
        {
            SetPhases(DUTY_CYCLE, PHASE_C, PHASE_A);
            break;
        }
        case 0x2: // 0x2, 0b010
        {
            SetPhases(DUTY_CYCLE, PHASE_C, PHASE_B);
            break;
        }
        case 0x3: // 0x3, 0b011
        {
            SetPhases(DUTY_CYCLE, PHASE_A, PHASE_B);
            break;
        }
        case 0x1: // 0x1, 0b001
        {
            SetPhases(DUTY_CYCLE, PHASE_A, PHASE_C);
            break;
        }
        case 0x5: // 0x5, 0b101
        {
            SetPhases(DUTY_CYCLE, PHASE_B, PHASE_C);
            break;
        }
        default:
        {
            // print an error msg to the serial console
            UARTprintf("Error: 0x%02X is an unknown commutation state.\n",HallState);
            break;
        }
    }
}

// Gets the current from the phase that is being PWM'd
// @return: The current reading on the phase that is being PWM'd
uint16_t GetCurrent(void) {
    // Array to store current reading
    uint32_t phaseCurrents[4] = {0};

    // Trigger the ADC conversion
    ADCProcessorTrigger(ADC0_BASE, 1);

    // Wait for conversion to be completed
    while(!ADCIntStatus(ADC0_BASE, 1, false)) {;}

    // Clear the ADC interrupt flag
    ADCIntClear(ADC0_BASE, 1);

    // phase_curr_arr must have 4 elements in it! No error checking here.
    ADCSequenceDataGet(ADC0_BASE, 1, phaseCurrents);

    switch(HighPhase) {
        case PHASE_A:
            return phaseCurrents[2] & 0x0FFF;
        case PHASE_B:
            return phaseCurrents[1] & 0x0FFF;
        case PHASE_C:
            return phaseCurrents[0] & 0x0FFF;
        default:
            return 0;
    }
}

// Hall sensor A interrupt handler
void HallAIntHandler(void) {
    GPIOIntClear(DRV8323RS_HALLA_PORT, DRV8323RS_HALLA_PIN);
    MotorCommutate();
}

// Hall sensor B interrupt handler
void HallBIntHandler(void) {
    GPIOIntClear(DRV8323RS_HALLB_PORT, DRV8323RS_HALLB_PIN);
    MotorCommutate();
}

// Hall sensor C interrupt handler
void HallCIntHandler(void) {
    GPIOIntClear(DRV8323RS_HALLC_PORT, DRV8323RS_HALLC_PIN);
    MotorCommutate();
}
