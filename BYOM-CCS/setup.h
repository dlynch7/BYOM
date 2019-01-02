//*************************************************************************************************
//
// setup.h
//
// Contains application-specific setup functions
//
//*************************************************************************************************

#ifndef SETUP_H
#define SETUP_H

// Macros
#define LED_RED GPIO_PIN_1
#define LED_BLUE GPIO_PIN_2 // BLUE on indicates buffer is empty
#define LED_GREEN GPIO_PIN_3 // GREEN on indicates buffer is full

// function prototypes
void LED_Init(void);

#endif
