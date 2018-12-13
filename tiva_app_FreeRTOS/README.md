# Tiva application for BYOM project
Demonstrates how to use FreeRTOS for motor control

## Primary components
- Timer-generated interrupt for motor control
- Serial interface
- Ring buffer shared between motor control interrupt service routine (ISR) and serial interface
