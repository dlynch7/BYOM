# Tiva application for BYOM project
Basic application, not using a RTOS (real-time operating system)

## Primary components
- Timer-generated interrupt for motor control
- Serial interface
- Ring buffer shared between motor control interrupt service routine (ISR) and serial interface
