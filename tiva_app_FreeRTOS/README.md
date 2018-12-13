# Tiva application for BYOM project
Demonstrates how to use FreeRTOS for motor control

## Primary components
- Timer-generated interrupt for motor control
- Serial I/O tasks (interface with client script on PC)
- Inter-process communication (IPC) between motor control ISR and serial I/O tasks
