# BYOM-CCS-skeleton
Comments-only template for BYOM project.
Most of the code is stripped out, except for a few basic functions.
You will follow the guidelines in the comments to write your own code.

## Basic guidelines
* `bldc.{c,h}` contains high-level commutation code and should be hardware-agnostic.
* `drv8323rs.{c,h}` contains hardware-specific code and not high-level commutation code.
* `setup.{c,h}` contains functions that initialize other things not essential to BLDC commutation.
* Call all setup functions from the main function inside `main.c`. Don't write out any setup functions inside the main function in `main.c`; define them in the appropriate C file.
* All hardware interrupts need to be registered in the interrupt vector table in `tm4c123gh6pm_startup_ccs.c`. Pay attention to the use of the `extern` keyword in the function prototypes of the Hall sensor interrupt handlers in `drv8323rs.h` (the functions are defined in `bldc.c`).
* If your program hangs during execution and you have debugged your functions, you may need to increase the stack size. CCS has made this hard to find.
    * Right-click on your project in the 'Project Explorer' window, then click on 'Properties', scroll down on the left and expand the options for 'ARM Linker', and then click on 'Basic Options'.
    * This should bring up a dialog with an option to 'Set C system stack size'; enter a larger number in the corresponding field (for example, change 512 to 1024).
