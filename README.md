# [B]uild [Y]our [O]wn [M]otor
Build and control a brushless DC (BLDC) motor.
Introductory project for Robot Design Studio (ME 495/EECS 495) 2019 at Northwestern University

## Parts list
- [Turnigy TrackStar 21.5T sensored BLDC motor](https://hobbyking.com/en_us/turnigy-trackstar-21-5t-sensored-brushless-motor-1855kv-roar-approved.html)
- Texas Instruments EK-TM4C123GXL LaunchPad (evaluation kit for TM4C123GH6PM microcontroller)
- Texas Instruments BOOSTXL-DRV8323RS (evaluation kit for DRV8323Rx gate driver IC)

## BLDC connector
The BLDC has a 6-pin connector, but because it isn't immediately clear what each pin does, I removed the top half of the can and did some probing with a multimeter.

Inside, the Hall sensors are mounted to a circular PCB that sits above the coils.
The Halls are surface-mount components much like [these ones from TI](http://www.ti.com/product/DRV5055), with GND on one side of the IC and Vcc and OUT both on the other side, which probably makes it harder to short across GND and Vcc.

Assuming the Halls inside this motor have the same pinout (and I saw many online with this pinout), I was able to figure out which of the 6 pins are Vcc and GND, and in short order I had figured out which of the remaining pins were the outputs for the three Hall sensors.

That still left one mystery pin, which I traced to a small 2-pad surface mount component that I suspect is a temperature sensor.
