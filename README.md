# [B]uild [Y]our [O]wn [M]otor
Build and control a brushless DC (BLDC) motor.

Introductory project for Robot Design Studio (ME 495/EECS 495) 2019 at Northwestern University.

The project name is a bit of a misnomer: you won't build a motor from scratch, however you will be able to select from a number of motor winding options, allowing you to make design decisions about the motor's speed-torque curve.

## Parts list
- [Turnigy TrackStar 21.5T sensored BLDC motor](https://hobbyking.com/en_us/turnigy-trackstar-21-5t-sensored-brushless-motor-1855kv-roar-approved.html)
- [Texas Instruments EK-TM4C123GXL LaunchPad](http://www.ti.com/tool/EK-TM4C123GXL) (evaluation kit for Tiva TM4C123GH6PM microcontroller)
- [Texas Instruments BOOSTXL-DRV8323RS](http://www.ti.com/general/docs/litabsmultiplefilelist.tsp?literatureNumber=slvub01c) (evaluation kit for DRV8323Rx gate driver IC)

## BLDC connector
The BLDC has a 6-pin connector, but because it isn't immediately clear what each pin does, I removed the top half of the can and did some probing with a multimeter.

Inside, the Hall sensors are mounted to a circular PCB that sits above the coils.
The Halls are surface-mount components much like [these ones from TI](http://www.ti.com/product/DRV5055), with GND on one side of the IC and VCC and OUT both on the other side, which probably makes it harder to short across GND and Vcc.

Assuming the Halls inside this motor have the same pinout (and I saw many online with this pinout), I was able to figure out which of the 6 pins are VCC and GND, and in short order I had figured out which of the remaining pins were the outputs for the three Hall sensors.

That still left one mystery pin, which I traced to a small 2-pad surface mount component that I suspect is a temperature sensor.

Thus, looking top-down with phase markings right-side-up, the connector pinout is
`GND HALL_1 HALL_2 HALL_3 TEMP VCC`.

## Pin compatibility check: Tiva and motor driver
The BOOSTXL-DRV8323 motor driver devboard was designed to work TI's MSP430 devboard, and although the BOOSTXL devboard snaps onto the Tiva devboard, I want to make sure the pins are compatibile.

TI's documentation for the BOOSTXL-DRV8323 devboard provides a nice table detailing the pin connections between the BOOSTXL-DRV8323 and the MSP430 devboard and explains each pin's functions.
I am reproducing it below and adding a column for the Tiva devboard pins.

<details><summary> Click to expand pin compatibility table</summary>
<p>

| BOOSTXL-DRV8323 pin   | BOOSTXL-DRV8323 function   | MSP430 pin, function  | Tiva pin, function |
| ------------          | -------------             | ------------          | -------------      |
| J3-1                  | 3.3 V                     | 3.3 V                 | 3.3 V              |
| J3-2                  | no function               | 5 V                   | 5 V                |
| J3-3                  | VSENVM                    | P6.5, ADC - A5        | PB5, AIN 11        |
| J3-4                  | GND                       | GND                   | GND                |
| J3-5                  | no function               | P3.4, I/O pin         | PB0, I/O pin       |
| J3-6                  | VSENA                     | P6.0, ADC - 0         | PD0, AIN 7         |
| J3-7                  | no function               | P3.3, I/O pin         | PB1, I/O pin       |
| J3-8                  | VSENB                     | P6.1, ADC - 1         | PD1, AIN 6         |
| J3-9                  | ENABLE                    | P1.6, I/O w/ INT      | PE4, I/O w/ INT    |
| J3-10                 | VSENC                     | P6.2, ADC - 2         | PD2, AIN 5         |
| J3-11                 | POT                       | P6.6, ADC - A6        | PE5, AIN 8         |
| J3-12                 | ISENC                     | P6.3, ADC - 3         | PD3, AIN 4         |
| J3-13                 | SCLK                      | P3.2, SPI CLK         | PB4, SSI2 CLK      |
| J3-14                 | ISENB                     | P6.4, ADC - 4         | PE1, AIN 2         |
| J3-15                 | NFAULT                    | P2.7, I/O w/ INT      | PA5, I/O w/ INT    |
| J3-16                 | ISENA                     | P7.0, ADC - 12        | PE2, AIN 1         |
| J3-17                 | no function               | P4.2, I/O pin         | PA6, I/O pin       |
| J3-18                 | IDRIVE                    | P3.6, I/O pin         | PE3, I/O pin       |
| J3-19                 | no function               | P4.1, I/O pin         | PA7, I/O pin       |
| J3-20                 | VDS                       | P3.5, I/O pin         | PF1, I/O pin       |
| J4-1                  | INHA                      | P2.5, TA2.2           | PF2, M1PWM6        |
| J4-2                  | GND                       | GND                   | GND                |
| J4-3                  | INLA                      | P2.4, TA2.1           | PF3, M1PWM7        |
| J4-4                  | HALLA                     | P2.0, SPI ENABLE      | PB2, I/O w/ INT    |
| J4-5                  | INHB                      | P1.5, TA0.4           | PB3, T3CCP1        |
| J4-6                  | HALLB                     | P2.2, I/O w/ INT      | PE0, I/O w/ INT    |
| J4-7                  | INLB                      | P1.4, TA0.3           | PC4, M0PWM6        |
| J4-8                  | no function               | P7.4, I/O pin         | PF0, I/O pin       |
| J4-9                  | INHC                      | P1.3, TA0.2           | PC5, M0PWM7        |
| J4-10                 | no function               | RST                   | RST                |
| J4-11                 | INLC                      | P1.2, TA0.1           | PC6                |
| J4-12                 | SDI                       | P3.0, MOSI            | PB7, MOSI          |
| J4-13                 | MODE                      | P4.3, I/O pin         | PC7                |
| J4-14                 | SDO                       | P3.1, MISO            | PB6, MISO          |
| J4-15                 | LED                       | P4.0, I/O pin         | PD6, I/O pin       |
| J4-16                 | HALLC                     | P2.6, I/O w/ INT      | PA4, I/O w/ INT    |
| J4-17                 | EVM ID                    | P3.7, I/O pin         | PD7, I/O pin       |
| J4-18                 | nSCS/GAIN                 | P2.2, I/O w/ INT      | PA3, I/O w/ INT    |
| J4-19                 | EVM ID                    | P8.2, I/O pin         | PF4, I/O pin       |
| J4-20                 | CAL                       | P8.1, I/O pin         | PA2, I/O pin       |

</p>
</details>
