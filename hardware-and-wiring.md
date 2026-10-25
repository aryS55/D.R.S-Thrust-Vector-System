# Hardware and Wiring

Source: [`tvc_rocket4.ino`](../src/tvc_rocket4/tvc_rocket4.ino). See also [architecture](architecture.md). The parts list is linked from the [README](../README.md).

**PCB Gerber files:** `[Add PCB Gerber zip link here]`
A PCB is highly recommended: it reduces wiring and weight. LEDs exist only on the PCB.

## Pin map

| Teensy pin | Pin type | Connected to |
|---|---|---|
| 0 (RX1) | UART receive | Receiver iBUS signal (115200 baud) |
| 1 (TX1) | PWM | Servo 1. The UART transmit function is unused. |
| 2 | PWM | Servo 2 |
| 3 | PWM | Servo 3 |
| 4 | PWM | Servo 4 |
| 5, 6 | PWM | Servos 5 and 6: spare outputs for extra uses, or replacements if another pin is damaged |
| 7 | PWM | ESC signal |
| 10 | Digital output (SPI chip select) | SD card CS |
| 11 / 12 / 13 | SPI MOSI / MISO / SCK | Shared by SD card and BMI323 |
| 18 (A4) | Digital output (SPI chip select) | BMI323 CS |
| 22 | Digital output | Parts LED (PCB only) |
| 23 | Digital output | Status LED (PCB only) |

Servos are numbered clockwise from above. In flight, servo 1 = front, 2 = right, 3 = back, 4 = left. The IMU on the PCB sits at a 45° offset to this, which the firmware's flight frame accounts for ([flight-control](flight-control-and-mixing.md#1-coordinate-frames)).

## LEDs (PCB only)

The PCB has three LEDs: a power LED (not connected to the Teensy, lights when the PCB is powered), plus the status LED (pin 23) and the red parts LED (pin 22), which the firmware drives. For a hand-wired build, comment out `#define USE_LEDS` in the sketch. Blink patterns: [logging-and-diagnostics](logging-and-diagnostics.md#leds-pcb-only).

## SD card (optional)

Not mandatory, but recommended for recording and analysing flights. If your build has no card, comment out `#define USE_SD`.

## Power

5 V BEC → ESC terminal → 5 V rail → servos 1–6, receiver, SD card regulator (XC6206, 3.3 V) and Teensy VIN through diode D1 (prevents back-feeding when USB and the BEC are both connected). The BMI323 takes 3.3 V from the Teensy. The PCB is 4-layer: signal / GND / 5 V / signal.
