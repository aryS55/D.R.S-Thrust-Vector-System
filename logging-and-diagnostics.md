# Logging, LEDs and Serial Output

Source: [`tvc_rocket4.ino`](../src/tvc_rocket4/tvc_rocket4.ino), sections 3.11, 3.12, 9, 16. Related: [architecture](architecture.md), [hardware-and-wiring](hardware-and-wiring.md).

**The SD card is optional.** It is not mandatory, but recommended for recording and analysing flights. Without it (comment out `USE_SD`), the rocket flies the same and live rows still stream over USB serial.

## How logging works

Log rows are written into a RAM buffer by the control loop, and copied to the card only in spare time between ticks and only while the motor is at idle. If the buffer fills (long powered runs), whole rows are dropped and counted in `log_drops`. The file is named `LOG0001.CSV`, `LOG0002.CSV`, … (first unused number).

| Function | Purpose |
|---|---|
| `buildRow` | Formats one CSV row (full for SD, 21 legacy columns for serial) |
| `sdEnqueue` | Puts text into the RAM buffer; drops the row if full |
| `updateSDLogging` | Writes buffered data to the card when it is safe to do so |
| `logEvent` / `logEventf` | Adds an `#EVT,time_ms,TYPE,message` line to the log |
| `initSD` | Opens the card and file at startup; skipped if `USE_SD` is off |

## CSV columns

| Group | Contents |
|---|---|
| Legacy 21 | time, angles, gyro, accelerometer, setpoints, errors, PID outputs, servo angles |
| PID terms | P/I/D for X, Y, Z |
| IMU and timing | raw and calibrated samples, timestamps, dt |
| Fusion and EKF | estimates, bias, uncertainty, innovation, rejection state |
| System | state, fault mask, armed, receiver channels, ESC pulse |
| Loop health | tick time, lateness, overruns, log drops, SD write time, stale/duplicate/bad sample counts |
| Control extras | rates, saturation flags, fin targets, gyro and yaw feedforward values |

When analysing: skip lines starting with `#`; add `gyro_ff_x/y` to `p+i+d` to get `pid_x/y`; `pid_z = p_z + i_z + yaw_ff_ss + yaw_ff_dyn`.

## LEDs (PCB only)

The LEDs exist only on the custom PCB; comment out `USE_LEDS` for a hand-wired build. A separate power LED is not firmware-controlled.

| LED | Pattern | Meaning |
|---|---|---|
| Status (pin 23) | 500 ms blink | Calibrating |
| | Solid | Servo startup, or armed in flight |
| | 100 ms blink, forever | Startup failed |
| | 250 ms blink | Latched fault |
| | Off | Disarmed |
| Parts, red (pin 22) | Rapid blink | SD error |
| | Solid | BMI323 error |

## Serial

115200 baud: boot messages, then the 21-column CSV header and live rows at 20 Hz.

## Things to know

* If the SD card fails mid-run, logging stops until reboot; flight control is unaffected.
* Data not yet written to the card is lost if power is cut.
