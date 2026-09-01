# EDF Thrust-Vectoring Hover Rocket

A work-in-progress flight controller for a ducted-fan ("EDF") thrust-vectoring test rig. A gimbaled ring of 4 fins deflects the exhaust stream to control tilt and spin, driven by a PID loop reading a BMI323 IMU. This repository holds the firmware and a companion MATLAB tool for live tuning and offline log analysis.

**Status: under construction.** The motor/EDF is not yet wired up, so the controller currently only drives fins based on IMU feedback with no throttle scaling. Angle estimation is gyro-only (no accelerometer correction yet), so orientation estimates will drift over time — that's a known, expected limitation right now, not a bug.

## Repository Contents

| File | Description |
|---|---|
| `tvc_rocket.ino` | Arduino/Teensy flight controller firmware. Reads the BMI323 IMU over SPI, runs per-axis PID control, mixes output to 4 servos, and logs data to Serial and an SD card. |
| `tvc_tool.m` | MATLAB tool with two modes: a live serial logger/viewer with a real-time 3D orientation display, and an offline analyzer for saved CSV logs (INAV-style overlay plots, step-response metrics, and a scrubbable 3D orientation replay). |

## Hardware

- Arduino/Teensy-class microcontroller
- BMI323 IMU (SPI)
- 4x SG90 (or similar)
- microSD module (SPI, shares the bus with the BMI323)
- 70mm EDF Motor
- ESC capable of powering motor and includes a 5v BEC for powering hardware.

### Wiring

**BMI323 IMU (SPI)**
| Signal | Pin |
|---|---|
| CSB (Chip Select) | A4 |
| SCX (SPI Clock) | 13 (SCK) |
| SDX (SPI MOSI) | 11 (MOSI) |
| SDO (SPI MISO) | 12 (MISO) |
| VDD | 3.3V |
| GND | GND |

**Servos**
| Servo | Fin | Pin |
|---|---|---|
| Servo 1 | FIN1 (+Y) | 1 |
| Servo 2 | FIN2 (+X) | 2 |
| Servo 3 | FIN3 (-Y) | 3 |
| Servo 4 | FIN4 (-X) | 4 |

> Note: pins 1/2 double as Serial TX/RX (UART0) on most AVR/Teensy boards. This is fine as long as hardware Serial0 isn't also needed for USB/debug on those same pins. If Serial output stops working or looks garbled once servos are attached, move Servo1/Servo2 to different pins (e.g. 5 and 8).

**microSD module (SPI, shares the bus with the BMI323)**
| Signal | Pin |
|---|---|
| GND | GND |
| VCC | 3.3V |
| MISO | 12 |
| MOSI | 11 |
| SCK | 13 |
| CS | 10 |

## Axis / Fin Convention

The IMU is mounted flat (parallel to the ground) with the rocket standing nose-up:

- **+Z** points straight up along the rocket body
- **+Y** points toward FIN1 (horizontal plane)
- **+X** points toward FIN2 (horizontal plane, 90° from Y)

```
         FIN1 (+Y)
            |
FIN4 --- CENTER --- FIN2 (+X)
            |
         FIN3 (-Y)
```

**Control mapping:**
- Rotation about X ("pitch") → corrected by FIN2 & FIN4
- Rotation about Y ("yaw") → corrected by FIN1 & FIN3
- Rotation about Z ("roll") → corrected by all 4 fins deflecting together in the same rotational sense (spin/swirl correction)

## Data Logging

Both Serial (100 Hz CSV, for live tuning) and the SD card (same columns, for a full offline record) output:

```
time_ms,angle_x,angle_y,rate_z,gx,gy,gz,ax,ay,az,
angle_x_sp,angle_y_sp,err_x,err_y,
pid_x,pid_y,pid_z,
servo1,servo2,servo3,servo4
```

| Column | Description |
|---|---|
| `time_ms` | Timestamp, ms |
| `angle_x`, `angle_y` | Estimated tilt about X/Y, deg. Gyro-only integration currently (no accel correction), so it will drift slowly even at rest — expected for now. |
| `rate_z` | Spin rate about Z (vertical/roll), deg/s |
| `gx`, `gy`, `gz` | Bias-corrected gyro rates, deg/s |
| `ax`, `ay`, `az` | Raw accelerometer, g. Logged for debugging, not currently used in the angle estimate. |
| `angle_x_sp`, `angle_y_sp` | Setpoints, deg |
| `err_x`, `err_y` | Tracking error (setpoint − estimate), deg |
| `pid_x`, `pid_y` | Total PID controller output per axis, deg-equivalent |
| `pid_z` | Roll/spin PID output |
| `servo1`–`servo4` | Commanded fin deflection, deg (FIN1–FIN4) |

The SD card writes to the next free `LOGnnnn.CSV` each boot so old runs aren't overwritten. If SD init fails (no card, bad wiring), the firmware falls back to Serial-only logging and prints a warning rather than halting.

## Firmware (`tvc_rocket.ino`)

- Complementary-filter angle estimation (currently gyro-only, see above)
- Per-axis PID (X, Y, and a roll/spin axis Z), independently tunable gains at the top of the file
- Servo mixing: `pid_x` drives FIN2/FIN4 (mirrored), `pid_y` drives FIN1/FIN3 (mirrored), `pid_z` adds to all 4 fins with the same sign
- Per-servo mechanical offsets and direction-reversal flags for physically straightening/correcting fins
- Startup fin test sequence that exercises each control axis on boot, before the PID loop engages, so fin motion can be visually sanity-checked
- SD card init via `SdFat` (`SD.sdfs.begin()`) at a fixed 4 MHz SPI clock, with boot-time diagnostics (init success, card size, opened log filename, a write+flush test)
- Throttle/fin-authority scaling is **not implemented** — removed for now since the motor/EDF isn't wired up. This is a placeholder for gain-scheduling by throttle once flight testing begins, since fin effectiveness depends on the EDF exhaust's dynamic pressure on the fin (roughly throttle²), not on rocket CG geometry.

### Tunable Parameters

PID gains, setpoints, servo pin assignments, pulse-width limits, max fin deflection, per-servo offsets, and direction flags are all defined near the top of `tvc_rocket.ino`.

## MATLAB Tool (`tvc_tool.m`)

Single file, two modes selected via the `MODE` variable at the top of the script.

### `MODE = "live"`
Connects to the board over Serial and shows a live-updating view: attitude, PID output, and servo command plots, plus a rotating labeled 3D block (TOP/BOTTOM/FIN1–4 faces) that mirrors `angle_x`/`angle_y` in real time. Tilting the rig toward a fin should rotate the model toward that same face — a quick way to catch a mismounted IMU or a sign error. Saves everything to a timestamped CSV on close (independent of the SD card copy).

Config: `PORT`, `BAUD`, `LOG_DURATION_S`.

### `MODE = "analyze"`
Loads a saved CSV (a live-mode save or a `LOGnnnn.CSV` pulled from the SD card) and produces:
- Configurable overlay plots (`PLOT_GROUPS` / `GROUPS_TO_PLOT`) — INAV-style, e.g. `angle_x` vs `angle_x_sp` on the same axes
- Basic step-response metrics: peak value, settling time, oscillation period
- The same 3D orientation model as a scrubbable slider-driven replay, for visually re-checking the tilt-vs-fin-direction mapping after the fact

Config: `FILE`, `REPLAY_ORIENTATION`.

## Tuning Workflow

1. EDF off. Confirm the startup fin test sequence moves the fins in a direction that makes sense.
2. Tilt the rig by hand toward FIN1 — only `angle_y` should move. Tilt toward FIN2 — only `angle_x` should move. If both move together for a single-axis tilt, the IMU mount is misaligned with the fin cross; re-check the physical mounting against the diagram above.
3. Start with only `Kp_x`/`Kp_y` nonzero, `Ki = Kd = 0`. Increase `Kp` until oscillation, back off ~30–40%, add `Kd` to damp, add a little `Ki` last.

General tuning notes (also printed by the analyzer):
- Overshoot too high → increase Kd or decrease Kp
- Slow to respond / mushy → increase Kp
- Steady-state offset that never fully corrects → increase Ki slightly
- Oscillates and never settles → decrease Kp and/or increase Kd
- PID output pinned at limits (clipping) → reduce Kp or raise `MAX_FIN_DEG`

##Disclaimer

AI assisted with code and documentation
