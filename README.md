# D.R.S. — Delivery Rocket System (Thrust Vectoring Monocopter)

D.R.S. is a single-motor, ducted-fan (EDF) rocket that balances itself by steering its exhaust with four servo-driven fins.

**End goal:** a small delivery vehicle that takes off easily, transitions into horizontal flight, delivers a small item, and lands, essentially a VTOL.

**What exists today:** a flight controller that keeps the rocket upright and lets a pilot tilt it and spin it with a radio. It does not fly itself anywhere.

---

## 1. Overview

- **Brain:** Teensy 4.0 running one Arduino sketch ([`src/tvc_rocket4/tvc_rocket4.ino`](src/tvc_rocket4/tvc_rocket4.ino)).
- **Sensor:** BMI323 IMU (gyroscope and accelerometer) over SPI.
- **Pilot link:** FlySky FS-iA10B receiver using **iBUS** (the code requires iBUS).
- **Actuators:** four SG90 servos moving four fins (MG90 servos also work, same size) and one ESC driving a 70 mm EDF.
- **Logging (optional, recommended):** micro-SD card for recording and analysing flights.
- **PCB (highly recommended):** a custom PCB cuts wiring and weight.

Every 5 ms the firmware reads the IMU, estimates tilt, computes fin movements, sets the motor throttle from the stick (after safety checks) and logs data.

---

## 2. Where to find what

| I want to know… | Read |
|---|---|
| How everything fits together, boot sequence, modes, failures | [docs/architecture.md](docs/architecture.md) |
| Pins, wiring, power, PCB, LEDs, parts requirements | [docs/hardware-and-wiring.md](docs/hardware-and-wiring.md) |
| How the IMU is read and calibrated | [docs/imu-driver-and-calibration.md](docs/imu-driver-and-calibration.md) |
| How tilt is estimated (EKF, Fusion) | [docs/state-estimation.md](docs/state-estimation.md) |
| PID, yaw and gyroscopic compensation, fin mixing, servos | [docs/flight-control-and-mixing.md](docs/flight-control-and-mixing.md) |
| Arming, kill switch, throttle, safety trips, fault recovery | [docs/rc-input-motor-and-safety.md](docs/rc-input-motor-and-safety.md) |
| SD logs, CSV columns, LEDs, serial output | [docs/logging-and-diagnostics.md](docs/logging-and-diagnostics.md) |
| Building, uploading, testing, tuning parameters | [docs/setup-testing-and-tuning.md](docs/setup-testing-and-tuning.md) |

```text
.
├── README.md
├── src/tvc_rocket4/tvc_rocket4.ino   <- the entire firmware
└── docs/                              <- the eight documents above
```

---

## 3. Hardware and resources (placeholders: nothing here exists yet)

| Resource | Description | Link |
|---|---|---|
| Drone CAD | Airframe, EDF duct, fin and servo mounts, electronics bay | `[Add CAD model link here]` |
| Bill of materials | Components, quantities, specs, purchase links | https://docs.google.com/spreadsheets/d/1vA3PeWrljAsKrt4Yhg0Rkp3cE5VI5zf5TELnXwBfbgM/edit?gid=0#gid=0 |
| **PCB Gerber files** | Zip for ordering the custom PCB | pcb/Gerber_PCB1_2026-10-25.zip |
| Test rig CAD | Rig used for PID tuning and other testing | `[Add testing rig CAD link here]` |
| Test rig bill of materials | Rig parts list (Google Sheet) | `[Add testing rig parts list link here]` |
| Data Explorer | Standalone zip for exploring flight data | `[Add Data Explorer zip link here]` |
| Assembly video: drone | Build walkthrough | `[Add drone assembly video link here]` |
| Assembly video: test rig | Build walkthrough | `[Add test rig assembly video link here]` |

**Key requirements:** 70 mm EDF motor; **6S battery heavily recommended** (4S may not produce enough thrust); lower-mAh batteries are recommended because of weight limits. No ESC calibration is needed.

**PCB:** highly recommended, because it reduces the amount of wiring and the weight. A hand-wired build works too; see [hardware-and-wiring](docs/hardware-and-wiring.md).

**Test rig:** recommended for PID tuning and testing. Place weights around or on its base so it cannot move.

---

## 4. Setup

1. Install Arduino IDE (developed with 2.3.10) and Teensyduino for the Teensy 4.0; update both to the latest versions.
2. Open `src/tvc_rocket4/tvc_rocket4.ino`, select Teensy 4.0, upload.
3. If your build has no LEDs or no SD card, comment out `USE_LEDS` and/or `USE_SD` near the top of the sketch (section 3.0).

Details: [docs/setup-testing-and-tuning.md](docs/setup-testing-and-tuning.md).

---

## 5. Current status

**Working**
- Stabilised tilt control with pilot angle commands and a yaw-rate command.
- Fin test runs on every startup, plus a startup self-check of the SD card and BMI323. On the PCB, the red parts LED shows which one failed.
- Steady yaw compensation (feedforward) works in flight.
- Gyroscopic coupling between X and Y (a tilt command also pushes 90° away from it) is compensated.

**Partially working**
- Yaw during throttle changes: the single high-RPM motor acts like a gyroscope, so changing throttle spins the craft by reaction torque. The controller compensates this reasonably for slow changes but is weaker when the throttle changes quickly. A throttle slew limit and a dynamic feedforward are included to help.

**Not implemented:** no heading hold, position hold, altitude hold, or any other autonomous flight; no transition to horizontal flight, no delivery mechanism.

**Testing:** no automated tests exist in the repository.
