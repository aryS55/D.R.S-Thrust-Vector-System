# RC Input, Motor Output, Arming and Safety

Source: [`tvc_rocket4.ino`](../src/tvc_rocket4/tvc_rocket4.ino), sections 3.5, 3.7, 15. Related: [architecture](architecture.md), [flight-control](flight-control-and-mixing.md).

The code requires **iBUS** (FlySky FS-iA10B). The link counts as lost if no valid frame arrives for 150 ms.

| Channel | Use |
|---|---|
| Ch1 | Tilt setpoint (right/left) |
| Ch2 | Tilt setpoint (up/down) |
| Ch3 | Throttle |
| Ch4 | Yaw-rate command |
| Ch5 | Arm / kill switch (~2000 µs = armed) |

## Arming

The motor arms only when **all** of these hold:

| Condition | Value |
|---|---|
| Ch5 switch | ≥ 1700 µs, and it was seen low (≤ 1300 µs) since the last disarm |
| Receiver link | valid for ≥ 500 ms |
| Throttle stick | ≤ 1050 µs (at bottom) |
| Estimator and control | healthy, no latched fault |
| Tilt | < 12° |
| No tilt/spin trip pending | — |

No ESC calibration is needed.

## Disarm and trips

| Trigger | Result |
|---|---|
| Ch5 below 1500 µs, or receiver link lost | Motor off immediately |
| Tilt > 15° for 1.5 s | Motor off, fins neutral, until Ch5 goes low |
| Yaw spin > 300 °/s for 250 ms | Motor off, fins neutral, until Ch5 goes low |
| Latched fault | Motor off, fins neutral |
| Estimator invalid | Motor off |

After a disarm, the switch must go low before re-arming.

## Throttle

Throttle stick maps to 1000–2000 µs on the ESC. A slew limit (`THR_SLEW_LIMIT`) restricts changes to +0.5/s up and −0.4/s down, because the fins cannot cancel the yaw kick of a fast throttle change. Disarm, kill and failsafe bypass the limit and drop the motor to 1000 µs immediately.

## Latched faults

| Fault | Set when |
|---|---|
| `FAULT_BMI_DATA` | 8 consecutive bad IMU ticks |
| `FAULT_EKF_INVALID` | EKF state or covariance invalid |
| `FAULT_DISAGREE` | EKF and Fusion differ > 20° for 500 ms while armed |
| `FAULT_NONFINITE` | NaN/Inf in a control output |

A latched fault cuts the motor, neutralises the fins and blinks the status LED (PCB) at 250 ms.

### Fault recovery

A latched fault stays locked out. To clear it without power-cycling: with the throttle low and the rocket **still and upright**, switch Ch5 on and then low. The firmware re-initialises the sensor if needed, re-syncs the estimators and clears the fault; the control loop pauses while this runs. If it fails, the failure is logged and the next Ch5 cycle tries again. Startup failures cannot be recovered this way.

## Limits

There is no separate hardware cutoff: safety relies on this firmware holding the ESC at 1000 µs when disarmed.
