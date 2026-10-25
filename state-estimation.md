# State Estimation

The EKF was developed using [TinyEKF](https://github.com/simondlevy/TinyEKF). The Fusion AHRS is a port of [Fusion](https://github.com/xioTechnologies/Fusion) by xioTechnologies.

Source: [`tvc_rocket4.ino`](../src/tvc_rocket4/tvc_rocket4.ino), sections 8, 11, 12, 13. Related: [imu-driver](imu-driver-and-calibration.md), [flight-control](flight-control-and-mixing.md).

The firmware estimates **roll_x and pitch_y only**. There is no yaw angle (no compass), only yaw rate.

| Part | Purpose |
|---|---|
| Low-pass filters (`biquadStep`) | Smooth accelerometer (10 Hz) and gyro (20 Hz) |
| EKF (`ekfPredict`, `ekfUpdate`) | Primary attitude: blends gyro and gravity direction, estimates X/Y gyro bias, ignores accelerometer readings that look wrong |
| Fusion AHRS (`fusUpdate`) | Second attitude estimate used only as a cross-check; never used for control |
| `ekfHealthy` | Checks the EKF values are finite and not diverging |
| `initEstimators` / `resyncEstimatorsFromAccel` | Start or restart the estimators from gravity |

The Fusion cross-check is not fully independent: it uses the EKF's gyro bias estimate.

## Faults and warnings

| Condition | Result |
|---|---|
| EKF vs Fusion differ > 8° | Warning event in the log |
| Differ > 20° for 500 ms while armed | Latched fault `FAULT_DISAGREE`, motor off |
| EKF unhealthy | Latched fault `FAULT_EKF_INVALID`, motor off |
| Tilt ≥ 45° | Control invalid, fins neutral |

## Initialisation

`resyncEstimatorsFromAccel` runs before the first flight and during fault recovery. The rocket must be **still and roughly upright**; it fails if fewer than 80 of 100 samples are valid or the gravity magnitude is outside 0.85–1.15 g.

Tunable values: see the parameter table in [setup-testing-and-tuning](setup-testing-and-tuning.md#parameter-reference).
