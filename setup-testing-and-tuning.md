# Setup, Testing and Tuning

Related: [README](../README.md), [architecture](architecture.md), [hardware-and-wiring](hardware-and-wiring.md).

## Build and upload

* Arduino IDE (developed with 2.3.10) and Teensyduino for the Teensy 4.0. Update both to the most recent versions.
* Libraries: `Servo`, `SPI`, `SD` (bundled with Teensyduino). The sketch compiles as-is.
* Open [`src/tvc_rocket4/tvc_rocket4.ino`](../src/tvc_rocket4/tvc_rocket4.ino) (the folder name must match the sketch name), choose Teensy 4.0, set the Serial Monitor to 115200, upload.

### Build options (sketch section 3.0)

```cpp
#define USE_LEDS   // comment out if your build has no LEDs (LEDs are on the custom PCB only)
#define USE_SD     // comment out if your build has no SD card (optional, recommended for logging)
```

## Power-up sequence

1. Calibration (~5 s): keep the rocket **still and upright**.
2. BMI323 and SD startup checks. On the PCB the red parts LED shows which failed (rapid blink: SD, solid: BMI323).
3. Fin test (about 11 s): the fins run through a sequence to confirm they work.
4. Disarmed and ready. A startup failure halts the system and repeats an error message on USB serial.

## Suggested bench procedure

1. Watch the fin test; confirm fin numbering (1 front, 2 right, 3 back, 4 left) and directions. Adjust `dir1..4` and `offset1..4`.
2. With the motor disarmed, tilt the rocket by hand; the fins respond, which allows safe testing. If the response destabilises in flight, flip `TVC_SIGN_X/Y`.
3. Check receiver values (`ch_thr`, `ch_arm`, `ibus_ok`) in the serial or SD rows.
4. Arm and kill with Ch5.
5. For PID tuning and other testing, use the **test rig**, with weights placed around or on its base so it cannot move.

## Parameter reference

All tunables are in section 3 of the sketch. Most are `#define` constants (recompile to change); `Kp/Ki/Kd`, `offset1..4` and `dir1..4` are ordinary variables. Defaults in brackets.

### Loop timing

| Parameter | What it tunes |
|---|---|
| `EST_HZ` (200) | Rate of the estimator and control loop; must equal `BMI_ODR_HZ` |
| `LOG_HZ` (25) | SD log rows per second; must divide 200 |
| `SERIAL_HZ` (20) | Live USB rows per second; must divide 200 |
| `DT_MIN_S`, `DT_MAX_S` (2.5, 7.5 ms) | Valid loop-time window; outside it the integrators pause |
| `DT_EST_MAX_S` (30 ms) | Upper cap on the time step used by the estimators |
| `IMU_PLL_STEP_US`, `_TARGET_US`, `_MAX_STEP_US` (50, 150, 100 µs) | How the loop phase-locks to the sensor's data-ready timing |
| `LATE_REANCHOR_US` (1250 µs) | How late a tick can finish before the schedule is reset |

### Tilt control (X / Y)

| Parameter | What it tunes |
|---|---|
| `Kp_x`, `Kp_y` (0.9) | Tilt stiffness: fin degrees per degree of error |
| `Kd_x`, `Kd_y` (0.35) | Damping: fin degrees per °/s of rotation |
| `Ki_x`, `Ki_y` (0) | Integral correction for steady offset; 0 = off |
| `TVC_SIGN_X`, `TVC_SIGN_Y` (+1) | Correction direction; flip if it destabilises |
| `I_OUT_LIMIT_DEG` (3°) | Maximum fin authority the integral may use |
| `I_ERR_GATE_DEG` (6°), `I_RATE_GATE_DPS` (40) | Integral freezes while error or rate exceeds these |
| `D_TERM_LPF_HZ` (10) | Smoothing on the rate used for damping (0 = off) |
| `ANGLE_TRIM_X_DEG`, `ANGLE_TRIM_Y_DEG` (0) | Level offset for mounting error |
| `YAW_INTEG_MIN_THR` (0.15) | Throttle fraction above which all integrators run |

### Yaw rate loop

| Parameter | What it tunes |
|---|---|
| `Kp_z` (0.2) | Yaw-rate stiffness: fin degrees per °/s |
| `Ki_z` (0.15) | Yaw-rate integral correction |
| `TVC_SIGN_Z` (+1) | Direction of collective fin yaw torque |
| `I_OUT_LIMIT_Z_DEG` (4°) | Maximum fin authority of the yaw integral |
| `YAW_I_IDLE_DECAY` (0.98) | How quickly the yaw integral bleeds off at idle throttle |

### Yaw feedforward (throttle-related spin)

| Parameter | What it tunes |
|---|---|
| `YAW_FF_TBL_THR` / `YAW_FF_TBL_DEG` | Table of steady fin angle that cancels motor torque, versus throttle (0 at 0.25 up to 14° at 0.65) |
| `YAW_FF_SS_SCALE` (0.85) | Fraction of the table applied; 0 disables |
| `YAW_MOTOR_TAU_S` (0.08 s) | Assumed motor/ESC response lag of throttle |
| `YAW_DYN_K_UP` (1750), `YAW_DYN_K_DN` (6800) | Strength of the throttle-change spin when throttle rises / falls |
| `YAW_FIN_B` (70) | Assumed fin yaw authority; larger = smaller dynamic correction |
| `YAW_DYN_SCALE` (0.6) | Fraction of the dynamic correction applied; 0 disables |
| `YAW_DYN_MAX_DEG` (15°) | Cap on the dynamic correction |
| `YAW_DYN_MIN_THR` (0.30) | Below this throttle the dynamic correction is off |
| `THR_SLEW_LIMIT` | Enables the throttle slew limit |
| `THR_SLEW_UP_PER_S` (0.5), `THR_SLEW_DN_PER_S` (0.4) | Maximum throttle change per second, rising / falling |

### Gyroscopic precession feedforward

| Parameter | What it tunes |
|---|---|
| `GYRO_PRECESSION_FF_GAIN` (0.10) | Strength of the X/Y coupling compensation; 0 disables |
| `GYRO_PRECESSION_SIGN` (+1) | Direction of the compensation |
| `GYRO_PRECESSION_MIN_THR` (0.30) | Throttle below which it is off |
| `GYRO_PRECESSION_MAX_DEG` (4°) | Cap on the compensation |

### Radio and stick mapping

| Parameter | What it tunes |
|---|---|
| `ESC_MIN_US`, `ESC_MAX_US` (1000, 2000) | ESC pulse range |
| `THROTTLE_MAX_US` (2000) | ESC pulse at full stick; lower it if the ESC reaches full power early |
| `THR_IN_LOW_US`, `THR_IN_HIGH_US` (1000, 2000) | Throttle stick endpoints of your radio |
| `IBUS_CH_*` | Which iBUS channel is throttle, arm, tilt sticks and rudder |
| `RC_CENTER_US` (1500), `RC_DEADBAND_US` (30) | Stick centre and deadband |
| `RC_ATTITUDE_CONTROL` | Enables tilt commands from the sticks |
| `RC_MAX_ANGLE_DEG` (10°) | Tilt at full stick |
| `RC_SETPOINT_SLEW_DPS` (60) | How fast the tilt setpoint can move |
| `RC_SIGN_X`, `RC_SIGN_Y` (+1) | Stick direction |
| `RC_YAW_RATE_CONTROL` | Enables the rudder yaw-rate command |
| `RC_MAX_YAW_DPS` (90) | Yaw rate at full rudder |
| `RC_YAW_SIGN` (−1) | Rudder direction |
| `FLIGHT_FRAME_PLUS` | Selects the "+" flight frame (comment out for the raw X frame) |

### Servos and fins

| Parameter | What it tunes |
|---|---|
| `SERVO_CENTER_US` (1500), `SERVO_MIN_US`, `SERVO_MAX_US` (1000, 2000) | Servo pulse centre and limits |
| `US_PER_DEG` (10) | Pulse change per degree of fin |
| `MAX_FIN_DEG` (25°) | Maximum fin deflection |
| `SERVO_SMOOTH_HZ` (15) | Smoothing on fin commands (0 = off); lower = smoother but slower |
| `SERVO_MAX_SLEW_DPS` (300) | Maximum fin speed (0 = off) |
| `offset1..4` (0, −2, −2, 0) | Per-servo centre trim in degrees |
| `dir1..4` (1) | Per-servo direction (1 or −1) |
| `TEST_SWEEP_DEG` (15°), `TEST_HOLD_MS` (700) | Size and hold time of the startup fin test |

### Safety and failsafe

| Parameter | What it tunes |
|---|---|
| `ARM_SWITCH_HIGH_US` (1700) | Ch5 level that requests arming |
| `ARM_SWITCH_ON_ABOVE` (1500) | Ch5 level below which the motor is killed |
| `ARM_SWITCH_LOW_US` (1300) | Ch5 level counted as "low" (re-arm allowed, recovery trigger) |
| `ARM_THROTTLE_MAX_US` (1050) | Throttle must be below this to arm |
| `ARM_LINK_STABLE_MS` (500) | Receiver link must be stable this long to arm |
| `IBUS_TIMEOUT_MS` (150) | Time without a frame before the link counts as lost |
| `SPIN_LIMIT_DPS` (300), `SPIN_TRIP_MS` (250) | Yaw spin trip threshold and duration |
| `TILT_LIMIT_DEG` (15°), `TILT_TRIP_MS` (1500) | Tilt trip threshold and duration |
| `STARTUP_TILT_FATAL_DEG` (30°) | Maximum tilt at startup |
| `CONTROL_ANGLE_MAX_DEG` (45°) | Tilt beyond which fins go neutral |
| `IMU_FROZEN_LIMIT` (40) | Identical samples before the sensor counts as frozen |
| `IMU_GRACE_TICKS` (2) | Bad ticks held before fins go neutral |
| `IMU_INVALID_LIMIT` (8) | Bad ticks in a row before a latched fault |
| `BMI_DEAD_READ_LIMIT` (4) | Bad ticks before the sensor error is flagged |
| `IMU_DRDY_WAIT_US` (800) | Longest wait for fresh sensor data |
| `CROSSCHECK_WARN_DEG` (8°), `CROSSCHECK_MAX_DEG` (20°), `CROSSCHECK_MS` (500) | EKF vs Fusion warning level, fault level, and fault duration |
| `FAULT_RECOVERY_ENABLED` (true) | Allows clearing faults by cycling Ch5 |

### IMU configuration and filters

| Parameter | What it tunes |
|---|---|
| `BMI_MODE` (0x7), `BMI_ODR_CODE` (0x9), `BMI_ODR_HZ` (200) | Sensor performance mode and sample rate |
| `BMI_BW_QUARTER` (1) | Sensor bandwidth: ODR/4 (1) or ODR/2 (0) |
| `BMI_ACC_RANGE` (±4 g), `BMI_GYR_RANGE` (±500 °/s) | Measurement ranges |
| `BMI_INIT_ATTEMPTS` (3) | Sensor start-up retries |
| `ACC_LPF_HZ` (10), `GYR_LPF_HZ` (20) | Accelerometer and gyro smoothing |

### Estimators

| Parameter | What it tunes |
|---|---|
| `EKF_SIGMA_GYRO_DPS` (0.3) | Gyro trust; higher = lean on the accelerometer more |
| `EKF_SIGMA_BIAS_DPS_RTS` (0.02) | How fast gyro bias may drift |
| `EKF_SIGMA_ACC` (0.06) | Accelerometer trust; higher = trust it less |
| `EKF_ACC_GATE_G` (0.25) | Accelerometer readings further than this from 1 g are ignored |
| `EKF_ACC_DEV_SOFT_G` (0.05) | How quickly trust drops as the reading departs from 1 g |
| `EKF_P0_ANGLE_DEG` (3), `EKF_P0_BIAS_DPS` (0.5) | Initial uncertainty |
| `EKF_THETA_MAX_RAD` (1.45), `EKF_BIAS_MAX_DPS` (3) | Limits on pitch and bias estimates |
| `EKF_NIS_SOFT` (6), `EKF_NIS_HARD` (25) | Level at which inconsistent accelerometer data is down-weighted / rejected |
| `EKF_REJECT_RECOVER_S` (2) | Time of continuous rejection before the filter accepts data again |
| `FUS_GAIN` (0.25) | How fast Fusion corrects toward gravity |
| `FUS_ACC_REJECT_DEG` (10), `FUS_REJECT_TIMEOUT_S` (5) | Fusion's accelerometer rejection threshold and timeout |
| `FUS_INIT_STARTUP_GAIN` (10), `FUS_STARTUP_PERIOD_S` (3) | Fusion's fast-convergence start-up |

### Boot calibration

| Parameter | What it tunes |
|---|---|
| `CAL_SETTLE_MS` (1000), `CAL_SAMPLES` (800), `CAL_MAX_ATTEMPTS` (3) | Settle time, sample count and retries |
| `CAL_GYRO_STD_MAX_DPS` (1.0), `CAL_GYRO_BIAS_MAX_DPS` (15) | Maximum gyro noise and bias to accept |
| `CAL_ACC_NORM_MIN/MAX` (0.85–1.15 g), `CAL_ACC_STD_MAX_G` (0.05) | Accepted gravity magnitude and noise |

### Logging and LEDs

| Parameter | What it tunes |
|---|---|
| `SD_LOG_WHILE_ARMED` (true) | Whether rows are logged while armed |
| `LOG_RING_SIZE` (256 KB) | RAM buffer size; must be a power of two |
| `SD_CHUNK` (512), `LOG_ROW_MAX` (1024) | SD write block size and maximum row length |
| `SD_MIN_SLACK_US` (3500) | Spare time required before an SD write |
| `SD_SYNC_PERIOD_MS` (2000), `SD_FLYING_SYNC_PERIOD_MS` (0) | How often the file is flushed on the ground / in flight (0 = never) |
| `SD_IDLE_THR_MARGIN_US` (20), `SD_IDLE_HOLD_MS` (500) | How close to idle the ESC must be, and for how long, to count as idle |
| `LED_CAL_BLINK_MS` (500), `LED_FAST_BLINK_MS` (100), `LED_FAULT_BLINK_MS` (250) | LED blink periods |
| `ARMED_STATUS_LED` (true) | Whether the status LED turns solid when armed |
