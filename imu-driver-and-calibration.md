# IMU Driver and Boot Calibration

Source: [`tvc_rocket4.ino`](../src/tvc_rocket4/tvc_rocket4.ino), section 10. Related: [state-estimation](state-estimation.md), [architecture](architecture.md).

The BMI323 runs at 200 Hz, accelerometer ±4 g, gyro ±500 °/s, and the settings are read back and verified at boot.

| Function | Purpose |
|---|---|
| `initBMI323` | Checks chip ID, resets, writes and verifies the configuration, confirms valid data (up to 3 attempts) |
| `bmiProbeDrdy` | Tests whether the data-ready flags work; if so the loop locks its timing to the sensor |
| `readIMU` | Reads one sample, converts to g and °/s, subtracts gyro bias, flags invalid, stale or frozen data |
| `calibrateBMI323` | Measures gyro bias at boot |
| `bmiReadReg16` / `bmiWriteReg16` / `bmiBurstRead` | Low-level SPI access |

## Boot calibration

Keep the rocket **still and upright** during the ~5 s of calibration (1 s settle, then 800 samples).

A calibration attempt is accepted only if all of these hold:

| Check | Limit |
|---|---|
| Gyro noise (std dev) | ≤ 1.0 °/s |
| Gyro bias | ≤ 15 °/s |
| Accelerometer noise | ≤ 0.05 g |
| Accelerometer magnitude | 0.85–1.15 g |
| Valid samples | ≥ 95 % |

It tries 3 times, then halts startup. On success, the gyro means become the bias subtracted from every later sample, and the mean gravity vector sets the starting tilt. Only gyro bias is calibrated: there is no accelerometer calibration. Startup also fails if the starting tilt is over 30°.

If calibration keeps failing, the rocket is being moved or vibrated, or the sensor does not read about 1 g.

## Errors at runtime

| Condition | Result |
|---|---|
| Invalid or stale sample | Tick counted as bad; after 2 in a row fins go neutral |
| 8 bad ticks in a row | Latched fault `FAULT_BMI_DATA`, motor off |
| 40 identical raw samples (200 ms) | Treated as a frozen sensor |
| Recovery from this fault | Sensor is re-initialised; gyro bias is not re-measured |

Serial output and `#EVT` log lines at boot show the chip ID, configuration, calibration statistics and data-ready status. Log columns `stale_cnt`, `dup_cnt`, `bad_cnt` and `drdy_ok` track sensor health in flight.
