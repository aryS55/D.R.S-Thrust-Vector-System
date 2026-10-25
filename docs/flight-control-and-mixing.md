# Flight Control and Fin Mixing

Source: [`tvc_rocket4.ino`](../src/tvc_rocket4/tvc_rocket4.ino), sections 3.2–3.6, 14 and the control part of `estimatorControlTick()`. Related: [state-estimation](state-estimation.md), [rc-input-motor-and-safety](rc-input-motor-and-safety.md).

## 1. Coordinate frames

The IMU is mounted 45° offset to the fins. The firmware rotates angles and rates 45° into a **flight frame** where the fins form a "+" (servo 1 front, 2 right, 3 back, 4 left). Sticks, setpoints, PID and most log columns use the flight frame. Positive gz is counter-clockwise from above.

## 2. Control elements

| Element | What it does |
|---|---|
| X / Y attitude loop | Moves the fins to hold tilt at the setpoint: P on angle error, D on rotation rate. Integral is available but off (`Ki = 0`) |
| Yaw rate loop | Holds spin rate at the commanded value (0 unless the rudder stick is used) |
| Yaw steady feedforward | Applies the fin angle that cancels the motor's reaction torque at the current throttle (table from flight data) |
| Yaw dynamic feedforward | Adds a fin correction while the throttle is changing |
| Gyroscopic precession feedforward | Cancels the sideways push caused by the spinning motor when tilting |
| RC setpoints (`updateRcSetpoints`) | Stick → tilt setpoint (±10°) and rudder → yaw rate (±90 °/s), only when armed |
| `desaturateCmd` | Keeps all four fins within ±25°; tilt has priority over yaw |
| `finMix` | Converts X, Y and yaw commands into four fin angles |
| `mixAndDriveServos` | Applies offsets, smoothing and slew limit, and sends the servo pulses |
| Fin test | Runs the fins through a sequence at every startup to confirm they work |

## 3. Why yaw needs feedforward

The craft has one high-RPM motor, so it behaves like a gyroscope:

1. **X/Y coupling:** commanding a tilt one way also produces a force 90° from the command. The gyroscopic precession feedforward compensates this and is considered fixed.
2. **Throttle-induced spin:** changing the throttle changes the rotor's momentum and spins the craft by reaction torque. The steady feedforward works in flight. The dynamic feedforward and throttle slew limit help during changes, but compensation is weaker when the throttle changes quickly.

## 4. When the fins go neutral

Fins are set to zero (neutral) when any of these is true:

- the system is not in FLIGHT state (startup failed or latched fault),
- a tilt or spin trip is active,
- control is invalid (EKF unhealthy or tilt ≥ 45°),
- the IMU has been bad for more than 2 ticks.

The motor being **disarmed is not on this list**: with the motor disarmed, the fins still react to tilt and rotation. This helps with safe testing. The feedforwards and integrators run only when armed.

## 5. Tuning

All tunables are explained in the [parameter reference](setup-testing-and-tuning.md#parameter-reference). The main knobs are `Kp_x/y` and `Kd_x/y` (tilt response and damping), `Kp_z`/`Ki_z` (yaw rate), the `YAW_FF_*` and `YAW_DYN_*` values (throttle-related yaw compensation), and `offset1..4` / `dir1..4` (servo trim and direction). Use the test rig for tuning.
