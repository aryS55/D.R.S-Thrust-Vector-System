/*
  =====================================================================
  EDF THRUST-VECTORING "HOVER ROCKET" FLIGHT CONTROLLER  v2.4
  Teensy 4.0 + BMI323 (SPI) + 4x SG90 TVC servos + ESC via iBUS throttle
  EKF (primary attitude) + Madgwick Fusion AHRS (cross-check) + SD logger

  FILE LAYOUT
    1. Includes
    2. Hardware pin map
    3. TUNABLE VALUES  <-- everything you normally edit lives here
    4. Device constants (BMI323 registers, derived macros; do not edit)
    5. Types
    6. Prototypes
    7. Globals
    8. Helpers / math
    9. LEDs
   10. BMI323 driver + calibration
   11. EKF
   12. Fusion AHRS
   13. Estimator init / control state
   14. Control, mixing, servos
   15. iBUS, motor, arming, FAULT RECOVERY
   16. SD logging
   17. Main tick
   18. setup() / loop()

  v2.4 CHANGES vs v2.3 (from analysis of LOG0034/35/36):
    * Yaw feedforward was effectively zero (YAW_FF_DEG 0.1). The logs show the loop needs ~13 deg of collective fin at 0.6 throttle
      (the pilot's rudder was supplying most of it, and the yaw I-term sat at its 4 deg limit 85% of the time at high throttle).
      Now: throttle-dependent steady feedforward table (YAW_FF_TBL_*), driven by a motor-lag-filtered throttle.
    * Rotor-acceleration reaction torque: yaw acceleration is ~ -(K*(thr-0.25)) * d(thr)/dt (rise -> clockwise, fall -> counter-
      clockwise, falls ~1.4x stronger). Added a dynamic feedforward on the motor-lag throttle rate (YAW_DYN_*).
    * Optional throttle slew limit (THR_SLEW_LIMIT): fins cannot cancel fast throttle rises (needs ~1000 dps^2 per 1 thr/s at 0.6,
      available headroom ~400 dps^2), so rise/fall rates are bounded.
    * Log: appended yaw_ff_ss and yaw_ff_dyn columns.
    * FAULT RECOVERY: latched runtime faults (EKF invalid, EKF/Fusion disagree, BMI data, non-finite output) no longer require a power
      cycle. Flip Ch5 to ARMED then back to DISARMED (switch low) with throttle low: the sketch re-inits the sensor if needed, re-syncs the
      estimators from gravity (rig must be still, roughly upright) and clears the fault. Then arm as usual. If recovery fails it is
      logged, and the next switch cycle tries again. Startup (boot) failures still halt: those mean hardware is not working.
    * Yaw RC: RC_YAW_SIGN = -1 (stick right = clockwise = negative gz).

  AXES (sensor frame = body frame, BMI323 mounted flat):
    +Z up along the rocket, +X / +Y horizontal (right-handed).
    FINS ARE AN X-SHAPE: body +X and +Y point BETWEEN the servos, not at them.
    Roll_x  = rotation about X ("pitch" in TVC naming) -> all 4 fins, mixed (see finMix)
    Pitch_y = rotation about Y ("yaw"   in TVC naming) -> all 4 fins, mixed (see finMix)
    Z spin  ("roll" in TVC naming)                     -> all 4 fins together (yaw-rate loop + throttle FF cancels motor torque; Ch4 = yaw rate cmd)
    gz > 0 = counter-clockwise from above (left); gz < 0 = clockwise (right).
    Euler convention = ZYX (same as Fusion library):
        roll_x  = atan2(ay, az),  pitch_y = atan2(-ax, sqrt(ay^2+az^2))
    Accelerometer at rest reads +1 g on +Z when upright.

  SERVOS - numbered CLOCKWISE viewed from the top:
    FIN1 (pin 1) = +X+Y corner,  FIN2 (pin 2) = +X-Y corner,
    FIN3 (pin 3) = -X-Y corner,  FIN4 (pin 4) = -X+Y corner   (clockwise from above)
    ASSUMPTION: servo 1 is the +X+Y corner. If yours is rotated, edit finMix().
    FLIGHT FRAME (FLIGHT_FRAME_PLUS): in flight servo 1 = FRONT, 2 = RIGHT, 3 = BACK, 4 = LEFT. Control runs in a frame
    rotated 45 deg from the IMU so the fins form a "+" relative to the flight directions (sticks, setpoints, PID, logs).

  LEDs (single owner = updateLEDs):
    Status 23: calibrating = 500 ms blink; servo startup = solid; startup failed = 100 ms blink (forever);
               flight: OFF (disarmed) / solid ON (armed); latched runtime fault = 250 ms blink.
    Parts 22 : SD error = rapid blink; BMI error = solid; else OFF.   NOTE: pin 13 = SCK, never an LED.

  SAFETY SUMMARY: motor = 1000 us unless armed. Arm needs: Ch5 switch seen LOW then HIGH, throttle low, healthy estimator, iBUS stable
  >= 0.5 s, no fault, tilt < limit. Kill switch / iBUS loss => motor off immediately. Tilt > limit for 1.5 s while armed, or yaw spin
  > 300 dps for 250 ms => motor off + fins neutral until the switch is cycled low. Sensor/stale data: a tick only runs on a NEW valid IMU
  sample; after 2 missing ticks (10 ms) the fins go neutral and after 8 (40 ms) the motor is cut by a latched fault. EKF/Fusion
  cross-check: WARN at 8 deg, latched fault at 20 deg for 500 ms. 40 identical raw IMU samples => sensor fault.
  Latched faults are cleared by cycling Ch5 (see FAULT RECOVERY above), not by a power cycle.

  SD/timing: log rows queue in a RAM ring (RAM2). SD writes are only attempted in measured idle slack AND never while the ESC command is
  above idle. The ring drains on the ground. If it fills, whole rows are dropped (counted in log_drops).
  LOG: 21 legacy columns + PID terms + diagnostics (see LOG_HDR_*). Serial stays 21 legacy columns.
  Events: '#EVT,time_ms,TYPE,message' lines are interleaved in the SD file.
  NOTE: p_x+i_x+d_x no longer equals pid_x when the gyro feedforward is active; add gyro_ff_x (and gyro_ff_y) when post-processing.
        pid_z = p_z + i_z + yaw_ff_ss + yaw_ff_dyn (all before TVC_SIGN_Z is applied, which is +1).
  =====================================================================
*/

// ============================== 1. INCLUDES =========================
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// ============================== 2. HARDWARE PIN MAP =================
#define STATUS_LED_PIN 23
#define PARTS_LED_PIN  22
#define SERVO1_PIN 1   // FIN1, +X+Y corner, 1st clockwise
#define SERVO2_PIN 2   // FIN2, +X-Y corner
#define SERVO3_PIN 3   // FIN3, -X-Y corner
#define SERVO4_PIN 4   // FIN4, -X+Y corner
#define ESC_PIN    7
#define BMI323_CS_PIN A4
#define SD_CS_PIN  10

// =====================================================================
// ============================== 3. TUNABLE VALUES ====================
// =====================================================================

// ---------- 3.0 Build options: COMMENT OUT a line if your build does not have that hardware ----------
#define USE_LEDS   // Status LED (pin 23) and parts LED (pin 22). These exist only on the custom PCB; comment out for a hand-wired build.
#define USE_SD     // Micro-SD data logging. Not mandatory, but recommended for recording and analysing flights.

// ---------- 3.1 Loop timing ----------
#define EST_HZ         200                        // estimator + control rate (MUST equal BMI_ODR_HZ)
#define LOG_HZ         25                         // SD rows per second (ring ~19 s at 25 Hz; use 50 for ~9 s and finer transient resolution)
#define SERIAL_HZ      20                         // live Serial rows (legacy 21 columns)
#define DT_MIN_S       0.0025f                    // PID integration suspended when dt is outside [DT_MIN_S, DT_MAX_S]
#define DT_MAX_S       0.0075f
#define DT_EST_MAX_S   0.030f                     // estimator dt upper cap
#define IMU_PLL_STEP_US      50                   // tick phase lock to the sensor's ODR
#define IMU_PLL_TARGET_US    150
#define IMU_PLL_MAX_STEP_US  100
#define LATE_REANCHOR_US     1250                 // tick finished this late => re-anchor schedule

// ---------- 3.2 Attitude PID (X / Y), flight frame ----------
float Kp_x = 0.75, Ki_x = 0.000, Kd_x = 0.35;      // rotation about X -> all fins (mixed)
float Kp_y = 0.75, Ki_y = 0.000, Kd_y = 0.35;      // rotation about Y -> all fins (mixed)
#define TVC_SIGN_X (1.0f)                         // flip to -1 if the bench check shows the correction destabilizes
#define TVC_SIGN_Y (1.0f)
#define I_OUT_LIMIT_DEG    3.0f                   // max fin authority of the X/Y I term (limit applies to I OUTPUT)
#define I_ERR_GATE_DEG     6.0f                   // X/Y integrator frozen while |error| above this
#define I_RATE_GATE_DPS   40.0f                   // ... or |rate| above this
#define D_TERM_LPF_HZ     10.0f                   // extra low-pass on the D-term rate only (0 = off)
#define ANGLE_TRIM_X_DEG 0.0f                     // mounting offset removed from EKF angle
#define ANGLE_TRIM_Y_DEG 0.0f
#define YAW_INTEG_MIN_THR 0.15f                   // ALL integrators run only above this throttle fraction

// ---------- 3.3 Yaw (Z spin) rate loop + motor-torque compensation ----------
// Rate PI on gz (target 0 or the rudder command). Output goes to all 4 fins equally (fin degrees per deg/s).
float Kp_z = 0.15, Ki_z = 0.10;
#define TVC_SIGN_Z (1.0f)                         // logs confirm +1: positive collective produces +gz torque
#define I_OUT_LIMIT_Z_DEG  4.0f                   // max fin authority of the yaw I term
#define YAW_I_IDLE_DECAY   0.98f                  // per tick while throttle <= YAW_INTEG_MIN_THR

// Steady feedforward: collective fin (deg) that cancels rotor reaction torque, vs motor-lag throttle.
// Measured from LOG0034/35/36 steady windows (tilt-restrained rig): 0.32->1.9, 0.37->3.9, 0.43->6.4, 0.48->7.4, 0.53->10.3,
// 0.57->13.3, 0.63->13.9 deg. Table is smoothed. Above the last point the value is HELD (unverified extrapolation).
#define YAW_FF_N 8
static const float YAW_FF_TBL_THR[YAW_FF_N] = {0.25f, 0.32f, 0.40f, 0.49f, 0.54f, 0.58f, 0.615f, 0.65f};
static const float YAW_FF_TBL_DEG[YAW_FF_N] = {0.0f,  1.7f,  4.3f,  6.7f,  9.4f, 11.4f, 12.5f,  12.5f};
#define YAW_FF_SS_SCALE    1.0f                  // start < 1 (under-compensate); raise toward 1.0 as i_z shrinks. 0 disables steady FF
// Dynamic feedforward: reaction torque of rotor acceleration, dgz/dt = -K*(thr_m-0.25)*d(thr_m)/dt.
// Fin deg = SCALE * K*(thr_m-0.25)*d(thr_m)/dt / (B*thr_m), clamped. K fitted from the logs (+/-40%).
#define YAW_MOTOR_TAU_S    0.03f                  // rotor/ESC lag of throttle, fit range 0.04-0.12 s
#define YAW_DYN_K_UP       2400.0f                // dps^2 per (thr/s) per unit (thr_m-0.25), throttle rising
#define YAW_DYN_K_DN       2800.0f                // same, throttle falling (falls couple ~1.4x harder)
#define YAW_FIN_B          66.0f                  // fin yaw authority, dps^2 per (deg*thr), fit range 44-70
#define YAW_DYN_SCALE      0.9f                   // apply half first (0 disables dynamic FF)
#define YAW_DYN_MAX_DEG    12.0f
#define YAW_DYN_MIN_THR    0.30f                  // no dynamic FF below this lagged throttle
// Optional throttle slew limit (comment out THR_SLEW_LIMIT to disable). Bounds the yaw transient the fins cannot cancel.
#define THR_SLEW_LIMIT
#define THR_SLEW_UP_PER_S  0.35f                   // max throttle fraction increase per second (full range 0..1)
#define THR_SLEW_DN_PER_S  0.4f                   // max decrease per second (kill / disarm / failsafe are NOT slew limited)

// ---------- 3.4 Gyroscopic precession feedforward (X/Y cross-coupling) ----------
// LOG0023 fit at ~70% throttle: cancelling gain ~0.09 (X) / ~0.08 (Y) fin-deg per deg/s. Flat in throttle. Validate at 45% and 85%.
#define GYRO_PRECESSION_FF_GAIN  0.10f            // fin-deg per deg/s (0 disables)
#define GYRO_PRECESSION_SIGN     1.0f
#define GYRO_PRECESSION_MIN_THR  0.30f
#define GYRO_PRECESSION_MAX_DEG  4.0f

// ---------- 3.5 RC input mapping ----------
#define ESC_MIN_US       1000
#define ESC_MAX_US       2000
#define THROTTLE_MAX_US  2000                     // ESC pulse at FULL stick (lower it if the ESC reaches full power earlier)
#define THR_IN_LOW_US    1000                     // ch_thr at stick bottom / top (calibrate to your radio)
#define THR_IN_HIGH_US   2000
#define IBUS_CH_THROTTLE 2                        // Ch3
#define IBUS_CH_ARM      4                        // Ch5 arm/kill: ~1000 = DISARMED, ~2000 = ARMED
#define IBUS_CH_YAW      0                        // Ch1 right/left  (attitude setpoint)
#define IBUS_CH_PITCH    1                        // Ch2 up/down     (attitude setpoint)
#define IBUS_CH_RUDDER   3                        // Ch4 yaw rate command
#define RC_CENTER_US     1500
#define RC_DEADBAND_US   30
// Yaw RATE command on Ch4 (no compass: a rate command, not a heading hold). Comment out to disable.
#define RC_YAW_RATE_CONTROL
#define RC_MAX_YAW_DPS   90.0f
#define RC_YAW_SIGN      (-1.0f)                  // -1: stick right = yaw right (clockwise from above = negative gz)
// Attitude ("angle mode") stick control. Comment out to disable (setpoints stay 0).
#define RC_ATTITUDE_CONTROL
#define RC_MAX_ANGLE_DEG       10.0f
#define RC_SETPOINT_SLEW_DPS   60.0f
#define RC_SIGN_X              (1.0f)             // flip if stick right tilts toward -X
#define RC_SIGN_Y              (1.0f)             // flip if stick up tilts toward -Y
// Flight frame: rotate IMU angles/rates 45 deg so the X-shaped fins act as a "+". Comment out for the old X-frame behaviour.
#define FLIGHT_FRAME_PLUS

// ---------- 3.6 Servos / fins ----------
#define SERVO_CENTER_US 1500
#define SERVO_MIN_US    1000
#define SERVO_MAX_US    2000
#define US_PER_DEG      10.0f
#define MAX_FIN_DEG     25.0f
#define SERVO_SMOOTH_HZ    15.0f                  // fin command low-pass (0 = off)
#define SERVO_MAX_SLEW_DPS 300.0f                 // fin slew limit (0 = off); SG90 no-load ~600 dps
float offset1 = 0, offset2 = -2.0, offset3 = -2.0, offset4 = 0;   // deg; fins 2 and 3 measured ~2 deg off
int dir1 = 1, dir2 = 1, dir3 = 1, dir4 = 1;
#define TEST_SWEEP_DEG 15.0f                      // startup fin test
#define TEST_HOLD_MS   700

// ---------- 3.7 Safety, failsafe and fault recovery ----------
#define ARM_SWITCH_HIGH_US   1700                 // >= : arm request
#define ARM_SWITCH_ON_ABOVE  1500                 // <  : killed
#define ARM_SWITCH_LOW_US    1300                 // <= : "low" (re-arm permitted, fault recovery trigger)
#define ARM_THROTTLE_MAX_US  1050
#define ARM_LINK_STABLE_MS   500
#define IBUS_TIMEOUT_MS      150                  // no valid iBUS frame for this long => disarm
#define SPIN_LIMIT_DPS       300.0f               // yaw spin trip
#define SPIN_TRIP_MS         250
#define TILT_LIMIT_DEG        15.0f
#define TILT_TRIP_MS          1500
#define STARTUP_TILT_FATAL_DEG 30.0f
#define CONTROL_ANGLE_MAX_DEG  45.0f              // beyond this, fins neutral
#define IMU_FROZEN_LIMIT     40                   // identical raw samples => sensor dead (200 ms)
#define IMU_GRACE_TICKS      2                    // hold last fins this many bad ticks, then neutral
#define IMU_INVALID_LIMIT    8                    // consecutive bad ticks => latched fault (40 ms)
#define BMI_DEAD_READ_LIMIT  4
#define IMU_DRDY_WAIT_US     800
#define CROSSCHECK_WARN_DEG  8.0f                 // EKF vs Fusion
#define CROSSCHECK_MAX_DEG   20.0f
#define CROSSCHECK_MS        500
// Fault recovery (see header): attempted once per Ch5 cycle (switch seen ON, then LOW) while disarmed with throttle low.
#define FAULT_RECOVERY_ENABLED true

// ---------- 3.8 IMU configuration and filters ----------
#define BMI_MODE       0x7                        // high performance
#define BMI_ODR_CODE   0x9                        // 200 Hz (must equal EST_HZ)
#define BMI_ODR_HZ     200
#define BMI_BW_QUARTER 1                          // 1 = low-pass ODR/4 (~50 Hz); 0 = ODR/2
#define BMI_ACC_RANGE  0x1                        // +/-4 g
#define BMI_GYR_RANGE  0x2                        // +/-500 dps
#define BMI_INIT_ATTEMPTS 3
#define ACC_LPF_HZ 10.0f
#define GYR_LPF_HZ 20.0f

// ---------- 3.9 Estimators ----------
#define EKF_SIGMA_GYRO_DPS     0.3f
#define EKF_SIGMA_BIAS_DPS_RTS 0.02f
#define EKF_SIGMA_ACC          0.06f
#define EKF_ACC_GATE_G         0.25f
#define EKF_ACC_DEV_SOFT_G     0.05f
#define EKF_P0_ANGLE_DEG       3.0f
#define EKF_P0_BIAS_DPS        0.5f
#define EKF_THETA_MAX_RAD      1.45f
#define EKF_BIAS_MAX_DPS       3.0f
#ifndef EKF_NIS_SOFT
#define EKF_NIS_SOFT           6.0f
#endif
#ifndef EKF_NIS_HARD
#define EKF_NIS_HARD           25.0f
#endif
#define EKF_REJECT_RECOVER_S   2.0f
#define FUS_GAIN               0.25f
#define FUS_ACC_REJECT_DEG     10.0f
#define FUS_REJECT_TIMEOUT_S   5.0f
#define FUS_INIT_STARTUP_GAIN  10.0f
#define FUS_STARTUP_PERIOD_S   3.0f

// ---------- 3.10 Boot calibration ----------
#define CAL_SETTLE_MS          1000
#define CAL_SAMPLES            800                // 4 s at 200 Hz
#define CAL_MAX_ATTEMPTS       3
#define CAL_GYRO_STD_MAX_DPS   1.0f
#define CAL_GYRO_BIAS_MAX_DPS  15.0f
#define CAL_ACC_NORM_MIN       0.85f
#define CAL_ACC_NORM_MAX       1.15f
#define CAL_ACC_STD_MAX_G      0.05f

// ---------- 3.11 SD logging ----------
#define SD_LOG_WHILE_ARMED       true
#define LOG_RING_SIZE            262144UL         // power of two; 256 KB in RAM2
#define SD_CHUNK                 512
#define SD_MIN_SLACK_US          3500UL
#define SD_SYNC_PERIOD_MS        2000
#define SD_FLYING_SYNC_PERIOD_MS 0
#define SD_IDLE_THR_MARGIN_US    20
#define SD_IDLE_HOLD_MS          500
#define LOG_ROW_MAX              1024

// ---------- 3.12 LEDs ----------
#define LED_CAL_BLINK_MS   500
#define LED_FAST_BLINK_MS  100
#define LED_FAULT_BLINK_MS 250
#define ARMED_STATUS_LED   true

// =====================================================================
// ============================== 4. DEVICE CONSTANTS (do not edit) ====
// =====================================================================
#define EST_PERIOD_US (1000000UL / EST_HZ)
#define LOG_DECIM     (EST_HZ / LOG_HZ)
#define SERIAL_DECIM  (EST_HZ / SERIAL_HZ)
#define FRAME_K       0.70710678f
#define FIN_MIX_K     0.70710678f
#ifdef FLIGHT_FRAME_PLUS
  #define FIN_TEST_DIV 1.0f
#else
  #define FIN_TEST_DIV FIN_MIX_K
#endif

#define REG_CHIP_ID    0x00
#define REG_STATUS     0x02
#define REG_ACC_DATA_X 0x03      // 0x03..0x08 = ACC XYZ, GYR XYZ
#define REG_ACC_CONF   0x20
#define REG_GYR_CONF   0x21
#define REG_CMD        0x7E
#define CMD_SOFT_RESET 0xDEAF
#define BMI323_EXPECTED_CHIP_ID 0x43
#if BMI_ODR_HZ != EST_HZ
#error "BMI323 ODR must equal the control rate (EST_HZ): sampling the sensor at a different rate aliases vibration"
#endif
// bits: mode[14:12] avg_num[10:8] bw[7] range[6:4] odr[3:0]
#define ACC_CONF_VALUE ((uint16_t)((BMI_MODE << 12) | (BMI_BW_QUARTER << 7) | (BMI_ACC_RANGE << 4) | BMI_ODR_CODE))
#define GYR_CONF_VALUE ((uint16_t)((BMI_MODE << 12) | (BMI_BW_QUARTER << 7) | (BMI_GYR_RANGE << 4) | BMI_ODR_CODE))
#define STATUS_DRDY_GYR 0x0040
#define STATUS_DRDY_ACC 0x0080

SPISettings bmiSPISettings(8000000, MSBFIRST, SPI_MODE0);   // datasheet max 10 MHz

#define FAULT_BMI_DATA     0x01
#define FAULT_EKF_INVALID  0x02
#define FAULT_DISAGREE     0x04
#define FAULT_NONFINITE    0x08

#define EKF_ACC_IDLE       0
#define EKF_ACC_OK         1
#define EKF_ACC_SOFT       2
#define EKF_ACC_REJ_MAG    3
#define EKF_ACC_REJ_DIR    4
#define EKF_ACC_RECOVER    5
#define EKF_ACC_REJ_NUM    6

#define EVT_PENDING_MAX 96
#define EVT_LINE_MAX    128

// ============================== 5. TYPES ============================
enum SystemState {
  STATE_BOOT_CALIBRATING,
  STATE_SERVO_STARTUP,
  STATE_STARTUP_FAILED,
  STATE_FLIGHT,
  STATE_FAULT
};

struct V3   { float x, y, z; };
struct Quat { float w, x, y, z; };
struct Biquad { float b0, b1, b2, a1, a2, z1, z2; };

struct Ekf {
  float x[4];        // [roll_x, pitch_y] (rad), [bias_x, bias_y] (rad/s)
  float P[4][4];
  float innovNorm;
  float accNorm;
  bool  accUsed;
  float nis;
  float rScale;
  uint8_t accState;
  uint16_t rejStreak;
};

struct FusAhrs {
  float samplePeriod, gain, startupGainRate;
  bool  overrangeEnabled;
  float overrangeThreshold;
  float accelerationRejection;
  int32_t rejectionTimeout;
  Quat q;
  V3 accel;
  bool startup;
  float startupGain;
  bool overrangeRecovery;
  V3 halfAccResidual;
  int32_t accRecoveryTrigger, accRecoveryThreshold;
  bool accIgnored;
};

struct BmiConf { uint8_t mode, avg, bw, range, odr; };

struct ImuSample {
  uint32_t tUs;
  uint16_t waitUs;
  uint16_t frozenRun;
  bool fresh;
  int16_t raw[6];
  float acc[3];
  float gyr[3];
};

struct PidState { float integral, p, i, d; };

struct LogSnap {
  uint32_t tUs, dtUs;
  int16_t raw[6];
  float acc[3], gyr[3], accF[3], gyrF[3];
  float angX, angY, errX, errY, pidX, pidY, pidZ;
  float imuRoll, imuPitch;
  float rateXr, rateYr, rateXd, rateYd;
  float nis, rScale; uint32_t accState;
  float estDiffRoll, estDiffPitch;
  uint32_t dtRawUs;
  float pidP[3], pidI[3], pidD[3];
  float fusRoll, fusPitch, fusAccErr; uint32_t fusFlags;
  float biasX, biasY, sigX, sigY, sigBX, sigBY, innov;
  uint32_t accUsed; float accNorm;
  uint32_t tickUs;
  float gyroFFX, gyroFFY;
  float yawCmd;
  float yawFFss, yawFFdyn;   // v2.4: steady and dynamic yaw feedforward, fin degrees
};

struct RowWriter {
  char buf[LOG_ROW_MAX];
  uint16_t n;
  void reset() { n = 0; }
  void putChar(char c) { if (n < LOG_ROW_MAX - 2) buf[n++] = c; }
  void putStr(const char* s) { while (*s) putChar(*s++); }
  void putUnsigned(uint32_t u) {
    char t[11]; uint8_t k = 0;
    do { t[k++] = (char)('0' + (u % 10)); u /= 10; } while (u);
    while (k) putChar(t[--k]);
  }
  void u32(uint32_t v) { putUnsigned(v); putChar(','); }
  void i32(int32_t v) {
    if (v < 0) { putChar('-'); putUnsigned((uint32_t)(-(int64_t)v)); }
    else putUnsigned((uint32_t)v);
    putChar(',');
  }
  void f(float v, uint8_t dec) {
    static const uint32_t P10[7] = {1, 10, 100, 1000, 10000, 100000, 1000000};
    if (!((v - v) == 0.0f)) { putStr("nan"); putChar(','); return; }
    if (dec > 6) dec = 6;
    float s = fabsf(v) * (float)P10[dec];
    if (s > 4.0e9f) { putStr("ovf"); putChar(','); return; }
    uint32_t u = (uint32_t)(s + 0.5f);
    if (v < 0.0f && u != 0) putChar('-');
    putUnsigned(u / P10[dec]);
    if (dec) {
      putChar('.');
      uint32_t fp = u % P10[dec];
      uint32_t div = P10[dec] / 10;
      while (div) { putChar((char)('0' + (fp / div) % 10)); div /= 10; }
    }
    putChar(',');
  }
  void endRow() {
    if (n > 0 && buf[n - 1] == ',') buf[n - 1] = '\n';
    else putChar('\n');
  }
};

// ============================== 6. PROTOTYPES =======================
void updateLEDs();
void updateIBUS();
void delayWithLEDs(unsigned long ms);
void waitUntilUs(uint32_t t);
void failStartup(const char* reason);
void latchFault(uint8_t mask, const char* msg);
float clampf(float v, float lo, float hi);
bool isFiniteF(float x);
float wrapPi(float a);
V3 v3make(float x, float y, float z);
V3 vAdd(V3 a, V3 b);
V3 vScale(V3 a, float s);
V3 vCross(V3 a, V3 b);
float vDot(V3 a, V3 b);
float vNormSq(V3 a);
bool vIsZero(V3 a);
V3 vNormalise(V3 a);
Quat qAdd(Quat a, Quat b);
Quat qVecProduct(Quat q, V3 v);
Quat qNormalise(Quat q);
void biquadDesignLPF(Biquad* f, float fs, float fc);
void biquadReset(Biquad* f, float x0);
float biquadStep(Biquad* f, float x);
void ekfInit(float phi, float theta);
void ekfPredict(float gx, float gy, float gz, float dt);
bool ekfUpdate(const float acc[3]);
bool ekfHealthy();
bool chol3Solve(const float S[3][3], const float HP[3][4], float X[3][4]);
int32_t clampI32(int32_t v, int32_t lo, int32_t hi);
V3 fusHalfGravity(const FusAhrs* a);
V3 fusResidual(V3 sensor, V3 reference);
V3 fusHalfInclinationFeedback(FusAhrs* a, V3 halfGravity, V3 acc);
void fusRestart(FusAhrs* a);
void fusSoftRestart(FusAhrs* a);
void fusOverrange(FusAhrs* a, V3 gyro);
float fusStartup(FusAhrs* a);
void fusUpdate(FusAhrs* a, V3 gyroDps, V3 accG, float dt);
void fusInitFromRollPitch(float phi, float theta);
void fusEuler(const FusAhrs* a, float* rollDeg, float* pitchDeg);
void bmiWriteReg16(uint8_t addr, uint16_t val);
uint16_t bmiReadReg16(uint8_t addr);
void bmiBurstRead(uint8_t startAddr, int16_t* out, uint8_t numRegs);
bool bmiWriteVerify(uint8_t addr, uint16_t val);
bool rawSampleValid(const int16_t* w);
bool initBMI323();
bool calibrateBMI323(float accMeanOut[3]);
bool readIMU(ImuSample* s);
void bmiDecodeConf(uint16_t conf, BmiConf* c);
float bmiAccLsbPerG(uint8_t code);
float bmiGyrLsbPerDps(uint8_t code);
float bmiOdrHz(uint8_t code);
bool bmiProbeDrdy();
bool chol3SolveVec(const float S[3][3], const float b[3], float x[3]);
float wrapDeg180(float a);
void rotToFlightFrame(float a0, float a1, float* ox, float* oy);
uint8_t desaturateCmd(float* px, float* py, float* pz);
bool initEstimators(const float accMean[3]);
bool resyncEstimatorsFromAccel();
void resetControlState();
float pidStep(PidState* s, float err, float rateDps, float Kp, float Ki, float Kd, float dt, bool integrate);
float pidStepZ(PidState* s, float err, float rateDps, float Kp, float Ki, float Kd, float dt, bool integrate);
float yawFFTable(float thr);
void mixAndDriveServos(float pid_x, float pid_y, float pid_z);
void writeServoDeg(Servo &s, float deg, int dir);
void centerAllServos();
void centerAndHold(unsigned long hold_ms);
void runFinTestSequence();
bool ibusLinkOk(uint32_t nowMs);
void updateMotor(uint32_t nowMs, float tiltAbsDeg, bool estimatorOk);
void attemptFaultRecovery();
void initSD();
void sdFail(const char* why);
void sdEnqueue(const char* data, uint16_t n);
void updateSDLogging();
void buildRow(RowWriter* w, bool extended);
void estimatorControlTick();
void selectDt(float dtRaw, float* dtCtl, float* dtEst, bool* gap);
void finMix(float px, float py, float pz, float out[4]);
void driveFinsDirect(float px, float py, float pz);
void logEvent(const char* type, const char* msg);
void logEventf(const char* type, const char* fmt, ...);
void sdDrainNow();
float rcStick(uint16_t us);
void updateRcSetpoints(float dt);

// ============================== 7. GLOBALS ==========================
SystemState systemState = STATE_BOOT_CALIBRATING;
uint8_t faultMask = 0;
bool bmi_init_error = false, bmi_runtime_error = false;
bool sd_error = false, sd_ok = false;
bool serial1Started = false;

Servo servo1, servo2, servo3, servo4, esc;
float g_servo1_deg = 0, g_servo2_deg = 0, g_servo3_deg = 0, g_servo4_deg = 0;

// IMU scaling (decoded from the register readback at init)
float accLsbPerG = 8192.0f, gyrLsbPerDps = 65.536f, gyrFullScaleDps = 500.0f;
float gyroBiasDps[3] = {0, 0, 0};

// Estimators
Ekf ekf;
FusAhrs fusion;
Biquad fltAcc[3], fltGyr[3];
Biquad fltDX, fltDY;

// Control
PidState pidX = {0,0,0,0}, pidY = {0,0,0,0}, pidZ = {0,0,0,0};
float angleXdeg = 0, angleYdeg = 0;
float pidOutX = 0, pidOutY = 0, pidOutZ = 0;
float errXdeg = 0, errYdeg = 0;
bool  controlValid = false;
float angle_x_setpoint = 0.0, angle_y_setpoint = 0.0;
float yawThrM = 0.0f;            // motor-lag-filtered throttle fraction (yaw feedforward input)
float thrSlewF = 0.0f;           // slew-limited throttle fraction

// Motor / RC
uint16_t ibusCh[14];
uint8_t  ibusBuf[32];
uint8_t  ibusIdx = 0;
uint32_t ibusLastFrameMs = 0;
bool     ibusEverValid = false;
bool     spinOver = false;
uint32_t spinOverSinceMs = 0;
bool     motorArmed = false, armSwitchSeenLow = false, tiltTripped = false;
bool     tiltOver = false;
uint32_t tiltOverSinceMs = 0, chOkSinceMs = 0;
uint16_t motorCmdUs = ESC_MIN_US;
bool     recoverRequest = false;   // set by updateMotor, serviced by loop()

// Timing / health counters
uint32_t nextTickUs = 0, lastGoodReadUs = 0, tickCounter = 0;
uint32_t overrunCount = 0, logDrops = 0;
uint32_t lateMaxUs = 0, sdMaxWriteUs = 0;
uint16_t imuInvalidCount = 0;
bool     bmiDrdyUsable = false;
uint32_t staleSamples = 0, dupSamples = 0, badTicks = 0, curLateUs = 0, dtGapCount = 0;
uint16_t imuWaitUs = 0, consecBadMax = 0;
bool     lastSampleFresh = false;
bool     diffWarnActive = false;
float    estDiffRoll = 0, estDiffPitch = 0;
float    satX = 0, satY = 0, satZ = 0, finTgt[4] = {0, 0, 0, 0};
uint8_t  satFlags = 0;
bool     satLimX = false, satLimY = false, satLimZ = false;
bool     disagreeActive = false;
uint32_t disagreeSinceMs = 0;
float    estDiffDeg = 0;

// SD
File sdFile;
char sdFileName[16];
static uint8_t logRing[LOG_RING_SIZE] DMAMEM;
uint32_t ringHead = 0, ringTail = 0;
uint8_t  sdChunk[SD_CHUNK];
uint32_t lastSyncMs = 0;
bool     evUrgent = false;
bool     sdNeedFlush = false;

LogSnap snap;
RowWriter rw;

static char evPending[EVT_PENDING_MAX][EVT_LINE_MAX];
static uint8_t evPendingN = 0;

static const char LOG_HDR_LEGACY[] =
  "time_ms,angle_x,angle_y,rate_z,gx,gy,gz,ax,ay,az,angle_x_sp,angle_y_sp,"
  "err_x,err_y,pid_x,pid_y,pid_z,servo1,servo2,servo3,servo4";
static const char LOG_HDR_EXT[] =
  "p_x,i_x,d_x,p_y,i_y,d_y,p_z,i_z,d_z,"
  "imu_roll_x,imu_pitch_y,"
  "t_us,dt_us,raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz,"
  "cal_ax,cal_ay,cal_az,cal_gx,cal_gy,cal_gz,"
  "fus_roll_x,fus_pitch_y,fus_acc_err,fus_flags,"
  "ekf_bias_x,ekf_bias_y,ekf_sig_x,ekf_sig_y,ekf_sig_bx,ekf_sig_by,"
  "ekf_innov,ekf_acc_used,acc_norm,est_diff,"
  "sys_state,fault_mask,armed,ibus_ok,ibus_age_ms,ch_thr,ch_arm,ch1,ch2,esc_us,"
  "tick_us,late_max_us,overruns,log_drops,sd_max_us,"
  "rate_xr,rate_yr,rate_xd,rate_yd,"
  "sat_x,sat_y,sat_z,sat_flags,fin_tgt1,fin_tgt2,fin_tgt3,fin_tgt4,"
  "ekf_nis,ekf_acc_state,ekf_rscale,est_diff_roll,est_diff_pitch,"
  "dt_raw_us,late_us,stale_cnt,dup_cnt,bad_cnt,consec_bad,drdy_ok,dt_gap_cnt,"
  "gyro_ff_x,gyro_ff_y,ch4,yaw_cmd,yaw_ff_ss,yaw_ff_dyn";
static const float K_PI = 3.14159265358979f;
static const float K_DEG2RAD = 0.017453292519943295f;
static const float K_RAD2DEG = 57.29577951308232f;

// ============================== 8. HELPERS / MATH ===================
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
bool isFiniteF(float x) { return (x - x) == 0.0f; }
float wrapPi(float a) {
  if (!isFiniteF(a)) return a;
  while (a > K_PI) a -= 2.0f * K_PI;
  while (a < -K_PI) a += 2.0f * K_PI;
  return a;
}
int32_t clampI32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
float wrapDeg180(float a) {
  if (!isFiniteF(a)) return 999.0f;
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}
void selectDt(float dtRaw, float* dtCtl, float* dtEst, bool* gap) {
  *gap = !((dtRaw >= DT_MIN_S) && (dtRaw <= DT_MAX_S));
  float d = (dtRaw >= DT_MIN_S) ? dtRaw : DT_MIN_S;
  *dtCtl = d > DT_MAX_S ? DT_MAX_S : d;
  *dtEst = d > DT_EST_MAX_S ? DT_EST_MAX_S : d;
}
void rotToFlightFrame(float a0, float a1, float* ox, float* oy) {
#ifdef FLIGHT_FRAME_PLUS
  *ox = FRAME_K * (a0 - a1);
  *oy = FRAME_K * (a0 + a1);
#else
  *ox = a0; *oy = a1;
#endif
}

V3 v3make(float x, float y, float z) { V3 v; v.x = x; v.y = y; v.z = z; return v; }
V3 vAdd(V3 a, V3 b) { return v3make(a.x + b.x, a.y + b.y, a.z + b.z); }
V3 vScale(V3 a, float s) { return v3make(a.x * s, a.y * s, a.z * s); }
V3 vCross(V3 a, V3 b) {
  return v3make(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
float vDot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
float vNormSq(V3 a) { return vDot(a, a); }
bool vIsZero(V3 a) { return a.x == 0.0f && a.y == 0.0f && a.z == 0.0f; }
V3 vNormalise(V3 a) {
  float n2 = vNormSq(a);
  if (n2 < 1e-20f) return v3make(0, 0, 0);
  return vScale(a, 1.0f / sqrtf(n2));
}
Quat qAdd(Quat a, Quat b) { Quat r = {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z}; return r; }
Quat qVecProduct(Quat q, V3 v) {
  Quat r;
  r.w = -q.x * v.x - q.y * v.y - q.z * v.z;
  r.x =  q.w * v.x + q.y * v.z - q.z * v.y;
  r.y =  q.w * v.y - q.x * v.z + q.z * v.x;
  r.z =  q.w * v.z + q.x * v.y - q.y * v.x;
  return r;
}
Quat qNormalise(Quat q) {
  float n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  Quat r;
  if (n2 < 1e-20f) { r.w = 1; r.x = 0; r.y = 0; r.z = 0; return r; }
  float s = 1.0f / sqrtf(n2);
  r.w = q.w * s; r.x = q.x * s; r.y = q.y * s; r.z = q.z * s;
  return r;
}

// 2nd-order Butterworth low-pass, transposed direct form II
void biquadDesignLPF(Biquad* f, float fs, float fc) {
  float K = tanf(K_PI * fc / fs);
  float Q = 0.70710678f;
  float norm = 1.0f / (1.0f + K / Q + K * K);
  f->b0 = K * K * norm;
  f->b1 = 2.0f * f->b0;
  f->b2 = f->b0;
  f->a1 = 2.0f * (K * K - 1.0f) * norm;
  f->a2 = (1.0f - K / Q + K * K) * norm;
  f->z1 = 0; f->z2 = 0;
}
void biquadReset(Biquad* f, float x0) {
  f->z2 = (f->b2 - f->a2) * x0;
  f->z1 = (f->b1 - f->a1) * x0 + f->z2;
}
float biquadStep(Biquad* f, float x) {
  float y = f->b0 * x + f->z1;
  f->z1 = f->b1 * x - f->a1 * y + f->z2;
  f->z2 = f->b2 * x - f->a2 * y;
  return y;
}

// ============================== 9. LEDs =============================
void updateLEDs() {
  static uint32_t lastMs = 0xFFFFFFFFUL;
  uint32_t now = millis();
  if (now == lastMs) return;
  lastMs = now;
  bool statusOn = false, partsOn = false;
  switch (systemState) {
    case STATE_BOOT_CALIBRATING: statusOn = ((now / LED_CAL_BLINK_MS) % 2) == 0; break;
    case STATE_SERVO_STARTUP:    statusOn = true; break;
    case STATE_STARTUP_FAILED:   statusOn = ((now / LED_FAST_BLINK_MS) % 2) == 0; break;
    case STATE_FAULT:            statusOn = ((now / LED_FAULT_BLINK_MS) % 2) == 0; break;
    case STATE_FLIGHT:
    default:                     statusOn = (ARMED_STATUS_LED && motorArmed); break;
  }
  if (sd_error) partsOn = ((now / LED_FAST_BLINK_MS) % 2) == 0;
  else if (bmi_init_error || bmi_runtime_error) partsOn = true;
#ifdef USE_LEDS
  digitalWrite(STATUS_LED_PIN, statusOn ? HIGH : LOW);
  digitalWrite(PARTS_LED_PIN, partsOn ? HIGH : LOW);
#endif
}
void delayWithLEDs(unsigned long ms) {
  unsigned long start = millis();
  do { updateLEDs(); updateIBUS(); } while (millis() - start < ms);
}
void waitUntilUs(uint32_t t) {
  while ((int32_t)(micros() - t) < 0) { updateLEDs(); updateIBUS(); }
}
void failStartup(const char* reason) {
  systemState = STATE_STARTUP_FAILED;
  motorArmed = false;
  esc.writeMicroseconds(ESC_MIN_US);
  logEvent("ERROR", reason);
  if (!sd_ok && !sd_error) initSD();
  sdDrainNow();
  uint32_t lastPrint = 0;
  bool first = true;
  while (true) {
    updateLEDs();
    if (first || millis() - lastPrint >= 2000) {
      first = false; lastPrint = millis();
      Serial.print(F("STARTUP FAILED: ")); Serial.println(reason);
      Serial.println(F("System halted (motor 1000 us). Fix and reset."));
    }
  }
}
void latchFault(uint8_t mask, const char* msg) {
  if (faultMask & mask) return;
  faultMask |= mask;
  if (systemState == STATE_FLIGHT) systemState = STATE_FAULT;
  motorArmed = false;
  if (mask & FAULT_BMI_DATA) bmi_runtime_error = true;
  Serial.print(F("FAULT LATCHED (cycle Ch5 to recover): ")); Serial.println(msg);
  logEventf("ERROR", "FAULT LATCHED: %s", msg);
}

// ============================== 10. BMI323 DRIVER ===================
void bmiWriteReg16(uint8_t addr, uint16_t val) {
  SPI.beginTransaction(bmiSPISettings);
  digitalWrite(BMI323_CS_PIN, LOW);
  SPI.transfer(addr & 0x7F);
  SPI.transfer(val & 0xFF);
  SPI.transfer((val >> 8) & 0xFF);
  digitalWrite(BMI323_CS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(5);
}
uint16_t bmiReadReg16(uint8_t addr) {
  SPI.beginTransaction(bmiSPISettings);
  digitalWrite(BMI323_CS_PIN, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(0x00);
  uint8_t lsb = SPI.transfer(0x00);
  uint8_t msb = SPI.transfer(0x00);
  digitalWrite(BMI323_CS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(5);
  return ((uint16_t)msb << 8) | lsb;
}
void bmiBurstRead(uint8_t startAddr, int16_t* out, uint8_t numRegs) {
  SPI.beginTransaction(bmiSPISettings);
  digitalWrite(BMI323_CS_PIN, LOW);
  SPI.transfer(startAddr | 0x80);
  SPI.transfer(0x00);
  for (uint8_t i = 0; i < numRegs; i++) {
    uint8_t lsb = SPI.transfer(0x00);
    uint8_t msb = SPI.transfer(0x00);
    out[i] = (int16_t)(((uint16_t)msb << 8) | lsb);
  }
  digitalWrite(BMI323_CS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(3);
}
bool bmiWriteVerify(uint8_t addr, uint16_t val) {
  for (uint8_t i = 0; i < 5; i++) {
    bmiWriteReg16(addr, val);
    delayMicroseconds(500);
    if (bmiReadReg16(addr) == val) return true;
    delayWithLEDs(2);
  }
  return false;
}
bool rawSampleValid(const int16_t* w) {
  bool allZero = true, allOnes = true;
  for (uint8_t i = 0; i < 6; i++) {
    if (w[i] == INT16_MIN) return false;
    if (w[i] >= 32760 || w[i] <= -32760) return false;
    if (w[i] != 0) allZero = false;
    if (w[i] != -1) allOnes = false;
  }
  return !(allZero || allOnes);
}
static const float BMI_ACC_LSB_G[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
static const float BMI_GYR_LSB_DPS[5] = {262.144f, 131.072f, 65.536f, 32.768f, 16.384f};
static const float BMI_ODR_HZ_TAB[16] = {0.0f, 0.78125f, 1.5625f, 3.125f, 6.25f, 12.5f, 25.0f, 50.0f, 100.0f, 200.0f,
                                         400.0f, 800.0f, 1600.0f, 3200.0f, 6400.0f, 0.0f};
void bmiDecodeConf(uint16_t conf, BmiConf* c) {
  c->mode = (uint8_t)((conf >> 12) & 0x7);
  c->avg = (uint8_t)((conf >> 8) & 0x7);
  c->bw = (uint8_t)((conf >> 7) & 0x1);
  c->range = (uint8_t)((conf >> 4) & 0x7);
  c->odr = (uint8_t)(conf & 0xF);
}
float bmiAccLsbPerG(uint8_t code) { return code < 4 ? BMI_ACC_LSB_G[code] : 0.0f; }
float bmiGyrLsbPerDps(uint8_t code) { return code < 5 ? BMI_GYR_LSB_DPS[code] : 0.0f; }
float bmiOdrHz(uint8_t code) { return BMI_ODR_HZ_TAB[code & 0xF]; }
bool bmiProbeDrdy() {
  int16_t w[6];
  uint8_t setG = 0, setA = 0, clrG = 0, clrA = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 80) {
    uint16_t st = bmiReadReg16(REG_STATUS);
    bool g = (st & STATUS_DRDY_GYR) != 0, a = (st & STATUS_DRDY_ACC) != 0;
    if (g) setG++;
    if (a) setA++;
    if (g || a) {
      bmiBurstRead(REG_ACC_DATA_X, w, 6);
      uint16_t st2 = bmiReadReg16(REG_STATUS);
      if (g && !(st2 & STATUS_DRDY_GYR)) clrG++;
      if (a && !(st2 & STATUS_DRDY_ACC)) clrA++;
    }
    if (setG >= 6 && setA >= 6 && clrG >= 5 && clrA >= 5) return true;
    updateLEDs();
  }
  return false;
}

bool initBMI323() {
  pinMode(SD_CS_PIN, OUTPUT); digitalWrite(SD_CS_PIN, HIGH);
  pinMode(BMI323_CS_PIN, OUTPUT); digitalWrite(BMI323_CS_PIN, HIGH);
  SPI.begin();
  delayWithLEDs(10);
  for (uint8_t attempt = 1; attempt <= BMI_INIT_ATTEMPTS; attempt++) {
    Serial.print(F("BMI323 init attempt ")); Serial.println(attempt);
    (void)bmiReadReg16(REG_CHIP_ID);
    delayWithLEDs(2);
    uint16_t id = bmiReadReg16(REG_CHIP_ID) & 0xFF;
    Serial.print(F("  chip_id=0x")); Serial.println(id, HEX);
    logEventf("INFO", "BMI323 init attempt %u chip_id=0x%02X", attempt, (unsigned)id);
    if (id != BMI323_EXPECTED_CHIP_ID) { delayWithLEDs(20); continue; }
    bmiWriteReg16(REG_CMD, CMD_SOFT_RESET);
    delayWithLEDs(60);
    (void)bmiReadReg16(REG_CHIP_ID);
    delayWithLEDs(2);
    id = bmiReadReg16(REG_CHIP_ID) & 0xFF;
    if (id != BMI323_EXPECTED_CHIP_ID) {
      logEvent("WARN", "BMI323/cal: chip_id bad after reset");
      Serial.println(F("  chip_id bad after reset")); continue;
    }
    uint16_t status = bmiReadReg16(REG_STATUS);
    Serial.print(F("  STATUS=0x")); Serial.println(status, HEX);
    if (!(status & 0x0001)) { logEvent("WARN", "BMI323/cal: init-OK bit not set"); Serial.println(F("  init-OK bit not set")); continue; }
    if (!bmiWriteVerify(REG_ACC_CONF, ACC_CONF_VALUE)) { logEvent("WARN", "BMI323/cal: ACC_CONF verify failed"); Serial.println(F("  ACC_CONF verify failed")); continue; }
    if (!bmiWriteVerify(REG_GYR_CONF, GYR_CONF_VALUE)) { logEvent("WARN", "BMI323/cal: GYR_CONF verify failed"); Serial.println(F("  GYR_CONF verify failed")); continue; }
    delayWithLEDs(150);
    uint16_t accC = bmiReadReg16(REG_ACC_CONF), gyrC = bmiReadReg16(REG_GYR_CONF);
    BmiConf ac, gc;
    bmiDecodeConf(accC, &ac); bmiDecodeConf(gyrC, &gc);
    float aL = bmiAccLsbPerG(ac.range), gL = bmiGyrLsbPerDps(gc.range);
    bool cfgOk = aL > 0.0f && gL > 0.0f && ac.mode == BMI_MODE && gc.mode == BMI_MODE &&
                 ac.odr == BMI_ODR_CODE && gc.odr == BMI_ODR_CODE && ac.bw == BMI_BW_QUARTER && gc.bw == BMI_BW_QUARTER &&
                 ac.range == BMI_ACC_RANGE && gc.range == BMI_GYR_RANGE;
    if (!cfgOk) {
      logEventf("WARN", "BMI323 cfg readback mismatch acc=0x%04X gyr=0x%04X", (unsigned)accC, (unsigned)gyrC);
      Serial.println(F("  config readback does not match what was written")); continue;
    }
    accLsbPerG = aL; gyrLsbPerDps = gL; gyrFullScaleDps = 32768.0f / gyrLsbPerDps;
    Serial.print(F("  ACC_CONF=0x")); Serial.print(accC, HEX);
    Serial.print(F(" GYR_CONF=0x")); Serial.print(gyrC, HEX);
    Serial.print(F("  acc LSB/g=")); Serial.print(accLsbPerG, 1);
    Serial.print(F(" gyr LSB/dps=")); Serial.println(gyrLsbPerDps, 3);
    logEventf("INFO", "BMI cfg acc=0x%04X gyr=0x%04X odr=%.0fHz bw=ODR/%d", (unsigned)accC, (unsigned)gyrC, (double)bmiOdrHz(gc.odr), gc.bw ? 4 : 2);
    logEventf("INFO", "BMI scale acc=%.0f LSB/g gyr=%.3f LSB/dps (full scale %.0f dps)", (double)accLsbPerG, (double)gyrLsbPerDps, (double)gyrFullScaleDps);
    bool got = false;
    uint32_t t0 = millis();
    while (millis() - t0 < 300) {
      int16_t w[6]; bmiBurstRead(REG_ACC_DATA_X, w, 6);
      if (rawSampleValid(w)) { got = true; break; }
      delayWithLEDs(5);
    }
    if (!got) { logEvent("WARN", "BMI323/cal: no valid sample after config"); Serial.println(F("  no valid sample after config")); continue; }
    bmiDrdyUsable = bmiProbeDrdy();
    Serial.print(F("  data-ready polling: ")); Serial.println(bmiDrdyUsable ? F("OK") : F("unavailable (repeat detection only)"));
    logEventf("INFO", "BMI323 data-ready polling %s", bmiDrdyUsable ? "OK" : "unavailable, repeat detection only");
    bmi_init_error = false;
    Serial.println(F("BMI323 init OK.")); logEvent("INFO", "BMI323 init OK");
    return true;
  }
  bmi_init_error = true; logEvent("ERROR", "BMI323 init failed after all attempts");
  return false;
}

bool readIMU(ImuSample* s) {
  static int16_t prevRaw[6] = {0, 0, 0, 0, 0, 0};
  static bool prevValid = false;
  static uint16_t frozenRun = 0;
  s->fresh = true; s->waitUs = 0; s->frozenRun = 0;
  if (bmiDrdyUsable) {
    uint16_t st = bmiReadReg16(REG_STATUS);
    if (!(st & STATUS_DRDY_GYR)) {
      uint32_t t0 = micros();
      while ((uint32_t)(micros() - t0) < (uint32_t)IMU_DRDY_WAIT_US) {
        st = bmiReadReg16(REG_STATUS);
        if (st & STATUS_DRDY_GYR) break;
      }
      uint32_t w = micros() - t0; s->waitUs = (uint16_t)(w > 65535UL ? 65535UL : w);
    }
    s->fresh = (st & STATUS_DRDY_GYR) != 0;
  }
  bmiBurstRead(REG_ACC_DATA_X, s->raw, 6);
  s->tUs = micros();
  if (!rawSampleValid(s->raw)) return false;
  if (prevValid && memcmp(prevRaw, s->raw, sizeof(prevRaw)) == 0) {
    dupSamples++; s->fresh = false;
    if (frozenRun < 65535) frozenRun++;
  } else frozenRun = 0;
  s->frozenRun = frozenRun;
  memcpy(prevRaw, s->raw, sizeof(prevRaw)); prevValid = true;
  for (uint8_t i = 0; i < 3; i++) {
    s->acc[i] = (float)s->raw[i] / accLsbPerG;
    s->gyr[i] = (float)s->raw[3 + i] / gyrLsbPerDps - gyroBiasDps[i];
  }
  return true;
}

bool calibrateBMI323(float accMeanOut[3]) {
  Serial.println(F("Calibrating gyro (~5 s): keep the rig STILL and UPRIGHT..."));
  for (uint8_t attempt = 1; attempt <= CAL_MAX_ATTEMPTS; attempt++) {
    double sg[3] = {0, 0, 0}, sg2[3] = {0, 0, 0}, sa[3] = {0, 0, 0}, sa2[3] = {0, 0, 0};
    uint32_t good = 0;
    for (uint8_t i = 0; i < 3; i++) gyroBiasDps[i] = 0.0f;
    uint32_t settleTicks = ((uint32_t)CAL_SETTLE_MS * EST_HZ) / 1000UL;
    uint32_t next = micros() + EST_PERIOD_US;
    for (uint32_t k = 0; k < settleTicks + CAL_SAMPLES; k++) {
      waitUntilUs(next); next += EST_PERIOD_US;
      ImuSample smp; bool v = readIMU(&smp);
      if (k < settleTicks) continue;
      if (!v || !smp.fresh) continue;
      good++;
      for (uint8_t i = 0; i < 3; i++) {
        double g = (double)smp.gyr[i], a = (double)smp.acc[i];
        sg[i] += g; sg2[i] += g * g; sa[i] += a; sa2[i] += a * a;
      }
    }
    if (good < (uint32_t)(CAL_SAMPLES * 0.95f)) {
      logEventf("WARN", "BMI323/cal: too many invalid/stale samples (%lu of %d)", (unsigned long)good, CAL_SAMPLES);
      Serial.println(F("  cal: too many invalid/stale samples")); continue;
    }
    float mean[3], stdv[3], am[3], astd[3]; bool ok = true;
    for (uint8_t i = 0; i < 3; i++) {
      mean[i] = (float)(sg[i] / good);
      double var = sg2[i] / good - (double)mean[i] * mean[i]; stdv[i] = (float)sqrt(var > 0.0 ? var : 0.0);
      am[i] = (float)(sa[i] / good);
      double avar = sa2[i] / good - (double)am[i] * am[i]; astd[i] = (float)sqrt(avar > 0.0 ? avar : 0.0);
      if (stdv[i] > CAL_GYRO_STD_MAX_DPS || fabsf(mean[i]) > CAL_GYRO_BIAS_MAX_DPS) ok = false;
      if (astd[i] > CAL_ACC_STD_MAX_G) ok = false;
    }
    float an = sqrtf(am[0] * am[0] + am[1] * am[1] + am[2] * am[2]);
    if (an < CAL_ACC_NORM_MIN || an > CAL_ACC_NORM_MAX) ok = false;
    Serial.print(F("  attempt ")); Serial.print(attempt); Serial.print(F(": bias="));
    Serial.print(mean[0], 4); Serial.print(','); Serial.print(mean[1], 4); Serial.print(','); Serial.print(mean[2], 4);
    Serial.print(F(" dps  std=")); Serial.print(stdv[0], 3); Serial.print(','); Serial.print(stdv[1], 3); Serial.print(','); Serial.print(stdv[2], 3);
    Serial.print(F("  accStd=")); Serial.print(astd[0], 4); Serial.print(','); Serial.print(astd[1], 4); Serial.print(','); Serial.print(astd[2], 4);
    Serial.print(F("  |acc|=")); Serial.print(an, 3); Serial.println(ok ? F(" g OK") : F(" g REJECTED"));
    if (!ok) {
      logEventf("WARN", "Gyro cal attempt %u rejected: gyroStd=%.2f,%.2f,%.2f accStd=%.3f,%.3f,%.3f", attempt,
                (double)stdv[0], (double)stdv[1], (double)stdv[2], (double)astd[0], (double)astd[1], (double)astd[2]);
      continue;
    }
    for (uint8_t i = 0; i < 3; i++) { gyroBiasDps[i] = mean[i]; accMeanOut[i] = am[i]; }
    logEventf("INFO", "Gyro cal OK bias=%.3f,%.3f,%.3f dps |acc|=%.3f g", mean[0], mean[1], mean[2], an);
    logEventf("INFO", "Cal noise gyroStd=%.2f,%.2f,%.2f dps accStd=%.3f,%.3f,%.3f g", (double)stdv[0], (double)stdv[1], (double)stdv[2],
              (double)astd[0], (double)astd[1], (double)astd[2]);
    return true;
  }
  return false;
}

// ============================== 11. EKF =============================
void ekfInit(float phi, float theta) {
  memset(&ekf, 0, sizeof(ekf)); ekf.x[0] = phi; ekf.x[1] = theta;
  float sa = EKF_P0_ANGLE_DEG * K_DEG2RAD, sb = EKF_P0_BIAS_DPS * K_DEG2RAD;
  ekf.P[0][0] = sa * sa; ekf.P[1][1] = sa * sa; ekf.P[2][2] = sb * sb; ekf.P[3][3] = sb * sb;
}
void ekfPredict(float gx, float gy, float gz, float dt) {
  float phi = ekf.x[0], th = ekf.x[1];
  float p = gx - ekf.x[2], q = gy - ekf.x[3], r = gz;
  float thc = clampf(th, -EKF_THETA_MAX_RAD, EKF_THETA_MAX_RAD);
  float sp = sinf(phi), cp = cosf(phi), ct = cosf(thc), tt = tanf(thc), sec2 = 1.0f / (ct * ct);
  float phiDot = p + sp * tt * q + cp * tt * r;
  float thDot = cp * q - sp * r;
  float F[4][4] = {
    {1.0f + dt * (cp * tt * q - sp * tt * r), dt * (sp * q + cp * r) * sec2, -dt, -dt * sp * tt},
    {dt * (-sp * q - cp * r), 1.0f, 0.0f, -dt * cp},
    {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}
  };
  ekf.x[0] = wrapPi(phi + dt * phiDot);
  ekf.x[1] = clampf(th + dt * thDot, -EKF_THETA_MAX_RAD, EKF_THETA_MAX_RAD);
  float qa = EKF_SIGMA_GYRO_DPS * K_DEG2RAD * dt; qa *= qa;
  float qb = EKF_SIGMA_BIAS_DPS_RTS * K_DEG2RAD; qb = qb * qb * dt;
  float FP[4][4], Pn[4][4];
  for (uint8_t i = 0; i < 4; i++) for (uint8_t j = 0; j < 4; j++) {
    float s = 0; for (uint8_t k = 0; k < 4; k++) s += F[i][k] * ekf.P[k][j]; FP[i][j] = s;
  }
  for (uint8_t i = 0; i < 4; i++) for (uint8_t j = 0; j < 4; j++) {
    float s = 0; for (uint8_t k = 0; k < 4; k++) s += FP[i][k] * F[j][k]; Pn[i][j] = s;
  }
  Pn[0][0] += qa; Pn[1][1] += qa; Pn[2][2] += qb; Pn[3][3] += qb;
  for (uint8_t i = 0; i < 4; i++) for (uint8_t j = i; j < 4; j++) {
    float v = 0.5f * (Pn[i][j] + Pn[j][i]); ekf.P[i][j] = v; ekf.P[j][i] = v;
  }
}

bool chol3Solve(const float S[3][3], const float HP[3][4], float X[3][4]) {
  float L00 = S[0][0]; if (!(L00 > 1e-12f)) return false; L00 = sqrtf(L00);
  float L10 = S[1][0] / L00, L20 = S[2][0] / L00;
  float d1 = S[1][1] - L10 * L10; if (!(d1 > 1e-12f)) return false; float L11 = sqrtf(d1);
  float L21 = (S[2][1] - L20 * L10) / L11;
  float d2 = S[2][2] - L20 * L20 - L21 * L21; if (!(d2 > 1e-12f)) return false; float L22 = sqrtf(d2);
  for (uint8_t c = 0; c < 4; c++) {
    float y0 = HP[0][c] / L00, y1 = (HP[1][c] - L10 * y0) / L11;
    float y2 = (HP[2][c] - L20 * y0 - L21 * y1) / L22;
    float x2 = y2 / L22, x1 = (y1 - L21 * x2) / L11, x0 = (y0 - L10 * x1) / L00;
    X[0][c] = x0; X[1][c] = x1; X[2][c] = x2;
  }
  return true;
}
bool chol3SolveVec(const float S[3][3], const float b[3], float x[3]) {
  float L00 = S[0][0]; if (!(L00 > 1e-12f)) return false; L00 = sqrtf(L00);
  float L10 = S[1][0] / L00, L20 = S[2][0] / L00;
  float d1 = S[1][1] - L10 * L10; if (!(d1 > 1e-12f)) return false; float L11 = sqrtf(d1);
  float L21 = (S[2][1] - L20 * L10) / L11;
  float d2 = S[2][2] - L20 * L20 - L21 * L21; if (!(d2 > 1e-12f)) return false; float L22 = sqrtf(d2);
  float y0 = b[0] / L00, y1 = (b[1] - L10 * y0) / L11, y2 = (b[2] - L20 * y0 - L21 * y1) / L22;
  float x2 = y2 / L22, x1 = (y1 - L21 * x2) / L11, x0 = (y0 - L10 * x1) / L00;
  x[0] = x0; x[1] = x1; x[2] = x2; return true;
}
bool ekfUpdate(const float acc[3]) {
  ekf.accUsed = false;
  float n = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]); ekf.accNorm = n;
  float dev = fabsf(n - 1.0f);
  if (!(n > 0.5f) || dev > EKF_ACC_GATE_G) { ekf.accState = EKF_ACC_REJ_MAG; return false; }
  float z[3] = {acc[0] / n, acc[1] / n, acc[2] / n};
  float phi = ekf.x[0], th = ekf.x[1], sp = sinf(phi), cp = cosf(phi), st = sinf(th), ct = cosf(th);
  float h[3] = {-st, ct * sp, ct * cp};
  float H[3][4] = {{0.0f, -ct, 0.0f, 0.0f}, {ct * cp, -st * sp, 0.0f, 0.0f}, {-ct * sp, -st * cp, 0.0f, 0.0f}};
  float R0 = EKF_SIGMA_ACC * EKF_SIGMA_ACC;
  R0 *= 1.0f + (dev * dev) / (EKF_ACC_DEV_SOFT_G * EKF_ACC_DEV_SOFT_G);
  float HP[3][4], HPHt[3][3], S[3][3], X[3][4], K[4][3];
  for (uint8_t i = 0; i < 3; i++) for (uint8_t j = 0; j < 4; j++) {
    float ss = 0; for (uint8_t k = 0; k < 4; k++) ss += H[i][k] * ekf.P[k][j]; HP[i][j] = ss;
  }
  for (uint8_t i = 0; i < 3; i++) for (uint8_t j = 0; j < 3; j++) {
    float ss = 0; for (uint8_t k = 0; k < 4; k++) ss += HP[i][k] * H[j][k];
    HPHt[i][j] = ss; S[i][j] = ss + (i == j ? R0 : 0.0f);
  }
  float y[3] = {z[0] - h[0], z[1] - h[1], z[2] - h[2]}, v[3];
  if (!chol3SolveVec(S, y, v)) { ekf.accState = EKF_ACC_REJ_NUM; return false; }
  float nis = y[0] * v[0] + y[1] * v[1] + y[2] * v[2];
  if (!isFiniteF(nis)) { ekf.accState = EKF_ACC_REJ_NUM; return false; }
  ekf.nis = nis;
  const uint16_t recoverTicks = (uint16_t)(EKF_REJECT_RECOVER_S * (float)EST_HZ);
  float rscale = 1.0f; uint8_t state = EKF_ACC_OK;
  if (nis > EKF_NIS_HARD) {
    if (ekf.rejStreak < recoverTicks) { ekf.rejStreak++; ekf.accState = EKF_ACC_REJ_DIR; return false; }
    state = EKF_ACC_RECOVER;
  } else if (nis > EKF_NIS_SOFT) state = EKF_ACC_SOFT;
  if (nis > EKF_NIS_SOFT) rscale = nis / EKF_NIS_SOFT;
  float R = R0 * rscale;
  for (uint8_t i = 0; i < 3; i++) for (uint8_t j = 0; j < 3; j++) S[i][j] = HPHt[i][j] + (i == j ? R : 0.0f);
  if (!chol3Solve(S, HP, X)) { ekf.accState = EKF_ACC_REJ_NUM; return false; }
  for (uint8_t i = 0; i < 4; i++) for (uint8_t j = 0; j < 3; j++) K[i][j] = X[j][i];
  ekf.innovNorm = sqrtf(y[0] * y[0] + y[1] * y[1] + y[2] * y[2]);
  float xn[4];
  for (uint8_t i = 0; i < 4; i++) xn[i] = ekf.x[i] + K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];
  for (uint8_t i = 0; i < 4; i++) if (!isFiniteF(xn[i])) { ekf.accState = EKF_ACC_REJ_NUM; return false; }
  float A[4][4], B[4][4], Pn[4][4];
  for (uint8_t i = 0; i < 4; i++) for (uint8_t j = 0; j < 4; j++) {
    float ss = 0; for (uint8_t m = 0; m < 3; m++) ss += K[i][m] * H[m][j]; A[i][j] = (i == j ? 1.0f : 0.0f) - ss;
  }
  for (uint8_t i = 0; i < 4; i++) for (uint8_t j = 0; j < 4; j++) {
    float ss = 0; for (uint8_t k = 0; k < 4; k++) ss += A[i][k] * ekf.P[k][j]; B[i][j] = ss;
  }
  for (uint8_t i = 0; i < 4; i++) for (uint8_t j = 0; j < 4; j++) {
    float ss = 0, kr = 0;
    for (uint8_t k = 0; k < 4; k++) ss += B[i][k] * A[j][k];
    for (uint8_t m = 0; m < 3; m++) kr += K[i][m] * K[j][m];
    Pn[i][j] = ss + R * kr;
  }
  for (uint8_t i = 0; i < 4; i++) for (uint8_t j = i; j < 4; j++) {
    float vv = 0.5f * (Pn[i][j] + Pn[j][i]); ekf.P[i][j] = vv; ekf.P[j][i] = vv;
  }
  for (uint8_t i = 0; i < 4; i++) ekf.x[i] = xn[i];
  ekf.x[0] = wrapPi(ekf.x[0]); ekf.x[1] = clampf(ekf.x[1], -EKF_THETA_MAX_RAD, EKF_THETA_MAX_RAD);
  ekf.x[2] = clampf(ekf.x[2], -EKF_BIAS_MAX_DPS * K_DEG2RAD, EKF_BIAS_MAX_DPS * K_DEG2RAD);
  ekf.x[3] = clampf(ekf.x[3], -EKF_BIAS_MAX_DPS * K_DEG2RAD, EKF_BIAS_MAX_DPS * K_DEG2RAD);
  ekf.rejStreak = 0; ekf.rScale = rscale; ekf.accState = state; ekf.accUsed = true;
  return true;
}
bool ekfHealthy() {
  for (uint8_t i = 0; i < 4; i++) {
    if (!isFiniteF(ekf.x[i])) return false;
    for (uint8_t j = 0; j < 4; j++) if (!isFiniteF(ekf.P[i][j])) return false;
    if (ekf.P[i][i] < 0.0f) return false;
  }
  float aMax = 25.0f * K_DEG2RAD, bMax = 5.0f * K_DEG2RAD;
  if (ekf.P[0][0] > aMax * aMax || ekf.P[1][1] > aMax * aMax) return false;
  if (ekf.P[2][2] > bMax * bMax || ekf.P[3][3] > bMax * bMax) return false;
  return true;
}

// ============================== 12. FUSION AHRS (port) ==============
V3 fusHalfGravity(const FusAhrs* a) {
  const Quat& Q = a->q;
  return v3make(Q.x * Q.z - Q.w * Q.y, Q.y * Q.z + Q.w * Q.x, Q.w * Q.w - 0.5f + Q.z * Q.z);
}
V3 fusResidual(V3 sensor, V3 reference) {
  V3 cross = vCross(sensor, reference);
  if (vDot(sensor, reference) > 0.0f) return cross;
  if (vIsZero(cross)) return v3make(0, 0, 0);
  return vNormalise(cross);
}
V3 fusHalfInclinationFeedback(FusAhrs* a, V3 halfGravity, V3 acc) {
  V3 fb = v3make(0, 0, 0); a->accIgnored = true;
  if (!vIsZero(acc)) {
    a->halfAccResidual = fusResidual(vNormalise(acc), halfGravity);
    if (a->startup || (vNormSq(a->halfAccResidual) <= a->accelerationRejection)) { a->accIgnored = false; a->accRecoveryTrigger -= 9; }
    else a->accRecoveryTrigger += 1;
    if (a->accRecoveryTrigger > a->accRecoveryThreshold) { a->accRecoveryThreshold = 0; a->accIgnored = false; }
    else a->accRecoveryThreshold = a->rejectionTimeout;
    a->accRecoveryTrigger = clampI32(a->accRecoveryTrigger, 0, a->rejectionTimeout);
    if (!a->accIgnored) fb = a->halfAccResidual;
  }
  return fb;
}
void fusRestart(FusAhrs* a) {
  a->q.w = 1; a->q.x = 0; a->q.y = 0; a->q.z = 0; a->accel = v3make(0, 0, 0);
  a->startup = true; a->startupGain = FUS_INIT_STARTUP_GAIN; a->overrangeRecovery = false;
  a->halfAccResidual = v3make(0, 0, 0); a->accRecoveryTrigger = 0;
  a->accRecoveryThreshold = a->rejectionTimeout; a->accIgnored = false;
}
void fusSoftRestart(FusAhrs* a) { Quat q = a->q; V3 acc = a->accel; fusRestart(a); a->q = q; a->accel = acc; }
void fusOverrange(FusAhrs* a, V3 gyro) {
  if (!a->overrangeEnabled) return;
  if (fabsf(gyro.x) <= a->overrangeThreshold && fabsf(gyro.y) <= a->overrangeThreshold && fabsf(gyro.z) <= a->overrangeThreshold) return;
  fusSoftRestart(a); a->overrangeRecovery = true;
}
float fusStartup(FusAhrs* a) {
  if (!a->startup) return a->gain;
  a->startupGain -= a->startupGainRate;
  if (a->startupGain > a->gain) return a->startupGain;
  a->startup = false; a->overrangeRecovery = false; return a->gain;
}
void fusUpdate(FusAhrs* a, V3 gyroDps, V3 accG, float dt) {
  a->samplePeriod = dt; a->accel = accG; fusOverrange(a, gyroDps);
  float gain = fusStartup(a); V3 halfGyro = vScale(gyroDps, 0.5f * K_DEG2RAD);
  V3 halfGravity = fusHalfGravity(a), halfFb = fusHalfInclinationFeedback(a, halfGravity, accG);
  V3 halfRate = vAdd(halfGyro, vScale(halfFb, gain));
  a->q = qNormalise(qAdd(a->q, qVecProduct(a->q, vScale(halfRate, a->samplePeriod))));
}
void fusEuler(const FusAhrs* a, float* rollDeg, float* pitchDeg) {
  const Quat& Q = a->q;
  *rollDeg = K_RAD2DEG * atan2f(Q.y * Q.z + Q.w * Q.x, Q.w * Q.w + Q.z * Q.z - 0.5f);
  *pitchDeg = K_RAD2DEG * asinf(clampf(2.0f * (Q.w * Q.y - Q.x * Q.z), -1.0f, 1.0f));
}
void fusInitFromRollPitch(float phi, float theta) {
  fusion.samplePeriod = 1.0f / EST_HZ; fusion.gain = FUS_GAIN;
  fusion.startupGainRate = ((FUS_INIT_STARTUP_GAIN - FUS_GAIN) / FUS_STARTUP_PERIOD_S) * fusion.samplePeriod;
  fusion.overrangeEnabled = true; fusion.overrangeThreshold = 0.98f * gyrFullScaleDps;
  float s = 0.5f * sinf(FUS_ACC_REJECT_DEG * K_DEG2RAD);
  fusion.accelerationRejection = s * s; fusion.rejectionTimeout = (int32_t)(EST_HZ * FUS_REJECT_TIMEOUT_S);
  fusRestart(&fusion);
  float cp = cosf(0.5f * phi), sp = sinf(0.5f * phi), ct = cosf(0.5f * theta), st = sinf(0.5f * theta);
  fusion.q.w = ct * cp; fusion.q.x = ct * sp; fusion.q.y = st * cp; fusion.q.z = -st * sp;
  fusion.startup = false; fusion.overrangeRecovery = false;
}

// ============================== 13. ESTIMATOR INIT / CONTROL STATE ==
void resetControlState() {
  pidX.integral = pidY.integral = pidZ.integral = 0;
  pidX.p = pidX.i = pidX.d = pidY.p = pidY.i = pidY.d = pidZ.p = pidZ.i = pidZ.d = 0;
  pidOutX = pidOutY = pidOutZ = 0;
  yawThrM = 0.0f; thrSlewF = 0.0f;
}
bool initEstimators(const float accMean[3]) {
  float ax = accMean[0], ay = accMean[1], az = accMean[2];
  float phi = atan2f(ay, az), theta = atan2f(-ax, sqrtf(ay * ay + az * az));
  float tilt = fmaxf(fabsf(phi), fabsf(theta)) * K_RAD2DEG;
  Serial.print(F("Initial attitude: roll_x=")); Serial.print(phi * K_RAD2DEG, 2);
  Serial.print(F(" pitch_y=")); Serial.print(theta * K_RAD2DEG, 2); Serial.println(F(" deg"));
  logEventf("INFO", "Initial attitude roll_x=%.2f pitch_y=%.2f deg", phi * K_RAD2DEG, theta * K_RAD2DEG);
  if (tilt > STARTUP_TILT_FATAL_DEG) {
    Serial.println(F("Tilt too large at startup (rig not upright or sensor axes wrong)."));
    logEvent("ERROR", "Tilt too large at startup"); return false;
  }
  if (tilt > TILT_LIMIT_DEG) { Serial.println(F("WARNING: initial tilt exceeds TILT_LIMIT_DEG.")); logEvent("WARN", "Initial tilt exceeds TILT_LIMIT_DEG"); }
  ekfInit(phi, theta); fusInitFromRollPitch(phi, theta);
  for (uint8_t i = 0; i < 3; i++) {
    biquadDesignLPF(&fltAcc[i], (float)EST_HZ, ACC_LPF_HZ); biquadDesignLPF(&fltGyr[i], (float)EST_HZ, GYR_LPF_HZ);
    biquadReset(&fltAcc[i], accMean[i]); biquadReset(&fltGyr[i], 0.0f);
  }
  biquadDesignLPF(&fltDX, (float)EST_HZ, D_TERM_LPF_HZ > 0.0f ? D_TERM_LPF_HZ : 0.45f * EST_HZ);
  biquadDesignLPF(&fltDY, (float)EST_HZ, D_TERM_LPF_HZ > 0.0f ? D_TERM_LPF_HZ : 0.45f * EST_HZ);
  biquadReset(&fltDX, 0.0f); biquadReset(&fltDY, 0.0f); resetControlState();
  satLimX = satLimY = satLimZ = false; satFlags = 0;
  angleXdeg = phi * K_RAD2DEG - ANGLE_TRIM_X_DEG; angleYdeg = theta * K_RAD2DEG - ANGLE_TRIM_Y_DEG;
  return true;
}
bool resyncEstimatorsFromAccel() {
  double sa[3] = {0, 0, 0}; uint16_t good = 0;
  uint32_t next = micros() + EST_PERIOD_US;
  for (uint16_t k = 0; k < 100; k++) {
    waitUntilUs(next); next += EST_PERIOD_US; ImuSample s;
    if (!readIMU(&s) || !s.fresh) continue;
    good++; for (uint8_t i = 0; i < 3; i++) sa[i] += s.acc[i];
  }
  if (good < 80) { Serial.println(F("Resync: too few valid samples.")); logEvent("ERROR", "Resync: too few valid IMU samples"); return false; }
  float am[3]; for (uint8_t i = 0; i < 3; i++) am[i] = (float)(sa[i] / good);
  float an = sqrtf(am[0] * am[0] + am[1] * am[1] + am[2] * am[2]);
  if (an < CAL_ACC_NORM_MIN || an > CAL_ACC_NORM_MAX) { Serial.println(F("Resync: |acc| out of range.")); logEvent("ERROR", "Resync: |acc| out of range"); return false; }
  return initEstimators(am);
}

// ============================== 14. CONTROL / SERVOS ================
// X/Y PID: the I-term OUTPUT (Ki * integral) is limited to +/-I_OUT_LIMIT_DEG of fin authority.
float pidStep(PidState* s, float err, float rateDps, float Kp, float Ki, float Kd, float dt, bool integrate) {
  if (Ki > 0.0f) {
    if (integrate) s->integral += err * dt;
    const float lim = I_OUT_LIMIT_DEG / Ki;
    s->integral = clampf(s->integral, -lim, lim);
  }
  s->p = Kp * err; s->i = Ki * s->integral; s->d = -Kd * rateDps;
  return s->p + s->i + s->d;
}
// Yaw-rate PI: the I-term OUTPUT is limited to +/-I_OUT_LIMIT_Z_DEG.
float pidStepZ(PidState* s, float err, float rateDps, float Kp, float Ki, float Kd, float dt, bool integrate) {
  if (Ki > 0.0f) {
    if (integrate) s->integral += err * dt;
    const float lim = I_OUT_LIMIT_Z_DEG / Ki;
    s->integral = clampf(s->integral, -lim, lim);
  }
  s->p = Kp * err; s->i = Ki * s->integral; s->d = -Kd * rateDps;
  return s->p + s->i + s->d;
}
// Steady yaw feedforward (fin degrees) from the measured table, piecewise linear, held above the last point.
float yawFFTable(float thr) {
  if (thr <= YAW_FF_TBL_THR[0]) return 0.0f;
  if (thr >= YAW_FF_TBL_THR[YAW_FF_N - 1]) return YAW_FF_TBL_DEG[YAW_FF_N - 1];
  for (uint8_t i = 1; i < YAW_FF_N; i++) {
    if (thr <= YAW_FF_TBL_THR[i]) {
      float k = (thr - YAW_FF_TBL_THR[i - 1]) / (YAW_FF_TBL_THR[i] - YAW_FF_TBL_THR[i - 1]);
      return YAW_FF_TBL_DEG[i - 1] + k * (YAW_FF_TBL_DEG[i] - YAW_FF_TBL_DEG[i - 1]);
    }
  }
  return 0.0f;
}
float rcStick(uint16_t us) {
  float v = (float)((int)us - RC_CENTER_US), db = (float)RC_DEADBAND_US;
  if (fabsf(v) <= db) return 0.0f;
  float m = (fabsf(v) - db) / (500.0f - db); m = clampf(m, 0.0f, 1.0f);
  return v < 0 ? -m : m;
}
void updateRcSetpoints(float dt) {
  float tx = 0.0f, ty = 0.0f;
#ifdef RC_ATTITUDE_CONTROL
  if (motorArmed && ibusLinkOk(millis())) {
    uint16_t c1 = ibusCh[IBUS_CH_YAW], c2 = ibusCh[IBUS_CH_PITCH];
    if (c1 >= 800 && c1 <= 2200 && c2 >= 800 && c2 <= 2200) {
      float sx = RC_SIGN_X * rcStick(c1), sy = RC_SIGN_Y * rcStick(c2);
      float m = sqrtf(sx * sx + sy * sy); if (m > 1.0f) { sx /= m; sy /= m; }
      ty = sx * RC_MAX_ANGLE_DEG; tx = -sy * RC_MAX_ANGLE_DEG;
    }
  }
#endif
  float step = RC_SETPOINT_SLEW_DPS * dt;
  angle_x_setpoint += clampf(tx - angle_x_setpoint, -step, step);
  angle_y_setpoint += clampf(ty - angle_y_setpoint, -step, step);
}
void writeServoDeg(Servo &s, float deg, int dir) {
  int us = SERVO_CENTER_US + (int)lroundf(dir * deg * US_PER_DEG);
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US); s.writeMicroseconds(us);
}
void finMix(float px, float py, float pz, float out[4]) {
#ifdef FLIGHT_FRAME_PLUS
  out[0] = py + pz; out[1] = px + pz; out[2] = -py + pz; out[3] = -px + pz;
#else
  out[0] = FIN_MIX_K * (px + py) + pz;
  out[1] = FIN_MIX_K * (px - py) + pz;
  out[2] = FIN_MIX_K * (-px - py) + pz;
  out[3] = FIN_MIX_K * (-px + py) + pz;
#endif
}
void driveFinsDirect(float px, float py, float pz) {
  float m[4]; finMix(px, py, pz, m);
  writeServoDeg(servo1, m[0] + offset1, dir1); writeServoDeg(servo2, m[1] + offset2, dir2);
  writeServoDeg(servo3, m[2] + offset3, dir3); writeServoDeg(servo4, m[3] + offset4, dir4);
}
uint8_t desaturateCmd(float* px, float* py, float* pz) {
#ifdef FLIGHT_FRAME_PLUS
  const float L = MAX_FIN_DEG; uint8_t fl = 0;
  float yLo = fmaxf(-L - offset1, offset3 - L), yHi = fminf(L - offset1, L + offset3);
  float xLo = fmaxf(-L - offset2, offset4 - L), xHi = fminf(L - offset2, L + offset4);
  if (yLo > yHi) { yLo = 0.0f; yHi = 0.0f; }
  if (xLo > xHi) { xLo = 0.0f; xHi = 0.0f; }
  float cx = clampf(*px, xLo, xHi); if (cx != *px) fl |= 1; *px = cx;
  float cy = clampf(*py, yLo, yHi); if (cy != *py) fl |= 2; *py = cy;
  float zLo = fmaxf(fmaxf(-L - offset1 - *py, -L - offset3 + *py), fmaxf(-L - offset2 - *px, -L - offset4 + *px));
  float zHi = fminf(fminf(L - offset1 - *py, L - offset3 + *py), fminf(L - offset2 - *px, L - offset4 + *px));
  if (zLo > zHi) { zLo = 0.0f; zHi = 0.0f; }
  float cz = clampf(*pz, zLo, zHi); if (cz != *pz) fl |= 4; *pz = cz;
  return fl;
#else
  (void)px; (void)py; (void)pz; return 0;
#endif
}
void mixAndDriveServos(float pid_x, float pid_y, float pid_z) {
  float px = pid_x, py = pid_y, pz = pid_z;
  satFlags = desaturateCmd(&px, &py, &pz);
  satLimX = (satFlags & 1) != 0; satLimY = (satFlags & 2) != 0; satLimZ = (satFlags & 4) != 0;
  satX = px; satY = py; satZ = pz;
  float m[4]; finMix(px, py, pz, m);
  float fin1 = constrain(m[0] + offset1, -MAX_FIN_DEG, MAX_FIN_DEG);
  float fin2 = constrain(m[1] + offset2, -MAX_FIN_DEG, MAX_FIN_DEG);
  float fin3 = constrain(m[2] + offset3, -MAX_FIN_DEG, MAX_FIN_DEG);
  float fin4 = constrain(m[3] + offset4, -MAX_FIN_DEG, MAX_FIN_DEG);
  finTgt[0] = fin1; finTgt[1] = fin2; finTgt[2] = fin3; finTgt[3] = fin4;
  static float finSm[4]; static bool finInit = false; static uint32_t lastUs = 0;
  uint32_t nowUs = micros(); float dts = lastUs ? (float)(uint32_t)(nowUs - lastUs) * 1.0e-6f : 1.0f / EST_HZ; lastUs = nowUs;
  dts = clampf(dts, DT_MIN_S, DT_MAX_S);
  if (!finInit) { finSm[0] = g_servo1_deg; finSm[1] = g_servo2_deg; finSm[2] = g_servo3_deg; finSm[3] = g_servo4_deg; finInit = true; }
  float tgt[4] = {fin1, fin2, fin3, fin4};
  float alpha = (SERVO_SMOOTH_HZ > 0.0f) ? dts / (dts + 1.0f / (2.0f * K_PI * SERVO_SMOOTH_HZ)) : 1.0f;
  float maxStep = (SERVO_MAX_SLEW_DPS > 0.0f) ? SERVO_MAX_SLEW_DPS * dts : 1.0e9f;
  for (uint8_t i = 0; i < 4; i++) { float want = finSm[i] + alpha * (tgt[i] - finSm[i]); finSm[i] += clampf(want - finSm[i], -maxStep, maxStep); }
  fin1 = finSm[0]; fin2 = finSm[1]; fin3 = finSm[2]; fin4 = finSm[3];
  writeServoDeg(servo1, fin1, dir1); writeServoDeg(servo2, fin2, dir2); writeServoDeg(servo3, fin3, dir3); writeServoDeg(servo4, fin4, dir4);
  g_servo1_deg = fin1; g_servo2_deg = fin2; g_servo3_deg = fin3; g_servo4_deg = fin4;
}
void centerAllServos() {
  writeServoDeg(servo1, 0, dir1); writeServoDeg(servo2, 0, dir2); writeServoDeg(servo3, 0, dir3); writeServoDeg(servo4, 0, dir4);
  delayWithLEDs(300);
  writeServoDeg(servo1, offset1, dir1); writeServoDeg(servo2, offset2, dir2); writeServoDeg(servo3, offset3, dir3); writeServoDeg(servo4, offset4, dir4);
  g_servo1_deg = offset1; g_servo2_deg = offset2; g_servo3_deg = offset3; g_servo4_deg = offset4; delayWithLEDs(500);
}
void centerAndHold(unsigned long hold_ms) {
  writeServoDeg(servo1, offset1, dir1); writeServoDeg(servo2, offset2, dir2); writeServoDeg(servo3, offset3, dir3); writeServoDeg(servo4, offset4, dir4);
  delayWithLEDs(hold_ms);
}
void runFinTestSequence() {
  Serial.println(F("Starting fin test sequence (motor stays at 1000 us)...")); logEvent("INFO", "Fin test started");
  Serial.println(F("  [1] Center all fins (3s)")); centerAndHold(3000);
  const float cmd = TEST_SWEEP_DEG / FIN_TEST_DIV;
  Serial.println(F("  [2] X-axis command (pid_x) +/- sweep"));
  driveFinsDirect(cmd, 0, 0); delayWithLEDs(TEST_HOLD_MS); driveFinsDirect(-cmd, 0, 0); delayWithLEDs(TEST_HOLD_MS); centerAndHold(TEST_HOLD_MS);
  Serial.println(F("  [3] Y-axis command (pid_y) +/- sweep"));
  driveFinsDirect(0, cmd, 0); delayWithLEDs(TEST_HOLD_MS); driveFinsDirect(0, -cmd, 0); delayWithLEDs(TEST_HOLD_MS); centerAndHold(TEST_HOLD_MS);
  Serial.println(F("  [4] All 4 fins together"));
  writeServoDeg(servo1, TEST_SWEEP_DEG + offset1, dir1); writeServoDeg(servo2, TEST_SWEEP_DEG + offset2, dir2);
  writeServoDeg(servo3, TEST_SWEEP_DEG + offset3, dir3); writeServoDeg(servo4, TEST_SWEEP_DEG + offset4, dir4); delayWithLEDs(TEST_HOLD_MS);
  writeServoDeg(servo1, -TEST_SWEEP_DEG + offset1, dir1); writeServoDeg(servo2, -TEST_SWEEP_DEG + offset2, dir2);
  writeServoDeg(servo3, -TEST_SWEEP_DEG + offset3, dir3); writeServoDeg(servo4, -TEST_SWEEP_DEG + offset4, dir4); delayWithLEDs(TEST_HOLD_MS);
  centerAndHold(TEST_HOLD_MS); Serial.println(F("Fin test complete.")); logEvent("INFO", "Fin test complete");
}

// ============================== 15. iBUS + MOTOR + FAULT RECOVERY ===
void updateIBUS() {
  if (!serial1Started) return;
  uint8_t budget = 64;
  while (budget-- && Serial1.available()) {
    uint8_t v = (uint8_t)Serial1.read();
    if (ibusIdx == 0) { if (v != 0x20) continue; }
    else if (ibusIdx == 1) {
      if (v != 0x40) { if (v == 0x20) { ibusBuf[0] = 0x20; ibusIdx = 1; } else ibusIdx = 0; continue; }
    }
    ibusBuf[ibusIdx++] = v;
    if (ibusIdx == 32) {
      ibusIdx = 0; uint16_t cs = 0xFFFF;
      for (uint8_t i = 0; i < 30; i++) cs -= ibusBuf[i];
      uint16_t rx = (uint16_t)ibusBuf[30] | ((uint16_t)ibusBuf[31] << 8);
      if (cs == rx) {
        for (uint8_t i = 0; i < 14; i++) ibusCh[i] = (uint16_t)ibusBuf[2 + i * 2] | ((uint16_t)ibusBuf[3 + i * 2] << 8);
        ibusLastFrameMs = millis(); ibusEverValid = true;
      }
    }
  }
}
bool ibusLinkOk(uint32_t nowMs) { return ibusEverValid && (uint32_t)(nowMs - ibusLastFrameMs) < IBUS_TIMEOUT_MS; }

void updateMotor(uint32_t nowMs, float tiltAbsDeg, bool estimatorOk) {
  bool link = ibusLinkOk(nowMs); static bool lastLink = false;
  if (link != lastLink) { lastLink = link; if (link) logEvent("INFO", "iBUS link OK"); else if (ibusEverValid) logEvent("WARN", "iBUS link LOST"); }
  uint16_t thr = ibusCh[IBUS_CH_THROTTLE], arm = ibusCh[IBUS_CH_ARM];
  bool chOk = link && thr >= 800 && thr <= 2200 && arm >= 800 && arm <= 2200;
  if (!chOk) chOkSinceMs = 0; else if (chOkSinceMs == 0) chOkSinceMs = nowMs;
  bool switchLow = chOk && arm <= ARM_SWITCH_LOW_US;
  bool switchHigh = chOk && arm >= ARM_SWITCH_HIGH_US;
  bool switchOn = chOk && arm >= ARM_SWITCH_ON_ABOVE;
  if (switchLow) { armSwitchSeenLow = true; tiltTripped = false; }

  // Fault recovery trigger: one attempt per Ch5 cycle (switch seen ON, then LOW), disarmed, throttle low.
  static bool seenOn = false;
  if (switchOn) seenOn = true;
  if (switchLow) {
    if (FAULT_RECOVERY_ENABLED && !motorArmed && faultMask != 0 && seenOn && thr <= ARM_THROTTLE_MAX_US) recoverRequest = true;
    seenOn = false;
  }

  if (motorArmed) {
    bool disarm = false; const char* why = "";
    if (!chOk) { disarm = true; why = "iBUS lost/invalid"; }
    else if (!switchOn) { disarm = true; why = "kill switch (Ch5 low)"; }
    else if (systemState != STATE_FLIGHT || !estimatorOk) { disarm = true; why = "estimator invalid / fault"; }
    if (tiltAbsDeg > TILT_LIMIT_DEG) {
      if (!tiltOver) { tiltOver = true; tiltOverSinceMs = nowMs; logEventf("WARN", "Tilt over limit (%.1f deg), trip in %d ms", tiltAbsDeg, TILT_TRIP_MS); }
      else if ((uint32_t)(nowMs - tiltOverSinceMs) >= TILT_TRIP_MS) {
        disarm = true; tiltTripped = true; why = "tilt limit exceeded";
        Serial.println(F("TILT LIMIT EXCEEDED: motor off, fins neutral. Cycle Ch5 low to reset."));
        logEventf("ERROR", "TILT LIMIT EXCEEDED (%.1f deg): motor off, fins neutral", tiltAbsDeg);
      }
    } else tiltOver = false;
    if (disarm) {
      motorArmed = false; if (!switchLow) armSwitchSeenLow = false;
      Serial.println(F("DISARMED.")); logEventf("ARM", "DISARMED (%s)", why);
    }
  } else {
    tiltOver = false;
    if (switchHigh && armSwitchSeenLow && chOk && chOkSinceMs != 0 &&
        (uint32_t)(nowMs - chOkSinceMs) >= ARM_LINK_STABLE_MS && thr <= ARM_THROTTLE_MAX_US &&
        estimatorOk && systemState == STATE_FLIGHT && faultMask == 0 && !tiltTripped && tiltAbsDeg < 0.8f * TILT_LIMIT_DEG) {
      motorArmed = true; armSwitchSeenLow = false; resetControlState();
      Serial.println(F("ARMED.")); logEvent("ARM", "ARMED");
    }
  }
  if (motorArmed) {
    float f = clampf((float)((int)thr - THR_IN_LOW_US) / (float)(THR_IN_HIGH_US - THR_IN_LOW_US), 0.0f, 1.0f);
#ifdef THR_SLEW_LIMIT
    const float dtS = 1.0f / (float)EST_HZ;
    thrSlewF += clampf(f - thrSlewF, -THR_SLEW_DN_PER_S * dtS, THR_SLEW_UP_PER_S * dtS);
    f = thrSlewF;
#endif
    motorCmdUs = (uint16_t)(ESC_MIN_US + (int)(f * (float)(THROTTLE_MAX_US - ESC_MIN_US) + 0.5f));
  } else {
    thrSlewF = 0.0f;
    motorCmdUs = ESC_MIN_US;
  }
}

// Clears latched runtime faults without a power cycle. Called from loop() only (blocks ~0.5-1 s), only while disarmed.
void attemptFaultRecovery() {
  if (motorArmed || faultMask == 0) return;
  uint8_t old = faultMask;
  Serial.print(F("FAULT RECOVERY: attempting (mask 0x")); Serial.print(old, HEX); Serial.println(F("). Keep the craft still and upright."));
  logEventf("INFO", "Fault recovery attempt (mask 0x%02X)", (unsigned)old);
  esc.writeMicroseconds(ESC_MIN_US);
  if (old & FAULT_BMI_DATA) {
    if (!initBMI323()) {
      logEvent("ERROR", "Recovery failed: BMI323 re-init failed (cycle Ch5 to retry)");
      Serial.println(F("FAULT RECOVERY FAILED: BMI323 re-init. Cycle Ch5 to retry."));
      return;
    }
  }
  if (!resyncEstimatorsFromAccel()) {
    logEvent("ERROR", "Recovery failed: estimator resync (still/upright? cycle Ch5 to retry)");
    Serial.println(F("FAULT RECOVERY FAILED: estimator resync. Cycle Ch5 to retry."));
    return;
  }
  faultMask = 0; bmi_runtime_error = false; bmi_init_error = false;
  imuInvalidCount = 0; consecBadMax = 0;
  diffWarnActive = false; disagreeActive = false; spinOver = false; tiltOver = false; tiltTripped = false;
  controlValid = false;   // re-evaluated on the next tick
  systemState = STATE_FLIGHT;
  lastGoodReadUs = micros(); nextTickUs = micros() + EST_PERIOD_US;
  logEvent("INFO", "Fault cleared: ready to arm"); Serial.println(F("FAULT CLEARED. You can arm."));
}

// ============================== 16. SD LOGGING ======================
void sdFail(const char* why) {
  sd_ok = false; sd_error = true; sdFile.close(); Serial.print(F("SD FAILED: ")); Serial.println(why);
}
void initSD() {
#ifndef USE_SD
  Serial.println(F("SD logging disabled (USE_SD not defined)."));
  return;
#endif
  Serial.println(F("=== SD card init ==="));
  bool ok = SD.sdfs.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(4)));
  if (!ok) { Serial.println(F("SD.sdfs.begin() FAILED - continuing without SD logging.")); sd_ok = false; sd_error = true; return; }
  uint32_t sizeMB = (uint32_t)(SD.sdfs.card()->sectorCount() / 2048UL);
  Serial.print(F("Card size: ")); Serial.print(sizeMB); Serial.println(F(" MB"));
  for (int i = 1; i <= 9999; i++) { snprintf(sdFileName, sizeof(sdFileName), "LOG%04d.CSV", i); if (!SD.exists(sdFileName)) break; }
  sdFile = SD.open(sdFileName, FILE_WRITE);
  if (!sdFile) { Serial.print(F("Failed to open ")); Serial.println(sdFileName); sd_ok = false; sd_error = true; return; }
  size_t n1 = sdFile.print(LOG_HDR_LEGACY), n2 = sdFile.print(','), n3 = sdFile.println(LOG_HDR_EXT); sdFile.flush();
  if (n1 == 0 || n2 == 0 || n3 == 0) { sdFail("header write failed"); return; }
  ringHead = ringTail = 0; lastSyncMs = millis(); sd_ok = true; sd_error = false;
  Serial.print(F("Logging to ")); Serial.println(sdFileName);
  for (uint8_t i = 0; i < evPendingN; i++) sdEnqueue(evPending[i], (uint16_t)strlen(evPending[i]));
  evPendingN = 0; logEvent("INFO", "SD logging started");
}
void sdEnqueue(const char* data, uint16_t n) {
  uint32_t used = ringHead - ringTail;
  if (LOG_RING_SIZE - used < n) { logDrops++; return; }
  for (uint16_t i = 0; i < n; i++) logRing[(ringHead + i) & (LOG_RING_SIZE - 1)] = (uint8_t)data[i];
  ringHead += n;
}
void logEvent(const char* type, const char* msg) {
  char line[EVT_LINE_MAX];
  int n = snprintf(line, sizeof(line), "#EVT,%lu,%s,%s\n", (unsigned long)millis(), type, msg);
  if (n <= 0) return;
  if (n >= (int)sizeof(line)) { n = sizeof(line) - 1; line[n - 1] = '\n'; line[n] = 0; }
  if (sd_ok) { sdEnqueue(line, (uint16_t)n); evUrgent = true; }
  else if (!sd_error && evPendingN < EVT_PENDING_MAX) { memcpy(evPending[evPendingN], line, (size_t)n + 1); evPendingN++; }
}
void logEventf(const char* type, const char* fmt, ...) {
  char m[100]; va_list ap; va_start(ap, fmt); vsnprintf(m, sizeof(m), fmt, ap); va_end(ap); logEvent(type, m);
}
void sdDrainNow() {
  if (!sd_ok) return;
  while (ringTail != ringHead) {
    uint32_t c = ringHead - ringTail; if (c > SD_CHUNK) c = SD_CHUNK;
    for (uint32_t i = 0; i < c; i++) sdChunk[i] = logRing[(ringTail + i) & (LOG_RING_SIZE - 1)];
    if (sdFile.write(sdChunk, c) != c) break;
    ringTail += c;
  }
  sdFile.flush();
}
void updateSDLogging() {
  if (!sd_ok) return;
  if (!SD_LOG_WHILE_ARMED && motorArmed) return;
  int32_t slack = (int32_t)(nextTickUs - micros()); if (slack < (int32_t)SD_MIN_SLACK_US) return;
  static bool idleValid = false; static uint32_t idleSinceMs = 0; uint32_t nowMs = millis();
  if (motorCmdUs <= ESC_MIN_US + SD_IDLE_THR_MARGIN_US) { if (!idleValid) { idleValid = true; idleSinceMs = nowMs; } }
  else idleValid = false;
  // Never touch the card while the ESC is above idle; the ring buffers rows and drains once throttle returns to idle.
  if (motorArmed && motorCmdUs > ESC_MIN_US + SD_IDLE_THR_MARGIN_US) return;
  bool flying = motorArmed && !(idleValid && (uint32_t)(nowMs - idleSinceMs) >= SD_IDLE_HOLD_MS);
  uint32_t period = flying ? (uint32_t)SD_FLYING_SYNC_PERIOD_MS : (uint32_t)SD_SYNC_PERIOD_MS;
  bool syncDue = (period > 0) && ((uint32_t)(nowMs - lastSyncMs) >= period);
  bool eventSync = sdNeedFlush && ringTail == ringHead && !flying;
  if (eventSync || syncDue) {
    sdNeedFlush = false; lastSyncMs = nowMs; uint32_t t0 = micros(); sdFile.flush();
    uint32_t d = micros() - t0; if (d > sdMaxWriteUs) sdMaxWriteUs = d; return;
  }
  uint32_t count = ringHead - ringTail, want = 0;
  if (count >= SD_CHUNK) want = SD_CHUNK;
  else if (evUrgent && count > 0) want = count;
  else if (evUrgent) evUrgent = false;
  if (want) {
    for (uint16_t i = 0; i < want; i++) sdChunk[i] = logRing[(ringTail + i) & (LOG_RING_SIZE - 1)];
    uint32_t t0 = micros(); size_t w = sdFile.write(sdChunk, want); uint32_t d = micros() - t0;
    if (d > sdMaxWriteUs) sdMaxWriteUs = d;
    if (w != want) { sdFail("write failed mid-run (logging stopped, control unaffected)"); return; }
    ringTail += want;
    if (evUrgent && ringTail == ringHead) { evUrgent = false; sdNeedFlush = true; }
  }
}

void buildRow(RowWriter* w, bool extended) {
  w->reset(); w->u32(snap.tUs / 1000UL);
  w->f(snap.angX, 3); w->f(snap.angY, 3); w->f(snap.gyrF[2], 3);
  w->f(snap.gyrF[0], 3); w->f(snap.gyrF[1], 3); w->f(snap.gyrF[2], 3);
  w->f(snap.accF[0], 4); w->f(snap.accF[1], 4); w->f(snap.accF[2], 4);
  w->f(angle_x_setpoint, 2); w->f(angle_y_setpoint, 2); w->f(snap.errX, 3); w->f(snap.errY, 3);
  w->f(snap.pidX, 3); w->f(snap.pidY, 3); w->f(snap.pidZ, 3);
  w->f(g_servo1_deg, 2); w->f(g_servo2_deg, 2); w->f(g_servo3_deg, 2); w->f(g_servo4_deg, 2);
  if (!extended) { w->endRow(); return; }
  for (uint8_t a = 0; a < 3; a++) { w->f(snap.pidP[a], 4); w->f(snap.pidI[a], 4); w->f(snap.pidD[a], 4); }
  w->f(snap.imuRoll, 3); w->f(snap.imuPitch, 3); w->u32(snap.tUs); w->u32(snap.dtUs);
  for (uint8_t i = 0; i < 6; i++) w->i32(snap.raw[i]);
  w->f(snap.acc[0], 4); w->f(snap.acc[1], 4); w->f(snap.acc[2], 4);
  w->f(snap.gyr[0], 3); w->f(snap.gyr[1], 3); w->f(snap.gyr[2], 3);
  w->f(snap.fusRoll, 3); w->f(snap.fusPitch, 3); w->f(snap.fusAccErr, 2); w->u32(snap.fusFlags);
  w->f(snap.biasX, 4); w->f(snap.biasY, 4); w->f(snap.sigX, 3); w->f(snap.sigY, 3);
  w->f(snap.sigBX, 4); w->f(snap.sigBY, 4); w->f(snap.innov, 4); w->u32(snap.accUsed); w->f(snap.accNorm, 4); w->f(estDiffDeg, 3);
  w->u32((uint32_t)systemState); w->u32(faultMask); w->u32(motorArmed ? 1 : 0); w->u32(ibusLinkOk(millis()) ? 1 : 0);
  w->u32(ibusEverValid ? (uint32_t)(millis() - ibusLastFrameMs) : 99999UL);
  w->u32(ibusCh[IBUS_CH_THROTTLE]); w->u32(ibusCh[IBUS_CH_ARM]); w->u32(ibusCh[IBUS_CH_YAW]); w->u32(ibusCh[IBUS_CH_PITCH]); w->u32(motorCmdUs);
  w->u32(snap.tickUs); w->u32(lateMaxUs); w->u32(overrunCount); w->u32(logDrops); w->u32(sdMaxWriteUs);
  w->f(snap.rateXr, 3); w->f(snap.rateYr, 3); w->f(snap.rateXd, 3); w->f(snap.rateYd, 3);
  w->f(satX, 3); w->f(satY, 3); w->f(satZ, 3); w->u32(satFlags);
  w->f(finTgt[0], 2); w->f(finTgt[1], 2); w->f(finTgt[2], 2); w->f(finTgt[3], 2);
  w->f(snap.nis, 3); w->u32(snap.accState); w->f(snap.rScale, 2); w->f(snap.estDiffRoll, 3); w->f(snap.estDiffPitch, 3);
  w->u32(snap.dtRawUs); w->u32(curLateUs); w->u32(staleSamples); w->u32(dupSamples); w->u32(badTicks);
  w->u32(consecBadMax); w->u32(bmiDrdyUsable ? 1 : 0); w->u32(dtGapCount);
  w->f(snap.gyroFFX, 4); w->f(snap.gyroFFY, 4);
  w->u32(ibusCh[IBUS_CH_RUDDER]); w->f(snap.yawCmd, 2);
  w->f(snap.yawFFss, 3); w->f(snap.yawFFdyn, 3);   // v2.4
  w->endRow();
}

// ============================== 17. MAIN TICK =======================
// High-priority path: IMU -> filters -> estimators -> checks -> PID/feedforward -> servos/ESC.
void estimatorControlTick() {
  uint32_t tStart = micros();
  int32_t late = (int32_t)(tStart - nextTickUs); curLateUs = late > 0 ? (uint32_t)late : 0;
  if (late > 0 && (uint32_t)late > lateMaxUs) lateMaxUs = (uint32_t)late;
  tickCounter++;
  const bool logTick    = (tickCounter % LOG_DECIM) == 0;
  const bool serialTick = (tickCounter % SERIAL_DECIM) == 0;

  ImuSample s; bool rawOk = readIMU(&s); bool ok = rawOk && s.fresh;
  imuWaitUs = s.waitUs; lastSampleFresh = ok;
  if (rawOk && !s.fresh) staleSamples++;
  if (rawOk) {
    if (s.frozenRun == IMU_FROZEN_LIMIT) logEvent("ERROR", "IMU data frozen (identical raw samples)");
    if (s.frozenRun >= IMU_FROZEN_LIMIT) ok = false;
  }

  if (ok) {
    imuInvalidCount = 0; bmi_runtime_error = (faultMask & FAULT_BMI_DATA) != 0;
    float dtRaw = (float)(uint32_t)(s.tUs - lastGoodReadUs) * 1.0e-6f; lastGoodReadUs = s.tUs;
    float dt, dtEst; bool dtGap; selectDt(dtRaw, &dt, &dtEst, &dtGap);
    if (dtGap) dtGapCount++;
    float accF[3], gyrF[3];
    for (uint8_t i = 0; i < 3; i++) { accF[i] = biquadStep(&fltAcc[i], s.acc[i]); gyrF[i] = biquadStep(&fltGyr[i], s.gyr[i]); }

    // EKF primary estimator; gyro prediction uses unfiltered calibrated samples.
    ekfPredict(s.gyr[0] * K_DEG2RAD, s.gyr[1] * K_DEG2RAD, s.gyr[2] * K_DEG2RAD, dtEst);
    ekfUpdate(accF);
    fusUpdate(&fusion, v3make(s.gyr[0] - ekf.x[2] * K_RAD2DEG, s.gyr[1] - ekf.x[3] * K_RAD2DEG, s.gyr[2]),
              v3make(accF[0], accF[1], accF[2]), dtEst);
    float fusRoll, fusPitch; fusEuler(&fusion, &fusRoll, &fusPitch);
    bool ekfOk = ekfHealthy(); if (!ekfOk) latchFault(FAULT_EKF_INVALID, "EKF state/covariance invalid");
    float ekfRollDeg = ekf.x[0] * K_RAD2DEG, ekfPitchDeg = ekf.x[1] * K_RAD2DEG;
    estDiffRoll = fabsf(wrapDeg180(ekfRollDeg - fusRoll)); estDiffPitch = fabsf(wrapDeg180(ekfPitchDeg - fusPitch));
    estDiffDeg = fmaxf(estDiffRoll, estDiffPitch); if (!isFiniteF(estDiffDeg)) estDiffDeg = 999.0f;
    bool fusTrusted = !fusion.startup && !fusion.overrangeRecovery;
    if (fusTrusted && estDiffDeg > CROSSCHECK_WARN_DEG) {
      if (!diffWarnActive) { diffWarnActive = true; logEventf("WARN", "EKF vs Fusion differ %.1f deg (roll %.1f pitch %.1f)", (double)estDiffDeg, (double)estDiffRoll, (double)estDiffPitch); }
    } else if (estDiffDeg < 0.75f * CROSSCHECK_WARN_DEG) diffWarnActive = false;
    if (fusTrusted && estDiffDeg > CROSSCHECK_MAX_DEG) {
      if (!disagreeActive) { disagreeActive = true; disagreeSinceMs = millis(); logEventf("WARN", "EKF vs Fusion disagree %.1f deg", (double)estDiffDeg); }
      else if (motorArmed && (uint32_t)(millis() - disagreeSinceMs) > CROSSCHECK_MS) latchFault(FAULT_DISAGREE, "EKF and Fusion attitude disagree");
    } else disagreeActive = false;

    // ---- Control ----
    angleXdeg = ekfRollDeg - ANGLE_TRIM_X_DEG; angleYdeg = ekfPitchDeg - ANGLE_TRIM_Y_DEG;
    float rateXr = s.gyr[0] - ekf.x[2] * K_RAD2DEG;
    float rateYr = s.gyr[1] - ekf.x[3] * K_RAD2DEG;
    float rateZ = gyrF[2];
    if (motorArmed && fabsf(rateZ) > SPIN_LIMIT_DPS) {
      if (!spinOver) { spinOver = true; spinOverSinceMs = millis(); logEventf("WARN", "Yaw spin over limit (%.0f dps), trip in %d ms", rateZ, SPIN_TRIP_MS); }
      else if ((uint32_t)(millis() - spinOverSinceMs) >= SPIN_TRIP_MS) {
        motorArmed = false; tiltTripped = true; armSwitchSeenLow = false; spinOver = false;
        Serial.println(F("SPIN LIMIT EXCEEDED: motor off, fins neutral. Cycle Ch5 low to reset."));
        logEventf("ERROR", "SPIN LIMIT EXCEEDED (%.0f dps): motor off, fins neutral", rateZ);
      }
    } else spinOver = false;
    {
      float ax_, ay_, rx_, ry_; rotToFlightFrame(angleXdeg, angleYdeg, &ax_, &ay_); rotToFlightFrame(rateXr, rateYr, &rx_, &ry_);
      angleXdeg = ax_; angleYdeg = ay_; rateXr = rx_; rateYr = ry_;
    }
    float rateXd = biquadStep(&fltDX, rateXr), rateYd = biquadStep(&fltDY, rateYr);
    updateRcSetpoints(dt);
    errXdeg = angle_x_setpoint - angleXdeg; errYdeg = angle_y_setpoint - angleYdeg;
    float thrFrac = clampf((float)((int)motorCmdUs - ESC_MIN_US) / (float)(THROTTLE_MAX_US - ESC_MIN_US), 0.0f, 1.0f);
    float yawCmdDps = 0.0f;
#ifdef RC_YAW_RATE_CONTROL
    if (motorArmed && ibusLinkOk(millis())) {
      uint16_t c4 = ibusCh[IBUS_CH_RUDDER];
      if (c4 >= 800 && c4 <= 2200) yawCmdDps = RC_YAW_SIGN * rcStick(c4) * RC_MAX_YAW_DPS;
    }
#endif
    float errZ = yawCmdDps - rateZ;
    bool integ = motorArmed && thrFrac > YAW_INTEG_MIN_THR && !dtGap;
    bool intX = integ && !(satLimX && TVC_SIGN_X * errXdeg * pidOutX > 0.0f);
    bool intY = integ && !(satLimY && TVC_SIGN_Y * errYdeg * pidOutY > 0.0f);
    bool intZ = integ && !(satLimZ && TVC_SIGN_Z * errZ * pidOutZ > 0.0f);
    intX = intX && fabsf(errXdeg) < I_ERR_GATE_DEG && fabsf(rateXd) < I_RATE_GATE_DPS;
    intY = intY && fabsf(errYdeg) < I_ERR_GATE_DEG && fabsf(rateYd) < I_RATE_GATE_DPS;

    // ---- Ordinary X/Y PID feedback ----
    float rawPidX = pidStep(&pidX, errXdeg, rateXd, Kp_x, Ki_x, Kd_x, dt, intX);
    float rawPidY = pidStep(&pidY, errYdeg, rateYd, Kp_y, Ki_y, Kd_y, dt, intY);

    // ---- Gyroscopic precession feedforward ----
    float gyroRawX = 0.0f, gyroRawY = 0.0f;
    if (motorArmed && thrFrac >= GYRO_PRECESSION_MIN_THR && GYRO_PRECESSION_FF_GAIN != 0.0f) {
      const float ffScale = GYRO_PRECESSION_SIGN * GYRO_PRECESSION_FF_GAIN;
      gyroRawX = clampf(ffScale * rateYd, -GYRO_PRECESSION_MAX_DEG, GYRO_PRECESSION_MAX_DEG);
      gyroRawY = clampf(-ffScale * rateXd, -GYRO_PRECESSION_MAX_DEG, GYRO_PRECESSION_MAX_DEG);
    }
    float gyroFFX = TVC_SIGN_X * gyroRawX;
    float gyroFFY = TVC_SIGN_Y * gyroRawY;
    float ox = TVC_SIGN_X * rawPidX + gyroFFX;
    float oy = TVC_SIGN_Y * rawPidY + gyroFFY;

    // ---- Yaw: rate PI + throttle-dependent feedforward (v2.4) ----
    if (thrFrac <= YAW_INTEG_MIN_THR) pidZ.integral *= YAW_I_IDLE_DECAY;   // do not carry a wound-up yaw integral through idle
    // Motor-lag throttle and its analytic rate (d/dt of a first-order lag = (u - x)/tau; noise-free).
    if (!motorArmed) yawThrM = 0.0f;
    const float dThrM = (thrFrac - yawThrM) / YAW_MOTOR_TAU_S;             // thr fraction per second
    yawThrM += (dt / (YAW_MOTOR_TAU_S + dt)) * (thrFrac - yawThrM);
    float yawFFss = 0.0f, yawFFdyn = 0.0f;
    if (motorArmed) {
      yawFFss = YAW_FF_SS_SCALE * yawFFTable(yawThrM);
      if (yawThrM > YAW_DYN_MIN_THR && YAW_DYN_SCALE != 0.0f) {
        const float K = (dThrM >= 0.0f) ? YAW_DYN_K_UP : YAW_DYN_K_DN;
        yawFFdyn = clampf(YAW_DYN_SCALE * K * (yawThrM - 0.25f) * dThrM / (YAW_FIN_B * yawThrM), -YAW_DYN_MAX_DEG, YAW_DYN_MAX_DEG);
      }
    }
    float oz = TVC_SIGN_Z * (pidStepZ(&pidZ, errZ, 0.0f, Kp_z, Ki_z, 0.0f, dt, intZ) + yawFFss + yawFFdyn);
    if (!isFiniteF(ox) || !isFiniteF(oy) || !isFiniteF(oz)) {
      latchFault(FAULT_NONFINITE, "non-finite control output"); ox = oy = oz = 0;
    }
    pidOutX = ox; pidOutY = oy; pidOutZ = oz;
    controlValid = ekfOk && (sqrtf(angleXdeg * angleXdeg + angleYdeg * angleYdeg) < CONTROL_ANGLE_MAX_DEG);
    static bool lastCv = false;
    if (controlValid != lastCv) { lastCv = controlValid; if (controlValid) logEvent("INFO", "Control valid"); else logEvent("WARN", "Control INVALID (EKF unhealthy or angle > limit): fins neutral"); }

    // ---- Snapshot for logging (only on ticks whose row is actually emitted) ----
    if (logTick || serialTick) {
      snap.pidP[0] = TVC_SIGN_X * pidX.p; snap.pidI[0] = TVC_SIGN_X * pidX.i; snap.pidD[0] = TVC_SIGN_X * pidX.d;
      snap.pidP[1] = TVC_SIGN_Y * pidY.p; snap.pidI[1] = TVC_SIGN_Y * pidY.i; snap.pidD[1] = TVC_SIGN_Y * pidY.d;
      snap.pidP[2] = TVC_SIGN_Z * pidZ.p; snap.pidI[2] = TVC_SIGN_Z * pidZ.i; snap.pidD[2] = TVC_SIGN_Z * pidZ.d;
      snap.tUs = tStart; snap.dtUs = (uint32_t)(dt * 1.0e6f); snap.dtRawUs = (uint32_t)(dtRaw * 1.0e6f);
      for (uint8_t i = 0; i < 6; i++) snap.raw[i] = s.raw[i];
      for (uint8_t i = 0; i < 3; i++) { snap.acc[i] = s.acc[i]; snap.gyr[i] = s.gyr[i]; snap.accF[i] = accF[i]; snap.gyrF[i] = gyrF[i]; }
      snap.imuRoll = ekfRollDeg; snap.imuPitch = ekfPitchDeg; snap.angX = angleXdeg; snap.angY = angleYdeg;
      snap.errX = errXdeg; snap.errY = errYdeg; snap.rateXr = rateXr; snap.rateYr = rateYr; snap.rateXd = rateXd; snap.rateYd = rateYd;
      snap.nis = ekf.nis; snap.rScale = ekf.rScale; snap.accState = ekf.accState;
      snap.estDiffRoll = estDiffRoll; snap.estDiffPitch = estDiffPitch;
      snap.pidX = pidOutX; snap.pidY = pidOutY; snap.pidZ = pidOutZ;
      snap.gyroFFX = gyroFFX; snap.gyroFFY = gyroFFY; snap.yawCmd = yawCmdDps;
      snap.yawFFss = yawFFss; snap.yawFFdyn = yawFFdyn;
      snap.fusRoll = fusRoll; snap.fusPitch = fusPitch;
      snap.fusAccErr = K_RAD2DEG * asinf(clampf(2.0f * sqrtf(vNormSq(fusion.halfAccResidual)), 0.0f, 1.0f));
      snap.fusFlags = (fusion.startup ? 1u : 0u) | (fusion.overrangeRecovery ? 2u : 0u) |
                      ((fusion.accRecoveryTrigger > fusion.accRecoveryThreshold) ? 4u : 0u) | (fusion.accIgnored ? 8u : 0u);
      snap.biasX = ekf.x[2] * K_RAD2DEG; snap.biasY = ekf.x[3] * K_RAD2DEG;
      snap.sigX = sqrtf(fmaxf(ekf.P[0][0], 0.0f)) * K_RAD2DEG; snap.sigY = sqrtf(fmaxf(ekf.P[1][1], 0.0f)) * K_RAD2DEG;
      snap.sigBX = sqrtf(fmaxf(ekf.P[2][2], 0.0f)) * K_RAD2DEG; snap.sigBY = sqrtf(fmaxf(ekf.P[3][3], 0.0f)) * K_RAD2DEG;
      snap.innov = ekf.innovNorm; snap.accUsed = ekf.accUsed ? 1u : 0u; snap.accNorm = ekf.accNorm;
    }
  } else {
    imuInvalidCount++; badTicks++; if (imuInvalidCount > consecBadMax) consecBadMax = imuInvalidCount;
    if (imuInvalidCount >= BMI_DEAD_READ_LIMIT) bmi_runtime_error = true;
    if (imuInvalidCount == BMI_DEAD_READ_LIMIT) logEvent("WARN", "BMI323: consecutive invalid/stale samples");
    if (imuInvalidCount >= IMU_INVALID_LIMIT) latchFault(FAULT_BMI_DATA, "BMI323 data invalid/dead");
  }

  float tiltAbs = sqrtf(angleXdeg * angleXdeg + angleYdeg * angleYdeg);
  updateMotor(millis(), tiltAbs, controlValid && faultMask == 0);
  bool sensorLost = imuInvalidCount > IMU_GRACE_TICKS;
  bool neutral = (systemState != STATE_FLIGHT) || tiltTripped || !controlValid || sensorLost;
  if (neutral) mixAndDriveServos(0.0f, 0.0f, 0.0f);
  else mixAndDriveServos(pidOutX, pidOutY, pidOutZ);
  esc.writeMicroseconds(motorCmdUs);
  snap.tickUs = micros() - tStart;
  if (ok) {
    if (logTick && sd_ok && (SD_LOG_WHILE_ARMED || !motorArmed)) {
      buildRow(&rw, true); sdEnqueue(rw.buf, rw.n); lateMaxUs = 0; sdMaxWriteUs = 0; consecBadMax = 0;
    }
    if (serialTick) {
      buildRow(&rw, false); if (Serial.availableForWrite() >= (int)rw.n) Serial.write((const uint8_t*)rw.buf, rw.n);
    }
  }
}

// ============================== 18. SETUP / LOOP ====================
void setup() {
  Serial.begin(115200);
#ifdef USE_LEDS
  pinMode(STATUS_LED_PIN, OUTPUT); pinMode(PARTS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW); digitalWrite(PARTS_LED_PIN, LOW);
#endif
  esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US); esc.writeMicroseconds(ESC_MIN_US); motorArmed = false;
  for (uint8_t i = 0; i < 14; i++) ibusCh[i] = 1500;
  systemState = STATE_BOOT_CALIBRATING; updateLEDs();
  Serial.println(F("=== TVC controller v2.4 ===")); logEvent("BOOT", "TVC controller v2.4 boot");
  logEventf("INFO", "Gains X:%.2f/%.2f/%.2f Y:%.2f/%.2f/%.2f Z:%.2f/%.2f", Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z);
  logEventf("INFO", "I-term out limit %.1f deg, gates err %.0f deg rate %.0f dps", (double)I_OUT_LIMIT_DEG, (double)I_ERR_GATE_DEG, (double)I_RATE_GATE_DPS);
  logEventf("INFO", "Yaw FF ss scale %.2f (table %.1f deg @%.2f), dyn scale %.2f max %.1f deg, tau %.3f s, I_z lim %.1f deg, rudder sign %.0f",
            (double)YAW_FF_SS_SCALE, (double)YAW_FF_TBL_DEG[YAW_FF_N - 1], (double)YAW_FF_TBL_THR[YAW_FF_N - 1],
            (double)YAW_DYN_SCALE, (double)YAW_DYN_MAX_DEG, (double)YAW_MOTOR_TAU_S, (double)I_OUT_LIMIT_Z_DEG, (double)RC_YAW_SIGN);
#ifdef THR_SLEW_LIMIT
  logEventf("INFO", "Throttle slew limit up %.2f/s dn %.2f/s", (double)THR_SLEW_UP_PER_S, (double)THR_SLEW_DN_PER_S);
#else
  logEvent("INFO", "Throttle slew limit OFF");
#endif
  logEventf("INFO", "Gyro FF gain %.3f (min thr %.2f), spin limit %.0f dps, tilt limit %.0f deg",
            (double)GYRO_PRECESSION_FF_GAIN, (double)GYRO_PRECESSION_MIN_THR, (double)SPIN_LIMIT_DPS, (double)TILT_LIMIT_DEG);
  logEventf("INFO", "Filters Hz: acc %.0f gyr %.0f dterm %.0f servo %.0f | sensor ODR %d bw ODR/%d",
            (double)ACC_LPF_HZ, (double)GYR_LPF_HZ, (double)D_TERM_LPF_HZ, (double)SERVO_SMOOTH_HZ, BMI_ODR_HZ, BMI_BW_QUARTER ? 4 : 2);
  logEventf("INFO", "EKF gates: |a| %.2f g, NIS soft %.1f hard %.1f | xcheck warn %.0f max %.0f deg",
            (double)EKF_ACC_GATE_G, (double)EKF_NIS_SOFT, (double)EKF_NIS_HARD, (double)CROSSCHECK_WARN_DEG, (double)CROSSCHECK_MAX_DEG);
  logEventf("INFO", "Fault recovery by Ch5 cycle: %s", FAULT_RECOVERY_ENABLED ? "ON" : "OFF");
#ifdef FLIGHT_FRAME_PLUS
  logEvent("INFO", "Flight frame: + (servo1 front, 2 right, 3 back, 4 left)");
#else
  logEvent("INFO", "Flight frame: X (IMU frame, servo1 = +X+Y corner)");
#endif
  if (!initBMI323()) failStartup("BMI323 init failed (chip ID / status / config verify / no data).");
  float accMean[3]; if (!calibrateBMI323(accMean)) failStartup("BMI323 calibration failed (rig moved / bad gyro / |acc| wrong).");
  if (!initEstimators(accMean)) failStartup("Initial attitude check failed (rig not upright?).");
  Serial1.begin(115200); LPUART6_CTRL &= ~LPUART_CTRL_TE; serial1Started = true;
  systemState = STATE_SERVO_STARTUP; updateLEDs();
  servo1.attach(SERVO1_PIN, SERVO_MIN_US, SERVO_MAX_US); servo2.attach(SERVO2_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo3.attach(SERVO3_PIN, SERVO_MIN_US, SERVO_MAX_US); servo4.attach(SERVO4_PIN, SERVO_MIN_US, SERVO_MAX_US);
  centerAllServos(); runFinTestSequence();
  initSD();
  if (!resyncEstimatorsFromAccel()) failStartup("Pre-flight attitude resync failed.");
  Serial.println(LOG_HDR_LEGACY);
  armSwitchSeenLow = false; lastGoodReadUs = micros(); nextTickUs = micros() + EST_PERIOD_US;
  systemState = STATE_FLIGHT; logEvent("INFO", "Entered FLIGHT state (disarmed)"); updateLEDs();
}

void loop() {
  updateLEDs(); updateIBUS(); uint32_t now = micros();
  if (recoverRequest) {                       // fault recovery runs here, never inside the control tick; disarmed only
    recoverRequest = false;
    if (!motorArmed && faultMask != 0) attemptFaultRecovery();
    return;
  }
  if ((int32_t)(now - nextTickUs) >= 0) {
    estimatorControlTick(); nextTickUs += EST_PERIOD_US;
    if (bmiDrdyUsable) {
      int32_t adj;
      if (!lastSampleFresh) adj = IMU_PLL_MAX_STEP_US;
      else if (imuWaitUs == 0) adj = -(int32_t)IMU_PLL_STEP_US;
      else adj = ((int32_t)imuWaitUs - (int32_t)IMU_PLL_TARGET_US) / 2;
      if (adj > (int32_t)IMU_PLL_MAX_STEP_US) adj = (int32_t)IMU_PLL_MAX_STEP_US;
      if (adj < -(int32_t)IMU_PLL_MAX_STEP_US) adj = -(int32_t)IMU_PLL_MAX_STEP_US;
      nextTickUs += (uint32_t)adj;
    }
    now = micros(); int32_t behind = (int32_t)(now - nextTickUs);
    if (behind >= (int32_t)EST_PERIOD_US) {
      overrunCount++; static uint32_t lastOv = 0;
      if (millis() - lastOv > 1000) { lastOv = millis(); logEventf("WARN", "Control loop overrun (count %lu, lateMax %lu us)", (unsigned long)overrunCount, (unsigned long)lateMaxUs); }
    }
    if (behind > (int32_t)LATE_REANCHOR_US) while ((int32_t)(micros() - nextTickUs) >= 0) nextTickUs += EST_PERIOD_US;
  } else updateSDLogging();
}
