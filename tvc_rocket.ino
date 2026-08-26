/*
  ====================================================================
  EDF THRUST-VECTORING "HOVER ROCKET" FLIGHT CONTROLLER  (simplified)
  Arduino/Teensy + BMI323 IMU (SPI) + 4x SG90 servos on a gimbaled fin ring
  ====================================================================

  ORIENTATION / AXIS MAPPING (fixed per bench testing)
  ------------------------------------------------------
  Rocket stands nose-up. BMI323 is mounted FLAT (parallel to the
  ground) with:
    +Z  points straight up along the rocket body
    +Y  points toward FIN1 (in the horizontal plane)
    +X  points toward FIN2 (in the horizontal plane, 90 deg from Y)

  FIN LAYOUT (looking down from above, nose towards you):

            FIN1 (+Y direction)
               |
  FIN4 ----- CENTER ----- FIN2 (+X direction)
               |
            FIN3 (-Y direction)

  CONTROL MAPPING (per bench validation):
    - Rotation about X axis  ("pitch")  -> corrected by FIN2 & FIN4
    - Rotation about Y axis  ("yaw")    -> corrected by FIN1 & FIN3
    - Rotation about Z axis  ("roll")   -> corrected by ALL 4 fins,
                                            deflecting together in the
                                            same rotational sense
                                            (spin/swirl correction)

  WIRING
  -------------------
  BMI323 IMU (SPI):
    CSB (Chip Select) -> A4
    SCX (SPI Clock)   -> 13 (SCK)
    SDX (SPI MOSI)    -> 11 (MOSI)
    SDO (SPI MISO)    -> 12 (MISO)
    VDD -> 3.3V, GND -> GND
  Servo 1 (FIN1, +Y):  pin 1
  Servo 2 (FIN2, +X):  pin 2
  Servo 3 (FIN3, -Y):  pin 3
  Servo 4 (FIN4, -X):  pin 4
    NOTE: pins 1/2 double as Serial TX/RX (UART0) on most AVR/Teensy
    boards. Using them as Servo outputs is fine as long as you're not
    also using hardware Serial0 for USB/debug on those same pins -
    if Serial prints stop working or look garbled once servos are
    attached, that's the likely cause; move Servo1/Servo2 to
    different pins (e.g. 5 and 8) if so.
  microSD module (SPI, shares the bus with the BMI323):
    GND -> GND, VCC -> 3.3V
    MISO -> 12, MOSI -> 11, SCK -> 13, CS -> 10

  Power servos from a separate 5-6V BEC, common ground with the board.

  THROTTLE / FIN AUTHORITY (NOT IMPLEMENTED)
  --------------------------
  NOT IMPLEMENTED YET - removed for now since the motor/EDF isn't
  wired up. The PID output currently drives the fins directly with no
  throttle scaling. Once the motor is in and you're ready to fly, this
  is where gain-scheduling by throttle should go back in: fin
  effectiveness depends on the dynamic pressure of the EDF's exhaust
  hitting the fin (roughly throttle^2), which is a center-of-PRESSURE
  effect on the fin itself, not a center-of-mass effect - the moment
  arm from the rocket's CG to the fin is fixed geometry and doesn't
  change with throttle, only the force term does. Ask again when the
  motor's ready and this can be reintroduced.

  DATA LOGGING
  -------------
  Both Serial (100 Hz CSV, for live tuning) and the microSD card
  (same columns, for a full offline record) get:
    time_ms,angle_x,angle_y,rate_z,gx,gy,gz,ax,ay,az,
    angle_x_sp,angle_y_sp,err_x,err_y,
    pid_x,pid_y,pid_z,
    servo1,servo2,servo3,servo4

  SD card writes to the next free LOGnnnn.CSV each boot so old runs
  aren't overwritten. If SD init fails (no card, bad wiring), it
  just logs to Serial only and prints a warning - it won't halt.

  SD CARD NOTES (updated)
  --------------------------
  SD init now goes through SD.sdfs.begin(SdSpiConfig(...)) directly
  (SdFat under the SD library wrapper) at a fixed 4 MHz SPI clock,
  instead of the plain SD.begin(SD_CS_PIN) call. On this hardware the
  default driver/speed intermittently failed to initialize the card
  while SD_SCK_MHZ(4) reliably works, so that's what's used here. If
  you want more write throughput later and your wiring/card supports
  it, this can be bumped up (e.g. SD_SCK_MHZ(24)) - just retest that
  init still succeeds reliably at the higher speed before trusting it
  in flight.

  initSD() now also prints boot-time diagnostics (whether SD.sdfs.begin()
  succeeded, card size, which LOGnnnn.CSV file it opened, and a real
  write+flush test) so a bad card, wrong CS pin, wrong format, or a
  write-protected/full card shows up clearly in the Serial log instead
  of silently just not logging. As before, a failed SD init never
  blocks flight - it just falls back to Serial-only logging.

  Use tvc_logger.m for the live serial view, which also shows a 3D
  orientation widget (see below), and tvc_analyze.m for offline
  PID-tuning plots on a saved CSV pulled off the card.

  ORIENTATION VISUALIZATION
  ---------------------------
  tvc_logger.m now draws a labeled 3D block (FIN1/FIN2/FIN3/FIN4/TOP/
  BOTTOM faces) that rotates live to match angle_x/angle_y, so you can
  visually sanity-check that tilting the rig toward a given fin
  actually shows up as rotation toward that fin on screen - a fast way
  to catch a mismounted IMU or a sign error before it's a fins-vs-
  angle-direction argument.

  TUNING WORKFLOW
  ----------------
  1. EDF OFF. Confirm the startup fin test sequence moves the fins

     you expect, in a direction that makes sense.
  2. Tilt the rocket by hand toward FIN1 - only angle_y should move.
     Tilt toward FIN2 - only angle_x should move. If both move
     together for a single-axis tilt, the IMU mount is misaligned
     with the fin cross - re-check the physical mounting against the
     diagram above.
  3. Start with only Kp_x / Kp_y nonzero, Ki=Kd=0, tune per the usual
     process (increase Kp until oscillation, back off ~30-40%, add Kd
     to damp, add a little Ki last).
  ====================================================================
*/

#include <Servo.h>
#include <SPI.h>
#include <SD.h>

// ------------------- USER-TUNABLE PID GAINS -------------------
// "pitch" = rotation about X axis, corrected by FIN2/FIN4
float Kp_x = 1.8,  Ki_x = 0.0,  Kd_x = 0.10;
// "yaw" = rotation about Y axis, corrected by FIN1/FIN3
float Kp_y = 1.8,  Ki_y = 0.0,  Kd_y = 0.10;
// "roll" = rotation about Z axis (spin), corrected by all 4 fins together
float Kp_z = 0.0,  Ki_z = 0.0,  Kd_z = 0.0;

// Setpoints (deg) - for hovering straight up, both are 0
float angle_x_setpoint = 0.0;
float angle_y_setpoint = 0.0;

// ------------------- SERVO / MECHANICAL CONFIG -------------------
#define SERVO1_PIN 1   // FIN1, +Y
#define SERVO2_PIN 2   // FIN2, +X
#define SERVO3_PIN 3   // FIN3, -Y
#define SERVO4_PIN 4   // FIN4, -X

#define SERVO_CENTER_US 1500
#define SERVO_MIN_US    1000
#define SERVO_MAX_US    2000

// microseconds of servo pulse per degree of commanded fin deflection
#define US_PER_DEG 10.0

// Max fin deflection allowed in either direction (deg)
#define MAX_FIN_DEG 25.0

// Per-servo mechanical offset (deg) - use to physically straighten
// each fin since 0 deg commanded rarely means dead straight. Nudge in
// 1 deg steps until each fin sits straight at startup.
float offset1 = 0, offset2 = 0, offset3 = 0, offset4 = 0;

// Reverse a channel if it moves the wrong way (1 or -1)
int dir1 = 1, dir2 = 1, dir3 = 1, dir4 = 1;

Servo servo1, servo2, servo3, servo4;
float g_servo1_deg = 0, g_servo2_deg = 0, g_servo3_deg = 0, g_servo4_deg = 0;

// ------------------- BMI323 REGISTERS (SPI) -------------------
// Register map per Bosch BMI323 datasheet. SPI read = address byte
// with MSB set (addr | 0x80), followed by ONE dummy byte (the BMI3
// family always returns a throwaway byte first on SPI reads), then
// the actual data bytes, LSB first per 16-bit register.
#define BMI323_CS_PIN   A4
#define REG_CHIP_ID     0x00
#define REG_ACC_CONF    0x20
#define REG_GYR_CONF    0x21
#define REG_CMD         0x7E
#define REG_ACC_DATA_X  0x03  // ACC_DATA_X, Y, Z contiguous: 0x03-0x05
#define REG_GYR_DATA_X  0x06  // GYR_DATA_X, Y, Z contiguous: 0x06-0x08
#define BMI323_EXPECTED_CHIP_ID 0x43

// CAUTION: verify these two config words against the official BMI323
// datasheet's ACC_CONF/GYR_CONF bitfield tables before trusting the
// resulting scale factors below - the address/read-protocol are
// solid, but exact range/ODR bit positions should be double-checked
// against your datasheet copy. As configured here this targets
// +/-2g accel range and +/-250 deg/s gyro range at ~100Hz ODR to
// match the original MPU6050 scale factors.
#define ACC_CONF_VALUE  0x40A8  // ODR~100Hz, +/-2g range, normal mode (VERIFY)
#define GYR_CONF_VALUE  0x40A8  // ODR~100Hz, +/-250dps range, normal mode (VERIFY)

SPISettings bmiSPISettings(8000000, MSBFIRST, SPI_MODE0);

const float ACCEL_SCALE = 16384.0; // LSB/g at +/-2g (VERIFY against ACC_CONF_VALUE above)
const float GYRO_SCALE  = 131.0;   // LSB/(deg/s) at +/-250 deg/s (VERIFY against GYR_CONF_VALUE above)

int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;

float ax_g, ay_g, az_g;
float gx_dps, gy_dps, gz_dps;

float gx_bias = 0, gy_bias = 0, gz_bias = 0;

// Complementary filter state
float angle_x_est = 0.0; // deg, rotation about X, corrected by FIN2/FIN4
float angle_y_est = 0.0; // deg, rotation about Y, corrected by FIN1/FIN3
float rate_z_est = 0.0;  // deg/s, rotation about Z (spin), no absolute angle without magnetometer

// TEMPORARILY GYRO-ONLY: accelerometer leveling is disabled while the
// accel configuration/behavior is being debugged (see readMPU/
// updateComplementaryFilter below). angle_x_est/angle_y_est are pure
// gyro-rate integration for now, so expect them to drift over time -
// that's expected with this fallback, not a bug. Re-enable the
// complementary filter blend once accel readings check out (ax_g,
// ay_g, az_g in the Serial CSV should show ~1.0 on whichever axis
// points down, ~0 on the other two, and change sensibly as you tilt).

// ------------------- PID STATE -------------------
struct PIDState {
  float integral;
  float prev_error;
  float p_term, i_term, d_term; // last computed terms, for logging
};
PIDState pid_x_state = {0, 0, 0, 0, 0};
PIDState pid_y_state = {0, 0, 0, 0, 0};
PIDState pid_z_state = {0, 0, 0, 0, 0};

#define INTEGRAL_LIMIT 50.0

// Explicit prototype placed here (after the struct is defined) so the
// Arduino IDE's auto-generated prototype - which it inserts near the
// top of the file, before this struct exists - doesn't cause a
// "PIDState was not declared in this scope" error. Having our own
// prototype here stops Arduino from generating its own broken one.
float computePID(PIDState* s, float error, float Kp, float Ki, float Kd, float dt);

// ------------------- SD CARD -------------------
// CS on pin 10 - keeps clear of BMI323's CS (A4) and Servo1-4 (pins
// 1-4). MOSI/MISO/SCK (11/12/13) are shared with the BMI323, which is
// fine - SPI is a shared bus, each device just needs its own CS.
#define SD_CS_PIN 10
bool sd_ok = false;
File sdFile;
char sdFileName[16]; // 8.3 filename, e.g. "LOG0001.CSV"

// ------------------- TIMING -------------------
unsigned long last_loop_us = 0;
unsigned long last_log_ms = 0;
#define LOOP_HZ 200
#define LOOP_PERIOD_US (1000000UL / LOOP_HZ)
#define LOG_HZ 100
#define LOG_PERIOD_MS (1000UL / LOG_HZ)

void setup() {
  Serial.begin(115200);
  mpuInit();
  calibrateGyro();

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  centerAllServos();
  runFinTestSequence();

  initSD();

  Serial.println(F("time_ms,angle_x,angle_y,rate_z,gx,gy,gz,ax,ay,az,angle_x_sp,angle_y_sp,err_x,err_y,pid_x,pid_y,pid_z,servo1,servo2,servo3,servo4"));

  last_loop_us = micros();
  last_log_ms = millis();
}

void loop() {
  unsigned long now_us = micros();
  if (now_us - last_loop_us < LOOP_PERIOD_US) return;
  float dt = (now_us - last_loop_us) / 1000000.0;
  last_loop_us = now_us;

  readMPU();
  updateComplementaryFilter(dt);

  float err_x = angle_x_setpoint - angle_x_est;
  float err_y = angle_y_setpoint - angle_y_est;
  float err_z = 0.0 - rate_z_est; // rate-only "hold spin at 0"

  float pid_x = computePID(&pid_x_state, err_x, Kp_x, Ki_x, Kd_x, dt);
  float pid_y = computePID(&pid_y_state, err_y, Kp_y, Ki_y, Kd_y, dt);
  float pid_z = computePID(&pid_z_state, err_z, Kp_z, Ki_z, Kd_z, dt);

  mixAndDriveServos(pid_x, pid_y, pid_z);

  if (millis() - last_log_ms >= LOG_PERIOD_MS) {
    last_log_ms = millis();
    logData(err_x, err_y, pid_x, pid_y, pid_z, dt);
  }
}

// ==================== BMI323 (SPI) ====================

// Low-level 16-bit register write. BMI323 SPI write frame: address
// byte (MSB=0) followed by 2 data bytes, LSB first. No dummy byte
// needed on writes (only reads need it).
void bmiWriteReg16(uint8_t addr, uint16_t val) {
  SPI.beginTransaction(bmiSPISettings);
  digitalWrite(BMI323_CS_PIN, LOW);
  SPI.transfer(addr & 0x7F);
  SPI.transfer(val & 0xFF);
  SPI.transfer((val >> 8) & 0xFF);
  digitalWrite(BMI323_CS_PIN, HIGH);
  SPI.endTransaction();
}

// Low-level 16-bit register read. BMI323 SPI read frame: address
// byte with MSB set, then ONE dummy byte (sensor's internal pipeline
// delay), then 2 data bytes, LSB first.
uint16_t bmiReadReg16(uint8_t addr) {
  SPI.beginTransaction(bmiSPISettings);
  digitalWrite(BMI323_CS_PIN, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(0x00); // dummy byte
  uint8_t lsb = SPI.transfer(0x00);
  uint8_t msb = SPI.transfer(0x00);
  digitalWrite(BMI323_CS_PIN, HIGH);
  SPI.endTransaction();
  return ((uint16_t)msb << 8) | lsb;
}

// Burst-read numRegs contiguous 16-bit registers starting at startAddr
// into out[], one SPI transaction. Used to grab ACC_DATA_X..Z and
// GYR_DATA_X..Z in two calls (they're two separate contiguous blocks).
void bmiBurstRead(uint8_t startAddr, int16_t* out, uint8_t numRegs) {
  SPI.beginTransaction(bmiSPISettings);
  digitalWrite(BMI323_CS_PIN, LOW);
  SPI.transfer(startAddr | 0x80);
  SPI.transfer(0x00); // dummy byte
  for (uint8_t i = 0; i < numRegs; i++) {
    uint8_t lsb = SPI.transfer(0x00);
    uint8_t msb = SPI.transfer(0x00);
    out[i] = (int16_t)(((uint16_t)msb << 8) | lsb);
  }
  digitalWrite(BMI323_CS_PIN, HIGH);
  SPI.endTransaction();
}

void mpuInit() {
  pinMode(BMI323_CS_PIN, OUTPUT);
  digitalWrite(BMI323_CS_PIN, HIGH);
  SPI.begin();
  delay(10);

  uint16_t chipId = bmiReadReg16(REG_CHIP_ID) & 0xFF;
  Serial.print(F("BMI323 CHIP_ID: 0x"));
  Serial.println(chipId, HEX);
  if (chipId != BMI323_EXPECTED_CHIP_ID) {
    Serial.println(F("WARNING: BMI323 chip ID mismatch - check wiring/CS pin/SPI mode before trusting data."));
  }

  // Soft reset for a known state
  bmiWriteReg16(REG_CMD, 0xDEAF);
  delay(50);

  bmiWriteReg16(REG_ACC_CONF, ACC_CONF_VALUE);
  bmiWriteReg16(REG_GYR_CONF, GYR_CONF_VALUE);
  delay(100);
}

void readMPU() {
  int16_t accRaw[3];
  int16_t gyrRaw[3];
  bmiBurstRead(REG_ACC_DATA_X, accRaw, 3);
  bmiBurstRead(REG_GYR_DATA_X, gyrRaw, 3);

  ax_raw = accRaw[0]; ay_raw = accRaw[1]; az_raw = accRaw[2];
  gx_raw = gyrRaw[0]; gy_raw = gyrRaw[1]; gz_raw = gyrRaw[2];

  ax_g = ax_raw / ACCEL_SCALE;
  ay_g = ay_raw / ACCEL_SCALE;
  az_g = az_raw / ACCEL_SCALE;

  gx_dps = (gx_raw / GYRO_SCALE) - gx_bias;
  gy_dps = (gy_raw / GYRO_SCALE) - gy_bias;
  gz_dps = (gz_raw / GYRO_SCALE) - gz_bias;
}

void calibrateGyro() {
  Serial.println(F("Calibrating gyro, keep rocket still..."));
  const int N = 500;
  long sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < N; i++) {
    int16_t gyrRaw[3];
    bmiBurstRead(REG_GYR_DATA_X, gyrRaw, 3);
    sx += gyrRaw[0]; sy += gyrRaw[1]; sz += gyrRaw[2];
    delay(3);
  }
  gx_bias = (sx / (float)N) / GYRO_SCALE;
  gy_bias = (sy / (float)N) / GYRO_SCALE;
  gz_bias = (sz / (float)N) / GYRO_SCALE;
  Serial.println(F("Gyro calibration done."));
}

// GYRO-ONLY for now (accel leveling disabled while accel config is
// being debugged - see the diagnosis notes near angle_x_est above).
// This is plain gyro-rate integration: angle_est += rate * dt. It
// WILL drift over time since nothing pulls it back to a true level
// reference - that's expected here, not a bug. Re-enable the
// complementary blend with accel once ax_g/ay_g/az_g check out.
void updateComplementaryFilter(float dt) {
  angle_x_est += gx_dps * dt;
  angle_y_est += gy_dps * dt;

  rate_z_est = gz_dps;
}

// ==================== PID ====================

float computePID(PIDState* s, float error, float Kp, float Ki, float Kd, float dt) {
  s->integral += error * dt;
  if (s->integral > INTEGRAL_LIMIT) s->integral = INTEGRAL_LIMIT;
  if (s->integral < -INTEGRAL_LIMIT) s->integral = -INTEGRAL_LIMIT;

  float derivative = (dt > 0) ? (error - s->prev_error) / dt : 0;
  s->prev_error = error;

  s->p_term = Kp * error;
  s->i_term = Ki * s->integral;
  s->d_term = Kd * derivative;

  return s->p_term + s->i_term + s->d_term;
}

// ==================== SERVO MIXING ====================
// pid_x  -> corrected by FIN2 & FIN4 (mirrored: opposite sign so they
//           push the exhaust the same real-world direction)
// pid_y  -> corrected by FIN1 & FIN3 (mirrored, same idea)
// pid_z  -> spin/roll correction, added with the SAME sign to all 4
//           fins so they deflect together in one rotational sense

void mixAndDriveServos(float pid_x, float pid_y, float pid_z) {
  float fin1 = pid_y + pid_z + offset1;
  float fin3 = -pid_y + pid_z + offset3;
  float fin2 = pid_x + pid_z + offset2;
  float fin4 = -pid_x + pid_z + offset4;

  fin1 = constrain(fin1, -MAX_FIN_DEG, MAX_FIN_DEG);
  fin2 = constrain(fin2, -MAX_FIN_DEG, MAX_FIN_DEG);
  fin3 = constrain(fin3, -MAX_FIN_DEG, MAX_FIN_DEG);
  fin4 = constrain(fin4, -MAX_FIN_DEG, MAX_FIN_DEG);

  writeServoDeg(servo1, fin1, dir1);
  writeServoDeg(servo2, fin2, dir2);
  writeServoDeg(servo3, fin3, dir3);
  writeServoDeg(servo4, fin4, dir4);

  g_servo1_deg = fin1; g_servo2_deg = fin2; g_servo3_deg = fin3; g_servo4_deg = fin4;
}

void writeServoDeg(Servo &s, float deg, int dir) {
  int us = SERVO_CENTER_US + (int)(dir * deg * US_PER_DEG);
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  s.writeMicroseconds(us);
}

void centerAllServos() {
  // raw 0 first
  writeServoDeg(servo1, 0, dir1);
  writeServoDeg(servo2, 0, dir2);
  writeServoDeg(servo3, 0, dir3);
  writeServoDeg(servo4, 0, dir4);
  delay(300);
  // then apply mechanical offset so fins sit physically straight
  writeServoDeg(servo1, offset1, dir1);
  writeServoDeg(servo2, offset2, dir2);
  writeServoDeg(servo3, offset3, dir3);
  writeServoDeg(servo4, offset4, dir4);
  g_servo1_deg = offset1; g_servo2_deg = offset2;
  g_servo3_deg = offset3; g_servo4_deg = offset4;
  delay(500);
}

// ==================== STARTUP FIN TEST SEQUENCE ====================
// Exercises each control axis on boot so you can visually confirm
// correct fin motion before the PID loop ever engages.

#define TEST_SWEEP_DEG 15.0
#define TEST_HOLD_MS   700

void centerAndHold(unsigned long hold_ms) {
  writeServoDeg(servo1, offset1, dir1);
  writeServoDeg(servo2, offset2, dir2);
  writeServoDeg(servo3, offset3, dir3);
  writeServoDeg(servo4, offset4, dir4);
  delay(hold_ms);
}

void runFinTestSequence() {
  Serial.println(F("Starting fin test sequence..."));

  // 1. All fins to zero, hold 3 sec
  Serial.println(F("  [1] Center all fins (3s)"));
  centerAndHold(3000);

  // 2. FIN1 & FIN3 (yaw pair, Y axis) sweep left, right, back to zero
  Serial.println(F("  [2] Yaw pair: FIN1/FIN3 sweep"));
  writeServoDeg(servo1, TEST_SWEEP_DEG + offset1, dir1);
  writeServoDeg(servo3, -TEST_SWEEP_DEG + offset3, dir3);
  delay(TEST_HOLD_MS);
  writeServoDeg(servo1, -TEST_SWEEP_DEG + offset1, dir1);
  writeServoDeg(servo3, TEST_SWEEP_DEG + offset3, dir3);
  delay(TEST_HOLD_MS);
  centerAndHold(TEST_HOLD_MS);

  // 3. FIN2 & FIN4 (pitch pair, X axis) sweep back and forth, back to zero
  Serial.println(F("  [3] Pitch pair: FIN2/FIN4 sweep"));
  writeServoDeg(servo2, TEST_SWEEP_DEG + offset2, dir2);
  writeServoDeg(servo4, -TEST_SWEEP_DEG + offset4, dir4);
  delay(TEST_HOLD_MS);
  writeServoDeg(servo2, -TEST_SWEEP_DEG + offset2, dir2);
  writeServoDeg(servo4, TEST_SWEEP_DEG + offset4, dir4);
  delay(TEST_HOLD_MS);
  centerAndHold(TEST_HOLD_MS);

  // 4. All 4 fins sweep together one direction, then the other, then zero
  Serial.println(F("  [4] Roll/swirl: all 4 fins together"));
  writeServoDeg(servo1, TEST_SWEEP_DEG + offset1, dir1);
  writeServoDeg(servo2, TEST_SWEEP_DEG + offset2, dir2);
  writeServoDeg(servo3, TEST_SWEEP_DEG + offset3, dir3);
  writeServoDeg(servo4, TEST_SWEEP_DEG + offset4, dir4);
  delay(TEST_HOLD_MS);
  writeServoDeg(servo1, -TEST_SWEEP_DEG + offset1, dir1);
  writeServoDeg(servo2, -TEST_SWEEP_DEG + offset2, dir2);
  writeServoDeg(servo3, -TEST_SWEEP_DEG + offset3, dir3);
  writeServoDeg(servo4, -TEST_SWEEP_DEG + offset4, dir4);
  delay(TEST_HOLD_MS);
  centerAndHold(TEST_HOLD_MS);

  Serial.println(F("Fin test sequence complete. Starting control loop."));
}

// ==================== SD CARD ====================

void initSD() {
  Serial.println(F("=== SD card init ==="));
  Serial.print(F("Chip select pin: "));
  Serial.println(SD_CS_PIN);

  // Use SdFat directly via SD.sdfs.begin() with an explicit slow SPI
  // clock (4 MHz) instead of plain SD.begin(SD_CS_PIN). The default
  // driver/speed was unreliable on this hardware; 4 MHz is the speed
  // confirmed working. See the "SD CARD NOTES" block at the top of
  // this file for more detail.
  bool ok = SD.sdfs.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(4)));

  if (!ok) {
    Serial.println(F("SD FAILED to initialize."));
    Serial.println(F("  Checklist:"));
    Serial.println(F("  - Is a card inserted?"));
    Serial.println(F("  - Is CS wired to SD_CS_PIN (see #define above)?"));
    Serial.println(F("  - Are MOSI/MISO/SCK on the correct hardware SPI pins (shared with BMI323)?"));
    Serial.println(F("  - Is the card formatted FAT16/FAT32 (not exFAT)?"));
    Serial.println(F("  - Try a different/known-good SD card or module."));
    Serial.println(F("Continuing with Serial-only logging."));
    sd_ok = false;
    return;
  }
  Serial.println(F("SD.sdfs.begin() OK."));

  // Report basic card info so a bad/near-dead card is visible in the log.
  uint32_t sizeMB = (uint32_t)(SD.sdfs.card()->sectorCount() / 2048UL);
  Serial.print(F("Card size: "));
  Serial.print(sizeMB);
  Serial.println(F(" MB"));

  // Find the next free LOGnnnn.CSV filename so we don't overwrite runs
  for (int i = 1; i <= 9999; i++) {
    snprintf(sdFileName, sizeof(sdFileName), "LOG%04d.CSV", i);
    if (!SD.exists(sdFileName)) break;
  }

  sdFile = SD.open(sdFileName, FILE_WRITE);
  if (!sdFile) {
    Serial.print(F("FAILED to open log file: "));
    Serial.println(sdFileName);
    Serial.println(F("Continuing with Serial-only logging."));
    sd_ok = false;
    return;
  }

  Serial.print(F("Log file opened OK: "));
  Serial.println(sdFileName);

  // Quick round-trip write/flush test so a card that "opens" but can't
  // actually be written to (write-protected, near full, failing) is
  // caught here instead of silently losing data all flight.
  size_t written = sdFile.print(F("# sd_test_ok\n"));
  sdFile.flush();
  if (written == 0) {
    Serial.println(F("SD WRITE TEST FAILED (0 bytes written)."));
    Serial.println(F("Continuing with Serial-only logging."));
    sdFile.close();
    sd_ok = false;
    return;
  }

  sdFile.println(F("time_ms,angle_x,angle_y,rate_z,gx,gy,gz,ax,ay,az,"
                    "angle_x_sp,angle_y_sp,err_x,err_y,"
                    "pid_x,pid_y,pid_z,"
                    "servo1,servo2,servo3,servo4"));
  sdFile.flush();
  sd_ok = true;
  Serial.print(F("SD write test OK. Logging to "));
  Serial.println(sdFileName);
  Serial.println(F("===================="));
}

// ==================== LOGGING ====================

void logData(float err_x, float err_y, float pid_x, float pid_y, float pid_z, float dt) {
  // ---- Serial: lean stream for live tuning ----
  Serial.print(millis()); Serial.print(",");
  Serial.print(angle_x_est, 3); Serial.print(",");
  Serial.print(angle_y_est, 3); Serial.print(",");
  Serial.print(rate_z_est, 3); Serial.print(",");
  Serial.print(gx_dps, 3); Serial.print(",");
  Serial.print(gy_dps, 3); Serial.print(",");
  Serial.print(gz_dps, 3); Serial.print(",");
  Serial.print(ax_g, 3); Serial.print(",");
  Serial.print(ay_g, 3); Serial.print(",");
  Serial.print(az_g, 3); Serial.print(",");
  Serial.print(angle_x_setpoint, 2); Serial.print(",");
  Serial.print(angle_y_setpoint, 2); Serial.print(",");
  Serial.print(err_x, 3); Serial.print(",");
  Serial.print(err_y, 3); Serial.print(",");
  Serial.print(pid_x, 3); Serial.print(",");
  Serial.print(pid_y, 3); Serial.print(",");
  Serial.print(pid_z, 3); Serial.print(",");
  Serial.print(g_servo1_deg, 2); Serial.print(",");
  Serial.print(g_servo2_deg, 2); Serial.print(",");
  Serial.print(g_servo3_deg, 2); Serial.print(",");
  Serial.println(g_servo4_deg, 2);

  // ---- SD: same columns, written to the card so you keep a full
  // record beyond what's practical to scroll through live ----
  if (sd_ok) {
    sdFile.print(millis()); sdFile.print(",");
    sdFile.print(angle_x_est, 3); sdFile.print(",");
    sdFile.print(angle_y_est, 3); sdFile.print(",");
    sdFile.print(rate_z_est, 3); sdFile.print(",");
    sdFile.print(gx_dps, 3); sdFile.print(",");
    sdFile.print(gy_dps, 3); sdFile.print(",");
    sdFile.print(gz_dps, 3); sdFile.print(",");
    sdFile.print(ax_g, 3); sdFile.print(",");
    sdFile.print(ay_g, 3); sdFile.print(",");
    sdFile.print(az_g, 3); sdFile.print(",");
    sdFile.print(angle_x_setpoint, 2); sdFile.print(",");
    sdFile.print(angle_y_setpoint, 2); sdFile.print(",");
    sdFile.print(err_x, 3); sdFile.print(",");
    sdFile.print(err_y, 3); sdFile.print(",");
    sdFile.print(pid_x, 3); sdFile.print(",");
    sdFile.print(pid_y, 3); sdFile.print(",");
    sdFile.print(pid_z, 3); sdFile.print(",");
    sdFile.print(g_servo1_deg, 2); sdFile.print(",");
    sdFile.print(g_servo2_deg, 2); sdFile.print(",");
    sdFile.print(g_servo3_deg, 2); sdFile.print(",");
    sdFile.println(g_servo4_deg, 2);

    // Flush periodically rather than every line. Also treat a failed
    // write mid-flight (card pulled, full, etc.) as a reason to stop
    // trying, same as the boot-time write test above.
    static uint8_t flushCounter = 0;
    if (++flushCounter >= 20) { // ~every 0.2s at 100Hz
      sdFile.flush();
      flushCounter = 0;
    }
  }
}
