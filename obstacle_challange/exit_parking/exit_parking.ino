// =====================================================================
//  exit_parking.ino  -  Controlled C.H.A.O.S. v2  -  WRO 2026 FE Obstacle Challenge
//
//  Leaves the parking lot and stops with the CAMERA CENTRE (= robot centre) on the
//  same spot for both driving directions, heading along the driving direction:
//      CCW  -> robot ends facing right, CW -> robot ends facing left.
//
//  How it works
//    1. Reads the side ToFs: the side with the close wall is the outer wall.
//       wall on the RIGHT -> CCW,  wall on the LEFT -> CW.  (Robot starts facing the driving direction.)
//    2. Reads the front ToF (gap to the front magenta block) -> knows where it stands in the lot.
//    3. Works in a "lane frame" that is identical for CW and CCW (the whole code is just mirrored):
//         u = along the driving direction, 0 = rear block, LOT_LEN = front block
//         v = distance from the outer wall (toward the lane)
//         psi = heading, positive = turning toward the lane
//    4. Pose = encoder (distance) + gyro (heading). The lot is only 4 cm longer than the robot
//       at each end, so it zig-zags (forward on full lock toward the lane, reverse on full lock
//       the other way, ...) and stops every time the NEXT 6 mm would bring the robot body
//       (4 sides) closer than a margin to a block or wall. As soon as a full-lock forward arc
//       can take it out of the lot without touching anything, it does that.
//    5. In the free lane: S-curve to the lane line v = FINAL_V_MM, heading 0, then straight
//       forward/backward until the robot centre is at u = FINAL_U_MM (the middle of the lot).
//
//  Needs: ChaosPins.h / ChaosCheck.h (same folder as your open_challenge), library "VL53L1X" (Pololu).
//  Start: START button or 's' in Serial Monitor (115200). Press 'd' for the live debug view.
// =====================================================================
#include <Wire.h>
#include <VL53L1X.h>
#include "../../includes/includes.ino"
#include "../../includes/ChaosCheck.h"

// ---------------- LOT (WRO rules: 20 cm deep, 1.5 x robot length long, blocks 20 mm thick) ----------------
const float ROBOT_LEN_MM     = 160.0;
const float ROBOT_W_MM       = 140.0;
const float LOT_DEPTH_MM     = 200.0;
const float LOT_LEN_MM       = 1.5 * ROBOT_LEN_MM;   // 240
const float BLOCK_T_MM       = 20.0;

// ---------------- ROBOT GEOMETRY (MEASURE!) ----------------
const float WHEELBASE_MM       = 100.0;              // front axle <-> rear axle
const float REAR_OVERHANG_MM   = 27.0;               // rear axle centre -> rear-most point of the robot
const float FRONT_OVERHANG_MM  = ROBOT_LEN_MM - WHEELBASE_MM - REAR_OVERHANG_MM;   // front axle -> front-most point
const float STEER_LOCK_DEG     = 30.0;               // wheel angle at full lock (STEER_LEFT / STEER_RIGHT). MEASURE:
                                                     // atan(WHEELBASE / radius of the circle the rear axle centre drives at full lock)
const float PLAN_LOCK_FACTOR   = 0.9;                // the "can I leave now?" check assumes a 10 % wider turning circle (safety)
const int   ENC_SIGN           = 1;                  // +1 if encCount grows when driving FORWARD (check with 'd')

// ---------------- WHERE TO STOP ----------------
const float FINAL_V_MM       = 500.0;                // robot centre distance from the outer wall (1000 mm lane -> middle)
const float FINAL_U_MM       = LOT_LEN_MM / 2.0;     // robot centre in the middle of the lot length
const float PSI_APPROACH_DEG = 45.0;                 // heading of the straight part of the S-curve

// ---------------- SAFETY MARGINS ----------------
const float MARGIN_BLOCK_MM  = 8.0;                  // keep this far from the magenta blocks (touching = round over!).
                                                     // Do NOT raise above 8: the lot has only 40 mm slack per end, with 9 mm the exit becomes impossible.
const float MARGIN_WALL_MM   = 3.0;                  // outer wall may be touched in the Obstacle Challenge, but avoid it
const float LOOKAHEAD_MM     = 6.0;                  // stop when the next 6 mm would break the margin

// ---------------- SPEEDS / REGULATION ----------------
const int   MANEUVER_PWM     = 100;
const float KP_HOLD          = 1.5;                  // servo deg per deg of heading error
const float HOLD_MAX_DEG     = 15.0;
const unsigned long LEG_TIMEOUT_MS   = 3000;
const unsigned long PHASE_TIMEOUT_MS = 6000;
const int   MAX_LEGS         = 30;

// ---------------- MG995 (same as open_challenge) ----------------
const int   STEER_US_MIN     = 500;
const int   STEER_US_MAX     = 2500;

const float DEG2RAD = PI / 180.0;

// =====================================================================
//  Hardware helpers (copied from open_challenge.ino)
// =====================================================================
struct Tof {
  VL53L1X s;
  uint8_t ch;
  int     last;
  bool    fresh;
};
Tof tofRF = {VL53L1X(), MUX_TOF_RIGHT_FRONT, -1, false};
Tof tofRR = {VL53L1X(), MUX_TOF_RIGHT_REAR,  -1, false};
Tof tofLF = {VL53L1X(), MUX_TOF_LEFT_FRONT,  -1, false};
Tof tofLR = {VL53L1X(), MUX_TOF_LEFT_REAR,   -1, false};
Tof tofF  = {VL53L1X(), MUX_TOF_FRONT,       -1, false};

volatile long encCount = 0;
float gyroBias = 0;
float yawDeg   = 0;                   // + = turn to the left
unsigned long lastYawUs = 0;
int imuErrors = 0;

void IRAM_ATTR encISR() { if (digitalRead(ENC_B)) encCount++; else encCount--; }

void motor(int speed) {               // + forward, - reverse
  digitalWrite(STBY, HIGH);
  if (speed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else            { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); speed = -speed; }
  ledcWrite(PWMA, constrain(speed, 0, 255));
}
void brake()    { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); ledcWrite(PWMA, 255); }
void motorOff() { ledcWrite(PWMA, 0); digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); digitalWrite(STBY, LOW); }

void steerTo(float deg) {
  deg = constrain(deg, (float)STEER_LEFT, (float)STEER_RIGHT);
  long us = STEER_US_MIN + (long)(deg * (STEER_US_MAX - STEER_US_MIN) / 180.0);
  ledcWrite(STEER_PIN, us * ((1L << SERVO_PWM_BITS) - 1) / 20000L);
}

void mux(uint8_t ch) { Wire.beginTransmission(ADDR_MUX); Wire.write(1 << ch); Wire.endTransmission(); }

bool initTof(Tof &t, VL53L1X::DistanceMode mode, uint32_t budgetUs) {
  mux(t.ch);
  t.s.setTimeout(100);
  if (!t.s.init()) return false;
  t.s.setDistanceMode(mode);
  t.s.setMeasurementTimingBudget(budgetUs);
  t.s.setROISize(4, 4);
  t.s.setROICenter(199);
  t.s.startContinuous(budgetUs / 1000 + 5);
  return true;
}

int readTof(Tof &t) {                 // does not wait
  mux(t.ch);
  t.fresh = false;
  if (t.s.dataReady()) {
    t.s.read(false);
    t.last  = (t.s.ranging_data.range_status == VL53L1X::RangeValid) ? t.s.ranging_data.range_mm : -1;
    t.fresh = true;
  }
  return t.last;
}

void mpuWrite(uint8_t reg, uint8_t val) { Wire.beginTransmission(ADDR_MPU); Wire.write(reg); Wire.write(val); Wire.endTransmission(); }
int16_t gyroZraw() {
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((uint8_t)ADDR_MPU, (uint8_t)2) != 2) { imuErrors++; return (int16_t)gyroBias; }
  imuErrors = 0;
  uint8_t hi = Wire.read(), lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}
bool imuLost() { return imuErrors >= 5; }
void mpuInit() { mpuWrite(0x6B, 0x00); mpuWrite(0x1A, 0x03); mpuWrite(0x1B, 0x08); delay(100); }

void halt(const char *why) {
  motorOff();
  Serial.print("ERROR: "); Serial.println(why);
  while (true) delay(1000);
}

void gyroCalibrate() {
  long sum = 0;
  for (int i = 0; i < 500; i++) { sum += gyroZraw(); delay(2); }
  gyroBias = sum / 500.0;
  if (imuLost()) halt("MPU-6050 not responding during calibration");
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastYawUs) * 1e-6;
  lastYawUs = now;
  yawDeg += (gyroZraw() - gyroBias) / 65.5 * dt;     // + = left (same as open_challenge with GYRO_SIGN = 1)
}

bool startPressed() {
  if (digitalRead(START_PIN) == LOW) return true;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') return true;
  }
  return false;
}

// =====================================================================
//  POSE (lane frame). Reference point = centre of the REAR axle (that is what the encoder measures).
// =====================================================================
int   laneSign = 0;                   // +1 = CCW (lane on the LEFT), -1 = CW (lane on the RIGHT)
float pu = 0, pv = 0, psi = 0;        // rear axle centre [mm], heading [rad], positive = toward the lane
float pathMm = 0;                     // total distance driven (always counts up)
long  lastEnc = 0;

float centerOffsetMm() { return (WHEELBASE_MM + FRONT_OVERHANG_MM - REAR_OVERHANG_MM) / 2.0; }
float centerU() { return pu + centerOffsetMm() * cosf(psi); }
float centerV() { return pv + centerOffsetMm() * sinf(psi); }

void updatePose() {
  updateYaw();
  long e = encCount;
  float d = ENC_SIGN * (e - lastEnc) * WHEEL_CIRCUMFERENCE_MM / ENC_PULSES_PER_WHEEL_REV;
  lastEnc = e;
  float newPsi = laneSign * yawDeg * DEG2RAD;
  float pm = 0.5 * (psi + newPsi);
  pu += d * cosf(pm);
  pv += d * sinf(pm);
  psi = newPsi;
  pathMm += fabsf(d);
}

void settleMs(unsigned long ms) {     // wait, but keep the pose up to date
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { updatePose(); delay(2); }
}

// delta > 0 = wheels toward the lane
void steerLaneDeg(float delta) { steerTo(STEER_CENTER - laneSign * delta); }
void steerLock(bool towardLane) {
  bool left = (laneSign > 0) ? towardLane : !towardLane;
  steerTo(left ? STEER_LEFT : STEER_RIGHT);
}

void stopMotor() { brake(); settleMs(200); ledcWrite(PWMA, 0); }

// =====================================================================
//  FOOTPRINT vs. LOT  (4 sides of the robot sampled every ~15 mm)
// =====================================================================
const int NPT = 40;
float fpx[NPT], fpy[NPT];
void buildFootprint() {
  float x0 = -REAR_OVERHANG_MM, x1 = WHEELBASE_MM + FRONT_OVERHANG_MM, y0 = -ROBOT_W_MM / 2, y1 = ROBOT_W_MM / 2;
  float cx[4] = {x0, x0, x1, x1}, cy[4] = {y0, y1, y1, y0};
  int n = 0;
  for (int i = 0; i < 4; i++) {
    int j = (i + 1) % 4;
    for (int k = 0; k < 10; k++) { fpx[n] = cx[i] + (cx[j] - cx[i]) * k / 10.0; fpy[n] = cy[i] + (cy[j] - cy[i]) * k / 10.0; n++; }
  }
}

// true = robot at this pose is closer than the margin to a block or the wall. minV = lowest point of the robot.
bool footprintHits(float u, float v, float p, float *minV) {
  float c = cosf(p), s = sinf(p), low = 1e9;
  bool hit = false;
  for (int i = 0; i < NPT; i++) {
    float x = u + fpx[i] * c - fpy[i] * s;
    float y = v + fpx[i] * s + fpy[i] * c;
    if (y < low) low = y;
    if (y < MARGIN_WALL_MM) hit = true;
    if (y < LOT_DEPTH_MM + MARGIN_BLOCK_MM) {
      if (x > -BLOCK_T_MM - MARGIN_BLOCK_MM && x < MARGIN_BLOCK_MM) hit = true;                           // rear block
      if (x > LOT_LEN_MM - MARGIN_BLOCK_MM && x < LOT_LEN_MM + BLOCK_T_MM + MARGIN_BLOCK_MM) hit = true;  // front block
    }
  }
  if (minV) *minV = low;
  return hit;
}

bool hitsAt(float u, float v, float p) { float low; return footprintHits(u, v, p, &low); }

void arcStep(float &u, float &v, float &p, float d, float deltaRad) {   // bicycle model, d < 0 = reverse
  float dp = d / WHEELBASE_MM * tanf(deltaRad);
  float pm = p + dp / 2;
  u += d * cosf(pm);
  v += d * sinf(pm);
  p += dp;
}

bool outOfLot(float u, float v, float p) {
  float low;
  footprintHits(u, v, p, &low);
  return low > LOT_DEPTH_MM + MARGIN_BLOCK_MM;
}

// Would driving FORWARD on full lock toward the lane take us out of the lot without touching anything?
bool lockArcClearsLot(float u, float v, float p) {
  float lock = STEER_LOCK_DEG * PLAN_LOCK_FACTOR * DEG2RAD;
  for (int i = 0; i < 150; i++) {
    arcStep(u, v, p, 4.0, lock);
    if (hitsAt(u, v, p)) return false;
    if (outOfLot(u, v, p)) return true;
  }
  return false;
}

// =====================================================================
//  GENERIC "DRIVE UNTIL" BLOCK
//    dirn  = +1 forward, -1 reverse
//    mode  = M_TOWARD / M_AWAY = full lock toward / away from the lane, M_HOLD = hold heading holdPsiRad
// =====================================================================
enum { M_TOWARD, M_AWAY, M_HOLD };
enum { R_DONE, R_BLOCKED, R_TIMEOUT, R_FAIL };
float holdPsiRad = 0;

int driveUntil(int dirn, int mode, bool (*done)(), unsigned long timeoutMs) {
  const float lock = STEER_LOCK_DEG * DEG2RAD;
  if (mode == M_TOWARD) steerLock(true);
  else if (mode == M_AWAY) steerLock(false);
  else steerLaneDeg(0);
  settleMs(250);                                       // servo gets there

  int result;
  unsigned long t0 = millis();
  motor(dirn * MANEUVER_PWM);
  while (true) {
    updatePose();
    if (imuLost())                      { result = R_FAIL;    break; }
    if (millis() - t0 > timeoutMs)      { result = R_TIMEOUT; break; }
    if (done && done())                 { result = R_DONE;    break; }

    float delta = (mode == M_TOWARD) ? lock : (mode == M_AWAY) ? -lock : 0;
    if (mode == M_HOLD) {
      float err = (psi - holdPsiRad) / DEG2RAD;
      float cmd = constrain((dirn > 0 ? -1 : 1) * KP_HOLD * err, -HOLD_MAX_DEG, HOLD_MAX_DEG);
      steerLaneDeg(cmd);
    }
    float u = pu, v = pv, p = psi;                     // would the next few mm break the margin?
    arcStep(u, v, p, dirn * LOOKAHEAD_MM, delta);
    if (hitsAt(u, v, p)) { result = R_BLOCKED; break; }
    delay(2);
  }
  stopMotor();
  return result;
}

// ---- end conditions (plain functions, they read the globals) ----
float turnRadiusMm = 173;
float lastCheckPath = 0;
bool condCanLeave()   {                                // check only every 6 mm of driving (it is a small simulation)
  if (pathMm - lastCheckPath < 6.0) return false;
  lastCheckPath = pathMm;
  return lockArcClearsLot(pu, pv, psi);
}
bool condOutOfLot()   { return outOfLot(pu, pv, psi); }
bool condPsiApproach(){ return psi <= PSI_APPROACH_DEG * DEG2RAD; }
bool condVReached()   { return pv >= FINAL_V_MM - turnRadiusMm * (1.0 - cosf(psi)); }   // rear axle v: centre v = rear v at psi = 0
bool condPsiZero()    { return psi <= 0; }
int  uDir = 1;
bool condUReached()   { return (uDir > 0) ? (centerU() >= FINAL_U_MM) : (centerU() <= FINAL_U_MM); }

bool fail(const char *why) {
  brake(); delay(200); motorOff();
  steerTo(STEER_CENTER);
  Serial.print("STOP: "); Serial.println(why);
  return false;
}

// =====================================================================
//  START: direction + where in the lot we are
// =====================================================================
char decideSide(Tof &a, Tof &b) {                       // average of VALID readings, -1 = none
  int x = readTof(a); // left Tof reading
  int y = readTof(b); // right Tof reading
  if(abs(x-y)>50) return 'L';
  else return 'R';
}


// =====================================================================
//  THE MANEUVER
// =====================================================================
bool exitParkingLeft() {
  //car.turn_radius(-30, 100);
  car.straight(-40);
}


// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== exit_parking ===");

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(STBY, OUTPUT);
  ledcAttach(PWMA, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  motorOff();
  ledcAttach(STEER_PIN, SERVO_PWM_FREQ, SERVO_PWM_BITS);
  steerTo(STEER_CENTER);

  pinMode(ENC_A, INPUT); pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, RISING);
  pinMode(START_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  if (chaosI2CCheck(false) > 0) halt("I2C check failed - DO NOT DRIVE");
  mpuInit();
  if (!initTof(tofRF, VL53L1X::Short, 33000)) halt("right front ToF (channel 0)");
  if (!initTof(tofRR, VL53L1X::Short, 33000)) halt("right rear ToF (channel 1)");
  if (!initTof(tofLF, VL53L1X::Short, 33000)) halt("left front ToF (channel 2)");
  if (!initTof(tofF,  VL53L1X::Long,  50000)) halt("front ToF (channel 3)");
  if (!initTof(tofLR, VL53L1X::Short, 33000)) halt("left rear ToF (channel 4)");

  buildFootprint();
  Serial.println("Ready. START or 's' (put the robot in the lot facing the driving direction). 'd' = live debug.");
}

void loop() {
  if (!startPressed()) return;

  Serial.println("Do not touch the robot - gyro calibration...");
  delay(1000);
  gyroCalibrate();

  unsigned long t0 = millis();
  exitParkingLeft();
  Serial.printf("Time: %.1f s\n", (millis() - t0) / 1000.0);

  Serial.println("Run again: START or 's'.");
  while (Serial.available()) Serial.read();
  delay(500);
}