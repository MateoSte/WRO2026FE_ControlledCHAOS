// =====================================================================
//  open_challenge.ino  -  Controlled C.H.A.O.S. v2
//
//  Zadatak (3 kruga oko pravokutne staze, stop na startnom mjestu):
//    1. Od starta vozi ravno (drzi smjer ziroskopom) do 50 cm od prednjeg zida.
//    2. Stane, izmjeri lijevo i desno. Strana s BLISKIM zidom = vanjski zid,
//       daleka strana = slobodan put.
//         slobodno lijevo -> COUNTERCLOCKWISE: skrece lijevo, prati DESNI zid
//         slobodno desno  -> CLOCKWISE:        skrece desno,  prati LIJEVI zid
//    3. Ponavlja: skretanje 90 st. -> voznja uz vanjski zid na 30 cm
//       -> stop 50 cm prije prednjeg zida. Ukupno 12 skretanja (3 x 4).
//    4. Nakon 12. skretanja vozi jos onoliko koliko treba da stane
//       na startnom mjestu (racuna se iz enkodera, vidi STARTNO MJESTO).
//
//  Senzori (MUX): 0 desni prednji, 1 desni zadnji, 2 lijevi prednji,
//                 3 prednji, 4 lijevi zadnji.  MPU-6050 na glavnoj sabirnici.
//
//  Potrebno: ChaosPins.h i ChaosCheck.h u istoj mapi, biblioteka "VL53L1X" by Pololu.
//  Pokretanje: START tipkalo ili 's' u Serial Monitoru (115200).
//  ToF test:   't' u Serial Monitoru (bez voznje, bilo koja tipka ga zaustavlja).
// =====================================================================
#include <Wire.h>
#include <VL53L1X.h>
#include "../includes/includes.ino"
#include "../includes/ChaosCheck.h"   // provjera svih I2C uredjaja prije starta

// ---------------- STAZA ----------------
const int   LAPS            = 3;
const int   TURNS_PER_LAP   = 4;
const float WALL_TARGET_MM  = 200.0;  // 20 cm od vanjskog zida (od lica senzora)
const int   FRONT_STOP_MM   = 400;    // stani 40 cm prije prednjeg zida
const float MIN_STRAIGHT_MM = 300.0;  // nakon skretanja ne gledaj prednji senzor prvih 30 cm
const int   EMERGENCY_MM    = 150;    // zid blize od 15 cm = HITNI STOP, uvijek
const bool  STOP_AT_CORNERS = true;   // true = stane pred svakim zavojem (kao u prvom)

// ---------------- BRZINE ----------------
const int   DRIVE_PWM       = 200;
const int   TURN_PWM        = 100;

// ---------------- GEOMETRIJA (IZMJERITI!) ----------------
const float TOF_SPACING_R_MM = 120.0; // TODO: razmak desni prednji <-> desni zadnji ToF
const float TOF_SPACING_L_MM = 120.0; // TODO: razmak lijevi prednji <-> lijevi zadnji ToF

// ---------------- REGULACIJA (iz voznja_desni_zid) ----------------
const float KP_DIST         = 0.08;
const float KP_ANGLE        = 0.8;
const float KP_HEADING      = 1.0;
const int   WALL_MAX_MM     = 800;    // dalje od ovoga = ne pratimo zid, drzimo smjer

// ---------------- SKRETANJE ----------------
const float TURN_DEG        = 90.0;
const float TURN_EARLY_DEG  = 6.0;    // ista vrijednost kao u testu skretanja
const int   GYRO_SIGN       = 1;

// ---------------- SIGURNOST ----------------
const unsigned long STRAIGHT_TIMEOUT_MS = 10000;
const unsigned long TURN_TIMEOUT_MS     = 5000;
const bool  BEZ_MOTORA      = false;

// ---------------- ToF VIDNO POLJE ----------------
const uint8_t TOF_ROI_SIZE   = 4;     // 4x4 SPAD = najuze moguce (~15 st.); 16 = puno (~27 st.)
const uint8_t TOF_ROI_CENTER = 199;   // sredina SPAD polja (gleda ravno)

// ---------------- MG995 (isto kao servo_center.ino) ----------------
const int   STEER_US_MIN    = 500;
const int   STEER_US_MAX    = 2500;

// =====================================================================
//  ToF senzor iza MUX-a
// =====================================================================
struct Tof {
  VL53L1X s;
  uint8_t ch;
  int     last;    // zadnje valjano ocitanje (mm), -1 = nema cilja / neispravno
  bool    fresh;   // true = upravo stiglo novo mjerenje
  uint8_t status;  // zadnji range_status (0 = valjano), za dijagnostiku
};

Tof tofRF = {VL53L1X(), MUX_TOF_RIGHT_FRONT, -1, false};
Tof tofRR = {VL53L1X(), MUX_TOF_RIGHT_REAR,  -1, false};
Tof tofLF = {VL53L1X(), MUX_TOF_LEFT_FRONT,  -1, false};
Tof tofLR = {VL53L1X(), MUX_TOF_LEFT_REAR,   -1, false};
Tof tofF  = {VL53L1X(), MUX_TOF_FRONT,       -1, false};

volatile long encCount = 0;
float gyroBias = 0;
float yawDeg   = 0;                   // + = okret ulijevo
unsigned long lastYawUs = 0;

int dir = 0;                          // +1 = CCW (lijevo), -1 = CW (desno), 0 = jos ne znamo

// ---------------- ENKODER ----------------
void IRAM_ATTR encISR() {
  if (digitalRead(ENC_B)) encCount++; else encCount--;
}

float pulsesToMm(long p) { return labs(p) * WHEEL_CIRCUMFERENCE_MM / ENC_PULSES_PER_WHEEL_REV; }
long  mmToPulses(float mm) { return (long)(mm / WHEEL_CIRCUMFERENCE_MM * ENC_PULSES_PER_WHEEL_REV); }

// ---------------- MOTOR ----------------
void motor(int speed) {
  if (BEZ_MOTORA) speed = 0;
  digitalWrite(STBY, HIGH);
  if (speed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else            { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); speed = -speed; }
  ledcWrite(PWMA, constrain(speed, 0, 255));
}

void brake() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWMA, 255);
}

void motorOff() {
  ledcWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(STBY, LOW);
}

// ---------------- UPRAVLJANJE ----------------
void steerTo(float deg) {
  deg = constrain(deg, (float)STEER_LEFT, (float)STEER_RIGHT);
  long us = STEER_US_MIN + (long)(deg * (STEER_US_MAX - STEER_US_MIN) / 180.0);
  ledcWrite(STEER_PIN, us * ((1L << SERVO_PWM_BITS) - 1) / 20000L);
}

// ---------------- MUX + ToF ----------------
void mux(uint8_t ch) {
  Wire.beginTransmission(ADDR_MUX);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

bool initTof(Tof &t) {
  mux(t.ch);
  t.s.setTimeout(100);
  if (!t.s.init()) return false;
  t.s.setDistanceMode(VL53L1X::Short);
  t.s.setROISize(TOF_ROI_SIZE, TOF_ROI_SIZE);   // uzi stozac
  t.s.setROICenter(TOF_ROI_CENTER);
  t.s.setMeasurementTimingBudget(33000);
  t.s.startContinuous(40);
  return true;
}

int readTof(Tof &t) {                 // ne ceka
  mux(t.ch);
  t.fresh = false;
  if (t.s.dataReady()) {
    t.s.read(false);
    t.status = (uint8_t)t.s.ranging_data.range_status;
    t.last  = (t.s.ranging_data.range_status == VL53L1X::RangeValid) ? t.s.ranging_data.range_mm : -1;
    t.fresh = true;
  }
  return t.last;
}

// ---------------- MPU-6050 ----------------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void halt(const char *why);           // definirano nize
int imuErrors = 0;                    // uzastopne greske citanja MPU-a

int16_t gyroZraw() {
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((uint8_t)ADDR_MPU, (uint8_t)2) != 2) {
    imuErrors++;                       // MPU ne odgovara (npr. otkacen kabel)
    return (int16_t)gyroBias;          // "nema okretanja" umjesto smeca
  }
  imuErrors = 0;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

bool imuLost() { return imuErrors >= 5; }   // 5 gresaka zaredom = MPU izgubljen

void mpuInit() {
  mpuWrite(0x6B, 0x00);
  mpuWrite(0x1A, 0x03);
  mpuWrite(0x1B, 0x08);
  delay(100);
}

void gyroCalibrate() {
  long sum = 0;
  const int N = 500;
  for (int i = 0; i < N; i++) { sum += gyroZraw(); delay(2); }
  gyroBias = sum / (float)N;
  if (imuLost()) halt("MPU-6050 ne odgovara pri kalibraciji");
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastYawUs) * 1e-6;
  lastYawUs = now;
  yawDeg += GYRO_SIGN * (gyroZraw() - gyroBias) / 65.5 * dt;
}

void resetYaw() { yawDeg = 0; lastYawUs = micros(); }

void waitMs(unsigned long ms) {       // ceka, ali i dalje racuna kut
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { updateYaw(); delay(2); }
}

// ---------------- POMOCNO ----------------
void stopHere() {                     // kratko zaustavljanje izmedju faza
  brake();
  waitMs(300);
  ledcWrite(PWMA, 0);
}

void stopAll(const char *why) {
  brake();
  delay(300);
  motorOff();
  steerTo(STEER_CENTER);
  Serial.print("STOP: ");
  Serial.println(why);
}

void halt(const char *why) {
  motorOff();
  Serial.print("GRESKA: ");
  Serial.println(why);
  while (true) delay(1000);
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
//  VOZNJA RAVNO
//    side  = +1 prati desni zid, -1 prati lijevi zid, 0 samo drzi smjer
//    targetPulses = 0 -> stani na FRONT_STOP_MM od prednjeg zida
//    targetPulses > 0 -> stani nakon tocno toliko impulsa
//    ignoreFrontMm    = prvih toliko mm ne gleda prednji zid (samo nakon zavoja)
//    Hitni stop (EMERGENCY_MM) radi uvijek.
//  Vraca prijedjene impulse, ili -1 ako je istekao timeout.
// =====================================================================
long driveStraight(int side, long targetPulses, float ignoreFrontMm) {
  encCount = 0;
  resetYaw();
  const long ignorePulses = mmToPulses(ignoreFrontMm);
  Serial.printf("  ravnina: prednji senzor se gleda nakon %ld impulsa (%.0f mm)\n", ignorePulses, ignoreFrontMm);
  int nearCount = 0;
  unsigned long start = millis(), lastPrint = 0;
  motor(DRIVE_PWM);

  while (true) {
    if (millis() - start > STRAIGHT_TIMEOUT_MS) { stopAll("timeout na ravnini"); return -1; }
    updateYaw();
    if (imuLost()) { stopAll("MPU-6050 ne odgovara - provjeri kabel"); return -1; }
    long done = labs(encCount);

    // ---- prednji senzor: cita se UVIJEK (i za hitni stop) ----
    int f = readTof(tofF);
    if (tofF.fresh && f > 0 && f < EMERGENCY_MM) { stopAll("HITNO: zid ispred blize od 15 cm"); return -1; }

    // ---- kraj ravnine ----
    if (targetPulses > 0) {
      if (done >= targetPulses) break;
    } else if (tofF.fresh && done >= ignorePulses) {
      if (f > 0 && f <= FRONT_STOP_MM) nearCount++; else nearCount = 0;
      if (nearCount >= 2) break;        // 2 mjerenja zaredom - nije slucajni skok
    }

    // ---- upravljanje ----
    float corr, dist = -1, ang = 0;
    bool wall = false;
    if (side != 0) {
      Tof &fr = (side > 0) ? tofRF : tofLF;
      Tof &re = (side > 0) ? tofRR : tofLR;
      
      int dF = readTof(fr);
      int dR = readTof(re);
      float spacing = (side > 0) ? TOF_SPACING_R_MM : TOF_SPACING_L_MM;
      wall = (dF > 0 && dR > 0 && dF < WALL_MAX_MM && dR < WALL_MAX_MM);
      if (wall) {
        ang  = atan2f((float)(dF - dR), spacing) * 180.0 / PI;   // + = nos bjezi od zida
        dist = (dF + dR) / 2.0 * cosf(ang * PI / 180.0);
      }
    }
    if (wall) {
      float towardWall = KP_DIST * (dist - WALL_TARGET_MM) + KP_ANGLE * ang;
      corr = side * towardWall;         // desni zid: + = desno; lijevi zid: - = lijevo
    } else {
      corr = KP_HEADING * yawDeg;       // drzi smjer s pocetka ravnine
    }
    steerTo(STEER_CENTER + corr);

    if (millis() - lastPrint > 150) {
      lastPrint = millis();
      Serial.printf("  %5.0f mm | prednji %4d | zid %6.1f nagib %5.1f | yaw %6.1f | kor %5.1f%s\n",
                    pulsesToMm(done), tofF.last, dist, ang, yawDeg, corr, wall ? "" : " (smjer)");
    }
  }
  return labs(encCount);
}

// =====================================================================
//  SKENIRANJE: koja strana je slobodna?
// =====================================================================
float sideAverage(Tof &a, Tof &b) {
  long sum = 0;
  int n = 0;
  unsigned long t0 = millis();
  while (n < 10 && millis() - t0 < 1500) {
    int x = readTof(a);
    if (a.fresh) { sum += (x > 0) ? x : 4000; n++; }    // nema cilja = daleko
    int y = readTof(b);
    if (b.fresh) { sum += (y > 0) ? y : 4000; n++; }
    delay(5);
  }
  return n ? sum / (float)n : 4000;
}

int decideDirection() {
  float left  = sideAverage(tofLF, tofLR);
  float right = sideAverage(tofRF, tofRR);
  Serial.printf("Skeniranje: lijevo %.0f mm, desno %.0f mm\n", left, right);
  if (fabs(left - right) < 200) Serial.println("  UPOZORENJE: razlika mala, odluka nesigurna");
  // bliski zid = vanjski; skrecemo na daleku stranu
  return (left > right) ? +1 : -1;
}

// =====================================================================
//  SKRETANJE 90 st. (d = +1 lijevo, -1 desno)
// =====================================================================
bool turn90(int d) {
  resetYaw();
  steerTo(d > 0 ? STEER_LEFT : STEER_RIGHT);
  waitMs(200);                          // servo dolazi na kut

  unsigned long start = millis();
  motor(TURN_PWM);
  while (d * yawDeg < TURN_DEG - TURN_EARLY_DEG) {
    if (millis() - start > TURN_TIMEOUT_MS) { stopAll("timeout u skretanju"); return false; }
    updateYaw();
    if (imuLost()) { stopAll("MPU-6050 ne odgovara - provjeri kabel"); return false; }
    delay(2);
  }
  steerTo(STEER_CENTER);
  return true;
}

// =====================================================================
//  CIJELI ZADATAK
// =====================================================================
void runChallenge() {
  // 1. od starta do prvog zida (smjer jos ne znamo -> samo drzi smjer)
  Serial.println("Start -> prvi zid");
  long firstPulses = driveStraight(0, 0, 0);          // od starta: prednji senzor odmah
  if (firstPulses < 0) return;
  stopHere();

  // 2. odluka
  dir = decideDirection();
  Serial.println(dir > 0 ? "Smjer: COUNTERCLOCKWISE (lijevo, prati desni zid)"
                         : "Smjer: CLOCKWISE (desno, prati lijevi zid)");
  const int side = dir;                 // CCW -> desni zid (+1), CW -> lijevi zid (-1)

  // 3. krugovi
  const int totalTurns = LAPS * TURNS_PER_LAP;
  long startSectionSum = 0;
  int  startSectionN   = 0;

  for (int t = 1; t <= totalTurns; t++) {
    Serial.printf("Skretanje %d / %d (krug %d)\n", t, totalTurns, (t - 1) / TURNS_PER_LAP + 1);
    if (!turn90(dir)) return;
    if (t == totalTurns) break;         // nakon zadnjeg skretanja -> zavrsna ravnina

    long p = driveStraight(side, 0, MIN_STRAIGHT_MM);
    if (p < 0) return;
    if (t % TURNS_PER_LAP == 0) {       // ova ravnina je startna sekcija
      startSectionSum += p;
      startSectionN++;
    }
    if (STOP_AT_CORNERS) stopHere();
  }

  // 4. zavrsna ravnina do startnog mjesta
  //    startna sekcija cijela = (izlaz iz zavoja -> start) + (start -> stop pred zidom)
  //    => od izlaza iz zavoja do starta = cijela - firstPulses
  long finalPulses = 0;
  if (startSectionN > 0) finalPulses = startSectionSum / startSectionN - firstPulses;
  Serial.printf("Zavrsna ravnina: %.0f mm\n", pulsesToMm(finalPulses));
  if (finalPulses > 0) {
    if (driveStraight(side, finalPulses, 0) < 0) return;
  }
  stopAll("3 kruga gotova");
}

// =====================================================================
//  ToF TEST  ('t' u Serial Monitoru, bilo koja tipka ga zaustavlja)
//    Uzivo ispisuje svih 5 senzora + udaljenost i nagib zida za svaku stranu.
//    Na kraju daje statistiku: frekvenciju, % neispravnih, min/prosjek/max.
//    Motor se ne pokrece. Drzi robota ispred ravnog zida i pomicji ga rukom.
// =====================================================================
void printWall(int dF, int dR, float spacing) {
  if (dF > 0 && dR > 0) {
    float ang  = atan2f((float)(dF - dR), spacing) * 180.0 / PI;
    float dist = (dF + dR) / 2.0 * cosf(ang * PI / 180.0);
    Serial.printf(" %6.0f %6.1f", dist, ang);
  } else {
    Serial.print("      -      -");
  }
}

void testTof() {
  Tof *all[5] = {&tofRF, &tofRR, &tofLF, &tofLR, &tofF};
  const char *name[5] = {"RF", "RR", "LF", "LR", "F "};
  long sum[5] = {0, 0, 0, 0, 0};
  long ok[5]  = {0, 0, 0, 0, 0};
  long bad[5] = {0, 0, 0, 0, 0};
  int  mn[5], mx[5];
  for (int i = 0; i < 5; i++) { mn[i] = 32767; mx[i] = 0; }

  motorOff();
  steerTo(STEER_CENTER);
  while (Serial.available()) Serial.read();
  Serial.println("\n=== ToF test (bilo koja tipka = kraj) ===");
  Serial.println("    RF    RR    LF    LR     F |  desno: zid nagib |  lijevo: zid nagib | status RF RR LF LR F");

  unsigned long t0 = millis(), lastPrint = 0;
  while (!Serial.available()) {
    for (int i = 0; i < 5; i++) {
      int d = readTof(*all[i]);
      if (!all[i]->fresh) continue;
      if (d > 0) {
        sum[i] += d;
        ok[i]++;
        if (d < mn[i]) mn[i] = d;
        if (d > mx[i]) mx[i] = d;
      } else {
        bad[i]++;
      }
    }

    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      Serial.printf("%6d%6d%6d%6d%6d |", tofRF.last, tofRR.last, tofLF.last, tofLR.last, tofF.last);
      printWall(tofRF.last, tofRR.last, TOF_SPACING_R_MM);
      Serial.print(" |");
      printWall(tofLF.last, tofLR.last, TOF_SPACING_L_MM);
      Serial.printf(" | st %u %u %u %u %u", tofRF.status, tofRR.status, tofLF.status, tofLR.status, tofF.status);
      Serial.println();
    }
    delay(2);
  }
  while (Serial.available()) Serial.read();

  float sec = (millis() - t0) / 1000.0;
  Serial.printf("\n--- statistika (%.1f s) ---\n", sec);
  Serial.println("sen    Hz  neispr%   min  prosjek   max  raspon");
  for (int i = 0; i < 5; i++) {
    long tot = ok[i] + bad[i];
    if (tot == 0) { Serial.printf("%s   NEMA PODATAKA\n", name[i]); continue; }
    Serial.printf("%s %6.1f %7.0f %5d %8.0f %5d %7d\n", name[i],
                  tot / sec,
                  100.0 * bad[i] / tot,
                  ok[i] ? mn[i] : -1,
                  ok[i] ? sum[i] / (float)ok[i] : -1.0f,
                  ok[i] ? mx[i] : -1,
                  ok[i] ? mx[i] - mn[i] : 0);
  }
  Serial.println("(ocekivano ~25 Hz; raspon = sum, manje je bolje)");
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== open_challenge ===");

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  ledcAttach(PWMA, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  motorOff();

  ledcAttach(STEER_PIN, SERVO_PWM_FREQ, SERVO_PWM_BITS);
  steerTo(STEER_CENTER);

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, RISING);

  pinMode(START_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  if (chaosI2CCheck(false) > 0) halt("provjera I2C nije prosla - vidi popis gore, NE VOZITI");
  mpuInit();
  if (!initTof(tofRF)) halt("desni prednji ToF (kanal 0)");
  if (!initTof(tofRR)) halt("desni zadnji ToF (kanal 1)");
  if (!initTof(tofLF)) halt("lijevi prednji ToF (kanal 2)");
  if (!initTof(tofF))  halt("prednji ToF (kanal 3)");
  if (!initTof(tofLR)) halt("lijevi zadnji ToF (kanal 4)");

  if (BEZ_MOTORA) Serial.println("*** BEZ_MOTORA = true: motor nece voziti ***");
  Serial.println("Spreman. START ili 's', 't' = ToF test.");
}

void loop() {
  int c = Serial.peek();
  if (c == 't' || c == 'T') {           // ToF test, bez voznje
    Serial.read();
    testTof();
    Serial.println("Spreman. START ili 's', 't' = ToF test.");
    return;
  }
  if (!startPressed()) return;

  Serial.println("Ne diraj robota - kalibracija ziroskopa...");
  delay(1000);
  gyroCalibrate();

  unsigned long t0 = millis();
  runChallenge();
  Serial.printf("Vrijeme: %.1f s\n", (millis() - t0) / 1000.0);

  Serial.println("Za ponovno pokretanje: START ili 's'.");
  while (Serial.available()) Serial.read();
  delay(500);
}