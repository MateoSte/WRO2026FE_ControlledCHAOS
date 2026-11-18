// =====================================================================
//  voznja_desni_zid.ino  -  Controlled C.H.A.O.S. v2
//
//  Zadatak:
//    1. vozi naprijed 50 cm i drzi 20 cm od DESNOG zida
//       (desni prednji ToF = MUX kanal 0, desni zadnji ToF = MUX kanal 1)
//    2. skreni ULIJEVO za 90 stupnjeva (ziroskop MPU-6050)
//    3. stani
//
//  Potrebno:
//    - ChaosPins.h u istoj mapi
//    - biblioteka "VL53L1X" by Pololu (Library Manager)
//    - ESP32 core 3.x (ledcAttach / ledcWrite po pinu)
//
//  Pokretanje: START tipkalo ili 's' u Serial Monitoru (115200).
//  Nakon pokretanja robot 1 s miruje (kalibracija ziroskopa) - NE DIRATI.
// =====================================================================
#include <Wire.h>
#include <VL53L1X.h>
#include "../includes/includes.ino"

// ---------------- POSTAVKE ZADATKA ----------------
const float DRIVE_DIST_MM   = 1000.0;  // 50 cm
const float WALL_TARGET_MM  = 200.0;  // 20 cm od zida (od lica senzora)
const float TURN_DEG        = 90.0;   // skretanje ulijevo

const int   DRIVE_PWM       = 110;    // brzina voznje (0-255) - iznad mrtve zone!
const int   TURN_PWM        = 100;    // brzina u skretanju

// ---------------- GEOMETRIJA (IZMJERITI!) ----------------
const float TOF_SPACING_MM  = 117.0;  // TODO: razmak desni prednji <-> desni zadnji ToF, po duljini robota

// ---------------- REGULACIJA ----------------
//  korekcija > 0  = skreni DESNO (prema zidu)
//  korekcija < 0  = skreni LIJEVO (od zida)
const float KP_DIST         = 0.08;   // st. upravljanja po mm greske udaljenosti (100 mm -> 8 st.)
const float KP_ANGLE        = 0.8;    // st. upravljanja po st. nagiba prema zidu
const float KP_HEADING      = 1.0;    // kad nema zida: drzi smjer ziroskopom
const int   WALL_MAX_MM     = 1000;   // dalje od ovoga = "nema zida"

// ---------------- SKRETANJE ----------------
const float TURN_EARLY_DEG  = 6.0;    // stani malo prije 90 st. zbog inercije (namjestiti)
const int   GYRO_SIGN       = 1;      // ako okret ULIJEVO daje NEGATIVAN kut -> stavi -1

// ---------------- SIGURNOST ----------------
const unsigned long DRIVE_TIMEOUT_MS = 8000;
const unsigned long TURN_TIMEOUT_MS  = 5000;
const bool  BEZ_MOTORA      = false;  // true = motor ne vozi, samo upravljanje i ispis (suho testiranje)

// ---------------- MG995 mapiranje kuta u impuls ----------------
// MORA biti isto kao u servo_center.ino, inace 73/55/90 ne vrijede!
const int   STEER_US_MIN    = 500;    // 0 st.
const int   STEER_US_MAX    = 2500;   // 180 st.

// =====================================================================
VL53L1X tofRF, tofRR;                 // desni prednji, desni zadnji
int lastRF = -1, lastRR = -1;         // zadnje valjano ocitanje (mm), -1 = nema

volatile long encCount = 0;

float gyroBias = 0;
float yawDeg   = 0;                   // + = okret ulijevo
unsigned long lastYawUs = 0;

// ---------------- ENKODER ----------------
void IRAM_ATTR encISR() {
  if (digitalRead(ENC_B)) encCount++; else encCount--;
}

// ---------------- MOTOR ----------------
void motor(int speed) {               // -255 .. 255
  if (BEZ_MOTORA) speed = 0;
  digitalWrite(STBY, HIGH);
  if (speed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else            { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); speed = -speed; }
  ledcWrite(PWMA, constrain(speed, 0, 255));
}

void brake() {                        // kratki spoj motora = kocnica
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWMA, 255);
}

void motorOff() {                     // driver u standby
  ledcWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(STBY, LOW);
}

// ---------------- UPRAVLJANJE ----------------
void steerTo(float deg) {
  deg = constrain(deg, (float)STEER_LEFT, (float)STEER_RIGHT);   // nikad izvan granica
  long us = STEER_US_MIN + (long)(deg * (STEER_US_MAX - STEER_US_MIN) / 180.0);
  ledcWrite(STEER_PIN, us * ((1L << SERVO_PWM_BITS) - 1) / 20000L);
}

// ---------------- MUX ----------------
void mux(uint8_t ch) {
  Wire.beginTransmission(ADDR_MUX);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

// ---------------- ToF ----------------
bool initTof(VL53L1X &s, uint8_t ch) {
  mux(ch);
  s.setTimeout(100);
  if (!s.init()) return false;
  s.setDistanceMode(VL53L1X::Short);        // do ~1.3 m, najotporniji na svjetlo
  s.setMeasurementTimingBudget(33000);      // 33 ms
  s.startContinuous(40);                    // novo mjerenje svakih 40 ms
  return true;
}

// Ne ceka: ako je novo mjerenje spremno, procita ga; inace vraca zadnje.
int readTof(VL53L1X &s, uint8_t ch, int &last) {
  mux(ch);
  if (s.dataReady()) {
    s.read(false);
    if (s.ranging_data.range_status == VL53L1X::RangeValid) last = s.ranging_data.range_mm;
    else last = -1;
  }
  return last;
}

// ---------------- MPU-6050 (bez biblioteke) ----------------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t gyroZraw() {
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(0x47);                         // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADDR_MPU, (uint8_t)2);
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

void mpuInit() {
  mpuWrite(0x6B, 0x00);                     // probudi
  mpuWrite(0x1A, 0x03);                     // filtar ~44 Hz
  mpuWrite(0x1B, 0x08);                     // +-500 st/s -> 65.5 po st/s
  delay(100);
}

void gyroCalibrate() {                      // robot MORA mirovati
  long sum = 0;
  const int N = 500;
  for (int i = 0; i < N; i++) { sum += gyroZraw(); delay(2); }
  gyroBias = sum / (float)N;
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastYawUs) * 1e-6;
  lastYawUs = now;
  yawDeg += GYRO_SIGN * (gyroZraw() - gyroBias) / 65.5 * dt;
}

void resetYaw() {
  yawDeg = 0;
  lastYawUs = micros();
}

// ---------------- POMOCNO ----------------
void stopAll(const char *why) {
  brake();
  delay(300);
  motorOff();
  steerTo(STEER_CENTER);
  Serial.print("STOP: ");
  Serial.println(why);
}

void halt(const char *why) {                // greska pri pokretanju - nista ne vozi
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
//  FAZA 1: naprijed 50 cm uz desni zid
// =====================================================================
bool driveFollowRight() {
  const long target = (long)(DRIVE_DIST_MM / WHEEL_CIRCUMFERENCE_MM * ENC_PULSES_PER_WHEEL_REV);
  Serial.print("Faza 1: cilj ");
  Serial.print(target);
  Serial.println(" impulsa");

  encCount = 0;
  resetYaw();
  unsigned long start = millis(), lastPrint = 0;
  motor(DRIVE_PWM);

  while (labs(encCount) < target) {          // labs: radi i ako je smjer enkodera obrnut
    if (millis() - start > DRIVE_TIMEOUT_MS) { stopAll("timeout u fazi 1"); return false; }

    updateYaw();
    int dF = readTof(tofRF, MUX_TOF_RIGHT_FRONT, lastRF);
    int dR = readTof(tofRR, MUX_TOF_RIGHT_REAR,  lastRR);

    float corr, dist = -1, angDeg = 0;
    bool wall = (dF > 0 && dR > 0 && dF < WALL_MAX_MM && dR < WALL_MAX_MM);

    if (wall) {
      // nagib: prednji dalje od zadnjeg = nos bjezi od zida (ulijevo)
      angDeg = atan2f((float)(dF - dR), TOF_SPACING_MM) * 180.0 / PI;
      dist   = (dF + dR) / 2.0 * cosf(angDeg * PI / 180.0);   // okomita udaljenost
      corr   = KP_DIST * (dist - WALL_TARGET_MM) + KP_ANGLE * angDeg;
    } else {
      corr   = KP_HEADING * yawDeg;          // nema zida: zadrzi pocetni smjer
    }
    steerTo(STEER_CENTER + corr);

    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.printf("enc %6ld | F %4d R %4d | dist %6.1f nagib %5.1f | yaw %6.1f | korekcija %5.1f %s\n",
                    encCount, dF, dR, dist, angDeg, yawDeg, corr, wall ? "" : "(nema zida)");
    }
  }
  return true;
}

// =====================================================================
//  FAZA 2: ulijevo 90 stupnjeva
// =====================================================================
bool turnLeft90() {
  Serial.println("Faza 2: skretanje ulijevo");
  resetYaw();
  steerTo(STEER_LEFT);

  unsigned long t0 = millis();               // pricekaj da servo dodje (bez delay - yaw se racuna)
  while (millis() - t0 < 200) updateYaw();

  unsigned long start = millis(), lastPrint = 0;
  motor(TURN_PWM);
  while (yawDeg < TURN_DEG - TURN_EARLY_DEG) {
    if (millis() - start > TURN_TIMEOUT_MS) { stopAll("timeout u fazi 2"); return false; }
    updateYaw();
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.printf("yaw %6.1f\n", yawDeg);
    }
    delay(2);
  }
  return true;
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== voznja_desni_zid ===");

  // motor
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  ledcAttach(PWMA, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  motorOff();

  // upravljanje
  ledcAttach(STEER_PIN, SERVO_PWM_FREQ, SERVO_PWM_BITS);
  steerTo(STEER_CENTER);

  // enkoder
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, RISING);

  // tipkalo
  pinMode(START_PIN, INPUT_PULLUP);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  mpuInit();
  if (!initTof(tofRF, MUX_TOF_RIGHT_FRONT)) halt("desni prednji ToF (kanal 0) se ne javlja");
  if (!initTof(tofRR, MUX_TOF_RIGHT_REAR))  halt("desni zadnji ToF (kanal 1) se ne javlja");

  if (BEZ_MOTORA) Serial.println("*** BEZ_MOTORA = true: motor nece voziti ***");
  Serial.println("Postavi robota ~20 cm od desnog zida. START ili 's' za pokretanje.");
}

void loop() {
  if (!startPressed()) return;

  Serial.println("Ne diraj robota - kalibracija ziroskopa...");
  delay(1000);                               // makni ruku
  gyroCalibrate();

  if (driveFollowRight() && turnLeft90()) {
    stopAll("zadatak gotov");
    // jos 0.5 s mjeri kut da se vidi koliko je robot "proklizao" nakon kocenja
    unsigned long t0 = millis();
    while (millis() - t0 < 500) { updateYaw(); delay(2); }
    Serial.printf("Konacni kut: %.1f st. (cilj %.0f) -> namjesti TURN_EARLY_DEG\n", yawDeg, TURN_DEG);
  }

  Serial.println("Za ponovno pokretanje: START ili 's'.");
  while (Serial.available()) Serial.read();  // isprazni stare naredbe
  delay(500);
}
