// =====================================================================
//  obstacle_run.ino  -  Controlled C.H.A.O.S. v2  -  WRO 2026 FE Obstacle Challenge
//
//  Tijek (START tipkalo ili 's' u Serial Monitoru, 115200):
//    1. Kalibracija ziroskopa (robot miruje u parkingu).
//    2. SMJER iz bocnih ToF-ova: blizi DESNI zid -> CCW, blizi LIJEVI -> CW.
//    3. SKEN: toranj na 3 kuta (CCW 170/215/260 st., CW 100/55/10 st.,
//       naprijed = 135). Na svakom kutu ESP32 javi kameri "snimi kut i",
//       kamera odgovori "gotovo" i tek onda se toranj okrece dalje.
//       Na kraju kamera posalje mapu 12 linija; ispis npr.
//           kamera (sirovo):  -RG-R-------
//           mapa (pravila):   -G--R-------
//    4. PLAN: iz mape traka IN ili OUT za svaku sekciju (obstacle_plan.h).
//    5. Izlazak iz parkinga (potezi iz tests/parking/parking.ino), prijelaz
//       preko staze u prvu traku, 3 kruga uz zid na LANE_WALL_MM, zaokreti
//       od 90 st. u zavojima i kod promjene trake (oko linije L1).
//    6. Stop na mjestu starta (TODO: parkiranje).
//
//  Ostale naredbe (Serial Monitor, "Newline"):
//    k             samo smjer + sken + mapa + plan (robot se ne mice)
//    m -R--G-R-G--R  rucno upisi mapu (12 znakova R/G/-/?) za testiranje voznje
//    r             voznja s rucno upisanom mapom (bez kamere)
//    x             samo izlazak iz parkinga (provjera smjera poteza)
//    q             jedan zaokret 90 st. ulijevo - za mjerenje TURN_FWD_MM / TURN_SIDE_MM
//
//  Potrebno: includes/includes.ino, includes/ChaosCheck.h, obstacle_plan.h (ista mapa),
//            biblioteka "VL53L1X" by Pololu; na kameri ae3_mapa_i2c.py (kao main.py).
// =====================================================================

#include <Wire.h>
#include <WiFi.h>
#include <VL53L1X.h>
#include "../../includes/includes.ino"
#include "../../includes/ChaosCheck.h"
#include "obstacle_plan.h"

// ---------------- STAZA I ROBOT (mm) - vidi obstacle_plan.h ----------------
route::Geo G;                         // vrijednosti se postavljaju u setup() iz konstanti ispod
const float LANE_WALL_MM      = 150.0;  // traka: lice bocnog ToF-a -> zid
const float PARK_LANE_WALL_MM = 300.0;  // traka OUT u startnoj sekciji (parking je dubok 200)
const float SIDE_TO_CENTER_MM = 70.0;   // TODO izmjeriti: lice bocnog ToF-a -> sredina robota
const float FRONT_TO_CENTER_MM= 80.0;   // TODO izmjeriti: lice prednjeg ToF-a -> sredina robota
const float TURN_FWD_MM       = 147.0;  // TODO izmjeriti naredbom 'q' (teorija za R = 200: 200 - 53)
const float TURN_SIDE_MM      = 253.0;  // TODO izmjeriti naredbom 'q' (teorija za R = 200: 200 + 53)

// Polozaj sredine robota u parkingu (x od linije L0 startne sekcije u smjeru voznje,
// w od unutarnjeg zida). Parking je uz liniju L2 kad je smjer CCW, uz L0 kad je CW.
const float START_X_MM[2]  = {860.0, 140.0};   // [CCW, CW]  TODO provjeriti na stazi
const float START_W_MM     = 900.0;            // 1000 - 30 (razmak do zida) - 70 (pola sirine)
const float STOP_X_MM[2]   = {860.0, 140.0};   // nakon 3 kruga stani ovdje (TODO: parkiranje)
// Linije startne sekcije koje vrijede na POCETKU i na KRAJU (vidi obstacle_plan.h)
const int   START_FIRST_LINE[2]  = {2, 1};     // CCW: ispred robota samo L2; CW: L1 i L2
const int   FINISH_LAST_LINE[2]  = {1, 0};     // CCW: do stopa L0 i L1; CW: samo L0
const int   START_BLOCK_LINE[2]  = {-1, 0};    // CW: stup na L0 smeta prijelazu u IN odmah iza parkinga

// ---------------- TORANJ I KAMERA ----------------
const int   TOWER_FWD_DEG   = 135;             // kamera gleda naprijed (vidi napomenu o TOWER_CENTER)
const int   SCAN_ANGLES[3]  = {35, 80, 125};   // od "naprijed": CCW ulijevo (+), CW udesno (-)
const float TOWER_TOL_DEG   = 4.0;             // povratna veza: dosta blizu
const unsigned long TOWER_TIMEOUT_MS = 1500;   // ako povratna veza ne potvrdi (npr. ispod 90 st.)
const unsigned long TOWER_SETTLE_MS  = 200;    // nakon dolaska, da se toranj smiri
const unsigned long CAM_HELLO_MS     = 15000;  // koliko cekati da se kamera javi (pokretanje ~3 s)
const unsigned long CAM_VIEW_MS      = 8000;   // najdulje jedan kut (snimanje + mjerenje)

// Registri kamere (isti kao u ae3_mapa_i2c.py)
const uint8_t CR_MAGIC = 0, CR_STATUS = 2, CR_ACK = 3, CR_CMD = 16, CR_RAW = 32, CR_MAP = 48;
const uint8_t CAM_MAGIC = 0xC5;
const uint8_t CST_VIEW_DONE = 2, CST_MAP_DONE = 3, CST_ERROR = 4;
const uint8_t CCMD_SNAP = 1, CCMD_RESET = 2, CCMD_COMPOSE = 3;

// ---------------- VOZNJA ----------------
const int   DRIVE_PWM       = 150;     // uz zid (sporije nego open_challenge: preciznije okidanje zavoja)
const int   TURN_PWM        = 100;
const int   ACROSS_PWM      = 90;      // preko staze (prema prednjem zidu)
const int   EXIT_PWM        = 100;     // izlazak iz parkinga
const float KP_DIST         = 0.08;    // kao open_challenge
const float KP_ANGLE        = 0.8;
const float KP_HEADING      = 1.0;
const int   WALL_MAX_MM     = 700;     // dalje = nema zida, drzi smjer
const int   WALL_REJECT_MM  = 120;     // ocitanje blize od (cilj - 120) nije zid (magenta element, stup)
const float CORNER_ARM_X_MM = 700.0;   // zavoj se smije okinuti tek od ovog x
const int   ACROSS_TOL_MM   = 15;      // preko staze: dovoljno blizu cilja
const float TURN_EARLY_DEG  = 6.0;     // kao open_challenge
const int   EMERGENCY_MM    = 150;     // zid ispred blize = HITNI STOP
const int   ENC_SIGN        = -1;       // TODO: +1 ako encCount RASTE u voznji naprijed (naredba 'q' ispisuje)
const float TOF_SPACING_R_MM = 120.0;  // TODO kao u open_challenge
const float TOF_SPACING_L_MM = 120.0;
const unsigned long STRAIGHT_TIMEOUT_MS = 10000;
const unsigned long TURN_TIMEOUT_MS     = 5000;
const bool  BEZ_MOTORA      = false;   // true = sve osim pogona (sken, plan, ispis)

// Izlazak iz parkinga = potezi iz tests/parking/parking.ino (setup), zapisani u
// STVARNOM smjeru voznje (+ naprijed). Tamo driveMm() vozi preko klase Motor
// s Direction::COUNTERCLOCKWISE, pa je driveMm(+20) fizicki UNATRAG u odnosu
// na motor(+) iz open_challenge. Uz to tumacenje niz okrece robota PREMA stazi
// i zavrsava okomito na pocetni smjer (lice prema unutarnjem zidu) - u
// parking.ino zapisan je za CW (staza desno). Ovdje se zrcali po smjeru.
//   toward = +1: kotaci prema stazi, -1: od staze.   Provjera: naredba 'x'.
struct ExitLeg { int8_t toward; int16_t mm; };
const ExitLeg EXIT_LEGS[] = {{-1, -20}, {+1, 20}, {-1, -20}, {+1, 30},
                             {-1, -20}, {+1, 20}, {-1, -20}, {+1, 200}};

// ---------------- MG995 (isto kao open_challenge) ----------------
const int   STEER_US_MIN    = 500;
const int   STEER_US_MAX    = 2500;

// =====================================================================
//  HARDVER (kopija iz open_challenge.ino)
// =====================================================================
struct Tof {
  VL53L1X s;
  uint8_t ch;
  int     last;    // zadnje valjano ocitanje (mm), -1 = nema cilja
  bool    fresh;   // upravo stiglo novo mjerenje
  uint8_t status;  // zadnji range_status (0 = valjano)
};
Tof tofRF = {VL53L1X(), MUX_TOF_RIGHT_FRONT, -1, false, 255};
Tof tofRR = {VL53L1X(), MUX_TOF_RIGHT_REAR,  -1, false, 255};
Tof tofLF = {VL53L1X(), MUX_TOF_LEFT_FRONT,  -1, false, 255};
Tof tofLR = {VL53L1X(), MUX_TOF_LEFT_REAR,   -1, false, 255};
Tof tofF  = {VL53L1X(), MUX_TOF_FRONT,       -1, false, 255};

volatile long encCount = 0;
float gyroBias = 0;
float yawDeg   = 0;                   // + = okret ulijevo
unsigned long lastYawUs = 0;
int imuErrors = 0;

void IRAM_ATTR encISR() { if (digitalRead(ENC_B)) encCount++; else encCount--; }

void motor(int speed) {               // + naprijed, - natrag
  if (BEZ_MOTORA) speed = 0;
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
  t.s.startContinuous(budgetUs / 1000 + 7);
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
  Serial.print("GRESKA: "); Serial.println(why);
  while (true) delay(1000);
}

void gyroCalibrate() {
  long sum = 0;
  for (int i = 0; i < 500; i++) { sum += gyroZraw(); delay(2); }
  gyroBias = sum / 500.0;
  if (imuLost()) halt("MPU-6050 ne odgovara pri kalibraciji");
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastYawUs) * 1e-6;
  lastYawUs = now;
  yawDeg += (gyroZraw() - gyroBias) / 65.5 * dt;
}

bool startPressed() {
  if (digitalRead(START_PIN) == LOW) return true;
  if (Serial.peek() == 's' || Serial.peek() == 'S') { Serial.read(); return true; }
  return false;
}

// =====================================================================
//  ODOMETRIJA u okviru sekcije: x = uzduz sekcije od linije L0 (u smjeru
//  voznje), w = udaljenost SREDINE robota od unutarnjeg zida.
//  Enkoder (put) + ziroskop (smjer); w se ispravlja ocitanjem zida.
// =====================================================================
int   dir    = 0;                     // +1 = CCW, -1 = CW
float course = 0;                     // smjer trenutne sekcije (st., isti okvir kao yawDeg)
float odoX = 0, odoW = 0;
long  lastEncOdo = 0;

float mmPerPulse() { return WHEEL_CIRCUMFERENCE_MM / ENC_PULSES_PER_WHEEL_REV; }

void tick() {                         // zvati u svakoj petlji voznje
  updateYaw();
  long e = encCount;
  float ds = ENC_SIGN * (e - lastEncOdo) * mmPerPulse();
  lastEncOdo = e;
  float psi = (yawDeg - course) * DEG_TO_RAD;          // + = lijevo od smjera sekcije
  odoX += ds * cosf(psi);
  odoW -= dir * ds * sinf(psi);                        // CCW: lijevo = prema unutarnjem zidu
}

void waitTick(unsigned long ms) { unsigned long t0 = millis(); while (millis() - t0 < ms) { tick(); delay(2); } }

void stopHere() { brake(); waitTick(250); ledcWrite(PWMA, 0); }

bool fail(const char *why) {
  brake(); delay(300); motorOff();
  steerTo(STEER_CENTER);
  Serial.print("STOP: "); Serial.println(why);
  return false;
}

// =====================================================================
//  TORANJ KAMERE (SER0046 270 st. + analogna povratna veza)
//  NAPOMENA: includes.ino definira TOWER_CENTER dvaput (135 pa 81) - vrijedi
//  zadnja (81). Ovdje se zato koristi TOWER_FWD_DEG = 135, a povratna veza se
//  racuna oko 135 st. jer je TOWER_MV_CENTER izmjeren na 135 st.
// =====================================================================
void towerWriteDeg(int deg) {
  deg = constrain(deg, 0, TOWER_MAX_DEG);
  long us = TOWER_MIN_US + (long)deg * (TOWER_MAX_US - TOWER_MIN_US) / TOWER_MAX_DEG;
  ledcWrite(TOWER_PIN, us * ((1L << SERVO_PWM_BITS) - 1) / 20000L);
}

float towerFeedbackDeg() {
  float mv = analogReadMilliVolts(FEEDBACK_PIN);
  return TOWER_FWD_DEG + (mv - TOWER_MV_CENTER) / TOWER_MV_PER_DEG;
}

// Okreni toranj i cekaj da povratna veza potvrdi kut (3 ocitanja zaredom).
// Povratna veza je izmjerena samo za 90-270 st.; ispod toga vrijedi timeout.
bool towerMoveTo(int deg) {
  towerWriteDeg(deg);
  unsigned long t0 = millis();
  int ok = 0;
  float fb = 0;
  while (millis() - t0 < TOWER_TIMEOUT_MS) {
    fb = towerFeedbackDeg();
    if (fabsf(fb - deg) <= TOWER_TOL_DEG) { if (++ok >= 3) break; } else ok = 0;
    delay(20);
  }
  bool confirmed = ok >= 3;
  delay(TOWER_SETTLE_MS);
  Serial.printf("  toranj %3d st.: povratna veza %.0f st. (%s, %lu ms)\n", deg, fb,
                confirmed ? "potvrdjeno" : (deg < 90 ? "izvan kalibracije, timeout" : "NIJE potvrdjeno"), millis() - t0);
  return true;                        // bez potvrde se ipak nastavlja (kamera javlja sto vidi)
}

// =====================================================================
//  KAMERA (OpenMV AE3, I2C 0x12 na kanalu MUX_CAMERA) - protokol u ae3_mapa_i2c.py
// =====================================================================
uint8_t camSeq = 0;

bool camWrite(uint8_t reg, const uint8_t *data, uint8_t n) {
  mux(MUX_CAMERA);
  Wire.beginTransmission(ADDR_AE3);
  Wire.write(reg);
  Wire.write(data, n);
  return Wire.endTransmission() == 0;
}

bool camRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  mux(MUX_CAMERA);
  Wire.beginTransmission(ADDR_AE3);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // ponovljeni start, kao kod MPU-a
  if (Wire.requestFrom((uint8_t)ADDR_AE3, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool camHello() {                     // ceka da skripta na kameri radi; preuzima zadnji ACK
  unsigned long t0 = millis();
  uint8_t b[4];
  while (millis() - t0 < CAM_HELLO_MS) {
    if (camRead(CR_MAGIC, b, 4) && b[0] == CAM_MAGIC) {
      camSeq = b[CR_ACK];
      Serial.printf("Kamera spremna (protokol v%d, %lu ms)\n", b[1], millis() - t0);
      return true;
    }
    delay(100);
  }
  return false;
}

// Posalje naredbu (smjer, kut i naredba u ISTOJ poruci, SEQ zadnji) i ceka ACK.
// Vraca STATUS kamere ili -1 kod timeouta.
int camCommand(uint8_t kut, uint8_t cmd, unsigned long timeoutMs) {
  camSeq = (camSeq % 255) + 1;        // 1..255, nikad 0
  uint8_t d[4] = {(uint8_t)(dir > 0 ? 'C' : 'W'), kut, cmd, camSeq};
  if (!camWrite(CR_CMD, d, 4)) { Serial.println("  kamera: pisanje nije uspjelo"); return -1; }
  unsigned long t0 = millis();
  uint8_t b[4];
  while (millis() - t0 < timeoutMs) {
    delay(20);
    if (camRead(CR_MAGIC, b, 4) && b[0] == CAM_MAGIC && b[CR_ACK] == camSeq) return b[CR_STATUS];
  }
  return -1;
}

// Cijeli sken: 3 kuta, pa mapa. map/raw = 12 znakova + '\0'.
bool scanCourse(char *outMap, char *raw) {
  Serial.println("\n--- SKEN STAZE ---");
  if (!camHello()) return fail("kamera se ne javlja (main.py na kameri? kabel na kanalu 5?)");
  camCommand(0, CCMD_RESET, 1000);
  int status = -1;
  for (int k = 0; k < 3; k++) {
    int deg = TOWER_FWD_DEG + dir * SCAN_ANGLES[k];   // CCW 170/215/260, CW 100/55/10
    towerMoveTo(deg);
    unsigned long t0 = millis();
    status = camCommand(k, CCMD_SNAP, CAM_VIEW_MS);
    Serial.printf("  kut %d (%d st.): kamera status %d za %lu ms\n", k, deg, status, millis() - t0);
    if (status < 0)          { towerWriteDeg(TOWER_FWD_DEG); return fail("kamera nije odgovorila na kut"); }
    if (status == CST_ERROR) { towerWriteDeg(TOWER_FWD_DEG); return fail("kamera javlja gresku (vidi Serial Terminal u OpenMV IDE)"); }
  }
  towerWriteDeg(TOWER_FWD_DEG);       // natrag naprijed dok se cita mapa
  if (status != CST_MAP_DONE) status = camCommand(0, CCMD_COMPOSE, 2000);
  if (status != CST_MAP_DONE) return fail("kamera nije slozila mapu");
  uint8_t r[route::LINES], m[route::LINES];
  if (!camRead(CR_RAW, r, route::LINES) || !camRead(CR_MAP, m, route::LINES)) return fail("citanje mape nije uspjelo");
  for (int i = 0; i < route::LINES; i++) { raw[i] = (char)r[i]; outMap[i] = (char)m[i]; }
  raw[route::LINES] = outMap[route::LINES] = 0;
  return true;
}

// =====================================================================
//  ISPIS MAPE I PLANA
// =====================================================================
void printGrouped(const char *m) {
  for (int s = 0; s < 4; s++) Serial.printf("%c%c%c ", m[3 * s], m[3 * s + 1], m[3 * s + 2]);
}

void printMapAndPlan(const char *raw, const char *m, const route::Plan &p) {
  Serial.printf("\n=== MAPA PREPREKA  %s  (linija 0 = prva linija startne sekcije u smjeru voznje) ===\n",
                dir > 0 ? "CCW" : "CW");
  Serial.println("linija:           012345678901");
  if (raw) Serial.printf("kamera (sirovo):  %s\n", raw);
  Serial.printf("mapa (pravila):   %s\n", m);
  Serial.print  ("po sekcijama:     "); printGrouped(m); Serial.println();
  for (int s = 0; s < 4; s++)
    if (!route::tripletOk(m, s))   // allowed: "X--", "-X-", "--X", "X-Y" (rules, Figure 8c)
      Serial.printf("UPOZORENJE S%d: '%c%c%c' nije \"X--\", \"-X-\", \"--X\" ni \"X-Y\"\n", s, m[3 * s], m[3 * s + 1], m[3 * s + 2]);
  if (p.unknown) Serial.printf("UPOZORENJE: %d linija s '?' - za njih robot ne zna boju (zadrzava traku)\n", p.unknown);
  char buf[320];
  route::toText(p, buf, sizeof buf);
  Serial.printf("PLAN: %s\n", buf);
  Serial.println("      (> start, | kraj, * traka preuzeta iz sljedece sekcije, ! start prvo OUT)");
}

// =====================================================================
//  SMJER: blizi desni zid -> CCW, blizi lijevi -> CW
// =====================================================================
// U parkingu je bok robota ~3 cm od vanjskog zida - to je ispod najmanjeg
// dometa VL53L1X (~4 cm), pa senzor tada NE javlja RangeValid nego
// "MinRangeClipped" / "MinRangeFail". Takvo ocitanje znaci BLIZU (0 mm),
// a ne "nema cilja" - inace bi robot zakljucio da je zid daleko i krivo odabrao smjer.
int nearAwareMm(const Tof &t) {
  if (t.last > 0) return t.last;
  if (t.status == VL53L1X::RangeValidMinRangeClipped || t.status == VL53L1X::MinRangeFail) return 0;
  return 4000;                                           // nema cilja = daleko
}

float sideAverage(Tof &a, Tof &b) {
  long sum = 0;
  int n = 0;
  unsigned long t0 = millis();
  while (n < 10 && millis() - t0 < 1500) {
    readTof(a);
    if (a.fresh) { sum += nearAwareMm(a); n++; }
    readTof(b);
    if (b.fresh) { sum += nearAwareMm(b); n++; }
    delay(5);
  }
  Serial.printf("  status ToF %d/%d: %u %u\n", a.ch, b.ch, a.status, b.status);
  return n ? sum / (float)n : 4000;
}

int decideDirection() {
  float left  = sideAverage(tofLF, tofLR);
  float right = sideAverage(tofRF, tofRR);
  int d = (right < left) ? +1 : -1;
  Serial.printf("Smjer: lijevo %.0f mm, desno %.0f mm -> %s\n", left, right, d > 0 ? "CCW" : "CW");
  if (fabs(left - right) < 200) Serial.println("  UPOZORENJE: razlika mala, odluka nesigurna");
  return d;
}

// =====================================================================
//  OSNOVNI POKRETI
// =====================================================================

// Pravac s ToF-om naprijed: uz zid trake 'lane' (ili samo smjer ako zida nema).
// Staje kad je odoX >= stopX (ako stopX > -1e8) ili kad je prednji ToF <= frontStop
// (ako frontStop > 0, i to tek od odoX >= armX).
bool followLane(int section, int8_t lane, float stopX, float frontStop, float armX) {
  int side = (lane == route::IN) ? -dir : dir;          // +1 desni zid, -1 lijevi (kao open_challenge)
  float target = (lane == route::OUT && section == 0) ? PARK_LANE_WALL_MM : LANE_WALL_MM;
  Tof &fr = (side > 0) ? tofRF : tofLF;
  Tof &re = (side > 0) ? tofRR : tofLR;
  float spacing = (side > 0) ? TOF_SPACING_R_MM : TOF_SPACING_L_MM;
  int nearCnt = 0;
  unsigned long t0 = millis(), lastPrint = 0;
  motor(DRIVE_PWM);
  while (true) {
    tick();
    if (imuLost()) return fail("MPU-6050 ne odgovara");
    if (millis() - t0 > STRAIGHT_TIMEOUT_MS) return fail("timeout na pravcu");
    int f = readTof(tofF);
    if (tofF.fresh && f > 0 && f < EMERGENCY_MM) return fail("HITNO: zid ispred blize od 15 cm");
    if (stopX > -1e8 && odoX >= stopX) break;
    if (frontStop > 0 && odoX >= armX && tofF.fresh) {
      if (f > 0 && f <= frontStop) nearCnt++; else nearCnt = 0;
      if (nearCnt >= 2) break;                              // 2 mjerenja zaredom
    }
    int dF = readTof(fr), dR = readTof(re);
    bool wall = dF > 0 && dR > 0 && dF < WALL_MAX_MM && dR < WALL_MAX_MM &&
                min(dF, dR) > target - WALL_REJECT_MM;
    float corr, dist = -1;
    if (wall) {
      float ang = atan2f((float)(dF - dR), spacing) * RAD_TO_DEG;   // + = nos bjezi od zida
      dist = (dF + dR) / 2.0 * cosf(ang * DEG_TO_RAD);
      corr = side * (KP_DIST * (dist - target) + KP_ANGLE * ang);
      float wMeas = dist + SIDE_TO_CENTER_MM;
      if (lane == route::OUT) wMeas = G.corridor - wMeas;
      odoW += 0.2f * (wMeas - odoW);                    // zid ispravlja odometriju
    } else {
      corr = KP_HEADING * (yawDeg - course);            // drzi smjer sekcije
    }
    steerTo(STEER_CENTER + corr);
    if (millis() - lastPrint > 200) {
      lastPrint = millis();
      Serial.printf("    x %5.0f w %4.0f | %s zid %4.0f | F %4d | yaw %6.1f | kor %5.1f\n",
                    odoX, odoW, route::laneName(lane), dist, tofF.last, yawDeg - course, corr);
    }
    delay(1);
  }
  return true;
}

// Zaokret na punom zakretu do apsolutnog smjera 'target' (yaw, st.), naprijed.
bool turnToHeading(float target) {
  int d = (target > yawDeg) ? 1 : -1;
  steerTo(d > 0 ? STEER_LEFT : STEER_RIGHT);
  waitTick(200);
  unsigned long t0 = millis();
  motor(TURN_PWM);
  while (d * (target - yawDeg) > TURN_EARLY_DEG) {
    if (millis() - t0 > TURN_TIMEOUT_MS) return fail("timeout u zaokretu");
    tick();
    if (imuLost()) return fail("MPU-6050 ne odgovara");
    delay(2);
  }
  steerTo(STEER_CENTER);
  return true;
}

// Ravno u smjeru 'heading' (naprijed ili natrag) dok prednji ToF ne pokaze frontTarget.
bool driveAcross(float heading, float frontTarget) {
  int f = -1;
  unsigned long t0 = millis();
  while (millis() - t0 < 300) { int x = readTof(tofF); if (tofF.fresh) f = x; tick(); delay(5); }
  if (f > 0 && fabsf(f - frontTarget) <= ACROSS_TOL_MM) return true;
  int dirn = (f < 0 || f > frontTarget) ? 1 : -1;       // nema cilja = daleko = naprijed
  Serial.printf("    preko staze: F %d -> %.0f mm (%s)\n", f, frontTarget, dirn > 0 ? "naprijed" : "natrag");
  int ok = 0;
  t0 = millis();
  motor(dirn * ACROSS_PWM);
  while (true) {
    tick();
    if (imuLost()) return fail("MPU-6050 ne odgovara");
    if (millis() - t0 > STRAIGHT_TIMEOUT_MS) return fail("timeout preko staze");
    f = readTof(tofF);
    if (tofF.fresh) {
      if (f > 0 && f < EMERGENCY_MM) return fail("HITNO: zid ispred blize od 15 cm");
      bool done = (dirn > 0) ? (f > 0 && f <= frontTarget) : (f < 0 || f >= frontTarget);
      if (done) { if (++ok >= 2) break; } else ok = 0;
    }
    steerTo(STEER_CENTER + dirn * KP_HEADING * (yawDeg - heading));   // unatrag: obrnuti predznak
    delay(1);
  }
  stopHere();
  return true;
}

// Voznja zadanog puta (mm, + naprijed) s trenutnim zakretom kotaca.
bool driveDist(float mm, int pwm) {
  long e0 = encCount;
  float target = fabsf(mm);
  unsigned long t0 = millis();
  motor((mm > 0 ? 1 : -1) * pwm);
  while (labs(encCount - e0) * mmPerPulse() < target) {
    if (millis() - t0 > 3000) return fail("timeout u potezu");
    tick();
    delay(1);
  }
  stopHere();
  return true;
}

// =====================================================================
//  MANEVRI
// =====================================================================
bool exitParking() {
  Serial.println("Izlazak iz parkinga...");
  for (const ExitLeg &l : EXIT_LEGS) {
    bool left = (l.toward > 0) == (dir > 0);             // staza je lijevo u CCW, desno u CW
    steerTo(left ? STEER_LEFT : STEER_RIGHT);
    waitTick(250);
    if (!driveDist(l.mm, EXIT_PWM)) return false;
  }
  steerTo(STEER_CENTER);
  Serial.printf("  nakon izlaska: x %.0f w %.0f, smjer %.0f st. (ocekivano %d)\n", odoX, odoW, yawDeg - course, dir * 90);
  return true;
}

// Promjena trake usred sekcije: 90 st. prema novoj traci, preko staze, 90 st. natrag.
bool switchLane(int section, int8_t from, int8_t to) {
  float h1 = course + ((to == route::IN) ? dir : -dir) * 90.0f;   // prema unutarnjem zidu = lijevo u CCW
  Serial.printf("  promjena trake %s -> %s na x %.0f\n", route::laneName(from), route::laneName(to), odoX);
  if (!turnToHeading(h1)) return false;
  if (!driveAcross(h1, route::acrossFront(G, section, to))) return false;
  return turnToHeading(course);
}

// =====================================================================
//  CIJELI ZADATAK
// =====================================================================
char manualMap[route::LINES + 1] = "";

void printPose(const char *what) { Serial.printf("  %s: x %.0f w %.0f yaw %.1f\n", what, odoX, odoW, yawDeg - course); }

void runObstacle(bool useScan, bool drive) {
  Serial.println("\nNe diraj robota - kalibracija ziroskopa...");
  delay(500);
  gyroCalibrate();
  yawDeg = 0; lastYawUs = micros();
  course = 0;

  dir = decideDirection();
  int di = (dir > 0) ? 0 : 1;

  char obsMap[route::LINES + 1], raw[route::LINES + 1];
  if (useScan) {
    if (!scanCourse(obsMap, raw)) return;
  } else {
    if (strlen(manualMap) != route::LINES) { Serial.println("Nema rucne mape: 'm' + 12 znakova"); return; }
    strcpy(obsMap, manualMap);
  }
  route::Plan plan;
  route::build(plan, obsMap, dir, START_FIRST_LINE[di], FINISH_LAST_LINE[di], START_BLOCK_LINE[di]);
  printMapAndPlan(useScan ? raw : nullptr, obsMap, plan);
  if (!drive) { Serial.println("(samo sken - bez voznje)"); return; }

  // ---- start ----
  odoX = START_X_MM[di]; odoW = START_W_MM;
  lastEncOdo = encCount;
  yawDeg = 0; lastYawUs = micros();   // smjer u parkingu = smjer voznje
  unsigned long t0 = millis();

  if (!exitParking()) return;
  int8_t lane = plan.v[0].entry;
  float hIn = course + dir * 90.0f;   // lice prema unutarnjem zidu
  if (!driveAcross(hIn, route::startFront(G, lane))) return;
  if (!turnToHeading(course)) return;
  printPose("start u traci");

  // ---- 3 kruga ----
  for (int i = 0; i < route::VISITS; i++) {
    const route::Visit &v = plan.v[i];
    Serial.printf("[%2d] S%d %s%s%s  x %.0f w %.0f\n", i, v.section, route::laneName(v.entry),
                  v.midSwitch ? " -> " : "", v.midSwitch ? route::laneName(v.exit) : "", odoX, odoW);
    if (v.midSwitch) {
      float xs = route::switchStartX(G);
      if (odoX < xs) { if (!followLane(v.section, lane, xs, -1, 0)) return; }
      else Serial.printf("  (promjena trake kasni: x %.0f > %.0f)\n", odoX, xs);
      if (!switchLane(v.section, lane, v.exit)) return;
      lane = v.exit;
    }
    if (i == route::VISITS - 1) {
      if (!followLane(v.section, lane, STOP_X_MM[di], -1, 0)) return;
      fail("3 kruga gotova (TODO: parkiranje)");
      break;
    }
    const route::Visit &n = plan.v[i + 1];
    float fStop = route::cornerFront(G, n.section, n.entry);
    if (!followLane(v.section, lane, -1e9, fStop, CORNER_ARM_X_MM)) return;
    if (!turnToHeading(course + dir * 90.0f)) return;
    // novi okvir: stari unutarnji zid postaje linija L0, stari prednji zid novi vanjski
    float nx = -odoW, nw = odoX - 2.0f * G.lineStep;
    course += dir * 90.0f;
    odoX = nx; odoW = nw;
    lane = n.entry;
  }
  Serial.printf("Vrijeme: %.1f s\n", (millis() - t0) / 1000.0);
}

// 'q': jedan zaokret 90 st. ulijevo iz mirovanja - mjeri se pomak sredine robota
void turnTest() {
  gyroCalibrate();
  yawDeg = 0; lastYawUs = micros(); course = 0; dir = 1;
  odoX = 0; odoW = 500; lastEncOdo = encCount;
  long e0 = encCount;
  motor(TURN_PWM); waitTick(150);
  Serial.printf("Enkoder u voznji naprijed: %s (ENC_SIGN = %s)\n",
                encCount > e0 ? "RASTE" : "PADA", encCount > e0 ? "+1" : "-1");
  stopHere();
  odoX = 0; odoW = 500;
  Serial.println("Oznaci sredinu robota na podu. Zaokret za 2 s...");
  waitTick(2000);
  turnToHeading(90);
  stopHere();
  Serial.printf("Prema odometriji: naprijed %.0f mm, bocno %.0f mm. IZMJERI rucno i upisi\n", odoX, 500 - odoW);
  Serial.println("TURN_FWD_MM (u starom smjeru) i TURN_SIDE_MM (bocno).");
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== obstacle_run ===");
  WiFi.mode(WIFI_OFF);                // pravilo 11: bez bezicne komunikacije (Bluetooth se nigdje ne pokrece)

  G.laneWall = LANE_WALL_MM; G.parkLaneWall = PARK_LANE_WALL_MM;
  G.sideToCenter = SIDE_TO_CENTER_MM; G.frontToCenter = FRONT_TO_CENTER_MM;
  G.turnFwd = TURN_FWD_MM; G.turnSide = TURN_SIDE_MM;

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(STBY, OUTPUT);
  ledcAttach(PWMA, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  motorOff();
  ledcAttach(STEER_PIN, SERVO_PWM_FREQ, SERVO_PWM_BITS);
  steerTo(STEER_CENTER);
  ledcAttach(TOWER_PIN, SERVO_PWM_FREQ, SERVO_PWM_BITS);
  analogSetPinAttenuation(FEEDBACK_PIN, ADC_11db);
  towerWriteDeg(TOWER_FWD_DEG);

  pinMode(ENC_A, INPUT); pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, RISING);
  pinMode(START_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  if (chaosI2CCheck(false) > 0) halt("provjera I2C nije prosla - NE VOZITI");   // kamera se moze javiti kasnije
  mpuInit();
  if (!initTof(tofRF, VL53L1X::Short, 33000)) halt("desni prednji ToF (kanal 0)");
  if (!initTof(tofRR, VL53L1X::Short, 33000)) halt("desni zadnji ToF (kanal 1)");
  if (!initTof(tofLF, VL53L1X::Short, 33000)) halt("lijevi prednji ToF (kanal 2)");
  if (!initTof(tofF,  VL53L1X::Long,  50000)) halt("prednji ToF (kanal 3)");
  if (!initTof(tofLR, VL53L1X::Short, 33000)) halt("lijevi zadnji ToF (kanal 4)");

  if (BEZ_MOTORA) Serial.println("*** BEZ_MOTORA = true: pogon nece voziti ***");
  Serial.println("Spreman. START ili 's' | k sken | m <mapa> | r voznja s rucnom mapom | x izlaz | q zaokret");
}

void loop() {
  if (startPressed()) { runObstacle(true, true); return; }
  if (!Serial.available()) { delay(10); return; }
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = tolower(cmd[0]);
  if (c == 'k') runObstacle(true, false);
  else if (c == 'r') runObstacle(false, true);
  else if (c == 'm') {
    String m = cmd.substring(1);
    m.replace(" ", "");
    m.toUpperCase();
    if (m.length() == route::LINES) { strcpy(manualMap, m.c_str()); Serial.printf("Rucna mapa: %s\n", manualMap); }
    else Serial.println("Mapa mora imati 12 znakova R/G/-/? (razmaci dopusteni)");
  } else if (c == 'x') {
    gyroCalibrate(); yawDeg = 0; lastYawUs = micros(); course = 0;
    dir = decideDirection();
    odoX = START_X_MM[dir > 0 ? 0 : 1]; odoW = START_W_MM; lastEncOdo = encCount;
    exitParking();
  } else if (c == 'q') turnTest();
  else Serial.println("? START/s | k | m <mapa> | r | x | q");
  while (Serial.available()) Serial.read();
}
