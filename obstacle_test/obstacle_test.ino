// =====================================================================
//  obstacle_test.ino  -  Controlled C.H.A.O.S. v2  -  Obstacle Challenge (TEST)
//
//  Tijek (plan tima, koraci 1-6):
//    1. U parkingu: bocni ToF -> BLIZI zid = vanjski zid -> smjer voznje
//         zid DESNO blize  -> CCW (skrecemo lijevo, unutrasnjost je lijevo)
//         zid LIJEVO blize -> CW  (skrecemo desno, unutrasnjost je desno)
//    2. Toranj okrece kameru na NUM_POSES polozaja prema unutrasnjosti staze
//       (za CW isti kutovi, zrcaljeno).
//    3. Na svakom polozaju: ESP32 ceka da povratna veza servoa kaze "stigao",
//       ceka da se jarbol smiri, salje kameri SNIMI(polozaj, kut) i ceka
//       da kamera javi GOTOVO. Tek onda ide sljedeci polozaj.
//    4. Nakon zadnjeg polozaja salje ZAVRSI; kamera primijeni pravilo startne
//       sekcije, ESP32 procita kartu 24 mjesta (boja + visina stupa u px).
//    5. Iz karte se za svaku od 12 linija racuna ciljna udaljenost sredine
//       robota od vanjskog zida (vidi PLAN).
//    6. Izlaz iz parkinga (za sada RUCNO, vidi EXIT_RUCNO) i 3 kruga.
//
//  Numeracija mjesta (ista kao u ae3_test_ScanningWholeField.py):
//    mjesto = 6*sekcija + 2*linija + strana
//    sekcija 0 = startna, dalje redom kojim ih robot prolazi u svom smjeru
//    linija  0,1,2 = redom kojim ih robot prelazi
//    strana  0 = uz UNUTARNJI zid, 1 = uz VANJSKI zid
//
//  Koordinate u sekciji (iste za CW i CCW, kod je samo zrcaljen s 'dir'):
//    u   = uzduz sekcije, 0 = vanjski zid prethodne sekcije, 3000 = prednji zid
//    v   = udaljenost SREDINE robota od vanjskog zida
//    psi = smjer robota, 0 = uzduz sekcije, + = prema unutrasnjosti
//
//  Naredbe u Serial Monitoru (115200, "Newline"):
//    c   skeniranje + plan, motor se NE pokrece           <- prvo ovo
//    s   cijeli zadatak (ili START tipkalo)
//    p   ping kamere (provjera I2C protokola)
//    w   test tornja: svi polozaji za oba smjera + povratna veza
//    d   samo odluka o smjeru iz bocnih ToF-ova
//    m   plan iz upisane karte, npr.  m --R--- G----- -R--G- ----R- ccw
//
//  Potrebno: includes.ino + ChaosCheck.h, biblioteka "VL53L1X" by Pololu,
//            na kameri obstacle_challange/camera_scan/main.py (ISTI registri!)
// =====================================================================
#include <Wire.h>
#include <VL53L1X.h>
#include "../includes/includes.ino"
#include "../includes/ChaosCheck.h"

// =====================================================================
//  POSTAVKE
// =====================================================================
const bool BEZ_MOTORA     = true;   // true = sve radi, ali motor ne vozi
const bool EXIT_RUCNO     = true;    // true = nakon plana PRENESI robota na izlaznu poziciju i stisni START
                                     // (dok exit_parking.ino nije gotov). NA NATJECANJU NIJE DOPUSTENO.
const bool ALLOW_UNKNOWN  = true;    // true = mjesta '?' (bez ROI-ja) tretiraju se kao prazna (samo test!)

// ---------------- TORANJ KAMERE ----------------
// Kutovi od "ravno naprijed", + = prema unutrasnjosti staze. MORA biti isto kao KUTOVI u kameri.
// 270-st. servo daje najvise +-135 st. od "naprijed", i to samo ako je "naprijed" = 135 st. servoa.
const int   NUM_POSES = 3;
const int   POSE_DEG[NUM_POSES] = {35, 80, 125};
const int   TOWER_FORWARD_DEG = TOWER_CENTER; // servo kut kad kamera gleda ravno naprijed (vidi napomenu u includes.ino!)
const int   TOWER_SIGN        = +1;   // +1 ako VECI kut servoa okrece kameru ULIJEVO (provjeri naredbom 'w')
const float TOWER_FB_REF_DEG  = 135.0;// kut na kojem je izmjeren TOWER_MV_CENTER (NE TOWER_CENTER)
const float TOWER_TOL_DEG     = 4.0;  // "stigao" = povratna veza unutar +-4 st.
const int   TOWER_STABLE_N    = 3;    // toliko ocitanja zaredom unutar tolerancije
const unsigned long TOWER_TIMEOUT_MS = 2000;
const unsigned long TOWER_SETTLE_MS  = 300;   // jarbol se smiri nakon sto servo stane
const bool  TOWER_USE_FEEDBACK = true;        // false = samo fiksno cekanje

// ---------------- KAMERA (I2C protokol, ISTO kao u camera_scan/main.py) ----------------
//  ESP32 pise registre 0..4 JEDNOM transakcijom (SEQ je zadnji -> kamera vidi novu naredbu tek kad je cijela upisana)
const uint8_t CAM_REG_CMD     = 0;    // 0 CMD, 1 DIR (1=CCW 2=CW), 2 POSE, 3 ANGLE (st.), 4 SEQ
const uint8_t CAM_REG_STATUS  = 8;    // kamera pise: 8 STATUS, 9 ACK (=SEQ kad je gotova), 10 ERR, 11 INFO, 12 VERZIJA
const uint8_t CAM_REG_MAP     = 16;   // 16..39 boja mjesta ('R','G','-','?'), 40..63 visina stupa u px
const uint8_t CAM_VERSION     = 1;
enum { CMD_PING = 1, CMD_BEGIN = 2, CMD_CAPTURE = 3, CMD_FINISH = 4 };
enum { ST_IDLE = 0, ST_BUSY = 1, ST_DONE = 2, ST_ERROR = 3 };
enum { CAM_OK, CAM_ERR, CAM_TIMEOUT, CAM_I2C_FAIL };
const unsigned long CAM_BEGIN_TIMEOUT_MS   = 4000;  // kamera 1.5 s smiruje ekspoziciju i zakljuca balans
const unsigned long CAM_CAPTURE_TIMEOUT_MS = 3000;
const int   CAM_TRIES = 2;

// ---------------- SMJER IZ PARKINGA ----------------
const int   DIR_CLOSE_MM = 250;       // blize od ovoga = zid uz robota
const int   DIR_FAR_MM   = 400;       // dalje od ovoga = slobodna traka
const int   DIR_MIN_VALID = 3;        // najmanje valjanih ocitanja po strani

// ---------------- STAZA (IZMJERITI NA PODLOZI!) ----------------
const int   NSEC = 4, NLINE = 3, NSEAT = 24, NK = NSEC * NLINE;
const float CORRIDOR_MM = 1000.0;     // vanjski <-> unutarnji zid (Obstacle: uvijek 1000)
const float SECTION_MM  = 3000.0;     // duljina vanjskog zida jedne strane (kut do kuta)
const float LINE_U_MM[NLINE] = {1000.0, 1500.0, 2000.0};   // TODO: poprecne linije s mjestima za stupove
const float SEAT_V_MM[2]     = {667.0, 333.0};             // TODO: [0] unutarnje, [1] vanjsko mjesto, od vanjskog zida
const float ISLAND_U0_MM = 1000.0, ISLAND_U1_MM = 2000.0;  // gdje postoji unutarnji zid (inace kutna zona)

// ---------------- PLAN ----------------
const float CLEAR_MM       = 140.0;   // sredina robota <-> sredina stupa: 70 (pola robota) + 25 (pola stupa) + 45 rezerva
const float V_MIN_MM       = 120.0;   // najblize vanjskom zidu (sredina robota)
const float V_MAX_MM       = CORRIDOR_MM - 120.0;
const float LANE_MARGIN_MM = 40.0;    // koliko se uvuci u dopusteni pojas
const float PASS_MM        = 130.0;   // stup je "prijeden" kad je sredina robota toliko iza linije

// ---------------- ROBOT (IZMJERITI!) ----------------
const float SIDE_TOF_OFF_MM  = 70.0;  // sredina robota -> lice bocnog ToF-a
const float FRONT_TOF_OFF_MM = 80.0;  // sredina robota -> lice prednjeg ToF-a
const float TOF_SPACING_MM   = 120.0; // prednji <-> zadnji bocni ToF (isto kao open_challenge)
const float TURN_R_MM        = 200.0; // TODO: polumjer kruga SREDINE robota na punom zakretu (izmjeriti!)

// ---------------- VOZNJA ----------------
const int   DRIVE_PWM      = 150;
const int   TURN_PWM       = 110;
const float K_V            = 0.15;    // st. zeljenog smjera po mm greske v (100 mm -> 15 st.)
const float PSI_MAX_DEG    = 50.0;    // najstrmiji prijelaz izmedju traka
const float KP_PSI         = 1.0;     // st. servoa po st. greske smjera
const float K_FUSE         = 0.3;     // koliko vjerujemo zidu kod ispravka v
const float GATE_MM        = 150.0;   // ocitanje zida dalje od procjene = stup ili smece, ignoriraj
const float GATE_WIDE_MM   = 350.0;   // ako dugo nema ispravka (nakon zavoja), siri prozor
const float REFIX_AFTER_MM = 500.0;
const float TURN_EARLY_DEG = 6.0;
const float FRONT_MIN_MM   = 80.0;    // zavoj ODMAH ako je do prednjeg zida manje od TURN_R + ovo
const int   EMERGENCY_MM   = 100;
const unsigned long SEGMENT_TIMEOUT_MS = 15000;
const unsigned long TURN_TIMEOUT_MS    = 5000;

// ---------------- ToF / MG995 (isto kao open_challenge) ----------------
const uint8_t TOF_ROI_SIZE   = 4;
const uint8_t TOF_ROI_CENTER = 199;
const int   STEER_US_MIN = 500;
const int   STEER_US_MAX = 2500;
const float DEG2RAD = PI / 180.0;

// =====================================================================
//  ToF iza MUX-a (kopirano iz open_challenge, + MinRangeClipped)
// =====================================================================
struct Tof {
  VL53L1X s;
  uint8_t ch;
  int     last;     // mm, -1 = nema valjanog
  bool    fresh;
  uint8_t status;
};
Tof tofRF = {VL53L1X(), MUX_TOF_RIGHT_FRONT, -1, false, 255};
Tof tofRR = {VL53L1X(), MUX_TOF_RIGHT_REAR,  -1, false, 255};
Tof tofLF = {VL53L1X(), MUX_TOF_LEFT_FRONT,  -1, false, 255};
Tof tofLR = {VL53L1X(), MUX_TOF_LEFT_REAR,   -1, false, 255};
Tof tofF  = {VL53L1X(), MUX_TOF_FRONT,       -1, false, 255};

struct SideStat { int valid, clipped, med; };   // ovdje, prije prve funkcije (Arduino prototipovi)

volatile long encCount = 0;
float gyroBias = 0, yawDeg = 0;       // yaw: + = ulijevo, NE resetira se izmedju zavoja
unsigned long lastYawUs = 0;
int imuErrors = 0;

void IRAM_ATTR encISR() { if (digitalRead(ENC_B)) encCount++; else encCount--; }

void motor(int speed) {
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
  t.s.setROISize(TOF_ROI_SIZE, TOF_ROI_SIZE);
  t.s.setROICenter(TOF_ROI_CENTER);
  t.s.setMeasurementTimingBudget(budgetUs);
  t.s.startContinuous(budgetUs / 1000 + 5);
  return true;
}

// Ne ceka. "Clipped" (status 3) = cilj blize od ~4 cm, vrijednost je mala ali valjana.
int readTof(Tof &t) {
  mux(t.ch);
  t.fresh = false;
  if (t.s.dataReady()) {
    t.s.read(false);
    t.status = (uint8_t)t.s.ranging_data.range_status;
    bool ok = (t.status == VL53L1X::RangeValid || t.status == VL53L1X::RangeValidMinRangeClipped);
    t.last  = ok ? t.s.ranging_data.range_mm : -1;
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
void resetYaw() { yawDeg = 0; lastYawUs = micros(); }
void waitMs(unsigned long ms) { unsigned long t0 = millis(); while (millis() - t0 < ms) { updateYaw(); delay(2); } }

void stopAll(const char *why) {
  brake(); delay(300); motorOff();
  steerTo(STEER_CENTER);
  Serial.print("STOP: "); Serial.println(why);
}

bool startPressed() {
  if (digitalRead(START_PIN) == LOW) { delay(30); while (digitalRead(START_PIN) == LOW) delay(5); return true; }
  return false;
}

// Ceka START tipkalo ili 's' (ostalo iz Seriala se baca)
void waitStart(const char *msg) {
  Serial.println(msg);
  while (Serial.available()) Serial.read();
  while (true) {
    if (startPressed()) return;
    while (Serial.available()) { char c = Serial.read(); if (c == 's' || c == 'S') return; }
    delay(10);
  }
}

// =====================================================================
//  1. SMJER IZ PARKINGA
// =====================================================================
SideStat sampleSide(Tof &a, Tof &b) {
  int v[40]; int n = 0, clipped = 0, fresh = 0;
  unsigned long t0 = millis();
  while (fresh < 30 && millis() - t0 < 2000) {
    Tof *ab[2] = {&a, &b};
    for (int i = 0; i < 2; i++) {
      int x = readTof(*ab[i]);
      if (!ab[i]->fresh) continue;
      fresh++;
      if (ab[i]->status == VL53L1X::RangeValidMinRangeClipped) clipped++;
      if (x > 0 && n < 40) v[n++] = x;
    }
    delay(5);
  }
  for (int i = 1; i < n; i++) { int k = v[i], j = i - 1; while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; } v[j + 1] = k; }
  SideStat s = {n, clipped, n ? v[n / 2] : -1};
  return s;
}

// +1 = CCW, -1 = CW, 0 = ne znam (NE VOZITI)
int detectDirection() {
  for (int attempt = 1; attempt <= 3; attempt++) {
    SideStat L = sampleSide(tofLF, tofLR);
    SideStat R = sampleSide(tofRF, tofRR);
    Serial.printf("Smjer (pokusaj %d): lijevo med %d mm (%d valjanih, %d clipped) | desno med %d mm (%d valjanih, %d clipped)\n",
                  attempt, L.med, L.valid, L.clipped, R.med, R.valid, R.clipped);
    bool lOk = L.valid >= DIR_MIN_VALID, rOk = R.valid >= DIR_MIN_VALID;
    bool lClose = lOk && L.med < DIR_CLOSE_MM, rClose = rOk && R.med < DIR_CLOSE_MM;
    bool lFar   = lOk && L.med > DIR_FAR_MM,   rFar   = rOk && R.med > DIR_FAR_MM;
    if (rClose && !lClose) return +1;
    if (lClose && !rClose) return -1;
    // Senzor koji je PREBLIZU zidu cesto ne daje nista valjano. U parkingu jedna strana MORA biti blizu,
    // pa "slijepa" strana + "daleka" druga strana znaci: slijepa strana je zid.
    if (!rOk && lFar) { Serial.println("  desno nema ocitanja, lijevo daleko -> zid je desno (preblizu za ToF)"); return +1; }
    if (!lOk && rFar) { Serial.println("  lijevo nema ocitanja, desno daleko -> zid je lijevo (preblizu za ToF)"); return -1; }
    Serial.println("  nesigurno, ponavljam...");
    delay(200);
  }
  return 0;
}

// =====================================================================
//  2. TORANJ KAMERE
// =====================================================================
int towerServoDeg(int pose, int dir) { return TOWER_FORWARD_DEG + TOWER_SIGN * dir * POSE_DEG[pose]; }

void towerWrite(float deg) {
  int us = TOWER_MIN_US + (int)(deg * (TOWER_MAX_US - TOWER_MIN_US) / TOWER_MAX_DEG);
  ledcWrite(TOWER_PIN, (uint32_t)us * ((1UL << SERVO_PWM_BITS) - 1) / 20000UL);
}

float towerFeedbackDeg() {
  float mv = 0;
  for (int i = 0; i < 4; i++) mv += analogReadMilliVolts(FEEDBACK_PIN);
  mv /= 4;
  return TOWER_FB_REF_DEG + (mv - TOWER_MV_CENTER) / TOWER_MV_PER_DEG;
}

// Vraca true ako je povratna veza potvrdila polozaj. Uvijek ceka i TOWER_SETTLE_MS.
bool towerGo(int servoDeg, float *fbOut) {
  servoDeg = constrain(servoDeg, 0, TOWER_MAX_DEG);
  towerWrite(servoDeg);
  bool ok = false;
  float fb = NAN;
  unsigned long t0 = millis();
  if (TOWER_USE_FEEDBACK) {
    int stable = 0;
    while (millis() - t0 < TOWER_TIMEOUT_MS) {
      waitMs(20);
      fb = towerFeedbackDeg();
      stable = (fabsf(fb - servoDeg) <= TOWER_TOL_DEG) ? stable + 1 : 0;
      if (stable >= TOWER_STABLE_N) { ok = true; break; }
    }
  } else {
    waitMs(800);
  }
  waitMs(TOWER_SETTLE_MS);
  if (fbOut) *fbOut = fb;
  if (TOWER_USE_FEEDBACK && !ok)
    Serial.printf("  UPOZORENJE: toranj nije potvrdio %d st. (povratna veza %.1f st.)\n", servoDeg, fb);
  return ok || !TOWER_USE_FEEDBACK;
}

// Provjera da svi polozaji stanu u hod servoa za OBA smjera
bool checkTowerRange() {
  bool ok = true;
  for (int d = 1; d >= -1; d -= 2)
    for (int i = 0; i < NUM_POSES; i++) {
      int s = towerServoDeg(i, d);
      if (s < 0 || s > TOWER_MAX_DEG) {
        Serial.printf("GRESKA TORNJA: %s polozaj %d (%d st.) treba servo %d st. - izvan 0..%d!\n",
                      d > 0 ? "CCW" : "CW", i, POSE_DEG[i], s, TOWER_MAX_DEG);
        ok = false;
      }
    }
  return ok;
}

// =====================================================================
//  3./4. KAMERA (I2C registri)
// =====================================================================
uint8_t camSeq = 0;
uint8_t mapColor[NSEAT], mapH[NSEAT];

bool camWrite(uint8_t reg, const uint8_t *data, uint8_t n) {
  mux(MUX_CAMERA);
  Wire.beginTransmission(ADDR_AE3);
  Wire.write(reg);
  Wire.write(data, n);
  return Wire.endTransmission() == 0;
}

bool camRead(uint8_t reg, uint8_t *out, uint8_t n) {
  mux(MUX_CAMERA);
  Wire.beginTransmission(ADDR_AE3);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;     // repeated start
  if (Wire.requestFrom((uint8_t)ADDR_AE3, n) != n) return false;
  for (int i = 0; i < n; i++) out[i] = Wire.read();
  return true;
}

// Posalje naredbu i ceka da kamera vrati ACK == SEQ. err/info iz registara 10/11.
int camCommand(uint8_t cmd, uint8_t dirCode, uint8_t pose, uint8_t angle, unsigned long timeoutMs,
               uint8_t *err, uint8_t *info) {
  uint8_t st[5];
  if (camRead(CAM_REG_STATUS, st, 5)) camSeq = st[1];     // novi SEQ sigurno razlicit od zadnjeg ACK-a kamere
  camSeq++; if (camSeq == 0) camSeq = 1;
  uint8_t pkt[5] = {cmd, dirCode, pose, angle, camSeq};
  if (!camWrite(CAM_REG_CMD, pkt, 5)) return CAM_I2C_FAIL;
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    waitMs(20);
    if (!camRead(CAM_REG_STATUS, st, 5)) continue;
    if (st[1] != camSeq) continue;                        // kamera jos radi (ili nije ni vidjela naredbu)
    if (st[0] != ST_DONE && st[0] != ST_ERROR) continue;
    if (err)  *err  = st[2];
    if (info) *info = st[3];
    return st[0] == ST_DONE ? CAM_OK : CAM_ERR;
  }
  return CAM_TIMEOUT;
}

const char *camResultName(int r) {
  switch (r) { case CAM_OK: return "OK"; case CAM_ERR: return "GRESKA KAMERE"; case CAM_TIMEOUT: return "TIMEOUT"; default: return "I2C NE RADI"; }
}

int camCommandRetry(uint8_t cmd, uint8_t dirCode, uint8_t pose, uint8_t angle, unsigned long timeoutMs, uint8_t *info) {
  int r = CAM_I2C_FAIL;
  for (int t = 1; t <= CAM_TRIES; t++) {
    uint8_t err = 0;
    r = camCommand(cmd, dirCode, pose, angle, timeoutMs, &err, info);
    if (r == CAM_OK) return r;
    Serial.printf("  kamera: naredba %d pokusaj %d -> %s (err %d)\n", cmd, t, camResultName(r), err);
  }
  return r;
}

bool camPing(bool verbose) {
  uint8_t st[5];
  if (!camRead(CAM_REG_STATUS, st, 5)) { if (verbose) Serial.println("Ping: kamera se ne javlja na I2C (radi li main.py?)"); return false; }
  if (st[4] != CAM_VERSION) {
    if (verbose) Serial.printf("Ping: verzija protokola kamere %d, ocekujem %d - je li na kameri camera_scan/main.py?\n", st[4], CAM_VERSION);
    return false;
  }
  uint8_t payload = (uint8_t)(millis() & 0xFF), info = 0;
  unsigned long t0 = millis();
  int r = camCommand(CMD_PING, 0, payload, 0, 500, nullptr, &info);
  bool ok = (r == CAM_OK && info == (uint8_t)(payload ^ 0xFF));
  if (verbose) Serial.printf("Ping: %s, odgovor %s, %lu ms\n", camResultName(r), ok ? "ISPRAVAN" : "KRIV", millis() - t0);
  return ok;
}

bool camReadMap() {
  uint8_t buf[48];
  if (!camRead(CAM_REG_MAP, buf, 48)) return false;
  for (int i = 0; i < NSEAT; i++) { mapColor[i] = buf[i]; mapH[i] = buf[24 + i]; }
  return true;
}

void printMap() {
  Serial.print("Karta: ");
  for (int s = 0; s < NSEC; s++) {
    for (int k = 0; k < 6; k++) Serial.print((char)mapColor[6 * s + k]);
    Serial.print(' ');
  }
  Serial.println("   (grupa = sekcija; parovi = linija: unutarnje, vanjsko)");
  for (int i = 0; i < NSEAT; i++)
    if (mapColor[i] == 'R' || mapColor[i] == 'G')
      Serial.printf("  mjesto %2d (S%d L%d %c): %c, visina %d px\n", i, i / 6, (i % 6) / 2, (i % 2) ? 'V' : 'U', mapColor[i], mapH[i]);
}

// Koraci 2-4: cijelo skeniranje. Vraca true ako su svi polozaji uspjeli.
bool scanCourse(int dir) {
  uint8_t dc = (dir > 0) ? 1 : 2;
  uint8_t info = 0;
  Serial.printf("\n--- SKEN (%s) ---\n", dir > 0 ? "CCW" : "CW");

  // kamera zakljucava balans bijele na polozaju 0 - isto kao pri kalibraciji ROI-ja
  towerGo(towerServoDeg(0, dir), nullptr);
  int r = camCommandRetry(CMD_BEGIN, dc, 0, 0, CAM_BEGIN_TIMEOUT_MS, &info);
  if (r != CAM_OK) { Serial.println("Kamera nije prihvatila POCETAK (ima li ROI za ovaj smjer?)"); return false; }

  bool allOk = true;
  for (int i = 0; i < NUM_POSES; i++) {
    int sd = towerServoDeg(i, dir);
    float fb;
    unsigned long t0 = millis();
    towerGo(sd, &fb);
    unsigned long tTower = millis() - t0;
    r = camCommandRetry(CMD_CAPTURE, dc, i, (uint8_t)POSE_DEG[i], CAM_CAPTURE_TIMEOUT_MS, &info);
    Serial.printf("Polozaj %d: %3d st. (servo %d, povratna %.1f), toranj %lu ms, kamera %s, stupova %d, ukupno %lu ms\n",
                  i, POSE_DEG[i], sd, fb, tTower, camResultName(r), info, millis() - t0);
    if (r != CAM_OK) allOk = false;
  }

  r = camCommandRetry(CMD_FINISH, dc, 0, 0, CAM_CAPTURE_TIMEOUT_MS, &info);
  if (r != CAM_OK) { Serial.println("Kamera nije zavrsila kartu"); return false; }
  if (!camReadMap()) { Serial.println("Citanje karte nije uspjelo"); return false; }
  Serial.printf("Kamera: %d mjesta bez ROI-ja ('?')\n", info);

  towerGo(TOWER_FORWARD_DEG, nullptr); // kamera ravno, ne smeta u voznji
  return allOk;
}

// =====================================================================
//  5. PLAN  (cisti izracun, bez hardvera - testira se i na racunalu)
//     Za svaku liniju k = 3*sekcija + linija trazimo dopusteni pojas [lo, hi] za v:
//       crveni stup -> drzi DESNO, zeleni -> LIJEVO.
//       CCW: desno = vanjski zid -> crveni: v < stup - CLEAR;  zeleni: v > stup + CLEAR
//       CW:  zrcaljeno.
//     Slobodna linija preuzima pojas SLJEDECE zauzete (robot se rano pripremi).
//     Zatim jedan prolaz unaprijed: cilj = prethodni cilj stisnut u pojas
//     (najmanje vijuganja), uz LANE_MARGIN od rubova pojasa.
// =====================================================================
// ==== PLAN BEGIN
float planV[NK], planLo[NK], planHi[NK];
bool  planHas[NK], planBad[NK];

float clampInto(float x, float lo, float hi, float m) {
  if (hi - lo < 2 * m) return 0.5f * (lo + hi);       // uzak ili nemoguc pojas -> sredina
  if (x < lo + m) return lo + m;
  if (x > hi - m) return hi - m;
  return x;
}

// colors: 24 znaka 'R','G','-','?'; dir +1 CCW / -1 CW. Vraca broj nemogucih linija.
int buildPlan(const uint8_t *colors, int dir) {
  float lo[NK], hi[NK];
  for (int k = 0; k < NK; k++) { lo[k] = V_MIN_MM; hi[k] = V_MAX_MM; planHas[k] = false; planBad[k] = false; }
  for (int seat = 0; seat < NSEAT; seat++) {
    char c = (char)colors[seat];
    if (c != 'R' && c != 'G') continue;
    int k = seat / 2;                                   // = 3*sekcija + linija
    float sv = SEAT_V_MM[seat % 2];
    bool outerSide = ((c == 'R') == (dir > 0));         // prolazimo izmedju stupa i VANJSKOG zida
    if (outerSide) hi[k] = fminf(hi[k], sv - CLEAR_MM);
    else           lo[k] = fmaxf(lo[k], sv + CLEAR_MM);
    planHas[k] = true;
  }
  int k0 = -1, bad = 0;
  for (int k = 0; k < NK; k++) {
    if (planHas[k] && k0 < 0) k0 = k;
    if (planHas[k] && lo[k] > hi[k]) { planBad[k] = true; bad++; }
  }
  if (k0 < 0) {                                         // nema nijednog stupa
    for (int k = 0; k < NK; k++) { planLo[k] = lo[k]; planHi[k] = hi[k]; planV[k] = 0.5f * CORRIDOR_MM; }
    return 0;
  }
  for (int k = 0; k < NK; k++) {                        // slobodne linije: pojas sljedece zauzete (kruzno)
    int j = k;
    while (!planHas[j]) j = (j + 1) % NK;
    planLo[k] = lo[j]; planHi[k] = hi[j];
  }
  float prev = 0.5f * (planLo[k0] + planHi[k0]);
  for (int i = 0; i < NK; i++) {
    int k = (k0 + i) % NK;
    planV[k] = clampInto(prev, planLo[k], planHi[k], LANE_MARGIN_MM);
    prev = planV[k];
  }
  return bad;
}
// ==== PLAN END

void printPlan(int dir) {
  Serial.printf("\n--- PLAN (%s) --- v = udaljenost sredine robota od vanjskog zida\n", dir > 0 ? "CCW" : "CW");
  Serial.println("sek lin  U V   pojas [lo..hi]   cilj   napomena");
  for (int k = 0; k < NK; k++) {
    int s = k / 3, l = k % 3;
    Serial.printf(" %d   %d   %c %c   [%4.0f..%4.0f]   %4.0f   %s%s\n", s, l,
                  (char)mapColor[6 * s + 2 * l], (char)mapColor[6 * s + 2 * l + 1],
                  planLo[k], planHi[k], planV[k],
                  planHas[k] ? "" : "(slobodno, priprema)", planBad[k] ? " NEMOGUCE!" : "");
  }
  // brzi prijelazi: treba li strmiji smjer od PSI_MAX?
  for (int s = 0; s < NSEC; s++)
    for (int l = 0; l + 1 < NLINE; l++) {
      float dv = fabsf(planV[3 * s + l + 1] - planV[3 * s + l]);
      float du = LINE_U_MM[l + 1] - LINE_U_MM[l] - PASS_MM;
      float need = atan2f(dv, du) / DEG2RAD;
      if (planHas[3 * s + l] && need > PSI_MAX_DEG * 0.8)
        Serial.printf("  TIJESNO: S%d L%d->L%d mijenja traku %.0f mm na %.0f mm (treba ~%.0f st.)\n", s, l, l + 1, dv, du, need);
    }
  for (int s = 0; s < NSEC; s++) {                      // zavoj s unutarnje trake prema stupu na liniji 0
    float vOld = planV[3 * s + 2];
    int nx = (s + 1) % NSEC;
    if (planHas[3 * nx] && vOld + TURN_R_MM > LINE_U_MM[0] - PASS_MM)
      Serial.printf("  TIJESNO: zavoj S%d->S%d s v=%.0f zavrsava na u=%.0f, a stup je na liniji 0 (u=%.0f)\n",
                    s, nx, vOld, vOld + TURN_R_MM, LINE_U_MM[0]);
  }
}

// =====================================================================
//  6. VOZNJA
// =====================================================================
int   dir = 0;                        // +1 CCW, -1 CW
int   turnsDone = 0;
float posU = 0, posV = 0;             // sredina robota u koordinatama sekcije
long  lastEnc = 0;
float sinceFixMm = 0;

float psiDeg() { return dir * yawDeg - 90.0f * turnsDone; }

void updateOdo() {
  updateYaw();
  long e = encCount;
  float d = fabsf((float)(e - lastEnc)) * WHEEL_CIRCUMFERENCE_MM / ENC_PULSES_PER_WHEEL_REV;   // vozimo samo naprijed
  lastEnc = e;
  float p = psiDeg() * DEG2RAD;
  posU += d * cosf(p);
  posV += d * sinf(p);
  sinceFixMm += d;
}

Tof &outerF() { return dir > 0 ? tofRF : tofLF; }
Tof &outerR() { return dir > 0 ? tofRR : tofLR; }
Tof &innerF() { return dir > 0 ? tofLF : tofRF; }
Tof &innerR() { return dir > 0 ? tofLR : tofRR; }

// Udaljenost od zida okomito (mm od lica senzora), -1 = nema. ang: + = nos bjezi od zida.
float wallDist(Tof &f, Tof &r, float *angOut) {
  int a = readTof(f), b = readTof(r);
  if (a <= 0 || b <= 0) return -1;
  float ang = atan2f((float)(a - b), TOF_SPACING_MM);
  if (angOut) *angOut = ang / DEG2RAD;
  return 0.5f * (a + b) * cosf(ang);
}

int fuseWalls() {                     // 0 = nista, 1 = vanjski, 2 = unutarnji, 3 = oba
  int used = 0;
  float gate = (sinceFixMm > REFIX_AFTER_MM) ? GATE_WIDE_MM : GATE_MM;
  float dO = wallDist(outerF(), outerR(), nullptr);
  if (dO > 0) {
    float vm = dO + SIDE_TOF_OFF_MM;
    if (fabsf(vm - posV) < gate) { posV += K_FUSE * (vm - posV); used |= 1; }
  }
  if (posU > ISLAND_U0_MM + 50 && posU < ISLAND_U1_MM - 50) {       // unutarnji zid postoji samo uz otok
    float dI = wallDist(innerF(), innerR(), nullptr);
    if (dI > 0) {
      float vm = CORRIDOR_MM - dI - SIDE_TOF_OFF_MM;
      if (fabsf(vm - posV) < gate) { posV += K_FUSE * (vm - posV); used |= 2; }
    }
  }
  if (used) sinceFixMm = 0;
  return used;
}

// Cilj v za trenutni u: prva linija koju jos nismo prosli; nakon zadnje drzi njen cilj.
float targetV(int sec, float u) {
  for (int l = 0; l < NLINE; l++)
    if (u < LINE_U_MM[l] + PASS_MM) return planV[3 * sec + l];
  return planV[3 * sec + NLINE - 1];
}

// u nakon kojega su svi stupovi ove sekcije prijedjeni (0 = nema stupova)
float lastPillarU(int sec) {
  for (int l = NLINE - 1; l >= 0; l--) if (planHas[3 * sec + l]) return LINE_U_MM[l] + PASS_MM;
  return 0;
}

void steerLane(float vT) {
  float psiD = constrain(K_V * (vT - posV), -PSI_MAX_DEG, PSI_MAX_DEG);
  float cmd  = KP_PSI * (psiD - psiDeg());
  steerTo(STEER_CENTER - dir * cmd);  // + psi = prema unutrasnjosti: CCW lijevo (manji kut), CW desno
}

// Zavoj 90 st. prema unutrasnjosti. Dtrig = udaljenost sredine robota od prednjeg zida na pocetku.
bool turnCorner(float Dtrig) {
  float vOld = posV;
  steerTo(dir > 0 ? STEER_LEFT : STEER_RIGHT);
  motor(TURN_PWM);
  float target = 90.0f * (turnsDone + 1) - TURN_EARLY_DEG;
  unsigned long t0 = millis();
  while (dir * yawDeg < target) {
    if (millis() - t0 > TURN_TIMEOUT_MS) { stopAll("timeout u zavoju"); return false; }
    updateYaw();
    if (imuLost()) { stopAll("MPU-6050 ne odgovara"); return false; }
    delay(2);
  }
  turnsDone++;
  // geometrija luka polumjera R: robot ide R naprijed i R prema unutra
  posU = vOld + TURN_R_MM;            // novi u = stari v + R
  posV = Dtrig - TURN_R_MM;           // novi v = stara udaljenost do prednjeg zida - R
  lastEnc = encCount;                 // put u zavoju je vec uracunat geometrijom
  sinceFixMm = REFIX_AFTER_MM + 1;    // sljedeci zid prihvati sirim prozorom
  steerTo(STEER_CENTER);
  motor(DRIVE_PWM);
  Serial.printf("  zavoj %d gotov: u=%.0f v=%.0f psi=%.1f\n", turnsDone, posU, posV, psiDeg());
  return true;
}

// Jedna sekcija. finishU > 0 -> zadnja sekcija, stani na finishU. Inace zavrsi zavojem.
bool driveSection(int sec, float finishU) {
  int   nx    = (sec + 1) % NSEC;
  float vNext = planV[3 * nx];
  float uFree = lastPillarU(sec);
  unsigned long t0 = millis(), lastPrint = 0;
  motor(DRIVE_PWM);
  while (true) {
    if (millis() - t0 > SEGMENT_TIMEOUT_MS) { stopAll("timeout u sekciji"); return false; }
    updateOdo();
    if (imuLost()) { stopAll("MPU-6050 ne odgovara"); return false; }
    int used = fuseWalls();

    int f = readTof(tofF);
    if (tofF.fresh && f > 0 && f < EMERGENCY_MM) { stopAll("HITNO: nesto ispred blize od 10 cm"); return false; }
    // prednji zid ispravlja u samo kad smo iza zadnje linije (ispred nema stupova)
    if (tofF.fresh && f > 0 && posU > LINE_U_MM[NLINE - 1] - 200 && fabsf(psiDeg()) < 15) {
      float uMeas = SECTION_MM - (f + FRONT_TOF_OFF_MM);
      if (fabsf(uMeas - posU) < 400) posU += 0.3f * (uMeas - posU);
    }

    float vT = targetV(sec, posU);
    steerLane(vT);

    if (millis() - lastPrint > 150) {
      lastPrint = millis();
      Serial.printf("  S%d u%5.0f v%4.0f cilj%4.0f psi%6.1f F%5d zid%d\n", sec, posU, posV, vT, psiDeg(), tofF.last, used);
    }

    if (finishU > 0) {
      if (posU >= finishU) { stopAll("3 kruga gotova"); return true; }
      continue;
    }
    float D = SECTION_MM - posU;      // sredina robota -> prednji zid
    bool pillarsDone = posU >= uFree;
    if ((pillarsDone && D <= vNext + TURN_R_MM) || D <= TURN_R_MM + FRONT_MIN_MM) {
      Serial.printf("  zavoj S%d->S%d: D=%.0f v=%.0f -> cilj %.0f\n", sec, nx, D, posV, vNext);
      return turnCorner(D);
    }
  }
}

float medianFront(int n) {
  int v[15]; int k = 0;
  unsigned long t0 = millis();
  while (k < n && k < 15 && millis() - t0 < 1500) { int x = readTof(tofF); if (tofF.fresh && x > 0) v[k++] = x; delay(5); }
  if (k == 0) return -1;
  for (int i = 1; i < k; i++) { int a = v[i], j = i - 1; while (j >= 0 && v[j] > a) { v[j + 1] = v[j]; j--; } v[j + 1] = a; }
  return v[k / 2];
}

bool driveLaps() {
  turnsDone = 0;
  resetYaw();                          // robot gleda uzduz sekcije
  lastEnc = encCount;
  // gdje smo? u iz prednjeg ToF-a (Long), v iz vanjskog zida
  float f = medianFront(7);
  posU = (f > 0) ? SECTION_MM - (f + FRONT_TOF_OFF_MM) : 1500.0f;
  float dO = -1;
  for (int i = 0; i < 20 && dO <= 0; i++) { dO = wallDist(outerF(), outerR(), nullptr); delay(20); }
  posV = (dO > 0) ? dO + SIDE_TOF_OFF_MM : 0.5f * CORRIDOR_MM;
  sinceFixMm = 0;
  float finishU = posU;                // stajemo gdje smo krenuli (startna sekcija)
  Serial.printf("Start: u=%.0f mm (prednji %.0f), v=%.0f mm, cilj na kraju u=%.0f\n", posU, f, posV, finishU);

  const int totalTurns = 3 * NSEC;
  for (int t = 0; t < totalTurns; t++) {
    int sec = t % NSEC;
    Serial.printf("Sekcija %d (krug %d)\n", sec, t / NSEC + 1);
    if (!driveSection(sec, 0)) return false;
  }
  Serial.println("Zavrsna ravnina u startnoj sekciji");
  return driveSection(0, finishU);
}

// =====================================================================
//  CIJELI ZADATAK
// =====================================================================
void runObstacle(bool drive) {
  if (!checkTowerRange()) { Serial.println("Popravi POSE_DEG / TOWER_FORWARD_DEG prije skeniranja."); return; }
  Serial.println("Ne diraj robota - kalibracija ziroskopa...");
  delay(800);
  gyroCalibrate();
  resetYaw();

  // 1. smjer
  dir = detectDirection();
  if (dir == 0) { Serial.println("Smjer NIJE odredjen - NE VOZIM."); return; }
  Serial.println(dir > 0 ? "Smjer: CCW (zid desno, skrecemo lijevo)" : "Smjer: CW (zid lijevo, skrecemo desno)");

  // 2.-4. sken
  unsigned long t0 = millis();
  bool scanOk = scanCourse(dir);
  Serial.printf("Sken trajao %.1f s\n", (millis() - t0) / 1000.0);
  printMap();
  int unknown = 0;
  for (int i = 0; i < NSEAT; i++) if (mapColor[i] == '?') unknown++;
  if (!scanOk) { Serial.println("Sken NIJE uspio na svim polozajima - NE VOZIM."); return; }
  if (unknown && !ALLOW_UNKNOWN) { Serial.println("Ima mjesta '?' - NE VOZIM (ALLOW_UNKNOWN = false)."); return; }

  // 5. plan
  int bad = buildPlan(mapColor, dir);
  printPlan(dir);
  if (bad) { Serial.printf("%d linija nema prolaza - provjeri kartu ili SEAT_V_MM. NE VOZIM.\n", bad); return; }
  if (!drive) { Serial.println("Samo sken (c) - kraj."); return; }

  // 6. izlaz + voznja
  if (EXIT_RUCNO) {
    waitStart("\nPRENESI robota iz parkinga u traku (smjer voznje, uz parking) i stisni START / 's'.");
    delay(500);
  } else {
    // TODO: ovdje ide exit_parking (robot mora zavrsiti u traci, psi = 0)
    Serial.println("Automatski izlaz iz parkinga jos nije napisan.");
    return;
  }
  t0 = millis();
  driveLaps();
  Serial.printf("Voznja: %.1f s\n", (millis() - t0) / 1000.0);
}

// =====================================================================
//  TESTOVI
// =====================================================================
void testTower() {
  for (int d = 1; d >= -1; d -= 2) {
    Serial.printf("\n%s:\n", d > 0 ? "CCW (unutrasnjost lijevo)" : "CW (unutrasnjost desno)");
    for (int i = 0; i < NUM_POSES; i++) {
      int sd = towerServoDeg(i, d);
      float fb; unsigned long t0 = millis();
      bool ok = towerGo(sd, &fb);
      Serial.printf("  polozaj %d: %3d st. %s -> servo %3d, povratna %.1f, %s, %lu ms\n",
                    i, POSE_DEG[i], d > 0 ? "lijevo" : "desno", sd, fb, ok ? "OK" : "NIJE STIGAO", millis() - t0);
    }
  }
  towerGo(TOWER_FORWARD_DEG, nullptr);
  Serial.println("Kamera mora gledati u unutrasnjost staze; ako gleda van, promijeni TOWER_SIGN.");
}

void planFromText(String s) {         // "m --R--- G----- -R--G- ----R- ccw"
  s = s.substring(1);
  s.trim();
  String low = s; low.toLowerCase();
  int d = (low.endsWith(" cw") || low == "cw") ? -1 : +1;
  int n = 0;
  for (unsigned i = 0; i < s.length() && n < NSEAT; i++) {
    char c = toupper(s[i]);
    if (c == 'R' || c == 'G' || c == '-' || c == '?') mapColor[n++] = c;
    if (c == 'C') break;              // pocetak "ccw"/"cw"
  }
  if (n != NSEAT) { Serial.printf("Treba 24 znaka R/G/-, dobio %d.\n", n); return; }
  for (int i = 0; i < NSEAT; i++) mapH[i] = 0;
  dir = d;
  printMap();
  int bad = buildPlan(mapColor, d);
  printPlan(d);
  if (bad) Serial.printf("%d linija NEMOGUCE.\n", bad);
}

void printHelp() {
  Serial.println("\nNaredbe: c = sken+plan (bez voznje), s = cijeli zadatak, p = ping kamere,");
  Serial.println("         w = test tornja, d = smjer iz ToF-a, m <24 znaka> [cw|ccw] = plan iz karte");
}

void turn_tower_loop() {
  

  if (Serial.available() > 0) {
    int kut = Serial.parseInt();

    // Odbaci ostatak unosa, uključujući Enter
    while (Serial.available() > 0) {
      Serial.read();
    }

    // Provjeri je li kut unutar dopuštenog raspona
    if (kut < 0 || kut > TOWER_MAX_DEG) {
      Serial.printf("Neispravan kut! Unesi 0-%d stupnjeva.\n",
                    TOWER_MAX_DEG);
      return;
    }

    Serial.printf("Okrecem toranj na %d stupnjeva...\n", kut);

    float povratniKut;
    bool stigao = towerGo(kut, &povratniKut);

    if (stigao) {
      Serial.printf("Toranj je stigao na %d stupnjeva.\n", kut);
    } else {
      Serial.printf("Toranj nije stigao! Povratna veza: %.1f stupnjeva.\n",
                    povratniKut);
    }

    printHelp();
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== obstacle_test ===");

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(STBY, OUTPUT);
  ledcAttach(PWMA, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  motorOff();
  ledcAttach(STEER_PIN, SERVO_PWM_FREQ, SERVO_PWM_BITS);
  steerTo(STEER_CENTER);
  ledcAttach(TOWER_PIN, SERVO_PWM_FREQ, SERVO_PWM_BITS);
  analogSetPinAttenuation(FEEDBACK_PIN, ADC_11db);
  towerWrite(TOWER_FORWARD_DEG);

  pinMode(ENC_A, INPUT); pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, RISING);
  pinMode(START_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  if (chaosI2CCheck(false) > 0) halt("provjera I2C nije prosla - NE VOZITI");
  mpuInit();
  if (!initTof(tofRF, VL53L1X::Short, 33000)) halt("desni prednji ToF (kanal 0)");
  if (!initTof(tofRR, VL53L1X::Short, 33000)) halt("desni zadnji ToF (kanal 1)");
  if (!initTof(tofLF, VL53L1X::Short, 33000)) halt("lijevi prednji ToF (kanal 2)");
  if (!initTof(tofF,  VL53L1X::Long,  50000)) halt("prednji ToF (kanal 3)");     // Long: treba do ~2.5 m
  if (!initTof(tofLR, VL53L1X::Short, 33000)) halt("lijevi zadnji ToF (kanal 4)");

  Serial.printf("Toranj: naprijed = servo %d st., TOWER_SIGN %+d\n", TOWER_FORWARD_DEG, TOWER_SIGN);
  checkTowerRange();
  camPing(true);
  if (BEZ_MOTORA) Serial.println("*** BEZ_MOTORA = true ***");
  printHelp();
}

void loop() {
  if (startPressed()) { runObstacle(true); printHelp(); return; }
  //turn_tower_loop();
  if (!Serial.available()) { delay(5); return; }
  int line = Serial.readStringUntil();
  towerGo(line, nullptr);
  line.trim();
  if (line.length() == 0) return;
  char c = tolower(line[0]);
  switch (c) {
    case 's': runObstacle(true); break;
    case 'c': runObstacle(false); break;
    case 'p': camPing(true); break;
    case 'w': testTower(); break;
    case 'd': { int d = detectDirection(); Serial.println(d > 0 ? "-> CCW" : d < 0 ? "-> CW" : "-> NE ZNAM"); break; }
    case 'm': planFromText(line); break; 
    case 't': turn_tower_loop(line); break;
    default: break;
  }
  printHelp();
}
