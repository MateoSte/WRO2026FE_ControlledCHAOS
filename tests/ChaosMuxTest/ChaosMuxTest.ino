// =====================================================================
//  ChaosMuxTest.ino  -  Controlled C.H.A.O.S. v2  (Team Controlled CHAOS)
//  Test svih uredjaja na I2C sabirnici:
//    TCA9548A, MPU-6050, 5x VL53L1X, OpenMV AE3 (samo veza)
//
//  Plocica:   Soldered NULA DeepSleep ESP32-S3, Arduino IDE (core 3.x)
//  Knjiznica: "VL53L1X" by Pololu (Library Manager). MPU bez knjiznice.
//  Potrebno:  ChaosPins.h u istoj mapi. Svi pinovi, adrese i kanali
//             dolaze odande - ovdje se nista od toga ne definira.
//  Serial:    115200 baud
//
//  Naredbe nakon testa (slovo + Enter):
//    a  ponovi cijeli test
//    l  uzivo: 5x ToF + MPU, 10 puta u sekundi
//    m  raspored ToF (dlan redom ispred svakog senzora)
//    g  smjer osi MPU-6050 (zakret, podizanje)
//
//  Motor je cijelo vrijeme ugasen (STBY = LOW). Servo motori se ne mice.
//  Ugradjeni ToF i IMU kamere AE3 ovaj test ne provjerava.
//  Postavke MPU-a i predznaci osi: blok IMU u ChaosPins.h.
// =====================================================================
#include <Wire.h>
#include <VL53L1X.h>
#include "../../includes/includes.ino"

// ---------------------------------------------------------------------
//  ToF senzori: kanal iz ChaosPins.h, ime za ispis, kratica za tablicu
// ---------------------------------------------------------------------
struct Imu { float ax, ay, az, gx, gy, gz, t; };
struct TofDef { uint8_t ch; const char* name; const char* tag; };
const TofDef TOF[] = {
  {MUX_TOF_RIGHT_FRONT, "desno-naprijed",  "DN"},
  {MUX_TOF_RIGHT_REAR,  "desno-straga",    "DS"},
  {MUX_TOF_LEFT_FRONT,  "lijevo-naprijed", "LN"},
  {MUX_TOF_FRONT,       "sprijeda",        "SP"},
  {MUX_TOF_LEFT_REAR,   "lijevo-straga",   "LS"},
};
const int N_TOF = sizeof(TOF) / sizeof(TOF[0]);

// Pragovi testa
const int   TOF_SAMPLES    = 15;
const int   HAND_MM        = 80;     // dlan "ispred senzora"
const int   CLEAR_MM       = 150;    // slobodan prostor za test rasporeda
const float G_TOL          = 0.08f;  // |a| = 1 g +- 8 %
const float GYRO_BIAS_MAX  = 5.0f;   // st/s
const float GYRO_NOISE_MAX = 0.5f;   // st/s (standardna devijacija)
const int   CAM_TRIES      = 20;

// ---------------------------------------------------------------------
//  Stanje
// ---------------------------------------------------------------------
VL53L1X  tof[N_TOF];
bool     tofOk[N_TOF];
uint16_t tofMm[N_TOF];
uint8_t  tofSt[N_TOF];

bool  mpuOk = false;
float gbx = 0, gby = 0, gbz = 0;     // pomak nule ziroskopa

uint8_t mainBus[16];
int     nMain = 0;
uint8_t camChExtra[16];
int     nCamChExtra = 0;

int nPass, nWarn, nFail;

#define T_OK(...)   do { nPass++; Serial.print("[ OK ] "); Serial.printf(__VA_ARGS__); Serial.println(); } while (0)
#define T_WARN(...) do { nWarn++; Serial.print("[UPOZ] "); Serial.printf(__VA_ARGS__); Serial.println(); } while (0)
#define T_FAIL(...) do { nFail++; Serial.print("[GRES] "); Serial.printf(__VA_ARGS__); Serial.println(); } while (0)
#define T_INFO(...) do {          Serial.print("       "); Serial.printf(__VA_ARGS__); Serial.println(); } while (0)

// =====================================================================
//  I2C pomocne funkcije
// =====================================================================
bool probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// ch < 0 iskljucuje sve kanale
bool muxSelect(int ch) {
  Wire.beginTransmission((uint8_t)ADDR_MUX);
  Wire.write(ch < 0 ? (uint8_t)0x00 : (uint8_t)(1 << ch));
  return Wire.endTransmission() == 0;
}

// TCA9548A pri citanju vraca kontrolni registar (koji su kanali ukljuceni)
int muxReadControl() {
  if (Wire.requestFrom((uint8_t)ADDR_MUX, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

int scanBus(uint8_t* out, int maxN) {
  int n = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    if (probe(a) && n < maxN) out[n++] = a;
  }
  return n;
}

bool contains(const uint8_t* list, int n, uint8_t a) {
  for (int i = 0; i < n; i++) if (list[i] == a) return true;
  return false;
}

String hexList(const uint8_t* list, int n) {
  if (n == 0) return "(nista)";
  String s;
  char buf[8];
  for (int i = 0; i < n; i++) {
    snprintf(buf, sizeof(buf), "%s0x%02X", i ? ", " : "", list[i]);
    s += buf;
  }
  return s;
}

void flushInput() {
  delay(20);
  while (Serial.available()) Serial.read();
}

// =====================================================================
//  MPU-6050 (izravno preko registara)
// =====================================================================
// Postavke i predznaci osi dolaze iz ChaosPins.h (IMU_*).
// Imu vraca sirove osi cipa; predznaci se primjenjuju tek pri tumacenju.

bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission((uint8_t)ADDR_MPU);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission((uint8_t)ADDR_MPU);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADDR_MPU, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool mpuInit() {
  bool ok = mpuWrite(0x6B, 0x01)                      // probudi, takt iz ziroskopa X
         && mpuWrite(0x1A, IMU_DLPF_CFG)              // niskopropusni filtar
         && mpuWrite(0x1B, IMU_GYRO_FS_SEL << 3)      // raspon ziroskopa
         && mpuWrite(0x1C, IMU_ACCEL_FS_SEL << 3);    // raspon akcelerometra
  delay(100);
  return ok;
}

bool mpuReadAll(Imu& m) {
  uint8_t b[14];
  if (!mpuRead(0x3B, b, 14)) return false;
  auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  m.ax = s16(0)  / IMU_ACCEL_LSB_PER_G;
  m.ay = s16(2)  / IMU_ACCEL_LSB_PER_G;
  m.az = s16(4)  / IMU_ACCEL_LSB_PER_G;
  m.t  = s16(6)  / 340.0f + 36.53f;
  m.gx = s16(8)  / IMU_GYRO_LSB_PER_DPS;
  m.gy = s16(10) / IMU_GYRO_LSB_PER_DPS;
  m.gz = s16(12) / IMU_GYRO_LSB_PER_DPS;
  return true;
}

// =====================================================================
//  VL53L1X
// =====================================================================
bool tofInit(int i) {
  if (!muxSelect(TOF[i].ch)) return false;
  tof[i].setBus(&Wire);
  tof[i].setTimeout(500);
  if (!tof[i].init()) return false;            // init() provjerava i ID modela
  tof[i].setDistanceMode(VL53L1X::Long);       // do ~4 m
  tof[i].setMeasurementTimingBudget(50000);    // 50 ms
  tof[i].startContinuous(50);
  return true;
}

// Neblokirajuce citanje: true ako je stiglo novo mjerenje
bool tofUpdate(int i) {
  if (!tofOk[i]) return false;
  muxSelect(TOF[i].ch);
  if (!tof[i].dataReady()) return false;
  tof[i].read(false);
  tofMm[i] = tof[i].ranging_data.range_mm;
  tofSt[i] = tof[i].ranging_data.range_status;
  return true;
}

bool tofValid(uint8_t st) {
  return st == VL53L1X::RangeValid || st == VL53L1X::RangeValidMinRangeClipped;
}

const char* tofStatusText(uint8_t st) {
  return VL53L1X::rangeStatusToString((VL53L1X::RangeStatus)st);
}

void sortU16(uint16_t* a, int n) {
  for (int i = 1; i < n; i++) {
    uint16_t v = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
}

// =====================================================================
//  Sazetak
// =====================================================================
void summary() {
  Serial.println("\n----------------------------------------");
  Serial.printf("UKUPNO: %d OK, %d upozorenja, %d gresaka\n", nPass, nWarn, nFail);
  Serial.println("----------------------------------------");
}

// =====================================================================
//  1. MULTIPLEKSER
// =====================================================================
void testMux() {
  Serial.println("\n--- 1. Multiplekser TCA9548A ---");
  if (!probe(ADDR_MUX)) {
    T_FAIL("MUX se ne javlja na 0x%02X. Provjeri Qwiic kabel ESP32 -> MUX i Wire.begin(%d, %d).",
           ADDR_MUX, I2C_SDA, I2C_SCL);
    return;
  }
  muxSelect(-1);
  int ctl = muxReadControl();
  if (ctl == 0x00) T_OK("MUX 0x%02X se javlja, svi kanali iskljuceni.", ADDR_MUX);
  else             T_FAIL("MUX vraca kontrolni registar 0x%02X umjesto 0x00.", ctl);

  muxSelect(MUX_TOF_FRONT);
  ctl = muxReadControl();
  muxSelect(-1);
  if (ctl == (1 << MUX_TOF_FRONT)) T_OK("Odabir kanala radi (kanal %d -> registar 0x%02X).", MUX_TOF_FRONT, ctl);
  else T_FAIL("Odabir kanala ne radi: ocekivano 0x%02X, procitano 0x%02X.", 1 << MUX_TOF_FRONT, ctl);
}

// =====================================================================
//  2. GLAVNA SABIRNICA
// =====================================================================
void testMainBus() {
  Serial.println("\n--- 2. Glavna sabirnica (svi kanali MUX-a iskljuceni) ---");
  muxSelect(-1);
  nMain = scanBus(mainBus, 16);
  T_INFO("Pronadjeno: %s", hexList(mainBus, nMain).c_str());

  if (contains(mainBus, nMain, ADDR_MPU)) T_OK("MPU-6050 0x%02X je na glavnoj sabirnici.", ADDR_MPU);
  else T_FAIL("MPU-6050 (0x%02X) nije na glavnoj sabirnici. Provjeri Qwiic adapter i pass-through konektor MUX-a.", ADDR_MPU);

  if (contains(mainBus, nMain, ADDR_RTC)) T_OK("RTC na NULA plocici 0x%02X.", ADDR_RTC);
  else T_WARN("RTC 0x%02X se ne javlja (17. 11. je bio tu). Robotu ne treba, ali je cudno.", ADDR_RTC);

  for (int i = 0; i < nMain; i++) {
    uint8_t a = mainBus[i];
    if (a == ADDR_MUX || a == ADDR_MPU || a == ADDR_RTC) continue;
    if (a == ADDR_TOF)
      T_WARN("0x%02X je na glavnoj sabirnici: neki ToF je spojen na pass-through umjesto na kanal.", ADDR_TOF);
    else
      T_WARN("Neocekivana adresa 0x%02X na glavnoj sabirnici.", a);
  }
}

// =====================================================================
//  3. KANALI MUX-a
// =====================================================================
void testChannels() {
  Serial.println("\n--- 3. Kanali MUX-a 0-7 (adrese s glavne sabirnice izostavljene) ---");
  nCamChExtra = 0;
  for (int ch = 0; ch < 8; ch++) {
    muxSelect(ch);
    delay(2);
    uint8_t found[16];
    int n = scanBus(found, 16);
    uint8_t own[16];
    int nOwn = 0;
    for (int i = 0; i < n; i++)
      if (!contains(mainBus, nMain, found[i])) own[nOwn++] = found[i];

    int t = -1;
    for (int i = 0; i < N_TOF; i++) if (TOF[i].ch == ch) t = i;

    if (t >= 0) {
      if (contains(own, nOwn, ADDR_TOF))
        T_OK("Kanal %d (%s): VL53L1X 0x%02X.", ch, TOF[t].name, ADDR_TOF);
      else
        T_FAIL("Kanal %d (%s): nema VL53L1X. Vidim: %s", ch, TOF[t].name, hexList(own, nOwn).c_str());
      for (int i = 0; i < nOwn; i++)
        if (own[i] != ADDR_TOF) T_WARN("Kanal %d: neocekivana adresa 0x%02X.", ch, own[i]);
    } else if (ch == MUX_CAMERA) {
      T_INFO("Kanal %d (kamera): %s - kamera se provjerava u koraku 6.", ch, hexList(own, nOwn).c_str());
      for (int i = 0; i < nOwn; i++)
        if (own[i] != ADDR_AE3 && nCamChExtra < 16) camChExtra[nCamChExtra++] = own[i];
    } else {
      if (nOwn == 0) T_INFO("Kanal %d: prazan (ocekivano).", ch);
      else T_WARN("Kanal %d bi trebao biti prazan, a vidim: %s", ch, hexList(own, nOwn).c_str());
    }
  }
  muxSelect(-1);
}

// =====================================================================
//  4. MPU-6050
//  Ocekivano (cip gore, strelica X naprijed): Z gore, X naprijed, Y lijevo.
//  Nagibi: nos gore = +, lijeva strana gore = +
// =====================================================================
void testMpu() {
  Serial.println("\n--- 4. MPU-6050 (robot miruje, ravno) ---");
  mpuOk = false;
  muxSelect(-1);

  uint8_t who = 0;
  if (!mpuRead(0x75, &who, 1)) { T_FAIL("Ne mogu procitati WHO_AM_I."); return; }
  if (who == 0x68)      T_OK("WHO_AM_I = 0x68 (MPU-6050).");
  else if (who == 0x70) T_WARN("WHO_AM_I = 0x70: to je MPU-6500, ne MPU-6050. Za ovaj test radi isto.");
  else                  T_WARN("WHO_AM_I = 0x%02X: klon ili drugi cip. Za ovaj test radi isto.", who);

  if (!mpuInit()) { T_FAIL("Ne mogu upisati postavke u MPU."); return; }

  T_INFO("Ne diraj robota 3 s...");
  delay(1500);

  T_INFO("Postavke: ziroskop +-%d st/s, akcelerometar +-%d g, DLPF %d.",
         250 << IMU_GYRO_FS_SEL, 2 << IMU_ACCEL_FS_SEL, IMU_DLPF_CFG);

  const int N = IMU_CAL_SAMPLES;
  double sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;
  double qgx = 0, qgy = 0, qgz = 0, st = 0;
  int got = 0;
  for (int i = 0; i < N; i++) {
    Imu m;
    if (mpuReadAll(m)) {
      sax += m.ax; say += m.ay; saz += m.az;
      sgx += m.gx; sgy += m.gy; sgz += m.gz;
      qgx += m.gx * m.gx; qgy += m.gy * m.gy; qgz += m.gz * m.gz;
      st += m.t;
      got++;
    }
    delay(5);
  }
  if (got < N * 95 / 100) { T_FAIL("Citanje MPU-a nepouzdano: %d/%d uspjelih.", got, N); return; }
  mpuOk = true;

  float ax = sax / got, ay = say / got, az = saz / got;
  gbx = sgx / got; gby = sgy / got; gbz = sgz / got;
  float nx = sqrtf(fmaxf(0.0f, (float)(qgx / got) - gbx * gbx));
  float ny = sqrtf(fmaxf(0.0f, (float)(qgy / got) - gby * gby));
  float nz = sqrtf(fmaxf(0.0f, (float)(qgz / got) - gbz * gbz));
  float g  = sqrtf(ax * ax + ay * ay + az * az);

  T_INFO("Akcelerometar:  ax %+.3f  ay %+.3f  az %+.3f g   |a| = %.3f g", ax, ay, az, g);
  T_INFO("Ziroskop nula:  gx %+.2f  gy %+.2f  gz %+.2f st/s", gbx, gby, gbz);
  T_INFO("Ziroskop sum:   gx %.3f  gy %.3f  gz %.3f st/s", nx, ny, nz);
  T_INFO("Temperatura cipa: %.1f C", st / got);

  if (fabsf(g - 1.0f) <= G_TOL) T_OK("Akcelerometar mjeri 1 g (%.3f).", g);
  else T_WARN("|a| = %.3f g, ocekivano 1.00 +- %.2f. Robot se micao ili akcelerometar nije dobar.", g, G_TOL);

  if (az > 0.95f * g) {
    float pitch = IMU_PITCH_SIGN * atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;  // nos gore = +
    float roll  = IMU_ROLL_SIGN  * atan2f(ay, az) * 180.0f / PI;                        // lijevo gore = +
    T_OK("Os Z gleda gore (cip gore). Nagib: nos %+.1f st, lijeva strana %+.1f st.", pitch, roll);
  } else {
    T_WARN("Os Z ne gleda gore (az = %+.2f g). MPU je nagnut ili naopako - provjeri nosac.", az);
  }

  float bmax = fmaxf(fabsf(gbx), fmaxf(fabsf(gby), fabsf(gbz)));
  if (bmax <= GYRO_BIAS_MAX) T_OK("Pomak nule ziroskopa do %.2f st/s.", bmax);
  else T_WARN("Pomak nule ziroskopa %.2f st/s. Nije kvar, ali ga kod mora oduzimati (kalibracija na startu).", bmax);

  float nmax = fmaxf(nx, fmaxf(ny, nz));
  if (nmax <= GYRO_NOISE_MAX) T_OK("Sum ziroskopa do %.3f st/s.", nmax);
  else T_WARN("Sum ziroskopa %.3f st/s je velik: robot se micao, vibracije ili smetnje u napajanju.", nmax);
}

// =====================================================================
//  5. VL53L1X x5
// =====================================================================
void testTof() {
  Serial.println("\n--- 5. VL53L1X x5 (init + 15 mjerenja svaki) ---");
  for (int i = 0; i < N_TOF; i++) {
    tofOk[i] = tofInit(i);
    if (!tofOk[i]) T_FAIL("%s (kanal %d): init() nije uspio.", TOF[i].name, TOF[i].ch);
  }
  delay(200);

  for (int i = 0; i < N_TOF; i++) {
    if (!tofOk[i]) continue;
    muxSelect(TOF[i].ch);
    uint16_t v[TOF_SAMPLES];
    int nv = 0, timeouts = 0;
    int stCount[16] = {0};
    for (int k = 0; k < TOF_SAMPLES; k++) {
      tof[i].read(true);
      if (tof[i].timeoutOccurred()) { timeouts++; continue; }
      uint8_t s = tof[i].ranging_data.range_status;
      if (s < 16) stCount[s]++;
      if (tofValid(s)) v[nv++] = tof[i].ranging_data.range_mm;
    }
    if (timeouts > 0) {
      T_FAIL("%s (kanal %d): %d/%d mjerenja istekla (timeout).", TOF[i].name, TOF[i].ch, timeouts, TOF_SAMPLES);
      continue;
    }
    if (nv >= TOF_SAMPLES - 3) {
      sortU16(v, nv);
      T_OK("%-16s kanal %d: medijan %4u mm (min %u, max %u), %d/%d valjanih.",
           TOF[i].name, TOF[i].ch, v[nv / 2], v[0], v[nv - 1], nv, TOF_SAMPLES);
    } else {
      int worst = -1;
      for (int s = 0; s < 16; s++)
        if (!tofValid(s) && stCount[s] > 0 && (worst < 0 || stCount[s] > stCount[worst])) worst = s;
      T_WARN("%-16s kanal %d: samo %d/%d valjanih (najcesce: %s). Nema predmeta u dometu ili je staklo zaklonjeno?",
             TOF[i].name, TOF[i].ch, nv, TOF_SAMPLES, worst < 0 ? "nepoznato" : tofStatusText(worst));
    }
  }
  muxSelect(-1);
}

// =====================================================================
//  6. KAMERA AE3 - samo veza na MUX_CAMERA
//  Javlja se samo dok na kameri radi MicroPython skripta (I2C target).
// =====================================================================
void testCamera() {
  Serial.println("\n--- 6. OpenMV AE3 (samo veza) ---");
  muxSelect(MUX_CAMERA);
  int ack = 0;
  for (int i = 0; i < CAM_TRIES; i++) {
    if (probe(ADDR_AE3)) ack++;
    delay(100);
  }
  muxSelect(-1);

  if (ack == CAM_TRIES)
    T_OK("AE3 se javlja na 0x%02X, kanal %d (%d/%d).", ADDR_AE3, MUX_CAMERA, ack, CAM_TRIES);
  else if (ack > 0)
    T_WARN("AE3 se javlja povremeno (%d/%d). Skripta na kameri slusa samo dio vremena, ili je kabel los.", ack, CAM_TRIES);
  else
    T_WARN("AE3 se ne javlja na 0x%02X, kanal %d. Je li kabel spojen i radi li na kameri skripta (I2C target)?",
           ADDR_AE3, MUX_CAMERA);

  if (nCamChExtra > 0)
    T_INFO("Na kanalu kamere su jos: %s. Ovaj test ih ne provjerava.", hexList(camChExtra, nCamChExtra).c_str());
}

// =====================================================================
//  CIJELI TEST
// =====================================================================
void runAll() {
  nPass = nWarn = nFail = 0;
  Serial.println("\n========== C.H.A.O.S. v2 - test I2C senzora ==========");
  Serial.println("Robot neka stoji mirno na ravnoj podlozi, s nesto slobodnog");
  Serial.println("prostora (20 cm - 1 m) ispred svakog ToF senzora.");

  testMux();
  if (nFail > 0) { summary(); return; }   // bez MUX-a nema smisla dalje
  testMainBus();
  testChannels();
  testMpu();
  testTof();
  testCamera();
  muxSelect(-1);
  summary();
}

// =====================================================================
//  l - UZIVO
// =====================================================================
void liveMode() {
  Serial.println("\nUzivo, 10 puta u sekundi. Bilo koja tipka + Enter = izlaz.");
  Serial.println("Udaljenost u mm; '*' = mjerenje nije valjano. gz bez pomaka nule.");
  flushInput();
  int line = 0;
  while (!Serial.available()) {
    for (int i = 0; i < N_TOF; i++) tofUpdate(i);
    muxSelect(-1);
    Imu m = {};
    bool imu = mpuOk && mpuReadAll(m);

    if (line++ % 20 == 0) {
      for (int i = 0; i < N_TOF; i++) Serial.printf("   %s ", TOF[i].tag);
      Serial.println("|    ax     ay     az |    gz");
    }
    for (int i = 0; i < N_TOF; i++) {
      if (!tofOk[i]) Serial.print("   -- ");
      else Serial.printf("%5u%c", tofMm[i], tofValid(tofSt[i]) ? ' ' : '*');
    }
    if (imu) Serial.printf("| %+.2f  %+.2f  %+.2f | %+6.1f\n", m.ax, m.ay, m.az, IMU_YAW_SIGN * (m.gz - gbz));
    else     Serial.println("|   MPU nije spreman (pokreni 'a')");
    delay(100);
  }
  flushInput();
}

// =====================================================================
//  m - RASPORED ToF: dlan ispred svakog senzora redom
// =====================================================================
void mappingMode() {
  nPass = nWarn = nFail = 0;
  Serial.println("\nProvjera rasporeda ToF senzora.");
  Serial.printf("Ispred svih senzora neka bude barem %d cm slobodno.\n", CLEAR_MM / 10);

  // Senzor koji vec vidi nesto blizu ne moze sudjelovati
  bool blocked[N_TOF] = {false};
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    for (int i = 0; i < N_TOF; i++)
      if (tofUpdate(i) && tofValid(tofSt[i]) && tofMm[i] < CLEAR_MM) blocked[i] = true;
  }
  for (int i = 0; i < N_TOF; i++) {
    if (!tofOk[i]) T_WARN("%s: nije pokrenut, preskacem (pokreni 'a').", TOF[i].name);
    else if (blocked[i]) T_WARN("%s vec vidi nesto blize od %d cm, preskacem. Odmakni robota i ponovi.",
                                TOF[i].name, CLEAR_MM / 10);
  }

  for (int k = 0; k < N_TOF; k++) {
    if (!tofOk[k] || blocked[k]) continue;
    Serial.printf("\n>> Prinesi dlan ~5 cm ispred senzora: %s. Imas 15 s.\n", TOF[k].name);
    int hits[N_TOF] = {0};
    int found = -1;
    t0 = millis();
    while (millis() - t0 < 15000 && found < 0) {
      for (int i = 0; i < N_TOF && found < 0; i++) {
        if (blocked[i] || !tofUpdate(i)) continue;
        if (tofValid(tofSt[i]) && tofMm[i] < HAND_MM) { if (++hits[i] >= 3) found = i; }
        else hits[i] = 0;
      }
      delay(5);
    }
    if (found < 0)
      T_WARN("Nijedan senzor nije vidio dlan.");
    else if (found == k)
      T_OK("%s je na kanalu %d, kako pise u ChaosPins.h.", TOF[k].name, TOF[k].ch);
    else
      T_FAIL("Dlan ispred \"%s\" vidio je kanal %d (u ChaosPins.h: %s). Kabeli su u krivim kanalima ili je ChaosPins.h kriv.",
             TOF[k].name, TOF[found].ch, TOF[found].name);
    Serial.println("   Makni dlan.");
    delay(2000);
  }
  muxSelect(-1);
  summary();
}

// =====================================================================
//  g - SMJER OSI MPU-6050
//    zakret ulijevo      -> gz > 0
//    nos gore            -> ax > 0
//    lijeva strana gore  -> ay > 0
// =====================================================================
float waitAxis(char axis, float limit, uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    Imu m;
    if (mpuReadAll(m)) {
      float v = (axis == 'x') ? m.ax : m.ay;
      if (fabsf(v) >= limit) return v;
    }
    delay(10);
  }
  return 0;
}

// raw = sirova vrijednost osi cipa za pokret koji bi u osima robota trebao biti +
void checkSign(const char* define, const char* axis, float raw, float limit, int sign) {
  if (fabsf(raw) < limit) {
    T_WARN("%s: pokret nije izmjeren (%s = %+.2f). Ponovi 'g'.", define, axis, raw);
    return;
  }
  int measured = raw > 0 ? 1 : -1;
  if (measured == sign)
    T_OK("%s = %+d je tocan (%s = %+.2f).", define, sign, axis, raw);
  else
    T_FAIL("%s je %+d, a izmjereno je %+d (%s = %+.2f). U ChaosPins.h postavi %s na %d.",
           define, sign, measured, axis, raw, define, measured);
}

void axisMode() {
  nPass = nWarn = nFail = 0;
  if (!mpuOk) { Serial.println("\nMPU nije spreman. Pokreni prvo 'a'."); return; }
  muxSelect(-1);

  Serial.println("\n>> Robot ravno. Okreni ga ULIJEVO (odozgo: suprotno od kazaljke) za ~90 st. Imas 10 s.");
  delay(1000);
  float yaw = 0;
  uint32_t last = micros(), t0 = millis();
  while (millis() - t0 < 10000 && fabsf(yaw) < 60) {
    Imu m;
    if (mpuReadAll(m)) {
      uint32_t now = micros();
      yaw += (m.gz - gbz) * (now - last) * 1e-6f;
      last = now;
    }
    delay(5);
  }
  checkSign("IMU_YAW_SIGN", "gz", yaw, 60.0f, IMU_YAW_SIGN);

  Serial.println("\n>> Podigni PREDNJI kraj robota za ~30 st i drzi. Imas 10 s.");
  float v = waitAxis('x', 0.35f, 10000);
  checkSign("IMU_PITCH_SIGN", "ax", v, 0.35f, IMU_PITCH_SIGN);

  Serial.println("\n>> Spusti ga. Sad podigni LIJEVU stranu robota za ~30 st i drzi. Imas 10 s.");
  delay(1500);
  v = waitAxis('y', 0.35f, 10000);
  checkSign("IMU_ROLL_SIGN", "ay", v, 0.35f, IMU_ROLL_SIGN);

  summary();
}

// =====================================================================
void printMenu() {
  Serial.println("\nNaredbe:  a = cijeli test   l = uzivo   m = raspored ToF   g = osi MPU");
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  delay(500);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);   // motor ugasen tijekom testa

  if (!Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ)) Serial.println("Wire.begin() nije uspio!");
  Wire.setTimeOut(50);

  for (int i = 0; i < N_TOF; i++) { tofOk[i] = false; tofMm[i] = 0; tofSt[i] = 255; }

  runAll();
  printMenu();
}

void loop() {
  if (!Serial.available()) return;
  char c = tolower(Serial.read());
  if (c == '\r' || c == '\n' || c == ' ') return;
  switch (c) {
    case 'a': runAll();      break;
    case 'l': liveMode();    break;
    case 'm': mappingMode(); break;
    case 'g': axisMode();    break;
    default:  Serial.printf("Nepoznata naredba '%c'.\n", c); break;
  }
  flushInput();
  printMenu();
}
