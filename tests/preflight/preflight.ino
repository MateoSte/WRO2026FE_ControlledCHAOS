// =====================================================================
//  preflight.ino  -  Controlled C.H.A.O.S. v2
//  Provjera cijelog robota prije voznje.
//
//  Potrebno u istoj mapi: ChaosPins.h, ChaosCheck.h
//  Serial Monitor 115200. Na kraju: popis OK / WARN / FAIL.
//  Ugradjena RGB LED (ako je plocica definira): zelena = sve OK,
//  zuta = upozorenja, crvena = ne voziti.
//
//  Koraci:
//    1. I2C: MUX, RTC, MPU, 5 x ToF, kamera            (automatski)
//    2. Toranj: servo + povratna veza na IO6           (automatski, toranj se malo okrene)
//    3. Upravljanje MG995: lijevo - sredina - desno    (gledaj kotace)
//    4. Motor + enkoder: kratki trzaj naprijed         (na 'm'; robot podignut ili slobodan 10 cm)
//    5. START tipkalo: pritisni unutar 5 s
// =====================================================================
#include <Wire.h>
#include "../../includes/includes.ino"
#include "../../includes/ChaosCheck.h"

// ---- isto kao u programima voznje ----
const int   STEER_US_MIN = 500;
const int   STEER_US_MAX = 2500;
const float TOWER_TOL_MV = 150;   // dopusteno odstupanje povratne veze

int fails = 0, warns = 0;
volatile long encCount = 0;

void IRAM_ATTR encISR() {
  if (digitalRead(ENC_B)) encCount++; else encCount--;
}

void result(const char *status, const char *what, const char *note = "") {
  if (!strcmp(status, "FAIL")) fails++;
  if (!strcmp(status, "WARN")) warns++;
  ccLine(status, what, note);
}

void steerTo(float deg) {
  deg = constrain(deg, (float)STEER_LEFT, (float)STEER_RIGHT);
  long us = STEER_US_MIN + (long)(deg * (STEER_US_MAX - STEER_US_MIN) / 180.0);
  ledcWrite(STEER_PIN, us * ((1L << SERVO_PWM_BITS) - 1) / 20000L);
}

void towerTo(float deg) {
  long us = TOWER_MIN_US + (long)(deg * (TOWER_MAX_US - TOWER_MIN_US) / TOWER_MAX_DEG);
  ledcWrite(TOWER_PIN, us * ((1L << SERVO_PWM_BITS) - 1) / 20000L);
}

float feedbackMv() {
  long s = 0;
  for (int i = 0; i < 16; i++) s += analogReadMilliVolts(FEEDBACK_PIN);
  return s / 16.0;
}

// ceka tipku u Serial Monitoru; vraca znak ili 0 nakon isteka
char waitKey(unsigned long ms) {
  while (Serial.available()) Serial.read();
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c != '\r' && c != '\n') return c;
    }
    delay(10);
  }
  return 0;
}

void led(uint8_t r, uint8_t g, uint8_t b) {
#ifdef RGB_BUILTIN
  rgbLedWrite(RGB_BUILTIN, r, g, b);
#endif
}

// ---------------------------------------------------------------------
void testTower() {
  Serial.println("--- Toranj kamere (SER0046) ---");
  towerTo(TOWER_CENTER);
  delay(1000);
  float a = feedbackMv();
  towerTo(TOWER_CENTER - 30);
  delay(800);
  float b = feedbackMv();
  towerTo(TOWER_CENTER);
  delay(500);

  char note[64];
  float expectDiff = -30 * TOWER_MV_PER_DEG;      // ~ -304 mV
  snprintf(note, sizeof(note), "135 st.: %.0f mV (ocek. %.0f), pomak %.0f mV (ocek. %.0f)",
           a, TOWER_MV_CENTER, b - a, expectDiff);

  if (a < 100 || a > 3200)
    result("FAIL", "Povratna veza IO6", "nema napona: smedja zica ili napajanje servo");
  else if (fabs(b - a) < 50)
    result("FAIL", "Toranj se ne mice", "signal IO12 ili napajanje BEC1");
  else if (fabs(a - TOWER_MV_CENTER) > TOWER_TOL_MV || fabs((b - a) - expectDiff) > TOWER_TOL_MV)
    result("WARN", "Toranj radi, kalibracija odstupa", note);
  else
    result("OK", "Toranj + povratna veza", note);
}

void testSteering() {
  Serial.println("--- Upravljanje (MG995) ---");
  Serial.println("  Gledaj kotace: LIJEVO - SREDINA - DESNO - SREDINA");
  steerTo(STEER_LEFT);   delay(700);
  steerTo(STEER_CENTER); delay(700);
  steerTo(STEER_RIGHT);  delay(700);
  steerTo(STEER_CENTER); delay(500);
  Serial.println("  Jesu li se kotaci pomaknuli tim redom? d = da, n = ne (10 s)");
  char c = waitKey(10000);
  if (c == 'd' || c == 'D') result("OK", "Upravljanje MG995");
  else if (c == 'n' || c == 'N') result("FAIL", "Upravljanje MG995", "signal IO11 ili napajanje BEC1");
  else result("WARN", "Upravljanje MG995", "nije potvrdjeno");
}

void testMotor() {
  Serial.println("--- Motor + enkoder ---");
  Serial.println("  Podigni robota (ili 10 cm slobodno ispred). m = test, bilo sto drugo = preskoci (15 s)");
  char c = waitKey(15000);
  if (c != 'm' && c != 'M') { result("WARN", "Motor + enkoder", "preskoceno"); return; }

  encCount = 0;
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, 130);
  delay(300);
  digitalWrite(AIN2, HIGH);                 // kocnica
  ledcWrite(PWMA, 255);
  delay(200);
  ledcWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(STBY, LOW);

  long n = encCount;
  char note[48];
  snprintf(note, sizeof(note), "%ld impulsa naprijed", n);
  if (labs(n) < 20)
    result("FAIL", "Motor ili enkoder", "nema impulsa: M1/M2, STBY, driver ili zice enkodera");
  else if (n < 0)
    result("WARN", "Enkoder broji unatrag", "zamijeni ENC_A i ENC_B (ili AO1/AO2 ako kotac ide unatrag)");
  else
    result("OK", "Motor + enkoder", note);
}

void testButton() {
  Serial.println("--- START tipkalo ---");
  if (digitalRead(START_PIN) == LOW) {
    result("FAIL", "START tipkalo", "stalno pritisnuto: kratki spoj ili krivi kontakti");
    return;
  }
  Serial.println("  Pritisni START unutar 5 s");
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    if (digitalRead(START_PIN) == LOW) { result("OK", "START tipkalo"); return; }
    delay(10);
  }
  result("WARN", "START tipkalo", "nije pritisnuto ili nije spojeno (IO21 + GND)");
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=========== PREFLIGHT  C.H.A.O.S. v2 ===========");
  led(0, 0, 40);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);
  ledcAttach(PWMA, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcWrite(PWMA, 0);
  ledcAttach(STEER_PIN, SERVO_PWM_FREQ, SERVO_PWM_BITS);
  ledcAttach(TOWER_PIN, SERVO_PWM_FREQ, SERVO_PWM_BITS);
  steerTo(STEER_CENTER);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, RISING);
  pinMode(START_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

  // 1. I2C (ChaosCheck.h broji svoje greske)
  fails += chaosI2CCheck(false, &warns);

  // 2-5.
  testTower();
  testSteering();
  testMotor();
  testButton();

  Serial.println("==================================================");
  Serial.printf("REZULTAT: %d FAIL, %d WARN  ->  %s\n", fails, warns,
                fails ? "NE VOZITI" : (warns ? "provjeri upozorenja" : "SPREMAN"));
  Serial.println("Baterija se jos ne mjeri (djelitelj nije zalemljen) - izmjeri multimetrom: >= 11.0 V.");
  Serial.println("RESET za ponovnu provjeru.");

  if (fails) led(60, 0, 0);
  else if (warns) led(50, 30, 0);
  else led(0, 60, 0);
}

void loop() {}
