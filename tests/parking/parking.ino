// =====================================================================
//  obstacles.ino  -  Controlled C.H.A.O.S. v2  -  WRO 2026 FE Obstacle Challenge
//  Osnovne funkcije: scanTof(), steering(), driveMm().
//
//  Mjesto:   obstacle_challange/obstacles/obstacles.ino
//  Potrebno: classes/classes.ino (preko preambule), biblioteka "VL53L1X" by Pololu.
// =====================================================================

// Biblioteke PRIJE preambule: makroi iz includes.ino (AIN1, STBY, I2C_SDA...)
// inace mogu pokvariti imena unutar tudjih .h datoteka.
#include <Wire.h>
#include <VL53L1X.h>

// #####################################################################
// ###  POCETAK PREAMBULE  -  kopija includes/includes.ino            ###
// ###  (repo MateoSte/WRO2026FE_ControlledCHAOS, commit 93da3f4,     ###
// ###   22. 11. 2026.)  Mijenja se SAMO u includes.ino - ovo je kopija ###
// #####################################################################
#ifndef FE_INCLUDES
#define FE_INCLUDES
#include <Arduino.h>
#include "../../classes/classes.ino"   // PROMIJENJENO u kopiji: putanja za obstacle_challange/obstacles/ (original: "../classes/classes.ino")
using namespace pybricks;
using namespace hubs;
using namespace pupdevices;
using namespace tools;
using namespace parameters;

const int PIN_PWMA = 7, PIN_AIN1 = 4, PIN_AIN2 = 5, PIN_STBY = 10;
const int PIN_ENC_A = 13, PIN_ENC_B = 14;
const int PIN_SRV_STEER = 11, PIN_SRV_TOWER = 12;
const int PIN_BTN = 21;
const int PIN_VBAT = 1;
const int PIN_I2C_SDA = 8, PIN_I2C_SCL = 9;   // fill in the numbers from phase C, step 3
const uint8_t MUX_ADDR = 0x70;

// =====================================================================
//  ChaosPins.h  -  Controlled C.H.A.O.S. v2  (Team Controlled CHAOS)
//  WRO Future Engineers 2026
//
//  Plocica:  Soldered NULA DeepSleep ESP32-S3, Arduino IDE (core 3.x)
//  Stanje:   18. 11. 2026. (nakon i2c_scan, kalibracije MG995 i tornja)
//
//  Uporaba:  #include "ChaosPins.h"   (datoteka u istoj mapi kao .ino)
//  Pravilo:  pin se mijenja SAMO ovdje i u listi spojeva (Connection list).
// =====================================================================

// ---------------------------------------------------------------------
//  NAPAJANJE (samo za znanje, nisu pinovi)
// ---------------------------------------------------------------------
//  3S Li-ion -> osigurac 5 A -> prekidac -> +BAT
//  BEC1 5 V/5 A  -> servo motori (MG995, SER0046)
//  BEC2 5 V/3 A  -> dioda 1N5408 -> ESP32 VCC  (NIKAD BAT/JST na plocici!)
//  ESP32 3V3     -> logika drivera (TB6612 VCC) i enkoder

// ---------------------------------------------------------------------
//  POGONSKI MOTOR  -  TB6612FNG, kanal A, N20 12 V 1:150
// ---------------------------------------------------------------------
#define PWMA            7     // brzina, PWM
#define AIN1            5     // smjer
#define AIN2            4     // smjer
#define STBY           10     // HIGH = driver ukljucen

#define MOTOR_PWM_FREQ  20000 // Hz (izvan cujnog podrucja)
#define MOTOR_PWM_BITS  8     // 0-255
#define MOTOR_MIN_PWM   0     // TODO: izmjeriti mrtvu zonu (najmanja brzina kad se kotac okrece)

// ---------------------------------------------------------------------
//  ENKODER  -  N20 Hall, napajanje 3.3 V
// ---------------------------------------------------------------------
#define ENC_A          13     // interrupt, RISING
#define ENC_B          14     // smjer

#define ENC_PULSES_PER_OUTPUT_REV  1050   // 7 x 150, rastuci bridovi kanala A
#define ENC_PULSES_PER_WHEEL_REV   1050   // TODO: provjeriti rucno (1 okret kotaca)
#define WHEEL_CIRCUMFERENCE_MM     (PI*62.4f) // pi x 43.2 mm
// TODO: provjeriti smjer brojanja (naprijed = brojac raste); ako pada, zamijeni ENC_A i ENC_B

// ---------------------------------------------------------------------
//  UPRAVLJANJE  -  MG995 (BEC1, signal 50 Hz)
// ---------------------------------------------------------------------
#define STEER_PIN      11
#define STEER_CENTER   73     // ravno (izmjereno)
#define STEER_LEFT     55     // krajnje lijevo - NIKAD manje
#define STEER_RIGHT    90     // krajnje desno - NIKAD vise

// ---------------------------------------------------------------------
//  TORANJ KAMERE  -  DFRobot SER0046, 270 st., analogna povratna veza
// ---------------------------------------------------------------------
#define TOWER_PIN      12     // bijela zica, signal
#define FEEDBACK_PIN    6     // smeda zica, ADC1 (premjesteno s IO8 17. 11. 2026.)

#define TOWER_MAX_DEG     270
#define TOWER_CENTER      135      // kamera gleda naprijed (doraditi nakon montaze kamere)
#define TOWER_MIN_US      500      // 0 st.
#define TOWER_MAX_US      2500      // 270 st.
#define TOWER_MV_CENTER   1517.0f   // povratni napon na 135 st. (izmjereno)
#define TOWER_MV_PER_DEG  10.14f  // linearno (izmjereno 90-270 st.)
#define TOWER_CENTER      81

#define SERVO_PWM_FREQ  50    // Hz, oba servo motora
#define SERVO_PWM_BITS  14    // 0-16383

// ---------------------------------------------------------------------
//  START TIPKALO  (spaja se na kraju sklapanja)
// ---------------------------------------------------------------------
#define START_PIN      21     // INPUT_PULLUP, drugi kontakt na GND; pritisnuto = LOW

// ---------------------------------------------------------------------
//  I2C  -  Qwiic (potvrdjeno i2c_scan 17. 11. 2026.)
// ---------------------------------------------------------------------
#define I2C_SDA         8
#define I2C_SCL         9
#define I2C_FREQ   100000     // Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

// Adrese na glavnoj sabirnici
#define ADDR_MUX     0x70     // TCA9548A
#define ADDR_MPU     0x68     // MPU-6050 (pass-through Qwiic na MUX-u)
#define ADDR_RTC     0x51     // RTC na NULA plocici

// Adrese iza MUX-a
#define ADDR_TOF     0x29     // VL53L1X (svi isti, zato MUX)
#define ADDR_AE3     0x12     // OpenMV AE3 (samo dok radi MicroPython skripta)

// Kanali multipleksera
#define MUX_TOF_RIGHT_FRONT  0
#define MUX_TOF_RIGHT_REAR   1
#define MUX_TOF_LEFT_FRONT   2
#define MUX_TOF_FRONT        3
#define MUX_TOF_LEFT_REAR    4
#define MUX_CAMERA           5
// kanali 6 i 7 slobodni

// ---------------------------------------------------------------------
//  BATERIJA  -  djelitelj 100 k / 22 k  (JOS NIJE ZALEMLJEN)
// ---------------------------------------------------------------------
#define BAT_PIN        -1     // TODO: IO1 ili IO2 (jedini slobodni ADC1); -1 = nije spojeno
#define BAT_DIVIDER     5.545f // (100 + 22) / 22, uskladiti s multimetrom
#define BAT_STOP_V      9.6f   // 3.2 V po celiji - ispod toga robot staje
#define BAT_FULL_V     12.6f

// ---------------------------------------------------------------------
//  NE KORISTITI
// ---------------------------------------------------------------------
//  IO3        - strapping pin
//  IO19, IO20 - USB
//  IO8, IO9   - samo I2C
//  IO47       - slobodan (rezerva)

// ---------------------------------------------------------------------
//  IMU  -  MPU-6050 na glavnoj sabirnici (ADDR_MPU, pass-through MUX-a)
// ---------------------------------------------------------------------
//  Montaza: ravno, cip gore, strelica X naprijed.
//  Osi robota: X naprijed, Y lijevo, Z gore (desni koordinatni sustav).
#define IMU_GYRO_FS_SEL      1    // 0=250, 1=500, 2=1000, 3=2000 st/s (250 je premalo za brze zavoje)
#define IMU_ACCEL_FS_SEL     0    // 0=2, 1=4, 2=8, 3=16 g
#define IMU_DLPF_CFG         3    // niskopropusni filtar ~44 Hz
#define IMU_GYRO_LSB_PER_DPS  (131.0f   / (1 << IMU_GYRO_FS_SEL))    // 65.5 za 500 st/s
#define IMU_ACCEL_LSB_PER_G   (16384.0f / (1 << IMU_ACCEL_FS_SEL))   // 16384 za 2 g

// Predznaci osi (+1 ili -1)
// TODO: potvrditi naredbom 'g' u ChaosMuxTest; ako test kaze drukcije, promijeni ovdje
#define IMU_YAW_SIGN         1    // gz: zakret ulijevo = +
#define IMU_PITCH_SIGN       1    // ax: nos gore = +
#define IMU_ROLL_SIGN        1    // ay: lijeva strana gore = +

#define IMU_CAL_SAMPLES    300    // uzoraka za pomak nule ziroskopa (robot miruje, ~1.5 s)

Motor drive(PWMA, AIN1, AIN2, STBY, ENC_A, ENC_B, 7.0f, 150.0f, Direction::COUNTERCLOCKWISE);
Servo steer(PIN_SRV_STEER, 180.0f, 500, 2500, STEER_LEFT, STEER_RIGHT);
CarDriveBase car(drive, steer, 62.4f, 115.0f, 170.0f, STEER_CENTER, std::make_shared<PrimeHub>());

#endif

// #####################################################################
// ###  KRAJ PREAMBULE                                               ###
// #####################################################################

// =====================================================================
//  OSNOVNE FUNKCIJE (bez CarDriveBase)
//    scanTof(RF | RR | LF | LR | F)  -> udaljenost u mm (-1 = nema cilja)
//    steering(deg)                   -> kotaci: 0 = ravno, - = lijevo, + = desno
//    driveMm(mm)                     -> + naprijed, - natrag
//  Koriste se 'drive' (Motor) i 'steer' (Servo) iz preambule; 'car' se ne koristi.
// =====================================================================

// ---------------- POSTAVKE ----------------
const int   DRIVE_SPEED     = 100;   // drive.run(): 0..255
const int   SLOW_SPEED      = 50;    // zadnjih SLOW_ZONE_MM vozi sporije (manje prelijetanja)
const float SLOW_ZONE_MM    = 15.0;
const int   DRIVE_SIGN      = 1;     // ako driveMm(+20) vozi UNATRAG -> stavi -1
const unsigned long DRIVE_TIMEOUT_MS = 5000;
const int   STEER_MS_PER_DEG = 4;    // cekanje da servo stigne (MG995 ~0.2 s / 60 st.)
const int   TOF_SAMPLES     = 5;     // koliko valjanih mjerenja za medijan
const unsigned long TOF_TIMEOUT_MS = 400;

// mm po stupnju izlazne osovine motora (enkoder -> kotac). PROVJERITI: driveMm(1000) mora biti 1 m.
const float MM_PER_DEG = WHEEL_CIRCUMFERENCE_MM / 360.0f
                       * (float)ENC_PULSES_PER_OUTPUT_REV / ENC_PULSES_PER_WHEEL_REV;

// ToF senzori: vrijednost = kanal multipleksera
enum TofId : uint8_t {
  RF = MUX_TOF_RIGHT_FRONT,   // 0
  RR = MUX_TOF_RIGHT_REAR,    // 1
  LF = MUX_TOF_LEFT_FRONT,    // 2
  F  = MUX_TOF_FRONT,         // 3
  LR = MUX_TOF_LEFT_REAR      // 4
};
VL53L1X tof[5];               // indeks = kanal MUX-a

// ---------------------------------------------------------------------
//  1) SKENIRANJE: medijan TOF_SAMPLES valjanih mjerenja zadanog senzora.
//     Vraca mm, ili -1 ako nema valjanog cilja (predaleko / greska).
// ---------------------------------------------------------------------
int scanTof(TofId id) {
  Wire.beginTransmission(ADDR_MUX);
  Wire.write(1 << id);
  Wire.endTransmission();

  int v[TOF_SAMPLES];
  int n = 0;
  unsigned long t0 = millis();
  while (n < TOF_SAMPLES && millis() - t0 < TOF_TIMEOUT_MS) {
    if (tof[id].dataReady()) {
      tof[id].read(false);
      if (tof[id].ranging_data.range_status == VL53L1X::RangeValid) {
        int x = tof[id].ranging_data.range_mm, j = n - 1;          // umetni sortirano
        while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
        v[j + 1] = x;
        n++;
      }
    }
    delay(2);
  }
  return (n > 0) ? v[n / 2] : -1;
}

// ---------------------------------------------------------------------
//  2) SKRETANJE: kotaci za 'deg' stupnjeva servo motora od ravnog polozaja.
//     - = lijevo, + = desno, 0 = ravno. Ogranicava se na STEER_LEFT..STEER_RIGHT
//     (sada -18 .. +17). Ceka da servo stigne. Vraca stvarno postavljeni kut.
// ---------------------------------------------------------------------
float steering(float deg) {
  float target = constrain(STEER_CENTER + deg, (float)STEER_LEFT, (float)STEER_RIGHT);
  float change = fabsf(target - steer.angle());
  steer.angle(target);
  delay(50 + (int)(change * STEER_MS_PER_DEG));
  return target - STEER_CENTER;
}

// ---------------------------------------------------------------------
//  3) VOZNJA: 'mm' milimetara, + naprijed, - natrag, s trenutnim zakretom kotaca.
//     Mjeri enkoderom (apsolutna promjena, ne ovisi o smjeru brojanja).
//     Vraca stvarno prijedjeni put u mm (nakon kocenja).
// ---------------------------------------------------------------------
float driveMm(float mm) {
  if (mm == 0) return 0;
  int dirn = (mm > 0) ? 1 : -1;
  float target = fabsf(mm);
  float start = drive.rawAngle();
  unsigned long t0 = millis();

  while (true) {
    float done = fabsf(drive.rawAngle() - start) * MM_PER_DEG;
    if (done >= target) break;
    if (millis() - t0 > DRIVE_TIMEOUT_MS) { Serial.println("driveMm: TIMEOUT"); break; }
    int speed = (target - done > SLOW_ZONE_MM) ? DRIVE_SPEED : SLOW_SPEED;
    drive.run(DRIVE_SIGN * dirn * speed);
    delay(1);
  }
  drive.brake();
  delay(150);
  drive.stop();
  return fabsf(drive.rawAngle() - start) * MM_PER_DEG;
}

// =====================================================================
//  SETUP / LOOP  -  testna konzola u Serial Monitoru (115200, "Newline"):
//    RF  RR  LF  LR  F     -> scanTof
//    S -15                 -> steering(-15)
//    M 30   /   M -30      -> driveMm
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== obstacles: osnovne funkcije ===");

  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  drive.begin();
  steer.begin();
  steer.angle(STEER_CENTER);
  pinMode(START_PIN, INPUT_PULLUP);

  for (uint8_t ch = 0; ch < 5; ch++) {
    Wire.beginTransmission(ADDR_MUX);
    Wire.write(1 << ch);
    Wire.endTransmission();
    tof[ch].setTimeout(100);
    if (!tof[ch].init()) {
      Serial.printf("GRESKA: ToF na kanalu %d se ne javlja\n", ch);
      while (true) delay(1000);
    }
    tof[ch].setDistanceMode(ch == F ? VL53L1X::Long : VL53L1X::Short);
    tof[ch].setROISize(4, 4);
    tof[ch].setROICenter(199);
    tof[ch].setMeasurementTimingBudget(33000);
    tof[ch].startContinuous(40);
  }
  Serial.println("Spreman. Naredbe: RF RR LF LR F | S <st> | M <mm>");
  steering(-17.0);
  driveMm(20.0);
  steering(17.0);
  driveMm(-20.0);
  steering(-17.0);
  driveMm(20.0);
  steering(17.0);
  driveMm(-30.0);
  steering(-17.0);
  driveMm(20.0);
  steering(17.0);
  driveMm(-20.0);
  steering(-17.0);
  driveMm(20.0);
  steering(17.0);
  driveMm(-200.0);
}

void loop() {
  // if (!Serial.available()) { delay(10); return; }
  // String cmd = Serial.readStringUntil('\n');
  // cmd.trim();
  // cmd.toUpperCase();
  // if (cmd.length() == 0) return;


  // if      (cmd == "RF") Serial.printf("RF = %d mm\n", scanTof(RF));
  // else if (cmd == "RR") Serial.printf("RR = %d mm\n", scanTof(RR));
  // else if (cmd == "LF") Serial.printf("LF = %d mm\n", scanTof(LF));
  // else if (cmd == "LR") Serial.printf("LR = %d mm\n", scanTof(LR));
  // else if (cmd == "F")  Serial.printf("F = %d mm\n",  scanTof(F));
  // else if (cmd.startsWith("S")) Serial.printf("kotaci: %.0f st.\n", steering(cmd.substring(1).toFloat()));
  // else if (cmd.startsWith("M")) Serial.printf("prijedjeno: %.1f mm\n", driveMm(cmd.substring(1).toFloat()));
  // else Serial.println("? Naredbe: RF RR LF LR F | S <st> | M <mm>");
}
