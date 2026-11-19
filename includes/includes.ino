#ifndef FE_INCLUDES
#define FE_INCLUDES
#include <Arduino.h>
#include "../classes/classes.ino"
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
#define WHEEL_CIRCUMFERENCE_MM     PI*62.4f // pi x 43.2 mm
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
// CarDriveBase car(drive, steer, )

#endif