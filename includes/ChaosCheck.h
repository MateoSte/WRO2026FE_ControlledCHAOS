// =====================================================================
//  ChaosCheck.h  -  Controlled C.H.A.O.S. v2
//  Brza provjera svih I2C uredjaja (bez ikakvog pomicanja robota).
//
//  Uporaba (nakon Wire.begin):
//      int fails = chaosI2CCheck(false);   // false = kamera nije obavezna
//      (opcionalno: chaosI2CCheck(false, &warns) pribroji upozorenja u warns)
//      if (fails > 0) { ... ne voziti ... }
//
//  Provjerava:
//    - MUX 0x70 i RTC 0x51
//    - MPU-6050 0x68: javlja li se, WHO_AM_I, spava li, "zivi" li ziroskop
//    - ToF 0x29 na kanalima 0-4: javlja li se i je li to stvarno VL53L1X
//    - OpenMV AE3 0x12 na kanalu 5 (samo ako radi njezina skripta)
// =====================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "includes.ino"

inline void ccMux(uint8_t mask) {
  Wire.beginTransmission(ADDR_MUX);
  Wire.write(mask);
  Wire.endTransmission();
}

inline bool ccPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

inline bool ccRead8(uint8_t addr, uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  val = Wire.read();
  return true;
}

inline bool ccRead16reg(uint8_t addr, uint16_t reg, uint8_t &val) {   // VL53L1X: 16-bitni registri
  Wire.beginTransmission(addr);
  Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  val = Wire.read();
  return true;
}

inline void ccLine(const char *status, const char *what, const char *note = "") {
  Serial.printf("  [%-4s] %-32s %s\n", status, what, note);
}

// Vraca broj GRESAKA (FAIL). Upozorenja (WARN) se samo ispisu.
inline int chaosI2CCheck(bool cameraRequired, int *warnCount = nullptr) {
  int fails = 0, warns = 0;
  Serial.println("--- Provjera I2C uredjaja ---");

  // MUX
  ccMux(0x00);
  if (ccPresent(ADDR_MUX)) ccLine("OK", "MUX TCA9548A (0x70)");
  else {
    ccLine("FAIL", "MUX TCA9548A (0x70)", "Qwiic kabel ESP32 -> MUX");
    Serial.println("  Bez MUX-a nema ToF senzora ni kamere - prekid.");
    return 1;
  }

  // RTC (na plocici - ako ga nema, sabirnica je u problemu)
  if (ccPresent(ADDR_RTC)) ccLine("OK", "RTC na plocici (0x51)");
  else { warns++; ccLine("WARN", "RTC na plocici (0x51)", "cudno - provjeri SDA/SCL"); }

  // MPU-6050
  uint8_t who = 0, pwr = 0;
  if (!ccRead8(ADDR_MPU, 0x75, who)) {
    ccLine("FAIL", "MPU-6050 (0x68)", "ne javlja se: kabel MPU / pass-through na MUX-u");
    fails++;
  } else {
    char note[48];
    ccRead8(ADDR_MPU, 0x6B, pwr);
    // ziroskop "zivi": 20 ocitanja ne smiju biti potpuno ista
    int16_t first = 0; bool allSame = true;
    for (int i = 0; i < 20; i++) {
      uint8_t hi = 0, lo = 0;
      ccRead8(ADDR_MPU, 0x47, hi);
      ccRead8(ADDR_MPU, 0x48, lo);
      int16_t gz = (int16_t)((hi << 8) | lo);
      if (i == 0) first = gz; else if (gz != first) allSame = false;
      delay(3);
    }
    if (who != 0x68) {
      snprintf(note, sizeof(note), "WHO_AM_I 0x%02X (ocekivano 0x68)", who);
      warns++; ccLine("WARN", "MPU-6050 (0x68)", note);
    } else if (pwr & 0x40) {
      warns++; ccLine("WARN", "MPU-6050 (0x68)", "spava (bio reset) - program ga budi");
    } else if (allSame) {
      warns++; ccLine("WARN", "MPU-6050 (0x68)", "ziroskop daje uvijek istu vrijednost");
    } else {
      ccLine("OK", "MPU-6050 (0x68)");
    }
  }

  // ToF senzori
  const struct { uint8_t ch; const char *name; } tofs[] = {
    {MUX_TOF_RIGHT_FRONT, "ToF desni prednji (kanal 0)"},
    {MUX_TOF_RIGHT_REAR,  "ToF desni zadnji (kanal 1)"},
    {MUX_TOF_LEFT_FRONT,  "ToF lijevi prednji (kanal 2)"},
    {MUX_TOF_FRONT,       "ToF prednji (kanal 3)"},
    {MUX_TOF_LEFT_REAR,   "ToF lijevi zadnji (kanal 4)"},
  };
  for (auto &t : tofs) {
    ccMux(1 << t.ch);
    uint8_t id = 0;
    if (!ccRead16reg(ADDR_TOF, 0x010F, id)) {
      ccLine("FAIL", t.name, "ne javlja se: Qwiic kabel na tom kanalu");
      fails++;
    } else if (id != 0xEA) {
      char note[40];
      snprintf(note, sizeof(note), "ID 0x%02X - nije VL53L1X?", id);
      warns++; ccLine("WARN", t.name, note);
    } else {
      ccLine("OK", t.name);
    }
  }

  // Kamera
  ccMux(1 << MUX_CAMERA);
  bool cam = ccPresent(ADDR_AE3);
  if (cam) ccLine("OK", "OpenMV AE3 (kanal 5, 0x12)");
  else if (cameraRequired) { ccLine("FAIL", "OpenMV AE3 (kanal 5, 0x12)", "kabel ili skripta ne radi"); fails++; }
  else { warns++; ccLine("WARN", "OpenMV AE3 (kanal 5, 0x12)", "nije spojena ili skripta ne radi"); }

  ccMux(0x00);
  if (warnCount) *warnCount += warns;
  Serial.printf("--- I2C: %s (%d gresaka) ---\n", fails ? "NE VOZITI" : "u redu", fails);
  return fails;
}
