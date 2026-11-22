// =====================================================================
//  tof_view.ino  -  Controlled C.H.A.O.S. v2
//  Ispis svih 5 ToF senzora: udaljenost, status i jacina signala.
//  Sluzi da se vidi STO senzor stvarno "vidi" (zid, pod, dio robota...).
//
//  Serial Monitor 115200. Naredbe:
//    w = prednji senzor: siroki pogled 16x16 (~27 st., tvornicki)
//    n = prednji senzor: uski pogled 8x8   (~15 st.)
//    u = prednji senzor: vrlo uski 4x4     (~7 st.)
//    m = prednji senzor: Short <-> Long nacin
//
//  Potrebno: ChaosPins.h, biblioteka "VL53L1X" by Pololu.
// =====================================================================
#include <Wire.h>
#include <VL53L1X.h>
#include "../../includes/includes.ino"

VL53L1X tof[5];
const uint8_t ch[5]   = {MUX_TOF_RIGHT_FRONT, MUX_TOF_RIGHT_REAR, MUX_TOF_LEFT_FRONT,
                         MUX_TOF_FRONT, MUX_TOF_LEFT_REAR};
const char   *name[5] = {"D-pred", "D-zad", "L-pred", "PREDNJI", "L-zad"};
const int FRONT = 3;
bool longMode = false;

void mux(uint8_t c) {
  Wire.beginTransmission(ADDR_MUX);
  Wire.write(1 << c);
  Wire.endTransmission();
}

void setFrontRoi(uint8_t size) {
  mux(ch[FRONT]);
  tof[FRONT].stopContinuous();
  tof[FRONT].setROISize(size, size);
  tof[FRONT].startContinuous(longMode ? 60 : 40);
  Serial.printf("\n>>> PREDNJI: pogled %ux%u\n\n", size, size);
}

void setFrontMode(bool lng) {
  longMode = lng;
  mux(ch[FRONT]);
  tof[FRONT].stopContinuous();
  tof[FRONT].setDistanceMode(lng ? VL53L1X::Long : VL53L1X::Short);
  tof[FRONT].setMeasurementTimingBudget(lng ? 50000 : 33000);
  tof[FRONT].startContinuous(lng ? 60 : 40);
  Serial.printf("\n>>> PREDNJI: %s nacin\n\n", lng ? "Long (do ~4 m)" : "Short (do ~1.3 m)");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== tof_view ===");
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

  for (int i = 0; i < 5; i++) {
    mux(ch[i]);
    tof[i].setTimeout(100);
    if (!tof[i].init()) { Serial.printf("%s (kanal %u) se ne javlja!\n", name[i], ch[i]); continue; }
    tof[i].setDistanceMode(VL53L1X::Short);
    tof[i].setMeasurementTimingBudget(33000);
    tof[i].startContinuous(40);
  }
  Serial.println("mm = udaljenost, sig = jacina povratnog signala (MCPS), amb = okolno svjetlo");
  Serial.println("Naredbe: w / n / u = sirina pogleda prednjeg, m = Short/Long");
}

void loop() {
  static unsigned long lastPrint = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') setFrontRoi(16);
    if (c == 'n') setFrontRoi(8);
    if (c == 'u') setFrontRoi(4);
    if (c == 'm') setFrontMode(!longMode);
  }

  if (millis() - lastPrint < 200) return;
  lastPrint = millis();

  for (int i = 0; i < 5; i++) {
    mux(ch[i]);
    tof[i].read(true);          // ceka sljedece mjerenje (najvise ~40-60 ms)
    VL53L1X::RangingData &r = tof[i].ranging_data;
    Serial.printf("%-7s %4u mm %-16s sig %5.2f amb %5.2f | ",
                  name[i], r.range_mm, VL53L1X::rangeStatusToString(r.range_status),
                  r.peak_signal_count_rate_MCPS, r.ambient_count_rate_MCPS);
    if (i == 2) Serial.println();
  }
  Serial.println("\n");
}
