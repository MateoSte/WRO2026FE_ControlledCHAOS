#include <Wire.h>
#include "../includes/includes.ino"

void channel(int k) {                   // k = -1 closes all channels
  Wire.beginTransmission(MUX_ADDR);
  Wire.write((uint8_t)(k < 0 ? 0 : (1 << k)));
  Wire.endTransmission();
}

void scan(bool skipMainBus) {
  for (uint8_t a = 1; a < 127; a++) {
    if (skipMainBus && (a == 0x70 || a == 0x51)) continue;
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000);
  channel(-1);
  Serial.print("Main bus:");
  scan(false);
  for (int k = 0; k < 8; k++) {
    channel(k);
    Serial.printf("Channel %d:", k);
    scan(true);
  }
  channel(-1);
}

void loop() {}