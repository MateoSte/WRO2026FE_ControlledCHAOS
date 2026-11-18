#include "../includes/includes.ino"

const float FACTOR = (100.0 + 22.0) / 22.0;   // 100k / 22k divider; adjust to match the multimeter

void setup() {
  Serial.begin(115200);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);   // input range up to ~3.1 V
}

void loop() {
  float v = analogReadMilliVolts(PIN_VBAT) / 1000.0 * FACTOR;
  Serial.printf("Battery: %.2f V\n", v);
  delay(1000);
}
