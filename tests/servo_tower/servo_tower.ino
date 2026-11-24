#include "../../includes/includes.ino"

const int SERVO_PIN = PIN_SRV_TOWER;    // DFRobot SER0046, 270 degrees

void angle(int degrees) {               // 0..270 degrees -> 500..2500 microseconds
  int us = TOWER_MIN_US + degrees * (long)(TOWER_MAX_US - TOWER_MIN_US) / TOWER_MAX_DEG;
  ledcWrite(SERVO_PIN, us * 16383L / 20000);   // 50 Hz = 20 000 us, 14 bits
  Serial.printf("angle %d = %d us\n", degrees, us);

  delay(600);                           // give the servo time to reach the position
  float mv = analogReadMilliVolts(FEEDBACK_PIN);
  float estDeg = TOWER_CENTER + (mv - TOWER_MV_CENTER) / TOWER_MV_PER_DEG;
  Serial.printf("  feedback %.0f mV -> approx. %.1f deg\n", mv, estDeg);
}

void setup() {
  Serial.begin(115200);
  ledcAttach(SERVO_PIN, SERVO_PWM_FREQ, SERVO_PWM_BITS);
  analogSetPinAttenuation(FEEDBACK_PIN, ADC_11db);   // 0..~3.1 V range
  angle(TOWER_CENTER);                  // camera looks forward
  Serial.println("Type an angle (0-270, e.g. 135) and press Enter.");
}
//100,55,10
//170,215,260
void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() > 0) {               // an empty line is ignored, the servo does not jump to 0
      int a = s.toInt();
      if (a >= 0 && a <= TOWER_MAX_DEG) angle(a);
      else Serial.println("Out of range (0-270).");
    }
  }
}