#include "../../includes/includes.ino"

const int SERVO_PIN = PIN_SRV_STEER;    // for the camera tower: PIN_SRV_TOWER

void angle(int degrees) {               // 0..180 degrees -> 500..2500 microseconds
  int us = 500 + degrees * 2000L / 180;
  ledcWrite(SERVO_PIN, us * 16383L / 20000);   // 50 Hz = 20 000 us, 14 bits
  Serial.printf("angle %d = %d us\n", degrees, us);
}

void setup() {
  Serial.begin(115200);
  ledcAttach(SERVO_PIN, 50, 14);
  angle(90);
  Serial.println("Type an angle (e.g. 75) and press Enter.");
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() > 0) {               // an empty line is ignored, the servo does not jump to 0
      int a = s.toInt();
      if (a >= 0 && a <= 180) angle(a);
    }
  }
}