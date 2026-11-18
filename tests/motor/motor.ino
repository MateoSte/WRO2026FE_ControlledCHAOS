#include "../../includes/includes.ino"

volatile long pulses = 0;

void IRAM_ATTR onPulse() {              // called on every channel A pulse
  pulses += digitalRead(PIN_ENC_B) ? 1 : -1;
}

void drive(int speed) {                 // -100 .. 100
  digitalWrite(PIN_AIN1, speed > 0);
  digitalWrite(PIN_AIN2, speed < 0);
  ledcWrite(PIN_PWMA, abs(speed) * 1023 / 100);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  ledcAttach(PIN_PWMA, 20000, 10);      // 20 kHz, 10 bits (0..1023)
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onPulse, RISING);

  digitalWrite(PIN_STBY, HIGH);
  int speeds[] = {40, 0, -40, 0};
  for (int s : speeds) {
    drive(s);
    delay(2000);
    Serial.printf("speed %d, pulses %ld\n", s, pulses);
  }
  digitalWrite(PIN_STBY, LOW);          // driver off, the wheel can be turned by hand
}

void loop() {
  // Phase D, step 6: turn the wheel one revolution by hand and read the count.
  Serial.printf("pulses %ld\n", pulses);
  delay(500);
}