#include "../includes/includes.ino"

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN, INPUT_PULLUP);
  Serial.println("Waiting for START...");
}

void loop() {
  if (digitalRead(PIN_BTN) == LOW) {
    delay(30);                          // let the contact settle
    if (digitalRead(PIN_BTN) == LOW) {
      Serial.println("START");
      while (digitalRead(PIN_BTN) == LOW) delay(10);   // wait for release
    }
  }
  delay(10);
}