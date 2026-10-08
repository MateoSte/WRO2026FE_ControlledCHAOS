#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void camera() {
  Wire.requestFrom(0x12, 2);  // request 2 bytes now
  if (Wire.available() >= 2) {
    int result[2];
    result[0] = Wire.read();  // read first byte into result[0]
    result[1] = Wire.read();  // read second byte into result[1]

    if (result[0] == 1) {
      Serial.println("RED detected");
      Serial.print("distance:");
      Serial.println(result[1]);
      Serial.println("--------------------------");
    }
    else if (result[0] == 2) {
      Serial.println("GREEN detected");
      Serial.print("distance:");
      Serial.println(result[1]);
      Serial.println("--------------------------");
    }
    else {
      Serial.println("Nothing detected");
      Serial.println("--------------------------");
      }

  }
  delay(1000);
}

void loop() {
  camera();
}