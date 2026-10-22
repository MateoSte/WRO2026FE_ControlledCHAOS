#include <ESP32Servo.h>
#define SERVO_PIN 6

Servo servo;


void servoSetAngle(int deg) {
  servo.write(constrain(deg, -90, 90) + 90);   //range from -90 to 90
}

void setup() {
  Serial.begin(115200);
  servo.attach(SERVO_PIN, 500, 2500);
  
}

void loop() {
  //TEST
  servoSetAngle(-90);
  delay(2000);
  servoSetAngle(90);
  delay(2000);
  servoSetAngle(0);
  delay(2000);
}