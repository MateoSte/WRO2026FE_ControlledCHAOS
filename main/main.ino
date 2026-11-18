#include "../classes/classes.ino"

void setup() {
    Serial.begin(115200);
    left.begin();
}

void loop() {
    left.run_target(200, 360);
    delay(1000);
    Serial.println(left.rawAngle());

    left.run_target(200, 180);
    delay(1000);
    Serial.println(left.rawAngle());

    left.run_target(200, 0);
    delay(1000);
    Serial.println(left.rawAngle());
    Serial.println("---");
}