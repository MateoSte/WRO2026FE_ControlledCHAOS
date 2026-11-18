#include "../classes/classes.ino"
#include "../includes/includes.ino"

using namespace pybricks::pupdevices;

Motor left(7, 4, 5, 10, 12, 13, 11.0f, 100.0f);

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