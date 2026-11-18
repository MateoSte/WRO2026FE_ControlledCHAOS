#include "../classes/classes.ino"
#include "../includes/includes.ino"

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    drive.begin();
    delay(2000);
}

void loop() {
    Serial.println(drive.angle());
    delay(200);
}