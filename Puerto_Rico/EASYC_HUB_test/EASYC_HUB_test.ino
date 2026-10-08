#include <Wire.h>

// I2C adrese
#define CAMERA_ADDR 0x12
#define LTR507_ADDR 0x3A
#define TMP117_ADDR 0x49

// NULA DeepSleep ESP32-S3
#define SDA_PIN 8
#define SCL_PIN 9


// =====================================================
// CAMERA
// =====================================================

void camera()
{
  Wire.requestFrom(CAMERA_ADDR, (uint8_t)2);

  if (Wire.available() >= 2)
  {
    int result[2];

    result[0] = Wire.read();
    result[1] = Wire.read();

    if (result[0] == 1)
    {
      Serial.println("RED detected");
      Serial.print("distance: ");
      Serial.println(result[1]);
      Serial.println("--------------------------");
    }
    else if (result[0] == 2)
    {
      Serial.println("GREEN detected");
      Serial.print("distance: ");
      Serial.println(result[1]);
      Serial.println("--------------------------");
    }
    else
    {
      Serial.println("Nothing detected");
      Serial.println("--------------------------");
    }
  }
}


// =====================================================
// LTR-507
// =====================================================

void LTR()
{
  // Uključi ALS
  Wire.beginTransmission(LTR507_ADDR);
  Wire.write(0x80);
  Wire.write(0x02);
  Wire.endTransmission();

  // Uključi proximity
  Wire.beginTransmission(LTR507_ADDR);
  Wire.write(0x81);
  Wire.write(0x0E);
  Wire.endTransmission();

  delay(100);


  // -------------------------
  // Svjetlost
  // -------------------------

  Wire.beginTransmission(LTR507_ADDR);
  Wire.write(0x88);
  Wire.endTransmission(false);

  Wire.requestFrom(LTR507_ADDR, (uint8_t)2);

  uint16_t light = 0;

  if (Wire.available() >= 2)
  {
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();

    light = ((uint16_t)high << 8) | low;
  }


  // -------------------------
  // Proximity
  // -------------------------

  Wire.beginTransmission(LTR507_ADDR);
  Wire.write(0x8B);
  Wire.endTransmission(false);

  Wire.requestFrom(LTR507_ADDR, (uint8_t)2);

  uint16_t proximity = 0;

  if (Wire.available() >= 2)
  {
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();

    proximity = ((uint16_t)high << 8) | low;
  }


  // -------------------------
  // Ispis
  // -------------------------

  Serial.print("Light raw: ");
  Serial.println(light);

  Serial.print("Proximity raw: ");
  Serial.println(proximity);

  Serial.println("--------------------------");
}


// =====================================================
// TMP117
// =====================================================

void temp_sensor()
{
  // TMP117 registar temperature = 0x00

  Wire.beginTransmission(TMP117_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);

  Wire.requestFrom(TMP117_ADDR, (uint8_t)2);

  if (Wire.available() >= 2)
  {
    uint8_t high = Wire.read();
    uint8_t low = Wire.read();

    uint16_t raw = ((uint16_t)high << 8) | low;

    int16_t signedRaw = (int16_t)raw;

    float temperature = signedRaw * 0.0078125;

    Serial.print("Temperature: ");
    Serial.print(temperature, 3);
    Serial.println(" C");

    Serial.println("--------------------------");
  }
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  // NULA DeepSleep ESP32-S3
  // SDA = GPIO 8
  // SCL = GPIO 9
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println();
  Serial.println("================================");
  Serial.println("NULA - Camera + LTR-507 + TMP117");
  Serial.println("================================");
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  camera();

  temp_sensor();

  LTR();

  delay(1000);
}