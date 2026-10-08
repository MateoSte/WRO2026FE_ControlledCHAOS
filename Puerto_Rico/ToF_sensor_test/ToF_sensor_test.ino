#include <Wire.h>
#include <VL53L1X.h>


// =====================================================
// NULA DeepSleep ESP32-S3 - I2C
// =====================================================

#define SDA_PIN 8
#define SCL_PIN 9


// =====================================================
// VL53L1X
// =====================================================

VL53L1X distanceSensor;


// =====================================================
// N20 MOTOR + TB6612FNG
// =====================================================

#define PWMA 7
#define AIN1 4
#define AIN2 5
#define STBY 10


void motor_forward(int speed){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  analogWrite(PWMA, speed);
}


void motor_backward(int speed){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  analogWrite(PWMA, speed);
}


void motor_stop(){
  analogWrite(PWMA, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}


// =====================================================
// MOVE UNTIL TARGET DISTANCE
// =====================================================

void move_until_distance(uint16_t targetDistance, int speed){
  Serial.println();
  Serial.println("Starting movement...");
  
  Serial.print("Target distance: ");
  Serial.print(targetDistance);
  Serial.println(" mm");

  motor_forward(speed);

  while (true){
    uint16_t distance = distanceSensor.read();

    if (distanceSensor.timeoutOccurred())
    {
      Serial.println("VL53L1X TIMEOUT!");
      
      motor_stop();
      return;
    }

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");


    // Object is close enough
    if (distance <= targetDistance)
    {
      motor_stop();

      Serial.println("Target distance reached!");
      Serial.println("Motor stopped.");

      return;
    }

    delay(50);
  }
}


// =====================================================
// SETUP
// =====================================================

void setup(){
  Serial.begin(115200);

  delay(1000);


  // ---------------------------------------------------
  // I2C
  // ---------------------------------------------------

  Wire.begin(SDA_PIN, SCL_PIN);


  // ---------------------------------------------------
  // Motor pins
  // ---------------------------------------------------

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);


  // Aktiviraj TB6612FNG
  digitalWrite(STBY, HIGH);

  motor_stop();


  // ---------------------------------------------------
  // VL53L1X
  // ---------------------------------------------------

  distanceSensor.setTimeout(500);

  if (!distanceSensor.init())
  {
    Serial.println("VL53L1X not detected!");

    while (1)
    {
      motor_stop();
      delay(100);
    }
  }

  Serial.println("VL53L1X detected!");

  // Start continuous measurements
  distanceSensor.startContinuous(50);


  Serial.println();
  Serial.println("================================");
  Serial.println("N20 + VL53L1X");
  Serial.println("================================");
}


// =====================================================
// LOOP
// =====================================================

void loop(){
  // Move forward until the sensor is
  // 300 mm away from the object.
  //
  // 300 = target distance in millimeters
  // 150 = motor speed (0-255)

  move_until_distance(300, 150);


  // Wait after reaching the target
  delay(3000);
}

