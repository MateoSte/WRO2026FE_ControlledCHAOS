#define PWMA 7
#define AIN1 4
#define AIN2 5
#define STBY 10

void motor_forward(int speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, speed);
}

void motor_backward(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, speed);
}

void motor_stop() {
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Aktiviraj TB6612FNG
  digitalWrite(STBY, HIGH);

  motor_stop();
}

void loop() {

  Serial.println("FORWARD - speed 100");
  motor_forward(100);
  delay(2000);

  Serial.println("FORWARD - speed 180");
  motor_forward(180);
  delay(2000);

  Serial.println("FORWARD - speed 255");
  motor_forward(255);
  delay(2000);

  Serial.println("STOP");
  motor_stop();
  delay(1000);

  Serial.println("BACKWARD - speed 100");
  motor_backward(100);
  delay(2000);

  Serial.println("BACKWARD - speed 180");
  motor_backward(180);
  delay(2000);

  Serial.println("BACKWARD - speed 255");
  motor_backward(255);
  delay(2000);

  Serial.println("STOP");
  motor_stop();
  delay(2000);
}