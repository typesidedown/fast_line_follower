#include "movement.h"
#include "imu.h"

const int AIN1 = 3;
const int AIN2 = 2;
const int PWMA = 1;
const int BIN1 = 5;
const int BIN2 = 6;
const int PWMB = 7;
const int STBY = 4;
int Offset_motor_right = 0;
int delta = 30;
// track last commanded speeds (0-255)
static int lastLeftSpeed = 0;
static int lastRightSpeed = 0;

void initMovement() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void stopMotors(){
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  lastLeftSpeed = 0;
  lastRightSpeed = 0;
}

void moveForward(uint8_t speed){
  speed = constrain(speed, 0, 255);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);  // Left motor reversed
  analogWrite(PWMA, speed);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  // Right motor normal
  analogWrite(PWMB, speed);
  digitalWrite(STBY, HIGH);
  lastLeftSpeed = speed;
  lastRightSpeed = speed;
}

void moveBackward(uint8_t speed){
  speed = constrain(speed, 0, 255);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  // Left motor reversed
  analogWrite(PWMA, speed);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);  // Right motor normal
  analogWrite(PWMB, speed);
  digitalWrite(STBY, HIGH);
  lastLeftSpeed = speed;
  lastRightSpeed = speed;
}

void turnRight(uint8_t speed){
  speed = constrain(speed, 0, 255);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);  // Left motor reversed
  int leftPwm = constrain(speed + delta, 0, 255);
  int rightPwm = constrain(speed - delta, 0, 255);
  analogWrite(PWMA, leftPwm);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMB, rightPwm);
  digitalWrite(STBY, HIGH);
  // record commanded speeds so UI can show them
  lastLeftSpeed = leftPwm;
  lastRightSpeed = rightPwm;

  // use yaw from IMU: reset and rotate until absolute angle reaches ~90deg
  extern float yawDeg;
  yawDeg = 0;
  unsigned long start = millis();
  const unsigned long timeoutMs = 4000; // safety timeout
  while (fabs(yawDeg) < 90.0f && (millis() - start) < timeoutMs) {
    updateYawDeg();
    delay(5); // give IMU/I2C some breathing room
  }
  stopMotors();
}

void turnLeft(uint8_t speed){
  speed = constrain(speed, 0, 255);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);  // Left motor reversed
  int leftPwm = constrain(speed - delta, 0, 255);
  int rightPwm = constrain(speed + delta, 0, 255);
  analogWrite(PWMA, leftPwm);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMB, rightPwm);
  digitalWrite(STBY, HIGH);
  lastLeftSpeed = leftPwm;
  lastRightSpeed = rightPwm;

  extern float yawDeg;
  yawDeg = 0;
  unsigned long start = millis();
  const unsigned long timeoutMs = 4000;
  while (fabs(yawDeg) < 90.0f && (millis() - start) < timeoutMs) {
    updateYawDeg();
    delay(5);
  }
  stopMotors();
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);  // Left forward (reversed)
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  // Right forward
  analogWrite(PWMA, leftSpeed - Offset_motor_right);
  analogWrite(PWMB, rightSpeed + Offset_motor_right);
  digitalWrite(STBY, HIGH);
  lastLeftSpeed = leftSpeed;
  lastRightSpeed = rightSpeed;
  delay(1);
}

int getLeftSpeed() {
  return lastLeftSpeed;
}

int getRightSpeed() {
  return lastRightSpeed;
}
