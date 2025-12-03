#include "movement.h"
#include "imu.h"
#include "config.h"

const int AIN1 = 3;
const int AIN2 = 2;
const int PWMA = 1;
const int BIN1 = 5;
const int BIN2 = 6;
const int PWMB = 7;
const int STBY = 4;

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
  Serial.println("Movement system initialized");
}

// ============================================================================
// Core Motor Command - Centralized Control
// ============================================================================
void setMotorCommand(int leftSpeed, int rightSpeed, bool applyOffset = true) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Apply motor balance offset consistently
  if (applyOffset) {
    leftSpeed = constrain(leftSpeed - motorConfig.offsetMotorRight, 0, 255);
    rightSpeed = constrain(rightSpeed + motorConfig.offsetMotorRight, 0, 255);
  }

  // Forward direction for both motors
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);  // Left motor forward
  analogWrite(PWMA, leftSpeed);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);   // Right motor forward
  analogWrite(PWMB, rightSpeed);
  digitalWrite(STBY, HIGH);

  lastLeftSpeed = leftSpeed;
  lastRightSpeed = rightSpeed;
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  lastLeftSpeed = 0;
  lastRightSpeed = 0;
}

// ============================================================================
// Basic Movement Functions
// ============================================================================
void moveForward(uint8_t speed) {
  speed = constrain(speed, 0, 255);
  setMotorCommand(speed, speed, true);
}

void moveBackward(uint8_t speed) {
  speed = constrain(speed, 0, 255);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);   // Left motor backward
  analogWrite(PWMA, speed);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);  // Right motor backward
  analogWrite(PWMB, speed);
  digitalWrite(STBY, HIGH);

  lastLeftSpeed = speed;
  lastRightSpeed = speed;
}

// ============================================================================
// Turn Functions with Improved Logic
// ============================================================================
void turnRight(uint8_t speed) {
  speed = constrain(speed, 0, 255);
  currentState = BOT_TURNING;

  extern float yawDeg;
  yawDeg = -20;

  unsigned long start = millis();
  float targetAngle = -motorConfig.turnAngleDegrees;
  bool turnComplete = false;
  float peakYaw = 0;

  Serial.print("Starting RIGHT turn, target: ");
  Serial.print(targetAngle);
  Serial.println("°");

  while (!turnComplete && (millis() - start) < motorConfig.turnTimeoutMs) {
    updateYawDeg();

    // Proportional speed reduction as approaching target (prevents overshoot)
    float angleDiff = fabs(yawDeg) - targetAngle;
    float speedFactor = constrain(1.0f - (angleDiff / 15.0f), 0.3f, 1.0f);

    int adjustedSpeed = (int)(speed * speedFactor);
    adjustedSpeed = constrain(adjustedSpeed, motorConfig.minPWM, speed);

    int leftPwm = constrain(adjustedSpeed + motorConfig.turnDelta, 0, 255);
    int rightPwm = constrain(adjustedSpeed - motorConfig.turnDelta, 0, 255);

    setMotorCommand(leftPwm, rightPwm, false);  // Don't apply offset for turns

    peakYaw = fmax(peakYaw, fabs(yawDeg));

    // Exit when target reached
    if (fabs(yawDeg) >= fabs(targetAngle)) {
      turnComplete = true;
    }

    delay(5);
  }

  stopMotors();

  if (!turnComplete) {
    Serial.print("Turn timeout! Reached: ");
    Serial.print(peakYaw);
    Serial.println("°");
    diagnostics.turnsFailed++;
    currentState = BOT_ERROR;
  } else {
    Serial.print("RIGHT turn complete: ");
    Serial.print(peakYaw);
    Serial.println("°");
    currentState = BOT_LINE_FOLLOWING;
  }
}

void turnLeft(uint8_t speed) {
  speed = constrain(speed, 0, 255);
  currentState = BOT_TURNING;

  extern float yawDeg;
  yawDeg = 20;

  unsigned long start = millis();
  float targetAngle = motorConfig.turnAngleDegrees;  // Negative for left
  bool turnComplete = false;
  float peakYaw = 0;

  Serial.print("Starting LEFT turn, target: ");
  Serial.print(targetAngle - 10);
  Serial.println("°");

  while (!turnComplete && (millis() - start) < motorConfig.turnTimeoutMs) {
    updateYawDeg();

    // Proportional speed reduction
    float angleDiff = -fabs(yawDeg) +fabs(targetAngle);
    float speedFactor = constrain(1.0f - (angleDiff / 15.0f), 0.3f, 1.0f);

    int adjustedSpeed = (int)(speed * speedFactor);
    adjustedSpeed = constrain(adjustedSpeed, motorConfig.minPWM, speed);

    int leftPwm = constrain(adjustedSpeed - motorConfig.turnDelta, 0, 255);
    int rightPwm = constrain(adjustedSpeed + motorConfig.turnDelta, 0, 255);

    setMotorCommand(leftPwm, rightPwm, false);

    peakYaw = fmax(peakYaw, yawDeg);

    // Exit when target reached
    if (fabs(yawDeg) >= fabs(targetAngle)) {
      turnComplete = true;
    }

    delay(5);
  }

  stopMotors();

  if (!turnComplete) {
    Serial.print("Turn timeout! Reached: ");
    Serial.print(peakYaw);
    Serial.println("°");
    diagnostics.turnsFailed++;
    currentState = BOT_ERROR;
  } else {
    Serial.print("LEFT turn complete: ");
    Serial.print(peakYaw);
    Serial.println("°");
    currentState = BOT_LINE_FOLLOWING;
  }
}

void turnAround(uint8_t speed) {
  speed = constrain(speed, 0, 255);
  currentState = BOT_TURNING;

  extern float yawDeg;
  yawDeg = 0;

  unsigned long start = millis();
  float targetAngle = -motorConfig.turnAngleDegrees - 90;
  bool turnComplete = false;
  float peakYaw = 0;

  Serial.print("Starting RIGHT turn, target: ");
  Serial.print(targetAngle);
  Serial.println("°");

  while (!turnComplete && (millis() - start) < motorConfig.turnTimeoutMs) {
    updateYawDeg();

    // Proportional speed reduction as approaching target (prevents overshoot)
    float angleDiff = fabs(yawDeg) - targetAngle;
    float speedFactor = constrain(1.0f - (angleDiff / 15.0f), 0.3f, 1.0f);

    int adjustedSpeed = (int)(speed * speedFactor);
    adjustedSpeed = constrain(adjustedSpeed, motorConfig.minPWM, speed);

    int leftPwm = constrain(adjustedSpeed + motorConfig.turnDelta, 0, 255);
    int rightPwm = constrain(adjustedSpeed - motorConfig.turnDelta, 0, 255);

    setMotorCommand(leftPwm, rightPwm, false);  // Don't apply offset for turns

    peakYaw = fmax(peakYaw, fabs(yawDeg));

    // Exit when target reached
    if (fabs(yawDeg) >= fabs(targetAngle)) {
      turnComplete = true;
    }

    delay(5);
  }

  stopMotors();

  if (!turnComplete) {
    Serial.print("Turn timeout! Reached: ");
    Serial.print(peakYaw);
    Serial.println("°");
    diagnostics.turnsFailed++;
    currentState = BOT_ERROR;
  } else {
    Serial.print("RIGHT turn complete: ");
    Serial.print(peakYaw);
    Serial.println("°");
    currentState = BOT_LINE_FOLLOWING;
  }
}


// ============================================================================
// Line Following Motor Control
// ============================================================================
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  setMotorCommand(leftSpeed, rightSpeed, true);
}

// ============================================================================
// Getter Functions
// ============================================================================
int getLeftSpeed() {
  return lastLeftSpeed;
}

int getRightSpeed() {
  return lastRightSpeed;
}
