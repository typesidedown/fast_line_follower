#include <Arduino.h>
#include "movement.h"
#include "sensors.h"
#include "imu.h"
#include "oled.h"

void setup() {
  Serial.begin(115200);

  // initialize modules
  initMovement();
  initSensors();
  initIMU();
  initOLED();

  // optional: tune threshold here if needed
  // setIRThreshold(600);
  // Serial.print("IR threshold: ");
  // Serial.println(getIRThreshold());
}

void loop() {
  readIRArray();

  // print raw and digital IR readings
  Serial.print("IR raw: ");
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(rawIrReadings[i]);
    if (i < 7) Serial.print(",");
  }
  Serial.println();

  Serial.print("IR dig: ");
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(irReadings[i]);
    if (i < 7) Serial.print(",");
  }
  Serial.println();

  float yaw = updateYawDeg();
  Serial.print("Yaw: ");
  Serial.println(yaw);

  // update OLED status
  float err = calculate_error();
  printf("Error: %.3f\n", err);
  // get latest motor speeds
  int leftSpeed = getLeftSpeed();
  int rightSpeed = getRightSpeed();
  Serial.print("Left motor speed: ");
  Serial.println(leftSpeed);

  oledDisplayStatus(yaw, err, leftSpeed, rightSpeed);

  int turn = detect_turn();

  if (turn == -1) {
    turnLeft(100);
  } else if (turn == 1) {
    turnRight(100);
  } else if (turn == 0) {
    followLine();
  } else {
    stopMotors();
  }

  // delay(400);
}