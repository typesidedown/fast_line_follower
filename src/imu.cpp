#include "imu.h"
#include <Wire.h>
#include <SPI.h>
#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

float yawDeg = 0.0f;
float gyroZOffset = 0.0f;
unsigned long lastYawTime = 0;
const float GYRO_SENSITIVITY = 16.4f;

void calibrateGyroZ(int samples) {
  long sum = 0;
  int16_t accelGyro[6];
  for (int i = 0; i < samples; i++) {
    if (bmi160.getAccelGyroData(accelGyro) == 0) {
      sum += accelGyro[2];
    }
    delay(2);
  }
  float avgRaw = (float)sum / samples;
  gyroZOffset = avgRaw / GYRO_SENSITIVITY;
}

float updateYawDeg() {
  int16_t accelGyro[6] = {0};
  int rslt = bmi160.getAccelGyroData(accelGyro);
  if (rslt != 0) {
    return yawDeg;
  }
  unsigned long now = millis();
  float dt = (now - lastYawTime) / 1000.0f;
  lastYawTime = now;
  float gz_dps = (float)accelGyro[2] / GYRO_SENSITIVITY;
  gz_dps -= gyroZOffset;
  yawDeg += gz_dps * dt;
  if (yawDeg > 180.0f) yawDeg -= 360.0f;
  else if (yawDeg < -180.0f) yawDeg += 360.0f;
  return yawDeg;
}

void initIMU() {
  Wire.begin();
  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("BMI160 reset failed");
  } else {
    Serial.println("BMI160 reset OK");
  }
  delay(50);
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK) {
    Serial.println("BMI160 init failed");
  } else {
    Serial.println("BMI160 init OK");
  }
  delay(1000);
  calibrateGyroZ();
  lastYawTime = millis();
}
