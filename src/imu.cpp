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

// Distance estimation structure
DistanceEstimator distEstimator = {
  0.0f,        // accelOffsetX
  0.0f,        // accelOffsetY
  0.0f,        // velocityX
  0.0f,        // velocityY
  0.0f,        // distanceX
  0.0f,        // distanceY
  0.0f,        // totalDistance
  0,           // lastUpdateTime
  1.0f / 2048.0f,  // accelSensitivity (BMI160: ±16g range)
  50,          // distanceUnit: 50mm per unit
  false        // isTracking
};

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

// ============================================================================
// DISTANCE ESTIMATION FUNCTIONS
// ============================================================================

void initDistanceEstimator(int unitSize) {
  distEstimator.distanceUnit = unitSize;
  distEstimator.lastUpdateTime = millis();
  Serial.print("Distance estimator initialized: ");
  Serial.print(unitSize);
  Serial.println("mm per unit");
}

void calibrateAccelerometer(int samples) {
  long sumX = 0, sumY = 0;
  int16_t accelGyro[6] = {0};
  
  Serial.println("Calibrating accelerometer (keep bot still)...");
  
  for (int i = 0; i < samples; i++) {
    if (bmi160.getAccelGyroData(accelGyro) == 0) {
      // accelGyro[0] = accelX, accelGyro[1] = accelY, accelGyro[2] = accelZ (gyro)
      sumX += accelGyro[0];
      sumY += accelGyro[1];
    }
    delay(2);
  }
  
  // Average the readings and convert to m/s²
  float avgX = (float)sumX / samples;
  float avgY = (float)sumY / samples;
  
  distEstimator.accelOffsetX = avgX * distEstimator.accelSensitivity;
  distEstimator.accelOffsetY = avgY * distEstimator.accelSensitivity;
  
  Serial.print("Accel offset - X: ");
  Serial.print(distEstimator.accelOffsetX);
  Serial.print(" m/s²  Y: ");
  Serial.print(distEstimator.accelOffsetY);
  Serial.println(" m/s²");
}

void updateDistance() {
  if (!distEstimator.isTracking) {
    return;
  }
  
  int16_t accelGyro[6] = {0};
  if (bmi160.getAccelGyroData(accelGyro) != 0) {
    return;  // I2C error
  }
  
  unsigned long now = millis();
  float dt = (now - distEstimator.lastUpdateTime) / 1000.0f;  // Convert to seconds
  distEstimator.lastUpdateTime = now;
  
  // Prevent unrealistic dt values
  if (dt <= 0 || dt > 0.1f) {
    return;
  }
  
  // Convert raw accelerometer values to m/s²
  float accelX_raw = (float)accelGyro[0] * distEstimator.accelSensitivity;
  float accelY_raw = (float)accelGyro[1] * distEstimator.accelSensitivity;
  
  // Remove calibration offset
  float accelX = accelX_raw - distEstimator.accelOffsetX;
  float accelY = accelY_raw - distEstimator.accelOffsetY;
  
  // Apply dead zone to filter small accelerations (noise)
  const float ACCEL_DEADZONE = 0.05f;  // m/s²
  if (fabs(accelX) < ACCEL_DEADZONE) accelX = 0;
  if (fabs(accelY) < ACCEL_DEADZONE) accelY = 0;
  
  // Update velocity: v = v_old + a * dt
  distEstimator.velocityX += accelX * dt;
  distEstimator.velocityY += accelY * dt;
  
  // Apply velocity damping to reduce drift
  const float VELOCITY_DAMPING = 0.95f;
  distEstimator.velocityX *= VELOCITY_DAMPING;
  distEstimator.velocityY *= VELOCITY_DAMPING;
  
  // Update position: s = v * dt (in meters, then convert to mm)
  float distX_mm = distEstimator.velocityX * dt * 1000.0f;
  float distY_mm = distEstimator.velocityY * dt * 1000.0f;
  
  distEstimator.distanceX += distX_mm;
  distEstimator.distanceY += distY_mm;
  
  // Calculate total distance (Euclidean)
  distEstimator.totalDistance = sqrt(distEstimator.distanceX * distEstimator.distanceX + 
                                     distEstimator.distanceY * distEstimator.distanceY);
}

void resetDistance() {
  distEstimator.velocityX = 0.0f;
  distEstimator.velocityY = 0.0f;
  distEstimator.distanceX = 0.0f;
  distEstimator.distanceY = 0.0f;
  distEstimator.totalDistance = 0.0f;
  distEstimator.lastUpdateTime = millis();
  
  Serial.println("Distance reset to zero");
}

float getDistanceMM() {
  return distEstimator.totalDistance;
}

int getDistanceUnits() {
  // Round to nearest unit
  return (int)(distEstimator.totalDistance / distEstimator.distanceUnit + 0.5f);
}

void startDistanceTracking() {
  distEstimator.isTracking = true;
  distEstimator.lastUpdateTime = millis();
  Serial.println("Distance tracking started");
}

void stopDistanceTracking() {
  distEstimator.isTracking = false;
  Serial.print("Distance tracking stopped. Total: ");
  Serial.print(distEstimator.totalDistance);
  Serial.print("mm (");
  Serial.print(getDistanceUnits());
  Serial.println(" units)");
}
