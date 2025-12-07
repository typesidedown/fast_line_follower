#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

void initIMU();
void calibrateGyroZ(int samples = 500);
float updateYawDeg();
extern float yawDeg;

// ============================================================================
// DISTANCE ESTIMATION
// ============================================================================
struct DistanceEstimator {
  float accelOffsetX;        // Calibrated X acceleration baseline
  float accelOffsetY;        // Calibrated Y acceleration baseline
  float velocityX;           // Current velocity in X direction (m/s)
  float velocityY;           // Current velocity in Y direction (m/s)
  float distanceX;           // Distance traveled in X direction (mm)
  float distanceY;           // Distance traveled in Y direction (mm)
  float totalDistance;       // Total distance (mm)
  unsigned long lastUpdateTime;
  float accelSensitivity;    // LSB to m/s²: typically 1/2048 for ±16g
  int distanceUnit;          // Unit constant for maze distances (e.g., 50mm per unit)
  bool isTracking;           // Whether distance tracking is active
};

extern DistanceEstimator distEstimator;

void initDistanceEstimator(int unitSize = 50);  // unitSize in mm
void calibrateAccelerometer(int samples = 500);
void updateDistance();
void resetDistance();
float getDistanceMM();
int getDistanceUnits();
void stopDistanceTracking();
void startDistanceTracking();

#endif // IMU_H
