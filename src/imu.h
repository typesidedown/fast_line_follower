#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

void initIMU();
void calibrateGyroZ(int samples = 500);
float updateYawDeg();
extern float yawDeg;

#endif // IMU_H
