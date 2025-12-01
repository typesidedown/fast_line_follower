#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include "config.h"

// Motor pin definitions are private to movement.cpp
// All motor control goes through public functions

void initMovement();
void stopMotors();
void moveForward(uint8_t speed);
void moveBackward(uint8_t speed);
void turnRight(uint8_t speed);
void turnLeft(uint8_t speed);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void setMotorCommand(int leftSpeed, int rightSpeed, bool applyOffset);
int getLeftSpeed();
int getRightSpeed();

#endif // MOVEMENT_H
