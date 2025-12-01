#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>

extern const int AIN1;
extern const int AIN2;
extern const int PWMA;
extern const int BIN1;
extern const int BIN2;
extern const int PWMB;
extern const int STBY;
extern int Offset_motor_right;
extern int delta;

void initMovement();
void stopMotors();
void moveForward(uint8_t speed);
void moveBackward(uint8_t speed);
void turnRight(uint8_t speed);
void turnLeft(uint8_t speed);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
int getLeftSpeed();
int getRightSpeed();

#endif // MOVEMENT_H
