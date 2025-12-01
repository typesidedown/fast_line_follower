#ifndef OLED_H
#define OLED_H

#include <Arduino.h>

void initOLED();
void oledPrintLine(int row, const char* msg);
void oledDisplayStatus(float yaw, float error, int leftSpeed, int rightSpeed);

#endif // OLED_H
