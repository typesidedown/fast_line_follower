#ifndef OLED_H
#define OLED_H

#include <Arduino.h>

void initOLED();
void oledPrintLine(int row, const char* msg);
void oledDisplayStatus(float yaw, float error, int leftSpeed, int rightSpeed);
void oledDisplayIRReadings(uint16_t irReadings[8]);
void oledDisplayRawIRReadings(uint16_t rawIrReadings[8]);

#endif // OLED_H
