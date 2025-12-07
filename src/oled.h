#ifndef OLED_H
#define OLED_H

#include <Arduino.h>

void initOLED();
void oledPrintLine(int row, const char* msg);
void oledDisplayStatus(float yaw, float error, int leftSpeed, int rightSpeed);
void oledDisplayIRReadings(uint16_t irReadings[8]);
void oledDisplayRawIRReadings(uint16_t rawIrReadings[8]);

// Maze mode displays
void oledDisplayMazeIdle();
void oledDisplayDryRunStatus(int junctionCount, int dryRunStep, float distance);
void oledDisplayFinalRunStatus(int currentStep, int totalSteps, float distance);
void oledDisplayMazeComplete(float dryRunDist, float finalRunDist);

#endif // OLED_H
