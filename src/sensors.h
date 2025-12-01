#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <vector>
#include "config.h"

// IR sensor data exported
extern uint16_t irReadings[8];
extern uint16_t rawIrReadings[8];
extern int arr_rows;
extern int arr_cols;
extern std::vector<std::vector<int>> irValues;
extern bool Endpoint;

void initSensors();
int muxSelect(uint8_t channel);
void muxEnable(bool enable);
void readIRArray();
float calculate_error();
int detect_turn(uint16_t irR[8] = irReadings);
void followLine();
void keep_track_of_endpoint(std::vector<int> ir_array);
float calculatePDCorrection(float error);

// Runtime tuning functions
void setIRThreshold(uint8_t channel, uint16_t threshold);
void setControlGains(float kp, float kd);
void setBaseSpeed(int speed);
void setTurnDelta(int delta);
uint16_t getRawIRReading(uint8_t channel);
uint8_t getDigitalIRReading(uint8_t channel);

#endif // SENSORS_H
