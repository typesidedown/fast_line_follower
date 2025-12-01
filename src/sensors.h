#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <vector>

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
// void setIRThreshold(uint16_t thresh);
// uint16_t getIRThreshold();
float calculate_error();
int detect_turn(uint16_t irR[8] = irReadings);
void followLine();
void keep_track_of_endpoint(std::vector<int> ir_array);

#endif // SENSORS_H
