#include "sensors.h"
#include "movement.h"
#include <Arduino.h>

// MUX pins and settings
const int MUX_ADDR0 = 11;
const int MUX_ADDR1 = 10;
const int MUX_ADDR2 = 9;
const int MUX_ADDR3 = 8;
const int MUX_ENABLE_PIN = 14;
const int MUX_ENABLE_ACTIVE_LOW = 0;
const int MUX_SIG_PIN = 15;
const int IR_SENSORS_ARE_DIGITAL = 1;

// raw analog readings from mux
uint16_t rawIrReadings[8];
uint16_t irReadings[8];
const bool IR_INVERTED = true; // true if raw value is LOW when on the line

// threshold for converting analog->digital
static uint16_t irThreshold[8] = {900,700,700,700,700,700,700,700};
int arr_rows = 8;
int arr_cols = 8;
std::vector<std::vector<int>> irValues(arr_rows, std::vector<int>(arr_cols));
bool Endpoint = false;

// Line following PD parameters
const int BASE_SPEED = 100;
const int MAX_SPEED = 120;
const int MIN_SPEED = 80;
const float KP = 10.0f;
const float KD = 10.0f;
const int SIDE_DETECT_CONSECUTIVE = 7;

void initSensors() {
  pinMode(MUX_ADDR0, OUTPUT);
  pinMode(MUX_ADDR1, OUTPUT);
  pinMode(MUX_ADDR2, OUTPUT);
  pinMode(MUX_ADDR3, OUTPUT);
  pinMode(MUX_ENABLE_PIN, OUTPUT);
  pinMode(MUX_SIG_PIN, INPUT);
  muxEnable(false);
}

int muxSelect(uint8_t channel){
  int controlBits[16][4] = {
    {0,0,0,0}, {1,0,0,0}, {0,1,0,0}, {1,1,0,0},
    {0,0,1,0}, {1,0,1,0}, {0,1,1,0}, {1,1,1,0},
    {0,0,0,1}, {1,0,0,1}, {0,1,0,1}, {1,1,0,1},
    {0,0,1,1}, {1,0,1,1}, {0,1,1,1}, {1,1,1,1}
  };

  digitalWrite(MUX_ADDR0, controlBits[channel][0]);
  digitalWrite(MUX_ADDR1, controlBits[channel][1]);
  digitalWrite(MUX_ADDR2, controlBits[channel][2]);
  digitalWrite(MUX_ADDR3, controlBits[channel][3]);

  delayMicroseconds(5);
  return analogRead(MUX_SIG_PIN);
}

void muxEnable(bool enable){
  if(MUX_ENABLE_ACTIVE_LOW){
    digitalWrite(MUX_ENABLE_PIN, enable ? LOW : HIGH);
  } else {
    digitalWrite(MUX_ENABLE_PIN, enable ? HIGH : LOW);
  }
}

void readIRArray(){
  for(uint8_t ch=0; ch<8; ch++){
    uint16_t raw = muxSelect(ch);
    rawIrReadings[ch] = raw;
    if (IR_INVERTED) {
      irReadings[ch] = (raw < irThreshold[ch]) ? 1 : 0;
    } else {
      irReadings[ch] = (raw >= irThreshold[ch]) ? 1 : 0;
    }
  }
}

// void setIRThreshold(uint16_t thresh) {
//   irThreshold[] = thresh;
// }

// uint16_t getIRThreshold() {
//   return irThreshold;
// }

int detect_turn(uint16_t irR[8]){
  int line_on_right = 0;
  int line_on_left = 0;
  int str_line = 0;
  if (irR[0] == 1 && irR[1] == 1 && irR[2] == 1) {
    line_on_right = 1; // left turn
  }
  if (irR[5] == 1 && irR[6] == 1 && irR[7] == 1) {
    line_on_left = 1; // right turn
  }
  if (irR[3] == 1 && irR[4] == 1) {
    str_line = 1; // no turn
  }
  if (line_on_right == 0 && line_on_left == 0 ){
    Serial.println("No turn detected");
    return 0;
  } else if (line_on_right == 1 && line_on_left == 0){
    Serial.println("Right turn detected");
    return 1;
  } else if (line_on_left == 1 && line_on_right == 0){
    Serial.println("Left turn detected");
    return -1;
  } else {
    return 2;
  }
}

float calculate_error(){
  float err = 0.0f;
  for (int i = 0; i < 8; i++) {
    if (i < 3){
      if (irReadings[i] == 1){
        err = err - (4 - i);
      }
    }
    if (i > 4){
      if (irReadings[i] == 1){
        err = err + (i - 3);
      }
    }
  }
  return err;
}

void followLine() {
  static float lastError = 0.0f;
  float error = calculate_error();
  float derivative = error - lastError;
  lastError = error;

  float correction = (KP * error) + (KD * derivative);

  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  setMotorSpeeds(leftSpeed, rightSpeed);
}

void keep_track_of_endpoint(std::vector<int> ir_array ){
  for (int row = 0; row < arr_rows - 1; row++) {
    for (int col = 0; col < arr_cols; col++) {
      irValues[row][col] = irValues[row + 1][col];
    }
  }
  for (int col = 0; col < arr_cols; col++) {
    irValues[0][col] = ir_array[col];
  }
  int endpoint_count = 0;
  for (int row = 0; row < arr_rows; row++) {
    bool all_on_line = true;
    for (int col = 0; col < arr_cols; col++) {
      if (irValues[row][col] == 0) {
        all_on_line = false;
        break;
      }
    }
    if (all_on_line) {
      endpoint_count++;
    }
  }
  if (endpoint_count >= SIDE_DETECT_CONSECUTIVE) {
    Endpoint = true;
  } else {
    Endpoint = false;
  }
}
