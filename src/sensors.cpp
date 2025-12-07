#include "sensors.h"
#include "movement.h"
#include "config.h"
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

// Endpoint tracking
int arr_rows = 8;
int arr_cols = 8;
std::vector<std::vector<int>> irValues(arr_rows, std::vector<int>(arr_cols));
bool Endpoint = false;

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

void readIRArray() {
  for (uint8_t ch = 0; ch < 8; ch++) {
    uint16_t raw = muxSelect(ch);
    rawIrReadings[ch] = raw;
    if (IR_INVERTED) {
      irReadings[ch] = (raw < sensorConfig.irThreshold[ch]) ? 1 : 0;
    } else {
      irReadings[ch] = (raw >= sensorConfig.irThreshold[ch]) ? 1 : 0;
    }
  }
}


// ============================================================================
// Turn Detection with Debouncing
// ============================================================================
int detect_turn(uint16_t irR[8]) {
  int leftCount = 0;
  int rightCount = 0;
  int centerCount = 0;

  // Count active sensors by region
  for (int i = LEFT_SENSORS_START; i <= LEFT_SENSORS_END; i++) {
    if (irR[i] == 1) leftCount++;
  }
  for (int i = RIGHT_SENSORS_START; i <= RIGHT_SENSORS_END; i++) {
    if (irR[i] == 1) rightCount++;
  }

  centerCount = irR[CENTER_LEFT_SENSOR] + irR[CENTER_RIGHT_SENSOR];

  int currentDetection = 0;

  // Priority: left turn > right turn > straight > ambiguous
  if (leftCount >= LEFT_TURN_THRESHOLD && rightCount < RIGHT_TURN_THRESHOLD) {
    currentDetection = -1;  // LEFT TURN
  }
  else if (rightCount >= RIGHT_TURN_THRESHOLD && leftCount < LEFT_TURN_THRESHOLD) {
    currentDetection = 1;   // RIGHT TURN
  }
  else if (centerCount+leftCount+rightCount >= CENTER_STRAIGHT_THRESHOLD) {
    currentDetection = 0;   // LINE FOLLOW
  }
  else if(centerCount + rightCount + leftCount == 0){
    currentDetection = -2;
  }
  else if(centerCount + rightCount + leftCount == 8){
    currentDetection = 1;
  }
  else {
    currentDetection = 2;   // AMBIGUOUS/ERROR STATE
  }

  // Debouncing: only report if same detection for N consecutive frames
  if (currentDetection == turnState.lastTurnDetected) {
    turnState.consecutiveFrames++;
  } else {
    turnState.consecutiveFrames = 1;
    turnState.lastTurnDetected = currentDetection;
  }

  if (turnState.consecutiveFrames < turnState.debounceThreshold) {
    return 0;  // Unconfirmed, treat as straight
  }

  return currentDetection;
}

float calculate_error() {
  float err = 0.0f;
  int activeCount = 0;

  // Weighted error: left sensors negative, right sensors positive
  for (int i = LEFT_SENSORS_START; i <= LEFT_SENSORS_END; i++) {
    if (irReadings[i] == 1) {
      err += -(4 - i);  // -3, -2, -1
      activeCount++;
    }
  }
  for (int i = RIGHT_SENSORS_START; i <= RIGHT_SENSORS_END; i++) {
    if (irReadings[i] == 1) {
      err += (i - 3);   // 2, 3, 4
      activeCount++;
    }
  }

  // Track diagnostics
  if (activeCount > 0) {
    diagnostics.maxError = max(diagnostics.maxError, fabs(err));
    diagnostics.minError = min(diagnostics.minError, fabs(err));
  }

  return err;
}

// ============================================================================
// Improved PD Controller with Low-Pass Filtering
// ============================================================================
float calculatePDCorrection(float error) {
  static float lastFilteredDerivative = 0;

  // Proportional term
  float pTerm = pdController.kp * error;

  // Derivative term with low-pass filtering to reduce noise
  float rawDerivative = error - pdController.lastError;

  float filteredDerivative;
  if (pdController.useDerivativeLowPass) {
    filteredDerivative = 
      pdController.lowPassAlpha * rawDerivative + 
      (1.0f - pdController.lowPassAlpha) * lastFilteredDerivative;
    lastFilteredDerivative = filteredDerivative;
  } else {
    filteredDerivative = rawDerivative;
  }

  float dTerm = pdController.kd * filteredDerivative;

  pdController.lastError = error;

  // Combined correction with saturation
  float correction = pTerm + dTerm;
  correction = constrain(correction, -pdController.maxCorrection, pdController.maxCorrection);

  return correction;
}

void followLine() {
  float error = calculate_error();
  float correction = calculatePDCorrection(error);

  int leftSpeed = sensorConfig.baseSpeed - correction;
  int rightSpeed = sensorConfig.baseSpeed + correction;

  leftSpeed = constrain(leftSpeed, sensorConfig.minSpeed, sensorConfig.maxSpeed);
  rightSpeed = constrain(rightSpeed, sensorConfig.minSpeed, sensorConfig.maxSpeed);

  setMotorSpeeds(leftSpeed, rightSpeed);
}

void keep_track_of_endpoint(std::vector<int> ir_array) {
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
  if (endpoint_count >= sensorConfig.turnDetectConsecutive) {
    Endpoint = true;
  } else {
    Endpoint = false;
  }
}

// ============================================================================
// Runtime Tuning Functions
// ============================================================================
void setIRThreshold(uint8_t channel, uint16_t threshold) {
  if (channel < 8) {
    sensorConfig.irThreshold[channel] = constrain(threshold, 100, 1023);
    Serial.print("Set IR[");
    Serial.print(channel);
    Serial.print("] threshold to ");
    Serial.println(sensorConfig.irThreshold[channel]);
  }
}

void setControlGains(float kp, float kd) {
  sensorConfig.kp = max(0.0f, kp);
  sensorConfig.kd = max(0.0f, kd);
  pdController.kp = sensorConfig.kp;
  pdController.kd = sensorConfig.kd;
  Serial.print("Control gains updated: KP=");
  Serial.print(sensorConfig.kp);
  Serial.print(" KD=");
  Serial.println(sensorConfig.kd);
}

void setBaseSpeed(int speed) {
  sensorConfig.baseSpeed = constrain(speed, 30, 200);
  Serial.print("Base speed set to: ");
  Serial.println(sensorConfig.baseSpeed);
}

void setTurnDelta(int delta) {
  motorConfig.turnDelta = constrain(delta, 10, 100);
  Serial.print("Turn delta set to: ");
  Serial.println(motorConfig.turnDelta);
}

uint16_t getRawIRReading(uint8_t channel) {
  if (channel < 8) return rawIrReadings[channel];
  return 0;
}

uint8_t getDigitalIRReading(uint8_t channel) {
  if (channel < 8) return irReadings[channel];
  return 0;
}
