#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <DFRobot_BMI160.h>
#include <vector>
// #include <iostream>
DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

#define MUX_ADDR0 11
#define MUX_ADDR1 10
#define MUX_ADDR2 9
#define MUX_ADDR3 8
#define MUX_ENABLE_PIN 14
#define MUX_ENABLE_ACTIVE_LOW 1
#define MUX_SIG_PIN 15
#define Offset_motor_right 0

// If your sensors are digital (on/off) set to 1, otherwise 0 for analog reads
#define IR_SENSORS_ARE_DIGITAL 0

//.......................................................
float yawDeg = 0.0f;            // current yaw angle in degrees
float gyroZOffset = 0.0f;       // gyro Z bias (°/s)
unsigned long lastYawTime = 0;  // for dt calculation

// change this if your gyro range is different
// For BMI160 at ±2000 °/s -> 16.4 LSB per °/s
const float GYRO_SENSITIVITY = 16.4f; 

/////////////////////////////////////////////////
void calibrateGyroZ(int samples = 500) {
  long sum = 0;
  int16_t accelGyro[6];

  for (int i = 0; i < samples; i++) {
    if (bmi160.getAccelGyroData(accelGyro) == 0) {
      // Z gyro is accelGyro[2] (first 3 are gyro, as in your code)
      sum += accelGyro[2];
    }
    delay(2); // small delay between samples
  }

  float avgRaw = (float)sum / samples;
  gyroZOffset = avgRaw / GYRO_SENSITIVITY;  // convert to °/s
}

float updateYawDeg() {
  int16_t accelGyro[6] = {0};
  int rslt = bmi160.getAccelGyroData(accelGyro);
  if (rslt != 0) {
    // on error, just return last yaw
    return yawDeg;
  }

  unsigned long now = millis();
  float dt = (now - lastYawTime) / 1000.0f; // seconds
  lastYawTime = now;

  // raw Z gyro -> °/s
  float gz_dps = (float)accelGyro[2] / GYRO_SENSITIVITY;

  // remove bias from calibration
  gz_dps -= gyroZOffset;

  // integrate to get angle
  yawDeg += gz_dps * dt;

  // wrap to -180 .. 180
  if (yawDeg > 180.0f)      yawDeg -= 360.0f;
  else if (yawDeg < -180.0f) yawDeg += 360.0f;

  return yawDeg;
}


//.......................................................

uint16_t irReadings[8];
int arr_rows = 8;
int arr_cols = 8;
std::vector<std::vector<int>> irValues(arr_rows, std::vector<int>(arr_cols));
bool Endpoint = false;
#define IR_INVERTED 0 // set to 1 if raw sensor value is low when on the line
#define NO_LINE_SUM_THRESHOLD 0.02f
int delta = 30;

// Sensor weights to reduce influence of certain sensors (indices 0..7)
// Reduce weight for 3rd and 6th sensors (indices 2 and 5) to avoid over-correction
const float sensorWeights[8] = {
  1.0f, // sensor 0
  1.0f, // sensor 1
  1.0f, // sensor 2 (3rd sensor) - reduced influence
  1.0f, // sensor 3
  1.0f, // sensor 4
  1.0f, // sensor 5 (6th sensor) - reduced influence
  1.0f, // sensor 6
  1.0f  // sensor 7
};

// Select the given channel (0..7) on the mux
void muxSelect(uint8_t channel){
  digitalWrite(MUX_ADDR0, channel & 0x01);
  digitalWrite(MUX_ADDR1, (channel >> 1) & 0x01);
  digitalWrite(MUX_ADDR2, (channel >> 2) & 0x01);
  digitalWrite(MUX_ADDR3, (channel >> 3) & 0x01);
  
  // small settle delay to let the mux output stabilize
  delayMicroseconds(5);
}

// Enable or disable the mux output (depending on active low/high)
void muxEnable(bool enable){
  if(MUX_ENABLE_ACTIVE_LOW){
    digitalWrite(MUX_ENABLE_PIN, enable ? LOW : HIGH);
  }else{
    digitalWrite(MUX_ENABLE_PIN, enable ? HIGH : LOW);
  }
}

// Read all 8 sensors into irReadings[]
void readIRArray(){
  muxEnable(true);
  for(uint8_t ch=0; ch<8; ++ch){
    muxSelect(ch);
    // digital IR sensors only
    irReadings[ch] = analogRead(MUX_SIG_PIN);
  }
  muxEnable(false);

}

// Compute signed normalized error [-1 .. 1] using sensors 0..7 with center between 3 and 4


// ----- Motor Driver Pins -----
const int AIN1 = 3, AIN2 = 2, PWMA = 1;
const int BIN1 = 5, BIN2 = 6, PWMB = 7;
const int STBY = 4;

// Line following PD parameters
#define BASE_SPEED 100     // Base speed for straight line (0-255)
#define MAX_SPEED 120      // Maximum allowed speed (0-255)
#define MIN_SPEED 80       // Minimum allowed speed (0-255)
#define KP 70.0            // Proportional gain - adjust for responsiveness
#define KD 10.0            // Derivative gain - damps oscillations

// Side-detection and turning settings
#define SIDE_DETECT_CONSECUTIVE 7   // number of consecutive reads required

// Motor control helpers
void stopMotors(){
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void moveForward(uint8_t speed){
  speed = constrain(speed, 0, 255);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);  // Left motor reversed
  analogWrite(PWMA, speed);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  // Right motor normal
  analogWrite(PWMB, speed);
  digitalWrite(STBY, HIGH);
}

void moveBackward(uint8_t speed){
  speed = constrain(speed, 0, 255);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  // Left motor reversed
  analogWrite(PWMA, speed);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);  // Right motor normal
  analogWrite(PWMB, speed);
  digitalWrite(STBY, HIGH);
}

void turnRight(uint8_t speed){
  speed = constrain(speed, 0, 255);
  // left wheel backward (reversed), right wheel forward
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);  // Left motor reversed
  analogWrite(PWMA, speed + delta);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMB, speed - delta);
  digitalWrite(STBY, HIGH);
  yawDeg=0;
  while (yawDeg>-90){
    updateYawDeg();
  }
  stopMotors();
}

void turnLeft(uint8_t speed){
  speed = constrain(speed, 0, 255);
  // left wheel forward (reversed), right wheel backward
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);  // Left motor reversed
  analogWrite(PWMA, speed - delta);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMB, speed + delta);
  digitalWrite(STBY, HIGH);
  yawDeg=0;
  while (yawDeg<90){
    updateYawDeg();
  }
  stopMotors();
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);  // Left forward (reversed)
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  // Right forward
  analogWrite(PWMA, leftSpeed - Offset_motor_right);
  analogWrite(PWMB, rightSpeed + Offset_motor_right);
  digitalWrite(STBY, HIGH);
  delay(1);
}

int detect_turn(uint16_t irReadings[8] = irReadings){
    /*
  left: -1
  right: 1
  no turn: 0
  both: 2
  */
  int line_on_right = 0;
  int line_on_left = 0;
  int str_line = 0;
  if (irReadings[0] == 1 && irReadings[1] == 1 && irReadings[2] == 1) {
    line_on_left = 1; // left turn
  } 
  if (irReadings[5] == 1 && irReadings[6] == 1 && irReadings[7] == 1) {
    line_on_right = 1; // right turn
  } 
  if (irReadings[3] == 1 && irReadings[4] == 1) {
    str_line = 1; // no turn
  }
  if (line_on_right == 0 && line_on_left == 0 ){
    return 0;
  } else if (line_on_right == 1 && line_on_left == 0){
    return 1;
  } else if (line_on_left == 1 && line_on_right == 0){
    return -1;
  } else {
    return 2;
  }
}

float calculate_error(){
  float err;
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

  // PD controller
  float correction = (KP * error) + (KD * derivative);

  // Compute motor speeds
  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  // Constrain speeds to min/max
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  // Set motor speeds
  setMotorSpeeds(leftSpeed, rightSpeed);

}
void keep_track_of_endpoint(std::vector<int> ir_array ){
  // Shift existing readings down
  for (int row = 0; row < arr_rows - 1; row++) {
    for (int col = 0; col < arr_cols; col++) {
      irValues[row][col] = irValues[row + 1][col];   
    }
  }
  for (int col = 0; col < arr_cols; col++) {
    irValues[0][col] = ir_array[col]; // Add new reading at the end
  }
  int endpoint_count = 0;
  for (int row = 0; row < arr_rows; row++) {
    bool all_on_line = true;
    for (int col = 0; col < arr_cols; col++) {
      if (irValues[row][col] == 0) { // Assuming 1 means on the line
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

void setup() {
  Serial.begin(115200);

  // Initialize MUX pins
  pinMode(MUX_ADDR0, OUTPUT);
  pinMode(MUX_ADDR1, OUTPUT);
  pinMode(MUX_ADDR2, OUTPUT);
  pinMode(MUX_ADDR3, OUTPUT);
  pinMode(MUX_ENABLE_PIN, OUTPUT);
  // signal pin: either analog input or digital input
  pinMode(MUX_SIG_PIN, INPUT);
  // disable mux until we need it
  muxEnable(false);

  //Motor driver pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);


  Wire.begin();
  if (bmi160.softReset() != BMI160_OK){
    Serial.println("BMI160 reset failed");
  } else {
    Serial.println("BMI160 reset OK");
  }
  delay(50);
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("BMI160 init failed");
  } else {
    Serial.println("BMI160 init OK");
  }
  //.............................................
  delay(1000);        // let sensor settle
  calibrateGyroZ();   // calibrate while robot is still
  lastYawTime = millis();
  // ................................................
}

void loop() {
  Serial.print("IR: ");
  for(uint8_t i=0;i<8;i++){
    Serial.print(irReadings[i]);
    if(i<7) Serial.print(",");
  }
  Serial.println();
  //...........................................................................
  // delay(500);
  // turnRight(50);
  // delay(3000);
  // turnLeft(50);
  // delay(3000);
  // //...........................................................................

  float yaw = updateYawDeg();
  Serial.println(yaw);  // -180 .. 180

  // your other stuff here
  delay(10);
  // ............................................................................
  float err = calculate_error();
  printf("Error: %.3f\n", err);
  while (!Endpoint)
  {
    int turn = detect_turn();
    if (turn == -1){
      turnLeft(100);
    } else if (turn == 1){
      turnRight(100);
    } else if (turn == 0){
      followLine();
    } else {
      stopMotors();
    }
  }
}