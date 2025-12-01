# Line Follower Bot - Code Analysis & Recommendations

## Executive Summary
Your line follower project has a solid foundation with good modularization. However, there are several critical robustness issues and tuning limitations that should be addressed for reliable operation and easier parameter adjustment.

---

## Critical Issues & Recommendations

### 1. **SENSORS.CPP - Hardcoded Magic Numbers (Robustness ⚠️)**

**Current Problems:**
- IR threshold values are hardcoded: `{900,700,700,700,700,700,700,700}`
- PD controller gains (KP=10, KD=10) are constants with no way to tune
- Speed limits (BASE_SPEED=100, MAX_SPEED=120, MIN_SPEED=80) cannot be adjusted without recompilation
- Line detection logic is fragile with magic indices (0,1,2,5,6,7)

**Recommendations:**

```cpp
// Create a tunable configuration structure
struct SensorConfig {
  uint16_t irThreshold[8];
  float kp;
  float kd;
  int baseSpeed;
  int maxSpeed;
  int minSpeed;
  int turnDetectConsecutive;
};

extern SensorConfig sensorConfig;

// Initialize with defaults
SensorConfig sensorConfig = {
  {900, 700, 700, 700, 700, 700, 700, 700},  // irThreshold
  10.0f,      // kp
  10.0f,      // kd
  100,        // baseSpeed
  120,        // maxSpeed
  80,         // minSpeed
  7           // turnDetectConsecutive
};

// Function to update thresholds at runtime
void setIRThreshold(uint8_t channel, uint16_t threshold) {
  if (channel < 8) {
    sensorConfig.irThreshold[channel] = threshold;
  }
}

void setControlGains(float kp, float kd) {
  sensorConfig.kp = kp;
  sensorConfig.kd = kd;
}
```

---

### 2. **MOVEMENT.CPP - Motor Calibration Issues (Robustness ⚠️)**

**Current Problems:**
- Motor offset handling is inconsistent:
  - `turnRight()` and `turnLeft()` use raw `delta` without `Offset_motor_right`
  - `setMotorSpeeds()` applies offset differently than `moveForward()`
  - No validation that motors are actually moving
- Turn completion relies solely on 90° threshold - no momentum/overshoot handling
- Hard-coded 4-second timeout has no failure recovery

**Recommendations:**

```cpp
struct MotorConfig {
  int offsetMotorRight;  // Motor balance correction
  int turnDelta;         // Speed difference for 90° turns
  int minPWM;            // Minimum PWM to overcome static friction
  int maxPWM;            // Safety limit (typically 255)
  unsigned long turnTimeoutMs;
  float turnAngleDegrees; // Target turn angle (usually 90)
};

extern MotorConfig motorConfig;

// In movement.cpp initialization
MotorConfig motorConfig = {
  0,          // offsetMotorRight
  30,         // turnDelta
  40,         // minPWM threshold
  255,        // maxPWM
  4000,       // turnTimeoutMs
  90.0f       // turnAngleDegrees
};

// Consistent motor command function
void setMotorCommand(int leftSpeed, int rightSpeed, bool applyOffset = true) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Apply offset consistently
  if (applyOffset) {
    leftSpeed = constrain(leftSpeed - motorConfig.offsetMotorRight, 0, 255);
    rightSpeed = constrain(rightSpeed + motorConfig.offsetMotorRight, 0, 255);
  }

  digitalWrite(AIN1, LOW); 
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, leftSpeed);
  digitalWrite(BIN1, HIGH); 
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, rightSpeed);
  digitalWrite(STBY, HIGH);
  
  lastLeftSpeed = leftSpeed;
  lastRightSpeed = rightSpeed;
}

// Use this for motor control functions
void moveForward(uint8_t speed) {
  speed = constrain(speed, 0, 255);
  setMotorCommand(speed, speed);
}

// Improved turn logic with overshoot protection
void turnRight(uint8_t speed) {
  speed = constrain(speed, 0, 255);
  
  extern float yawDeg;
  yawDeg = 0;
  
  unsigned long start = millis();
  float targetAngle = motorConfig.turnAngleDegrees;
  bool turnComplete = false;
  float peakYaw = 0;

  while (!turnComplete && (millis() - start) < motorConfig.turnTimeoutMs) {
    updateYawDeg();
    
    // Slow down as approaching target (proportional slowdown)
    float angleDiff = fabs(yawDeg) - targetAngle;
    float speedFactor = constrain(1.0f - (angleDiff / 10.0f), 0.4f, 1.0f);
    
    int adjustedSpeed = (int)(speed * speedFactor);
    adjustedSpeed = constrain(adjustedSpeed, motorConfig.minPWM, speed);
    
    int leftPwm = constrain(adjustedSpeed + motorConfig.turnDelta, 0, 255);
    int rightPwm = constrain(adjustedSpeed - motorConfig.turnDelta, 0, 255);
    
    setMotorCommand(leftPwm, rightPwm, false); // Already balanced
    
    peakYaw = fmax(peakYaw, fabs(yawDeg));
    
    // Exit when target reached
    if (fabs(yawDeg) >= targetAngle) {
      turnComplete = true;
    }
    
    delay(5);
  }
  
  // Timeout or completed
  if (!turnComplete) {
    Serial.print("Turn timeout! Reached: ");
    Serial.print(peakYaw);
    Serial.println(" degrees");
  }
  
  stopMotors();
}
```

---

### 3. **LINE DETECTION LOGIC - Fragile & Ambiguous (Robustness ⚠️)**

**Current Problems:**
- `detect_turn()` has overlapping conditions (what if all 8 sensors see line?)
- Magic indices (0,1,2) vs (5,6,7) with no constants
- No debouncing for noisy sensor readings
- Error calculation in `calculate_error()` only uses active sensors, ignoring inactive ones

**Recommendations:**

```cpp
// In sensors.h
#define LEFT_SENSORS_START 0
#define LEFT_SENSORS_END 2
#define CENTER_LEFT_SENSOR 3
#define CENTER_RIGHT_SENSOR 4
#define RIGHT_SENSORS_START 5
#define RIGHT_SENSORS_END 7

// Detection constants
#define LEFT_TURN_THRESHOLD 3     // 3 left sensors on line
#define RIGHT_TURN_THRESHOLD 3    // 3 right sensors on line
#define CENTER_STRAIGHT_THRESHOLD 2  // At least 2 center sensors

// Debouncing structure
struct TurnDetectionState {
  int lastTurnDetected;
  int consecutiveFrames;
  int debounceThreshold;  // e.g., 3 frames
};

extern TurnDetectionState turnState;

// In sensors.cpp
TurnDetectionState turnState = {0, 0, 3};

int detect_turn(uint16_t irR[8]) {
  int leftCount = 0, rightCount = 0, centerCount = 0;
  
  // Count active sensors by region
  for (int i = LEFT_SENSORS_START; i <= LEFT_SENSORS_END; i++) {
    if (irR[i] == 1) leftCount++;
  }
  for (int i = RIGHT_SENSORS_START; i <= RIGHT_SENSORS_END; i++) {
    if (irR[i] == 1) rightCount++;
  }
  
  centerCount = irR[CENTER_LEFT_SENSOR] + irR[CENTER_RIGHT_SENSOR];
  
  int currentDetection = 0;
  
  // Priority: left turn > right turn > no turn
  if (leftCount >= LEFT_TURN_THRESHOLD && rightCount < RIGHT_TURN_THRESHOLD) {
    currentDetection = -1;  // LEFT TURN
  } 
  else if (rightCount >= RIGHT_TURN_THRESHOLD && leftCount < LEFT_TURN_THRESHOLD) {
    currentDetection = 1;   // RIGHT TURN
  } 
  else if (centerCount >= CENTER_STRAIGHT_THRESHOLD) {
    currentDetection = 0;   // STRAIGHT
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

// Improved error calculation
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
  
  // Normalize if no sensors active
  if (activeCount == 0) {
    err = 0.0f;  // Or maintain last known error direction
  }
  
  return err;
}
```

---

### 4. **MAIN.CPP - No Runtime Tuning Interface (Ergonomics ⚠️)**

**Current Problems:**
- Serial output is verbose but no input parsing
- Cannot adjust parameters without USB flashing
- No way to test motor calibration without code changes
- No diagnostic commands

**Recommendations:**

```cpp
// Add serial command parser to main.cpp
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("kp=")) {
      float kp = command.substring(3).toFloat();
      sensorConfig.kp = kp;
      Serial.print("KP set to: ");
      Serial.println(kp);
    }
    else if (command.startsWith("kd=")) {
      float kd = command.substring(3).toFloat();
      sensorConfig.kd = kd;
      Serial.print("KD set to: ");
      Serial.println(kd);
    }
    else if (command.startsWith("delta=")) {
      int delta = command.substring(6).toInt();
      motorConfig.turnDelta = delta;
      Serial.print("Turn delta set to: ");
      Serial.println(delta);
    }
    else if (command.startsWith("speed=")) {
      int speed = command.substring(6).toInt();
      sensorConfig.baseSpeed = constrain(speed, 40, 150);
      Serial.print("Base speed set to: ");
      Serial.println(sensorConfig.baseSpeed);
    }
    else if (command == "motor_test_forward") {
      moveForward(100);
      delay(1000);
      stopMotors();
      Serial.println("Motor test forward done");
    }
    else if (command == "motor_test_left") {
      turnLeft(100);
      Serial.println("Motor test left done");
    }
    else if (command == "motor_test_right") {
      turnRight(100);
      Serial.println("Motor test right done");
    }
    else if (command == "calibrate_threshold") {
      Serial.println("Place sensor on line, waiting...");
      delay(2000);
      calibrateThresholds();
    }
    else if (command == "status") {
      Serial.print("KP="); Serial.print(sensorConfig.kp);
      Serial.print(" KD="); Serial.print(sensorConfig.kd);
      Serial.print(" Delta="); Serial.print(motorConfig.turnDelta);
      Serial.print(" BaseSpeed="); Serial.println(sensorConfig.baseSpeed);
    }
    else if (command == "help") {
      Serial.println("Commands:");
      Serial.println("  kp=<float>     - Set proportional gain");
      Serial.println("  kd=<float>     - Set derivative gain");
      Serial.println("  delta=<int>    - Set turn speed delta");
      Serial.println("  speed=<int>    - Set base speed");
      Serial.println("  motor_test_forward");
      Serial.println("  motor_test_left");
      Serial.println("  motor_test_right");
      Serial.println("  calibrate_threshold");
      Serial.println("  status");
    }
  }
}

// In loop():
void loop() {
  handleSerialCommands();
  
  readIRArray();
  // ... rest of loop
}
```

---

### 5. **ERROR STATE HANDLING - Missing (Robustness ⚠️)**

**Current Problems:**
- No validation for impossible sensor states
- IMU drift over time - no recalibration mechanism
- No low-battery detection
- `followLine()` has no bounds checking for extreme errors
- Motor commands don't verify they succeeded

**Recommendations:**

```cpp
enum BotState {
  BOT_IDLE = 0,
  BOT_LINE_FOLLOWING = 1,
  BOT_TURNING = 2,
  BOT_ERROR = 3,
  BOT_CALIBRATING = 4
};

struct DiagnosticData {
  unsigned long loopCount;
  float maxError;
  float minError;
  int turnsFailed;
  int imuFailures;
  unsigned long lastRecalibration;
};

extern BotState currentState;
extern DiagnosticData diagnostics;

// Add validation functions
bool validateSensorReadings(uint16_t irR[8]) {
  // Check if sensor reading is physically possible
  int activeCount = 0;
  for (int i = 0; i < 8; i++) {
    if (irR[i] < 100 || irR[i] > 1023) return false;  // Sanity check
    if (irReadings[i] == 1) activeCount++;
  }
  
  // If all sensors active, likely noise or sensor failure
  if (activeCount == 8) {
    Serial.println("WARNING: All sensors active - possible failure");
    return false;
  }
  
  return true;
}

bool validateIMU() {
  static unsigned long lastSuccessfulRead = millis();
  
  int16_t accelGyro[6] = {0};
  int rslt = bmi160.getAccelGyroData(accelGyro);
  
  if (rslt != 0) {
    diagnostics.imuFailures++;
    if (millis() - lastSuccessfulRead > 5000) {
      currentState = BOT_ERROR;
      Serial.println("ERROR: IMU communication failure");
      return false;
    }
  } else {
    lastSuccessfulRead = millis();
  }
  
  return true;
}
```

---

### 6. **PD CONTROLLER TUNING ISSUES (Robustness ⚠️)**

**Current Problems:**
- No anti-windup for integral component (if added)
- PD derivative term is noisy with sensor jitter
- No saturation logic for extreme errors
- Speed limits don't account for error magnitude

**Recommendations:**

```cpp
struct PDController {
  float kp;
  float kd;
  float lastError;
  int maxCorrection;      // Max speed adjustment
  bool useDerivativeLowPass;
  float lowPassAlpha;     // 0.2-0.5 typical
};

extern PDController pdController;

void initPDController() {
  pdController.kp = sensorConfig.kp;
  pdController.kd = sensorConfig.kd;
  pdController.lastError = 0;
  pdController.maxCorrection = 60;  // Don't adjust by more than 60
  pdController.useDerivativeLowPass = true;
  pdController.lowPassAlpha = 0.3f;
}

float calculatePDCorrection(float error) {
  static float lastFilteredDerivative = 0;
  
  // Proportional term
  float pTerm = pdController.kp * error;
  
  // Derivative term with low-pass filtering
  float rawDerivative = error - pdController.lastError;
  
  if (pdController.useDerivativeLowPass) {
    float filteredDerivative = 
      pdController.lowPassAlpha * rawDerivative + 
      (1.0f - pdController.lowPassAlpha) * lastFilteredDerivative;
    lastFilteredDerivative = filteredDerivative;
    
    float dTerm = pdController.kd * filteredDerivative;
  } else {
    float dTerm = pdController.kd * rawDerivative;
  }
  
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
```

---

## Summary of Changes by File

### **sensors.cpp/h**
- [ ] Create `SensorConfig` struct for runtime tuning
- [ ] Add sensor threshold functions
- [ ] Implement sensor state debouncing
- [ ] Add symbolic constants for sensor indices
- [ ] Improve error calculation robustness

### **movement.cpp/h**
- [ ] Create `MotorConfig` struct
- [ ] Consolidate motor command logic
- [ ] Add proportional speed reduction for turns
- [ ] Improve timeout handling with diagnostics
- [ ] Add minimum PWM threshold handling

### **main.cpp**
- [ ] Add serial command parser for runtime tuning
- [ ] Implement state machine for bot modes
- [ ] Add diagnostic data collection
- [ ] Implement IMU validation

### **New optional files**
- [ ] `config.h` - Centralized configuration
- [ ] `diagnostics.h` - Diagnostic utilities
- [ ] `pid_controller.h` - Advanced PD control

---

## Tuning Guide (Quick Reference)

```
1. **Motor Calibration** (Run first)
   - Use "motor_test_forward" to check if motors are balanced
   - Adjust `Offset_motor_right` until both wheels move equally
   
2. **IR Sensor Thresholds**
   - Run "calibrate_threshold" with sensor on/off line
   - Verify with debug output showing binary readings
   
3. **Line Following Speed**
   - Increase `baseSpeed` gradually (80-120 range)
   - Increase `maxSpeed` for faster response (current: 120)
   
4. **Proportional Gain (KP)**
   - Start at 10, increase by 2 if oscillation occurs
   - Decrease if bot overshoots line corrections
   
5. **Derivative Gain (KD)**
   - Increase if corrections are too harsh
   - Helps dampen oscillations
   
6. **Turn Delta**
   - Increase if 90° turns feel slow
   - Decrease if turns overshoot (>100°)
   
7. **Debounce Threshold**
   - Increase if false turn detections occur
   - Decrease if turns are missed
```

---

## Implementation Priority

**Phase 1 (Critical - Do First):**
1. Create SensorConfig and MotorConfig structs
2. Consolidate motor command logic
3. Add serial command interface for parameter tuning

**Phase 2 (Important - Do Next):**
1. Implement turn debouncing
2. Add proportional turn speed adjustment
3. Improve error calculation logic

**Phase 3 (Nice to Have):**
1. Add full diagnostics
2. Implement advanced PD filtering
3. Add state machine
