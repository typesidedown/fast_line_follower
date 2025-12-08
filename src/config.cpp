#include "config.h"

// Initialize all configuration structures with defaults

SensorConfig sensorConfig = {
  {1000, 700, 700, 700, 700, 700, 700, 700},  // irThreshold - adjust for your sensors
  20.0f,      // kp - proportional gain
  0.0f,      // kd - derivative gain
  90,        // baseSpeed
  120,        // maxSpeed
  0,         // minSpeed
  3           // turnDetectConsecutive - debounce frames
};

MotorConfig motorConfig = {
  0,          // offsetMotorRight
  70,         // turnDelta
  20,         // minPWM threshold
  200,        // maxPWM
  4000,       // turnTimeoutMs
  75.0f       // turnAngleDegrees
};

PDController pdController = {
  15.0f,      // kp
  2.0f,      // kd
  0.0f,       // lastError
  60,         // maxCorrection
  true,       // useDerivativeLowPass
  0.3f        // lowPassAlpha
};

TurnDetectionState turnState = {
  0,          // lastTurnDetected
  0,          // consecutiveFrames
  3           // debounceThreshold
};

BotState currentState = BOT_IDLE;

DiagnosticData diagnostics = {
  0,          // loopCount
  0.0f,       // maxError
  0.0f,       // minError
  0,          // turnsFailed
  0,          // imuFailures
  0           // lastRecalibration
};

void initializeConfig() {
  // Sync PD controller gains with sensor config
  pdController.kp = sensorConfig.kp;
  pdController.kd = sensorConfig.kd;
  pdController.lastError = 0;
  
  diagnostics.lastRecalibration = millis();
}

void printConfig() {
  Serial.println("\n===== CONFIGURATION =====");
  Serial.print("KP: ");
  Serial.print(sensorConfig.kp);
  Serial.print(" | KD: ");
  Serial.println(sensorConfig.kd);
  
  Serial.print("Base Speed: ");
  Serial.print(sensorConfig.baseSpeed);
  Serial.print(" | Max Speed: ");
  Serial.print(sensorConfig.maxSpeed);
  Serial.print(" | Min Speed: ");
  Serial.println(sensorConfig.minSpeed);
  
  Serial.print("Turn Delta: ");
  Serial.print(motorConfig.turnDelta);
  Serial.print(" | Motor Offset: ");
  Serial.println(motorConfig.offsetMotorRight);
  
  Serial.print("Turn Angle: ");
  Serial.print(motorConfig.turnAngleDegrees);
  Serial.print("° | Turn Timeout: ");
  Serial.print(motorConfig.turnTimeoutMs);
  Serial.println("ms");
  
  Serial.println("IR Thresholds: ");
  for (int i = 0; i < 8; i++) {
    Serial.print("  Ch");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(sensorConfig.irThreshold[i]);
  }
  Serial.println("========================\n");
}

void resetDiagnostics() {
  diagnostics.loopCount = 0;
  diagnostics.maxError = 0;
  diagnostics.minError = 0;
  diagnostics.turnsFailed = 0;
  diagnostics.imuFailures = 0;
}
