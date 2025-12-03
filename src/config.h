#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
struct SensorConfig {
  uint16_t irThreshold[8];
  float kp;
  float kd;
  int baseSpeed;
  int maxSpeed;
  int minSpeed;
  int turnDetectConsecutive;
};

// Default sensor configuration
extern SensorConfig sensorConfig;

// Sensor index constants
#define LEFT_SENSORS_START 0
#define LEFT_SENSORS_END 3
#define CENTER_LEFT_SENSOR 3
#define CENTER_RIGHT_SENSOR 4
#define RIGHT_SENSORS_START 4
#define RIGHT_SENSORS_END 7

#define LEFT_TURN_THRESHOLD 4
#define RIGHT_TURN_THRESHOLD 4
#define CENTER_STRAIGHT_THRESHOLD 2

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================
struct MotorConfig {
  int offsetMotorRight;      // Motor balance correction
  int turnDelta;             // Speed difference for 90° turns
  int minPWM;                // Minimum PWM to overcome static friction
  int maxPWM;                // Safety limit (typically 255)
  unsigned long turnTimeoutMs;
  float turnAngleDegrees;    // Target turn angle (usually 90)
};

extern MotorConfig motorConfig;

// ============================================================================
// PD CONTROLLER CONFIGURATION
// ============================================================================
struct PDController {
  float kp;
  float kd;
  float lastError;
  int maxCorrection;         // Max speed adjustment
  bool useDerivativeLowPass;
  float lowPassAlpha;        // 0.2-0.5 typical
};

extern PDController pdController;

// ============================================================================
// TURN DETECTION STATE
// ============================================================================
struct TurnDetectionState {
  int lastTurnDetected;
  int consecutiveFrames;
  int debounceThreshold;
};

extern TurnDetectionState turnState;

// ============================================================================
// BOT STATE MACHINE
// ============================================================================
enum BotState {
  BOT_IDLE = 0,
  BOT_LINE_FOLLOWING = 1,
  BOT_TURNING = 2,
  BOT_ERROR = 3,
  BOT_CALIBRATING = 4
};

extern BotState currentState;

// ============================================================================
// DIAGNOSTIC DATA
// ============================================================================
struct DiagnosticData {
  unsigned long loopCount;
  float maxError;
  float minError;
  int turnsFailed;
  int imuFailures;
  unsigned long lastRecalibration;
};

extern DiagnosticData diagnostics;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================
void initializeConfig();
void printConfig();
void resetDiagnostics();

#endif // CONFIG_H
