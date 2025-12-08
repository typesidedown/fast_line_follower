#include <Arduino.h>
#include "movement.h"
#include "sensors.h"
#include "imu.h"
#include "oled.h"
#include "config.h"

// ============================================================================
// DISPLAY MODE SELECTION
// ============================================================================
enum DisplayMode {
  DISPLAY_STATUS = 0,     // Yaw, Error, Motor speeds
  DISPLAY_IR_DIGITAL = 1, // Digital IR readings
  DISPLAY_IR_RAW = 2      // Raw ADC IR values
};
DisplayMode currentDisplayMode = DISPLAY_STATUS;

// ============================================================================
// SERIAL COMMAND PARSER FOR RUNTIME TUNING
// ============================================================================
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() == 0) return;

    // --- Proportional Gain ---
    if (command.startsWith("kp=")) {
      float kp = command.substring(3).toFloat();
      setControlGains(kp, sensorConfig.kd);
    }
    // --- Derivative Gain ---
    else if (command.startsWith("kd=")) {
      float kd = command.substring(3).toFloat();
      setControlGains(sensorConfig.kp, kd);
    }
    // --- Turn Speed Delta ---
    else if (command.startsWith("delta=")) {
      int delta = command.substring(6).toInt();
      setTurnDelta(delta);
    }
    // --- Base Speed ---
    else if (command.startsWith("speed=")) {
      int speed = command.substring(6).toInt();
      setBaseSpeed(speed);
    }
    // --- Motor Offset ---
    else if (command.startsWith("offset=")) {
      int offset = command.substring(7).toInt();
      motorConfig.offsetMotorRight = constrain(offset, -50, 50);
      Serial.print("Motor offset set to: ");
      Serial.println(motorConfig.offsetMotorRight);
    }
    // --- IR Threshold for channel ---
    else if (command.startsWith("thresh_")) {
      int channel = command.substring(7, 8).toInt();
      int threshold = command.substring(9).toInt();
      setIRThreshold(channel, threshold);
    }
    // --- Debounce setting ---
    else if (command.startsWith("debounce=")) {
      int val = command.substring(9).toInt();
      turnState.debounceThreshold = constrain(val, 1, 10);
      Serial.print("Debounce threshold set to: ");
      Serial.println(turnState.debounceThreshold);
    }
    // --- Motor Test Forward ---
    else if (command == "motor_test_forward") {
      Serial.println("Testing: FORWARD at 100 PWM for 2 seconds...");
      moveForward(100);
      delay(2000);
      stopMotors();
      Serial.println("Motor test forward done");
    }
    // --- Motor Test Left ---
    else if (command == "motor_test_left") {
      Serial.println("Testing: LEFT TURN at 100 PWM...");
      turnLeft(100);
      Serial.println("Motor test left done");
    }
    // --- Motor Test Right ---
    else if (command == "motor_test_right") {
      Serial.println("Testing: RIGHT TURN at 100 PWM...");
      turnRight(100);
      Serial.println("Motor test right done");
    }
    // --- Display Current Configuration ---
    else if (command == "status") {
      printConfig();
    }
    // --- Print Raw Sensor Readings ---
    else if (command == "sensors") {
      Serial.println("Raw IR readings:");
      for (int i = 0; i < 8; i++) {
        Serial.print("  Ch");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(rawIrReadings[i]);
      }
      Serial.println("Digital IR readings:");
      for (int i = 0; i < 8; i++) {
        Serial.print("  Ch");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(irReadings[i]);
      }
    }
    // --- Switch Display Mode ---
    else if (command == "display_status") {
      currentDisplayMode = DISPLAY_STATUS;
      Serial.println("Display mode: STATUS (Yaw, Error, Motor speeds)");
    }
    else if (command == "display_ir_digital") {
      currentDisplayMode = DISPLAY_IR_DIGITAL;
      Serial.println("Display mode: IR DIGITAL READINGS");
    }
    else if (command == "display_ir_raw") {
      currentDisplayMode = DISPLAY_IR_RAW;
      Serial.println("Display mode: IR RAW ADC VALUES");
    }
    // --- Distance Tracking Commands ---
    else if (command == "dist_start") {
      startDistanceTracking();
    }
    else if (command == "dist_stop") {
      stopDistanceTracking();
    }
    else if (command == "dist_reset") {
      resetDistance();
    }
    else if (command == "dist_calibrate") {
      calibrateAccelerometer(500);
    }
    else if (command == "dist_get") {
      Serial.print("Distance: ");
      Serial.print(getDistanceMM());
      Serial.print(" mm (");
      Serial.print(getDistanceUnits());
      Serial.println(" units)");
    }
    else if (command.startsWith("dist_unit=")) {
      int unitSize = command.substring(10).toInt();
      distEstimator.distanceUnit = constrain(unitSize, 10, 500);
      Serial.print("Distance unit set to: ");
      Serial.print(distEstimator.distanceUnit);
      Serial.println(" mm");
    }
    // --- Diagnostics ---
    else if (command == "diag") {
      Serial.println("\n===== DIAGNOSTICS =====");
      Serial.print("Loop count: ");
      Serial.println(diagnostics.loopCount);
      Serial.print("Max error: ");
      Serial.print(diagnostics.maxError);
      Serial.print(" | Min error: ");
      Serial.println(diagnostics.minError);
      Serial.print("Failed turns: ");
      Serial.println(diagnostics.turnsFailed);
      Serial.print("IMU failures: ");
      Serial.println(diagnostics.imuFailures);
      Serial.println("=======================\n");
    }
    // --- Reset Diagnostics ---
    else if (command == "reset_diag") {
      resetDiagnostics();
      Serial.println("Diagnostics reset");
    }
    // --- Help ---
    else if (command == "help" || command == "?") {
      Serial.println("\n===== SERIAL COMMANDS =====");
      Serial.println("--- PD Controller Tuning ---");
      Serial.println("  kp=<float>        - Set proportional gain (e.g., kp=12.5)");
      Serial.println("  kd=<float>        - Set derivative gain (e.g., kd=8.0)");
      Serial.println("  delta=<int>       - Set turn speed delta (30-50 typical)");
      Serial.println("  speed=<int>       - Set base forward speed (80-120)");
      Serial.println("  offset=<int>      - Set motor balance offset (-50 to +50)");
      Serial.println("  thresh_<ch>=<val> - Set IR threshold for channel 0-7");
      Serial.println("  debounce=<int>    - Set turn debounce frames (1-10)");
      Serial.println("\n--- Motor Testing ---");
      Serial.println("  motor_test_forward");
      Serial.println("  motor_test_left");
      Serial.println("  motor_test_right");
      Serial.println("\n--- Display on OLED ---");
      Serial.println("  display_status       - Show status (Yaw, Error, Speeds)");
      Serial.println("  display_ir_digital   - Show digital IR readings");
      Serial.println("  display_ir_raw       - Show raw ADC IR values");
      Serial.println("\n--- Distance Tracking ---");
      Serial.println("  dist_calibrate       - Calibrate accelerometer");
      Serial.println("  dist_start           - Start distance tracking");
      Serial.println("  dist_stop            - Stop distance tracking");
      Serial.println("  dist_reset           - Reset distance to zero");
      Serial.println("  dist_get             - Get current distance");
      Serial.println("  dist_unit=<int>      - Set unit size in mm (10-500)");
      Serial.println("\n--- Monitoring ---");
      Serial.println("  status            - Show current configuration");
      Serial.println("  sensors           - Show IR sensor readings");
      Serial.println("  diag              - Show diagnostic data");
      Serial.println("  reset_diag        - Reset diagnostics");
      Serial.println("  help / ?          - Show this help");
      Serial.println("===========================\n");
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(command);
      Serial.println("Type 'help' for available commands");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);  // Give serial time to initialize

  // Serial.println("\n===== LINE FOLLOWER BOT STARTUP =====");
  // Serial.println("Initializing systems...");

  // initialize modules
  initMovement();
  initSensors();
  initIMU();
  initOLED();
  initializeConfig();
  
  // Initialize distance estimation with 50mm per unit (adjust as needed)
  initDistanceEstimator(50);

  // Serial.println("All systems initialized!");
  printConfig();
  // Serial.println("Type 'help' for available commands");
  // Serial.println("=====================================\n");
}

// ============================================================================
// JUNCTION STATE MACHINE
// ============================================================================
enum JunctionPhase {
  PHASE_NORMAL_FOLLOW = 0,          // Normal line following
  PHASE_JUNCTION_DETECTED = 1,      // Junction detected, about to probe
  PHASE_PROBING_FORWARD = 2,        // Moving forward to probe for straight
  PHASE_ANALYSIS = 3,               // Analyzing sensor readings
  PHASE_DECISION_MADE = 4,          // Decision made, ready to execute
  PHASE_EXECUTING_DECISION = 5      // Executing the decided turn/action
};

struct JunctionState_Main {
  JunctionPhase phase;
  unsigned long phaseStartTime;
  int detectedLeftRight;            // Detected left/right path before probing
  int detectedStraight;             // Detected straight path after probing
  int finalDecision;                // Final decision made
  bool decisionExecuted;            // Whether we've started executing
};

JunctionState_Main juncState = {PHASE_NORMAL_FOLLOW, 0, 0, 0, 0, false};

// ============================================================================
// DETECT LEFT AND RIGHT PATHS AT JUNCTION
// ============================================================================
int detectLeftRightPaths() {
  int leftCount = 0;
  int rightCount = 0;

  // Count active sensors by region
  for (int i = LEFT_SENSORS_START; i <= LEFT_SENSORS_END; i++) {
    if (irReadings[i] == 1) leftCount++;
  }
  for (int i = RIGHT_SENSORS_START; i <= RIGHT_SENSORS_END; i++) {
    if (irReadings[i] == 1) rightCount++;
  }

  // Return: -1 (left), 1 (right), 2 (both), 0 (neither)
  if (leftCount >= LEFT_TURN_THRESHOLD && rightCount >= RIGHT_TURN_THRESHOLD) {
    return 2;  // Both paths available
  } else if (leftCount >= LEFT_TURN_THRESHOLD) {
    return -1; // Left path available
  } else if (rightCount >= RIGHT_TURN_THRESHOLD) {
    return 1;  // Right path available
  } else {
    return 0;  // Neither left nor right
  }
}

// ============================================================================
// DETECT STRAIGHT PATH (after probing forward)
// ============================================================================
int detectStraightPath() {
  int centerCount = irReadings[CENTER_LEFT_SENSOR] + irReadings[CENTER_RIGHT_SENSOR];
  int totalActive = 0;
  
  for (int i = 0; i < 8; i++) {
    if (irReadings[i] == 1) totalActive++;
  }

  // Straight path if center sensors are active
  if (centerCount >= 2 || totalActive >= CENTER_STRAIGHT_THRESHOLD) {
    return 1;  // Straight path detected
  }
  return 0;   // No straight path
}

// ============================================================================
// MAKE JUNCTION DECISION - Priority: LEFT > FORWARD > RIGHT > TURNAROUND
// ============================================================================
int makeJunctionDecision(int leftRightPaths, int straightPath) {
  // Priority order:
  // 1. LEFT if available
  // 2. FORWARD/STRAIGHT if available
  // 3. RIGHT if available
  // 4. TURNAROUND if none available

  if (leftRightPaths == -1 || leftRightPaths == 2) {
    // Left path available (either alone or with right)
    Serial.println("[DECISION] LEFT path available -> Turn LEFT");
    return -1;
  } 
  else if (straightPath == 1) {
    // No left, but straight available
    Serial.println("[DECISION] No LEFT, STRAIGHT path available -> Go FORWARD");
    return 0;
  } 
  else if (leftRightPaths == 1) {
    // No left or straight, but right available
    Serial.println("[DECISION] No LEFT/FORWARD, RIGHT path available -> Turn RIGHT");
    return 1;
  } 
  else {
    // No paths available
    Serial.println("[DECISION] No paths available -> TURNAROUND");
    return -2;
  }
}

// ============================================================================
// EXECUTE JUNCTION DECISION
// ============================================================================
void executeJunctionDecision(int decision) {
  switch (decision) {
    case -1:  // LEFT TURN
      Serial.println("-> Executing LEFT TURN");
      turnLeft(sensorConfig.baseSpeed);
      juncState.decisionExecuted = true;
      break;

    case 0:   // STRAIGHT PATH / FORWARD
      Serial.println("-> Following STRAIGHT PATH");
      followLine();
      juncState.decisionExecuted = true;
      break;

    case 1:   // RIGHT TURN
      Serial.println("-> Executing RIGHT TURN");
      turnRight(sensorConfig.baseSpeed);
      juncState.decisionExecuted = true;
      break;

    case -2:  // DEAD END - TURNAROUND
      Serial.println("-> DEAD END, executing TURNAROUND");
      turnAround(sensorConfig.baseSpeed);
      juncState.decisionExecuted = true;
      break;

    default:
      Serial.println("-> Unknown decision, following line");
      followLine();
      break;
  }
}

void loop() {
  // Process serial commands for tuning
  handleSerialCommands();

  // Read sensor inputs
  readIRArray();
  diagnostics.loopCount++;

  // Update distance estimation
  updateDistance();

  // ========================================================================
  // JUNCTION STATE MACHINE
  // ========================================================================
  
  switch (juncState.phase) {
    
    case PHASE_NORMAL_FOLLOW: {
      // Normal line following - detect if we hit a junction
      int detectedPath = detect_turn(irReadings);
      
      if (detectedPath == 3) {
        // Junction detected!
        Serial.println("\n[JUNCTION] Detected! Stopping to analyze...");
        stopMotors();
        delay(50);  // Brief pause to settle
        
        // Read and detect left/right paths
        readIRArray();
        juncState.detectedLeftRight = detectLeftRightPaths();
        Serial.print("[JUNCTION] Left/Right paths: ");
        Serial.println(juncState.detectedLeftRight);
        
        // Transition to probing phase
        juncState.phase = PHASE_PROBING_FORWARD;
        juncState.phaseStartTime = millis();
        Serial.println("[PROBING] Moving forward to check for straight path...");
      } 
      else if (detectedPath == 0) {
        // Straight path - continue following
        followLine();
      }
      else if (detectedPath == -1) {
        // Slight left bias
        followLine();
      }
      else if (detectedPath == 1) {
        // Slight right bias
        followLine();
      }
      else if (detectedPath == -2) {
        // No path - treat as dead end
        Serial.println("[DEAD END] No line detected!");
        juncState.finalDecision = -2;
        juncState.phase = PHASE_DECISION_MADE;
      }
      break;
    }

    case PHASE_PROBING_FORWARD: {
      // Move forward for 100ms to probe for straight path
      unsigned long elapsedTime = millis() - juncState.phaseStartTime;
      
      if (elapsedTime < 100) {
        // Still probing - move forward
        moveForward(sensorConfig.baseSpeed);
      } 
      else if (elapsedTime < 120) {
        // Stop and re-read sensors
        stopMotors();
        delay(10);
        readIRArray();
        juncState.detectedStraight = detectStraightPath();
        Serial.print("[PROBING] Straight path detected: ");
        Serial.println(juncState.detectedStraight);
      } 
      else {
        // Probing complete, move to analysis
        stopMotors();
        juncState.phase = PHASE_ANALYSIS;
      }
      break;
    }

    case PHASE_ANALYSIS: {
      // Analyze the sensor readings and make decision
      juncState.finalDecision = makeJunctionDecision(
        juncState.detectedLeftRight,
        juncState.detectedStraight
      );
      juncState.phase = PHASE_DECISION_MADE;
      break;
    }

    case PHASE_DECISION_MADE: {
      // Execute the decided action once
      if (!juncState.decisionExecuted) {
        executeJunctionDecision(juncState.finalDecision);
      }
      
      // Wait for turn to complete, then return to normal following
      // (Turn functions are blocking, so we wait here)
      juncState.decisionExecuted = false;
      juncState.phase = PHASE_NORMAL_FOLLOW;
      Serial.println("[JUNCTION] Decision executed. Returning to normal line following.\n");
      break;
    }

    default:
      // Fallback to normal follow
      juncState.phase = PHASE_NORMAL_FOLLOW;
      followLine();
      break;
  }

}