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

void loop() {
  // Process serial commands for tuning
  handleSerialCommands();

  // Read sensor inputs
  readIRArray();
  diagnostics.loopCount++;

  // Update distance estimation (must be before any turn/movement)
  updateDistance();

  // update OLED status based on display mode
  // float err = calculate_error();
  // int leftSpeed = getLeftSpeed();
  // int rightSpeed = getRightSpeed();
  
  // switch (currentDisplayMode) {
  //   case DISPLAY_STATUS:
  //     oledDisplayStatus(yaw, err, leftSpeed, rightSpeed);
  //     break;
  //   case DISPLAY_IR_DIGITAL:
  //     oledDisplayIRReadings(irReadings);
  //     break;
  //   case DISPLAY_IR_RAW:
  //     oledDisplayRawIRReadings(rawIrReadings);
  //     break;
  // }

  // Detect turn (with debouncing)
  int turn = detect_turn();

  // State machine for movement
  if (turn == -1) {
    // Serial.println(">>> RIGHT TURN DETECTED");
    turnRight(100);
  } 
  else if (turn == 1) {
    // Serial.println(">>> LEFT TURN DETECTED");
    turnLeft(100);
  } 
  else if (turn == 0) {
    currentState = BOT_LINE_FOLLOWING;
    followLine();
  } 
  else if (turn == -2) {
    // turnRight(100);
    stopMotors();
  }
  else{
    followLine();
  }

  // Small delay for stability
  delay(5);
}