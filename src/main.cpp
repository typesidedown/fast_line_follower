#include <Arduino.h>
#include "movement.h"
#include "sensors.h"
#include "imu.h"
#include "oled.h"
#include "config.h"
#include "maze.h"
#include "buttons.h"

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
  initButtons();
  initMaze();
  
  // Initialize distance estimation with 50mm per unit (adjust as needed)
  initDistanceEstimator(50);

  // Serial.println("All systems initialized!");
  printConfig();
  oledDisplayMazeIdle();
  // Serial.println("Type 'help' for available commands");
  // Serial.println("=====================================\n");
}

void loop() {
  // ========================================================================
  // BUTTON HANDLING - CHECK FOR MODE CHANGES
  // ========================================================================
  updateButtonStates();
  
  if (wasButton1Pressed()) {
    // Button 1: Start DRY RUN
    if (mazeState.currentMode != MAZE_DRY_RUN) {
      switchMazeMode(MAZE_DRY_RUN);
    }
  }
  
  if (wasButton2Pressed()) {
    // Button 2: Start FINAL RUN (only if dry run complete)
    if (mazeState.endJunctionIndex >= 0 && mazeState.currentMode != MAZE_FINAL_RUN) {
      switchMazeMode(MAZE_FINAL_RUN);
    }
  }
  
  // ========================================================================
  // SERIAL COMMAND HANDLING (TUNING AND DEBUG)
  // ========================================================================
  handleSerialCommands();
  
  // ========================================================================
  // SENSOR READING
  // ========================================================================
  readIRArray();
  diagnostics.loopCount++;
  
  // Update distance estimation (must be before any turn/movement)
  updateDistance();
  
  // ========================================================================
  // MAZE SOLVING STATE MACHINE
  // ========================================================================
  
  if (mazeState.currentMode == MAZE_DRY_RUN) {
    // ====================================================================
    // DRY RUN PHASE: EXPLORE AND LEARN
    // ====================================================================
    
    // Update display with exploration progress
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 500) {
      oledDisplayDryRunStatus(
        mazeState.junctions.size(),
        mazeState.dryRunPath.size(),
        getDistanceMM()
      );
      lastDisplayUpdate = millis();
    }
    
    // Detect junction
    int turnDetected = detect_turn();
    
    if (turnDetected != 0) {
      // Junction detected - decide which way to go
      Serial.print("[MAIN] Turn detected: ");
      Serial.println(turnDetected);
      
      // Calculate new grid position based on current direction
      int newX = mazeState.junctions[mazeState.currentJunction].x;
      int newY = mazeState.junctions[mazeState.currentJunction].y;
      
      switch (mazeState.currentDirection) {
        case DIR_NORTH:
          newY--;
          break;
        case DIR_EAST:
          newX++;
          break;
        case DIR_SOUTH:
          newY++;
          break;
        case DIR_WEST:
          newX--;
          break;
      }
      
      // Add or move to junction
      addJunction(newX, newY, mazeState.currentDirection);
      
      // Handle next action at junction
      handleJunction();
    } else {
      // No junction - continue straight line following
      float error = calculate_error();
      float correction = calculatePDCorrection(error);
      
      int leftSpeed = sensorConfig.baseSpeed + correction;
      int rightSpeed = sensorConfig.baseSpeed - correction;
      
      leftSpeed = constrain(leftSpeed, sensorConfig.minSpeed, sensorConfig.maxSpeed);
      rightSpeed = constrain(rightSpeed, sensorConfig.minSpeed, sensorConfig.maxSpeed);
      
      setMotorSpeeds(leftSpeed, rightSpeed);
    }
    
  } else if (mazeState.currentMode == MAZE_FINAL_RUN) {
    // ====================================================================
    // FINAL RUN PHASE: FOLLOW SHORTEST PATH
    // ====================================================================
    
    // Update display with progress
    static unsigned long lastDisplayUpdate2 = 0;
    if (millis() - lastDisplayUpdate2 > 500) {
      oledDisplayFinalRunStatus(
        mazeState.finalRunStep,
        mazeState.shortestPath.length,
        getDistanceMM()
      );
      lastDisplayUpdate2 = millis();
    }
    
    // Detect if we've reached the next junction
    int turnDetected = detect_turn();
    
    if (turnDetected != 0) {
      // At junction - move to next step in path
      executeNextStep();
    } else {
      // No junction - continue following line with PD controller
      float error = calculate_error();
      float correction = calculatePDCorrection(error);
      
      int leftSpeed = sensorConfig.baseSpeed + correction;
      int rightSpeed = sensorConfig.baseSpeed - correction;
      
      leftSpeed = constrain(leftSpeed, sensorConfig.minSpeed, sensorConfig.maxSpeed);
      rightSpeed = constrain(rightSpeed, sensorConfig.minSpeed, sensorConfig.maxSpeed);
      
      setMotorSpeeds(leftSpeed, rightSpeed);
    }
    
  } else {
    // IDLE MODE - Wait for button input
    static unsigned long lastDisplayUpdate3 = 0;
    if (millis() - lastDisplayUpdate3 > 1000) {
      oledDisplayMazeIdle();
      lastDisplayUpdate3 = millis();
    }
    
    // Check if maze is complete (both runs done)
    if (mazeState.endJunctionIndex >= 0 && 
        mazeState.dryRunEndTime > 0 && 
        mazeState.finalRunEndTime > 0) {
      oledDisplayMazeComplete(mazeState.dryRunDistance, mazeState.finalRunDistance);
    }
  }
}
