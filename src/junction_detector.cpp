#include "junction_detector.h"
#include "sensors.h"
#include "movement.h"
#include "config.h"
#include <Arduino.h>

// ============================================================================
// ENHANCED TURN DETECTION WITH LOOKAHEAD
// ============================================================================

// Static state for lookahead movement
static unsigned long lookaheadStartTime = 0;
static float lookaheadDistance = 0;
static bool inLookahead = false;
static float lookaheadSpeed = 80;  // Default lookahead speed

int detect_turn_enhanced(uint16_t irR[8], PathAvailability& outPaths) {
  // Initialize paths struct
  outPaths.canGoStraight = false;
  outPaths.canGoLeft = false;
  outPaths.canGoRight = false;
  outPaths.canGoBack = false;
  outPaths.junctionType = JUNCTION_NONE;
  outPaths.pathCount = 0;

  // First, scan current position to detect junction
  scanCurrentJunction(irR, outPaths);
  
  // Determine junction type
  outPaths.junctionType = determineJunctionType(outPaths);

  // If no junction detected, return early
  if (outPaths.junctionType == JUNCTION_NONE) {
    return 0;  // No junction
  }

  // For uncertain cases (when straight is not clear from sensors alone),
  // return the current detection with available paths
  return (int)outPaths.junctionType;
}

void scanCurrentJunction(uint16_t irR[8], PathAvailability& paths) {
  // Count active sensors in different regions
  int leftCount = 0;
  int rightCount = 0;
  int centerCount = 0;

  // Scan left sensors (indices 0-3)
  for (int i = 0; i <= 3; i++) {
    if (irR[i] == 1) leftCount++;
  }
  
  // Scan right sensors (indices 4-7)
  for (int i = 4; i <= 7; i++) {
    if (irR[i] == 1) rightCount++;
  }
  
  // Center sensors
  centerCount = irR[3] + irR[4];

  /*
   * JUNCTION DETECTION LOGIC:
   * - If both left and right have line: T-junction (left + right turns available)
   * - If left only: Left turn available
   * - If right only: Right turn available
   * - If center only: Straight path available (no turns)
   * - If no line: Dead end or junction with all sides open
   */

  // Count how many "sides" have line
  int sidesWithLine = 0;
  if (leftCount >= 2) {
    paths.canGoLeft = true;
    sidesWithLine++;
  }
  if (rightCount >= 2) {
    paths.canGoRight = true;
    sidesWithLine++;
  }
  if (centerCount >= 1) {
    paths.canGoStraight = true;
    sidesWithLine++;
  }

  // Back is always available (worst case: turn around)
  paths.canGoBack = true;
  
  paths.pathCount = sidesWithLine + 1;  // +1 for always available back option

  Serial.print("[JUNCTION] Scan: L=");
  Serial.print(leftCount);
  Serial.print(" C=");
  Serial.print(centerCount);
  Serial.print(" R=");
  Serial.print(rightCount);
  Serial.print(" → Paths: Left=");
  Serial.print(paths.canGoLeft);
  Serial.print(" Straight=");
  Serial.print(paths.canGoStraight);
  Serial.print(" Right=");
  Serial.println(paths.canGoRight);
}

bool hasLineInCurrent(uint16_t irR[8]) {
  // Check if any sensor detects a line
  for (int i = 0; i < 8; i++) {
    if (irR[i] == 1) return true;
  }
  return false;
}

void moveLookaheadDistance(float millimeters) {
  // Move forward at lookahead speed for a set distance
  inLookahead = true;
  lookaheadDistance = millimeters;
  lookaheadStartTime = millis();
  
  // Start moving forward
  moveForward(lookaheadSpeed);
  
  Serial.print("[LOOKAHEAD] Moving ");
  Serial.print(millimeters);
  Serial.println(" mm to check straight path...");
}

void stopLookahead() {
  inLookahead = false;
  lookaheadStartTime = 0;
  lookaheadDistance = 0;
  stopMotors();
  
  Serial.println("[LOOKAHEAD] Lookahead movement complete");
}

JunctionType determineJunctionType(PathAvailability& paths) {
  // Count available paths
  int availablePaths = 0;
  if (paths.canGoLeft) availablePaths++;
  if (paths.canGoStraight) availablePaths++;
  if (paths.canGoRight) availablePaths++;

  // Determine junction type based on available paths
  if (availablePaths == 0) {
    return JUNCTION_DEAD_END;  // No paths forward
  }
  else if (availablePaths == 1) {
    if (paths.canGoStraight) return JUNCTION_NONE;    // Just straight (not a junction)
    else if (paths.canGoLeft) return JUNCTION_LEFT_TURN;
    else if (paths.canGoRight) return JUNCTION_RIGHT_TURN;
  }
  else if (availablePaths == 2) {
    if (paths.canGoLeft && paths.canGoRight) {
      return JUNCTION_T_SHAPE;  // T-junction (left + right)
    }
    else if (paths.canGoLeft && paths.canGoStraight) {
      return JUNCTION_T_SHAPE;  // T-junction (left + straight)
    }
    else if (paths.canGoRight && paths.canGoStraight) {
      return JUNCTION_T_SHAPE;  // T-junction (right + straight)
    }
  }
  else if (availablePaths == 3) {
    return JUNCTION_4_WAY;  // 4-way junction
  }

  return JUNCTION_NONE;
}

const char* getJunctionTypeString(JunctionType jType) {
  switch (jType) {
    case JUNCTION_NONE:
      return "NONE";
    case JUNCTION_T_SHAPE:
      return "T_JUNCTION";
    case JUNCTION_4_WAY:
      return "4_WAY";
    case JUNCTION_LEFT_TURN:
      return "LEFT_ONLY";
    case JUNCTION_RIGHT_TURN:
      return "RIGHT_ONLY";
    case JUNCTION_DEAD_END:
      return "DEAD_END";
    default:
      return "UNKNOWN";
  }
}

WallMap getWallConfiguration(uint16_t irR[8], int currentDir) {
  WallMap walls;
  
  // Scan sensors to determine wall presence
  int leftCount = 0, rightCount = 0, centerCount = 0;
  
  for (int i = 0; i <= 3; i++) leftCount += irR[i];
  for (int i = 4; i <= 7; i++) rightCount += irR[i];
  centerCount = irR[3] + irR[4];

  // A wall exists in a direction if there's NO line detected there
  // This needs to be interpreted based on current direction
  walls.wallStraight = (centerCount < 1);
  walls.wallLeft = (leftCount < 2);
  walls.wallRight = (rightCount < 2);
  walls.wallBack = false;  // Back is never blocked (can always turn around)
  
  // Set cardinal directions based on current direction
  // (For now, simplified - set all to false, caller should interpret)
  walls.wallNorth = false;
  walls.wallEast = false;
  walls.wallSouth = false;
  walls.wallWest = false;

  return walls;
}

// ============================================================================
// LOOKAHEAD VERIFICATION LOGIC FOR MAIN LOOP
// ============================================================================

bool updateLookaheadMovement(float actualDistanceMM) {
  if (!inLookahead) return false;
  
  // Check if we've moved far enough
  if (actualDistanceMM >= lookaheadDistance) {
    stopLookahead();
    return true;  // Lookahead complete
  }
  
  return false;  // Still in lookahead
}

// After lookahead movement, check if line continues
bool checkLineAfterLookahead(uint16_t irR[8]) {
  return hasLineInCurrent(irR);
}

// ============================================================================
// DEBUG FUNCTIONS
// ============================================================================

void printPathAvailability(PathAvailability& paths) {
  Serial.print("[PATHS] Available: ");
  if (paths.canGoLeft) Serial.print("LEFT ");
  if (paths.canGoStraight) Serial.print("STRAIGHT ");
  if (paths.canGoRight) Serial.print("RIGHT ");
  if (paths.canGoBack) Serial.print("BACK ");
  Serial.print("| Type: ");
  Serial.println(getJunctionTypeString(paths.junctionType));
}
