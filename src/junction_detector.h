#ifndef JUNCTION_DETECTOR_H
#define JUNCTION_DETECTOR_H

#include <Arduino.h>

// ============================================================================
// ENHANCED JUNCTION DETECTION WITH LOOKAHEAD
// ============================================================================

// Junction types/states
enum JunctionType {
  JUNCTION_NONE = 0,           // No junction detected
  JUNCTION_T_SHAPE = 1,        // T-junction (3 ways)
  JUNCTION_4_WAY = 2,          // 4-way junction (all paths)
  JUNCTION_LEFT_TURN = 3,      // Only left turn available
  JUNCTION_RIGHT_TURN = 4,     // Only right turn available
  JUNCTION_DEAD_END = 5        // Dead end (can't continue straight)
};

// Direction path availability
struct PathAvailability {
  bool canGoStraight;
  bool canGoLeft;
  bool canGoRight;
  bool canGoBack;
  JunctionType junctionType;
  int pathCount;  // Number of available paths
};

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Enhanced turn detection that also returns path availability
int detect_turn_enhanced(uint16_t irR[8], PathAvailability& outPaths);

// Check if line exists in current sensor readings
bool hasLineInCurrent(uint16_t irR[8]);

// Scan current position to determine walls
void scanCurrentJunction(uint16_t irR[8], PathAvailability& paths);

// Move forward slightly and check if line continues (for straight detection)
void moveLookaheadDistance(float millimeters);

// Stop after lookahead movement
void stopLookahead();

// Determine junction type from available paths
JunctionType determineJunctionType(PathAvailability& paths);

// Get descriptive string for junction type
const char* getJunctionTypeString(JunctionType jType);

// Wall configuration at junction
struct WallMap {
  bool wallNorth;
  bool wallEast;
  bool wallSouth;
  bool wallWest;
  bool wallStraight;   // Simplified wall indicators
  bool wallLeft;
  bool wallRight;
  bool wallBack;
};

// Get wall map from sensor readings at junction
WallMap getWallConfiguration(uint16_t irR[8], int currentDir);

// Update lookahead movement progress (returns true when complete)
bool updateLookaheadMovement(float actualDistanceMM);

// Check if line continues after lookahead
bool checkLineAfterLookahead(uint16_t irR[8]);

// Stop lookahead movement
void stopLookahead();

// Debug: Print path availability
void printPathAvailability(PathAvailability& paths);

#endif // JUNCTION_DETECTOR_H
