#include "maze.h"
#include "movement.h"
#include "sensors.h"
#include "imu.h"
#include "oled.h"
#include "config.h"
#include "pathfinder.h"
#include "junction_detector.h"
#include <Arduino.h>

// ============================================================================
// GLOBAL MAZE STATE
// ============================================================================
MazeState mazeState;

// ============================================================================
// INITIALIZATION
// ============================================================================
void initMaze() {
  mazeState.currentMode = MAZE_IDLE;
  mazeState.currentDirection = DIR_NORTH;
  mazeState.currentJunction = 0;
  mazeState.endJunctionIndex = -1;
  mazeState.finalRunStep = 0;
  
  // Add starting junction (0, 0)
  Junction startJunction;
  startJunction.x = 0;
  startJunction.y = 0;
  startJunction.visited = true;
  startJunction.isEnd = false;
  startJunction.pathCount = 0;
  startJunction.wallNorth = false;
  startJunction.wallEast = false;
  startJunction.wallSouth = false;
  startJunction.wallWest = false;
  startJunction.neighbors[0] = -1;
  startJunction.neighbors[1] = -1;
  startJunction.neighbors[2] = -1;
  startJunction.neighbors[3] = -1;
  
  mazeState.junctions.push_back(startJunction);
  
  mazeState.dryRunDistance = 0;
  mazeState.finalRunDistance = 0;
  
  Serial.println("[MAZE] System initialized");
}

// ============================================================================
// MODE SWITCHING
// ============================================================================
void switchMazeMode(MazeMode newMode) {
  if (mazeState.currentMode == newMode) return;
  
  Serial.print("[MAZE] Switching mode: ");
  Serial.println(newMode == MAZE_DRY_RUN ? "DRY RUN" : (newMode == MAZE_FINAL_RUN ? "FINAL RUN" : "IDLE"));
  
  mazeState.currentMode = newMode;
  
  if (newMode == MAZE_DRY_RUN) {
    startDryRun();
  } else if (newMode == MAZE_FINAL_RUN) {
    startFinalRun();
  } else {
    stopMotors();
  }
}

void startDryRun() {
  resetMazeData();
  mazeState.currentMode = MAZE_DRY_RUN;
  mazeState.currentJunction = 0;
  mazeState.currentDirection = DIR_NORTH;
  mazeState.dryRunStartTime = millis();
  
  // Reset distance tracking
  resetDistance();
  startDistanceTracking();
  
  moveForward(sensorConfig.baseSpeed);
  Serial.println("[MAZE] Dry run started - bot exploring...");
}

void startFinalRun() {
  // Compute shortest path from junctions map
  computeShortestPath();
  
  mazeState.currentMode = MAZE_FINAL_RUN;
  mazeState.currentJunction = 0;
  mazeState.currentDirection = DIR_NORTH;
  mazeState.finalRunStep = 0;
  mazeState.finalRunStartTime = millis();
  
  // Reset distance tracking
  resetDistance();
  startDistanceTracking();
  
  moveForward(sensorConfig.baseSpeed);
  Serial.println("[MAZE] Final run started - executing shortest path...");
  Serial.print("[MAZE] Path length: ");
  Serial.println(mazeState.shortestPath.length);
}

void resetMazeData() {
  // Keep junctions discovered, but reset path tracking
  for (int i = 0; i < mazeState.junctions.size(); i++) {
    mazeState.junctions[i].visited = (i == 0);  // Only start is visited initially
    mazeState.junctions[i].pathCount = 0;
  }
  
  mazeState.dryRunPath.clear();
  mazeState.dryRunJunctions.clear();
  mazeState.currentJunction = 0;
  mazeState.currentDirection = DIR_NORTH;
}

// ============================================================================
// DRY RUN FUNCTIONS
// ============================================================================
void handleJunction() {
  // Called when junction is detected
  detectEndpoint();
  
  // Check available paths (walls) and decide which way to go
  int leftChoice = -1, straightChoice = -1, rightChoice = -1, backChoice = -1;
  
  // Scan for walls using IR sensors
  // If no wall detected in a direction, there's an opening
  
  int currentIdx = mazeState.currentJunction;
  Junction& current = mazeState.junctions[currentIdx];
  
  Serial.print("[MAZE] Junction ");
  Serial.print(currentIdx);
  Serial.print(" at (");
  Serial.print(current.x);
  Serial.print(",");
  Serial.print(current.y);
  Serial.println(")");
  
  // For now, implement simple left-wall-following strategy during dry run
  Direction leftDir = (Direction)((mazeState.currentDirection + 3) % 4);  // Turn left
  Direction straightDir = mazeState.currentDirection;
  Direction rightDir = (Direction)((mazeState.currentDirection + 1) % 4);  // Turn right
  Direction backDir = (Direction)((mazeState.currentDirection + 2) % 4);   // Turn back
  
  // Preference order: left > straight > right > back (left-hand rule)
  if (!current.wallWest && mazeState.currentDirection == DIR_NORTH) {
    // Turn left (west)
    turnLeft(sensorConfig.baseSpeed + sensorConfig.baseSpeed);  // Higher speed for turn
    delay(1000);  // Allow turn to complete
    mazeState.currentDirection = leftDir;
    recordTurn(leftDir);
  } else if (!current.wallNorth && mazeState.currentDirection == DIR_NORTH) {
    // Go straight
    moveForward(sensorConfig.baseSpeed);
    recordTurn(straightDir);
  } else if (!current.wallEast && mazeState.currentDirection == DIR_NORTH) {
    // Turn right (east)
    turnRight(sensorConfig.baseSpeed + sensorConfig.baseSpeed);
    delay(1000);
    mazeState.currentDirection = rightDir;
    recordTurn(rightDir);
  } else if (!current.wallSouth && mazeState.currentDirection == DIR_NORTH) {
    // Turn around (south)
    turnAround(sensorConfig.baseSpeed + sensorConfig.baseSpeed);
    delay(1500);
    mazeState.currentDirection = backDir;
    recordTurn(backDir);
  } else {
    Serial.println("[MAZE] ERROR: Dead end!");
    stopMotors();
  }
}

void recordTurn(Direction turn) {
  mazeState.dryRunPath.push_back(turn);
  mazeState.dryRunJunctions.push_back(mazeState.currentJunction);
  
  Serial.print("[MAZE] Turn recorded: ");
  Serial.println(turn);
}

void detectEndpoint() {
  // Check if we've reached the endpoint (large white square)
  bool isWhiteSquare = checkForWhiteSquare();
  
  if (isWhiteSquare) {
    Serial.println("[MAZE] ENDPOINT DETECTED!");
    
    mazeState.dryRunEndTime = millis();
    mazeState.dryRunDistance = getDistanceMM();
    
    int currentIdx = mazeState.currentJunction;
    if (currentIdx >= 0 && currentIdx < mazeState.junctions.size()) {
      mazeState.junctions[currentIdx].isEnd = true;
      mazeState.endJunctionIndex = currentIdx;
    }
    
    stopDistanceTracking();
    stopMotors();
    switchMazeMode(MAZE_IDLE);
  }
}

bool checkForWhiteSquare() {
  // The endpoint is a large white square
  // All IR sensors will read 0 (no line detected) when over white area
  
  int whiteSensorCount = 0;
  for (int i = 0; i < 8; i++) {
    if (irReadings[i] == 0) {  // 0 = no line (white/no IR reflection)
      whiteSensorCount++;
    }
  }
  
  // If most sensors see white, we're likely on the endpoint
  if (whiteSensorCount >= 6) {
    return true;
  }
  
  return false;
}

void addJunction(int x, int y, Direction entryDir) {
  // Check if junction already exists at this location
  for (int i = 0; i < mazeState.junctions.size(); i++) {
    if (mazeState.junctions[i].x == x && mazeState.junctions[i].y == y) {
      mazeState.currentJunction = i;
      mazeState.junctions[i].pathCount++;
      return;
    }
  }
  
  // Create new junction
  Junction newJunction;
  newJunction.x = x;
  newJunction.y = y;
  newJunction.visited = true;
  newJunction.isEnd = false;
  newJunction.pathCount = 1;
  newJunction.wallNorth = false;
  newJunction.wallEast = false;
  newJunction.wallSouth = false;
  newJunction.wallWest = false;
  newJunction.neighbors[0] = -1;
  newJunction.neighbors[1] = -1;
  newJunction.neighbors[2] = -1;
  newJunction.neighbors[3] = -1;
  
  mazeState.junctions.push_back(newJunction);
  mazeState.currentJunction = mazeState.junctions.size() - 1;
  
  Serial.print("[MAZE] New junction added at (");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(") - Total: ");
  Serial.println(mazeState.junctions.size());
}

// ============================================================================
// PATHFINDING FUNCTIONS
// ============================================================================
void computeShortestPath() {
  if (mazeState.endJunctionIndex < 0) {
    Serial.println("[MAZE] ERROR: No endpoint found!");
    return;
  }
  
  // Use optimal pathfinding with BFS for guaranteed shortest path
  std::vector<int> pathJunctions;
  findOptimalPath(0, mazeState.endJunctionIndex, pathJunctions, STRATEGY_BFS);
  
  // Store the path
  mazeState.shortestPath.junctionIndices = pathJunctions;
  mazeState.shortestPath.length = pathJunctions.size();
  
  Serial.print("[MAZE] Shortest path computed: ");
  Serial.print(mazeState.shortestPath.length);
  Serial.println(" junctions");
  
  // Print path for debugging
  if (mazeState.shortestPath.length > 0) {
    Serial.print("Path: ");
    for (int i = 0; i < pathJunctions.size(); i++) {
      Serial.print(pathJunctions[i]);
      if (i < pathJunctions.size() - 1) Serial.print(" -> ");
    }
    Serial.println();
  }
}

int findShortestPathBFS() {
  // This function is deprecated - computeShortestPath() now calls findOptimalPath()
  return mazeState.shortestPath.length;
}

Direction getNextTurn() {
  if (mazeState.finalRunStep >= mazeState.shortestPath.length - 1) {
    return mazeState.currentDirection;  // Already at destination
  }
  
  int currentJuncIdx = mazeState.shortestPath.junctionIndices[mazeState.finalRunStep];
  int nextJuncIdx = mazeState.shortestPath.junctionIndices[mazeState.finalRunStep + 1];
  
  // Determine direction to next junction
  Junction& current = mazeState.junctions[currentJuncIdx];
  Junction& next = mazeState.junctions[nextJuncIdx];
  
  Direction nextDir = DIR_NORTH;
  
  if (next.x > current.x) {
    nextDir = DIR_EAST;
  } else if (next.x < current.x) {
    nextDir = DIR_WEST;
  } else if (next.y > current.y) {
    nextDir = DIR_SOUTH;
  } else if (next.y < current.y) {
    nextDir = DIR_NORTH;
  }
  
  return nextDir;
}

void executeNextStep() {
  mazeState.finalRunStep++;
  
  if (mazeState.finalRunStep >= mazeState.shortestPath.length) {
    mazeState.finalRunEndTime = millis();
    mazeState.finalRunDistance = getDistanceMM();
    
    Serial.println("[MAZE] Final run complete!");
    stopDistanceTracking();
    stopMotors();
    switchMazeMode(MAZE_IDLE);
  }
}

bool isMazeComplete() {
  return mazeState.currentMode == MAZE_IDLE && mazeState.endJunctionIndex >= 0;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================
Direction getTurnDirection(Direction currentDir, Direction targetDir) {
  int diff = (targetDir - currentDir + 4) % 4;
  return (Direction)diff;
}

Direction turnLeft() {
  return (Direction)((mazeState.currentDirection + 3) % 4);
}

Direction turnRight() {
  return (Direction)((mazeState.currentDirection + 1) % 4);
}

Direction turnAround() {
  return (Direction)((mazeState.currentDirection + 2) % 4);
}

void printMazeState() {
  Serial.println("\n===== MAZE STATE =====");
  Serial.print("Mode: ");
  Serial.println(mazeState.currentMode);
  Serial.print("Current direction: ");
  Serial.println(mazeState.currentDirection);
  Serial.print("Current junction: ");
  Serial.println(mazeState.currentJunction);
  Serial.print("Total junctions: ");
  Serial.println(mazeState.junctions.size());
  Serial.print("Endpoint index: ");
  Serial.println(mazeState.endJunctionIndex);
  Serial.print("Dry run distance: ");
  Serial.print(mazeState.dryRunDistance);
  Serial.println(" mm");
  Serial.print("Final run distance: ");
  Serial.print(mazeState.finalRunDistance);
  Serial.println(" mm");
  Serial.println("=======================\n");
}

void printJunction(int idx) {
  if (idx < 0 || idx >= mazeState.junctions.size()) {
    Serial.println("Invalid junction index");
    return;
  }
  
  Junction& j = mazeState.junctions[idx];
  Serial.print("Junction ");
  Serial.print(idx);
  Serial.print(": (");
  Serial.print(j.x);
  Serial.print(",");
  Serial.print(j.y);
  Serial.print(") Visited:");
  Serial.print(j.visited);
  Serial.print(" End:");
  Serial.print(j.isEnd);
  Serial.print(" PathCount:");
  Serial.println(j.pathCount);
}

void printPath(const Path& path) {
  Serial.print("Path length: ");
  Serial.println(path.length);
  Serial.print("Junctions: ");
  for (int i = 0; i < path.junctionIndices.size(); i++) {
    Serial.print(path.junctionIndices[i]);
    if (i < path.junctionIndices.size() - 1) Serial.print(" -> ");
  }
  Serial.println();
}
