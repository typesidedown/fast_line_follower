#ifndef MAZE_H
#define MAZE_H

#include <Arduino.h>
#include <vector>
#include <queue>

// ============================================================================
// MAZE SOLVING STATE MACHINE
// ============================================================================
enum MazeMode {
  MAZE_IDLE = 0,
  MAZE_DRY_RUN = 1,        // Exploration phase - learn the maze
  MAZE_FINAL_RUN = 2       // Execution phase - follow shortest path
};

// ============================================================================
// MOVEMENT DIRECTIONS (using yaw-based encoding)
// ============================================================================
enum Direction {
  DIR_NORTH = 0,            // 0°
  DIR_EAST = 1,             // 90° (right)
  DIR_SOUTH = 2,            // 180° (back)
  DIR_WEST = 3              // 270° (left)
};

// ============================================================================
// JUNCTION/CELL DATA STRUCTURE
// ============================================================================
struct Junction {
  int x, y;                  // Grid coordinates
  bool visited;              // Has this junction been visited?
  bool isEnd;                // Is this the endpoint (white square)?
  int pathCount;             // Number of times visited (for pathfinding)
  
  // Wall indicators: true = wall exists in that direction
  bool wallNorth;
  bool wallEast;
  bool wallSouth;
  bool wallWest;
  
  // Connections to neighboring junctions
  int neighbors[4];          // Indices: DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST (-1 if no passage)
};

// ============================================================================
// PATHFINDING RESULT
// ============================================================================
struct Path {
  std::vector<int> junctionIndices;    // Indices of junctions in the path
  std::vector<Direction> turns;         // Turns to make at each junction
  int length;                           // Number of steps/junctions
};

// ============================================================================
// MAZE SOLVING STATE
// ============================================================================
struct MazeState {
  MazeMode currentMode;
  Direction currentDirection;           // Current heading (0=N, 1=E, 2=S, 3=W)
  int currentJunction;                  // Index of current junction
  int endJunctionIndex;                 // Index of the endpoint junction
  
  // Dry run data
  std::vector<Junction> junctions;      // All discovered junctions
  std::vector<Direction> dryRunPath;    // Sequence of turns made during dry run
  std::vector<int> dryRunJunctions;     // Sequence of junction indices visited
  
  // Final run data
  Path shortestPath;                    // Computed shortest path
  int finalRunStep;                     // Current position in final run
  
  // Statistics
  unsigned long dryRunStartTime;
  unsigned long dryRunEndTime;
  unsigned long finalRunStartTime;
  unsigned long finalRunEndTime;
  float dryRunDistance;
  float finalRunDistance;
};

extern MazeState mazeState;

// ============================================================================
// INITIALIZATION AND CORE FUNCTIONS
// ============================================================================
void initMaze();
void switchMazeMode(MazeMode newMode);
void startDryRun();
void startFinalRun();
void resetMazeData();

// ============================================================================
// DRY RUN (EXPLORATION) FUNCTIONS
// ============================================================================
void handleJunction();
void recordTurn(Direction turn);
void detectEndpoint();
bool checkForWhiteSquare();
void addJunction(int x, int y, Direction entryDir);

// ============================================================================
// PATHFINDING AND FINAL RUN FUNCTIONS
// ============================================================================
void computeShortestPath();
int findShortestPathBFS();
Direction getNextTurn();
void executeNextStep();
bool isMazeComplete();

// ============================================================================
// UTILITY AND DEBUG FUNCTIONS
// ============================================================================
Direction getTurnDirection(Direction currentDir, Direction targetDir);
Direction turnLeft();
Direction turnRight();
Direction turnAround();
void printMazeState();
void printJunction(int idx);
void printPath(const Path& path);

#endif // MAZE_H
