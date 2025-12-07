# Line Follower Bot - Maze Solver Implementation

## Overview

This document describes the maze-solving system implemented for the line follower robot. The system uses a **two-phase approach**: a dry run for exploration and mapping, followed by a final run executing the shortest path.

---

## System Architecture

### Components

1. **maze.h / maze.cpp** - Core maze state machine and logic
2. **buttons.h / buttons.cpp** - Button input handling for mode control
3. **pathfinder.h / pathfinder.cpp** - Advanced pathfinding algorithms
4. **oled.h / oled.cpp** - Display functions for each mode
5. **main.cpp** - Main loop integrating all components

### Hardware Requirements

- **2 Push Buttons** (Pins 2 and 3)
  - Button 1 (Pin 2): Initiates DRY RUN
  - Button 2 (Pin 3): Initiates FINAL RUN
- **OLED Display** - Shows current mode and progress
- **IR Sensor Array** (8 sensors) - Line following
- **IMU** - Distance tracking
- **Motors** - Movement control

---

## Operating Modes

### Mode 1: IDLE
- **State**: Waiting for user input
- **Display**: Shows "MAZE SOLVER" menu with button instructions
- **Actions**: None; motors stopped
- **Transition**: Button 1 pressed → DRY RUN

### Mode 2: DRY RUN (Exploration Phase)
- **Purpose**: Bot explores the maze and learns its layout
- **Strategy**: **Left-Wall-Following** algorithm
  - Bot keeps its left hand on the wall
  - Preference order at junctions: Left → Straight → Right → Back
  - Records all visited junctions and paths taken
- **Exit Condition**: Bot detects endpoint (large white square)
- **Data Collected**:
  - All junctions discovered (grid coordinates)
  - Connections between junctions
  - Total distance traveled
  - Path taken (sequence of turns)
- **Display**: Live junction count, step count, distance traveled
- **Transition**: Endpoint detected → IDLE (ready for final run)

### Mode 3: FINAL RUN (Optimization Phase)
- **Purpose**: Execute the shortest path from start to endpoint
- **Strategy**: Uses computed **shortest path** from pathfinding
- **Pathfinding Algorithms** (in order of efficiency):
  1. **BFS (Breadth-First Search)** - Guarantees shortest in junction count
  2. **Dijkstra** - Optimizes by actual distance between junctions
  3. **A\*** - Heuristic-based, fastest computation
- **Execution**: 
  - Bot follows line segments between junctions
  - At each junction, turns according to computed path
  - Continues until reaching endpoint
- **Exit Condition**: Entire path completed
- **Display**: Progress bar showing completion percentage
- **Results**:
  - Final distance traveled
  - Efficiency ratio (dry run distance / final run distance)
  - Both distances displayed on OLED

---

## Data Structures

### Junction
```cpp
struct Junction {
  int x, y;                  // Grid coordinates
  bool visited;              // Visited in current run
  bool isEnd;                // Is this the endpoint?
  int pathCount;             // Times visited
  bool wallNorth/East/South/West;  // Wall configuration
  int neighbors[4];          // Indices: N, E, S, W (-1 if blocked)
};
```

### MazeState
```cpp
struct MazeState {
  MazeMode currentMode;      // IDLE, DRY_RUN, or FINAL_RUN
  Direction currentDirection; // Current heading (N=0, E=1, S=2, W=3)
  int currentJunction;        // Current position
  
  // Dry run data
  std::vector<Junction> junctions;
  std::vector<Direction> dryRunPath;
  std::vector<int> dryRunJunctions;
  
  // Final run data
  Path shortestPath;
  int finalRunStep;
  
  // Statistics
  float dryRunDistance;
  float finalRunDistance;
};
```

### Path
```cpp
struct Path {
  std::vector<int> junctionIndices;    // Sequence of junctions
  std::vector<Direction> turns;        // Turns at each junction
  int length;                          // Number of steps
};
```

---

## Algorithm Details

### Dry Run - Left-Wall-Following Algorithm

The bot uses the **left-hand rule** (also called left-wall-following):

```
At each junction:
  1. Check if can turn left → Turn left
  2. Else if can go straight → Go straight
  3. Else if can turn right → Turn right
  4. Else turn around (only at dead ends)
```

This guarantees the bot will explore the entire maze and reach the endpoint.

**Advantages:**
- Simple and reliable
- Explores all passages
- No dead reckoning errors

**Disadvantages:**
- May not be optimal
- Visits some areas multiple times

### Final Run - Shortest Path Computation

After the dry run, the system builds a graph of junctions and uses pathfinding:

#### BFS (Breadth-First Search)
```
Guarantees: Shortest path by junction count
Time: O(V + E) where V=junctions, E=connections
Space: O(V)
Best for: Small mazes, guaranteed optimality
```

#### Dijkstra's Algorithm
```
Guarantees: Shortest path by distance
Time: O(V²) with simple implementation
Space: O(V)
Best for: Weighted graphs, distance-optimized
```

#### A* Algorithm
```
Guarantees: Optimal with admissible heuristic
Time: O(E * log(V)) with good heuristic
Space: O(V)
Best for: Large mazes, fast computation
```

**Graph Building Process:**
1. Find all adjacent junctions (differ by 1 in x or y)
2. Connect them bidirectionally
3. Store connections in `neighbors[4]` array

---

## Button Interface

### Button 1 (Pin 2) - DRY RUN
```
Press → Bot enters DRY RUN mode
        Starts exploration
        OLED shows progress
```

### Button 2 (Pin 3) - FINAL RUN
```
Press → (Only if dry run complete)
        Bot enters FINAL RUN mode
        Executes shortest path
        OLED shows progress
```

### Debouncing
- 50ms debounce timer prevents false triggers
- Rising edge detection with state tracking

---

## OLED Display Modes

### Maze Idle Screen
```
┌─────────────────────┐
│ MAZE SOLVER         │
│ ─────────────────── │
│                     │
│ Button 1: DRY RUN   │
│ Button 2: FINAL RUN │
│                     │
│ Press button...     │
└─────────────────────┘
```

### Dry Run Screen (Live)
```
┌─────────────────────┐
│ DRY RUN-EXPLORING   │
│ ─────────────────── │
│ Junctions: 12       │
│ Step: 45            │
│ Distance: 2340 mm   │
│                     │
│ Seeking endpoint... │
└─────────────────────┘
```

### Final Run Screen (Live)
```
┌─────────────────────┐
│ FINAL RUN-OPTIMIZED │
│ ─────────────────── │
│ Progress: 8/12      │
│ [=======-------]    │
│ Distance: 1450 mm   │
└─────────────────────┘
```

### Maze Complete Screen
```
┌─────────────────────┐
│ MAZE COMPLETE!      │
│ ─────────────────── │
│ Dry Run:  2340 mm   │
│ Final Run: 1450 mm  │
│                     │
│ Efficiency: 1.61x   │
└─────────────────────┘
```

---

## Endpoint Detection

The endpoint is detected by checking if most IR sensors read "white" (no line):

```cpp
int whiteSensorCount = 0;
for (int i = 0; i < 8; i++) {
  if (irReadings[i] == 0) whiteSensorCount++;
}
if (whiteSensorCount >= 6) {  // Most sensors see white
  // Endpoint detected
}
```

This works because:
- The line has high IR reflectance
- The white square has very low IR reflectance
- When robot is on white square, no line is detected

---

## Main Loop Flow

```
Loop() {
  1. Update button states
     └─ Check for mode changes
  
  2. Handle serial commands (debug/tuning)
  
  3. Read IR sensors
  
  4. Update IMU distance estimation
  
  5. Execute maze state machine:
  
     IF DRY_RUN:
       ├─ Detect junctions
       ├─ Record path
       ├─ Apply left-wall-following logic
       ├─ Follow line between junctions
       └─ Detect endpoint → Switch to IDLE
     
     ELSE IF FINAL_RUN:
       ├─ Follow line segment
       ├─ Detect junction
       ├─ Move to next step in path
       └─ Reach end → Switch to IDLE
     
     ELSE (IDLE):
       ├─ Display menu
       ├─ Wait for button input
       └─ Show final results if complete
}
```

---

## Serial Commands for Debugging

### Maze Commands
```
maze_status         - Print current maze state
maze_junctions      - List all discovered junctions
maze_graph          - Print adjacency graph
maze_path           - Print computed shortest path
maze_reset          - Reset all maze data
```

### Navigation Commands
```
dist_calibrate      - Calibrate accelerometer
dist_get            - Get current distance
dist_start          - Start distance tracking
dist_stop           - Stop distance tracking
diag                - Show diagnostics
```

---

## Tuning Parameters

### Control Gains (from config.h)
- `kp` - Proportional gain (line centering)
- `kd` - Derivative gain (smoothing)
- `baseSpeed` - Forward speed (60-120)
- `turnDelta` - Speed difference for turns (30-50)

### Sensor Thresholds
```
thresh_0=800        - Set channel 0 threshold to 800
```

### Distance Unit
```
dist_unit=50        - Set distance unit to 50mm per unit
```

---

## Coordinate System

The maze uses a 2D grid coordinate system:

```
      -Y (North)
        ↑
        │
← -X (West) │ +X (East) →
        │
        ↓
      +Y (South)

Start at (0, 0)
```

Directions:
- `DIR_NORTH (0)` - Movement up (-Y)
- `DIR_EAST (1)` - Movement right (+X)
- `DIR_SOUTH (2)` - Movement down (+Y)
- `DIR_WEST (3)` - Movement left (-X)

---

## Performance Metrics

### Dry Run
- **Distance Traveled**: Total meters/millimeters to reach endpoint
- **Time Taken**: Duration from start to endpoint
- **Junctions Found**: Total unique grid intersections discovered

### Final Run
- **Distance Traveled**: Optimized path distance
- **Time Taken**: Duration following shortest path
- **Efficiency Ratio**: `DryRunDistance / FinalRunDistance`
  - Ratio > 1 indicates optimization success
  - Higher ratio = better optimization

### Expected Performance
- **Small Maze**: 2-3x improvement
- **Large Maze**: 1.5-2.5x improvement
- **Complex Maze**: 1.2-2x improvement

---

## Limitations and Future Improvements

### Current Limitations
1. Left-wall-following assumes 90° turns only
2. No loop detection (can revisit junctions)
3. Distance estimation depends on IMU calibration
4. Limited to small maze size due to memory constraints

### Future Improvements
1. **Smart Junction Detection**: Use multiple sensor readings
2. **Loop Elimination**: Detect and skip previously visited paths
3. **Turn Estimation**: Track actual heading with gyroscope
4. **Path Smoothing**: Optimize turns to reduce distance
5. **Memory Optimization**: Use compressed maze representation
6. **Dynamic Pathfinding**: Recompute if new obstacles found

---

## Troubleshooting

### Bot doesn't start
- Check button pin connections (pins 2 and 3)
- Verify button debounce is working
- Check serial output for error messages

### Endpoint not detected
- Verify white square is actually white (high luminance)
- Adjust IR threshold values
- Check if `whiteSensorCount >= 6` threshold is correct

### Final run doesn't execute
- Verify dry run reached endpoint
- Check if `endJunctionIndex >= 0`
- Ensure pathfinding found valid path
- Check serial output for pathfinding errors

### Inaccurate distance
- Recalibrate accelerometer: `dist_calibrate`
- Adjust acceleration sensitivity in IMU
- Verify distance unit matches actual movement: `dist_unit=<value>`

### Path suboptimal
- Verify all junctions were discovered in dry run
- Check graph building: `maze_graph`
- Ensure no sensor errors caused missed junctions

---

## Code Structure Summary

```
Project Structure:
├── src/
│   ├── main.cpp           - Main loop, mode switching
│   ├── maze.h/cpp         - Maze state machine
│   ├── buttons.h/cpp      - Button input handling
│   ├── pathfinder.h/cpp   - Pathfinding algorithms
│   ├── movement.h/cpp     - Motor control
│   ├── sensors.h/cpp      - IR sensor reading & line following
│   ├── imu.h/cpp          - IMU and distance estimation
│   ├── oled.h/cpp         - Display functions
│   ├── config.h/cpp       - Configuration & tuning
│   └── [other files]
```

---

## Getting Started

1. **Hardware Setup**
   - Connect button 1 to pin 2, button 2 to pin 3
   - Verify OLED is working
   - Calibrate IR sensors with `thresh_` commands

2. **Initialization**
   - Power on the bot
   - Bot enters IDLE mode
   - OLED shows menu

3. **Dry Run**
   - Press Button 1
   - Bot explores maze using left-wall-following
   - Watch progress on OLED
   - Bot stops when white square detected

4. **Final Run**
   - Press Button 2
   - Bot follows computed shortest path
   - Watch progress bar on OLED
   - Bot stops at endpoint

5. **Review Results**
   - OLED shows efficiency ratio
   - Check serial output for detailed statistics

---

## References

- **BFS**: Standard graph traversal for unweighted shortest path
- **Dijkstra**: Classical algorithm for weighted shortest path
- **A\***: Heuristic search with admissible heuristic (Manhattan distance)
- **Left-Wall-Following**: Maze solving algorithm based on wall following rule

---

**Last Updated**: December 2025
**Version**: 1.0
**Author**: Line Follower Bot Project Team
