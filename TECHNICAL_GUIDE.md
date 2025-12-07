# Technical Implementation Guide

## File-by-File Implementation Details

### maze.h / maze.cpp
Core maze solving state machine

**Key Structures:**
- `MazeMode enum` - Three modes: IDLE, DRY_RUN, FINAL_RUN
- `Direction enum` - Four directions: N, E, S, W
- `Junction struct` - Grid cell with connectivity info
- `MazeState struct` - Global state tracker

**Key Functions:**
- `initMaze()` - Initialize with start junction at (0,0)
- `switchMazeMode()` - Change operating mode
- `handleJunction()` - Implement left-wall-following logic
- `detectEndpoint()` - Check for white square
- `computeShortestPath()` - Call pathfinding
- `executeNextStep()` - Move along final path

**Left-Wall-Following Algorithm:**
```cpp
void handleJunction() {
  // Get current junction
  Junction& current = mazeState.junctions[currentIdx];
  
  // Preference order: Left > Straight > Right > Back
  if (!current.wallWest) {
    // Turn left and follow
    turnLeft(...);
    recordTurn(LEFT);
  } else if (!current.wallNorth) {
    // Go straight
    moveForward(...);
    recordTurn(STRAIGHT);
  } else if (!current.wallEast) {
    // Turn right
    turnRight(...);
    recordTurn(RIGHT);
  } else {
    // Dead end - turn around
    turnAround(...);
    recordTurn(BACK);
  }
}
```

**State Transitions:**
```
IDLE --[Button 1]--> DRY_RUN
        ^                |
        |                v
        +----[Endpoint detected]
        
IDLE --[Button 2]--> FINAL_RUN
        ^                |
        |                v
        +----[Path complete]
```

---

### buttons.h / buttons.cpp
Hardware button interface with debouncing

**Configuration:**
```cpp
#define BUTTON_MODE_1_PIN 2      // Pull-up INPUT
#define BUTTON_MODE_2_PIN 3      // Pull-up INPUT
#define BUTTON_DEBOUNCE_MS 50    // Debounce time
```

**Debouncing Logic:**
```cpp
void updateButtonStates() {
  // Read current state (LOW when pressed due to pull-up)
  bool current = (digitalRead(PIN) == LOW);
  
  if (current && !wasPressed) {
    // Rising edge detected
    if (millis() - lastTime > DEBOUNCE_MS) {
      pressed = true;
      triggered = true;
      lastTime = millis();
    }
  }
  
  if (!current && wasPressed) {
    // Falling edge - button released
    pressed = false;
  }
}

bool wasButtonPressed() {
  if (triggered) {
    triggered = false;
    return true;
  }
  return false;
}
```

**Integration in main.cpp:**
```cpp
updateButtonStates();  // Call every loop iteration

if (wasButton1Pressed()) {
  switchMazeMode(MAZE_DRY_RUN);
}

if (wasButton2Pressed()) {
  switchMazeMode(MAZE_FINAL_RUN);
}
```

---

### pathfinder.h / pathfinder.cpp
Advanced pathfinding algorithms

**Graph Building:**
```cpp
void buildMazeGraph() {
  // For each pair of junctions
  for (i = 0; i < size; i++) {
    for (j = i+1; j < size; j++) {
      if (areMazeJunctionsAdjacent(i, j)) {
        // Connect bidirectionally
        junctions[i].neighbors[dir] = j;
        junctions[j].neighbors[opposite] = i;
      }
    }
  }
}

bool areMazeJunctionsAdjacent(j1, j2) {
  dx = abs(j2.x - j1.x);
  dy = abs(j2.y - j1.y);
  return (dx + dy == 1);  // Manhattan neighbors
}
```

**BFS Implementation:**
```cpp
int findPathBFS(start, end, outPath) {
  queue q;
  parent[start] = start;
  visited[start] = true;
  q.push(start);
  
  while (!q.empty()) {
    current = q.front();
    q.pop();
    
    if (current == end) break;
    
    // Check all 4 directions
    for (dir = 0; dir < 4; dir++) {
      neighbor = junctions[current].neighbors[dir];
      if (neighbor >= 0 && !visited[neighbor]) {
        visited[neighbor] = true;
        parent[neighbor] = current;
        q.push(neighbor);
      }
    }
  }
  
  // Reconstruct path
  current = end;
  while (current != start) {
    outPath.insert(outPath.begin(), current);
    current = parent[current];
  }
  outPath.insert(outPath.begin(), start);
  
  return outPath.size();
}
```

**Complexity Analysis:**

| Algorithm | Time | Space | Best Case |
|-----------|------|-------|-----------|
| BFS | O(V+E) | O(V) | Small mazes |
| Dijkstra | O(V²) | O(V) | Distance optimization |
| A* | O(E*log V) | O(V) | Large mazes |

---

### main.cpp Loop Structure

**Full Main Loop Flow:**
```cpp
void loop() {
  // 1. INPUT HANDLING
  updateButtonStates();
  if (wasButton1Pressed()) switchMazeMode(MAZE_DRY_RUN);
  if (wasButton2Pressed()) switchMazeMode(MAZE_FINAL_RUN);
  
  // 2. SERIAL COMMANDS
  handleSerialCommands();
  
  // 3. SENSOR INPUT
  readIRArray();
  diagnostics.loopCount++;
  
  // 4. DISTANCE TRACKING
  updateDistance();
  
  // 5. STATE MACHINE
  if (MAZE_DRY_RUN) {
    // EXPLORATION LOGIC
    updateDisplay();
    turnDetected = detect_turn();
    
    if (turnDetected) {
      // Junction found
      addJunction(newX, newY, direction);
      handleJunction();  // Left-wall-following
    } else {
      // Between junctions
      error = calculate_error();
      correction = calculatePDCorrection(error);
      setMotorSpeeds(baseSpeed + correction, baseSpeed - correction);
    }
    
  } else if (MAZE_FINAL_RUN) {
    // OPTIMIZATION LOGIC
    updateDisplay();
    turnDetected = detect_turn();
    
    if (turnDetected) {
      // At junction
      executeNextStep();  // Move along path
    } else {
      // Between junctions
      error = calculate_error();
      correction = calculatePDCorrection(error);
      setMotorSpeeds(baseSpeed + correction, baseSpeed - correction);
    }
    
  } else {
    // IDLE LOGIC
    updateDisplay();
    // Motors stopped, wait for input
  }
}
```

---

## Integration with Existing Systems

### Line Following (sensors.cpp)

The maze solver **reuses** existing line following:

```cpp
// Existing function - unchanged
float error = calculate_error();
float correction = calculatePDCorrection(error);

// Used in main.cpp for both dry run and final run
int leftSpeed = baseSpeed + correction;
int rightSpeed = baseSpeed - correction;
setMotorSpeeds(leftSpeed, rightSpeed);
```

### Motor Control (movement.cpp)

Uses existing motor functions:
```cpp
moveForward(speed);    // Used between junctions
turnLeft(speed);       // Used at junctions
turnRight(speed);      // Used at junctions
turnAround(speed);     // Used at dead ends
stopMotors();          // Used when stopping
setMotorSpeeds(L, R);  // For line centering
```

### Distance Tracking (imu.cpp)

Uses existing IMU integration:
```cpp
initDistanceEstimator(50);  // 50mm per unit
startDistanceTracking();    // Start in dry run
updateDistance();           // Every loop
float mm = getDistanceMM();  // For display
stopDistanceTracking();     // At endpoint
```

### OLED Display (oled.cpp)

New display functions added:
```cpp
oledDisplayMazeIdle();           // Menu screen
oledDisplayDryRunStatus(...);    // Progress during dry run
oledDisplayFinalRunStatus(...);  // Progress during final run
oledDisplayMazeComplete(...);    // Results screen
```

---

## Memory Usage Analysis

### Stack Memory (Temporary)
```cpp
std::vector<Junction> junctions;          // ~40 bytes each
std::vector<Direction> dryRunPath;        // ~1 byte each
std::vector<int> dryRunJunctions;         // ~4 bytes each
std::queue<int> bfs_queue;                // ~4 bytes per element
```

**For small maze (16 junctions):**
- Junctions: 640 bytes
- Paths: ~100 bytes  
- Queue: ~64 bytes
- **Total: ~1 KB**

### Static Memory (MazeState)
```cpp
MazeState mazeState;  // ~200 bytes baseline
Path shortestPath;    // ~100 bytes
// Total: ~300 bytes
```

### Total Expected Usage
- Small maze: 1.5-2 KB
- Medium maze: 3-5 KB
- Large maze: 8-12 KB

**Typical Teensy 4.0**: 1024 KB RAM → Plenty of headroom

---

## Communication Protocol

### Serial Output Format

**Maze Status:**
```
[MAZE] Switching mode: DRY RUN
[MAZE] Junction 1 at (0,-1)
[MAZE] Turn recorded: 0
[MAZE] ENDPOINT DETECTED!
[MAZE] System initialized
```

**Pathfinding Debug:**
```
[PATHFINDER] Building maze graph...
[PATHFINDER] Graph building complete
[PATHFINDER] BFS path found: 8
Path: 0 -> 1 -> 3 -> 5 -> 7 -> ...
```

**Button Events:**
```
[BUTTONS] Initialized
[BUTTONS] Button 1 pressed
[BUTTONS] Button 2 pressed
```

### Received Commands

Format: `<command>=<value>` or `<command>`

Examples:
```
kp=12.5           → Set Kp gain
speed=100         → Set base speed
maze_status       → Show maze state
dist_calibrate    → Calibrate IMU
```

---

## Error Handling

### No Endpoint Found
```cpp
void detectEndpoint() {
  bool isWhite = checkForWhiteSquare();
  if (isWhite) {
    // Success
    junctions[idx].isEnd = true;
    switchMazeMode(MAZE_IDLE);
  }
  // Otherwise: Keep exploring
}
```

### No Path Found
```cpp
void computeShortestPath() {
  if (endJunctionIndex < 0) {
    Serial.println("ERROR: No endpoint found!");
    return;  // Stay in IDLE
  }
  // Pathfinding
}
```

### Invalid Button Press
```cpp
if (wasButton2Pressed()) {
  // Only start if dry run complete
  if (endJunctionIndex >= 0 && 
      currentMode != MAZE_FINAL_RUN) {
    switchMazeMode(MAZE_FINAL_RUN);
  }
}
```

---

## Performance Optimization Tips

### 1. Faster Turn Detection
```cpp
// Current: O(8) reads per detect_turn()
int leftCount = 0, rightCount = 0;
for (int i = 0; i < 8; i++) {
  if (i < 4) leftCount += irReadings[i];
  else rightCount += irReadings[i];
}
return (rightCount > 4) ? RIGHT : (leftCount > 4) ? LEFT : STRAIGHT;
```

### 2. Efficient Junction Storage
```cpp
// Current: std::vector with duplication
// Optimization: Use hash table or fixed array with ID map
std::map<int, Junction> junctionMap;  // Key = (x * 16 + y)
```

### 3. Faster Pathfinding
```cpp
// Use A* instead of BFS for large mazes
findOptimalPath(start, end, path, STRATEGY_ASTAR);
```

### 4. Reduced Display Updates
```cpp
// Current: Update every 500ms
// Optimization: Update only when changed
static int lastJunctionCount = 0;
if (junctions.size() != lastJunctionCount) {
  updateDisplay();
  lastJunctionCount = junctions.size();
}
```

---

## Testing Checklist

### Unit Tests
- [ ] Button debouncing works
- [ ] Maze state transitions correctly
- [ ] BFS finds shortest path
- [ ] Junction detection accurate
- [ ] Endpoint detection reliable

### Integration Tests
- [ ] Main loop runs without blocking
- [ ] Display updates smoothly
- [ ] Serial commands responsive
- [ ] Distance tracking accumulates

### System Tests
- [ ] Dry run explores entire maze
- [ ] Final run follows correct path
- [ ] Efficiency ratio improves
- [ ] Bot stops at endpoint

---

## Debugging Techniques

### Serial Monitor Output
```cpp
Serial.print("[MAZE] Current junction: ");
Serial.println(currentJunction);
Serial.print("[MAZE] Found ");
Serial.print(junctions.size());
Serial.println(" junctions so far");
```

### OLED Display Debug
```cpp
display.setCursor(0, 50);
display.print("Junction: ");
display.print(currentJunction);
display.display();
```

### Maze Graph Visualization
```
Serial output from maze_graph command:
Junction 0 (0,0) Neighbors: 1(N) 4(E) -1(S) -1(W)
Junction 1 (0,-1) Neighbors: -1(N) 2(E) 0(S) -1(W)
Junction 2 (1,-1) Neighbors: -1(N) -1(E) 3(S) 1(W)
...
```

---

## Future Enhancement Ideas

1. **Obstacle Detection**
   - Monitor for unexpected wall collisions
   - Record obstacle locations
   - Recompute path if blocked

2. **Loop Elimination**
   - Detect cycles in exploration
   - Skip revisited areas in final run
   - Reduce redundant movement

3. **Adaptive Speed**
   - Slow down at junctions
   - Speed up on straightaways
   - Smooth acceleration/deceleration

4. **Path Smoothing**
   - Optimize turn sequences
   - Reduce sharp angles
   - Minimize distance between junctions

5. **Persistent Storage**
   - Save maze map to EEPROM
   - Reload for repeated runs
   - Compare multiple mazes

---

**Version**: 1.0 - December 2025
