# Maze Solver - Architecture & Data Flow Diagrams

## System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         MAIN LOOP (main.cpp)                     │
│                     Runs at ~100 Hz Loop Frequency               │
└─────────────────────────┬───────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
        ▼                 ▼                 ▼
┌───────────────┐ ┌──────────────┐ ┌──────────────┐
│   BUTTONS     │ │   SENSORS    │ │     IMU      │
│  INPUT        │ │  + CONTROL   │ │   (Dist.)    │
│ (pins 2,3)    │ │ (IR array)   │ │              │
└───────┬───────┘ └──────┬───────┘ └──────┬───────┘
        │                │                │
        │ Button Events  │ Line Data      │ Distance Data
        │                │                │
        ▼                ▼                ▼
    ┌─────────────────────────────────────────┐
    │      STATE MACHINE LOGIC                 │
    │  (controls maze state transitions)       │
    └──────────────┬──────────────────────────┘
                   │
        ┌──────────┼──────────┐
        │          │          │
        ▼          ▼          ▼
    ┌────────┐ ┌─────────┐ ┌─────────┐
    │ IDLE   │ │ DRY RUN │ │ FINAL   │
    │        │ │ (Explore)  │ RUN     │
    └────────┘ └──┬──────┘ └────┬────┘
                  │             │
            ┌─────▼──────┐  ┌───▼──────┐
            │  PATHFIND  │  │ FOLLOW   │
            │  ALGORITHM │  │ PATH     │
            └────────────┘  └──────────┘
```

---

## Operating Mode State Machine

```
                    ┌─────────────┐
                    │    IDLE     │
                    │  (Ready)    │
                    └──────┬──────┘
                           │
                ┌──────────┴──────────┐
                │                     │
        Button 1 pressed      Button 2 pressed
       (Only if dry run      (Only if endpt found
        not running)          AND dry run complete)
                │                     │
                ▼                     ▼
         ┌──────────────┐      ┌────────────────┐
         │  DRY RUN     │      │  FINAL RUN     │
         │ (Exploring)  │      │ (Optimizing)   │
         │              │      │                │
         │ Actions:     │      │ Actions:       │
         │ • Detect     │      │ • Follow line  │
         │   junctions  │      │ • Execute path │
         │ • Map maze   │      │ • Count steps  │
         │ • Follow left│      │ • Record dist  │
         │   wall rule  │      │                │
         │              │      │                │
         │ Exit when:   │      │ Exit when:     │
         │ White square │      │ Path complete  │
         │ detected     │      │                │
         └──────┬───────┘      └────────┬───────┘
                │                       │
                └───────────┬───────────┘
                            │
                    Endpoint found
                  Final run complete
                            │
                            ▼
                    ┌─────────────┐
                    │    IDLE     │
                    │ (Show Results)
                    └─────────────┘
```

---

## Dry Run Execution Flow

```
Start Dry Run
    │
    ▼
┌─────────────────────────┐
│ Read IR Sensor Array    │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ Calculate Line Error    │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ Run PD Controller       │
│ → Calculate motor speed │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ Check for Junction      │
│ detect_turn()           │
└────────┬────────────────┘
         │
         ├─ No Junction ──▶ Move Forward
         │                 (set motor speeds)
         │
         └─ Junction ──────▶┌──────────────────────┐
                            │ Update Grid Position │
                            │ Add/Goto Junction    │
                            └──────┬───────────────┘
                                   │
                                   ▼
                            ┌──────────────────────┐
                            │ Check for Endpoint   │
                            │ (White Square?)      │
                            └──────┬───────────────┘
                                   │
                        ┌──────────┴──────────┐
                        │                     │
                    Endpoint Found      Continue Exploring
                        │                     │
                        ▼                     ▼
                   Record Statistics    ┌──────────────────┐
                   Save Results         │ Left-Wall Logic  │
                   Return to IDLE       │ (Choose direction)
                                        └──────┬───────────┘
                                               │
                                        ┌──────▼──────────┐
                                        │ Execute Turn &  │
                                        │ Move Forward    │
                                        └─────────────────┘
```

---

## Final Run Execution Flow

```
Start Final Run (Pathfinding Complete)
    │
    ▼
┌──────────────────────────────┐
│ Build Maze Graph             │
│ Connect discovered junctions │
└──────────┬───────────────────┘
           │
           ▼
┌──────────────────────────────┐
│ Run Pathfinding Algorithm    │
│ (BFS/Dijkstra/A*)            │
└──────────┬───────────────────┘
           │
           ▼
┌──────────────────────────────┐
│ Get Shortest Path as List    │
│ [J0→J3→J5→J8→Endpoint]       │
└──────────┬───────────────────┘
           │
           ▼
┌──────────────────────────────┐
│ Reset Counters               │
│ Reset Distance Tracker       │
│ Step = 0                     │
└──────────┬───────────────────┘
           │
           ▼
         Loop:
         │
         ▼
┌──────────────────────────────┐
│ Read IR Sensors              │
└──────────┬───────────────────┘
           │
           ▼
┌──────────────────────────────┐
│ PD Controller → Motor Speed  │
└──────────┬───────────────────┘
           │
           ▼
┌──────────────────────────────┐
│ Check for Next Junction      │
└──────────┬───────────────────┘
           │
    ┌──────┴──────┐
    │             │
No Junction   Junction
    │             │
    ▼             ▼
Move Forward  Execute Next Step
              │
              ▼
         ┌──────────────┐
         │ Step++       │
         │ Check if Done│
         └──────┬───────┘
                │
        ┌───────┴───────┐
        │               │
    More Steps      Path Done
        │               │
        ▼               ▼
    Continue       Record Statistics
    Loop           Save Results
                   Return to IDLE
```

---

## Data Structure Relationships

```
┌──────────────────────────────────────────────────────────┐
│                  MazeState (Global)                       │
├──────────────────────────────────────────────────────────┤
│                                                            │
│  ┌─ currentMode: IDLE | DRY_RUN | FINAL_RUN              │
│  ├─ currentDirection: NORTH | EAST | SOUTH | WEST        │
│  ├─ currentJunction: int (index into junctions vector)    │
│  │                                                         │
│  ├─ junctions[ ] ──────┬──────────────────────────┐       │
│  │  std::vector        │ For each Junction:       │       │
│  │                     │                          │       │
│  │                     ▼                          │       │
│  │          ┌──────────────────────┐              │       │
│  │          │ Junction struct       │              │       │
│  │          ├──────────────────────┤              │       │
│  │          │ x, y (grid position) │              │       │
│  │          │ visited (bool)       │              │       │
│  │          │ isEnd (bool)         │              │       │
│  │          │ neighbors[4]         │──┐           │       │
│  │          │ (N/E/S/W directions) │  │           │       │
│  │          └──────────────────────┘  │           │       │
│  │                                    │           │       │
│  │                    Points to other junction    │       │
│  │                    indices in junctions[ ]    │       │
│  │                                               │       │
│  ├─ dryRunPath[ ]──────────────────────────────┐  │       │
│  │  Sequence of Direction enum values:          │  │       │
│  │  [DIR_NORTH, DIR_EAST, DIR_SOUTH, ...]      │  │       │
│  │                                             │  │       │
│  ├─ shortestPath ──────────────────────────────┴──┼─┐     │
│  │  ┌──────────────────────────────────────┐      │ │     │
│  │  │ Path struct                          │      │ │     │
│  │  ├──────────────────────────────────────┤      │ │     │
│  │  │ junctionIndices[ ] ──────────────────┼──────┘ │     │
│  │  │ [0, 3, 5, 8, 12]  ← Index numbers   │        │     │
│  │  │ turns[ ]                             │        │     │
│  │  │ length                               │        │     │
│  │  └──────────────────────────────────────┘        │     │
│  │                                                   │     │
│  ├─ dryRunDistance: float (mm)                       │     │
│  ├─ finalRunDistance: float (mm)                     │     │
│  └─ Statistics for display & comparison              │     │
│                                                      │     │
└──────────────────────────────────────────────────────┴─────┘
```

---

## Function Call Hierarchy

```
main()
│
├─ setup()
│  ├─ initMovement()
│  ├─ initSensors()
│  ├─ initIMU()
│  ├─ initOLED()
│  ├─ initializeConfig()
│  ├─ initButtons()          ← NEW
│  ├─ initMaze()             ← NEW
│  └─ initDistanceEstimator()
│
└─ loop() [Infinite]
   ├─ updateButtonStates()   ← NEW
   │  ├─ wasButton1Pressed() → switchMazeMode(DRY_RUN)
   │  └─ wasButton2Pressed() → switchMazeMode(FINAL_RUN)
   │
   ├─ handleSerialCommands()
   │
   ├─ readIRArray()
   ├─ updateDistance()
   │
   └─ STATE MACHINE:
      │
      ├─ IF MAZE_DRY_RUN:
      │  ├─ oledDisplayDryRunStatus()
      │  ├─ detect_turn()
      │  │  ├─ addJunction()        ← NEW
      │  │  └─ handleJunction()     ← NEW (calls turnLeft/Right/Around)
      │  ├─ calculate_error()
      │  ├─ calculatePDCorrection()
      │  └─ setMotorSpeeds()
      │
      ├─ ELSE IF MAZE_FINAL_RUN:
      │  ├─ oledDisplayFinalRunStatus()
      │  ├─ detect_turn()
      │  │  └─ executeNextStep()    ← NEW
      │  ├─ calculate_error()
      │  ├─ calculatePDCorrection()
      │  └─ setMotorSpeeds()
      │
      └─ ELSE (IDLE):
         ├─ oledDisplayMazeIdle()
         └─ oledDisplayMazeComplete()  (if finished)

[Initial] switchMazeMode() ← NEW
          ├─ startDryRun()            ← NEW
          │  └─ moveForward()
          ├─ startFinalRun()          ← NEW
          │  ├─ computeShortestPath() ← NEW
          │  │  └─ findOptimalPath()  ← NEW (in pathfinder.cpp)
          │  │     ├─ buildMazeGraph()
          │  │     └─ findPathBFS/Dijkstra/AStar()
          │  └─ moveForward()
          └─ stopMotors()
```

---

## Pathfinding Algorithm Flow

```
computeShortestPath()
│
├─ Check: endJunctionIndex valid?
│
├─ findOptimalPath(start=0, end=endJunctionIndex)
│  │
│  ├─ SELECT Algorithm based on STRATEGY:
│  │
│  ├─ IF STRATEGY_BFS:
│  │  │
│  │  ├─ Initialize: parent[ ], visited[ ]
│  │  ├─ Queue: push(start)
│  │  │
│  │  └─ BFS Loop:
│  │     │
│  │     ├─ Pop current from queue
│  │     ├─ If current == end: BREAK
│  │     │
│  │     ├─ For each neighbor in 4 directions:
│  │     │  ├─ neighbor = current.neighbors[dir]
│  │     │  ├─ If neighbor not visited:
│  │     │  │  ├─ Mark visited[neighbor] = true
│  │     │  │  ├─ Set parent[neighbor] = current
│  │     │  │  └─ Push neighbor to queue
│  │     │  └─ Next neighbor
│  │     │
│  │     └─ Next iteration
│  │
│  ├─ RECONSTRUCT PATH:
│  │  │
│  │  ├─ Start from: end
│  │  ├─ Follow parent pointers back to start
│  │  ├─ Build path as sequence of junction indices
│  │  └─ Reverse to get: [start → ... → end]
│  │
│  └─ Return: path length
│
└─ Store result in: mazeState.shortestPath
```

---

## Control Loop Timing

```
Loop Frequency: ~100 Hz (10ms per iteration)

┌────────────────────────────────────┐
│        LOOP ITERATION              │
│        (10 milliseconds)           │
├────────────────────────────────────┤
│                                    │
│ 1. Button Processing    ┐          │
│    (< 1ms)              │          │
│                         │          │
│ 2. Serial Commands      │ ~7ms     │
│    (if any, < 5ms)      │          │
│                         │          │
│ 3. Sensor Reading       │          │
│    (IR array, ~2ms)     ┘          │
│                                    │
│ 4. PD Control          ┐           │
│    (1-2ms)             │ ~3ms      │
│                        │           │
│ 5. Display Update      │           │
│    (if time, ~1ms)     ┘           │
│                                    │
│ 6. Wait for next cycle             │
│                                    │
└────────────────────────────────────┘
         ↓
    Repeat
```

---

## Memory Layout

```
┌─────────────────────────────────────────────────────────┐
│            Teensy 4.0 RAM (1024 KB)                      │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌────────────────────────────────────┐                  │
│  │  Arduino Runtime (100 KB)          │                  │
│  └────────────────────────────────────┘                  │
│                                                           │
│  ┌────────────────────────────────────┐                  │
│  │  Configuration & Tuning (5 KB)     │                  │
│  │  - sensorConfig                     │                  │
│  │  - motorConfig                      │                  │
│  │  - pdController                     │                  │
│  └────────────────────────────────────┘                  │
│                                                           │
│  ┌────────────────────────────────────┐                  │
│  │  Maze Data (Variable, ~3-10 KB)    │                  │
│  │  - junctions[ ]                     │                  │
│  │  - dryRunPath[ ]                    │                  │
│  │  - shortestPath                     │                  │
│  │  - buttonState                      │                  │
│  └────────────────────────────────────┘                  │
│                                                           │
│  ┌────────────────────────────────────┐                  │
│  │  IMU & Sensor Data (10 KB)         │                  │
│  │  - distEstimator                    │                  │
│  │  - irReadings[ ]                    │                  │
│  │  - rawIrReadings[ ]                 │                  │
│  └────────────────────────────────────┘                  │
│                                                           │
│  ┌────────────────────────────────────┐                  │
│  │  OLED Display Buffer (1 KB)        │                  │
│  └────────────────────────────────────┘                  │
│                                                           │
│  ┌────────────────────────────────────┐                  │
│  │  Serial Buffer (4 KB)              │                  │
│  └────────────────────────────────────┘                  │
│                                                           │
│  ┌────────────────────────────────────┐                  │
│  │  Stack & Working Memory (~800 KB)  │                  │
│  │  (Functions, temporary variables)   │                  │
│  └────────────────────────────────────┘                  │
│                                                           │
└─────────────────────────────────────────────────────────┘

Typical usage: ~150-200 KB (15-20%)
Leaves: ~800 KB for runtime (plenty of headroom)
```

---

## Serial Communication Protocol

```
BOT → COMPUTER (Output)
│
├─ System Messages
│  ├─ [BUTTONS] Button 1 pressed
│  ├─ [MAZE] Switching mode: DRY RUN
│  ├─ [MAZE] Junction 5 at (2,-1)
│  ├─ [MAZE] ENDPOINT DETECTED!
│  └─ [PATHFINDER] BFS path found: 8
│
├─ Debug/Status
│  ├─ Loop count: 1234
│  ├─ Distance: 2450 mm
│  └─ === DIAGNOSTICS ===
│
└─ Query Responses
   ├─ Kp: 12.5, Kd: 8.0
   ├─ Raw IR: 450 520 600 ...
   ├─ Digital IR: 01100110
   └─ === MAZE STATE ===

COMPUTER → BOT (Input)
│
├─ Control Commands
│  ├─ kp=<float>         (Proportional gain)
│  ├─ kd=<float>         (Derivative gain)
│  ├─ speed=<int>        (Base speed)
│  └─ offset=<int>       (Motor balance)
│
├─ Sensor Commands
│  ├─ thresh_0=<int>     (Sensor threshold)
│  └─ sensors            (Show readings)
│
├─ Motor Commands
│  ├─ motor_test_forward
│  ├─ motor_test_left
│  └─ motor_test_right
│
├─ Maze Commands
│  ├─ maze_status
│  ├─ maze_graph
│  └─ maze_reset
│
└─ Status Commands
   ├─ help
   ├─ status
   └─ diag

All at 115200 baud
```

---

## Efficiency Calculation

```
Efficiency = Dry Run Distance / Final Run Distance

Example:
┌──────────────────────────────────┐
│  Dry Run:   2340 mm traveled     │ (explores + backtracks)
│  Final Run: 1450 mm traveled     │ (optimized path)
│                                  │
│  Efficiency = 2340 / 1450 = 1.61x │
│                                  │
│  Meaning: Final run is 61% shorter
│           Saved 890 mm (38% reduction)
└──────────────────────────────────┘

Typical Ranges:
├─ Simple maze: 3-4x improvement
├─ Moderate maze: 1.5-2.5x improvement
└─ Complex maze: 1.2-1.8x improvement
```

---

**Version**: 1.0  
**Date**: December 2025
