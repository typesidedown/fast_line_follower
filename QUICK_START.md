# Maze Solver Quick Start Guide

## Hardware Checklist

Before running the maze solver, ensure:

- [ ] 2 Push buttons connected to pins 2 and 3
- [ ] OLED display functioning
- [ ] 8 IR sensors calibrated
- [ ] Motors responding to commands
- [ ] IMU connected and working
- [ ] Battery fully charged

## Pin Configuration

| Component | Pin | Purpose |
|-----------|-----|---------|
| Button 1 | 2 | Start DRY RUN |
| Button 2 | 3 | Start FINAL RUN |
| OLED SDA | Wire1 SDA | I2C Display |
| OLED SCL | Wire1 SCL | I2C Display |

## Step-by-Step Operation

### Step 1: Power On
```
Bot starts in IDLE mode
OLED shows: "MAZE SOLVER" menu
Status: Ready
```

### Step 2: Start Dry Run
```
Press Button 1
Bot enters DRY RUN mode
Starts exploring maze
OLED shows: Live progress (junctions, steps, distance)
Watch for: Junction markers, smooth turns
```

**What the bot does:**
- Follows line straight sections
- Detects junctions (intersections)
- Uses left-wall-following algorithm to navigate
- Records all junctions and paths
- Continues until white square detected

**Duration:** Depends on maze size (30 seconds - 5 minutes typical)

### Step 3: Check Results
```
Bot stops when reaching endpoint
OLED shows: IDLE menu again
Status: Dry run complete, ready for final run
```

Review in serial monitor:
- Total junctions found
- Path taken
- Distance traveled

### Step 4: Start Final Run
```
Press Button 2
Bot enters FINAL RUN mode
Executes computed shortest path
OLED shows: Progress bar, step count, distance
Watch for: Smooth movement, quick execution
```

**What the bot does:**
- Follows computed shortest path
- Turns at correct junctions
- Navigates to endpoint
- Calculates efficiency ratio

**Duration:** Should be faster than dry run

### Step 5: View Results
```
Bot completes final run
OLED shows: Final results
- Dry run distance
- Final run distance
- Efficiency ratio
```

## Common Issues & Quick Fixes

| Issue | Cause | Fix |
|-------|-------|-----|
| Bot doesn't move | Buttons not connected | Check pin 2 & 3 connections |
| OLED blank | Display not initialized | Power cycle bot |
| Endpoint not found | White square too small/wrong color | Increase white sensor count threshold in code |
| Path too long | Junctions not properly detected | Tune IR thresholds |
| Motor imbalance | Asymmetric motor speeds | Use `offset=<value>` command |
| Distance inaccurate | IMU not calibrated | Run `dist_calibrate` |

## Serial Commands (for Debugging)

```
# Check status
help                    - Show all commands
status                  - Show configuration
maze_status             - Show maze state
diag                    - Show diagnostics

# Motor testing
motor_test_forward      - Test forward movement
motor_test_left         - Test left turn
motor_test_right        - Test right turn

# Tuning
kp=12.5                 - Set proportional gain
kd=8.0                  - Set derivative gain
speed=100               - Set forward speed (80-120)
delta=40                - Set turn speed delta (30-50)
offset=5                - Balance left/right motors (-50 to +50)

# Distance tracking
dist_calibrate          - Calibrate accelerometer
dist_start              - Start tracking
dist_stop               - Stop tracking
dist_get                - Get current distance
dist_reset              - Reset to zero
dist_unit=50            - Set distance unit (mm)

# IR Sensors
thresh_0=800            - Set channel 0 threshold
thresh_1=800            - Set channel 1 threshold (etc.)
sensors                 - Show IR readings
display_ir_digital      - Show digital IR on OLED
display_ir_raw          - Show raw ADC values on OLED

# Maze debugging
maze_graph              - Print junction connections
maze_path               - Print computed path
maze_reset              - Reset all maze data
```

## Performance Expectations

### Typical Maze (8x8 grid)
- **Dry Run Time**: 2-3 minutes
- **Dry Run Distance**: 1500-2500 mm
- **Final Run Time**: 30-60 seconds
- **Final Run Distance**: 800-1200 mm
- **Efficiency**: 1.5-2.5x

### Small Maze (4x4 grid)
- **Dry Run Distance**: 400-800 mm
- **Final Run Distance**: 200-400 mm
- **Efficiency**: 2-4x

## Optimization Tips

1. **Better Line Following**
   - Adjust `kp` and `kd` for smoother curves
   - Set `offset` to balance left/right motors
   - Tune `speed` for current battery voltage

2. **Accurate Junction Detection**
   - Calibrate IR threshold values for your lighting
   - Ensure consistent sensor height above line

3. **Faster Execution**
   - Increase `speed` for final run (carefully)
   - Ensure motors aren't slipping

4. **Better Endpoint Detection**
   - Verify white square is large enough
   - Adjust white sensor count threshold if needed

## Understanding the Results

### Efficiency Ratio

```
Efficiency = Dry Run Distance / Final Run Distance

Example:
- Dry Run: 2340 mm
- Final Run: 1450 mm
- Efficiency: 2340 / 1450 = 1.61x

Meaning: Final run is 61% shorter (39% of original)
```

### Why Final Run is Shorter

1. **Dry run explores** - visits many dead ends and backtracks
2. **Final run optimizes** - skips unnecessary paths
3. **Pathfinding works** - finds mathematically shortest path

### Expected Ratios

- Simple maze: 3-4x improvement
- Moderate maze: 1.5-2.5x improvement
- Complex maze: 1.2-1.8x improvement

## Data Saved in Bot Memory

After successful completion, the bot remembers:

1. **Junction Map**
   - All discovered grid positions
   - Connections between junctions

2. **Computed Path**
   - Sequence of junctions to reach goal
   - Optimal routing stored

3. **Statistics**
   - Times for both runs
   - Distances traveled
   - Efficiency metrics

**Note**: Data is lost on power cycle. Consider adding EEPROM storage for persistent data.

## Troubleshooting Checklist

Before maze run:
- [ ] All buttons working (try each)
- [ ] OLED displaying text
- [ ] IR sensors detecting line
- [ ] Motors responding to speed commands
- [ ] Gyroscope returning data

During dry run:
- [ ] Bot following line smoothly
- [ ] Detecting turns correctly
- [ ] Making proper 90° turns
- [ ] Distance tracking increasing

During final run:
- [ ] Bot following same path faster
- [ ] Making turns at right positions
- [ ] Final distance less than dry run
- [ ] Reaching correct endpoint

## Safety Notes

- Keep fingers clear of spinning wheels
- Ensure clear test area (no obstacles)
- Stop bot immediately if erratic behavior
- Check battery voltage if performance degrades
- Don't modify hardware during operation

## Next Steps

1. **Improve Path Optimization**
   - Try `STRATEGY_DIJKSTRA` for distance-based optimization
   - Try `STRATEGY_ASTAR` for faster pathfinding

2. **Add Features**
   - Obstacle avoidance
   - Loop detection in maze
   - Path smoothing for faster turns

3. **Performance Tuning**
   - Record multiple runs
   - Find optimal control parameters
   - Calibrate distance estimation

---

**Happy Maze Solving!** 🤖
