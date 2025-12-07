# Maze Solver Configuration & Tuning Guide

## Overview

This guide explains how to configure and optimize the maze solver for your specific hardware and environment.

---

## 1. IR Sensor Calibration

### Initial Calibration

Before running the maze, calibrate IR sensor thresholds.

**Steps:**

1. **Measure baseline on the line:**
   ```
   Place bot on black line
   Serial command: sensors
   Note down the 8 readings
   ```

2. **Measure baseline off the line:**
   ```
   Place bot on white area (off line)
   Serial command: sensors
   Note down the 8 readings
   ```

3. **Set thresholds as midpoint:**
   ```
   If on-line = 200, off-line = 900
   Threshold = (200 + 900) / 2 = 550
   
   Command: thresh_0=550 (repeat for 0-7)
   ```

### Testing Calibration

```
Serial command: display_ir_digital

While moving on line:   Output should be: 00000000 or 11111111
While turning:          Output should show pattern like: 01000000

If output is flickering randomly:
  → Threshold too close to actual value
  → Increase margin: (line + off_line) / 2 ± 50
```

### Sensor-Specific Tuning

Some sensors may need different thresholds due to:
- Aging IR LED brightness
- Manufacturing tolerance
- Dust/contamination on lens

Command: `thresh_<channel>=<value>`

```
thresh_0=550   # Center-left
thresh_1=560   # Left
thresh_2=570   # Left-outer
thresh_3=580   # Left-far
thresh_4=540   # Right-center
thresh_5=550   # Right
thresh_6=560   # Right-outer
thresh_7=570   # Right-far
```

---

## 2. PD Controller Tuning

The bot uses a PD controller for line following:

```
Correction = Kp * error + Kd * (error - lastError)
LeftSpeed = BaseSpeed + Correction
RightSpeed = BaseSpeed - Correction
```

### Initial Tuning (Safe Values)

```
kp=10.0         # Start conservative
kd=5.0          # Damping factor
baseSpeed=80    # Slow for testing
```

### Tuning Process

**1. Increase Kp (Proportional Gain)**

Start at kp=10, increase in steps of 5:
```
kp=10  → Test on straight line
        → Measure oscillation
        → If stable, increase
        
kp=15  → More responsive
kp=20  → Reaching limit?

Stop when: Bot oscillates significantly
Optimal: Just before oscillation starts
```

**Symptoms of High Kp:**
- Jerky movements
- Sharp oscillations
- Unstable on curves
- Response: Reduce Kp by 5

**Symptoms of Low Kp:**
- Slow to respond
- Drifts off line slowly
- Stable but imprecise
- Response: Increase Kp by 5

**2. Tune Kd (Derivative Gain)**

Once Kp is set, adjust Kd (controls damping):

```
kd=2   → Minimal damping
kd=5   → Medium damping
kd=8   → Heavy damping
kd=10  → Maximum smoothing
```

**Symptoms of High Kd:**
- Very smooth, sluggish response
- Slow to correct errors
- May miss tight turns
- Response: Reduce Kd

**Symptoms of Low Kd:**
- Jerky, twitchy movements
- Overshoots corrections
- Unstable
- Response: Increase Kd

### Tuning for Maze Corners

At 90° corners:

1. Slow down before corner
2. Increase Kd for smooth turn entry
3. Return to normal for straight section

```
# For smooth corners
kp=12.0
kd=8.0
baseSpeed=80
```

### Testing Your Tuning

```
# Run in circle (for testing turns)
Place bot at corner
Command: kp=15, kd=5, speed=100

Observe:
□ Enters corner smoothly
□ Maintains line contact
□ Exits without overshooting
□ No oscillation

If failed: Adjust Kp/Kd and retry
```

---

## 3. Speed & Turn Configuration

### Base Speed

Controls forward movement speed (PWM 0-255):

```
speed=60    # Very slow, precise (calibration)
speed=80    # Slow, safe (maze exploration)
speed=100   # Medium, normal operation
speed=120   # Fast, for final run
speed=150   # Very fast (risky, needs good tuning)
```

**Effects:**
- Low speed: More torque, better accuracy, takes longer
- High speed: Faster but less control, battery drain

### Turn Delta

Speed difference between motors during turns:

```
delta=30    # Gentle 90° turns
delta=40    # Standard 90° turns
delta=50    # Aggressive turns
delta=60    # Sharp turns (risky)
```

**How it works:**
```
When turning left:
  LeftMotor = BaseSpeed - TurnDelta
  RightMotor = BaseSpeed + TurnDelta
  
Example (BaseSpeed=80, Delta=40):
  Left = 40
  Right = 120
  → Left motor slower → bot turns left
```

**Tuning for Your Maze:**
- If turns are too slow: Increase delta
- If turns overshoot: Decrease delta
- If motors slip: Increase base speed slightly

### Motor Offset

Corrects imbalanced motors (permanent asymmetry):

```
offset=0      # No correction
offset=5      # Right motor slightly faster
offset=-5     # Left motor slightly faster
offset=10     # Significant right bias
offset=-10    # Significant left bias
```

**How to find correct offset:**

1. Run straight on line
2. Observe drift direction
   - Drifts left → Right motor slower → Increase offset
   - Drifts right → Left motor slower → Decrease offset
3. Adjust by ±2 and retest
4. Fine-tune until straight

```
Example sequence:
offset=0    # Drifts right → Need positive offset
offset=5    # Still drifts right → Need more
offset=10   # Drifts left now → Too much
offset=7    # Perfect! Save this value
```

---

## 4. Distance Unit Configuration

The IMU estimates distance traveled:

```
dist_unit=50      # Default: 50mm per unit
```

This affects "distance" display and calculations.

### Calibration Steps

1. **Measure actual distance**
   ```
   Mark start position with tape
   Mark end position exactly 1 meter away
   Place bot at start, facing end
   ```

2. **Run distance tracking**
   ```
   Command: dist_reset
   Command: dist_calibrate
   Command: motor_test_forward
   Let bot run toward 1m mark
   Command: dist_stop (when bot reaches mark)
   Command: dist_get
   ```

3. **Calculate actual unit size**
   ```
   If bot traveled 1000mm and reported:
   Distance: 20 units
   
   Actual unit size = 1000 / 20 = 50mm per unit
   (Use this value with dist_unit=<value>)
   ```

### Tuning for Accuracy

If distance estimates are off:

```
Reported < Actual (bot thinks it went less than it did)
→ Unit size too large
→ Use smaller value: dist_unit=40

Reported > Actual (bot thinks it went more)
→ Unit size too small
→ Use larger value: dist_unit=60
```

### Why Calibration Matters

- Display shows correct progress
- Path distance calculations accurate
- Efficiency ratio meaningful
- Useful for obstacle detection

---

## 5. Endpoint Detection Tuning

The white square detection uses IR sensor data:

```cpp
// Current threshold: 6+ sensors see no line
if (whiteSensorCount >= 6) → Endpoint detected
```

### Adjusting Sensitivity

**If endpoint is not detected:**

1. Check white square is actually white
2. Reduce required sensor count
   - Edit maze.cpp, function `checkForWhiteSquare()`
   - Change `>= 6` to `>= 5` or `>= 4`
   - Recompile and upload

3. Test again
   ```
   Run dry run
   Watch serial output
   [MAZE] ENDPOINT DETECTED!
   ```

**If endpoint is detected too early:**

1. Increase required sensor count
   - Change `>= 6` to `>= 7`
   - Recompile

2. Verify white square is large enough
   - Must be larger than robot width

---

## 6. Recommended Configurations

### Configuration 1: Precise & Safe (Default)
```
kp=10.0
kd=5.0
baseSpeed=80
delta=40
offset=0
thresh_0-7=550-580 (adjust for your sensors)
dist_unit=50
```

**Use for:**
- First test run
- Difficult mazes
- Learning the system
- Safe operation

### Configuration 2: Fast & Balanced
```
kp=12.5
kd=7.0
baseSpeed=100
delta=45
offset=0 (after tuning)
thresh_0-7=550-580
dist_unit=50
```

**Use for:**
- Standard operation
- Small mazes
- Good sensor conditions
- Medium battery voltage

### Configuration 3: Maximum Speed (Risky)
```
kp=15.0
kd=8.0
baseSpeed=120
delta=50
offset=0 (must be tuned!)
thresh_0-7=550-580
dist_unit=50
```

**Use for:**
- Final competitive run
- Excellent tuning
- Fresh battery
- Expert users only

### Configuration 4: Sharp Turns
```
kp=14.0
kd=9.0
baseSpeed=90
delta=50
offset=0
thresh_0-7=550-580
dist_unit=50
```

**Use for:**
- Tight turn mazes
- Narrow corridors
- Smooth IR sensors

---

## 7. Testing & Validation

### Line Following Test
```
Test on a straight black line (2-3 meters)

Command: kp=<value>, kd=<value>, speed=80

Observe:
✓ Stays centered on line
✓ Smooth response
✓ No oscillation
✓ Recovers from nudges

If failed:
✗ Oscillates → Reduce Kp or increase Kd
✗ Drifts → Wrong offset or low Kp
✗ Jerky → High Kd or unbalanced motors
✗ Slow → Low Kp or low speed
```

### Turn Test
```
Test on a 90° turn

Command: delta=<value>, speed=80
Manually push bot through corner

Observe:
✓ Smooth arc entry
✓ Maintains line contact
✓ No slip or stall
✓ Exits cleanly

If failed:
✗ Too slow → Increase delta
✗ Too fast/overshoots → Decrease delta
✗ Drifts out → Bad offset or offset needed
```

### Distance Accuracy Test
```
Commands:
dist_reset
dist_calibrate
dist_start
motor_test_forward    # Let run 2 seconds
dist_get              # Check distance

Repeat with known distances:
1 meter, 2 meters, 5 meters

Compare reported vs actual
Adjust dist_unit until accurate
```

### Full Dry Run Test
```
Set up small test maze
Command: Button 1 pressed (or code simulation)
Observe:
✓ Bot explores all paths
✓ Finds endpoint correctly
✓ Displays progress on OLED
✓ Stops at endpoint

Check serial:
[MAZE] Junctions found: <count>
[MAZE] Distance: <mm>
```

---

## 8. Troubleshooting Guide

| Problem | Diagnosis | Solution |
|---------|-----------|----------|
| Bot drifts to one side | Unbalanced motors | Use `offset=<value>` |
| Oscillates on straight | Kp too high | Reduce Kp by 3-5 |
| Slow response to error | Kp too low | Increase Kp by 3-5 |
| Jerky/twitchy | Kd too low | Increase Kd by 2-3 |
| Sluggish turns | Kd too high | Reduce Kd by 2-3 |
| Misses endpoints | Threshold too high | Increase sensor whiteness check |
| Enters wrong corners | Turn delta wrong | Adjust delta ±5 |
| Distance inaccurate | IMU drift or wrong unit | Recalibrate and adjust dist_unit |
| Stops at random spots | Battery low | Recharge, may need speed adjustment |

---

## 9. Configuration Persistence

Current system: **Values reset on power cycle**

To save configurations:

```cpp
// Future enhancement: EEPROM storage
#include <EEPROM.h>

void saveConfig() {
  EEPROM.write(0, (uint8_t)sensorConfig.kp);
  // ... save other values
}

void loadConfig() {
  sensorConfig.kp = EEPROM.read(0);
  // ... load other values
}
```

For now: **Send configuration commands after power-up**

Or: **Record optimal values and code them as defaults**

---

## 10. Performance Monitoring

Monitor these metrics for optimal performance:

**Dry Run:**
- Distance traveled (should be longer, explores)
- Time taken (depends on speed and maze size)
- Junctions found (should match actual maze)
- Smooth movement (check video)

**Final Run:**
- Distance traveled (should be shorter than dry run)
- Time taken (usually 30-60% of dry run)
- Path accuracy (should reach exact endpoint)
- Efficiency ratio (goal: 1.5x - 3x improvement)

**Quick Optimization Workflow:**
```
1. Run full test maze
2. Check efficiency ratio
3. If < 1.5x:
   → Improve line following (tune Kp/Kd)
   → Ensure all junctions detected (tune thresholds)
   → Check for motor slipping
4. If > 4x:
   → Check if maze detected fully
   → Verify pathfinding found shortest path
```

---

**Happy Tuning!** 🎯

*Version 1.0 - December 2025*
