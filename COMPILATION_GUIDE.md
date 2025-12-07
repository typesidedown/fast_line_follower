# Maze Solver - Compilation & Upload Guide

## Quick Start Compilation

### Prerequisites
- PlatformIO IDE or CLI installed
- Teensy board connected to computer
- All source files in `src/` directory

### Compilation Steps

#### Using PlatformIO IDE
1. Open the project in VS Code with PlatformIO
2. Click **Build** (checkmark icon) in PlatformIO sidebar
3. Wait for compilation to complete
4. Check terminal for errors

#### Using PlatformIO CLI
```bash
# Navigate to project directory
cd "c:\Users\ISHAN\OneDrive\Documents\PlatformIO\Projects\Line_follower"

# Build the project
pio run

# Build and upload
pio run -t upload
```

### Upload to Teensy

#### Using PlatformIO IDE
1. Connect Teensy 4.0 via USB
2. Click **Upload** (arrow icon) in PlatformIO sidebar
3. Wait for upload to complete (takes 10-30 seconds)
4. Teensy will reboot automatically

#### Using PlatformIO CLI
```bash
pio run -t upload
```

### Verify Upload

Open Serial Monitor:
1. PlatformIO sidebar → Serial Monitor
2. Set baud rate to **115200**
3. Power cycle the bot
4. Should see initialization messages

Expected output:
```
[BUTTONS] Initialized
[MAZE] System initialized
[configuration messages...]
```

---

## Troubleshooting Compilation

### Error: "fatal error: maze.h: No such file or directory"

**Cause**: New header files not recognized

**Solution**:
```
1. Verify files are in: src/maze.h, src/maze.cpp
2. Clean build: pio run --target clean
3. Rebuild: pio run
```

### Error: "undefined reference to `buildMazeGraph()'"

**Cause**: pathfinder.cpp not compiling

**Solution**:
```
1. Check pathfinder.h has all function declarations
2. Verify pathfinder.cpp has implementations
3. Check no typos in function names
4. Rebuild: pio run
```

### Error: "error: 'std::vector' was not declared"

**Cause**: Missing `#include <vector>`

**Solution**:
1. Open maze.h
2. Add: `#include <vector>` near top
3. Rebuild

### Error: "insufficient memory"

**Cause**: Program too large for Teensy

**Solution**:
```
1. Increase stack size in platformio.ini:
   build_flags = -Wl,--gc-sections -ffunction-sections
   
2. Remove debug symbols:
   build_type = release
   
3. Clean and rebuild
```

### Error: "COM port not found"

**Cause**: Teensy not detected

**Solution**:
```
1. Check USB cable connection
2. Press reset button on Teensy
3. Wait 2 seconds for device to appear
4. Try upload again
5. If still fails: Install Teensy drivers
```

---

## Troubleshooting Upload

### Upload Hangs or Fails

**Solution 1: Reset Teensy**
```
Press physical reset button on Teensy board
Try upload immediately after (within 10 seconds)
```

**Solution 2: Force Upload Mode**
```
PlatformIO: Try upload twice quickly
Or press reset twice rapidly on Teensy
Then upload
```

**Solution 3: Check Baud Rate**
```
platformio.ini should have:
upload_speed = 2000000
```

### "waiting for serial connection"

**Solution**:
```
1. Reset Teensy (physical button)
2. Upload starts automatically
3. If not, try upload again
```

### Upload Successful But Bot Doesn't Respond

**Solution**:
```
1. Open Serial Monitor
2. Press reset button on Teensy
3. Should see initialization
4. Type 'help' and press Enter
5. Should see command list
```

---

## Project File Structure

```
Line_follower/
├── src/
│   ├── main.cpp           ← Main loop
│   ├── maze.h / maze.cpp  ← Maze solving (NEW)
│   ├── buttons.h / buttons.cpp  ← Button control (NEW)
│   ├── pathfinder.h / pathfinder.cpp  ← Pathfinding (NEW)
│   ├── movement.h / movement.cpp
│   ├── sensors.h / sensors.cpp
│   ├── imu.h / imu.cpp
│   ├── oled.h / oled.cpp  ← Updated
│   ├── config.h / config.cpp
│   └── [other files]
├── include/
│   └── README
├── lib/
│   └── README
├── platformio.ini         ← Build configuration
├── MAZE_SOLVER_DOCUMENTATION.md  (NEW)
├── QUICK_START.md         (NEW)
├── TECHNICAL_GUIDE.md     (NEW)
├── TUNING_GUIDE.md        (NEW)
├── IMPLEMENTATION_SUMMARY.md  (NEW)
└── [other config files]
```

---

## Build Configuration Check

### platformio.ini

Verify these settings exist:

```ini
[env:teensy40]
platform = teensy
board = teensy40
framework = arduino
upload_speed = 2000000
monitor_speed = 115200

; Optional: Add this for larger projects
build_flags = 
    -Wl,--gc-sections
    -ffunction-sections
    -Werror=all
```

### Adding Build Flags

If compilation fails with memory issues:

```ini
build_flags = 
    -DTEENSY40
    -O2
    -Wl,--gc-sections
    -ffunction-sections
```

---

## Clean Build from Scratch

If you encounter persistent issues:

```bash
# Navigate to project
cd "c:\Users\ISHAN\OneDrive\Documents\PlatformIO\Projects\Line_follower"

# Remove build artifacts
pio run --target clean

# Rebuild completely
pio run

# Upload
pio run -t upload
```

---

## Verifying Compilation Success

After successful compilation, check:

### Serial Monitor Test
```
1. Open Serial Monitor (115200 baud)
2. Power cycle bot
3. Should see:
   - Button initialization
   - Maze initialization
   - Configuration output
   
4. Type: help
   Expected: Full command list including new commands
```

### Button Test
```
1. Press Button 1 (Pin 2)
   Should see: "[BUTTONS] Button 1 pressed"
   
2. Press Button 2 (Pin 3)
   Should see: "[BUTTONS] Button 2 pressed"
```

### OLED Display Test
```
1. Bot should show "MAZE SOLVER" menu
2. Buttons should be labeled on OLED
3. Display should update when buttons pressed
```

### Movement Test
```
Command: motor_test_forward
Response: Bot moves forward 2 seconds
          "Motor test forward done"
```

---

## Common Compilation Warnings (Safe to Ignore)

```
warning: comparison of integers of different signedness
→ Integer sign mismatch in loop conditions
→ Safe to ignore for this project

warning: unused variable
→ Debug code or future features
→ Safe to ignore

warning: deprecated conversion between string literal and char*
→ Arduino compatibility
→ Safe to ignore
```

---

## Development Workflow

### Adding New Features

1. Create new .h and .cpp files in `src/`
2. Add declarations to .h file
3. Add implementations to .cpp file
4. Include in main.cpp if needed
5. Run `pio run` to compile
6. Fix any errors
7. Upload to Teensy
8. Test functionality

### Modifying Existing Code

1. Edit file in src/
2. Run `pio run` to check compilation
3. Fix any errors
4. When ready, run `pio run -t upload`
5. Test on hardware

### Debugging Compilation Errors

1. Read error message carefully
2. Note line number and file
3. Check for:
   - Missing includes
   - Typos in function names
   - Mismatched braces
   - Missing semicolons
4. Fix and recompile
5. Repeat until successful

---

## Performance Monitoring

After upload, monitor performance:

### Check Loop Speed
```
Serial command: diag
Output shows: Loop count (should increment rapidly)
```

### Memory Usage
```
PlatformIO shows after compilation:
Used: XX KB out of 1024 KB available
Ensure: Used < 800 KB (leave 20% buffer)
```

### Execution Time
```
Measure dry run time:
Start: Record time
Bot exploring
End: When endpoint detected
Calculate: Total duration
```

---

## Backup Before Modifications

Always backup before making changes:

```bash
# Create backup
Copy entire project folder
Rename to "Line_follower_backup_DATE"
Store in safe location
```

### Version Control (Git)

If using Git:
```bash
git init                    # Initialize repo
git add .                   # Stage all files
git commit -m "Maze solver implementation"
git branch -m main          # Main branch
```

---

## Platform-Specific Notes

### Windows (Your System)

```
Serial ports typically: COM3, COM4, COM5, etc.
Path separators: Use backslashes or forward slashes
Teensy drivers: Download from pjrc.com
```

### macOS/Linux

```
Serial ports typically: /dev/tty.usbmodem*
May need: sudo for upload
```

---

## Next Steps After Successful Upload

1. **Test Dry Run**
   ```
   Press Button 1
   Watch bot explore maze
   Should reach endpoint
   ```

2. **Test Final Run**
   ```
   Press Button 2
   Watch bot follow shortest path
   Should reach endpoint faster
   ```

3. **Tune Configuration**
   - Adjust PD gains for smoother movement
   - Calibrate IR thresholds
   - Optimize speed and turn delta

4. **Test on Various Mazes**
   - Small maze (4x4 grid)
   - Medium maze (8x8 grid)
   - Complex maze with many turns

5. **Measure Performance**
   - Record efficiency ratio
   - Check for improvements
   - Identify bottlenecks

---

## Support & Troubleshooting

### If Compilation Fails

1. Check all 4 new files exist:
   - src/maze.h
   - src/maze.cpp
   - src/buttons.h
   - src/buttons.cpp
   - src/pathfinder.h
   - src/pathfinder.cpp

2. Verify no syntax errors:
   - All braces matched
   - All semicolons present
   - No typos

3. Check includes:
   - maze.h includes necessary headers
   - main.cpp includes all needed headers

### If Upload Fails

1. Check USB connection
2. Reset Teensy board
3. Try PlatformIO CLI instead of IDE
4. Check monitor_speed = 115200

### If Bot Doesn't Respond

1. Check serial monitor
2. Verify baud rate is 115200
3. Type 'help' and press Enter
4. Should see command list

---

## Documentation Files

Read these for detailed information:

1. **MAZE_SOLVER_DOCUMENTATION.md** - Complete system overview
2. **QUICK_START.md** - Quick operation guide
3. **TECHNICAL_GUIDE.md** - Implementation details
4. **TUNING_GUIDE.md** - Configuration and optimization
5. **IMPLEMENTATION_SUMMARY.md** - Summary of changes

---

## Compilation Checklist

Before declaring success:

```
Pre-Compilation:
  □ All 6 new .h/.cpp files in src/
  □ main.cpp includes maze.h and buttons.h
  □ platformio.ini configured correctly
  □ No duplicate file names

Compilation:
  □ pio run completes without errors
  □ Warning messages only (not errors)
  □ Binary size shown (should be < 800 KB)

Upload:
  □ pio run -t upload succeeds
  □ Teensy resets and reboots
  □ No timeout errors

Verification:
  □ Serial monitor shows messages
  □ OLED displays maze menu
  □ Buttons detected and responsive
  □ Motor test works
```

---

## Quick Reference

```bash
# Full workflow
pio run --target clean      # Clean build
pio run                     # Compile
pio run -t upload           # Upload
pio device monitor          # Serial monitor (Ctrl+C to exit)

# Just compile
pio run

# Just upload
pio run -t upload

# Open serial monitor
pio device monitor -b 115200
```

---

**Happy Compiling!** 🔧

*Last Updated: December 2025*
