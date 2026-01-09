# ESP32 Robot Arm - How to Use on PC

This guide explains how to launch, configure, and operate the ESP32 Robot Arm control software from your PC.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Building and Uploading](#building-and-uploading)
3. [Connecting to the Robot](#connecting-to-the-robot)
4. [Web Interface Overview](#web-interface-overview)
5. [IK Solver Selection](#ik-solver-selection)
6. [Motor Type Selection](#motor-type-selection)
7. [Simulation vs Real Motion](#simulation-vs-real-motion)
8. [Safety Features](#safety-features)
9. [Common Issues and Solutions](#common-issues-and-solutions)

---

## Prerequisites

### Software Requirements

1. **PlatformIO** - Install via VS Code extension or command line
2. **Web Browser** - Chrome, Firefox, or Edge (modern versions)
3. **Serial Monitor** - Built into PlatformIO or use PuTTY/Tera Term

### Hardware Requirements

1. ESP32 development board
2. PCA9685 PWM driver (for servo motors)
3. 6-axis robot arm with servos or DC motors
4. Power supply (appropriate for your motors)
5. USB cable for programming

---

## Building and Uploading

### Step 1: Clone/Open the Project

```bash
cd c:\Users\pb55g\OneDrive\Desktop\ESP32_RobotArm
```

### Step 2: Configure WiFi Credentials

Edit `src/main.cpp` to set your WiFi credentials:

```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

### Step 3: Build the Firmware

```bash
platformio run
```

Expected output: `SUCCESS` with no errors.

### Step 4: Upload Firmware

```bash
platformio run --target upload
```

### Step 5: Upload Web Interface Files

```bash
platformio run --target uploadfs
```

This uploads the web UI files from the `data/` folder to the ESP32's SPIFFS filesystem.

### Step 6: Open Serial Monitor

```bash
platformio device monitor
```

You will see the ESP32's IP address printed:

```
WiFi Connected!
IP Address: 192.168.1.xxx
Web server started
```

---

## Connecting to the Robot

1. Ensure your PC is on the same WiFi network as the ESP32
2. Open a web browser
3. Navigate to the IP address shown in the serial monitor (e.g., `http://192.168.1.xxx`)
4. The web interface should load with the industrial 3-panel layout

### Connection Status

The status indicator in the toolbar shows:
- 🟢 **Green** - Connected to ESP32
- 🔴 **Red** - Disconnected (retrying automatically)

---

## Web Interface Overview

The interface has 6 main tabs in the right panel:

### 1. Motion Tab
- **Jog Joint** - Move individual joints with +/- buttons
- **Speed/Acceleration** - Adjust motion parameters
- **Cartesian Target** - Enter X, Y, Z coordinates for IK-based movement

### 2. IK Tab (Inverse Kinematics)
- **Solver Selection** - Choose from 4 IK solver types
- **Preference** - Select Speed, Accuracy, or Stability
- **Live Preview** - See reachability before executing

### 3. Motors Tab
- **Global Motor Type** - Switch between Servo and DC modes
- **Per-Axis Config** - Configure gear ratios, pulse widths, PID gains

### 4. Mode Tab
- **Mode 1 (Open Loop)** - Time-based motion, no encoder feedback
- **Mode 2 (Closed Loop)** - IK + encoder verification

### 5. Safety Tab
- **Status Display** - Homed, Motion Allowed, Faults, Alignment
- **E-STOP** - Emergency stop button
- **Clear Fault** - Reset after fault condition

### 6. Diag Tab (Diagnostics)
- **Alignment Status** - Per-joint error display
- **Error Charts** - Real-time error history
- **Fault History** - Log of past faults

---

## IK Solver Selection

The system provides 4 IK solver types:

| Solver | Best For | Speed | Accuracy |
|--------|----------|-------|----------|
| **Analytical** | Simple movements | ⭐⭐⭐ Fastest | ⭐⭐ Good |
| **Numerical** | Complex poses | ⭐ Slowest | ⭐⭐⭐ Best |
| **Servo-Friendly** | PWM servos | ⭐⭐ Fast | ⭐⭐ Good |
| **DC-Friendly** | DC + encoders | ⭐⭐ Fast | ⭐⭐ Good |

### Changing the Solver

1. Go to the **IK Tab**
2. Select solver from dropdown
3. Choose your preference (Speed/Accuracy/Stability)
4. Preview results before execution

### IK Warnings

- 🟡 **Yellow border** - Near singularity, may cause unexpected motion
- 🔴 **Red border** - Target unreachable, motion blocked

---

## Motor Type Selection

### Servo Mode (Default)
- Uses PCA9685 PWM driver
- Open-loop position control
- Best for hobby-grade servos (MG996R, etc.)

### DC Mode
- Uses encoder feedback
- Closed-loop PID control
- Requires proper wiring of motor driver and encoders

### Switching Motor Types

> ⚠️ **WARNING**: Motor type can only be changed when motion is stopped.

1. Go to **Motors Tab**
2. Click the desired motor type button
3. System will verify motion is stopped before switching
4. Configure per-axis settings as needed

---

## Simulation vs Real Motion

### Local Simulation (3D View)

The center panel shows a 3D visualization of the robot:
- **Blue robot** - Current/simulated position
- **Ghost robot** - IK preview position (when enabled)

**Simulation does NOT move the real robot!** Use it to:
- Preview motion paths
- Check reachability
- Verify IK solver behavior

### Real Robot Motion

To move the actual robot:
1. Ensure connection status is 🟢 Green
2. Use the **Motion Tab** controls
3. Click "Move Linear" or jog buttons
4. Robot will execute the motion

> ⚠️ **SAFETY**: Always verify simulation before executing on real hardware!

---

## Safety Features

### E-Stop Button

The physical E-Stop button (GPIO 4) or software button will:
1. Immediately stop all motors
2. Disable PWM output
3. Set fault code 0x20

Press **Clear Fault** to resume after addressing the issue.

### Alignment Checking

The system continuously monitors:
- **IK vs Encoder** - Commanded position vs actual position
- **FK vs TCP** - Forward kinematics vs expected tool position
- **Motor Response** - Command vs motor behavior

| Error Level | Threshold | Action |
|-------------|-----------|--------|
| Normal | < 2° | Full speed |
| Warning | 2° - 5° | Speed reduced 50% |
| Critical | > 5° | Motion stopped |

### Joint Limits

All IK solvers validate joint limits before returning a solution:
- **Soft limits** - Trigger warnings, allow motion
- **Hard limits** - Block motion completely

---

## Simulator Mode (New!)

The system now includes a full **Digital Twin Simulator** mode.

### Mode Toggle

In the header bar, you'll see a toggle switch:
- **HARDWARE** (Orange) - Commands sent to real motors
- **SIMULATOR** (Blue) - Commands run locally only

### Switching Modes

**To enter Simulator Mode:**
1. Ensure robot is NOT moving
2. Click the SIMULATOR button or toggle
3. Motors will be disabled automatically

**To enter Hardware Mode:**
1. Ensure no E-Stop is active
2. Clear any alignment faults
3. Click HARDWARE button
4. System will ARM the motors

> ⚠️ **WARNING**: In HARDWARE mode, all motion commands will move the real robot!

### Simulator Features

When in Simulator Mode:
- **3D Robot Visualization** - See robot in browser
- **IK/FK Computation** - Runs in Web Worker for performance
- **Joint Limits** - Still enforced
- **Safety Logic** - Alignment checks still run
- **No Motor Output** - PWM is disabled

### Connections Graph

Navigate to the **Connections** tab to see:
- Live system architecture diagram
- Node status (Active/Warning/Fault)
- Data flow animation
- Click any node for details

### Safety Guarantees

| Guarantee | Enforcement |
|-----------|-------------|
| Simulator cannot move motors | Firmware blocks PWM in sim mode |
| Hardware mode requires safety check | E-Stop, faults, alignment checked |
| Disconnect enters safe mode | 500ms grace, then auto-SIMULATOR |
| UI cannot bypass firmware | All commands validated server-side |

### Performance Notes

The React-based simulator uses:
- **Web Workers** for IK/FK (60fps rendering)
- **WebSocket** for real-time updates (10Hz)
- **Code-splitting** to reduce bundle size

If performance is slow:
1. Close other browser tabs
2. Reduce window size
3. Disable TCP path tracing

---

## Common Issues and Solutions

### Problem: Cannot connect to web interface

**Solutions:**
1. Verify ESP32 is powered and running (check serial monitor)
2. Confirm PC is on same WiFi network
3. Try accessing via IP address directly (not hostname)
4. Check firewall settings on PC

### Problem: IK says "Target Unreachable"

**Solutions:**
1. Verify target is within robot's workspace
2. Check that X/Y/Z values are reasonable (in mm)
3. Try a different IK solver (Numerical is most flexible)
4. Adjust joint limits if too restrictive

### Problem: Robot moves but not to correct position

**Solutions:**
1. Verify servo pulse width calibration in Motors tab
2. Check gear ratio settings
3. Ensure encoders are properly connected (for DC mode)
4. Calibrate/home the robot

### Problem: Alignment warnings constantly appearing

**Solutions:**
1. Reduce motion speed
2. Check mechanical backlash in joints
3. Verify encoder connections
4. Adjust alignment thresholds in diagnostics

### Problem: Motor type change fails

**Solutions:**
1. Ensure robot is completely stopped
2. Wait for any in-progress motion to complete
3. Check serial monitor for error messages

---

## Quick Reference

### Keyboard Shortcuts (when 3D view is focused)

| Key | Action |
|-----|--------|
| `R` | Reset camera view |
| `Space` | Stop all motion |
| `1-6` | Select joint for jog |

### API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/status` | GET | Get robot state |
| `/api/motion/joint` | POST | Move single joint |
| `/api/motion/cartesian` | POST | Move to Cartesian target |
| `/api/motion/stop` | POST | Stop motion |
| `/api/safety/estop` | POST | Emergency stop |
| `/api/ik/config` | GET | Get IK configuration |
| `/api/ik/preview` | POST | Preview IK result |
| `/api/motors/config` | GET | Get motor configuration |
| `/api/alignment/status` | GET | Get alignment status |
| `/api/mode/simulator` | POST | Switch Hardware/Simulator |
| `/api/mode/status` | GET | Get current mode status |
| `/api/system/graph` | GET | Get system graph node states |
| `/ws` | WebSocket | Real-time state updates |

---

## Contact & Support

For issues with this software, check:
1. Serial monitor output for error messages
2. Web browser console (F12) for JavaScript errors
3. Verify hardware connections match pin definitions in code

---

*ESP32 Robot Arm Control System - Industrial IK & Motor Control Edition*
