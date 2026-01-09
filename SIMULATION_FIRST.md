# ESP32 Robot Arm - Simulation-First Guide

This system is designed to run **completely in simulation mode** without any physical hardware.

---

## Quick Start (No Hardware Required!)

```bash
# 1. Install dependencies
cd web-ui
npm install

# 2. Start development server
npm run dev

# 3. Open browser to http://localhost:5173
```

That's it! The robot simulator runs entirely in your browser.

---

## What Works Without Hardware

| Feature | Status |
|---------|--------|
| 3D Robot Visualization | ✅ Works |
| Joint Jog Controls | ✅ Works |
| Cartesian Movement | ✅ Works |
| IK/FK Computation | ✅ Works |
| Coordinate Frames | ✅ Works |
| TCP Path Tracing | ✅ Works |
| Safety Logic (limits) | ✅ Works |
| Singularity Warnings | ✅ Works |
| Alignment Checking | ✅ Works |
| Connections Graph | ✅ Works |
| E-Stop Simulation | ✅ Works |

---

## Architecture: Simulation vs Hardware

```
┌─────────────────────────────────────────────────────────────┐
│                    SIMULATION MODE (DEFAULT)                 │
│                                                              │
│  Browser                                                     │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  React UI                                              │  │
│  │  ├── RobotSimulator.jsx (3D view)                     │  │
│  │  ├── MotionPanel.jsx (controls)                       │  │
│  │  ├── ConnectionsGraph.jsx                             │  │
│  │  └── useSimulatedWebSocket.js ────┐                   │  │
│  │                                    │                   │  │
│  │  ┌─────────────────────────────────▼─────────────────┐│  │
│  │  │  SimulatedRobot.js                                ││  │
│  │  │  ├── Simulated Motor Controllers                  ││  │
│  │  │  ├── Simulated Encoders                           ││  │
│  │  │  ├── IK/FK Computation                            ││  │
│  │  │  ├── Safety Logic                                 ││  │
│  │  │  └── Alignment Checking                           ││  │
│  │  └───────────────────────────────────────────────────┘│  │
│  └───────────────────────────────────────────────────────┘  │
│                                                              │
│  NO ESP32 REQUIRED                                           │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                 HARDWARE MODE (OPTIONAL)                     │
│                                                              │
│  Requires:                                                   │
│  - ESP32 dev board                                           │
│  - PCA9685 PWM driver                                        │
│  - Motors (servo or DC)                                      │
│  - Encoders (for closed loop)                                │
│                                                              │
│  To enable:                                                  │
│  1. Flash firmware: platformio run --target upload           │
│  2. Upload UI: platformio run --target uploadfs              │
│  3. Connect to ESP32's IP address                            │
│  4. Switch from SIMULATOR to HARDWARE mode in UI             │
└─────────────────────────────────────────────────────────────┘
```

---

## Mode Separation (Safety Gate)

| Rule | Enforcement |
|------|-------------|
| Simulator mode by default | `useSimulatedWebSocket` is used, not real WebSocket |
| No GPIO calls in simulation | `SimulatedRobot.js` has no hardware imports |
| Hardware mode requires user action | Must explicitly toggle to HARDWARE |
| Mode flag is firmware-authoritative | When hardware connected, ESP32 controls the flag |

---

## Files Structure

### Simulation-Only Files (Browser)
```
web-ui/src/
├── simulation/
│   └── SimulatedRobot.js      # Motor/encoder/IK simulation
├── hooks/
│   ├── useSimulatedWebSocket.js  # Local state updates
│   └── useWebSocket.js           # Real ESP32 connection (unused in sim)
├── components/
│   ├── RobotSimulator.jsx        # 3D visualization
│   ├── MotionPanel.jsx           # Motion controls
│   ├── DiagnosticsPanel.jsx      # Status display
│   └── ConnectionsGraph.jsx      # System diagram
└── stores/
    └── robotStore.js             # Zustand state
```

### Hardware Files (ESP32)
```
src/
├── RobotCore.cpp          # Main control loop
├── RobotUI.cpp            # Web server + WebSocket
├── motors/                # Real motor controllers
├── ik/                    # IK solvers
└── SafetyManager.cpp      # Hardware safety
```

---

## Simulator Features

### Joint Control
- Click on joint in dropdown
- Use jog buttons for ±5° increments
- Adjustable speed (5-120 deg/s)

### Cartesian Movement
- Enter X, Y, Z coordinates
- Click "Move Linear"
- IK computed automatically
- Target validation (reachability, limits)

### Safety Simulation
- Joint limits enforced
- Singularity detection
- Alignment monitoring (simulated noise)
- E-Stop simulation

### 3D Visualization
- Rotate: Left mouse drag
- Zoom: Scroll wheel
- Pan: Right mouse drag
- TCP path shown in red

---

## What Requires Hardware

| Feature | Requires Hardware |
|---------|-------------------|
| Real motor movement | Yes |
| Encoder feedback | Yes |
| Physical E-Stop button | Yes |
| WiFi connection | Yes |
| PWM output | Yes |
| Mode 2 (Closed Loop) | Yes (needs encoders) |

---

## Development Workflow

### Simulation Only
```bash
cd web-ui
npm run dev
# Open http://localhost:5173
# All features work without hardware
```

### Build for Production
```bash
cd web-ui
npm run build
# Outputs to ../data/ folder
```

### With Hardware (Later)
```bash
# Configure WiFi in src/main.cpp
platformio run --target upload
platformio run --target uploadfs
# Access via ESP32's IP
```

---

## Free Libraries Used

| Library | License | Purpose |
|---------|---------|---------|
| React 18 | MIT | UI framework |
| React Three Fiber | MIT | 3D graphics |
| React Flow | MIT | Node diagram |
| Zustand | MIT | State management |
| Vite | MIT | Build tool |
| gl-matrix | MIT | Matrix math |

---

## Safety Guarantees

1. **Simulation cannot drive real motors** - No hardware code runs in simulation
2. **Same validation logic** - IK, limits, safety checked identically
3. **Mode switching gated** - Cannot switch to hardware during motion
4. **All state visible** - Diagnostics shows full system state
