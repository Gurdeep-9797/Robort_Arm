# Industrial Robot Simulator UI - Complete Documentation

**APPEND TO EXISTING DOCUMENTATION - DO NOT REPLACE**

---

## Overview

This UI transforms the ESP32 robot control interface into a professional industrial-grade simulator, visually and functionally similar to:
- RoboDK
- KUKA Sim
- ABB RobotStudio
- Fanuc ROBOGUIDE
  
### Key Principle
**Simulation runs on laptop/browser power. ESP32 remains execution authority.**

---

## Architecture

### Three-Layer Design

```
┌─────────────────────────────────────────┐
│  PRESENTATION LAYER (Browser)           │
│  - Three.js 3D rendering                │
│  - Real-time visualization              │
│  - STL model loading                    │
│  - Trajectory preview                   │
└──────────────┬──────────────────────────┘
               │ REST API + WebSocket
┌──────────────▼──────────────────────────┐
│  UI SERVER LAYER (ESP32)                │
│  - ui_server.cpp                        │
│  - ui_state_manager.cpp                 │
│  - JSON processing                      │
└──────────────┬──────────────────────────┘
               │ Function Calls
┌──────────────▼──────────────────────────┐
│  EXECUTION LAYER (ESP32 Core)           │
│  - motion_planner.cpp                   │
│  - control_loop.cpp                     │
│  - safety_manager.cpp                   │
└─────────────────────────────────────────┘
```

---

## UI Layout Structure

### 1. Top Toolbar (48px height)
**Purpose:** Quick access to simulation controls

**Buttons:**
- 📁 Load STL - Upload robot models
- ▶ Play - Start simulation
- ⏸ Pause - Pause motion
- ⏹ Stop - Stop and reset
- 🔄 Reset View - Reset camera
- 📐 Frames - Toggle coordinate axes
- 💾 Export - Download session data

**Status Indicator:**
- Green dot = Connected to ESP32
- Red dot = Disconnected
- Pulsing = Attempting connection

---

### 2. Left Panel - Project Tree (260px width)

**Structure:**
```
🤖 Robot Arm (expandable)
  ├─ ⚙ Joint 0: 0.0°
  ├─ ⚙ Joint 1: 45.2°
  ├─ ⚙ Joint 2: -30.1°
  ├─ ⚙ Joint 3: 0.0°
  ├─ ⚙ Joint 4: 90.5°
  ├─ ⚙ Joint 5: 0.0°
  └─ 🎯 TCP (Tool Center Point)

📐 Reference Frames (expandable)
  ├─ Base Frame
  └─ Tool Frame

✈ User Planes (expandable)
  └─ (User-defined planes)

📍 Taught Points (expandable)
  └─ (Saved positions)

📦 STL Models (expandable)
  └─ (Loaded models)
```

**Behavior:**
- Click header to expand/collapse
- Joint values update in real-time
- Color-coded: teal = active values

---

### 3. Center Panel - 3D Simulation View (flex: 1)

**Features:**
- **Three.js WebGL Renderer** - Hardware-accelerated 3D
- **OrbitControls** - Pan, zoom, rotate with mouse
- **Grid Floor** - 1000x1000mm grid, 20 divisions
- **Shadow Mapping** - Realistic lighting
- **Coordinate Axes** - World origin indicator (200mm)
- **Joint Axes** - Per-joint coordinate frames (80mm, toggleable)

**Default Camera:**
- Position: (800, 600, 800)
- Looking at: (0, 0, 0)
- FOV: 50°

**Lighting:**
- Ambient Light: 60% intensity
- Directional Light: 80% intensity, shadows enabled
- Hemisphere Light: 40% intensity

**Overlay Info (Top-Left):**
- Current mode (MODE 1 / MODE 2)
- Status (Ready / Moving / Fault)
- Deviation panel (when Mode 2 active)

---

### 4. Right Panel - Control & Parameters (320px width)

**Tabbed Interface:**

#### Tab 1: Motion Control
- **Jog Joint:** Select joint 0-5, +/- buttons
- **Speed Slider:** 5-120 deg/s
- **Acceleration Slider:** 50-500 deg/s²
- **Cartesian Target:** X, Y, Z inputs + Move button
- **STOP ALL:** Emergency stop button

#### Tab 2: Mode Selection
- **Mode 1 Card:** Open Loop, Time-based
- **Mode 2 Card:** Closed Loop, IK + Encoders
- **Current Mode Display**

#### Tab 3: Configuration
- **Edit Joint Limits:** Select joint, set min/max
- **Save to ESP32:** Persist config to NVS

#### Tab 4: Safety
- **Status Display:**
  - Homed: Yes/No
  - Motion Allowed: Yes/No
  - Fault Code: 0x00000000
- **Emergency Stop Button**
- **Clear Fault Button**

---

## 3D Robot Model

### Default Geometric Model

**When no STL loaded, uses procedural geometry:**

1. **Base:** Cylinder (60mm top, 40mm bottom, 100mm height)
2. **Joints 0-5:** Spheres (30mm radius) + Cylinder links
3. **Links:** 15mm radius cylinders
4. **TCP:** Red cone (20mm radius, 60mm height)

**Materials:**
- Base/Links: Gray metallic (#505050, #707070)
- Joints: Teal (#4ec9b0)
- TCP: Red with emissive (#ff4444)

### STL Model Loading

**Upload Process:**
1. Click "Load STL" in toolbar
2. Select joint assignment:
   - Base
   - Joint 0-5
   - Tool/TCP
3. Upload .STL file (ASCII or binary)
4. Model automatically:
   - Centers geometry at joint origin
   - Applies metallic material
   - Enables shadows
   - Attaches to joint hierarchy

**STL Requirements:**
- Format: STL (ASCII or Binary)
- Units: Millimeters
- Origin: Should be at joint rotation axis
- No size limit (browser memory permitting)

**Joint Hierarchy:**
```
Scene
└─ Base
   └─ Joint 0
      └─ Joint 1
         └─ Joint 2
            └─ Joint 3
               └─ Joint 4
                  └─ Joint 5
                     └─ TCP
```

**Important:** STL files are NEVER uploaded to ESP32. They exist only in browser memory for visualization.

---

## Digital Twin Synchronization

### Motion Flow

**Step 1: User Command**
```
User clicks "Jog Joint 1 +"
  ↓
app.js updates simulator.simulatedPositions[1] += 5°
  ↓
simulator.updateRobotPose() renders new pose
  ↓
Preview motion visually
```

**Step 2: Execute on ESP32**
```
app.js → POST /api/motion/joint
  ↓
ui_server.cpp → handleMoveJoint()
  ↓
ui_state_manager.cpp → validates
  ↓
MotionPlanner::getInstance().moveJoint()
  ↓
Real servos move
```

**Step 3: Feedback Loop**
```
ESP32 encoders read actual position
  ↓
control_loop updates state
  ↓
ui_state_manager caches state
  ↓
GET /api/telemetry (polled at 500ms)
  ↓
app.js receives real positions
  ↓
simulator.updateRobotPose(realPositions, true)
  ↓
Update deviation display
```

### Deviation Display

**When Mode 2 Active:**
- Shows per-joint deviation (simulated vs real)
- Color coded:
  - Green (#4ec9b0): < 2° deviation
  - Red (#e74c3c): > 2° deviation
- Displays average deviation across all joints

**Example:**
```
J0: 0.15°  (green)
J1: 2.45°  (red)
J2: 0.82°  (green)
J3: 0.05°  (green)
J4: 1.23°  (green)
J5: 0.34°  (green)
────────────────
Avg: 0.84°
```

---

## STL Assembly Workflow

### Complete Assembly Example

**Goal:** Load complete robot CAD model

**Files Needed:**
- `base.stl` - Robot base
- `joint1.stl` - Shoulder link
- `joint2.stl` - Upper arm link
- `joint3.stl` - Forearm link
- `joint4.stl` - Wrist roll link
- `joint5.stl` - Wrist pitch link
- `joint6.stl` - Wrist yaw link
- `tool.stl` - End effector

**Step-by-Step:**

1. **Load Base**
   - Click "Load STL"
   - Select "Base"
   - Upload `base.stl`
   - Verify centered at origin

2. **Load Joint 0 (Shoulder)**
   - Click "Load STL"
   - Select "Joint 0"
   - Upload `joint1.stl`
   - Should attach to base

3. **Repeat for Joints 1-5**
   - Each joint attaches to previous
   - Check motion by jogging

4. **Load Tool**
   - Select "Tool/TCP"
   - Upload `tool.stl`
   - Attaches to Joint 5 end

**Verification:**
- Jog each joint individually
- Check visual motion matches expected
- Verify no interpenetration
- Check coordinate frames align

---

## REST API Summary (UI-Specific)

### Existing Endpoints Used

| Endpoint | Method | Purpose | Source File |
|----------|--------|---------|-------------|
| `/api/status` | GET | Full robot state | ui_server.cpp |
| `/api/joints` | GET | Joint angles only | ui_server.cpp |
| `/api/telemetry` | GET | Real-time data (500ms) | ui_server.cpp |
| `/api/motion/joint` | POST | Move single joint | ui_server.cpp |
| `/api/motion/cartesian` | POST | Cartesian move | ui_server.cpp |
| `/api/motion/stop` | POST | Stop motion | ui_server.cpp |
| `/api/safety/estop` | POST | Emergency stop | ui_server.cpp |
| `/api/safety/clear` | POST | Clear faults | ui_server.cpp |
| `/api/mode` | POST | Switch mode | ui_server.cpp |
| `/api/session/export` | GET | Export accuracy data | ui_server.cpp |

### WebSocket Events

**Connection:** `ws://<ESP32-IP>/ws`

**Message Types:**
```json
{
  "type": "state_update",
  "is_moving": false,
  "joints": [0.0, 45.0, -30.0, 0.0, 90.0, 0.0]
}
```

**Frequency:** 100ms (10 Hz)

---

## Key JavaScript Classes

### Class: RobotSimulator

**File:** `simulator.js`

**Purpose:** 3D visualization and animation

**Properties:**
```javascript
- scene: THREE.Scene
- camera: THREE.PerspectiveCamera
- renderer: THREE.WebGLRenderer
- controls: THREE.OrbitControls
- robot.base: THREE.Mesh
- robot.joints[]: THREE.Group[]
- robot.tcp: THREE.Mesh
- simulatedPositions[]: float[6]
- realPositions[]: float[6]
- stlModels{}: Object
- dhParameters[]: Object[6]
```

**Key Methods:**
```javascript
init() - Initialize Three.js scene
setupLights() - Configure lighting
createRobot() - Build default geometry
updateRobotPose(angles, isReal) - Update joint positions
loadSTLModel(file, jointId) - Load STL into scene
toggleFrames() - Show/hide coordinate axes
resetCamera() - Reset view to default
highlightJoint(index) - Highlight active joint
updateDeviationDisplay() - Show sim vs real
animate() - Render loop (60 FPS)
```

### UI Helper Functions

**File:** `simulator.js` (global functions)

```javascript
toggleTreeNode(header) - Expand/collapse tree
switchTab(tabName) - Change right panel tab
loadSTLDialog() - Open STL upload dialog
uploadSTL() - Process STL file
updateTreeJointValues() - Sync tree with values
jogJoint(direction) - Jog selected joint
moveCartesianFromPanel() - Execute cart move
updateLimits() - Update config
saveConfigToESP32() - Persist config
```

---

## Simulation vs Reality Debug Strategy

### Problem: "Why doesn't the robot match my simulation?"

### Debug Workflow

**1. Check Connection**
```
Toolbar status indicator:
- Green = Connected ✓
- Red = Disconnected ✗
- Pulsing = Connecting...
```

**2. Verify Mode**
```
Mode 1: Deviations expected (no feedback)
Mode 2: Should match closely
```

**3. Monitor Telemetry**
```javascript
// Open browser console
setInterval(() => {
  fetch('/api/telemetry').then(r => r.json()).then(console.log);
}, 1000);
```

**4. Check Deviation Panel**
```
If deviation > 5°:
  → Check encoder calibration
  → Verify DH parameters
  → Check mechanical backlash
```

**5. Export Session Data**
```
Click "Export" button
Open downloaded JSON
Plot errors in spreadsheet
```

### Common Issues & Solutions

| Issue | Likely Cause | Solution |
|-------|--------------|----------|
| Large deviation in Mode 1 | Normal (no feedback) | Switch to Mode 2 |
| Deviation grows over time | Accumulating error | Re-home robot |
| Sudden deviation spike | Encoder slip | Check wiring |
| Constant offset | DH parameters wrong | Recalibrate DH |
| Visual model wrong | STL not centered | Re-export STL |

---

## Performance Optimization

### Browser-Side

**Three.js Rendering:**
- Target: 60 FPS (16.6ms per frame)
- Shadow map: 2048x2048 (adjustable)
- Antialiasing: Enabled (can disable for performance)

**Memory Usage:**
- Default robot: ~5 MB
- Per STL: ~1-50 MB (depends on mesh complexity)
- Recommended: < 500MB total

**Optimization Tips:**
- Decimate STL meshes in CAD software
- Use binary STL (smaller than ASCII)
- Limit shadow casting to essential objects
- Reduce grid size if needed

### ESP32-Side

**No changes needed** - ESP32 never receives STL data

**Network Traffic:**
- WebSocket: ~1 KB/s (state updates)
- REST API: On-demand only
- Telemetry polling: ~2 KB/s

---

## Session Export Format

**File:** `session_<timestamp>.json`

**Structure:**
```json
{
  "session_start": 1234567890,
  "session_end": 1234599999,
  "robot_config": {
    "dh_parameters": [...],
    "joint_limits": [...]
  },
  "stl_metadata": {
    "base": "base.stl",
    "joint0": "joint1.stl",
    ...
  },
  "motion_log": [
    {
      "timestamp": 1234567900,
      "command": "move_joint",
      "joint": 1,
      "target": 45.0,
      "actual": 44.8,
      "error": 0.2
    },
    ...
  ],
  "statistics": {
    "mean_error": 0.34,
    "max_deviation": 1.2,
    "rms_error": 0.41
  }
}
```

**Usage:**
- Import into Excel for plotting
- Use Python pandas for analysis
- Compare multiple sessions

---

## Browser Compatibility

### Tested Browsers:
- ✅ Chrome 90+ (Recommended)
- ✅ Firefox 88+
- ✅ Edge 90+
- ✅ Safari 14+ (some limitations)

### Required Features:
- WebGL 2.0
- ES6 JavaScript
- FileReader API (for STL loading)
- WebSocket support

### Mobile Support:
- Touch controls work for orbit
- Limited STL upload support
- Reduced performance on older devices

---

## Industrial Design Principles Applied

### 1. Predictability
- No surprising animations
- Clear button labels
- Consistent terminology (Joint, TCP, Frame)

### 2. Information Density
- Tree view shows live data
- Tabbed interface reduces clutter
- Overlay info doesn't block view

### 3. Professional Aesthetics
- Dark theme (reduces eye strain)
- Muted colors (not distracting)
- Flat icons (modern, clean)

### 4. Engineer-Oriented
- Technical terminology (DH, TCP, IK)
- Numeric precision (0.1° display)
- Explicit units (deg, mm, deg/s)

### 5. Safety First
- Emergency stop prominent (red)
- Fault display always visible
- Mode clearly indicated

---

## Extension Points

### Adding New Features

**1. Custom Tool Models**
```javascript
// In simulator.js
function attachCustomTool(geometry) {
  const tool = new THREE.Mesh(geometry, material);
  simulator.robot.tcp.add(tool);
}
```

**2. Trajectory Preview**
```javascript
// Draw path before execution
function previewTrajectory(waypoints) {
  const points = waypoints.map(wp => new THREE.Vector3(wp.x, wp.y, wp.z));
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  const line = new THREE.Line(geometry, lineMaterial);
  scene.add(line);
}
```

**3. Collision Detection**
```javascript
// Check if links intersect
function checkSelfCollision() {
  for (let i = 0; i < joints.length; i++) {
    for (let j = i + 2; j < joints.length; j++) {
      if (checkIntersection(joints[i], joints[j])) {
        return true; // Collision detected
      }
    }
  }
  return false;
}
```

**4. Virtual Buttons/Sensors**
```javascript
// Add interactive objects
const button = new THREE.Mesh(buttonGeometry, buttonMaterial);
button.onClick = () => {
  // Trigger action
};
```

---

## Troubleshooting

### Problem: 3D View Blank

**Check:**
1. Browser console for errors
2. WebGL support: Visit https://get.webgl.org/
3. Three.js library loaded: `console.log(THREE)`

**Solution:**
- Update browser
- Enable hardware acceleration
- Check firewall isn't blocking CDN

---

### Problem: STL Won't Load

**Check:**
1. File format (must be .stl)
2. File size (< 50MB recommended)
3. Browser console for errors

**Solution:**
- Decimate mesh in CAD software
- Convert to binary STL
- Check file isn't corrupted

---

### Problem: Robot Moves Erratically

**Check:**
1. Connection status (green dot)
2. Mode (Mode 1 or Mode 2)
3. Serial Monitor for ESP32 errors

**Solution:**
- Reconnect to WiFi
- Clear browser cache
- Reset ESP32

---

### Problem: High Deviation

**Check:**
1. DH parameters correct
2. Encoder calibration
3. Mechanical backlash

**Solution:**
- Re-home robot
- Update DH params in config
- Check mechanical assembly

---

## Files Modified Summary

| File | Type | Changes |
|------|------|---------|
| `index.html` | Complete Rewrite | Industrial 3-panel layout |
| `style.css` | Complete Rewrite | Dark theme, professional styling |
| `simulator.js` | New File | Three.js robot visualization |
| `app.js` | Updated | Integrated with simulator |
| `ui_server.cpp` | No Changes | (Uses existing endpoints) |
| `ui_state_manager.cpp` | No Changes | (Uses existing state mgmt) |

---

## Quick Start Guide

### First Time Setup:

1. Upload updated files to ESP32:
   ```bash
   pio run --target uploadfs
   ```

2. Open browser to `http://<ESP32-IP>`

3. Verify 3D view loads

4. (Optional) Load STL models:
   - Click "Load STL"
   - Select joint
   - Upload file

5. Test motion:
   - Click "Motion" tab
   - Jog a joint
   - Verify visual matches real

6. Switch to Mode 2:
   - Click "Mode" tab
   - Click "Mode 2"
   - Home robot first

7. Monitor deviation:
   - Deviation panel shows automatically
   - Green = good (< 2°)
   - Red = check calibration (> 2°)

---

## Future Enhancements

### Planned Features:
- [ ] Trajectory recording/playback
- [ ] Multi-robot coordination
- [ ] VR/AR support (WebXR)
- [ ] CAM integration (G-code import)
- [ ] Advanced collision detection
- [ ] Force feedback visualization
- [ ] Video recording/screenshots
- [ ] Cloud session storage

### Community Contributions Welcome:
- Additional robot models
- Custom tool designs
- UI themes
- Language translations

---

## End of Industrial UI Documentation

This UI transforms the ESP32 robot control into a professional-grade simulator without changing any firmware logic. All simulation runs in the browser, while the ESP32 remains the authoritative execution platform.