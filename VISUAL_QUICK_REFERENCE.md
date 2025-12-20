# Industrial Robot Simulator - Visual Quick Reference

## UI Layout Map

```
┌─────────────────────────────────────────────────────────────────────────┐
│  ① TOOLBAR (48px)                                        ⑦ CONNECTION   │
│  📁 Load STL  |  ▶ Play  |  ⏸ Pause  |  ⏹ Stop  |  More...   🟢 Connected  │
├─────────────┬────────────────────────────────────────────┬──────────────┤
│             │                                            │              │
│  ② PROJECT  │  ③ 3D SIMULATION VIEW (MAIN)              │  ④ CONTROLS  │
│    TREE     │                                            │              │
│  (260px)    │            [WebGL Canvas]                  │   (320px)    │
│             │                                            │              │
│  🤖 Robot   │     ┌──── Y (Green)                        │  ┌─Motion──┐ │
│   ├ J0: 0°  │     │                                      │  │Jog ± │ │
│   ├ J1: 45° │     │     [Robot Model]                    │  │Speed    │ │
│   └ TCP     │     └──→ X (Red)                           │  │Accel    │ │
│             │          /                                 │  └─────────┘ │
│  📐 Frames  │         Z (Blue)                           │              │
│  ✈ Planes   │                                            │  ┌─Mode────┐ │
│  📍 Points  │     [Grid Floor 1000x1000mm]               │  │Mode 1   │ │
│  📦 STLs    │     [Shadows Enabled]                      │  │Mode 2   │ │
│             │                                            │  └─────────┘ │
│             │  ⑥ OVERLAY INFO                            │              │
│             │  Mode: MODE 1  |  Status: Ready            │  ┌─Config──┐ │
│             │                                            │  │Limits   │ │
│             │  ⑧ DEVIATION (Mode 2)                      │  │DH       │ │
│             │  J0: 0.15°  J1: 2.45°                      │  └─────────┘ │
│             │  Avg: 0.84°                                │              │
│             │                                            │  ┌─Safety──┐ │
│             │                                            │  │E-Stop   │ │
│             │                                            │  │Clear    │ │
│             │                                            │  └─────────┘ │
│             │                                            │              │
└─────────────┴────────────────────────────────────────────┴──────────────┘
     ⑤ MOUSE CONTROLS:
     • Left Click + Drag = Rotate camera
     • Right Click + Drag = Pan view
     • Scroll Wheel = Zoom in/out
```

---

## Element Guide

### ① Toolbar - Quick Actions

```
┌──────────────────────────────────────────────────────────┐
│  📁   ▶   ⏸   ⏹   │   🔄   📐   💾   │     🟢 Connected │
│  Load Play Pause Stop│  Reset Frames Export│   Status     │
└──────────────────────────────────────────────────────────┘
```

**Function Map:**
- **📁 Load STL** → Opens dialog to upload robot models
- **▶ Play** → Start motion simulation
- **⏸ Pause** → Pause current motion
- **⏹ Stop** → Stop and reset to zero
- **🔄 Reset View** → Reset camera to default position
- **📐 Frames** → Toggle coordinate axes visibility
- **💾 Export** → Download session data (JSON)
- **🟢 Status** → Connection indicator (green=ok, red=disconnected)

---

### ② Project Tree - Live Data

```
▼ 🤖 Robot Arm
  ├ ⚙ Joint 0: 0.0°    ← Live angle
  ├ ⚙ Joint 1: 45.2°   ← Updates real-time
  ├ ⚙ Joint 2: -30.1°
  ├ ⚙ Joint 3: 0.0°
  ├ ⚙ Joint 4: 90.5°
  ├ ⚙ Joint 5: 0.0°
  └ 🎯 TCP

▶ 📐 Reference Frames    ← Click to expand
▶ ✈ User Planes         ← Shows defined planes
▶ 📍 Taught Points      ← Shows saved positions
▶ 📦 STL Models         ← Shows loaded models
```

**Interaction:**
- Click ▶/▼ to expand/collapse
- Values auto-update from ESP32
- Color-coded: 🟢 Teal = active data

---

### ③ 3D Simulation View - Main Workspace

```
        Y (Green)
        ↑
        │
        │    [Robot Model]
        │      /  \
        │     /    \
        │    Base  Links
        │
        └────────────→ X (Red)
       /
      /
     Z (Blue)

┌────────────────────┐
│ Grid: 1000x1000mm  │
│ Divisions: 20      │
│ Shadows: Enabled   │
│ Lighting: Realistic│
└────────────────────┘
```

**Camera Controls:**
- **Left Mouse + Drag** → Rotate around robot
- **Right Mouse + Drag** → Pan view
- **Scroll Wheel** → Zoom in/out
- **Double Click** → Focus on point

**Visual Elements:**
- 🔴 Red Axis = X direction
- 🟢 Green Axis = Y direction
- 🔵 Blue Axis = Z direction
- Grid squares = 50mm each
- Shadows = Real-time calculated

---

### ④ Controls Panel - Tabbed Interface

```
┌─────────────────────────────┐
│ [Motion] Mode Config Safety │ ← Click to switch
├─────────────────────────────┤
│                             │
│  Selected Tab Content       │
│  Shows Here                 │
│                             │
└─────────────────────────────┘
```

#### Tab 1: Motion Control
```
┌───────────────────────┐
│ Jog Joint:            │
│ [Select: Joint 0 ▼]   │
│ [ ◀ - ] [ + ▶ ]       │
│                       │
│ Speed: 30 deg/s       │
│ [═══●═══════] 5-120   │
│                       │
│ Acceleration: 100     │
│ [═══●═══════] 50-500  │
│                       │
│ Cartesian Target:     │
│ X: [300  ] mm         │
│ Y: [0    ] mm         │
│ Z: [250  ] mm         │
│ [Move Linear]         │
│                       │
│ [  ⏹ STOP ALL  ]      │
└───────────────────────┘
```

#### Tab 2: Mode Selection
```
┌────────────────────────────┐
│  ┌────────┐  ┌────────┐   │
│  │   🔓   │  │   🔒   │   │
│  │ MODE 1 │  │ MODE 2 │   │
│  │ Open   │  │Closed  │   │
│  │ Loop   │  │ Loop   │   │
│  └────────┘  └────────┘   │
│                            │
│  Current: MODE 1           │
└────────────────────────────┘
```

#### Tab 3: Configuration
```
┌────────────────────────┐
│ Edit Joint Limits:     │
│ [Select: Joint 0 ▼]    │
│ Min: [-180 ] deg       │
│ Max: [ 180 ] deg       │
│ [Update Limits]        │
│                        │
│ [💾 Save to ESP32]     │
└────────────────────────┘
```

#### Tab 4: Safety
```
┌────────────────────────┐
│ Status:                │
│ • Homed: Yes           │
│ • Motion Allowed: Yes  │
│ • Faults: 0x00000000   │
│                        │
│ [ 🛑 EMERGENCY STOP ]  │
│ [  ✓ Clear Fault   ]   │
└────────────────────────┘
```

---

### ⑤ Mouse Control Reference

```
╔═══════════════════════════════════╗
║  MOUSE CONTROLS (3D View)         ║
╠═══════════════════════════════════╣
║                                   ║
║  🖱️ Left Button + Drag            ║
║  → Rotate camera around robot     ║
║                                   ║
║  🖱️ Right Button + Drag           ║
║  → Pan/move view                  ║
║                                   ║
║  🖱️ Scroll Wheel                  ║
║  → Zoom in/out                    ║
║                                   ║
║  🖱️ Double Click                  ║
║  → Focus on clicked point         ║
║                                   ║
╚═══════════════════════════════════╝
```

**Pro Tips:**
- Hold Shift while rotating = faster
- Hold Ctrl while panning = slower (precise)
- Reset if lost: Click "Reset View" button

---

### ⑥ Overlay Info - Status Display

```
┌──────────────────────────────────┐
│  Mode: MODE 1  │  Status: Ready  │  ← Top-left overlay
└──────────────────────────────────┘

Translucent background
Always visible
Updates in real-time
```

**Status Values:**
- **Ready** = Idle, no motion
- **Moving** = Executing trajectory
- **Fault** = Error detected (check safety tab)

---

### ⑦ Connection Indicator

```
🟢 Connected     = Good, receiving data
🔴 Disconnected  = Lost connection
🟡 Connecting... = Attempting connection (pulsing)
```

**Troubleshooting:**
- 🔴 Red = Check WiFi, verify IP address
- 🟡 Pulsing = Wait 3-5 seconds
- 🟢 Green = All systems operational

---

### ⑧ Deviation Panel (Mode 2 Only)

```
┌─────────────────────────┐
│ Simulation vs Reality   │
├─────────────────────────┤
│ J0: 0.15° 🟢           │
│ J1: 2.45° 🔴           │ ← Red if >2°
│ J2: 0.82° 🟢           │
│ J3: 0.05° 🟢           │
│ J4: 1.23° 🟢           │
│ J5: 0.34° 🟢           │
├─────────────────────────┤
│ Avg: 0.84°             │
└─────────────────────────┘
```

**Color Code:**
- 🟢 Green (<2°) = Excellent accuracy
- 🔴 Red (>2°) = Check calibration

**Only visible in Mode 2** (closed-loop with encoders)

---

## STL Upload Dialog

```
╔════════════════════════════════════╗
║  Load STL Model               [×]  ║
╠════════════════════════════════════╣
║                                    ║
║  Select Joint:                     ║
║  [Joint 1 ▼]                       ║
║    ├─ Base                         ║
║    ├─ Joint 0                      ║
║    ├─ Joint 1                      ║
║    ├─ ...                          ║
║    └─ Tool/TCP                     ║
║                                    ║
║  Upload STL File:                  ║
║  [Choose File] joint1.stl          ║
║                                    ║
║  [ Load Model ]                    ║
║                                    ║
╚════════════════════════════════════╝
```

**Workflow:**
1. Click "📁 Load STL" in toolbar
2. Select which joint
3. Choose .STL file from computer
4. Click "Load Model"
5. Model appears in 3D view

---

## Color Coding Reference

### Robot Components:
- 🔵 **Base/Links** = Gray metallic (#505050)
- 🟢 **Joints** = Teal (#4ec9b0)
- 🔴 **TCP** = Red with glow (#ff4444)

### UI Elements:
- 🟦 **Primary Actions** = Blue (#0e639c)
- ⬜ **Secondary Actions** = Gray (#3e3e42)
- 🟧 **Warnings** = Orange (#f39c12)
- 🟥 **Dangers** = Red (#e74c3c)
- 🟩 **Success** = Green (#27ae60)

### Data Display:
- 🟢 **Active Values** = Teal (#4ec9b0)
- ⚪ **Labels** = Gray (#888888)
- ⚪ **Text** = Light gray (#e0e0e0)

---

## Keyboard Shortcuts

```
╔═══════════════════════════════════╗
║  KEYBOARD SHORTCUTS               ║
╠═══════════════════════════════════╣
║  ESC        → Close dialogs       ║
║  Space      → Stop motion         ║
║  R          → Reset camera        ║
║  F          → Toggle frames       ║
║  1-4        → Switch tabs         ║
║  +/-        → Zoom in/out         ║
║  Arrow Keys → Pan view            ║
╚═══════════════════════════════════╝
```

*(Not all shortcuts may be implemented - check browser console)*

---

## Workflow Examples

### Example 1: Simple Joint Move

```
1. Click "Motion" tab              → ④ Controls Panel
2. Select "Joint 1"                → Dropdown
3. Set speed to 30 deg/s           → Slider
4. Click "+" button                → Jog forward
5. Watch robot move in 3D view     → ③ Main View
6. Check deviation panel           → ⑧ (Mode 2 only)
```

### Example 2: Cartesian Move

```
1. Click "Motion" tab              → ④ Controls Panel
2. Enter coordinates:
   X: 300, Y: 0, Z: 250           → Input fields
3. Click "Move Linear"             → Button
4. Watch trajectory in 3D          → ③ Main View
5. Verify arrival                  → Check tree ②
```

### Example 3: Load Robot Model

```
1. Export CAD model to STL         → External CAD software
2. Click "📁 Load STL"              → ① Toolbar
3. Select "Joint 1"                → Dialog
4. Choose file "joint1.stl"       → File picker
5. Click "Load Model"              → Button
6. Repeat for all joints           → Steps 2-5
7. Jog joints to verify assembly   → Motion tab
```

---

## Status Messages Explained

| Message | Meaning | Action |
|---------|---------|--------|
| "Connected" | ESP32 reachable | ✅ Normal operation |
| "Disconnected" | Lost WiFi | 🔧 Check network |
| "Ready" | Idle, can accept commands | ✅ Send command |
| "Moving" | Executing trajectory | ⏸️ Wait or stop |
| "Fault" | Error detected | 🛑 Check safety tab |
| "Homed: Yes" | Mode 2 ready | ✅ Can use IK |
| "Homed: No" | Need to home | 🏠 Home robot first |

---

## Best Practices

### ✅ DO:
- Start with default geometric model
- Test with Mode 1 first
- Jog one joint at a time initially
- Use "Reset View" if camera lost
- Export session data regularly
- Check deviation panel in Mode 2

### ❌ DON'T:
- Upload huge STL files (keep <50MB)
- Command motion during "Moving" status
- Skip homing in Mode 2
- Ignore red deviation values
- Close browser during motion
- Forget to save configuration

---

## Quick Troubleshooting

```
Problem: 3D View Blank
├─ Check: Browser console (F12)
├─ Check: Three.js loaded?
└─ Fix: Refresh page (Ctrl+F5)

Problem: Robot Not Moving
├─ Check: Connection status (green?)
├─ Check: Safety tab (faults?)
└─ Fix: Clear fault, check WiFi

Problem: High Deviation
├─ Check: Mode 2 active?
├─ Check: Robot homed?
└─ Fix: Re-home, check encoders

Problem: STL Won't Load
├─ Check: File size (<50MB?)
├─ Check: File format (.stl?)
└─ Fix: Decimate mesh in CAD
```

---

## File Structure Reference

```
ESP32_RobotArm/
└── data/
    ├── index.html      ← Main UI structure
    ├── style.css       ← Industrial styling
    ├── app.js          ← Connection logic
    └── simulator.js    ← 3D visualization
```

**Upload Command:**
```bash
pio run --target uploadfs
```

---

## Support Resources

- **Browser Console** (F12) - See JavaScript errors
- **Serial Monitor** - See ESP32 output
- **Documentation** - INDUSTRIAL_UI_DOCUMENTATION.md
- **Update Guide** - UPDATE_SUMMARY.md

---

**Remember:** All simulation runs in browser. ESP32 firmware is unchanged. This is a visual upgrade only! 🎨🤖