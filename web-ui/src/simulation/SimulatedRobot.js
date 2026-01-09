/**
 * Simulated Robot Backend
 * 
 * Provides a complete mock of the ESP32 firmware for browser-only operation.
 * This runs entirely in the browser - no ESP32, no WebSocket to real hardware.
 * 
 * Features:
 * - Simulated motor controllers
 * - Simulated encoders with realistic feedback
 * - IK/FK computation
 * - Safety logic (limits, singularities)
 * - Alignment checking
 */

// ============================================================================
// SIMULATED ROBOT STATE
// ============================================================================

const SimulatedRobot = {
    // Mode
    simulatorMode: true,  // ALWAYS true in this file - this IS the simulator

    // Joint state
    jointPositions: [0, 0, 0, 0, 0, 0],
    jointVelocities: [0, 0, 0, 0, 0, 0],
    jointTargets: [0, 0, 0, 0, 0, 0],

    // Motion state
    isMoving: false,
    motionProgress: 0,
    motionStartTime: 0,
    motionDuration: 0,
    motionStartPositions: [0, 0, 0, 0, 0, 0],

    // Safety state
    faultCode: 0,
    estopActive: false,
    isHomed: false,
    motionAllowed: true,

    // Alignment state
    aligned: true,
    alignmentWarning: false,
    alignmentCritical: false,
    maxJointError: 0,
    speedModifier: 1.0,

    // Motor type (simulated)
    motorType: 'SERVO',

    // IK solver
    activeIKSolver: 'NUMERICAL',

    // Operating mode
    operatingMode: 'MODE_1_OPEN_LOOP',

    // Uptime
    startTime: Date.now(),
};

// ============================================================================
// JOINT LIMITS
// ============================================================================

const JOINT_LIMITS = [
    { min: -180, max: 180, maxVel: 120 },  // Joint 0 - Base
    { min: -90, max: 90, maxVel: 90 },     // Joint 1 - Shoulder
    { min: -135, max: 135, maxVel: 100 },  // Joint 2 - Elbow
    { min: -180, max: 180, maxVel: 150 },  // Joint 3 - Wrist roll
    { min: -120, max: 120, maxVel: 120 },  // Joint 4 - Wrist pitch
    { min: -180, max: 180, maxVel: 180 },  // Joint 5 - Wrist yaw
];

// ============================================================================
// DH PARAMETERS (MUTABLE - User configurable)
// ============================================================================

let DH = {
    L1: 150,  // Base to shoulder (mm)
    L2: 300,  // Shoulder to elbow (mm)
    L3: 250,  // Elbow to wrist (mm)
    L4: 100,  // Wrist to TCP (mm)
};

// FIX P2: Allow runtime geometry configuration
function setDHParams(params) {
    DH = { ...DH, ...params };
    console.log('[SIMULATED] DH params updated:', DH);
}

function getDHParams() {
    return { ...DH };
}

// Mutable joint limits
let jointLimits = [...JOINT_LIMITS];

function setJointLimits(limits) {
    jointLimits = limits.map((l, i) => ({ ...JOINT_LIMITS[i], ...l }));
    console.log('[SIMULATED] Joint limits updated:', jointLimits);
}

function getJointLimits() {
    return jointLimits.map(l => ({ ...l }));
}

// ============================================================================
// FORWARD KINEMATICS
// ============================================================================

function computeFK(joints) {
    const q = joints.map(j => j * Math.PI / 180);

    const c0 = Math.cos(q[0]);
    const s0 = Math.sin(q[0]);
    const c1 = Math.cos(q[1]);
    const s1 = Math.sin(q[1]);
    const c2 = Math.cos(q[2]);
    const s2 = Math.sin(q[2]);

    // Simplified FK for 3-DOF arm position
    const r = DH.L2 * c1 + DH.L3 * Math.cos(q[1] + q[2]);
    const z = DH.L1 + DH.L2 * s1 + DH.L3 * Math.sin(q[1] + q[2]);

    const x = r * c0;
    const y = r * s0;

    return {
        position: [x, y, z],
        orientation: [joints[3], joints[4], joints[5]],
    };
}

// ============================================================================
// INVERSE KINEMATICS
// ============================================================================

function computeIK(target, seed) {
    const x = target[0];
    const y = target[1];
    const z = target[2];

    // Base rotation
    const theta0 = Math.atan2(y, x) * 180 / Math.PI;

    // Arm geometry
    const r = Math.sqrt(x * x + y * y);
    const s = z - DH.L1;
    const D = Math.sqrt(r * r + s * s);

    const L1 = DH.L2;
    const L2 = DH.L3;

    // Reachability check
    if (D > L1 + L2 * 0.99 || D < Math.abs(L1 - L2) * 1.01) {
        return {
            feasible: false,
            error_code: 2,
            error_message: 'Target unreachable',
        };
    }

    // Elbow angle
    let cosTheta2 = (D * D - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    cosTheta2 = Math.max(-1, Math.min(1, cosTheta2));
    const theta2 = Math.atan2(Math.sqrt(1 - cosTheta2 * cosTheta2), cosTheta2) * 180 / Math.PI;

    // Shoulder angle
    const alpha = Math.atan2(s, r);
    const beta = Math.atan2(L2 * Math.sin(theta2 * Math.PI / 180),
        L1 + L2 * Math.cos(theta2 * Math.PI / 180));
    const theta1 = (alpha - beta) * 180 / Math.PI;

    // Wrist angles
    const theta3 = target[3] || 0;
    const theta4 = target[4] || 0;
    const theta5 = target[5] || 0;

    const joints = [
        normalizeAngle(theta0),
        normalizeAngle(theta1),
        normalizeAngle(theta2),
        normalizeAngle(theta3),
        normalizeAngle(theta4),
        normalizeAngle(theta5),
    ];

    // Validate limits
    for (let i = 0; i < 6; i++) {
        if (joints[i] < JOINT_LIMITS[i].min || joints[i] > JOINT_LIMITS[i].max) {
            return {
                feasible: false,
                error_code: 3,
                error_message: `Joint ${i} limit exceeded`,
                limiting_joint: i,
            };
        }
    }

    // Singularity check
    const singularityMetric = Math.abs(Math.sin(theta2 * Math.PI / 180));

    return {
        feasible: true,
        joints_deg: joints,
        velocities_deg_s: [30, 40, 35, 20, 25, 30],
        confidence: singularityMetric > 0.3 ? 0.9 : 0.6,
        error_mm: 0.5,
        singularity_metric: singularityMetric,
        singularity_warning: singularityMetric < 0.15,
        joint_limit_warning: false,
        limiting_joint: -1,
    };
}

function normalizeAngle(angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// ============================================================================
// MOTION SIMULATION
// ============================================================================

function startMotion(targetJoints, velocity = 30) {
    if (!SimulatedRobot.motionAllowed || SimulatedRobot.estopActive) {
        return { success: false, error: 'Motion not allowed' };
    }

    // Validate targets
    for (let i = 0; i < 6; i++) {
        if (targetJoints[i] < JOINT_LIMITS[i].min || targetJoints[i] > JOINT_LIMITS[i].max) {
            return { success: false, error: `Joint ${i} limit exceeded` };
        }
    }

    // Calculate motion duration based on longest move
    let maxTime = 0;
    for (let i = 0; i < 6; i++) {
        const distance = Math.abs(targetJoints[i] - SimulatedRobot.jointPositions[i]);
        const jointVel = Math.min(velocity, JOINT_LIMITS[i].maxVel);
        const time = distance / jointVel;
        if (time > maxTime) maxTime = time;
    }

    SimulatedRobot.motionStartPositions = [...SimulatedRobot.jointPositions];
    SimulatedRobot.jointTargets = [...targetJoints];
    SimulatedRobot.motionStartTime = Date.now();
    SimulatedRobot.motionDuration = maxTime * 1000; // Convert to ms
    SimulatedRobot.isMoving = true;
    SimulatedRobot.motionProgress = 0;

    return { success: true, duration: maxTime };
}

function updateMotion() {
    if (!SimulatedRobot.isMoving) return;

    const elapsed = Date.now() - SimulatedRobot.motionStartTime;
    const progress = Math.min(1, elapsed / SimulatedRobot.motionDuration);

    // Smooth easing
    const ease = (1 - Math.cos(progress * Math.PI)) / 2;

    // Interpolate positions
    for (let i = 0; i < 6; i++) {
        const start = SimulatedRobot.motionStartPositions[i];
        const end = SimulatedRobot.jointTargets[i];
        SimulatedRobot.jointPositions[i] = start + (end - start) * ease;

        // Compute velocity
        if (SimulatedRobot.motionDuration > 0) {
            const distance = end - start;
            // Velocity follows sine curve (derivative of cosine ease)
            const velFactor = Math.sin(progress * Math.PI);
            SimulatedRobot.jointVelocities[i] = (distance / (SimulatedRobot.motionDuration / 1000)) * velFactor * 1.57;
        }
    }

    SimulatedRobot.motionProgress = progress;

    if (progress >= 1) {
        SimulatedRobot.isMoving = false;
        SimulatedRobot.jointVelocities = [0, 0, 0, 0, 0, 0];
    }
}

function stopMotion() {
    SimulatedRobot.isMoving = false;
    SimulatedRobot.jointTargets = [...SimulatedRobot.jointPositions];
    SimulatedRobot.jointVelocities = [0, 0, 0, 0, 0, 0];
}

function emergencyStop() {
    stopMotion();
    SimulatedRobot.faultCode = 32; // E-STOP fault
    SimulatedRobot.estopActive = true;
    SimulatedRobot.motionAllowed = false;
}

function clearFault() {
    SimulatedRobot.faultCode = 0;
    SimulatedRobot.estopActive = false;
    SimulatedRobot.motionAllowed = true;
}

// ============================================================================
// ALIGNMENT SIMULATION
// ============================================================================

function updateAlignment() {
    // Simulate encoder noise
    const noiseLevel = 0.3; // degrees
    let maxError = 0;
    let worstJoint = 0;

    for (let i = 0; i < 6; i++) {
        const simNoise = (Math.random() - 0.5) * noiseLevel;
        const error = Math.abs(simNoise);
        if (error > maxError) {
            maxError = error;
            worstJoint = i;
        }
    }

    SimulatedRobot.maxJointError = maxError;

    // Check thresholds
    if (maxError > 5) {
        SimulatedRobot.alignmentCritical = true;
        SimulatedRobot.alignmentWarning = true;
        SimulatedRobot.aligned = false;
        SimulatedRobot.speedModifier = 0;
    } else if (maxError > 2) {
        SimulatedRobot.alignmentCritical = false;
        SimulatedRobot.alignmentWarning = true;
        SimulatedRobot.aligned = false;
        SimulatedRobot.speedModifier = 0.5;
    } else {
        SimulatedRobot.alignmentCritical = false;
        SimulatedRobot.alignmentWarning = false;
        SimulatedRobot.aligned = true;
        SimulatedRobot.speedModifier = 1.0;
    }
}

// ============================================================================
// STATE GENERATION
// ============================================================================

function getState() {
    const tcp = computeFK(SimulatedRobot.jointPositions);

    return {
        version: 1,
        timestamp_ms: Date.now(),
        type: 'STATE_UPDATE',
        payload: {
            mode: SimulatedRobot.operatingMode,
            is_moving: SimulatedRobot.isMoving,
            is_homed: SimulatedRobot.isHomed,
            motion_allowed: SimulatedRobot.motionAllowed,
            fault_code: SimulatedRobot.faultCode,
            joints: {
                positions_deg: [...SimulatedRobot.jointPositions],
                velocities_deg_s: [...SimulatedRobot.jointVelocities],
                targets_deg: [...SimulatedRobot.jointTargets],
            },
            tcp: {
                position_mm: tcp.position,
                orientation_deg: tcp.orientation,
            },
            motor_type: SimulatedRobot.motorType,
            active_frame: 'BASE',
        },
    };
}

function getAlignmentStatus() {
    return {
        version: 1,
        timestamp_ms: Date.now(),
        type: 'ALIGNMENT_STATUS',
        payload: {
            aligned: SimulatedRobot.aligned,
            warning: SimulatedRobot.alignmentWarning,
            critical: SimulatedRobot.alignmentCritical,
            speed_modifier: SimulatedRobot.speedModifier,
            max_joint_error_deg: SimulatedRobot.maxJointError,
            worst_joint: 0,
            tcp_error_mm: SimulatedRobot.maxJointError * 2,
            fault_code: SimulatedRobot.alignmentCritical ? 64 : 0,
            fault_message: SimulatedRobot.alignmentCritical ? 'Alignment critical' : '',
        },
    };
}

function getModeStatus() {
    return {
        version: 1,
        timestamp_ms: Date.now(),
        type: 'MODE_STATUS',
        payload: {
            mode: 'SIMULATOR',  // Always simulator in this file
            simulator_allowed: true,
            hardware_armed: false,  // Never armed - no hardware
            estop_active: SimulatedRobot.estopActive,
        },
    };
}

function getSystemGraph() {
    return {
        version: 1,
        timestamp_ms: Date.now(),
        type: 'SYSTEM_GRAPH',
        nodes: [
            { id: 'core', status: SimulatedRobot.isMoving ? 'ACTIVE' : 'IDLE', uptime_ms: Date.now() - SimulatedRobot.startTime },
            { id: 'ik_mgr', status: 'ACTIVE', solver: SimulatedRobot.activeIKSolver },
            { id: 'motor_mgr', status: SimulatedRobot.isMoving ? 'ACTIVE' : 'IDLE', type: 'SIMULATED' },
            { id: 'safety', status: SimulatedRobot.faultCode ? 'FAULT' : 'ACTIVE', fault_code: SimulatedRobot.faultCode },
            { id: 'align', status: SimulatedRobot.alignmentCritical ? 'FAULT' : (SimulatedRobot.alignmentWarning ? 'WARNING' : 'ACTIVE') },
            { id: 'servo_ctrl', status: 'DISABLED', note: 'SIMULATED' },
            { id: 'dc_ctrl', status: 'DISABLED', note: 'SIMULATED' },
            { id: 'encoders', status: 'ACTIVE', note: 'SIMULATED' },
        ],
    };
}

// ============================================================================
// EXPORTS
// ============================================================================

export {
    SimulatedRobot,
    JOINT_LIMITS,
    DH,
    computeFK,
    computeIK,
    startMotion,
    updateMotion,
    stopMotion,
    emergencyStop,
    clearFault,
    updateAlignment,
    getState,
    getAlignmentStatus,
    getModeStatus,
    getSystemGraph,
    // P2: Geometry configuration exports
    setDHParams,
    getDHParams,
    setJointLimits,
    getJointLimits,
};
