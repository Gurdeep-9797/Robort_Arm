/**
 * Simulator Worker
 * 
 * Runs IK/FK computations off main thread.
 * Uses gl-matrix for performance.
 */

importScripts('https://cdnjs.cloudflare.com/ajax/libs/gl-matrix/3.4.3/gl-matrix-min.js');

// ============================================================================
// ROBOT KINEMATICS PARAMETERS (DH)
// ============================================================================

const DH_PARAMS = {
    // Link lengths in mm
    L1: 150,    // Base to shoulder height
    L2: 300,    // Shoulder to elbow
    L3: 250,    // Elbow to wrist
    L4: 100,    // Wrist to tool
};

const JOINT_LIMITS = [
    { min: -180, max: 180 },   // Joint 0 - Base
    { min: -90, max: 90 },     // Joint 1 - Shoulder
    { min: -135, max: 135 },   // Joint 2 - Elbow
    { min: -180, max: 180 },   // Joint 3 - Wrist roll
    { min: -120, max: 120 },   // Joint 4 - Wrist pitch
    { min: -180, max: 180 },   // Joint 5 - Wrist yaw
];

// ============================================================================
// MESSAGE HANDLER
// ============================================================================

self.onmessage = function (e) {
    const { type, id, payload } = e.data;

    let result;

    switch (type) {
        case 'FK':
            result = computeFK(payload.joints);
            break;

        case 'IK':
            result = computeIK(payload.target, payload.seed);
            break;

        case 'VALIDATE':
            result = validateJoints(payload.joints);
            break;

        case 'TRAJECTORY':
            result = computeTrajectory(payload.start, payload.end, payload.steps);
            break;

        default:
            result = { error: 'Unknown command' };
    }

    self.postMessage({ type: type + '_RESULT', id, payload: result });
};

// ============================================================================
// FORWARD KINEMATICS
// ============================================================================

function computeFK(joints) {
    const startTime = performance.now();

    // Convert to radians
    const q = joints.map(j => j * Math.PI / 180);

    // Base rotation
    const c0 = Math.cos(q[0]);
    const s0 = Math.sin(q[0]);
    const c1 = Math.cos(q[1]);
    const s1 = Math.sin(q[1]);
    const c2 = Math.cos(q[2]);
    const s2 = Math.sin(q[2]);

    // Simplified 3-DOF arm FK (first 3 joints for position)
    const r = DH_PARAMS.L2 * c1 + DH_PARAMS.L3 * Math.cos(q[1] + q[2]);
    const z = DH_PARAMS.L1 + DH_PARAMS.L2 * s1 + DH_PARAMS.L3 * Math.sin(q[1] + q[2]);

    const x = r * c0;
    const y = r * s0;

    // Orientation from wrist joints (simplified)
    const rx = joints[3];
    const ry = joints[4];
    const rz = joints[5];

    const computeTime = performance.now() - startTime;

    return {
        position: [x, y, z],
        orientation: [rx, ry, rz],
        compute_time_ms: computeTime,
    };
}

// ============================================================================
// INVERSE KINEMATICS
// ============================================================================

function computeIK(target, seed) {
    const startTime = performance.now();

    const x = target[0];
    const y = target[1];
    const z = target[2];

    // Base rotation
    const theta0 = Math.atan2(y, x) * 180 / Math.PI;

    // Arm geometry
    const r = Math.sqrt(x * x + y * y);
    const s = z - DH_PARAMS.L1;
    const D = Math.sqrt(r * r + s * s);

    const L1 = DH_PARAMS.L2;
    const L2 = DH_PARAMS.L3;

    // Reachability check
    if (D > L1 + L2 * 0.99 || D < Math.abs(L1 - L2) * 1.01) {
        return {
            feasible: false,
            error_code: 2,
            error_message: 'Target unreachable',
            compute_time_ms: performance.now() - startTime,
        };
    }

    // Elbow angle (cosine law)
    let cosTheta2 = (D * D - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    cosTheta2 = Math.max(-1, Math.min(1, cosTheta2));

    const theta2 = Math.atan2(Math.sqrt(1 - cosTheta2 * cosTheta2), cosTheta2) * 180 / Math.PI;

    // Shoulder angle
    const alpha = Math.atan2(s, r);
    const beta = Math.atan2(L2 * Math.sin(theta2 * Math.PI / 180),
        L1 + L2 * Math.cos(theta2 * Math.PI / 180));
    const theta1 = (alpha - beta) * 180 / Math.PI;

    // Wrist angles (from target orientation or seed)
    const theta3 = target[3] || (seed ? seed[3] : 0);
    const theta4 = target[4] || (seed ? seed[4] : 0);
    const theta5 = target[5] || (seed ? seed[5] : 0);

    const joints = [
        normalizeAngle(theta0),
        normalizeAngle(theta1),
        normalizeAngle(theta2),
        normalizeAngle(theta3),
        normalizeAngle(theta4),
        normalizeAngle(theta5),
    ];

    // Validate limits
    const validation = validateJoints(joints);
    if (!validation.valid) {
        return {
            feasible: false,
            error_code: 3,
            error_message: 'Joint limit exceeded',
            limiting_joint: validation.limiting_joint,
            compute_time_ms: performance.now() - startTime,
        };
    }

    // Compute error
    const fk = computeFK(joints);
    const dx = fk.position[0] - x;
    const dy = fk.position[1] - y;
    const dz = fk.position[2] - z;
    const error = Math.sqrt(dx * dx + dy * dy + dz * dz);

    // Singularity check (elbow angle)
    const singularityMetric = Math.abs(Math.sin(theta2 * Math.PI / 180));

    const computeTime = performance.now() - startTime;

    return {
        feasible: true,
        joints_deg: joints,
        velocities_deg_s: [30, 40, 35, 20, 25, 30], // Default velocities
        confidence: singularityMetric > 0.3 ? 0.9 : 0.6,
        error_mm: error,
        singularity_metric: singularityMetric,
        singularity_warning: singularityMetric < 0.15,
        joint_limit_warning: false,
        limiting_joint: -1,
        compute_time_ms: computeTime,
    };
}

// ============================================================================
// VALIDATION
// ============================================================================

function validateJoints(joints) {
    for (let i = 0; i < 6; i++) {
        if (joints[i] < JOINT_LIMITS[i].min || joints[i] > JOINT_LIMITS[i].max) {
            return {
                valid: false,
                limiting_joint: i,
                error: `Joint ${i} (${joints[i].toFixed(1)}°) exceeds limit [${JOINT_LIMITS[i].min}, ${JOINT_LIMITS[i].max}]`,
            };
        }
    }
    return { valid: true, limiting_joint: -1 };
}

// ============================================================================
// TRAJECTORY
// ============================================================================

function computeTrajectory(start, end, steps) {
    const trajectory = [];

    for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        const joints = [];

        for (let j = 0; j < 6; j++) {
            // Smooth interpolation (cosine easing)
            const ease = (1 - Math.cos(t * Math.PI)) / 2;
            joints.push(start[j] + (end[j] - start[j]) * ease);
        }

        const fk = computeFK(joints);
        trajectory.push({
            joints,
            position: fk.position,
            t,
        });
    }

    return { trajectory };
}

// ============================================================================
// HELPERS
// ============================================================================

function normalizeAngle(angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}
