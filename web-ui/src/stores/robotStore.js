/**
 * Robot State Store (Zustand)
 * 
 * AUTHORITY MODEL:
 * - Firmware is authoritative for robot state
 * - UI state is local only (commands, preferences)
 * - Mode changes require firmware approval
 */

import { create } from 'zustand';

// ============================================================================
// STORE DEFINITION
// ============================================================================

const useRobotStore = create((set, get) => ({
    // ========== CONNECTION STATE ==========
    connected: false,
    latencyMs: 0,
    lastUpdateMs: 0,

    // ========== MODE STATE (Firmware Authoritative) ==========
    simulatorMode: true,          // FIX P0: Default to true (simulation-only mode)
    hardwareArmed: false,         // true = motors can move
    modeChangePending: false,
    modeChangeError: null,

    // ========== ROBOT STATE (Firmware Authoritative) ==========
    mode: 'MODE_1_OPEN_LOOP',     // Operating mode
    isMoving: false,
    isHomed: false,
    motionAllowed: true,
    faultCode: 0,

    // Joint state
    jointPositions: [0, 0, 0, 0, 0, 0],
    jointVelocities: [0, 0, 0, 0, 0, 0],
    jointTargets: [0, 0, 0, 0, 0, 0],

    // TCP state
    tcpPosition: [0, 0, 0],
    tcpOrientation: [0, 0, 0],

    // Motor state
    motorType: 'SERVO',           // 'SERVO' | 'DC_ENCODER'

    // Active frame
    activeFrame: 'BASE',          // 'WORLD' | 'BASE' | 'TOOL'

    // ========== IK STATE ==========
    activeIKSolver: 'NUMERICAL',
    ikPreference: 'ACCURACY',
    lastIKResult: null,
    ikPreviewEnabled: true,

    // ========== SAFETY STATE (Firmware Authoritative) ==========
    aligned: true,
    alignmentWarning: false,
    alignmentCritical: false,
    speedModifier: 1.0,
    maxJointError: 0,
    tcpError: 0,
    estopActive: false,

    // ========== UI LOCAL STATE ==========
    selectedJoint: 0,
    jogSpeed: 30,
    cartesianTarget: [300, 0, 250],
    showGhostRobot: false,
    ghostJoints: [0, 0, 0, 0, 0, 0],

    // ========== GRAPH STATE ==========
    graphNodes: [],
    graphEdges: [],
    selectedNode: null,

    // ========== ACTIONS ==========

    // Connection
    setConnected: (connected) => set({ connected }),
    setLatency: (latencyMs) => set({ latencyMs }),

    // Mode control (requests firmware approval)
    requestModeChange: (simulatorMode) => {
        set({ modeChangePending: true, modeChangeError: null });
        // Actual change handled by WebSocket response
    },

    applyModeChange: (response) => {
        if (response.success) {
            set({
                simulatorMode: response.current_mode === 'SIMULATOR',
                hardwareArmed: response.current_mode === 'HARDWARE',
                modeChangePending: false,
                modeChangeError: null,
            });
        } else {
            set({
                modeChangePending: false,
                modeChangeError: response.rejection_reason,
            });
        }
    },

    // State updates from firmware
    updateFromFirmware: (state) => {
        set({
            lastUpdateMs: Date.now(),
            mode: state.mode || get().mode,
            isMoving: state.is_moving ?? get().isMoving,
            isHomed: state.is_homed ?? get().isHomed,
            motionAllowed: state.motion_allowed ?? get().motionAllowed,
            faultCode: state.fault_code ?? get().faultCode,
            jointPositions: state.joints?.positions_deg || get().jointPositions,
            jointVelocities: state.joints?.velocities_deg_s || get().jointVelocities,
            jointTargets: state.joints?.targets_deg || get().jointTargets,
            tcpPosition: state.tcp?.position_mm || get().tcpPosition,
            tcpOrientation: state.tcp?.orientation_deg || get().tcpOrientation,
            motorType: state.motor_type || get().motorType,
            activeFrame: state.active_frame || get().activeFrame,
        });
    },

    // Alignment updates
    updateAlignment: (status) => {
        set({
            aligned: status.aligned,
            alignmentWarning: status.warning,
            alignmentCritical: status.critical,
            speedModifier: status.speed_modifier,
            maxJointError: status.max_joint_error_deg,
            tcpError: status.tcp_error_mm,
        });
    },

    // Safety updates
    updateSafety: (fault) => {
        set({
            faultCode: fault.fault_code,
            estopActive: fault.fault_name === 'E_STOP_ACTIVE',
            motionAllowed: !fault.motion_blocked,
        });
    },

    // IK result
    setIKResult: (result) => set({ lastIKResult: result }),
    setIKSolver: (solver) => set({ activeIKSolver: solver }),
    setIKPreference: (pref) => set({ ikPreference: pref }),

    // Ghost robot for preview
    setGhostRobot: (joints) => set({
        showGhostRobot: true,
        ghostJoints: joints
    }),
    hideGhostRobot: () => set({ showGhostRobot: false }),

    // UI controls
    setSelectedJoint: (joint) => set({ selectedJoint: joint }),
    setJogSpeed: (speed) => set({ jogSpeed: speed }),
    setCartesianTarget: (target) => set({ cartesianTarget: target }),
    setActiveFrame: (frame) => set({ activeFrame: frame }),

    // Graph state
    setGraphNodes: (nodes) => set({ graphNodes: nodes }),
    setGraphEdges: (edges) => set({ graphEdges: edges }),
    setSelectedNode: (nodeId) => set({ selectedNode: nodeId }),

    // Simulator-only actions (local state only)
    simulateJointMove: (joints) => {
        if (get().simulatorMode) {
            set({ jointPositions: joints });
        }
    },
}));

export default useRobotStore;
