/**
 * Simulated WebSocket Hook
 * 
 * For SIMULATION MODE ONLY - no real WebSocket connection.
 * Uses the SimulatedRobot module to generate state updates.
 */

import { useEffect, useRef, useCallback } from 'react';
import useRobotStore from '../stores/robotStore';
import * as SimRobot from '../simulation/SimulatedRobot';

export function useSimulatedWebSocket() {
    const updateIntervalRef = useRef(null);
    const alignmentIntervalRef = useRef(null);

    const {
        setConnected,
        setLatency,
        updateFromFirmware,
        updateAlignment,
        setIKResult,
        applyModeChange,
    } = useRobotStore();

    // Start simulation loop
    useEffect(() => {
        // Mark as "connected" immediately (simulated)
        setConnected(true);
        setLatency(0); // No latency in simulation

        // Apply simulator mode
        applyModeChange({
            success: true,
            current_mode: 'SIMULATOR',
        });

        // State update loop (10Hz)
        updateIntervalRef.current = setInterval(() => {
            SimRobot.updateMotion();
            const state = SimRobot.getState();
            updateFromFirmware(state.payload);
        }, 100);

        // Alignment update loop (2Hz)
        alignmentIntervalRef.current = setInterval(() => {
            SimRobot.updateAlignment();
            const status = SimRobot.getAlignmentStatus();
            updateAlignment(status.payload);
        }, 500);

        return () => {
            clearInterval(updateIntervalRef.current);
            clearInterval(alignmentIntervalRef.current);
        };
    }, [setConnected, setLatency, updateFromFirmware, updateAlignment, applyModeChange]);

    // ========== SIMULATED SEND FUNCTIONS ==========

    const requestIKPreview = useCallback((target, seed, solver) => {
        const result = SimRobot.computeIK(target, seed);
        setIKResult(result);
        return true;
    }, [setIKResult]);

    const requestModeSwitch = useCallback((toSimulator) => {
        // In simulation mode, we're always in simulator
        // Hardware mode is not available without ESP32
        if (!toSimulator) {
            applyModeChange({
                success: false,
                current_mode: 'SIMULATOR',
                rejection_reason: 'No hardware connected - simulation only mode',
                rejection_code: 99,
            });
        } else {
            applyModeChange({
                success: true,
                current_mode: 'SIMULATOR',
            });
        }
    }, [applyModeChange]);

    const sendMotionCommand = useCallback((command) => {
        if (command.type === 'joint_move') {
            return SimRobot.startMotion(command.targets, command.velocity || 30);
        } else if (command.type === 'cartesian_move') {
            const ikResult = SimRobot.computeIK(command.target);
            if (ikResult.feasible) {
                return SimRobot.startMotion(ikResult.joints_deg, command.velocity || 30);
            } else {
                return { success: false, error: ikResult.error_message };
            }
        } else if (command.type === 'stop') {
            SimRobot.stopMotion();
            return { success: true };
        } else if (command.type === 'estop') {
            SimRobot.emergencyStop();
            return { success: true };
        } else if (command.type === 'clear_fault') {
            SimRobot.clearFault();
            return { success: true };
        }
        return { success: false, error: 'Unknown command' };
    }, []);

    const jogJoint = useCallback((jointIndex, direction, velocity = 30) => {
        const currentPos = SimRobot.SimulatedRobot.jointPositions[jointIndex];
        const newTargets = [...SimRobot.SimulatedRobot.jointPositions];
        newTargets[jointIndex] = currentPos + direction * 5; // 5 degree increments

        // Clamp to limits
        const limit = SimRobot.JOINT_LIMITS[jointIndex];
        newTargets[jointIndex] = Math.max(limit.min, Math.min(limit.max, newTargets[jointIndex]));

        return SimRobot.startMotion(newTargets, velocity);
    }, []);

    // Dummy send function (for compatibility)
    const send = useCallback((type, payload) => {
        console.log('[SIMULATED] Message:', type, payload);
        return true;
    }, []);

    return {
        send,
        requestIKPreview,
        requestModeSwitch,
        sendMotionCommand,
        jogJoint,
        isSimulated: true,
    };
}

export default useSimulatedWebSocket;
