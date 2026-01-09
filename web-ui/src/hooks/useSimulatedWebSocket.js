/**
 * Simulated WebSocket Hook
 * 
 * For SIMULATION MODE ONLY - no real WebSocket connection.
 * Uses the SimulatedRobot module to generate state updates.
 * 
 * FIXES APPLIED:
 * - P0: Mode switch is synchronous (no race condition)
 * - P0: Fault clearing properly syncs estopActive to Zustand
 */

import { useEffect, useRef, useCallback } from 'react';
import useRobotStore from '../stores/robotStore';
import * as SimRobot from '../simulation/SimulatedRobot';

// Track if hook has initialized (singleton pattern to prevent double init)
let isInitialized = false;

export function useSimulatedWebSocket() {
    const updateIntervalRef = useRef(null);
    const alignmentIntervalRef = useRef(null);

    // Get store methods via getState() to avoid dependency array issues
    const store = useRobotStore;

    // ========== SYNCHRONOUS INITIALIZATION (FIX P0: Mode Hang) ==========
    // Use empty dependency array and getState() to prevent race conditions
    useEffect(() => {
        // Prevent double initialization in React StrictMode
        if (isInitialized) {
            return;
        }
        isInitialized = true;

        // SYNCHRONOUS mode application - no async, no pending state
        store.getState().setConnected(true);
        store.getState().setLatency(0);
        store.getState().applyModeChange({
            success: true,
            current_mode: 'SIMULATOR',
        });

        // State update loop (10Hz) using RAF for better performance
        const runUpdateLoop = () => {
            SimRobot.updateMotion();
            const state = SimRobot.getState();
            store.getState().updateFromFirmware(state.payload);
        };

        // Alignment update loop (2Hz)
        const runAlignmentLoop = () => {
            SimRobot.updateAlignment();
            const status = SimRobot.getAlignmentStatus();
            store.getState().updateAlignment(status.payload);
        };

        // Start intervals
        updateIntervalRef.current = setInterval(runUpdateLoop, 100);
        alignmentIntervalRef.current = setInterval(runAlignmentLoop, 500);

        // Run immediately once
        runUpdateLoop();
        runAlignmentLoop();

        return () => {
            isInitialized = false;
            clearInterval(updateIntervalRef.current);
            clearInterval(alignmentIntervalRef.current);
        };
    }, []); // Empty deps - runs once only

    // ========== IK PREVIEW ==========
    const requestIKPreview = useCallback((target, seed, solver) => {
        const result = SimRobot.computeIK(target, seed);
        store.getState().setIKResult(result);
        return result;
    }, []);

    // ========== MODE SWITCH (FIX P0: Immediate resolution) ==========
    const requestModeSwitch = useCallback((toSimulator) => {
        // Immediately resolve - no pending state
        if (!toSimulator) {
            // Hardware mode requested - reject immediately
            store.getState().applyModeChange({
                success: false,
                current_mode: 'SIMULATOR',
                rejection_reason: 'No hardware connected - simulation only mode',
                rejection_code: 99,
            });
        } else {
            // Simulator mode - accept immediately
            store.getState().applyModeChange({
                success: true,
                current_mode: 'SIMULATOR',
            });
        }
    }, []);

    // ========== MOTION COMMANDS (FIX P0: Fault sync) ==========
    const sendMotionCommand = useCallback((command) => {
        if (command.type === 'joint_move') {
            return SimRobot.startMotion(command.targets, command.velocity || 30);
        }
        else if (command.type === 'cartesian_move') {
            const ikResult = SimRobot.computeIK(command.target);
            if (ikResult.feasible) {
                return SimRobot.startMotion(ikResult.joints_deg, command.velocity || 30);
            } else {
                return { success: false, error: ikResult.error_message };
            }
        }
        else if (command.type === 'stop') {
            SimRobot.stopMotion();
            return { success: true };
        }
        else if (command.type === 'estop') {
            SimRobot.emergencyStop();
            // IMMEDIATELY sync fault state to Zustand
            store.getState().updateFromFirmware({
                fault_code: SimRobot.SimulatedRobot.faultCode,
                motion_allowed: false,
            });
            store.getState().updateSafety({
                fault_code: SimRobot.SimulatedRobot.faultCode,
                fault_name: 'E_STOP_ACTIVE',
                motion_blocked: true,
            });
            return { success: true };
        }
        else if (command.type === 'clear_fault') {
            // FIX P0: Clear fault AND immediately sync to Zustand
            SimRobot.clearFault();

            // IMMEDIATELY update store with cleared state
            store.getState().updateFromFirmware({
                fault_code: 0,
                motion_allowed: true,
            });
            store.getState().updateSafety({
                fault_code: 0,
                fault_name: '',
                motion_blocked: false,
            });

            return { success: true };
        }
        return { success: false, error: 'Unknown command' };
    }, []);

    // ========== JOG JOINT ==========
    const jogJoint = useCallback((jointIndex, direction, velocity = 30) => {
        const currentPos = SimRobot.SimulatedRobot.jointPositions[jointIndex];
        const newTargets = [...SimRobot.SimulatedRobot.jointPositions];
        newTargets[jointIndex] = currentPos + direction * 5; // 5 degree increments

        // Clamp to limits
        const limit = SimRobot.JOINT_LIMITS[jointIndex];
        newTargets[jointIndex] = Math.max(limit.min, Math.min(limit.max, newTargets[jointIndex]));

        return SimRobot.startMotion(newTargets, velocity);
    }, []);

    // ========== GEOMETRY CONFIG ==========
    const setRobotGeometry = useCallback((geometry) => {
        SimRobot.setDHParams(geometry);
        return { success: true };
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
        setRobotGeometry,
        isSimulated: true,
    };
}

export default useSimulatedWebSocket;
