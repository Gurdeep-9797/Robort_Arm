/**
 * WebSocket Hook
 * 
 * Manages connection to ESP32 firmware with:
 * - Auto-reconnect with exponential backoff
 * - Message versioning
 * - Disconnect = Safe mode trigger
 */

import { useEffect, useRef, useCallback } from 'react';
import useRobotStore from '../stores/robotStore';

const WS_RECONNECT_DELAY_MS = 500;
const WS_MAX_RECONNECT_DELAY_MS = 10000;
const WS_DISCONNECT_GRACE_MS = 500;

export function useWebSocket(url = null) {
    const wsRef = useRef(null);
    const reconnectTimeoutRef = useRef(null);
    const disconnectTimerRef = useRef(null);
    const reconnectDelayRef = useRef(WS_RECONNECT_DELAY_MS);

    const {
        setConnected,
        setLatency,
        updateFromFirmware,
        updateAlignment,
        updateSafety,
        setIKResult,
        applyModeChange,
        requestModeChange,
    } = useRobotStore();

    // Determine WebSocket URL
    const wsUrl = url || `ws://${window.location.host}/ws`;

    // ========== MESSAGE HANDLERS ==========

    const handleMessage = useCallback((event) => {
        try {
            const message = JSON.parse(event.data);

            // Validate version
            if (message.version !== 1) {
                console.warn('Unknown message version:', message.version);
                return;
            }

            // Update latency
            if (message.timestamp_ms) {
                setLatency(Date.now() - message.timestamp_ms);
            }

            // Route by type
            switch (message.type) {
                case 'STATE_UPDATE':
                    updateFromFirmware(message.payload);
                    break;

                case 'ALIGNMENT_STATUS':
                    updateAlignment(message.payload);
                    break;

                case 'SAFETY_FAULT':
                    updateSafety(message.payload);
                    break;

                case 'IK_PREVIEW_RESULT':
                    setIKResult(message.payload);
                    break;

                case 'IK_FAILURE':
                    setIKResult({
                        feasible: false,
                        error_code: message.payload.error_code,
                        error_message: message.payload.error_message,
                    });
                    break;

                case 'MODE_CHANGE_RESPONSE':
                    applyModeChange(message.payload);
                    break;

                case 'MODE_STATUS':
                    applyModeChange({
                        success: true,
                        current_mode: message.payload.mode,
                    });
                    break;

                default:
                    console.log('Unknown message type:', message.type);
            }
        } catch (err) {
            console.error('WebSocket message parse error:', err);
        }
    }, [updateFromFirmware, updateAlignment, updateSafety, setIKResult, applyModeChange, setLatency]);

    // ========== CONNECTION MANAGEMENT ==========

    const connect = useCallback(() => {
        if (wsRef.current?.readyState === WebSocket.OPEN) {
            return;
        }

        try {
            wsRef.current = new WebSocket(wsUrl);

            wsRef.current.onopen = () => {
                console.log('[WS] Connected');
                setConnected(true);
                reconnectDelayRef.current = WS_RECONNECT_DELAY_MS;

                // Clear disconnect timer
                if (disconnectTimerRef.current) {
                    clearTimeout(disconnectTimerRef.current);
                    disconnectTimerRef.current = null;
                }
            };

            wsRef.current.onclose = () => {
                console.log('[WS] Disconnected');

                // Start grace period before entering safe mode
                if (!disconnectTimerRef.current) {
                    disconnectTimerRef.current = setTimeout(() => {
                        setConnected(false);
                        // Request simulator mode on disconnect (safe state)
                        requestModeChange(true);
                        console.warn('[WS] Grace period expired - entering safe mode');
                    }, WS_DISCONNECT_GRACE_MS);
                }

                // Schedule reconnect with exponential backoff
                reconnectTimeoutRef.current = setTimeout(() => {
                    reconnectDelayRef.current = Math.min(
                        reconnectDelayRef.current * 2,
                        WS_MAX_RECONNECT_DELAY_MS
                    );
                    connect();
                }, reconnectDelayRef.current);
            };

            wsRef.current.onerror = (error) => {
                console.error('[WS] Error:', error);
            };

            wsRef.current.onmessage = handleMessage;

        } catch (err) {
            console.error('[WS] Connection failed:', err);
        }
    }, [wsUrl, setConnected, handleMessage, requestModeChange]);

    // ========== SEND FUNCTIONS ==========

    const send = useCallback((type, payload) => {
        if (wsRef.current?.readyState !== WebSocket.OPEN) {
            console.warn('[WS] Cannot send - not connected');
            return false;
        }

        const message = {
            version: 1,
            timestamp_ms: Date.now(),
            type,
            payload,
        };

        wsRef.current.send(JSON.stringify(message));
        return true;
    }, []);

    const requestIKPreview = useCallback((target, seed, solver) => {
        return send('IK_PREVIEW_REQUEST', {
            target_mm: target.slice(0, 3),
            orientation_deg: target.slice(3, 6),
            seed_joints_deg: seed,
            solver,
        });
    }, [send]);

    const requestModeSwitch = useCallback((toSimulator) => {
        return send('MODE_CHANGE_REQUEST', {
            target_mode: toSimulator ? 'SIMULATOR' : 'HARDWARE',
            force: false,
        });
    }, [send]);

    const sendMotionCommand = useCallback((command) => {
        // Safety check: never send motion in simulator mode
        const store = useRobotStore.getState();
        if (store.simulatorMode) {
            console.warn('[WS] Motion command blocked - simulator mode');
            return false;
        }
        return send('MOTION_COMMAND', command);
    }, [send]);

    // ========== LIFECYCLE ==========

    useEffect(() => {
        connect();

        return () => {
            if (reconnectTimeoutRef.current) {
                clearTimeout(reconnectTimeoutRef.current);
            }
            if (disconnectTimerRef.current) {
                clearTimeout(disconnectTimerRef.current);
            }
            if (wsRef.current) {
                wsRef.current.close();
            }
        };
    }, [connect]);

    return {
        send,
        requestIKPreview,
        requestModeSwitch,
        sendMotionCommand,
    };
}

export default useWebSocket;
