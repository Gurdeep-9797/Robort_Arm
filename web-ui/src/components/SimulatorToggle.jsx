/**
 * Simulator vs Hardware Toggle
 * 
 * Safety-critical component:
 * - Cannot switch during motion
 * - Requires firmware approval
 * - Visual confirmation of mode
 * 
 * FIX P0: Uses simulated WebSocket (not real hardware)
 * FIX P0: Does not set modeChangePending since simulation is synchronous
 */

import React from 'react';
import useRobotStore from '../stores/robotStore';
// FIX P0: Use simulated WebSocket since we're in simulation-only mode
import useSimulatedWebSocket from '../hooks/useSimulatedWebSocket';

function SimulatorToggle() {
    const {
        simulatorMode,
        hardwareArmed,
        modeChangePending,
        modeChangeError,
        isMoving,
        faultCode,
        estopActive,
    } = useRobotStore();

    const { requestModeSwitch } = useSimulatedWebSocket();

    // Determine if switch is allowed
    const canSwitch = !isMoving && !modeChangePending && !estopActive;

    const handleToggle = () => {
        if (!canSwitch) return;

        // FIX P0: Just call requestModeSwitch - it handles everything synchronously
        // Don't call requestModeChange which sets pending=true
        requestModeSwitch(!simulatorMode);
    };

    // Determine blocking reason
    let blockReason = null;
    if (isMoving) blockReason = 'Motion in progress';
    if (modeChangePending) blockReason = 'Switching...';
    if (estopActive) blockReason = 'E-Stop active';

    return (
        <div className="simulator-toggle">
            <div className="toggle-container">
                {/* Hardware Mode Button */}
                <button
                    className={`mode-button hardware ${!simulatorMode ? 'active' : ''}`}
                    onClick={() => simulatorMode && handleToggle()}
                    disabled={!canSwitch || !simulatorMode}
                >
                    <span className="mode-icon">⚙️</span>
                    <span className="mode-label">HARDWARE</span>
                    {hardwareArmed && !simulatorMode && (
                        <span className="armed-indicator">ARMED</span>
                    )}
                </button>

                {/* Toggle Switch */}
                <div
                    className={`toggle-switch ${simulatorMode ? 'simulator' : 'hardware'} ${!canSwitch ? 'disabled' : ''}`}
                    onClick={handleToggle}
                    role="switch"
                    aria-checked={simulatorMode}
                >
                    <div className="toggle-thumb"></div>
                </div>

                {/* Simulator Mode Button */}
                <button
                    className={`mode-button simulator ${simulatorMode ? 'active' : ''}`}
                    onClick={() => !simulatorMode && handleToggle()}
                    disabled={!canSwitch || simulatorMode}
                >
                    <span className="mode-icon">🎮</span>
                    <span className="mode-label">SIMULATOR</span>
                </button>
            </div>

            {/* Status Message */}
            <div className="toggle-status">
                {modeChangePending && (
                    <span className="status-pending">Switching mode...</span>
                )}
                {modeChangeError && (
                    <span className="status-error">{modeChangeError}</span>
                )}
                {blockReason && !modeChangePending && (
                    <span className="status-blocked">{blockReason}</span>
                )}
            </div>

            {/* Safety Warning for Hardware Mode */}
            {!simulatorMode && (
                <div className="hardware-warning">
                    ⚠️ Commands will move real motors
                </div>
            )}
        </div>
    );
}

export default SimulatorToggle;
