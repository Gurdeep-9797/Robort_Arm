/**
 * Motion Control Panel
 * 
 * Provides:
 * - Joint jog controls
 * - Cartesian move
 * - Speed/acceleration settings
 * - Stop/E-Stop buttons
 */

import React, { useState } from 'react';
import useRobotStore from '../stores/robotStore';
import useSimulatedWebSocket from '../hooks/useSimulatedWebSocket';

function MotionPanel() {
    const { jointPositions, isMoving, motionAllowed, estopActive } = useRobotStore();
    const { jogJoint, sendMotionCommand } = useSimulatedWebSocket();

    const [selectedJoint, setSelectedJoint] = useState(0);
    const [speed, setSpeed] = useState(30);
    const [cartesianTarget, setCartesianTarget] = useState({ x: 300, y: 0, z: 250 });

    const handleJog = (direction) => {
        if (!motionAllowed || estopActive) return;
        jogJoint(selectedJoint, direction, speed);
    };

    const handleCartesianMove = () => {
        if (!motionAllowed || estopActive) return;
        sendMotionCommand({
            type: 'cartesian_move',
            target: [cartesianTarget.x, cartesianTarget.y, cartesianTarget.z, 0, 0, 0],
            velocity: speed,
        });
    };

    const handleStop = () => {
        sendMotionCommand({ type: 'stop' });
    };

    const handleEStop = () => {
        sendMotionCommand({ type: 'estop' });
    };

    const handleClearFault = () => {
        sendMotionCommand({ type: 'clear_fault' });
    };

    return (
        <div className="panel motion-panel">
            <h3>Motion Control</h3>

            {/* Jog Controls */}
            <div className="control-group">
                <label>Jog Joint</label>
                <select
                    value={selectedJoint}
                    onChange={(e) => setSelectedJoint(parseInt(e.target.value))}
                >
                    {[0, 1, 2, 3, 4, 5].map(i => (
                        <option key={i} value={i}>
                            Joint {i}: {jointPositions[i]?.toFixed(1) || 0}°
                        </option>
                    ))}
                </select>
                <div className="jog-buttons">
                    <button
                        className="jog-btn"
                        onClick={() => handleJog(-1)}
                        disabled={!motionAllowed || isMoving}
                    >
                        ◀ -5°
                    </button>
                    <button
                        className="jog-btn"
                        onClick={() => handleJog(1)}
                        disabled={!motionAllowed || isMoving}
                    >
                        +5° ▶
                    </button>
                </div>
            </div>

            {/* Speed Control */}
            <div className="control-group">
                <label>Speed: {speed} deg/s</label>
                <input
                    type="range"
                    min="5"
                    max="120"
                    value={speed}
                    onChange={(e) => setSpeed(parseInt(e.target.value))}
                />
            </div>

            {/* Cartesian Target */}
            <div className="control-group">
                <label>Cartesian Target (mm)</label>
                <div className="input-row">
                    <input
                        type="number"
                        placeholder="X"
                        value={cartesianTarget.x}
                        onChange={(e) => setCartesianTarget({ ...cartesianTarget, x: parseFloat(e.target.value) || 0 })}
                    />
                    <input
                        type="number"
                        placeholder="Y"
                        value={cartesianTarget.y}
                        onChange={(e) => setCartesianTarget({ ...cartesianTarget, y: parseFloat(e.target.value) || 0 })}
                    />
                    <input
                        type="number"
                        placeholder="Z"
                        value={cartesianTarget.z}
                        onChange={(e) => setCartesianTarget({ ...cartesianTarget, z: parseFloat(e.target.value) || 0 })}
                    />
                </div>
                <button
                    className="primary-btn"
                    onClick={handleCartesianMove}
                    disabled={!motionAllowed || isMoving}
                >
                    Move Linear
                </button>
            </div>

            {/* Safety Controls */}
            <div className="control-group safety-controls">
                <button
                    className="warning-btn"
                    onClick={handleStop}
                    disabled={!isMoving}
                >
                    ⏹ STOP
                </button>
                <button
                    className="danger-btn"
                    onClick={handleEStop}
                >
                    🛑 E-STOP
                </button>
                <button
                    className="secondary-btn"
                    onClick={handleClearFault}
                    disabled={!estopActive}
                >
                    ✓ Clear Fault
                </button>
            </div>

            {/* Status */}
            <div className="motion-status">
                {isMoving && <span className="status-moving">⚡ Motion in progress...</span>}
                {estopActive && <span className="status-estop">🛑 E-STOP ACTIVE</span>}
                {!motionAllowed && !estopActive && <span className="status-blocked">Motion blocked</span>}
            </div>
        </div>
    );
}

export default MotionPanel;
