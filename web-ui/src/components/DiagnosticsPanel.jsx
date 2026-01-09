/**
 * Diagnostics Panel
 * 
 * Shows:
 * - Alignment status per joint
 * - Fault history
 * - System health
 */

import React from 'react';
import useRobotStore from '../stores/robotStore';

function DiagnosticsPanel() {
    const {
        jointPositions,
        jointVelocities,
        aligned,
        alignmentWarning,
        alignmentCritical,
        speedModifier,
        maxJointError,
        faultCode,
        estopActive,
        isMoving,
        motorType,
        activeIKSolver,
    } = useRobotStore();

    return (
        <div className="panel diagnostics-panel">
            <h3>System Diagnostics</h3>

            {/* Alignment Status */}
            <div className="diag-section">
                <h4>Alignment Status</h4>
                <div className={`alignment-indicator ${alignmentCritical ? 'critical' : alignmentWarning ? 'warning' : 'ok'}`}>
                    <span className="indicator-icon">
                        {alignmentCritical ? '🔴' : alignmentWarning ? '🟡' : '🟢'}
                    </span>
                    <span className="indicator-text">
                        {alignmentCritical ? 'CRITICAL' : alignmentWarning ? 'WARNING' : 'ALIGNED'}
                    </span>
                </div>
                <div className="diag-stats">
                    <div className="stat-item">
                        <label>Max Error:</label>
                        <span>{maxJointError?.toFixed(2) || 0}°</span>
                    </div>
                    <div className="stat-item">
                        <label>Speed Modifier:</label>
                        <span>{((speedModifier || 1) * 100).toFixed(0)}%</span>
                    </div>
                </div>
            </div>

            {/* Joint Positions */}
            <div className="diag-section">
                <h4>Joint Positions</h4>
                <div className="joint-grid">
                    {[0, 1, 2, 3, 4, 5].map(i => (
                        <div key={i} className="joint-row">
                            <span className="joint-label">J{i}</span>
                            <div className="joint-bar-container">
                                <div
                                    className="joint-bar"
                                    style={{
                                        width: `${Math.abs((jointPositions[i] || 0) / 180) * 100}%`,
                                        marginLeft: (jointPositions[i] || 0) >= 0 ? '50%' : `${50 - Math.abs((jointPositions[i] || 0) / 180) * 50}%`,
                                        backgroundColor: (jointPositions[i] || 0) >= 0 ? '#4ecdc4' : '#ff6b6b',
                                    }}
                                />
                            </div>
                            <span className="joint-value">{(jointPositions[i] || 0).toFixed(1)}°</span>
                            <span className="joint-velocity">{(jointVelocities[i] || 0).toFixed(1)}°/s</span>
                        </div>
                    ))}
                </div>
            </div>

            {/* System Status */}
            <div className="diag-section">
                <h4>System Status</h4>
                <div className="status-grid">
                    <div className="status-row">
                        <label>Mode:</label>
                        <span className="badge simulator">SIMULATOR</span>
                    </div>
                    <div className="status-row">
                        <label>Motor Type:</label>
                        <span>{motorType || 'SIMULATED'}</span>
                    </div>
                    <div className="status-row">
                        <label>IK Solver:</label>
                        <span>{activeIKSolver || 'NUMERICAL'}</span>
                    </div>
                    <div className="status-row">
                        <label>Moving:</label>
                        <span className={isMoving ? 'active' : ''}>{isMoving ? 'Yes' : 'No'}</span>
                    </div>
                    <div className="status-row">
                        <label>E-Stop:</label>
                        <span className={estopActive ? 'fault' : ''}>{estopActive ? 'ACTIVE' : 'Clear'}</span>
                    </div>
                    <div className="status-row">
                        <label>Fault Code:</label>
                        <span className={faultCode ? 'fault' : ''}>0x{(faultCode || 0).toString(16).padStart(8, '0')}</span>
                    </div>
                </div>
            </div>

            {/* Hardware Note */}
            <div className="diag-section hardware-note">
                <h4>ℹ️ Hardware Status</h4>
                <p>
                    Running in <strong>SIMULATION MODE</strong>. No physical hardware connected.
                    All motion is simulated in the browser.
                </p>
                <p>
                    To connect real hardware, configure the ESP32 firmware and WebSocket endpoint.
                </p>
            </div>
        </div>
    );
}

export default DiagnosticsPanel;
