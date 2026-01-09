/**
 * Geometry Configuration Panel
 * 
 * FIX P2: Provides UI for configuring robot geometry
 * - Link lengths (L1, L2, L3, L4)
 * - TCP offset
 * - Joint limits
 * 
 * These values drive FK/IK computation in the simulator.
 */

import React, { useState, useCallback } from 'react';
import * as SimRobot from '../simulation/SimulatedRobot';

function GeometryConfigPanel() {
    // Initialize from current simulated values
    const [dhParams, setDhParams] = useState(() => SimRobot.getDHParams());
    const [jointLimits, setJointLimits] = useState(() => SimRobot.getJointLimits());
    const [tcpOffset, setTcpOffset] = useState({ x: 0, y: 0, z: 100 });
    const [saved, setSaved] = useState(false);

    // Handle DH parameter change
    const handleDHChange = useCallback((key, value) => {
        const numValue = parseFloat(value) || 0;
        setDhParams(prev => ({ ...prev, [key]: numValue }));
        setSaved(false);
    }, []);

    // Handle joint limit change
    const handleLimitChange = useCallback((jointIndex, field, value) => {
        const numValue = parseFloat(value) || 0;
        setJointLimits(prev => {
            const updated = [...prev];
            updated[jointIndex] = { ...updated[jointIndex], [field]: numValue };
            return updated;
        });
        setSaved(false);
    }, []);

    // Apply all changes to simulator
    const handleApply = useCallback(() => {
        SimRobot.setDHParams(dhParams);
        SimRobot.setJointLimits(jointLimits);
        setSaved(true);
        console.log('[GEOMETRY] Applied new configuration');
    }, [dhParams, jointLimits]);

    // Reset to defaults
    const handleReset = useCallback(() => {
        const defaultDH = { L1: 150, L2: 300, L3: 250, L4: 100 };
        const defaultLimits = SimRobot.JOINT_LIMITS.map(l => ({ ...l }));
        setDhParams(defaultDH);
        setJointLimits(defaultLimits);
        SimRobot.setDHParams(defaultDH);
        SimRobot.setJointLimits(defaultLimits);
        setSaved(true);
    }, []);

    return (
        <div className="panel geometry-panel">
            <h3>🔧 Robot Geometry</h3>
            <p className="panel-info">Configure link lengths and joint limits. Changes affect FK/IK computation.</p>

            {/* Link Lengths */}
            <div className="config-section">
                <h4>Link Lengths (mm)</h4>
                <div className="config-grid">
                    <div className="config-row">
                        <label>L1 (Base → Shoulder):</label>
                        <input
                            type="number"
                            value={dhParams.L1}
                            onChange={(e) => handleDHChange('L1', e.target.value)}
                            min="0"
                            step="10"
                        />
                    </div>
                    <div className="config-row">
                        <label>L2 (Shoulder → Elbow):</label>
                        <input
                            type="number"
                            value={dhParams.L2}
                            onChange={(e) => handleDHChange('L2', e.target.value)}
                            min="0"
                            step="10"
                        />
                    </div>
                    <div className="config-row">
                        <label>L3 (Elbow → Wrist):</label>
                        <input
                            type="number"
                            value={dhParams.L3}
                            onChange={(e) => handleDHChange('L3', e.target.value)}
                            min="0"
                            step="10"
                        />
                    </div>
                    <div className="config-row">
                        <label>L4 (Wrist → TCP):</label>
                        <input
                            type="number"
                            value={dhParams.L4}
                            onChange={(e) => handleDHChange('L4', e.target.value)}
                            min="0"
                            step="10"
                        />
                    </div>
                </div>
            </div>

            {/* TCP Offset */}
            <div className="config-section">
                <h4>TCP Offset (mm)</h4>
                <div className="config-grid horizontal">
                    <div className="config-item">
                        <label>X:</label>
                        <input
                            type="number"
                            value={tcpOffset.x}
                            onChange={(e) => setTcpOffset(prev => ({ ...prev, x: parseFloat(e.target.value) || 0 }))}
                            step="1"
                        />
                    </div>
                    <div className="config-item">
                        <label>Y:</label>
                        <input
                            type="number"
                            value={tcpOffset.y}
                            onChange={(e) => setTcpOffset(prev => ({ ...prev, y: parseFloat(e.target.value) || 0 }))}
                            step="1"
                        />
                    </div>
                    <div className="config-item">
                        <label>Z:</label>
                        <input
                            type="number"
                            value={tcpOffset.z}
                            onChange={(e) => setTcpOffset(prev => ({ ...prev, z: parseFloat(e.target.value) || 0 }))}
                            step="1"
                        />
                    </div>
                </div>
            </div>

            {/* Joint Limits */}
            <div className="config-section">
                <h4>Joint Limits (degrees)</h4>
                <div className="joint-limits-grid">
                    <div className="joint-header">
                        <span>Joint</span>
                        <span>Min</span>
                        <span>Max</span>
                        <span>MaxVel</span>
                    </div>
                    {jointLimits.map((limit, i) => (
                        <div key={i} className="joint-limit-row">
                            <span className="joint-label">J{i}</span>
                            <input
                                type="number"
                                value={limit.min}
                                onChange={(e) => handleLimitChange(i, 'min', e.target.value)}
                                step="5"
                            />
                            <input
                                type="number"
                                value={limit.max}
                                onChange={(e) => handleLimitChange(i, 'max', e.target.value)}
                                step="5"
                            />
                            <input
                                type="number"
                                value={limit.maxVel}
                                onChange={(e) => handleLimitChange(i, 'maxVel', e.target.value)}
                                step="10"
                            />
                        </div>
                    ))}
                </div>
            </div>

            {/* Action Buttons */}
            <div className="config-actions">
                <button className="primary-btn" onClick={handleApply}>
                    ✓ Apply Changes
                </button>
                <button className="secondary-btn" onClick={handleReset}>
                    ↺ Reset to Defaults
                </button>
                {saved && <span className="save-indicator">✓ Saved</span>}
            </div>

            {/* Info */}
            <div className="config-note">
                <strong>Note:</strong> These values are used by the simulator only.
                Hardware mode uses firmware-defined geometry.
            </div>
        </div>
    );
}

export default GeometryConfigPanel;
