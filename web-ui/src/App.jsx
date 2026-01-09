/**
 * Main App Component
 * 
 * Industrial Robot Simulator with:
 * - Tab-based navigation
 * - 3D Robot view
 * - Connection graph
 * - Safety status bar
 * 
 * SIMULATION-FIRST: Runs without hardware by default.
 * Hardware mode requires explicit configuration.
 */

import React, { useState, Suspense, lazy } from 'react';
import useRobotStore from './stores/robotStore';
// Use SIMULATED WebSocket by default (no hardware required)
import useSimulatedWebSocket from './hooks/useSimulatedWebSocket';
import SimulatorToggle from './components/SimulatorToggle';

// Lazy load heavy components
const RobotSimulator = lazy(() => import('./components/RobotSimulator'));
const ConnectionsGraph = lazy(() => import('./components/ConnectionsGraph'));
const GeometryConfigPanel = lazy(() => import('./components/GeometryConfigPanel'));

// ============================================================================
// TAB DEFINITIONS
// ============================================================================

const TABS = [
    { id: 'simulator', label: '3D Simulator', icon: '🤖' },
    { id: 'connections', label: 'Connections', icon: '🔗' },
    { id: 'motion', label: 'Motion', icon: '🎯' },
    { id: 'geometry', label: 'Geometry', icon: '📐' },
    { id: 'diagnostics', label: 'Diagnostics', icon: '📊' },
];

// ============================================================================
// APP COMPONENT
// ============================================================================

function App() {
    const [activeTab, setActiveTab] = useState('simulator');
    const { connected, simulatorMode, faultCode, aligned, estopActive } = useRobotStore();

    // Initialize SIMULATED WebSocket (no hardware required)
    // This runs entirely in browser - no ESP32 connection needed
    const { jogJoint, sendMotionCommand } = useSimulatedWebSocket();

    return (
        <div className="app-container">
            {/* ========== HEADER ========== */}
            <header className="app-header">
                <div className="header-left">
                    <h1 className="app-title">ESP32 Robot Arm</h1>
                    <span className="app-subtitle">Industrial Simulator</span>
                </div>

                <div className="header-center">
                    <SimulatorToggle />
                </div>

                <div className="header-right">
                    <ConnectionStatus connected={connected} />
                    <SafetyIndicator
                        estop={estopActive}
                        fault={faultCode !== 0}
                        aligned={aligned}
                    />
                </div>
            </header>

            {/* ========== TAB NAVIGATION ========== */}
            <nav className="tab-navigation">
                {TABS.map(tab => (
                    <button
                        key={tab.id}
                        className={`tab-button ${activeTab === tab.id ? 'active' : ''}`}
                        onClick={() => setActiveTab(tab.id)}
                    >
                        <span className="tab-icon">{tab.icon}</span>
                        <span className="tab-label">{tab.label}</span>
                    </button>
                ))}
            </nav>

            {/* ========== MAIN CONTENT ========== */}
            <main className="main-content">
                <Suspense fallback={<LoadingSpinner />}>
                    {activeTab === 'simulator' && <RobotSimulator />}
                    {activeTab === 'connections' && <ConnectionsGraph />}
                    {activeTab === 'motion' && <MotionPanel />}
                    {activeTab === 'geometry' && <GeometryConfigPanel />}
                    {activeTab === 'diagnostics' && <DiagnosticsPanel />}
                </Suspense>
            </main>

            {/* ========== STATUS BAR ========== */}
            <footer className="status-bar">
                <StatusItem label="Mode" value={simulatorMode ? 'SIMULATOR' : 'HARDWARE'} />
                <StatusItem label="Lat" value={`${useRobotStore.getState().latencyMs}ms`} />
                <StatusItem label="Fault" value={`0x${faultCode.toString(16).padStart(8, '0')}`} />
            </footer>
        </div>
    );
}

// ============================================================================
// SUB-COMPONENTS
// ============================================================================

function ConnectionStatus({ connected }) {
    return (
        <div className={`connection-status ${connected ? 'connected' : 'disconnected'}`}>
            <span className="status-dot"></span>
            <span className="status-text">{connected ? 'Connected' : 'Disconnected'}</span>
        </div>
    );
}

function SafetyIndicator({ estop, fault, aligned }) {
    let status = 'ok';
    let text = 'Safe';

    if (estop) {
        status = 'estop';
        text = 'E-STOP';
    } else if (fault) {
        status = 'fault';
        text = 'FAULT';
    } else if (!aligned) {
        status = 'warning';
        text = 'Misaligned';
    }

    return (
        <div className={`safety-indicator ${status}`}>
            <span className="safety-icon">⚡</span>
            <span className="safety-text">{text}</span>
        </div>
    );
}

function StatusItem({ label, value }) {
    return (
        <div className="status-item">
            <span className="status-label">{label}:</span>
            <span className="status-value">{value}</span>
        </div>
    );
}

function LoadingSpinner() {
    return (
        <div className="loading-spinner">
            <div className="spinner"></div>
            <span>Loading...</span>
        </div>
    );
}

// Placeholder components
const MotionPanel = lazy(() => import('./components/MotionPanel'));
const DiagnosticsPanel = lazy(() => import('./components/DiagnosticsPanel'));

export default App;
