/**
 * 3D Robot Simulator
 * 
 * React Three Fiber based visualization:
 * - FK-based joint animation
 * - Coordinate frames
 * - Ghost robot preview
 * - TCP path tracing
 */

import React, { useRef, useMemo, useEffect, useState } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Line, Html } from '@react-three/drei';
import * as THREE from 'three';
import useRobotStore from '../stores/robotStore';

// ============================================================================
// ROBOT GEOMETRY CONSTANTS
// ============================================================================

const LINK_COLORS = {
    base: '#3d5a80',
    shoulder: '#98c1d9',
    elbow: '#e0fbfc',
    wrist: '#ee6c4d',
    tool: '#293241',
    ghost: '#00ff0044',
};

const DH = {
    L1: 0.15,   // Base height (meters for Three.js)
    L2: 0.30,   // Shoulder to elbow
    L3: 0.25,   // Elbow to wrist
    L4: 0.10,   // Wrist to TCP
};

// ============================================================================
// MAIN COMPONENT
// ============================================================================

function RobotSimulator() {
    const { jointPositions, showGhostRobot, ghostJoints, activeFrame } = useRobotStore();
    const [tcpPath, setTcpPath] = useState([]);

    return (
        <div className="robot-simulator">
            <Canvas
                camera={{ position: [1, 1, 1], fov: 50 }}
                shadows
            >
                {/* Lighting */}
                <ambientLight intensity={0.4} />
                <directionalLight position={[5, 10, 5]} intensity={0.8} castShadow />
                <pointLight position={[-5, 5, -5]} intensity={0.3} />

                {/* Grid and Axes */}
                <Grid
                    args={[2, 2]}
                    cellSize={0.1}
                    cellColor="#6e6e6e"
                    sectionSize={0.5}
                    sectionColor="#9e9e9e"
                    fadeDistance={5}
                />
                <axesHelper args={[0.5]} />

                {/* Robot Arm */}
                <RobotArm
                    joints={jointPositions}
                    color={LINK_COLORS}
                    onTcpUpdate={(pos) => {
                        setTcpPath(prev => {
                            const newPath = [...prev, pos];
                            return newPath.length > 100 ? newPath.slice(-100) : newPath;
                        });
                    }}
                />

                {/* Ghost Robot for IK Preview */}
                {showGhostRobot && (
                    <RobotArm
                        joints={ghostJoints}
                        color={{ ...LINK_COLORS, base: '#00ff0044', shoulder: '#00ff0044', elbow: '#00ff0044', wrist: '#00ff0044' }}
                        isGhost={true}
                    />
                )}

                {/* TCP Path Trace */}
                {tcpPath.length > 2 && (
                    <Line
                        points={tcpPath}
                        color="#ff6b6b"
                        lineWidth={2}
                    />
                )}

                {/* Coordinate Frames */}
                <CoordinateFrames activeFrame={activeFrame} />

                {/* Camera Controls */}
                <OrbitControls
                    enablePan={true}
                    enableZoom={true}
                    enableRotate={true}
                    minDistance={0.5}
                    maxDistance={3}
                />
            </Canvas>

            {/* Overlay Controls */}
            <div className="simulator-overlay">
                <FrameSelector />
                <JointDisplay />
            </div>
        </div>
    );
}

// ============================================================================
// ROBOT ARM COMPONENT
// ============================================================================

function RobotArm({ joints, color, isGhost = false, onTcpUpdate }) {
    const groupRef = useRef();
    const tcpRef = useRef();

    // Convert degrees to radians
    const q = joints.map(j => j * Math.PI / 180);

    useFrame(() => {
        if (tcpRef.current && onTcpUpdate) {
            const worldPos = new THREE.Vector3();
            tcpRef.current.getWorldPosition(worldPos);
            onTcpUpdate([worldPos.x, worldPos.y, worldPos.z]);
        }
    });

    return (
        <group ref={groupRef} position={[0, 0, 0]}>
            {/* Base */}
            <mesh position={[0, DH.L1 / 2, 0]} castShadow>
                <cylinderGeometry args={[0.08, 0.1, DH.L1, 32]} />
                <meshStandardMaterial color={color.base} transparent={isGhost} opacity={isGhost ? 0.3 : 1} />
            </mesh>

            {/* Joint 0 - Base rotation */}
            <group rotation={[0, q[0], 0]} position={[0, DH.L1, 0]}>

                {/* Joint 1 - Shoulder */}
                <group rotation={[0, 0, q[1]]}>
                    {/* Upper arm */}
                    <mesh position={[DH.L2 / 2, 0, 0]} castShadow>
                        <boxGeometry args={[DH.L2, 0.06, 0.06]} />
                        <meshStandardMaterial color={color.shoulder} transparent={isGhost} opacity={isGhost ? 0.3 : 1} />
                    </mesh>

                    {/* Joint sphere */}
                    <mesh castShadow>
                        <sphereGeometry args={[0.04, 16, 16]} />
                        <meshStandardMaterial color={color.wrist} />
                    </mesh>

                    {/* Joint 2 - Elbow */}
                    <group position={[DH.L2, 0, 0]} rotation={[0, 0, q[2]]}>
                        {/* Forearm */}
                        <mesh position={[DH.L3 / 2, 0, 0]} castShadow>
                            <boxGeometry args={[DH.L3, 0.05, 0.05]} />
                            <meshStandardMaterial color={color.elbow} transparent={isGhost} opacity={isGhost ? 0.3 : 1} />
                        </mesh>

                        {/* Joint sphere */}
                        <mesh castShadow>
                            <sphereGeometry args={[0.035, 16, 16]} />
                            <meshStandardMaterial color={color.wrist} />
                        </mesh>

                        {/* Joint 3-5 - Wrist (combined) */}
                        <group
                            position={[DH.L3, 0, 0]}
                            rotation={[q[3], q[4], q[5]]}
                            ref={tcpRef}
                        >
                            {/* Tool */}
                            <mesh position={[DH.L4 / 2, 0, 0]} castShadow>
                                <cylinderGeometry args={[0.02, 0.03, DH.L4, 16]} rotation={[0, 0, Math.PI / 2]} />
                                <meshStandardMaterial color={color.tool} transparent={isGhost} opacity={isGhost ? 0.3 : 1} />
                            </mesh>

                            {/* TCP Indicator */}
                            <mesh position={[DH.L4, 0, 0]}>
                                <sphereGeometry args={[0.015, 16, 16]} />
                                <meshStandardMaterial color="#ff0000" emissive="#ff0000" emissiveIntensity={0.5} />
                            </mesh>

                            {/* Tool Frame Axes */}
                            <axesHelper args={[0.05]} position={[DH.L4, 0, 0]} />
                        </group>
                    </group>
                </group>
            </group>
        </group>
    );
}

// ============================================================================
// COORDINATE FRAMES
// ============================================================================

function CoordinateFrames({ activeFrame }) {
    return (
        <group>
            {/* World Frame (always visible) */}
            <group position={[0, 0, 0]}>
                <axesHelper args={[0.3]} />
                <Html position={[0.32, 0, 0]}>
                    <div className="frame-label world">World</div>
                </Html>
            </group>

            {/* Base Frame */}
            <group position={[0, DH.L1, 0]}>
                <axesHelper args={[0.2]} />
                <Html position={[0.22, 0, 0]}>
                    <div className="frame-label base">Base</div>
                </Html>
            </group>
        </group>
    );
}

// ============================================================================
// OVERLAY COMPONENTS
// ============================================================================

function FrameSelector() {
    const { activeFrame, setActiveFrame } = useRobotStore();

    return (
        <div className="frame-selector">
            <label>Coordinate Frame:</label>
            <select
                value={activeFrame}
                onChange={(e) => setActiveFrame(e.target.value)}
            >
                <option value="WORLD">World</option>
                <option value="BASE">Base</option>
                <option value="TOOL">Tool</option>
            </select>
        </div>
    );
}

function JointDisplay() {
    const { jointPositions, tcpPosition } = useRobotStore();

    return (
        <div className="joint-display">
            <h4>Joint Positions</h4>
            <div className="joint-values">
                {jointPositions.map((pos, i) => (
                    <div key={i} className="joint-item">
                        <span className="joint-label">J{i}:</span>
                        <span className="joint-value">{pos.toFixed(1)}°</span>
                    </div>
                ))}
            </div>
            <h4>TCP Position</h4>
            <div className="tcp-values">
                <span>X: {(tcpPosition[0] * 1000).toFixed(1)} mm</span>
                <span>Y: {(tcpPosition[1] * 1000).toFixed(1)} mm</span>
                <span>Z: {(tcpPosition[2] * 1000).toFixed(1)} mm</span>
            </div>
        </div>
    );
}

export default RobotSimulator;
