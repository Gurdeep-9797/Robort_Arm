/**
 * Connections Graph
 * 
 * React Flow based system architecture visualization:
 * - Real-time node status from WebSocket
 * - Animated data flow edges
 * - Fault propagation highlighting
 */

import React, { useCallback, useMemo } from 'react';
import ReactFlow, {
    Background,
    Controls,
    MiniMap,
    useNodesState,
    useEdgesState,
    MarkerType,
} from 'reactflow';
import 'reactflow/dist/style.css';
import useRobotStore from '../stores/robotStore';

// ============================================================================
// INITIAL NODE POSITIONS
// ============================================================================

const INITIAL_NODES = [
    // Row 1: UI Layer
    { id: 'ui', type: 'customNode', position: { x: 400, y: 0 }, data: { label: 'UI / Simulator', icon: '🖥️', type: 'UI_SIMULATOR' } },

    // Row 2: Communication
    { id: 'ws', type: 'customNode', position: { x: 400, y: 100 }, data: { label: 'WebSocket API', icon: '🔌', type: 'WEBSOCKET_API' } },

    // Row 3: Core
    { id: 'core', type: 'customNode', position: { x: 400, y: 200 }, data: { label: 'RobotCore', icon: '🤖', type: 'ROBOT_CORE' } },

    // Row 4: Managers
    { id: 'ik_mgr', type: 'customNode', position: { x: 200, y: 300 }, data: { label: 'IK Manager', icon: '🎯', type: 'IK_MANAGER' } },
    { id: 'motor_mgr', type: 'customNode', position: { x: 400, y: 300 }, data: { label: 'Motor Manager', icon: '⚙️', type: 'MOTOR_MANAGER' } },
    { id: 'safety', type: 'customNode', position: { x: 600, y: 300 }, data: { label: 'Safety Manager', icon: '🛡️', type: 'SAFETY_MANAGER' } },

    // Row 5: Solvers & Controllers
    { id: 'ik_solver', type: 'customNode', position: { x: 100, y: 400 }, data: { label: 'Active IK Solver', icon: '📐', type: 'IK_SOLVER' } },
    { id: 'servo_ctrl', type: 'customNode', position: { x: 300, y: 400 }, data: { label: 'Servo Controller', icon: '🔧', type: 'SERVO_CONTROLLER' } },
    { id: 'dc_ctrl', type: 'customNode', position: { x: 500, y: 400 }, data: { label: 'DC Controller', icon: '🔩', type: 'DC_CONTROLLER' } },
    { id: 'align', type: 'customNode', position: { x: 700, y: 400 }, data: { label: 'Alignment Checker', icon: '📏', type: 'ALIGNMENT_CHECKER' } },

    // Row 6: Hardware
    { id: 'encoders', type: 'customNode', position: { x: 500, y: 500 }, data: { label: 'Encoders', icon: '📊', type: 'ENCODERS' } },
];

const INITIAL_EDGES = [
    // UI → WebSocket
    { id: 'e-ui-ws', source: 'ui', target: 'ws', type: 'smoothstep', animated: true, markerEnd: { type: MarkerType.ArrowClosed }, data: { type: 'COMMAND' } },
    { id: 'e-ws-ui', source: 'ws', target: 'ui', type: 'smoothstep', animated: true, style: { stroke: '#4ecdc4' }, data: { type: 'FEEDBACK' } },

    // WebSocket → Core
    { id: 'e-ws-core', source: 'ws', target: 'core', type: 'smoothstep', animated: true, markerEnd: { type: MarkerType.ArrowClosed }, data: { type: 'COMMAND' } },
    { id: 'e-core-ws', source: 'core', target: 'ws', type: 'smoothstep', animated: true, style: { stroke: '#4ecdc4' }, data: { type: 'FEEDBACK' } },

    // Core → Managers
    { id: 'e-core-ik', source: 'core', target: 'ik_mgr', type: 'smoothstep', animated: true, markerEnd: { type: MarkerType.ArrowClosed }, data: { type: 'COMMAND' } },
    { id: 'e-ik-motor', source: 'ik_mgr', target: 'motor_mgr', type: 'smoothstep', animated: true, markerEnd: { type: MarkerType.ArrowClosed }, data: { type: 'COMMAND' } },

    // IK Manager → Solver
    { id: 'e-ik-solver', source: 'ik_mgr', target: 'ik_solver', type: 'smoothstep', animated: true, data: { type: 'COMMAND' } },
    { id: 'e-solver-ik', source: 'ik_solver', target: 'ik_mgr', type: 'smoothstep', style: { stroke: '#4ecdc4' }, data: { type: 'FEEDBACK' } },

    // Motor Manager → Controllers
    { id: 'e-motor-servo', source: 'motor_mgr', target: 'servo_ctrl', type: 'smoothstep', animated: true, data: { type: 'COMMAND' } },
    { id: 'e-motor-dc', source: 'motor_mgr', target: 'dc_ctrl', type: 'smoothstep', animated: true, data: { type: 'COMMAND' } },

    // Encoders → Alignment
    { id: 'e-enc-align', source: 'encoders', target: 'align', type: 'smoothstep', style: { stroke: '#4ecdc4' }, data: { type: 'FEEDBACK' } },
    { id: 'e-motor-align', source: 'motor_mgr', target: 'align', type: 'smoothstep', style: { stroke: '#f9c74f' }, data: { type: 'CONFIG' } },

    // Alignment → Safety
    { id: 'e-align-safety', source: 'align', target: 'safety', type: 'smoothstep', style: { stroke: '#4ecdc4' }, data: { type: 'FEEDBACK' } },

    // Safety Gates (dashed)
    { id: 'e-safety-motor', source: 'safety', target: 'motor_mgr', type: 'smoothstep', style: { stroke: '#e63946', strokeDasharray: '5,5' }, markerEnd: { type: MarkerType.ArrowClosed }, data: { type: 'SAFETY_GATE' } },
    { id: 'e-safety-core', source: 'safety', target: 'core', type: 'smoothstep', style: { stroke: '#e63946', strokeDasharray: '5,5' }, markerEnd: { type: MarkerType.ArrowClosed }, data: { type: 'SAFETY_GATE' } },
];

// ============================================================================
// CUSTOM NODE COMPONENT
// ============================================================================

const CustomNode = ({ data, selected }) => {
    const status = data.status || 'IDLE';

    const statusColors = {
        IDLE: '#9e9e9e',
        ACTIVE: '#4caf50',
        WARNING: '#ff9800',
        FAULT: '#f44336',
        DISABLED: '#616161',
    };

    return (
        <div
            className={`graph-node ${status.toLowerCase()} ${selected ? 'selected' : ''}`}
            style={{ borderColor: statusColors[status] }}
        >
            <div className="node-icon">{data.icon}</div>
            <div className="node-label">{data.label}</div>
            <div className="node-status" style={{ backgroundColor: statusColors[status] }}></div>
            {data.value && <div className="node-value">{data.value}</div>}
        </div>
    );
};

const nodeTypes = {
    customNode: CustomNode,
};

// ============================================================================
// MAIN COMPONENT
// ============================================================================

function ConnectionsGraph() {
    const {
        isMoving, faultCode, aligned, motorType, activeIKSolver,
        connected, simulatorMode, alignmentCritical,
    } = useRobotStore();

    // Calculate node statuses
    const nodes = useMemo(() => {
        return INITIAL_NODES.map(node => {
            let status = 'IDLE';
            let value = null;

            switch (node.data.type) {
                case 'UI_SIMULATOR':
                    status = connected ? 'ACTIVE' : 'FAULT';
                    value = simulatorMode ? 'SIM' : 'HW';
                    break;
                case 'WEBSOCKET_API':
                    status = connected ? 'ACTIVE' : 'FAULT';
                    break;
                case 'ROBOT_CORE':
                    status = isMoving ? 'ACTIVE' : 'IDLE';
                    break;
                case 'IK_MANAGER':
                    status = 'ACTIVE';
                    value = activeIKSolver;
                    break;
                case 'MOTOR_MANAGER':
                    status = isMoving ? 'ACTIVE' : 'IDLE';
                    value = motorType;
                    break;
                case 'SAFETY_MANAGER':
                    status = faultCode !== 0 ? 'FAULT' : 'ACTIVE';
                    break;
                case 'ALIGNMENT_CHECKER':
                    status = alignmentCritical ? 'FAULT' : (aligned ? 'ACTIVE' : 'WARNING');
                    break;
                case 'SERVO_CONTROLLER':
                    status = motorType === 'SERVO' ? 'ACTIVE' : 'DISABLED';
                    break;
                case 'DC_CONTROLLER':
                    status = motorType === 'DC_ENCODER' ? 'ACTIVE' : 'DISABLED';
                    break;
                default:
                    status = 'IDLE';
            }

            return {
                ...node,
                data: { ...node.data, status, value },
            };
        });
    }, [isMoving, faultCode, aligned, motorType, activeIKSolver, connected, simulatorMode, alignmentCritical]);

    // Calculate edge statuses
    const edges = useMemo(() => {
        return INITIAL_EDGES.map(edge => {
            let animated = edge.data?.type === 'COMMAND' && isMoving;
            let style = { ...edge.style };

            // Highlight faulted paths
            if (faultCode !== 0 && edge.data?.type === 'SAFETY_GATE') {
                style.stroke = '#f44336';
                style.strokeWidth = 3;
                animated = true;
            }

            return { ...edge, animated, style };
        });
    }, [isMoving, faultCode]);

    const [nodeState, setNodes, onNodesChange] = useNodesState(nodes);
    const [edgeState, setEdges, onEdgesChange] = useEdgesState(edges);

    // Update nodes when status changes
    React.useEffect(() => {
        setNodes(nodes);
        setEdges(edges);
    }, [nodes, edges, setNodes, setEdges]);

    const onNodeClick = useCallback((_, node) => {
        useRobotStore.getState().setSelectedNode(node.id);
    }, []);

    return (
        <div className="connections-graph">
            <ReactFlow
                nodes={nodeState}
                edges={edgeState}
                onNodesChange={onNodesChange}
                onEdgesChange={onEdgesChange}
                onNodeClick={onNodeClick}
                nodeTypes={nodeTypes}
                fitView
                attributionPosition="bottom-left"
            >
                <Background color="#333" gap={16} />
                <Controls />
                <MiniMap
                    nodeColor={(node) => {
                        const status = node.data?.status || 'IDLE';
                        const colors = { IDLE: '#9e9e9e', ACTIVE: '#4caf50', WARNING: '#ff9800', FAULT: '#f44336', DISABLED: '#616161' };
                        return colors[status];
                    }}
                />
            </ReactFlow>

            {/* Legend */}
            <div className="graph-legend">
                <h4>Legend</h4>
                <div className="legend-item"><span className="dot active"></span> Active</div>
                <div className="legend-item"><span className="dot warning"></span> Warning</div>
                <div className="legend-item"><span className="dot fault"></span> Fault</div>
                <div className="legend-item"><span className="dot disabled"></span> Disabled</div>
                <div className="legend-item"><span className="line command"></span> Command</div>
                <div className="legend-item"><span className="line feedback"></span> Feedback</div>
                <div className="legend-item"><span className="line safety"></span> Safety Gate</div>
            </div>
        </div>
    );
}

export default ConnectionsGraph;
