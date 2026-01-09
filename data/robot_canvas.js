/**
 * ESP32 Industrial Robot Control - Unified Canvas
 * 
 * ARCHITECTURE:
 * - Simulator runs LOCALLY (laptop/browser) for preview and validation
 * - ESP32 receives HIGH-LEVEL commands only (target positions, not frames)
 * - WebSocket for telemetry (actual robot state from ESP32)
 * - Separation ensures zero latency in simulation, safety in execution
 * 
 * EXECUTION MODEL:
 * [Browser Simulator] → Validates motion → [ESP32 Command] → [Physical Robot]
 *                     ↑ Telemetry feedback ↑   
 */

// ============================================================================
// GLOBAL STATE & CONFIGURATION
// ============================================================================

const RobotState = {
    // Network
    ws: null,
    wsConnected: false,
    
    // Time tracking
    startTime: Date.now(),
    
    // Digital twin state
    digitalTwin: {
        simulatedPositions: [0, 0, 0, 0, 0, 0],
        actualPositions: [0, 0, 0, 0, 0, 0],
        deviations: []
    },
    
    // Robot configuration (matches ESP32 DH parameters)
    dhParameters: [
        {a: 0, alpha: 90, d: 150, theta: 0},      // Joint 0: Base rotation
        {a: 300, alpha: 0, d: 0, theta: 0},       // Joint 1: Shoulder
        {a: 250, alpha: 0, d: 0, theta: 0},       // Joint 2: Elbow
        {a: 0, alpha: 90, d: 150, theta: 0},      // Joint 3: Wrist roll
        {a: 0, alpha: -90, d: 0, theta: 0},       // Joint 4: Wrist pitch
        {a: 0, alpha: 0, d: 100, theta: 0}        // Joint 5: Wrist yaw
    ],
    
    // UI state
    currentMode: 'MODE_1_OPEN_LOOP',
    isHomed: false,
    isMoving: false,
    faultCode: 0,
    motionAllowed: true
};

// ============================================================================
// THREE.JS ROBOT SIMULATOR (LOCAL EXECUTION)
// ============================================================================

class RobotSimulator {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        if (!this.container) {
            console.error('Simulator container not found:', containerId);
            return;
        }
        
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        
        this.robot = {
            base: null,
            joints: [],
            links: [],
            tcp: null
        };
        
        // Local simulation state (independent of ESP32)
        this.simulatedPositions = [0, 0, 0, 0, 0, 0];
        this.realPositions = [0, 0, 0, 0, 0, 0];
        this.showFrames = true;
        this.showRealOverlay = false;
        
        this.stlModels = {};
        this.dhParameters = RobotState.dhParameters;
        
        this.init();
    }
    
    init() {
        // Scene setup
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1e1e1e);
        
        // Camera setup
        this.camera = new THREE.PerspectiveCamera(
            50,
            this.container.clientWidth / this.container.clientHeight,
            0.1,
            5000
        );
        this.camera.position.set(800, 600, 800);
        this.camera.lookAt(0, 0, 0);
        
        // Renderer setup
        this.renderer = new THREE.WebGLRenderer({ 
            antialias: true,
            alpha: false
        });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.container.appendChild(this.renderer.domElement);
        
        // Setup scene elements
        this.setupOrbitControls();
        this.setupLights();
        this.createGrid();
        this.createAxes();
        this.createRobot();
        
        // Event listeners
        window.addEventListener('resize', () => this.onResize());
        
        // Start animation loop (runs locally, never blocks)
        this.animate();
    }
    
    setupOrbitControls() {
        // Load OrbitControls dynamically
        const script = document.createElement('script');
        script.src = 'https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js';
        script.onload = () => {
            if (THREE.OrbitControls) {
                this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
                this.controls.enableDamping = true;
                this.controls.dampingFactor = 0.05;
                this.controls.screenSpacePanning = false;
                this.controls.minDistance = 200;
                this.controls.maxDistance = 2000;
                this.controls.maxPolarAngle = Math.PI / 2;
            }
        };
        document.head.appendChild(script);
    }
    
    setupLights() {
        // Ambient light
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);
        
        // Directional light with shadows
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(500, 800, 300);
        directionalLight.castShadow = true;
        directionalLight.shadow.camera.left = -1000;
        directionalLight.shadow.camera.right = 1000;
        directionalLight.shadow.camera.top = 1000;
        directionalLight.shadow.camera.bottom = -1000;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        this.scene.add(directionalLight);
        
        // Hemisphere light
        const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.4);
        this.scene.add(hemiLight);
    }
    
    createGrid() {
        // Grid helper
        const gridHelper = new THREE.GridHelper(1000, 20, 0x3e3e42, 0x2a2a2a);
        gridHelper.material.opacity = 0.5;
        gridHelper.material.transparent = true;
        this.scene.add(gridHelper);
        
        // Shadow receiving plane
        const planeGeometry = new THREE.PlaneGeometry(1000, 1000);
        const planeMaterial = new THREE.ShadowMaterial({ opacity: 0.2 });
        const plane = new THREE.Mesh(planeGeometry, planeMaterial);
        plane.rotation.x = -Math.PI / 2;
        plane.receiveShadow = true;
        this.scene.add(plane);
    }
    
    createAxes() {
        const axes = new THREE.AxesHelper(200);
        axes.material.linewidth = 2;
        axes.name = 'world_axes';
        this.scene.add(axes);
    }
    
    createRobot() {
        // Base
        const baseGeometry = new THREE.CylinderGeometry(40, 60, 100, 32);
        const baseMaterial = new THREE.MeshStandardMaterial({ 
            color: 0x505050,
            metalness: 0.6,
            roughness: 0.4
        });
        this.robot.base = new THREE.Mesh(baseGeometry, baseMaterial);
        this.robot.base.position.y = 50;
        this.robot.base.castShadow = true;
        this.scene.add(this.robot.base);
        
        // Create 6 joints with links
        for (let i = 0; i < 6; i++) {
            const jointGroup = new THREE.Group();
            jointGroup.name = `joint_${i}`;
            
            // Joint sphere
            const jointGeometry = new THREE.SphereGeometry(30, 16, 16);
            const jointMaterial = new THREE.MeshStandardMaterial({ 
                color: 0x4ec9b0,
                metalness: 0.7,
                roughness: 0.3
            });
            const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial);
            jointMesh.castShadow = true;
            jointGroup.add(jointMesh);
            
            // Link cylinder
            const linkLength = this.dhParameters[i].a || 150;
            const linkGeometry = new THREE.CylinderGeometry(15, 15, linkLength, 16);
            const linkMaterial = new THREE.MeshStandardMaterial({ 
                color: 0x707070,
                metalness: 0.5,
                roughness: 0.5
            });
            const linkMesh = new THREE.Mesh(linkGeometry, linkMaterial);
            linkMesh.rotation.z = Math.PI / 2;
            linkMesh.position.x = linkLength / 2;
            linkMesh.castShadow = true;
            jointGroup.add(linkMesh);
            
            // Coordinate frame
            const axesHelper = new THREE.AxesHelper(80);
            axesHelper.visible = this.showFrames;
            jointGroup.add(axesHelper);
            
            this.robot.joints.push(jointGroup);
            
            // Attach to kinematic chain
            if (i === 0) {
                this.robot.base.add(jointGroup);
                jointGroup.position.y = 50;
            } else {
                this.robot.joints[i - 1].add(jointGroup);
                const prevDH = this.dhParameters[i - 1];
                jointGroup.position.x = prevDH.a || 150;
            }
        }
        
        // Tool Center Point (TCP)
        const tcpGeometry = new THREE.ConeGeometry(20, 60, 16);
        const tcpMaterial = new THREE.MeshStandardMaterial({ 
            color: 0xff4444,
            emissive: 0x882222
        });
        this.robot.tcp = new THREE.Mesh(tcpGeometry, tcpMaterial);
        this.robot.tcp.rotation.x = Math.PI;
        this.robot.tcp.castShadow = true;
        
        if (this.robot.joints[5]) {
            this.robot.joints[5].add(this.robot.tcp);
            this.robot.tcp.position.x = this.dhParameters[5].d || 100;
        }
    }
    
    /**
     * Update robot pose (LOCAL SIMULATION ONLY)
     * @param {Array} jointAngles - Array of 6 joint angles in degrees
     * @param {boolean} isReal - If true, updates real robot overlay (from telemetry)
     */
    updateRobotPose(jointAngles, isReal = false) {
        if (isReal) {
            // Update actual positions from ESP32 telemetry
            this.realPositions = [...jointAngles];
            RobotState.digitalTwin.actualPositions = [...jointAngles];
            
            if (this.showRealOverlay) {
                this.updateDeviationDisplay();
            }
        } else {
            // Update simulated positions (local preview)
            this.simulatedPositions = [...jointAngles];
            RobotState.digitalTwin.simulatedPositions = [...jointAngles];
            
            // Apply rotations to Three.js joints
            for (let i = 0; i < Math.min(6, jointAngles.length); i++) {
                if (this.robot.joints[i]) {
                    const angleRad = (jointAngles[i] * Math.PI) / 180;
                    
                    // Apply rotation based on joint type
                    if (i === 0) {
                        this.robot.joints[i].rotation.y = angleRad;
                    } else if (i === 1 || i === 2) {
                        this.robot.joints[i].rotation.z = angleRad;
                    } else if (i === 3) {
                        this.robot.joints[i].rotation.x = angleRad;
                    } else if (i === 4) {
                        this.robot.joints[i].rotation.z = angleRad;
                    } else if (i === 5) {
                        this.robot.joints[i].rotation.x = angleRad;
                    }
                }
            }
        }
    }
    
    updateDeviationDisplay() {
        let totalDeviation = 0;
        let deviationHTML = '<div style="font-size: 11px;">';
        
        for (let i = 0; i < 6; i++) {
            const deviation = Math.abs(this.simulatedPositions[i] - this.realPositions[i]);
            totalDeviation += deviation;
            deviationHTML += `<div style="margin: 4px 0;">
                J${i}: <span style="color: ${deviation > 2 ? '#e74c3c' : '#4ec9b0'}">
                ${deviation.toFixed(2)}°
                </span>
            </div>`;
        }
        
        const avgDeviation = totalDeviation / 6;
        deviationHTML += `<div style="margin-top: 8px; padding-top: 8px; border-top: 1px solid #3e3e42;">
            <strong>Avg:</strong> <span style="color: #4ec9b0">${avgDeviation.toFixed(2)}°</span>
        </div>`;
        deviationHTML += '</div>';
        
        const deviationPanel = document.getElementById('deviation-panel');
        const deviationContent = document.getElementById('deviation-content');
        
        if (deviationPanel && deviationContent) {
            deviationPanel.style.display = 'block';
            deviationContent.innerHTML = deviationHTML;
        }
        
        // Store deviation history
        RobotState.digitalTwin.deviations.push({
            timestamp: Date.now(),
            deviation: avgDeviation
        });
        
        if (RobotState.digitalTwin.deviations.length > 100) {
            RobotState.digitalTwin.deviations.shift();
        }
    }
    
    loadSTLModel(file, jointId) {
        if (!THREE.STLLoader) {
            const script = document.createElement('script');
            script.src = 'https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js';
            script.onload = () => {
                this.loadSTLModelInternal(file, jointId);
            };
            document.head.appendChild(script);
        } else {
            this.loadSTLModelInternal(file, jointId);
        }
    }
    
    loadSTLModelInternal(file, jointId) {
        const loader = new THREE.STLLoader();
        const reader = new FileReader();
        
        reader.onload = (event) => {
            const geometry = loader.parse(event.target.result);
            
            const material = new THREE.MeshStandardMaterial({
                color: 0x606060,
                metalness: 0.6,
                roughness: 0.4
            });
            
            const mesh = new THREE.Mesh(geometry, material);
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            
            // Center geometry
            geometry.computeBoundingBox();
            const center = new THREE.Vector3();
            geometry.boundingBox.getCenter(center);
            geometry.translate(-center.x, -center.y, -center.z);
            
            // Replace existing geometry
            if (jointId === 'base') {
                this.replaceJointGeometry(this.robot.base, mesh, 0);
                this.stlModels.base = mesh;
            } else if (jointId === 'tool') {
                if (this.robot.tcp) {
                    this.replaceJointGeometry(this.robot.tcp, mesh, null);
                    this.stlModels.tool = mesh;
                }
            } else {
                const index = parseInt(jointId);
                if (this.robot.joints[index]) {
                    this.replaceJointGeometry(this.robot.joints[index], mesh, index + 1);
                    this.stlModels[`joint${index}`] = mesh;
                }
            }
            
            updateSTLList();
            showNotification(`STL loaded for ${jointId}`, 'success');
        };
        
        reader.onerror = () => {
            showNotification('Failed to read STL file', 'error');
        };
        
        reader.readAsArrayBuffer(file);
    }
    
    replaceJointGeometry(parent, newMesh, nextJointIndex) {
        const childrenToKeep = [];
        
        // Keep axes helpers and child joints
        parent.children.forEach(child => {
            if (child instanceof THREE.AxesHelper || 
                (child instanceof THREE.Group && child.name.startsWith('joint_')) ||
                (child === this.robot.tcp)) {
                childrenToKeep.push(child);
            }
        });
        
        // Clear parent
        while(parent.children.length > 0) {
            parent.remove(parent.children[0]);
        }
        
        // Add new mesh
        parent.add(newMesh);
        
        // Re-add kept children
        childrenToKeep.forEach(child => {
            parent.add(child);
        });
    }
    
    toggleFrames() {
        this.showFrames = !this.showFrames;
        
        this.robot.joints.forEach(joint => {
            joint.children.forEach(child => {
                if (child instanceof THREE.AxesHelper) {
                    child.visible = this.showFrames;
                }
            });
        });
        
        const worldAxes = this.scene.getObjectByName('world_axes');
        if (worldAxes) {
            worldAxes.visible = this.showFrames;
        }
    }
    
    resetCamera() {
        this.camera.position.set(800, 600, 800);
        this.camera.lookAt(0, 0, 0);
        if (this.controls) {
            this.controls.target.set(0, 0, 0);
            this.controls.update();
        }
    }
    
    highlightJoint(jointIndex) {
        this.robot.joints.forEach((joint, i) => {
            if (joint.children[0]) {
                if (i === jointIndex) {
                    joint.children[0].material.emissive = new THREE.Color(0x4ec9b0);
                    joint.children[0].material.emissiveIntensity = 0.5;
                } else {
                    joint.children[0].material.emissive = new THREE.Color(0x000000);
                }
            }
        });
    }
    
    onResize() {
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    }
    
    animate() {
        requestAnimationFrame(() => this.animate());
        
        if (this.controls) {
            this.controls.update();
        }
        
        this.renderer.render(this.scene, this.camera);
    }
}

// ============================================================================
// ESP32 COMMUNICATION LAYER (HIGH-LEVEL COMMANDS ONLY)
// ============================================================================

const ESP32API = {
    /**
     * Connect WebSocket for telemetry (non-blocking)
     */
    connectWebSocket() {
        const wsUrl = `ws://${location.hostname}/ws`;
        console.log('Connecting to ESP32:', wsUrl);
        
        RobotState.ws = new WebSocket(wsUrl);
        
        RobotState.ws.onopen = () => {
            console.log('WebSocket connected');
            RobotState.wsConnected = true;
            updateConnectionStatus(true);
            ESP32API.fetchStatus();
        };
        
        RobotState.ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'state_update') {
                ESP32API.handleStateUpdate(data);
            }
        };
        
        RobotState.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
            updateConnectionStatus(false);
        };
        
        RobotState.ws.onclose = () => {
            console.log('WebSocket disconnected');
            RobotState.wsConnected = false;
            updateConnectionStatus(false);
            setTimeout(() => ESP32API.connectWebSocket(), 3000);
        };
    },
    
    /**
     * Fetch full robot status from ESP32
     */
    fetchStatus() {
        fetch('/api/status')
            .then(r => r.json())
            .then(data => {
                ESP32API.updateUIFromStatus(data);
            })
            .catch(err => console.error('Status fetch failed:', err));
    },
    
    /**
     * Handle WebSocket state updates (telemetry)
     */
    handleStateUpdate(data) {
        if (data.is_moving !== undefined) {
            RobotState.isMoving = data.is_moving;
            const statusElem = document.getElementById('sim-status');
            if (statusElem) {
                statusElem.textContent = data.is_moving ? 'Moving' : 'Ready';
            }
        }
        
        if (data.joints && data.joints.length > 0) {
            // Update simulator with ACTUAL robot positions from ESP32
            if (window.simulator) {
                const positions = data.joints.map(angle => typeof angle === 'number' ? angle : 0);
                window.simulator.updateRobotPose(positions, true);
                
                // Update UI displays
                for (let i = 0; i < data.joints.length; i++) {
                    const valueSpan = document.getElementById(`joint-${i}-value`);
                    if (valueSpan) {
                        valueSpan.textContent = data.joints[i].toFixed(1) + '°';
                    }
                    
                    const treeNode = document.getElementById(`joint-tree-${i}`);
                    if (treeNode) {
                        const treeValue = treeNode.querySelector('.joint-value');
                        if (treeValue) {
                            treeValue.textContent = data.joints[i].toFixed(1) + '°';
                        }
                    }
                    
                    const slider = document.getElementById(`joint-${i}-slider`);
                    if (slider) {
                        slider.value = data.joints[i];
                    }
                }
            }
        }
    },
    
    /**
     * Update UI from full status response
     */
    updateUIFromStatus(data) {
        if (data.mode) {
            const modeText = data.mode === 'MODE_1_OPEN_LOOP' ? 'MODE 1' : 'MODE 2';
            RobotState.currentMode = data.mode;
            
            const simMode = document.getElementById('sim-mode');
            const currentMode = document.getElementById('current-mode-display');
            if (simMode) simMode.textContent = modeText;
            if (currentMode) currentMode.textContent = modeText;
        }
        
        if (data.is_homed !== undefined) {
            RobotState.isHomed = data.is_homed;
            const homedElem = document.getElementById('safety-homed');
            if (homedElem) homedElem.textContent = data.is_homed ? 'Yes' : 'No';
        }
        
        if (data.is_moving !== undefined) {
            RobotState.isMoving = data.is_moving;
            const statusElem = document.getElementById('sim-status');
            if (statusElem) statusElem.textContent = data.is_moving ? 'Moving' : 'Ready';
        }
        
        if (data.fault_code !== undefined) {
            RobotState.faultCode = data.fault_code;
            const faultsElem = document.getElementById('safety-faults');
            if (faultsElem) {
                faultsElem.textContent = '0x' + data.fault_code.toString(16).padStart(8, '0').toUpperCase();
            }
        }
        
        if (data.motion_allowed !== undefined) {
            RobotState.motionAllowed = data.motion_allowed;
            const motionElem = document.getElementById('safety-motion');
            if (motionElem) motionElem.textContent = data.motion_allowed ? 'Yes' : 'No';
        }
        
        if (data.joints && data.joints.length > 0) {
            createJointControls(data.joints);
        }
        
        if (data.cartesian) {
            const cartX = document.getElementById('cart-x');
            const cartY = document.getElementById('cart-y');
            const cartZ = document.getElementById('cart-z');
            if (cartX) cartX.textContent = data.cartesian.x.toFixed(1) + ' mm';
            if (cartY) cartY.textContent = data.cartesian.y.toFixed(1) + ' mm';
            if (cartZ) cartZ.textContent = data.cartesian.z.toFixed(1) + ' mm';
        }
    },
    
    /**
     * Send joint target to ESP32 (HIGH-LEVEL COMMAND)
     */
    moveJoint(joint, position, velocity = 30.0) {
        return fetch('/api/motion/joint', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
                joint: joint,
                position: position,
                velocity: velocity
            })
        })
        .then(r => r.json());
    },
    
    /**
     * Send cartesian target to ESP32 (HIGH-LEVEL COMMAND)
     */
    moveCartesian(target, velocity, respectPlane = false) {
        return fetch('/api/motion/cartesian', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
                target: target,
                velocity: velocity,
                respect_plane: respectPlane
            })
        })
        .then(r => r.json());
    },
    
    /**
     * Emergency stop
     */
    emergencyStop() {
        return fetch('/api/safety/estop', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: '{}'
        })
        .then(r => r.json());
    },
    
    /**
     * Stop all motion
     */
    stopMotion() {
        return fetch('/api/motion/stop', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: '{}'
        })
        .then(r => r.json());
    }
};

// ============================================================================
// UI CONTROL FUNCTIONS
// ============================================================================

function updateConnectionStatus(connected) {
    const statusDot = document.getElementById('status-dot-toolbar');
    const statusText = document.getElementById('status-text-toolbar');
    
    if (connected) {
        if (statusDot) statusDot.className = 'status-dot connected';
        if (statusText) statusText.textContent = 'Connected';
    } else {
        if (statusDot) statusDot.className = 'status-dot disconnected';
        if (statusText) statusText.textContent = 'Disconnected';
    }
}

function createJointControls(joints) {
    const container = document.getElementById('joint-controls');
    if (!container) return;
    
    if (container.children.length > 0) {
        return; // Already created
    }
    
    for (let i = 0; i < joints.length; i++) {
        const joint = joints[i];
        
        const controlDiv = document.createElement('div');
        controlDiv.className = 'joint-control';
        
        const label = document.createElement('div');
        label.className = 'joint-label';
        label.innerHTML = `
            <span class="joint-name">Joint ${joint.id}</span>
            <span class="joint-value" id="joint-${joint.id}-value">${joint.position.toFixed(1)}°</span>
        `;
        
        const slider = document.createElement('input');
        slider.type = 'range';
        slider.id = `joint-${joint.id}-slider`;
        slider.min = -180;
        slider.max = 180;
        slider.value = joint.position;
        slider.step = 1;
        slider.className = 'joint-slider';
        
        slider.oninput = function() {
            const valueSpan = document.getElementById(`joint-${joint.id}-value`);
            if (valueSpan) {
                valueSpan.textContent = this.value + '°';
            }
            
            // Update LOCAL simulator preview immediately
            if (window.simulator) {
                const newPositions = [...window.simulator.simulatedPositions];
                newPositions[joint.id] = parseFloat(this.value);
                window.simulator.updateRobotPose(newPositions, false);
            }
        };
        
        slider.onchange = function() {
            moveJoint(joint.id, parseFloat(this.value));
        };
        
        controlDiv.appendChild(label);
        controlDiv.appendChild(slider);
        container.appendChild(controlDiv);
    }
}

/**
 * Move joint (updates local sim, then sends to ESP32)
 */
function moveJoint(joint, position) {
    // STEP 1: Update local simulator FIRST (instant feedback)
    if (window.simulator) {
        const newPositions = [...window.simulator.simulatedPositions];
        newPositions[joint] = position;
        window.simulator.updateRobotPose(newPositions, false);
    }
    
    // STEP 2: Send command to ESP32 (actual hardware motion)
    ESP32API.moveJoint(joint, position)
        .then(data => {
            if (!data.success) {
                showNotification('Error: ' + (data.error || 'Motion failed'), 'error');
            }
        })
        .catch(err => {
            console.error('Move joint failed:', err);
            showNotification('Move failed: ' + err.message, 'error');
        });
}

/**
 * Move cartesian (validates locally, then sends to ESP32)
 */
function moveCartesian(respectPlane) {
    const xInput = document.getElementById('target-x');
    const yInput = document.getElementById('target-y');
    const zInput = document.getElementById('target-z');
    const velInput = document.getElementById('cart-velocity');
    
    if (!xInput || !yInput || !zInput) {
        showNotification('Cartesian inputs not found', 'error');
        return;
    }
    
    const x = parseFloat(xInput.value);
    const y = parseFloat(yInput.value);
    const z = parseFloat(zInput.value);
    const velocity = velInput ? parseFloat(velInput.value) : 50.0;
    
    const target = {
        x: x,
        y: y,
        z: z,
        roll: 0,
        pitch: 0,
        yaw: 0
    };
    
    // Send to ESP32 (IK solved on ESP32)
    ESP32API.moveCartesian(target, velocity, respectPlane)
        .then(data => {
            if (data.success) {
                showNotification('Moving to target position', 'success');
            } else {
                showNotification('Error: ' + (data.error || 'IK failed'), 'error');
            }
        })
        .catch(err => {
            console.error('Cartesian move failed:', err);
            showNotification('Move failed: ' + err.message, 'error');
        });
}

function stopMotion() {
    ESP32API.stopMotion()
        .then(data => {
            showNotification('Motion stopped', 'success');
        })
        .catch(err => {
            console.error('Stop failed:', err);
        });
}

function emergencyStop() {
    if (confirm('Trigger Emergency Stop?')) {
        ESP32API.emergencyStop()
            .then(data => {
                showNotification('EMERGENCY STOP ACTIVATED', 'error');
            })
            .catch(err => {
                console.error('E-stop failed:', err);
            });
    }
}

function clearFault() {
    fetch('/api/safety/clear', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: '{}'
    })
    .then(r => r.json())
    .then(data => {
        showNotification('Faults cleared', 'success');
        ESP32API.fetchStatus();
    })
    .catch(err => {
        console.error('Clear fault failed:', err);
    });
}

function setMode(mode) {
    const modeStr = mode === 1 ? 'MODE_1_OPEN_LOOP' : 'MODE_2_CLOSED_LOOP';
    
    fetch('/api/mode', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({mode: modeStr})
    })
    .then(r => r.json())
    .then(data => {
        if (data.success) {
            showNotification('Mode switched to ' + modeStr, 'success');
            ESP32API.fetchStatus();
        } else {
            showNotification('Error: ' + (data.error || 'Mode switch failed'), 'error');
        }
    })
    .catch(err => {
        console.error('Set mode failed:', err);
    });
}

function definePlane() {
    const p1Input = document.getElementById('p1');
    const p2Input = document.getElementById('p2');
    const p3Input = document.getElementById('p3');
    const nameInput = document.getElementById('plane-name');
    
    if (!p1Input || !p2Input || !p3Input) {
        showNotification('Plane inputs not found', 'error');
        return;
    }
    
    const p1 = p1Input.value.split(',').map(parseFloat);
    const p2 = p2Input.value.split(',').map(parseFloat);
    const p3 = p3Input.value.split(',').map(parseFloat);
    const name = nameInput ? nameInput.value : 'Plane';
    
    if (p1.length !== 3 || p2.length !== 3 || p3.length !== 3) {
        showNotification('Invalid point format. Use: x,y,z', 'error');
        return;
    }
    
    fetch('/api/plane/define', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            p1: p1,
            p2: p2,
            p3: p3,
            name: name
        })
    })
    .then(r => r.json())
    .then(data => {
        if (data.success) {
            showNotification('Plane defined: ' + name, 'success');
        } else {
            showNotification('Error: ' + (data.error || 'Plane definition failed'), 'error');
        }
    })
    .catch(err => {
        console.error('Define plane failed:', err);
    });
}

function showNotification(message, type) {
    const notification = document.createElement('div');
    notification.className = `notification notification-${type}`;
    notification.textContent = message;
    
    document.body.appendChild(notification);
    
    setTimeout(() => {
        notification.style.opacity = '0';
        setTimeout(() => {
            if (notification.parentNode) {
                document.body.removeChild(notification);
            }
        }, 300);
    }, 3000);
}

// ============================================================================
// UI HELPER FUNCTIONS
// ============================================================================

function toggleTreeNode(header) {
    const node = header.parentElement;
    const children = node.querySelector('.tree-node-children');
    const icon = header.querySelector('.tree-icon');
    
    if (node.classList.contains('expanded')) {
        node.classList.remove('expanded');
        if (children) children.style.display = 'none';
        if (icon) icon.textContent = '▶';
    } else {
        node.classList.add('expanded');
        if (children) children.style.display = 'block';
        if (icon) icon.textContent = '▼';
    }
}

function switchTab(event, tabName) {
    document.querySelectorAll('.tab-content').forEach(tab => {
        tab.style.display = 'none';
    });
    
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.classList.remove('active');
    });
    
    const targetTab = document.getElementById(`tab-${tabName}`);
    if (targetTab) {
        targetTab.style.display = 'block';
    }
    
    if (event && event.target) {
        event.target.classList.add('active');
    }
}

function updateSpeedValue(value) {
    const speedValue = document.getElementById('speed-value');
    if (speedValue) {
        speedValue.textContent = value;
    }
}

function updateAccelValue(value) {
    const accelValue = document.getElementById('accel-value');
    if (accelValue) {
        accelValue.textContent = value;
    }
}

function loadSTLDialog() {
    const dialog = document.getElementById('stl-dialog');
    if (dialog) {
        dialog.style.display = 'flex';
    }
}

function closeSTLDialog() {
    const dialog = document.getElementById('stl-dialog');
    if (dialog) {
        dialog.style.display = 'none';
    }
}

function uploadSTL() {
    const fileInput = document.getElementById('stl-file-input');
    const jointSelect = document.getElementById('stl-joint-select');
    
    if (!fileInput || !jointSelect) {
        showNotification('Upload form not found', 'error');
        return;
    }
    
    if (fileInput.files.length > 0) {
        const file = fileInput.files[0];
        const jointId = jointSelect.value;
        
        if (!file.name.toLowerCase().endsWith('.stl')) {
            showNotification('Please select an STL file', 'error');
            return;
        }
        
        if (window.simulator) {
            window.simulator.loadSTLModel(file, jointId);
            closeSTLDialog();
            fileInput.value = '';
        } else {
            showNotification('Simulator not ready', 'warning');
        }
    } else {
        showNotification('Please select an STL file', 'error');
    }
}

function updateSTLList() {
    const stlList = document.getElementById('stl-list');
    if (stlList && window.simulator) {
        stlList.innerHTML = '';
        
        const modelKeys = Object.keys(window.simulator.stlModels);
        if (modelKeys.length === 0) {
            stlList.innerHTML = '<div class="tree-leaf empty">No STL loaded</div>';
        } else {
            modelKeys.forEach(key => {
                const leaf = document.createElement('div');
                leaf.className = 'tree-leaf';
                leaf.innerHTML = `📦 ${key}`;
                stlList.appendChild(leaf);
            });
        }
    }
}

function toggleFrames() {
    if (window.simulator) {
        window.simulator.toggleFrames();
    }
}

function resetCamera() {
    if (window.simulator) {
        window.simulator.resetCamera();
    }
}

function playSimulation() {
    showNotification('Simulation playback started', 'success');
}

function pauseSimulation() {
    showNotification('Simulation paused', 'warning');
}

function stopSimulation() {
    if (window.simulator) {
        window.simulator.updateRobotPose([0, 0, 0, 0, 0, 0], false);
    }
    showNotification('Simulation stopped', 'warning');
}

/**
 * Jog joint (local preview + ESP32 command)
 */
function jogJoint(direction) {
    const jointSelect = document.getElementById('jog-joint-select');
    if (!jointSelect) return;
    
    const joint = parseInt(jointSelect.value);
    const speedSlider = document.getElementById('speed-slider');
    const speed = speedSlider ? parseFloat(speedSlider.value) : 30;
    
    if (!window.simulator) {
        showNotification('Simulator not ready', 'warning');
        return;
    }
    
    // Update local simulation first
    const currentPos = window.simulator.simulatedPositions[joint] || 0;
    const newPos = currentPos + (direction * 5);
    
    window.simulator.simulatedPositions[joint] = newPos;
    window.simulator.updateRobotPose(window.simulator.simulatedPositions, false);
    window.simulator.highlightJoint(joint);
    
    updateTreeJointValues();
    
    // Send to ESP32
    ESP32API.moveJoint(joint, newPos, speed)
        .then(data => {
            if (!data.success) {
                showNotification('Error: ' + (data.error || 'Motion failed'), 'error');
            }
        })
        .catch(err => {
            console.error('Jog failed:', err);
            showNotification('Network error', 'error');
        });
}

function moveCartesianFromPanel() {
    moveCartesian(false);
}

function updateTreeJointValues() {
    for (let i = 0; i < 6; i++) {
        const treeNode = document.getElementById(`joint-tree-${i}`);
        if (treeNode && window.simulator) {
            const valueSpan = treeNode.querySelector('.joint-value');
            if (valueSpan) {
                const value = window.simulator.simulatedPositions[i] || 0;
                valueSpan.textContent = value.toFixed(1) + '°';
            }
        }
    }
}

function updateLimits() {
    const jointSelect = document.getElementById('config-joint-select');
    const minInput = document.getElementById('limit-min');
    const maxInput = document.getElementById('limit-max');
    
    if (!jointSelect || !minInput || !maxInput) {
        showNotification('Configuration inputs not found', 'error');
        return;
    }
    
    const joint = parseInt(jointSelect.value);
    const min = parseFloat(minInput.value);
    const max = parseFloat(maxInput.value);
    
    if (isNaN(min) || isNaN(max) || min >= max) {
        showNotification('Invalid limit values', 'error');
        return;
    }
    
    fetch('/api/config/joint/' + joint + '/limits', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ min: min, max: max })
    })
    .then(r => r.json())
    .then(data => {
        if (data.success) {
            showNotification(`Limits updated for Joint ${joint}`, 'success');
        } else {
            showNotification('Failed to update limits', 'error');
        }
    })
    .catch(err => {
        console.error('Update limits failed:', err);
        showNotification('Network error', 'error');
    });
}

function saveConfigToESP32() {
    fetch('/api/config/save', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: '{}'
    })
    .then(r => r.json())
    .then(data => {
        if (data.success) {
            showNotification('Configuration saved to ESP32', 'success');
        } else {
            showNotification('Failed to save configuration', 'error');
        }
    })
    .catch(err => {
        console.error('Save failed:', err);
        showNotification('Network error', 'error');
    });
}

function updateUptime() {
    const uptimeElem = document.getElementById('uptime');
    if (uptimeElem) {
        const elapsed = Math.floor((Date.now() - RobotState.startTime) / 1000);
        const hours = Math.floor(elapsed / 3600);
        const minutes = Math.floor((elapsed % 3600) / 60);
        const seconds = elapsed % 60;
        
        uptimeElem.textContent = `${hours}h ${minutes}m ${seconds}s`;
    }
}

function fetchTelemetry() {
    fetch('/api/telemetry')
        .then(r => r.json())
        .then(data => {
            if (data.joints && window.simulator) {
                const realPositions = data.joints.map(j => j.pos || 0);
                window.simulator.updateRobotPose(realPositions, true);
            }
            
            if (data.mode !== undefined) {
                const modeElem = document.getElementById('sim-mode');
                if (modeElem) {
                    modeElem.textContent = data.mode === 1 ? 'MODE 1' : 'MODE 2';
                }
            }
            
            if (data.is_moving !== undefined) {
                const statusElem = document.getElementById('sim-status');
                if (statusElem) {
                    statusElem.textContent = data.is_moving ? 'Moving' : 'Ready';
                }
            }
        })
        .catch(err => {
            // Silently fail - telemetry is non-critical
        });
}

function exportSession() {
    fetch('/api/session/export')
        .then(r => r.json())
        .then(data => {
            const blob = new Blob([JSON.stringify(data, null, 2)], {type: 'application/json'});
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `session_${Date.now()}.json`;
            a.click();
            showNotification('Session exported', 'success');
        })
        .catch(err => {
            console.error('Export failed:', err);
            showNotification('Export failed', 'error');
        });
}

function resetSession() {
    if (confirm('Reset session data?')) {
        fetch('/api/session/reset', {method: 'POST'})
            .then(r => r.json())
            .then(data => {
                showNotification('Session reset', 'success');
                RobotState.digitalTwin.deviations = [];
            })
            .catch(err => {
                console.error('Reset failed:', err);
            });
    }
}

// ============================================================================
// APPLICATION BOOTSTRAP
// ============================================================================

window.addEventListener('load', () => {
    console.log('ESP32 Robot Control - Initializing...');
    
    // Initialize simulator (runs locally, never blocks)
    setTimeout(() => {
        const container = document.getElementById('simulation-container');
        if (container) {
            window.simulator = new RobotSimulator('simulation-container');
            console.log('✓ Simulator initialized (local execution)');
        } else {
            console.warn('Simulation container not found');
        }
    }, 100);
    
    // Connect to ESP32 WebSocket (non-blocking)
    ESP32API.connectWebSocket();
    console.log('✓ Connecting to ESP32...');
    
    // Start periodic updates
    setInterval(updateUptime, 1000);
    setInterval(() => ESP32API.fetchStatus(), 2000);
    setInterval(fetchTelemetry, 500);
    
    console.log('✓ Application ready');
});

// ============================================================================
// EXPORT FOR DEBUGGING
// ============================================================================

window.RobotDebug = {
    state: RobotState,
    api: ESP32API,
    simulator: () => window.simulator
};