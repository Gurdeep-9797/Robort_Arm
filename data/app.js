let ws = null;
let startTime = Date.now();

function connect() {
    const wsUrl = `ws://${location.hostname}/ws`;
    console.log('Connecting to:', wsUrl);
    
    ws = new WebSocket(wsUrl);
    
    ws.onopen = () => {
        console.log('WebSocket connected');
        updateConnectionStatus(true);
        fetchStatus();
    };
    
    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        if (data.type === 'state_update') {
            updateStateFromWebSocket(data);
        }
    };
    
    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        updateConnectionStatus(false);
    };
    
    ws.onclose = () => {
        console.log('WebSocket disconnected');
        updateConnectionStatus(false);
        setTimeout(connect, 3000);
    };
}

function updateConnectionStatus(connected) {
    const statusDot = document.getElementById('status-dot');
    const statusText = document.getElementById('status-text');
    
    if (connected) {
        statusDot.className = 'status-dot connected';
        statusText.textContent = 'Connected';
    } else {
        statusDot.className = 'status-dot disconnected';
        statusText.textContent = 'Disconnected';
    }
}

function fetchStatus() {
    fetch('/api/status')
        .then(r => r.json())
        .then(data => {
            updateUI(data);
        })
        .catch(err => console.error('Status fetch failed:', err));
}

function updateUI(data) {
    if (data.mode) {
        document.getElementById('mode').textContent = data.mode;
    }
    
    if (data.is_homed !== undefined) {
        document.getElementById('homed').textContent = data.is_homed ? 'Yes' : 'No';
    }
    
    if (data.is_moving !== undefined) {
        document.getElementById('moving').textContent = data.is_moving ? 'Yes' : 'No';
    }
    
    if (data.fault_code !== undefined) {
        document.getElementById('faults').textContent = '0x' + data.fault_code.toString(16).padStart(8, '0').toUpperCase();
    }
    
    if (data.joints && data.joints.length > 0) {
        createJointControls(data.joints);
    }
    
    if (data.cartesian) {
        document.getElementById('cart-x').textContent = data.cartesian.x.toFixed(1) + ' mm';
        document.getElementById('cart-y').textContent = data.cartesian.y.toFixed(1) + ' mm';
        document.getElementById('cart-z').textContent = data.cartesian.z.toFixed(1) + ' mm';
    }
}

function updateStateFromWebSocket(data) {
    if (data.is_moving !== undefined) {
        document.getElementById('moving').textContent = data.is_moving ? 'Yes' : 'No';
    }
    
    if (data.joints && data.joints.length > 0) {
        for (let i = 0; i < data.joints.length; i++) {
            const valueSpan = document.getElementById(`joint-${i}-value`);
            if (valueSpan) {
                valueSpan.textContent = data.joints[i].toFixed(1) + '°';
            }
            const slider = document.getElementById(`joint-${i}-slider`);
            if (slider) {
                slider.value = data.joints[i];
            }
        }
    }
}

function createJointControls(joints) {
    const container = document.getElementById('joint-controls');
    
    if (container.children.length > 0) {
        return;
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
            valueSpan.textContent = this.value + '°';
        };
        
        slider.onchange = function() {
            moveJoint(joint.id, parseFloat(this.value));
        };
        
        controlDiv.appendChild(label);
        controlDiv.appendChild(slider);
        container.appendChild(controlDiv);
    }
}

function moveJoint(joint, position) {
    fetch('/api/motion/joint', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            joint: joint,
            position: position,
            velocity: 30.0
        })
    })
    .then(r => r.json())
    .then(data => {
        if (!data.success) {
            showNotification('Error: ' + data.error, 'error');
        }
    })
    .catch(err => {
        console.error('Move joint failed:', err);
        showNotification('Move failed: ' + err.message, 'error');
    });
}

function moveCartesian(respectPlane) {
    const x = parseFloat(document.getElementById('target-x').value);
    const y = parseFloat(document.getElementById('target-y').value);
    const z = parseFloat(document.getElementById('target-z').value);
    const velocity = parseFloat(document.getElementById('cart-velocity').value);
    
    fetch('/api/motion/cartesian', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            target: {
                x: x,
                y: y,
                z: z,
                roll: 0,
                pitch: 0,
                yaw: 0
            },
            velocity: velocity,
            respect_plane: respectPlane
        })
    })
    .then(r => r.json())
    .then(data => {
        if (data.success) {
            showNotification('Moving to target position', 'success');
        } else {
            showNotification('Error: ' + data.error, 'error');
        }
    })
    .catch(err => {
        console.error('Cartesian move failed:', err);
        showNotification('Move failed: ' + err.message, 'error');
    });
}

function stopMotion() {
    fetch('/api/motion/stop', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: '{}'
    })
    .then(r => r.json())
    .then(data => {
        showNotification('Motion stopped', 'success');
    })
    .catch(err => {
        console.error('Stop failed:', err);
    });
}

function emergencyStop() {
    if (confirm('Trigger Emergency Stop?')) {
        fetch('/api/safety/estop', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: '{}'
        })
        .then(r => r.json())
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
        fetchStatus();
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
            fetchStatus();
        } else {
            showNotification('Error: ' + data.error, 'error');
        }
    })
    .catch(err => {
        console.error('Set mode failed:', err);
    });
}

function definePlane() {
    const p1 = document.getElementById('p1').value.split(',').map(parseFloat);
    const p2 = document.getElementById('p2').value.split(',').map(parseFloat);
    const p3 = document.getElementById('p3').value.split(',').map(parseFloat);
    const name = document.getElementById('plane-name').value;
    
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
            showNotification('Error: ' + data.error, 'error');
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
            document.body.removeChild(notification);
        }, 300);
    }, 3000);
}

function updateUptime() {
    const elapsed = Math.floor((Date.now() - startTime) / 1000);
    const hours = Math.floor(elapsed / 3600);
    const minutes = Math.floor((elapsed % 3600) / 60);
    const seconds = elapsed % 60;
    
    document.getElementById('uptime').textContent = 
        `${hours}h ${minutes}m ${seconds}s`;
}

setInterval(updateUptime, 1000);
setInterval(fetchStatus, 2000);

window.onload = connect;