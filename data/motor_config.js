/**
 * Motor Configuration UI Module
 * 
 * Provides UI controls for:
 * - Global motor type selection (Servo/DC)
 * - Per-axis motor configuration
 * - Safety warnings for unsafe configurations
 */

// ============================================================================
// MOTOR STATE
// ============================================================================

const MotorState = {
    activeType: 'SERVO',
    motorTypes: [
        { id: 'SERVO', name: 'Servo Motors', desc: 'PWM servo control via PCA9685' },
        { id: 'DC_ENCODER', name: 'DC Motors', desc: 'Encoder feedback with PID control' }
    ],
    axisConfigs: [],
    warnings: []
};

// ============================================================================
// MOTOR CONFIGURATION PANEL
// ============================================================================

function createMotorConfigPanel() {
    return `
        <div class="motor-config-panel">
            <h4>Motor Configuration</h4>
            
            <div class="motor-type-section">
                <label>Global Motor Type</label>
                <div class="motor-type-selector">
                    ${MotorState.motorTypes.map(t => `
                        <button class="motor-type-btn ${t.id === MotorState.activeType ? 'active' : ''}" 
                                onclick="onMotorTypeChange('${t.id}')"
                                data-type="${t.id}">
                            <span class="motor-icon">${t.id === 'SERVO' ? '⚙️' : '🔄'}</span>
                            <span class="motor-name">${t.name}</span>
                            <span class="motor-desc">${t.desc}</span>
                        </button>
                    `).join('')}
                </div>
            </div>
            
            <div class="motor-warnings" id="motor-warnings" style="display: none;">
                <div class="warning-banner">
                    ⚠️ Motor type change requires motion stop
                </div>
            </div>
            
            <div class="axis-config-section">
                <h5>Per-Axis Configuration</h5>
                <div class="axis-selector">
                    <label>Select Axis:</label>
                    <select id="axis-select" onchange="onAxisSelect(this.value)">
                        ${Array.from({ length: 6 }, (_, i) =>
        `<option value="${i}">Joint ${i}</option>`
    ).join('')}
                    </select>
                </div>
                
                <div class="axis-config-form" id="axis-config-form">
                    ${createAxisConfigForm(0)}
                </div>
            </div>
            
            <div class="config-actions">
                <button class="btn-secondary" onclick="loadMotorConfig()">
                    Reset to Saved
                </button>
                <button class="btn-primary" onclick="saveMotorConfig()">
                    💾 Save Configuration
                </button>
            </div>
        </div>
    `;
}

// ============================================================================
// AXIS CONFIGURATION FORM
// ============================================================================

function createAxisConfigForm(axisIndex) {
    const config = MotorState.axisConfigs[axisIndex] || getDefaultAxisConfig();
    const isServo = MotorState.activeType === 'SERVO';

    return `
        <div class="axis-form">
            <div class="form-row">
                <label>Hardware Type</label>
                <select id="hw-type-${axisIndex}" onchange="onAxisConfigChange(${axisIndex})">
                    <option value="STANDARD_SERVO" ${config.hardwareType === 'STANDARD_SERVO' ? 'selected' : ''}>Standard Servo</option>
                    <option value="DIGITAL_SERVO" ${config.hardwareType === 'DIGITAL_SERVO' ? 'selected' : ''}>Digital Servo</option>
                    <option value="BRUSHED_DC" ${config.hardwareType === 'BRUSHED_DC' ? 'selected' : ''}>Brushed DC</option>
                    <option value="BRUSHLESS_DC" ${config.hardwareType === 'BRUSHLESS_DC' ? 'selected' : ''}>Brushless DC</option>
                </select>
            </div>
            
            <div class="form-row">
                <label>Encoder Type</label>
                <select id="enc-type-${axisIndex}" onchange="onAxisConfigChange(${axisIndex})">
                    <option value="NONE" ${config.encoderType === 'NONE' ? 'selected' : ''}>None (Open Loop)</option>
                    <option value="INCREMENTAL" ${config.encoderType === 'INCREMENTAL' ? 'selected' : ''}>Incremental</option>
                    <option value="ABSOLUTE" ${config.encoderType === 'ABSOLUTE' ? 'selected' : ''}>Absolute</option>
                    <option value="POTENTIOMETER" ${config.encoderType === 'POTENTIOMETER' ? 'selected' : ''}>Potentiometer</option>
                </select>
            </div>
            
            <div class="form-row">
                <label>Gear Ratio</label>
                <input type="number" id="gear-ratio-${axisIndex}" 
                       value="${config.gearRatio}" 
                       min="0.1" max="500" step="0.1"
                       onchange="onAxisConfigChange(${axisIndex})">
            </div>
            
            ${isServo ? `
                <div class="form-row servo-only">
                    <label>Min Pulse (μs)</label>
                    <input type="number" id="pulse-min-${axisIndex}" 
                           value="${config.servoMinPulse}" 
                           min="100" max="3000" step="10"
                           onchange="onAxisConfigChange(${axisIndex})">
                </div>
                
                <div class="form-row servo-only">
                    <label>Max Pulse (μs)</label>
                    <input type="number" id="pulse-max-${axisIndex}" 
                           value="${config.servoMaxPulse}" 
                           min="100" max="3000" step="10"
                           onchange="onAxisConfigChange(${axisIndex})">
                </div>
            ` : `
                <div class="form-row dc-only">
                    <label>Max Current (A)</label>
                    <input type="number" id="max-current-${axisIndex}" 
                           value="${config.maxCurrent}" 
                           min="0.1" max="20" step="0.1"
                           onchange="onAxisConfigChange(${axisIndex})">
                </div>
                
                <div class="form-row dc-only">
                    <label>Position PID (Kp)</label>
                    <input type="number" id="pid-kp-${axisIndex}" 
                           value="${config.positionKp}" 
                           min="0" max="100" step="0.1"
                           onchange="onAxisConfigChange(${axisIndex})">
                </div>
                
                <div class="form-row dc-only">
                    <label>Position PID (Ki)</label>
                    <input type="number" id="pid-ki-${axisIndex}" 
                           value="${config.positionKi}" 
                           min="0" max="10" step="0.01"
                           onchange="onAxisConfigChange(${axisIndex})">
                </div>
                
                <div class="form-row dc-only">
                    <label>Position PID (Kd)</label>
                    <input type="number" id="pid-kd-${axisIndex}" 
                           value="${config.positionKd}" 
                           min="0" max="10" step="0.01"
                           onchange="onAxisConfigChange(${axisIndex})">
                </div>
            `}
            
            <div class="form-row">
                <label>
                    <input type="checkbox" id="invert-dir-${axisIndex}" 
                           ${config.invertDirection ? 'checked' : ''}
                           onchange="onAxisConfigChange(${axisIndex})">
                    Invert Direction
                </label>
            </div>
            
            <div class="axis-warnings" id="axis-warnings-${axisIndex}"></div>
        </div>
    `;
}

function getDefaultAxisConfig() {
    return {
        hardwareType: 'STANDARD_SERVO',
        encoderType: 'NONE',
        gearRatio: 1.0,
        servoMinPulse: 500,
        servoMaxPulse: 2500,
        maxCurrent: 2.0,
        positionKp: 2.0,
        positionKi: 0.1,
        positionKd: 0.05,
        invertDirection: false
    };
}

// ============================================================================
// EVENT HANDLERS
// ============================================================================

function onMotorTypeChange(motorType) {
    // Check if motion is in progress
    if (RobotState && RobotState.isMoving) {
        showNotification('Cannot change motor type while moving', 'error');
        return;
    }

    const previousType = MotorState.activeType;
    MotorState.activeType = motorType;

    // Update UI
    document.querySelectorAll('.motor-type-btn').forEach(btn => {
        btn.classList.remove('active');
        if (btn.dataset.type === motorType) {
            btn.classList.add('active');
        }
    });

    // Regenerate axis form for new motor type
    const axisSelect = document.getElementById('axis-select');
    const axisForm = document.getElementById('axis-config-form');
    if (axisSelect && axisForm) {
        axisForm.innerHTML = createAxisConfigForm(parseInt(axisSelect.value));
    }

    // Send to ESP32
    fetch('/api/motors/configure', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ motor_type: motorType })
    })
        .then(r => r.json())
        .then(data => {
            if (data.success) {
                showNotification(`Motor type changed to ${motorType}`, 'success');
            } else {
                // Revert on failure
                MotorState.activeType = previousType;
                showNotification(data.error || 'Failed to change motor type', 'error');
            }
        })
        .catch(err => {
            MotorState.activeType = previousType;
            showNotification('Motor configuration failed', 'error');
        });
}

function onAxisSelect(axisIndex) {
    const axisForm = document.getElementById('axis-config-form');
    if (axisForm) {
        axisForm.innerHTML = createAxisConfigForm(parseInt(axisIndex));
    }
}

function onAxisConfigChange(axisIndex) {
    // Read current form values
    const config = {
        hardwareType: document.getElementById(`hw-type-${axisIndex}`)?.value || 'STANDARD_SERVO',
        encoderType: document.getElementById(`enc-type-${axisIndex}`)?.value || 'NONE',
        gearRatio: parseFloat(document.getElementById(`gear-ratio-${axisIndex}`)?.value) || 1.0,
        servoMinPulse: parseInt(document.getElementById(`pulse-min-${axisIndex}`)?.value) || 500,
        servoMaxPulse: parseInt(document.getElementById(`pulse-max-${axisIndex}`)?.value) || 2500,
        maxCurrent: parseFloat(document.getElementById(`max-current-${axisIndex}`)?.value) || 2.0,
        positionKp: parseFloat(document.getElementById(`pid-kp-${axisIndex}`)?.value) || 2.0,
        positionKi: parseFloat(document.getElementById(`pid-ki-${axisIndex}`)?.value) || 0.1,
        positionKd: parseFloat(document.getElementById(`pid-kd-${axisIndex}`)?.value) || 0.05,
        invertDirection: document.getElementById(`invert-dir-${axisIndex}`)?.checked || false
    };

    // Store locally
    MotorState.axisConfigs[axisIndex] = config;

    // Validate and show warnings
    validateAxisConfig(axisIndex, config);
}

function validateAxisConfig(axisIndex, config) {
    const warningsDiv = document.getElementById(`axis-warnings-${axisIndex}`);
    if (!warningsDiv) return;

    const warnings = [];

    // Check gear ratio
    if (config.gearRatio > 100) {
        warnings.push('⚠️ High gear ratio may cause slow response');
    }
    if (config.gearRatio < 0.5) {
        warnings.push('⚠️ Low gear ratio may cause position instability');
    }

    // Check current limit
    if (config.maxCurrent > 5) {
        warnings.push('⚠️ High current limit - ensure motor can handle');
    }

    // Check servo pulse width
    if (config.servoMinPulse >= config.servoMaxPulse) {
        warnings.push('❌ Min pulse must be less than max pulse');
    }

    // Check PID gains
    if (config.positionKp > 10) {
        warnings.push('⚠️ High Kp may cause oscillation');
    }

    // Display warnings
    if (warnings.length > 0) {
        warningsDiv.innerHTML = warnings.map(w =>
            `<div class="axis-warning">${w}</div>`
        ).join('');
        warningsDiv.style.display = 'block';
    } else {
        warningsDiv.style.display = 'none';
    }
}

// ============================================================================
// SAVE/LOAD CONFIGURATION
// ============================================================================

function saveMotorConfig() {
    const axisSelect = document.getElementById('axis-select');
    const axisIndex = parseInt(axisSelect?.value) || 0;

    const config = MotorState.axisConfigs[axisIndex];
    if (!config) {
        showNotification('No configuration to save', 'warning');
        return;
    }

    fetch('/api/motors/axis/configure', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            axis: axisIndex,
            config: config
        })
    })
        .then(r => r.json())
        .then(data => {
            if (data.success) {
                showNotification(`Axis ${axisIndex} configuration saved`, 'success');
            } else {
                showNotification(data.error || 'Save failed', 'error');
            }
        })
        .catch(err => {
            showNotification('Save failed: ' + err.message, 'error');
        });
}

function loadMotorConfig() {
    fetch('/api/motors/config')
        .then(r => r.json())
        .then(data => {
            if (data.motor_type) {
                MotorState.activeType = data.motor_type;
            }
            if (data.axis_configs) {
                MotorState.axisConfigs = data.axis_configs;
            }

            // Refresh UI
            const axisSelect = document.getElementById('axis-select');
            const axisForm = document.getElementById('axis-config-form');
            if (axisSelect && axisForm) {
                axisForm.innerHTML = createAxisConfigForm(parseInt(axisSelect.value));
            }

            showNotification('Configuration loaded', 'success');
        })
        .catch(err => {
            showNotification('Failed to load configuration', 'error');
        });
}

// ============================================================================
// INITIALIZATION
// ============================================================================

function initMotorConfig() {
    // Initialize with 6 default configs
    for (let i = 0; i < 6; i++) {
        MotorState.axisConfigs[i] = getDefaultAxisConfig();
    }

    // Load actual config from ESP32
    loadMotorConfig();
}

// Initialize when ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initMotorConfig);
} else {
    initMotorConfig();
}
