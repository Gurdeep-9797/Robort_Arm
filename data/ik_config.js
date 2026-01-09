/**
 * IK Configuration UI Module
 * 
 * Provides UI controls for:
 * - IK solver type selection
 * - Preference selection (speed/accuracy/stability)
 * - Live IK preview with reachability
 * - Singularity warnings
 * - Joint limit visualization
 */

// ============================================================================
// IK STATE
// ============================================================================

const IKState = {
    activeSolver: 'NUMERICAL',
    preference: 'ACCURACY',
    previewEnabled: true,
    lastResult: null,
    solverTypes: [
        { id: 'ANALYTICAL', name: 'Analytical', desc: 'Fast closed-form solution' },
        { id: 'NUMERICAL', name: 'Numerical', desc: 'Iterative Jacobian-based' },
        { id: 'SERVO_FRIENDLY', name: 'Servo-Friendly', desc: 'Position-dominant' },
        { id: 'DC_FRIENDLY', name: 'DC-Friendly', desc: 'Velocity/torque aware' }
    ],
    preferences: [
        { id: 'SPEED', name: 'Speed', desc: 'Minimize computation time' },
        { id: 'ACCURACY', name: 'Accuracy', desc: 'Minimize position error' },
        { id: 'STABILITY', name: 'Stability', desc: 'Avoid singularities' }
    ]
};

// ============================================================================
// IK CONFIGURATION PANEL CREATION
// ============================================================================

function createIKConfigPanel() {
    return `
        <div class="ik-config-panel">
            <h4>Inverse Kinematics Configuration</h4>
            
            <div class="config-section">
                <label>IK Solver Type</label>
                <select id="ik-solver-select" onchange="onIKSolverChange(this.value)">
                    ${IKState.solverTypes.map(s => 
                        `<option value="${s.id}" ${s.id === IKState.activeSolver ? 'selected' : ''}>
                            ${s.name}
                        </option>`
                    ).join('')}
                </select>
                <span class="solver-desc" id="solver-desc">
                    ${IKState.solverTypes.find(s => s.id === IKState.activeSolver)?.desc || ''}
                </span>
            </div>
            
            <div class="config-section">
                <label>Optimization Preference</label>
                <div class="preference-buttons">
                    ${IKState.preferences.map(p => `
                        <button class="pref-btn ${p.id === IKState.preference ? 'active' : ''}" 
                                onclick="onPreferenceChange('${p.id}')" 
                                title="${p.desc}">
                            ${p.name}
                        </button>
                    `).join('')}
                </div>
            </div>
            
            <div class="config-section">
                <label>
                    <input type="checkbox" id="ik-preview-toggle" 
                           ${IKState.previewEnabled ? 'checked' : ''} 
                           onchange="onPreviewToggle(this.checked)">
                    Live IK Preview
                </label>
            </div>
            
            <div class="ik-preview-panel" id="ik-preview-panel">
                <h5>IK Preview Result</h5>
                <div id="ik-preview-content">
                    <div class="ik-status idle">Enter target to preview</div>
                </div>
            </div>
            
            <div class="ik-warnings" id="ik-warnings" style="display: none;">
                <div class="warning-header">⚠️ IK Warnings</div>
                <div id="ik-warning-content"></div>
            </div>
        </div>
    `;
}

// ============================================================================
// EVENT HANDLERS
// ============================================================================

function onIKSolverChange(solverType) {
    IKState.activeSolver = solverType;
    
    // Update description
    const desc = IKState.solverTypes.find(s => s.id === solverType)?.desc || '';
    const descElem = document.getElementById('solver-desc');
    if (descElem) descElem.textContent = desc;
    
    // Send to ESP32
    fetch('/api/ik/configure', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ solver: solverType })
    })
    .then(r => r.json())
    .then(data => {
        if (data.success) {
            showNotification(`IK solver changed to ${solverType}`, 'success');
        } else {
            showNotification('Failed to change IK solver', 'error');
        }
    })
    .catch(err => {
        console.error('IK config failed:', err);
        showNotification('IK configuration failed', 'error');
    });
    
    // Refresh preview if enabled
    if (IKState.previewEnabled) {
        requestIKPreview();
    }
}

function onPreferenceChange(preference) {
    IKState.preference = preference;
    
    // Update button states
    document.querySelectorAll('.pref-btn').forEach(btn => {
        btn.classList.remove('active');
    });
    event.target.classList.add('active');
    
    // Send to ESP32
    fetch('/api/ik/configure', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ preference: preference })
    });
}

function onPreviewToggle(enabled) {
    IKState.previewEnabled = enabled;
    const previewPanel = document.getElementById('ik-preview-panel');
    if (previewPanel) {
        previewPanel.style.opacity = enabled ? '1' : '0.5';
    }
}

// ============================================================================
// IK PREVIEW
// ============================================================================

function requestIKPreview() {
    const xInput = document.getElementById('target-x');
    const yInput = document.getElementById('target-y');
    const zInput = document.getElementById('target-z');
    
    if (!xInput || !yInput || !zInput) return;
    
    const target = {
        x: parseFloat(xInput.value) || 0,
        y: parseFloat(yInput.value) || 0,
        z: parseFloat(zInput.value) || 0,
        rx: 0, ry: 0, rz: 0
    };
    
    fetch('/api/ik/preview', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ target: [target.x, target.y, target.z, target.rx, target.ry, target.rz] })
    })
    .then(r => r.json())
    .then(data => {
        updateIKPreviewDisplay(data);
        IKState.lastResult = data;
    })
    .catch(err => {
        console.error('IK preview failed:', err);
    });
}

function updateIKPreviewDisplay(result) {
    const content = document.getElementById('ik-preview-content');
    const warnings = document.getElementById('ik-warnings');
    const warningContent = document.getElementById('ik-warning-content');
    
    if (!content) return;
    
    if (result.feasible) {
        let html = `
            <div class="ik-status success">✓ Reachable</div>
            <div class="ik-details">
                <div class="ik-metric">
                    <span class="label">Confidence:</span>
                    <span class="value">${(result.confidence * 100).toFixed(1)}%</span>
                </div>
                <div class="ik-metric">
                    <span class="label">Error:</span>
                    <span class="value">${result.error_metric.toFixed(2)} mm</span>
                </div>
                <div class="ik-joints">
                    <span class="label">Joint Angles:</span>
                    <div class="joint-values">
        `;
        
        for (let i = 0; i < 6; i++) {
            html += `<span class="joint-val">J${i}: ${result.joint_angles[i].toFixed(1)}°</span>`;
        }
        
        html += `
                    </div>
                </div>
            </div>
        `;
        
        content.innerHTML = html;
        
        // Update 3D preview
        if (window.simulator && IKState.previewEnabled) {
            window.simulator.showGhostRobot(result.joint_angles);
        }
    } else {
        content.innerHTML = `
            <div class="ik-status error">✗ Unreachable</div>
            <div class="ik-error-msg">${result.error_message || 'Target cannot be reached'}</div>
        `;
        
        // Hide ghost robot
        if (window.simulator) {
            window.simulator.hideGhostRobot();
        }
    }
    
    // Show warnings if any
    if (result.singularity_warning || result.joint_limit_warning) {
        warnings.style.display = 'block';
        let warnHtml = '';
        
        if (result.singularity_warning) {
            warnHtml += `<div class="warning-item singularity">
                ⚠️ Near singularity (metric: ${result.singularity_metric.toFixed(2)})
            </div>`;
        }
        
        if (result.joint_limit_warning) {
            warnHtml += `<div class="warning-item limit">
                ⚠️ Joint ${result.limiting_joint} near limit
            </div>`;
        }
        
        warningContent.innerHTML = warnHtml;
    } else {
        warnings.style.display = 'none';
    }
}

// ============================================================================
// REACHABILITY VISUALIZATION
// ============================================================================

function showReachabilityOverlay(reachable) {
    const overlay = document.getElementById('reachability-overlay');
    if (!overlay) return;
    
    if (reachable) {
        overlay.className = 'reachability-overlay reachable';
        overlay.innerHTML = '✓ Target Reachable';
    } else {
        overlay.className = 'reachability-overlay unreachable';
        overlay.innerHTML = '✗ Target Unreachable';
    }
    
    overlay.style.display = 'block';
    
    // Auto-hide after 2 seconds
    setTimeout(() => {
        overlay.style.display = 'none';
    }, 2000);
}

// ============================================================================
// SINGULARITY VISUALIZATION
// ============================================================================

function updateSingularityWarning(metric) {
    const indicator = document.getElementById('singularity-indicator');
    if (!indicator) return;
    
    if (metric < 0.1) {
        indicator.className = 'sing-indicator critical';
        indicator.innerHTML = '🔴 SINGULARITY';
        indicator.style.display = 'block';
    } else if (metric < 0.3) {
        indicator.className = 'sing-indicator warning';
        indicator.innerHTML = '🟡 Near Singularity';
        indicator.style.display = 'block';
    } else {
        indicator.style.display = 'none';
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

function initIKConfig() {
    // Fetch current IK config from ESP32
    fetch('/api/ik/config')
        .then(r => r.json())
        .then(data => {
            if (data.solver) {
                IKState.activeSolver = data.solver;
            }
            if (data.preference) {
                IKState.preference = data.preference;
            }
            
            // Update UI
            const select = document.getElementById('ik-solver-select');
            if (select) select.value = IKState.activeSolver;
        })
        .catch(err => {
            console.warn('Could not fetch IK config:', err);
        });
    
    // Add input listeners for live preview
    ['target-x', 'target-y', 'target-z'].forEach(id => {
        const input = document.getElementById(id);
        if (input) {
            input.addEventListener('input', debounce(requestIKPreview, 300));
        }
    });
}

// Helper: debounce function
function debounce(func, wait) {
    let timeout;
    return function(...args) {
        clearTimeout(timeout);
        timeout = setTimeout(() => func.apply(this, args), wait);
    };
}

// Initialize when document ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initIKConfig);
} else {
    initIKConfig();
}
