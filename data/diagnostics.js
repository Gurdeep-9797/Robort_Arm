/**
 * Diagnostics Page Module
 * 
 * Provides:
 * - Real-time alignment status display
 * - Encoder vs command error plots
 * - Fault history table
 * - IK vs FK drift visualization
 */

// ============================================================================
// DIAGNOSTICS STATE
// ============================================================================

const DiagState = {
    updateInterval: 500,    // ms
    maxHistoryPoints: 100,
    alignmentHistory: [],
    faultHistory: [],
    isStreaming: false
};

// ============================================================================
// DIAGNOSTICS PANEL CREATION
// ============================================================================

function createDiagnosticsPanel() {
    return `
        <div class="diagnostics-panel">
            <h4>System Diagnostics</h4>
            
            <div class="diag-section">
                <h5>Alignment Status</h5>
                <div class="alignment-status-grid" id="alignment-status-grid">
                    ${createAlignmentStatusGrid()}
                </div>
            </div>
            
            <div class="diag-section">
                <h5>Joint Error History</h5>
                <div class="error-chart" id="error-chart">
                    <canvas id="error-chart-canvas" width="400" height="150"></canvas>
                </div>
            </div>
            
            <div class="diag-section">
                <h5>Fault History</h5>
                <div class="fault-history-controls">
                    <button class="btn-small" onclick="clearFaultHistory()">Clear History</button>
                    <button class="btn-small" onclick="exportDiagnostics()">Export Data</button>
                </div>
                <div class="fault-history-table" id="fault-history-table">
                    <table>
                        <thead>
                            <tr>
                                <th>Time</th>
                                <th>Type</th>
                                <th>Joint</th>
                                <th>Error</th>
                                <th>Action</th>
                            </tr>
                        </thead>
                        <tbody id="fault-history-body">
                            <tr><td colspan="5" class="empty">No faults recorded</td></tr>
                        </tbody>
                    </table>
                </div>
            </div>
            
            <div class="diag-section">
                <h5>System Health</h5>
                <div class="health-indicators">
                    <div class="health-item">
                        <span class="health-label">Alignment</span>
                        <span class="health-value" id="health-alignment">OK</span>
                    </div>
                    <div class="health-item">
                        <span class="health-label">Communication</span>
                        <span class="health-value" id="health-comm">OK</span>
                    </div>
                    <div class="health-item">
                        <span class="health-label">Motor Response</span>
                        <span class="health-value" id="health-motor">OK</span>
                    </div>
                    <div class="health-item">
                        <span class="health-label">IK Solver</span>
                        <span class="health-value" id="health-ik">OK</span>
                    </div>
                </div>
            </div>
            
            <div class="diag-controls">
                <button class="btn-stream ${DiagState.isStreaming ? 'active' : ''}" 
                        onclick="toggleDiagStream()">
                    ${DiagState.isStreaming ? '⏸ Pause' : '▶ Stream'}
                </button>
                <span class="stream-status" id="stream-status">
                    ${DiagState.isStreaming ? 'Streaming...' : 'Paused'}
                </span>
            </div>
        </div>
    `;
}

function createAlignmentStatusGrid() {
    let html = '';
    for (let i = 0; i < 6; i++) {
        html += `
            <div class="alignment-joint" id="align-joint-${i}">
                <div class="joint-label">J${i}</div>
                <div class="joint-error" id="align-error-${i}">0.00°</div>
                <div class="joint-bar">
                    <div class="error-bar" id="error-bar-${i}" style="width: 0%;"></div>
                </div>
                <div class="joint-status" id="align-status-${i}">●</div>
            </div>
        `;
    }
    return html;
}

// ============================================================================
// ALIGNMENT STATUS UPDATE
// ============================================================================

function updateAlignmentDisplay(status) {
    if (!status) return;

    // Update per-joint display
    for (let i = 0; i < 6; i++) {
        const error = status.command_vs_response_error ? status.command_vs_response_error[i] : 0;
        const errorElem = document.getElementById(`align-error-${i}`);
        const barElem = document.getElementById(`error-bar-${i}`);
        const statusElem = document.getElementById(`align-status-${i}`);
        const jointElem = document.getElementById(`align-joint-${i}`);

        if (errorElem) {
            errorElem.textContent = error.toFixed(2) + '°';
        }

        if (barElem) {
            // Scale: 0-5 degrees = 0-100%
            const barWidth = Math.min(100, (error / 5) * 100);
            barElem.style.width = barWidth + '%';

            // Color based on threshold
            if (error >= 5) {
                barElem.className = 'error-bar critical';
            } else if (error >= 2) {
                barElem.className = 'error-bar warning';
            } else {
                barElem.className = 'error-bar ok';
            }
        }

        if (statusElem && jointElem) {
            if (error >= 5) {
                statusElem.className = 'joint-status critical';
                jointElem.className = 'alignment-joint critical';
            } else if (error >= 2) {
                statusElem.className = 'joint-status warning';
                jointElem.className = 'alignment-joint warning';
            } else {
                statusElem.className = 'joint-status ok';
                jointElem.className = 'alignment-joint ok';
            }
        }
    }

    // Update health indicators
    updateHealthIndicator('alignment', status.aligned ? 'OK' : (status.critical ? 'CRITICAL' : 'WARNING'));

    // Add to history for chart
    DiagState.alignmentHistory.push({
        timestamp: Date.now(),
        maxError: status.max_joint_error || 0,
        avgError: status.average_joint_error || 0
    });

    // Limit history size
    if (DiagState.alignmentHistory.length > DiagState.maxHistoryPoints) {
        DiagState.alignmentHistory.shift();
    }

    // Update chart
    updateErrorChart();
}

// ============================================================================
// ERROR CHART
// ============================================================================

let errorChartCtx = null;

function updateErrorChart() {
    const canvas = document.getElementById('error-chart-canvas');
    if (!canvas) return;

    if (!errorChartCtx) {
        errorChartCtx = canvas.getContext('2d');
    }

    const ctx = errorChartCtx;
    const width = canvas.width;
    const height = canvas.height;

    // Clear
    ctx.fillStyle = '#1e1e1e';
    ctx.fillRect(0, 0, width, height);

    if (DiagState.alignmentHistory.length < 2) return;

    // Draw grid
    ctx.strokeStyle = '#3e3e42';
    ctx.lineWidth = 1;
    for (let y = 0; y <= height; y += height / 5) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
    }

    // Plot max error line
    const maxVal = 5;  // 5 degree scale
    ctx.strokeStyle = '#e74c3c';
    ctx.lineWidth = 2;
    ctx.beginPath();

    DiagState.alignmentHistory.forEach((point, i) => {
        const x = (i / DiagState.maxHistoryPoints) * width;
        const y = height - (point.maxError / maxVal) * height;

        if (i === 0) {
            ctx.moveTo(x, y);
        } else {
            ctx.lineTo(x, y);
        }
    });
    ctx.stroke();

    // Plot avg error line
    ctx.strokeStyle = '#4ec9b0';
    ctx.lineWidth = 2;
    ctx.beginPath();

    DiagState.alignmentHistory.forEach((point, i) => {
        const x = (i / DiagState.maxHistoryPoints) * width;
        const y = height - (point.avgError / maxVal) * height;

        if (i === 0) {
            ctx.moveTo(x, y);
        } else {
            ctx.lineTo(x, y);
        }
    });
    ctx.stroke();

    // Draw legend
    ctx.font = '10px Arial';
    ctx.fillStyle = '#e74c3c';
    ctx.fillText('Max Error', 10, 15);
    ctx.fillStyle = '#4ec9b0';
    ctx.fillText('Avg Error', 80, 15);
}

// ============================================================================
// FAULT HISTORY
// ============================================================================

function updateFaultHistory(faults) {
    const tbody = document.getElementById('fault-history-body');
    if (!tbody || !faults) return;

    DiagState.faultHistory = faults;

    if (faults.length === 0) {
        tbody.innerHTML = '<tr><td colspan="5" class="empty">No faults recorded</td></tr>';
        return;
    }

    let html = '';
    faults.slice(-20).reverse().forEach(fault => {
        const time = new Date(fault.timestamp_ms).toLocaleTimeString();
        const typeClass = fault.motion_blocked ? 'critical' : 'warning';

        html += `
            <tr class="${typeClass}">
                <td>${time}</td>
                <td>${getFaultTypeName(fault.fault_code)}</td>
                <td>J${fault.worst_joint || '-'}</td>
                <td>${fault.max_error?.toFixed(2) || '-'}°</td>
                <td>${fault.motion_blocked ? 'Motion Blocked' : 'Speed Reduced'}</td>
            </tr>
        `;
    });

    tbody.innerHTML = html;
}

function getFaultTypeName(code) {
    const names = {
        0: 'None',
        1: 'Joint Warning',
        2: 'Joint Critical',
        3: 'TCP Warning',
        4: 'TCP Critical',
        5: 'Encoder Drift',
        6: 'Motor Fault',
        7: 'Comm Error'
    };
    return names[code] || `Unknown (${code})`;
}

function clearFaultHistory() {
    fetch('/api/diagnostics/clear', { method: 'POST' })
        .then(r => r.json())
        .then(() => {
            DiagState.faultHistory = [];
            DiagState.alignmentHistory = [];
            updateFaultHistory([]);
            showNotification('Fault history cleared', 'success');
        });
}

// ============================================================================
// HEALTH INDICATORS
// ============================================================================

function updateHealthIndicator(type, status) {
    const elem = document.getElementById(`health-${type}`);
    if (!elem) return;

    elem.textContent = status;
    elem.className = 'health-value ' + status.toLowerCase();
}

// ============================================================================
// STREAMING CONTROL
// ============================================================================

let diagStreamInterval = null;

function toggleDiagStream() {
    DiagState.isStreaming = !DiagState.isStreaming;

    const btn = document.querySelector('.btn-stream');
    const status = document.getElementById('stream-status');

    if (DiagState.isStreaming) {
        if (btn) btn.textContent = '⏸ Pause';
        if (btn) btn.classList.add('active');
        if (status) status.textContent = 'Streaming...';

        diagStreamInterval = setInterval(fetchDiagnostics, DiagState.updateInterval);
    } else {
        if (btn) btn.textContent = '▶ Stream';
        if (btn) btn.classList.remove('active');
        if (status) status.textContent = 'Paused';

        if (diagStreamInterval) {
            clearInterval(diagStreamInterval);
            diagStreamInterval = null;
        }
    }
}

function fetchDiagnostics() {
    fetch('/api/alignment/status')
        .then(r => r.json())
        .then(data => {
            updateAlignmentDisplay(data);
        })
        .catch(err => {
            updateHealthIndicator('comm', 'ERROR');
        });

    fetch('/api/diagnostics/history')
        .then(r => r.json())
        .then(data => {
            updateFaultHistory(data.faults || []);
        });
}

// ============================================================================
// EXPORT
// ============================================================================

function exportDiagnostics() {
    const data = {
        timestamp: new Date().toISOString(),
        alignmentHistory: DiagState.alignmentHistory,
        faultHistory: DiagState.faultHistory
    };

    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);

    const a = document.createElement('a');
    a.href = url;
    a.download = `robot_diagnostics_${Date.now()}.json`;
    a.click();

    URL.revokeObjectURL(url);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

function initDiagnostics() {
    // Start streaming by default
    toggleDiagStream();
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initDiagnostics);
} else {
    initDiagnostics();
}
