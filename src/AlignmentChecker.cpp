/**
 * @file AlignmentChecker.cpp
 * @brief Implementation of continuous alignment verification
 * 
 * SAFETY-CRITICAL: This module runs continuously to verify
 * system alignment and triggers safety responses.
 */

#include "../include/AlignmentChecker.h"
#include "../include/RobotCore.h"
#include "../include/ik/IKSolver.h"

// ============================================================================
// SINGLETON INSTANCE
// ============================================================================

AlignmentChecker& AlignmentChecker::getInstance() {
    static AlignmentChecker instance;
    return instance;
}

// ============================================================================
// CONSTRUCTOR
// ============================================================================

AlignmentChecker::AlignmentChecker() {
    mutex_ = xSemaphoreCreateMutex();
    enabled_ = true;
    check_interval_ms_ = 100;  // 10Hz default
    speed_modifier_ = 1.0f;
    history_index_ = 0;
    task_handle_ = nullptr;
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        expected_positions_[i] = 0.0f;
    }
    for (int i = 0; i < 3; i++) {
        expected_tcp_[i] = 0.0f;
    }
    
    memset(history_, 0, sizeof(history_));
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool AlignmentChecker::begin() {
    // Create alignment check task (low priority, non-critical timing)
    xTaskCreatePinnedToCore(
        alignmentTask,
        "Alignment",
        4096,           // Stack size
        this,           // Task parameter
        1,              // Low priority (safety task is higher)
        &task_handle_,
        0               // Core 0 (non-real-time)
    );
    
    Serial.println("[AlignmentChecker] Started alignment monitoring");
    return true;
}

// ============================================================================
// ALIGNMENT TASK
// ============================================================================

void AlignmentChecker::alignmentTask(void* param) {
    AlignmentChecker* checker = (AlignmentChecker*)param;
    
    while (true) {
        if (checker->enabled_) {
            AlignmentStatus status = checker->check();
            
            // Notify safety manager if needed
            checker->notifySafetyManager(status);
        }
        
        vTaskDelay(pdMS_TO_TICKS(checker->check_interval_ms_));
    }
}

// ============================================================================
// MAIN CHECK FUNCTION
// ============================================================================

AlignmentStatus AlignmentChecker::check() {
    AlignmentStatus status;
    uint32_t start_time = micros();
    status.timestamp_ms = millis();
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    
    // ===== Step 1: Get Current Positions from Encoders =====
    float encoder_positions[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++) {
        encoder_positions[i] = EncoderReader::getInstance().getPositionDegrees(i);
    }
    
    // ===== Step 2: Get Commanded Positions =====
    float commanded_positions[NUM_JOINTS];
    ControlLoop::getInstance().getCurrentPositions(commanded_positions);
    
    // ===== Step 3: Calculate Per-Joint Errors =====
    float total_error = 0.0f;
    status.max_joint_error = 0.0f;
    status.worst_joint = 0;
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        // IK vs Encoder (for closed-loop mode)
        status.ik_vs_encoder_error[i] = fabs(expected_positions_[i] - encoder_positions[i]);
        
        // Command vs Response
        status.command_vs_response_error[i] = fabs(commanded_positions[i] - encoder_positions[i]);
        
        // Track max error
        float joint_error = status.command_vs_response_error[i];
        total_error += joint_error;
        
        if (joint_error > status.max_joint_error) {
            status.max_joint_error = joint_error;
            status.worst_joint = i;
        }
    }
    
    status.average_joint_error = total_error / NUM_JOINTS;
    
    // ===== Step 4: Calculate TCP Error via FK =====
    float actual_tcp[6];
    Kinematics::getInstance().forwardKinematics(encoder_positions, actual_tcp);
    
    float dx = actual_tcp[0] - expected_tcp_[0];
    float dy = actual_tcp[1] - expected_tcp_[1];
    float dz = actual_tcp[2] - expected_tcp_[2];
    status.fk_vs_tcp_error = sqrt(dx*dx + dy*dy + dz*dz);
    
    // ===== Step 5: Compare Against Thresholds =====
    status.aligned = true;
    status.warning = false;
    status.critical = false;
    
    // Check joint errors
    if (status.max_joint_error >= thresholds_.critical_joint_deg) {
        status.critical = true;
        status.aligned = false;
        status.fault_code = AlignmentFault::JOINT_CRITICAL;
        snprintf(status.fault_message, 31, "Joint %d: %.1f deg", 
                 status.worst_joint, status.max_joint_error);
    } else if (status.max_joint_error >= thresholds_.warning_joint_deg) {
        status.warning = true;
        status.fault_code = AlignmentFault::JOINT_WARNING;
        snprintf(status.fault_message, 31, "Joint %d drift", status.worst_joint);
    }
    
    // Check TCP error
    if (status.fk_vs_tcp_error >= thresholds_.critical_tcp_mm) {
        status.critical = true;
        status.aligned = false;
        status.fault_code = AlignmentFault::TCP_CRITICAL;
        snprintf(status.fault_message, 31, "TCP error: %.1f mm", status.fk_vs_tcp_error);
    } else if (status.fk_vs_tcp_error >= thresholds_.warning_tcp_mm) {
        status.warning = true;
        if (status.fault_code == AlignmentFault::NONE) {
            status.fault_code = AlignmentFault::TCP_WARNING;
            snprintf(status.fault_message, 31, "TCP drift");
        }
    }
    
    // ===== Step 6: Update Speed Modifier =====
    if (status.critical) {
        speed_modifier_ = 0.0f;  // Full stop
    } else if (status.warning) {
        speed_modifier_ = thresholds_.speed_reduction_factor;
    } else {
        speed_modifier_ = 1.0f;  // Full speed
    }
    
    // ===== Step 7: Record Timing =====
    status.check_duration_us = micros() - start_time;
    
    // ===== Step 8: Store Result and History =====
    last_status_ = status;
    addHistoryEntry(status);
    
    xSemaphoreGive(mutex_);
    
    return status;
}

// ============================================================================
// STATUS QUERIES
// ============================================================================

AlignmentStatus AlignmentChecker::getLastStatus() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    AlignmentStatus status = last_status_;
    xSemaphoreGive(mutex_);
    return status;
}

bool AlignmentChecker::isAligned() {
    return last_status_.aligned;
}

bool AlignmentChecker::hasWarning() {
    return last_status_.warning;
}

bool AlignmentChecker::isCritical() {
    return last_status_.critical;
}

float AlignmentChecker::getSpeedModifier() {
    return speed_modifier_;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void AlignmentChecker::setThresholds(const AlignmentThresholds& thresholds) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    thresholds_ = thresholds;
    xSemaphoreGive(mutex_);
}

AlignmentThresholds AlignmentChecker::getThresholds() {
    return thresholds_;
}

void AlignmentChecker::setCheckInterval(uint32_t interval_ms) {
    check_interval_ms_ = interval_ms;
}

void AlignmentChecker::setEnabled(bool enabled) {
    enabled_ = enabled;
}

bool AlignmentChecker::isEnabled() {
    return enabled_;
}

// ============================================================================
// HISTORY MANAGEMENT
// ============================================================================

void AlignmentChecker::addHistoryEntry(const AlignmentStatus& status) {
    if (status.fault_code != AlignmentFault::NONE) {
        history_[history_index_].timestamp_ms = status.timestamp_ms;
        history_[history_index_].max_error = status.max_joint_error;
        history_[history_index_].fault_code = status.fault_code;
        history_[history_index_].motion_blocked = status.critical;
        
        history_index_ = (history_index_ + 1) % HISTORY_SIZE;
    }
}

uint8_t AlignmentChecker::getHistory(AlignmentHistoryEntry* history, uint8_t max_entries) {
    if (history == nullptr) return 0;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    
    uint8_t count = 0;
    for (int i = 0; i < min(max_entries, HISTORY_SIZE); i++) {
        int idx = (history_index_ - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
        if (history_[idx].timestamp_ms > 0) {
            history[count] = history_[idx];
            count++;
        }
    }
    
    xSemaphoreGive(mutex_);
    return count;
}

void AlignmentChecker::clearHistory() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    memset(history_, 0, sizeof(history_));
    history_index_ = 0;
    xSemaphoreGive(mutex_);
}

uint16_t AlignmentChecker::getFaultCount(uint16_t seconds) {
    uint32_t cutoff = millis() - (seconds * 1000);
    uint16_t count = 0;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    for (int i = 0; i < HISTORY_SIZE; i++) {
        if (history_[i].timestamp_ms >= cutoff && history_[i].fault_code != 0) {
            count++;
        }
    }
    xSemaphoreGive(mutex_);
    
    return count;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void AlignmentChecker::updateExpectedPositions() {
    // Get current target positions from motion planner
    ControlLoop::getInstance().getCurrentPositions(expected_positions_);
    
    // Calculate expected TCP
    float tcp[6];
    Kinematics::getInstance().forwardKinematics(expected_positions_, tcp);
    expected_tcp_[0] = tcp[0];
    expected_tcp_[1] = tcp[1];
    expected_tcp_[2] = tcp[2];
}

void AlignmentChecker::notifySafetyManager(const AlignmentStatus& status) {
    // Integrate with SafetyManager
    if (status.critical) {
        // Set fault code for alignment critical error
        // Fault code bit 6 = alignment fault
        uint32_t current_faults = SafetyManager::getInstance().getFaults();
        if ((current_faults & (1 << 6)) == 0) {
            // New fault - log it
            Serial.printf("[ALIGNMENT CRITICAL] %s\n", status.fault_message);
        }
    }
    
    if (status.warning && !status.critical) {
        // Just log warning
        static uint32_t last_warning = 0;
        if (millis() - last_warning > 5000) {  // Throttle warnings to every 5s
            Serial.printf("[ALIGNMENT WARNING] %s\n", status.fault_message);
            last_warning = millis();
        }
    }
}
