/**
 * @file AlignmentChecker.h
 * @brief Continuous alignment verification system
 * 
 * SAFETY-CRITICAL: Monitors alignment between:
 * - IK output vs encoder feedback
 * - FK result vs expected TCP pose
 * - Motor command vs actual response
 * 
 * Misalignment triggers:
 * - Speed reduction (warning level)
 * - Motion stop (critical level)
 * 
 * NO SILENT FAILURES ALLOWED.
 */

#ifndef ALIGNMENT_CHECKER_H
#define ALIGNMENT_CHECKER_H

#include <Arduino.h>
#include "RobotCore.h"

// ============================================================================
// ALIGNMENT STATUS STRUCTURE
// ============================================================================

/**
 * @struct AlignmentStatus
 * @brief Complete alignment check result
 */
struct AlignmentStatus {
    // ===== Per-Joint Errors =====
    float ik_vs_encoder_error[NUM_JOINTS];      // Degrees
    float command_vs_response_error[NUM_JOINTS]; // Degrees
    
    // ===== Global Errors =====
    float fk_vs_tcp_error;          // mm (position error at TCP)
    float average_joint_error;       // Degrees
    float max_joint_error;           // Degrees
    uint8_t worst_joint;             // Joint with largest error
    
    // ===== Status Flags =====
    bool aligned;                    // All checks passed
    bool warning;                    // Minor misalignment detected
    bool critical;                   // Critical misalignment - motion blocked
    
    // ===== Fault Information =====
    uint8_t fault_code;              // 0 = no fault
    char fault_message[32];          // Human-readable fault
    
    // ===== Timing =====
    uint32_t timestamp_ms;           // When check was performed
    uint32_t check_duration_us;      // How long check took
    
    // Default constructor
    AlignmentStatus() {
        for (int i = 0; i < NUM_JOINTS; i++) {
            ik_vs_encoder_error[i] = 0.0f;
            command_vs_response_error[i] = 0.0f;
        }
        fk_vs_tcp_error = 0.0f;
        average_joint_error = 0.0f;
        max_joint_error = 0.0f;
        worst_joint = 0;
        aligned = true;
        warning = false;
        critical = false;
        fault_code = 0;
        memset(fault_message, 0, 32);
        timestamp_ms = 0;
        check_duration_us = 0;
    }
};

// ============================================================================
// ALIGNMENT THRESHOLDS
// ============================================================================

struct AlignmentThresholds {
    float warning_joint_deg;         // Joint error to trigger warning
    float critical_joint_deg;        // Joint error to trigger stop
    float warning_tcp_mm;            // TCP error to trigger warning
    float critical_tcp_mm;           // TCP error to trigger stop
    float speed_reduction_factor;    // Speed multiplier on warning (0.0-1.0)
    
    // Default constructor with safe values
    AlignmentThresholds() {
        warning_joint_deg = 2.0f;    // Warn at 2 degrees
        critical_joint_deg = 5.0f;   // Stop at 5 degrees
        warning_tcp_mm = 5.0f;       // Warn at 5mm TCP error
        critical_tcp_mm = 15.0f;     // Stop at 15mm TCP error
        speed_reduction_factor = 0.5f; // Reduce speed to 50% on warning
    }
};

// ============================================================================
// ALIGNMENT FAULT CODES
// ============================================================================

namespace AlignmentFault {
    constexpr uint8_t NONE = 0;
    constexpr uint8_t JOINT_WARNING = 1;
    constexpr uint8_t JOINT_CRITICAL = 2;
    constexpr uint8_t TCP_WARNING = 3;
    constexpr uint8_t TCP_CRITICAL = 4;
    constexpr uint8_t ENCODER_DRIFT = 5;
    constexpr uint8_t MOTOR_FAULT = 6;
    constexpr uint8_t COMMUNICATION_ERROR = 7;
}

// ============================================================================
// ALIGNMENT HISTORY (for diagnostics)
// ============================================================================

struct AlignmentHistoryEntry {
    uint32_t timestamp_ms;
    float max_error;
    uint8_t fault_code;
    bool motion_blocked;
};

// ============================================================================
// ALIGNMENT CHECKER CLASS
// ============================================================================

/**
 * @class AlignmentChecker
 * @brief Continuous alignment verification system
 * 
 * Runs in background task, checks alignment at configurable intervals.
 * Integrates with SafetyManager to trigger speed reduction or stop.
 */
class AlignmentChecker {
public:
    static AlignmentChecker& getInstance();
    
    /**
     * @brief Initialize alignment checker
     * @return true if successful
     */
    bool begin();
    
    /**
     * @brief Perform alignment check NOW
     * @return AlignmentStatus with current alignment state
     */
    AlignmentStatus check();
    
    /**
     * @brief Get last alignment status (cached)
     * @return Most recent AlignmentStatus
     */
    AlignmentStatus getLastStatus();
    
    /**
     * @brief Check if system is currently aligned
     * @return true if aligned within thresholds
     */
    bool isAligned();
    
    /**
     * @brief Check if warning level exceeded
     * @return true if warning active
     */
    bool hasWarning();
    
    /**
     * @brief Check if critical level exceeded
     * @return true if motion should be blocked
     */
    bool isCritical();
    
    // ===== Configuration =====
    
    /**
     * @brief Set alignment thresholds
     * @param thresholds New threshold values
     */
    void setThresholds(const AlignmentThresholds& thresholds);
    
    /**
     * @brief Get current thresholds
     * @return AlignmentThresholds
     */
    AlignmentThresholds getThresholds();
    
    /**
     * @brief Set check interval
     * @param interval_ms Milliseconds between checks
     */
    void setCheckInterval(uint32_t interval_ms);
    
    /**
     * @brief Enable/disable alignment checking
     * @param enabled True to enable
     */
    void setEnabled(bool enabled);
    
    /**
     * @brief Check if alignment checking is enabled
     * @return true if enabled
     */
    bool isEnabled();
    
    // ===== Speed Modification =====
    
    /**
     * @brief Get current speed modifier based on alignment
     * @return 1.0 = full speed, 0.0 = stopped
     */
    float getSpeedModifier();
    
    // ===== Diagnostics =====
    
    /**
     * @brief Get alignment history for diagnostics
     * @param history Output array
     * @param max_entries Maximum entries to return
     * @return Number of entries returned
     */
    uint8_t getHistory(AlignmentHistoryEntry* history, uint8_t max_entries);
    
    /**
     * @brief Clear fault history
     */
    void clearHistory();
    
    /**
     * @brief Get fault count in last N seconds
     * @param seconds Time window
     * @return Number of faults
     */
    uint16_t getFaultCount(uint16_t seconds);
    
private:
    AlignmentChecker();
    
    // ===== State =====
    AlignmentStatus last_status_;
    AlignmentThresholds thresholds_;
    bool enabled_;
    uint32_t check_interval_ms_;
    float speed_modifier_;
    
    // ===== Expected Values (from last command) =====
    float expected_positions_[NUM_JOINTS];
    float expected_tcp_[3];  // x, y, z
    
    // ===== History =====
    static const uint8_t HISTORY_SIZE = 20;
    AlignmentHistoryEntry history_[HISTORY_SIZE];
    uint8_t history_index_;
    
    // ===== Synchronization =====
    SemaphoreHandle_t mutex_;
    
    // ===== Task =====
    static void alignmentTask(void* param);
    TaskHandle_t task_handle_;
    
    // ===== Helper Functions =====
    void updateExpectedPositions();
    void addHistoryEntry(const AlignmentStatus& status);
    void notifySafetyManager(const AlignmentStatus& status);
};

#endif // ALIGNMENT_CHECKER_H
