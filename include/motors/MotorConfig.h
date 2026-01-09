/**
 * @file MotorConfig.h
 * @brief Per-axis motor configuration structures
 * 
 * Defines hardware-specific parameters for each motor axis.
 * These values are typically set once during commissioning.
 * 
 * @note All values use static storage - no dynamic allocation
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <Arduino.h>
#include "../RobotCore.h"

// ============================================================================
// ENCODER TYPE ENUMERATION
// ============================================================================

enum class EncoderType : uint8_t {
    NONE = 0,           // No encoder (open-loop servo)
    INCREMENTAL = 1,    // Quadrature incremental encoder
    ABSOLUTE = 2,       // Absolute encoder (SSI, SPI, etc.)
    POTENTIOMETER = 3   // Analog potentiometer feedback
};

// ============================================================================
// MOTOR HARDWARE TYPE
// ============================================================================

enum class MotorHardwareType : uint8_t {
    STANDARD_SERVO = 0,     // Standard PWM hobby servo
    DIGITAL_SERVO = 1,      // High-speed digital servo
    BRUSHED_DC = 2,         // Brushed DC motor with H-bridge
    BRUSHLESS_DC = 3,       // Brushless DC (requires ESC)
    STEPPER = 4             // Stepper motor (future support)
};

// ============================================================================
// PID GAINS STRUCTURE
// ============================================================================

struct PIDGains {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float i_max;        // Integral windup limit
    float output_max;   // Maximum output value
    
    PIDGains() : kp(1.0f), ki(0.0f), kd(0.0f), i_max(100.0f), output_max(255.0f) {}
    
    PIDGains(float p, float i, float d, float imax = 100.0f, float omax = 255.0f)
        : kp(p), ki(i), kd(d), i_max(imax), output_max(omax) {}
};

// ============================================================================
// AXIS MOTOR CONFIGURATION
// ============================================================================

/**
 * @struct AxisMotorConfig
 * @brief Complete motor configuration for a single axis
 */
struct AxisMotorConfig {
    // ===== Hardware Identification =====
    MotorHardwareType hardware_type;
    EncoderType encoder_type;
    
    // ===== Mechanical Parameters =====
    float gear_ratio;               // Motor:output gear ratio (e.g., 50:1 = 50.0)
    float encoder_counts_per_rev;   // Encoder counts per motor revolution
    bool invert_direction;          // Invert motor direction
    bool invert_encoder;            // Invert encoder direction
    
    // ===== Electrical Parameters =====
    float max_current_amps;         // Maximum motor current (for DC motors)
    float nominal_voltage;          // Nominal motor voltage
    float torque_constant;          // Torque constant Kt (Nm/A) for DC motors
    float back_emf_constant;        // Back-EMF constant Ke (V/rad/s)
    
    // ===== Servo-Specific Parameters =====
    uint16_t servo_min_pulse_us;    // Minimum pulse width (microseconds)
    uint16_t servo_max_pulse_us;    // Maximum pulse width (microseconds)
    float servo_min_angle;          // Angle at min pulse
    float servo_max_angle;          // Angle at max pulse
    
    // ===== DC Motor PID Gains =====
    PIDGains position_pid;          // Position control PID
    PIDGains velocity_pid;          // Velocity control PID
    
    // ===== Motion Limits (from RobotConfig, mirrored for convenience) =====
    float min_position;             // Minimum allowed position (degrees)
    float max_position;             // Maximum allowed position (degrees)
    float max_velocity;             // Maximum velocity (degrees/second)
    float max_acceleration;         // Maximum acceleration (degrees/second^2)
    
    // ===== Default Constructor =====
    AxisMotorConfig() {
        hardware_type = MotorHardwareType::STANDARD_SERVO;
        encoder_type = EncoderType::NONE;
        
        gear_ratio = 1.0f;
        encoder_counts_per_rev = 4096.0f;
        invert_direction = false;
        invert_encoder = false;
        
        max_current_amps = 2.0f;
        nominal_voltage = 12.0f;
        torque_constant = 0.01f;
        back_emf_constant = 0.01f;
        
        // PCA9685 default pulse range (0.5ms - 2.5ms for 360° servo)
        servo_min_pulse_us = 500;
        servo_max_pulse_us = 2500;
        servo_min_angle = -180.0f;
        servo_max_angle = 180.0f;
        
        min_position = -180.0f;
        max_position = 180.0f;
        max_velocity = 120.0f;
        max_acceleration = 500.0f;
    }
};

// ============================================================================
// GLOBAL MOTOR CONFIGURATION
// ============================================================================

/**
 * @class MotorConfigManager
 * @brief Singleton managing motor configurations for all axes
 */
class MotorConfigManager {
public:
    static MotorConfigManager& getInstance();
    
    /**
     * @brief Initialize with default configurations
     * @return true if successful
     */
    bool begin();
    
    /**
     * @brief Get configuration for an axis
     * @param axis Joint index
     * @return Reference to axis configuration
     */
    AxisMotorConfig& getAxisConfig(uint8_t axis);
    
    /**
     * @brief Set configuration for an axis
     * @param axis Joint index
     * @param config New configuration
     * @return true if configuration valid and set
     */
    bool setAxisConfig(uint8_t axis, const AxisMotorConfig& config);
    
    /**
     * @brief Validate configuration safety
     * @param axis Joint index
     * @return true if configuration is safe
     */
    bool validateConfig(uint8_t axis);
    
    /**
     * @brief Check if configuration is unsafe
     * @param axis Joint index
     * @param warning_msg Output buffer for warning message (32 chars)
     * @return true if configuration has safety warnings
     */
    bool hasWarnings(uint8_t axis, char* warning_msg);
    
    /**
     * @brief Save configurations to NVS
     * @return true if save successful
     */
    bool saveToNVS();
    
    /**
     * @brief Load configurations from NVS
     * @return true if load successful
     */
    bool loadFromNVS();
    
private:
    MotorConfigManager();
    
    AxisMotorConfig configs_[NUM_JOINTS];
    SemaphoreHandle_t mutex_;
    
    // Validation helpers
    bool validateGearRatio(float ratio);
    bool validateCurrentLimit(float current);
    bool validatePulseWidth(uint16_t min_us, uint16_t max_us);
};

// ============================================================================
// DEFAULT CONFIGURATIONS
// ============================================================================

namespace MotorDefaults {
    // Standard MG996R 360° servo defaults
    constexpr uint16_t SERVO_MIN_PULSE = 500;
    constexpr uint16_t SERVO_MAX_PULSE = 2500;
    constexpr float SERVO_MIN_ANGLE = -180.0f;
    constexpr float SERVO_MAX_ANGLE = 180.0f;
    
    // DC motor defaults
    constexpr float DC_DEFAULT_KP = 2.0f;
    constexpr float DC_DEFAULT_KI = 0.1f;
    constexpr float DC_DEFAULT_KD = 0.05f;
    constexpr float DC_MAX_CURRENT = 3.0f;
    
    // Encoder defaults
    constexpr float ENCODER_CPR = 4096.0f;  // Counts per revolution
}

#endif // MOTOR_CONFIG_H
