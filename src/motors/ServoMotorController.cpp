/**
 * @file ServoMotorController.cpp
 * @brief Servo motor controller implementation using PCA9685
 * 
 * SAFETY-CRITICAL: Controls physical servo motors
 * Migrated from existing ServoController with MotorInterface compliance
 * 
 * @note Uses static memory only - no dynamic allocation
 * @note All operations validate limits before execution
 */

#include "../include/motors/MotorInterface.h"
#include "../include/motors/MotorConfig.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============================================================================
// PCA9685 DRIVER SETTINGS (from existing RobotCore.cpp)
// ============================================================================

#define PCA9685_ADDRESS     0x40
#define PCA9685_FREQ        50      // 50Hz for standard servos
#define PCA9685_OSC_FREQ    27000000

// ============================================================================
// SERVO MOTOR CONTROLLER IMPLEMENTATION
// ============================================================================

/**
 * @class ServoMotorController
 * @brief PCA9685-based servo motor controller
 * 
 * Implements IMotorController interface for PWM servo motors.
 * Maintains backward compatibility with existing ServoController usage.
 */
class ServoMotorController : public IMotorController {
public:
    static ServoMotorController& getInstance();
    
    // IMotorController interface implementation
    bool begin() override;
    void enable(uint8_t axis) override;
    void disable(uint8_t axis) override;
    void enableAll() override;
    void disableAll() override;
    void emergencyStop() override;
    bool setTarget(uint8_t axis, float position, float velocity = 0) override;
    bool setAllTargets(const float* positions, float velocity = 0) override;
    float getPosition(uint8_t axis) override;
    float getVelocity(uint8_t axis) override;
    void getAllPositions(float* positions) override;
    void getAllVelocities(float* velocities) override;
    bool isEnabled(uint8_t axis) override;
    MotorStatus getStatus(uint8_t axis) override;
    bool setControlMode(uint8_t axis, MotorControlMode mode) override;
    MotorType getType() override;
    void update() override;

private:
    ServoMotorController();
    
    Adafruit_PWMServoDriver pwm_;
    
    // Per-axis state (static arrays - no dynamic allocation)
    bool enabled_[NUM_JOINTS];
    float current_positions_[NUM_JOINTS];
    float target_positions_[NUM_JOINTS];
    float current_velocities_[NUM_JOINTS];
    uint32_t last_update_time_[NUM_JOINTS];
    
    // Helper functions
    uint16_t angleToPulse(uint8_t axis, float angle);
    float pulseToAngle(uint8_t axis, uint16_t pulse);
    float constrainAngle(uint8_t axis, float angle);
};

// ============================================================================
// SINGLETON INSTANCE
// ============================================================================

ServoMotorController& ServoMotorController::getInstance() {
    static ServoMotorController instance;
    return instance;
}

// ============================================================================
// CONSTRUCTOR
// ============================================================================

ServoMotorController::ServoMotorController() 
    : pwm_(Adafruit_PWMServoDriver(PCA9685_ADDRESS)) {
    
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        enabled_[i] = false;
        current_positions_[i] = 0.0f;
        target_positions_[i] = 0.0f;
        current_velocities_[i] = 0.0f;
        last_update_time_[i] = 0;
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool ServoMotorController::begin() {
    // Initialize I2C (SDA=21, SCL=22 for ESP32)
    Wire.begin(21, 22);
    
    // Initialize PCA9685
    pwm_.begin();
    pwm_.setOscillatorFrequency(PCA9685_OSC_FREQ);
    pwm_.setPWMFreq(PCA9685_FREQ);
    
    delay(10);  // Allow oscillator to stabilize
    
    // Initialize all joints to home position (0 degrees)
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        enabled_[i] = true;
        target_positions_[i] = 0.0f;
        current_positions_[i] = 0.0f;
        last_update_time_[i] = millis();
        
        // Set initial pulse
        uint16_t pulse = angleToPulse(i, 0.0f);
        pwm_.setPWM(i, 0, pulse);
    }
    
    return true;
}

// ============================================================================
// ENABLE / DISABLE
// ============================================================================

void ServoMotorController::enable(uint8_t axis) {
    if (axis >= NUM_JOINTS) return;
    enabled_[axis] = true;
}

void ServoMotorController::disable(uint8_t axis) {
    if (axis >= NUM_JOINTS) return;
    
    // Stop PWM signal to disable servo
    pwm_.setPWM(axis, 0, 0);
    enabled_[axis] = false;
}

void ServoMotorController::enableAll() {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        enabled_[i] = true;
    }
}

void ServoMotorController::disableAll() {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        pwm_.setPWM(i, 0, 0);
        enabled_[i] = false;
    }
}

// ============================================================================
// EMERGENCY STOP (SAFETY-CRITICAL)
// ============================================================================

void ServoMotorController::emergencyStop() {
    // CRITICAL: Immediate stop - no validation, no delays
    // Must complete within 1ms
    
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        pwm_.setPWM(i, 0, 0);  // Cut PWM immediately
        enabled_[i] = false;
        current_velocities_[i] = 0.0f;
    }
}

// ============================================================================
// TARGET SETTING
// ============================================================================

bool ServoMotorController::setTarget(uint8_t axis, float position, float velocity) {
    if (axis >= NUM_JOINTS) return false;
    if (!enabled_[axis]) return false;
    
    // Get axis configuration for limit checking
    AxisMotorConfig& config = MotorConfigManager::getInstance().getAxisConfig(axis);
    
    // Constrain position to limits (SAFETY)
    float constrained = constrainAngle(axis, position);
    
    // Check if position was constrained (indicates limit violation attempt)
    if (constrained != position) {
        // Log warning but accept constrained value
        Serial.printf("[SERVO] Axis %d: Position %.1f constrained to %.1f\n", 
                      axis, position, constrained);
    }
    
    target_positions_[axis] = constrained;
    
    // For servos, we send position immediately (servos handle their own speed)
    uint16_t pulse = angleToPulse(axis, constrained);
    pwm_.setPWM(axis, 0, pulse);
    
    return true;
}

bool ServoMotorController::setAllTargets(const float* positions, float velocity) {
    if (positions == nullptr) return false;
    
    bool all_success = true;
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        if (!setTarget(i, positions[i], velocity)) {
            all_success = false;
        }
    }
    return all_success;
}

// ============================================================================
// STATE QUERIES
// ============================================================================

float ServoMotorController::getPosition(uint8_t axis) {
    if (axis >= NUM_JOINTS) return 0.0f;
    // For open-loop servos, return target position as current position
    return current_positions_[axis];
}

float ServoMotorController::getVelocity(uint8_t axis) {
    if (axis >= NUM_JOINTS) return 0.0f;
    return current_velocities_[axis];
}

void ServoMotorController::getAllPositions(float* positions) {
    if (positions == nullptr) return;
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        positions[i] = current_positions_[i];
    }
}

void ServoMotorController::getAllVelocities(float* velocities) {
    if (velocities == nullptr) return;
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        velocities[i] = current_velocities_[i];
    }
}

bool ServoMotorController::isEnabled(uint8_t axis) {
    if (axis >= NUM_JOINTS) return false;
    return enabled_[axis];
}

MotorStatus ServoMotorController::getStatus(uint8_t axis) {
    MotorStatus status;
    
    if (axis >= NUM_JOINTS) {
        memset(&status, 0, sizeof(status));
        return status;
    }
    
    AxisMotorConfig& config = MotorConfigManager::getInstance().getAxisConfig(axis);
    
    status.enabled = enabled_[axis];
    status.current_position = current_positions_[axis];
    status.current_velocity = current_velocities_[axis];
    status.target_position = target_positions_[axis];
    status.target_velocity = 0.0f;  // Servos use internal speed
    status.position_error = target_positions_[axis] - current_positions_[axis];
    
    // Check if at limit
    status.at_limit = (current_positions_[axis] <= config.min_position + 0.5f) ||
                      (current_positions_[axis] >= config.max_position - 0.5f);
    
    status.fault = false;
    status.fault_code = 0;
    
    return status;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

bool ServoMotorController::setControlMode(uint8_t axis, MotorControlMode mode) {
    // Servos only support position mode
    if (mode != MotorControlMode::POSITION) {
        return false;
    }
    return true;
}

MotorType ServoMotorController::getType() {
    return MotorType::SERVO;
}

// ============================================================================
// UPDATE LOOP
// ============================================================================

void ServoMotorController::update() {
    uint32_t now = millis();
    
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        if (!enabled_[i]) continue;
        
        // Calculate time delta
        float dt = (now - last_update_time_[i]) / 1000.0f;
        if (dt <= 0.0f) continue;
        
        // For open-loop servos, simulate position tracking
        // In reality, position moves at servo's internal speed
        float error = target_positions_[i] - current_positions_[i];
        
        // Estimate velocity (degrees per second)
        if (dt > 0.001f) {
            // Servo typically moves at ~60 deg/sec for standard servo
            float max_step = 60.0f * dt;  // Estimated servo speed
            
            if (fabs(error) > max_step) {
                // Still moving
                float step = (error > 0) ? max_step : -max_step;
                current_positions_[i] += step;
                current_velocities_[i] = step / dt;
            } else {
                // Arrived at target
                current_positions_[i] = target_positions_[i];
                current_velocities_[i] = 0.0f;
            }
        }
        
        last_update_time_[i] = now;
    }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

uint16_t ServoMotorController::angleToPulse(uint8_t axis, float angle) {
    AxisMotorConfig& config = MotorConfigManager::getInstance().getAxisConfig(axis);
    
    // Map angle to pulse width
    // Default: -180° to +180° maps to 500us to 2500us
    float range_angle = config.servo_max_angle - config.servo_min_angle;
    float range_pulse = config.servo_max_pulse_us - config.servo_min_pulse_us;
    
    float normalized = (angle - config.servo_min_angle) / range_angle;
    float pulse_us = config.servo_min_pulse_us + (normalized * range_pulse);
    
    // Convert microseconds to PCA9685 ticks
    // At 50Hz, period = 20ms = 20000us
    // 4096 ticks per period
    // pulse_ticks = pulse_us * 4096 / 20000
    uint16_t pulse_ticks = (uint16_t)(pulse_us * 4096.0f / 20000.0f);
    
    return pulse_ticks;
}

float ServoMotorController::pulseToAngle(uint8_t axis, uint16_t pulse) {
    AxisMotorConfig& config = MotorConfigManager::getInstance().getAxisConfig(axis);
    
    // Convert PCA9685 ticks to microseconds
    float pulse_us = pulse * 20000.0f / 4096.0f;
    
    // Map pulse width to angle
    float range_angle = config.servo_max_angle - config.servo_min_angle;
    float range_pulse = config.servo_max_pulse_us - config.servo_min_pulse_us;
    
    float normalized = (pulse_us - config.servo_min_pulse_us) / range_pulse;
    float angle = config.servo_min_angle + (normalized * range_angle);
    
    return angle;
}

float ServoMotorController::constrainAngle(uint8_t axis, float angle) {
    AxisMotorConfig& config = MotorConfigManager::getInstance().getAxisConfig(axis);
    
    if (angle < config.min_position) return config.min_position;
    if (angle > config.max_position) return config.max_position;
    
    return angle;
}
