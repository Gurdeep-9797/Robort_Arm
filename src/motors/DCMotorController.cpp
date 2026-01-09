/**
 * @file DCMotorController.cpp
 * @brief DC motor controller implementation with encoder feedback
 * 
 * SAFETY-CRITICAL: Controls physical DC motors with closed-loop control
 * 
 * Features:
 * - Position control with PID
 * - Velocity control with PID
 * - Torque/current limiting
 * - Encoder feedback integration
 * 
 * @note Uses static memory only - no dynamic allocation
 * @note PID loops are deterministic for real-time control
 */

#include "../include/motors/MotorInterface.h"
#include "../include/motors/MotorConfig.h"

// ============================================================================
// DC MOTOR PIN DEFINITIONS
// ============================================================================

// H-Bridge control pins (dual H-bridge driver like L298N or TB6612)
// These can be customized per axis in production
struct DCMotorPins {
    uint8_t pwm_pin;        // PWM speed control
    uint8_t dir_a_pin;      // Direction A
    uint8_t dir_b_pin;      // Direction B (for brake/coast)
    uint8_t enable_pin;     // Enable pin (optional)
};

// Default pin configuration (can be overridden)
static const DCMotorPins DEFAULT_DC_PINS[NUM_JOINTS] = {
    {25, 26, 27, 0},    // Joint 0
    {14, 12, 13, 0},    // Joint 1
    {15, 2, 4, 0},      // Joint 2
    {5, 18, 19, 0},     // Joint 3
    {23, 22, 21, 0},    // Joint 4 (Note: 21/22 conflict with I2C if servo used)
    {32, 33, 35, 0}     // Joint 5
};

// ============================================================================
// PID CONTROLLER (STATIC, DETERMINISTIC)
// ============================================================================

class PIDController {
public:
    PIDController() {
        reset();
    }
    
    void setGains(float kp, float ki, float kd, float i_max, float out_max) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        i_max_ = i_max;
        output_max_ = out_max;
    }
    
    void setGains(const PIDGains& gains) {
        kp_ = gains.kp;
        ki_ = gains.ki;
        kd_ = gains.kd;
        i_max_ = gains.i_max;
        output_max_ = gains.output_max;
    }
    
    float compute(float setpoint, float measurement, float dt) {
        if (dt <= 0.0f) return 0.0f;
        
        float error = setpoint - measurement;
        
        // Proportional
        float p_term = kp_ * error;
        
        // Integral with anti-windup
        integral_ += error * dt;
        if (integral_ > i_max_) integral_ = i_max_;
        if (integral_ < -i_max_) integral_ = -i_max_;
        float i_term = ki_ * integral_;
        
        // Derivative (on measurement to avoid derivative kick)
        float derivative = (measurement - last_measurement_) / dt;
        float d_term = -kd_ * derivative;
        
        last_measurement_ = measurement;
        last_error_ = error;
        
        // Sum and constrain output
        float output = p_term + i_term + d_term;
        if (output > output_max_) output = output_max_;
        if (output < -output_max_) output = -output_max_;
        
        return output;
    }
    
    void reset() {
        integral_ = 0.0f;
        last_error_ = 0.0f;
        last_measurement_ = 0.0f;
    }
    
private:
    float kp_ = 1.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float i_max_ = 100.0f;
    float output_max_ = 255.0f;
    
    float integral_ = 0.0f;
    float last_error_ = 0.0f;
    float last_measurement_ = 0.0f;
};

// ============================================================================
// DC MOTOR CONTROLLER IMPLEMENTATION
// ============================================================================

class DCMotorController : public IMotorController {
public:
    static DCMotorController& getInstance();
    
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
    DCMotorController();
    
    // Per-axis state (static arrays)
    bool enabled_[NUM_JOINTS];
    MotorControlMode control_mode_[NUM_JOINTS];
    
    float target_positions_[NUM_JOINTS];
    float target_velocities_[NUM_JOINTS];
    
    float current_positions_[NUM_JOINTS];
    float current_velocities_[NUM_JOINTS];
    float last_positions_[NUM_JOINTS];
    
    uint32_t last_update_time_[NUM_JOINTS];
    
    // PID controllers (static allocation)
    PIDController position_pid_[NUM_JOINTS];
    PIDController velocity_pid_[NUM_JOINTS];
    
    // Encoder counts (from EncoderReader)
    int32_t last_encoder_counts_[NUM_JOINTS];
    
    // Motor pins
    DCMotorPins pins_[NUM_JOINTS];
    
    // Helper functions
    void setMotorPWM(uint8_t axis, float output);
    float encoderToAngle(uint8_t axis, int32_t counts);
    float constrainAngle(uint8_t axis, float angle);
    void updatePositionFromEncoder(uint8_t axis);
};

// ============================================================================
// SINGLETON INSTANCE
// ============================================================================

DCMotorController& DCMotorController::getInstance() {
    static DCMotorController instance;
    return instance;
}

// ============================================================================
// CONSTRUCTOR
// ============================================================================

DCMotorController::DCMotorController() {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        enabled_[i] = false;
        control_mode_[i] = MotorControlMode::POSITION;
        
        target_positions_[i] = 0.0f;
        target_velocities_[i] = 0.0f;
        current_positions_[i] = 0.0f;
        current_velocities_[i] = 0.0f;
        last_positions_[i] = 0.0f;
        
        last_update_time_[i] = 0;
        last_encoder_counts_[i] = 0;
        
        // Copy default pin configuration
        pins_[i] = DEFAULT_DC_PINS[i];
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool DCMotorController::begin() {
    // Configure motor control pins
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        if (pins_[i].pwm_pin > 0) {
            pinMode(pins_[i].pwm_pin, OUTPUT);
            analogWrite(pins_[i].pwm_pin, 0);
        }
        if (pins_[i].dir_a_pin > 0) {
            pinMode(pins_[i].dir_a_pin, OUTPUT);
            digitalWrite(pins_[i].dir_a_pin, LOW);
        }
        if (pins_[i].dir_b_pin > 0) {
            pinMode(pins_[i].dir_b_pin, OUTPUT);
            digitalWrite(pins_[i].dir_b_pin, LOW);
        }
        if (pins_[i].enable_pin > 0) {
            pinMode(pins_[i].enable_pin, OUTPUT);
            digitalWrite(pins_[i].enable_pin, LOW);
        }
        
        // Initialize PID controllers with config values
        AxisMotorConfig& config = MotorConfigManager::getInstance().getAxisConfig(i);
        position_pid_[i].setGains(config.position_pid);
        velocity_pid_[i].setGains(config.velocity_pid);
        
        enabled_[i] = false;
        last_update_time_[i] = millis();
    }
    
    return true;
}

// ============================================================================
// ENABLE / DISABLE
// ============================================================================

void DCMotorController::enable(uint8_t axis) {
    if (axis >= NUM_JOINTS) return;
    
    enabled_[axis] = true;
    
    if (pins_[axis].enable_pin > 0) {
        digitalWrite(pins_[axis].enable_pin, HIGH);
    }
    
    // Reset PID on enable
    position_pid_[axis].reset();
    velocity_pid_[axis].reset();
}

void DCMotorController::disable(uint8_t axis) {
    if (axis >= NUM_JOINTS) return;
    
    // Stop motor first
    setMotorPWM(axis, 0);
    
    enabled_[axis] = false;
    
    if (pins_[axis].enable_pin > 0) {
        digitalWrite(pins_[axis].enable_pin, LOW);
    }
}

void DCMotorController::enableAll() {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        enable(i);
    }
}

void DCMotorController::disableAll() {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        disable(i);
    }
}

// ============================================================================
// EMERGENCY STOP (SAFETY-CRITICAL)
// ============================================================================

void DCMotorController::emergencyStop() {
    // CRITICAL: Immediate stop - brake all motors
    // Must complete within 1ms
    
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        // Set PWM to 0
        if (pins_[i].pwm_pin > 0) {
            analogWrite(pins_[i].pwm_pin, 0);
        }
        
        // Active brake: set both direction pins HIGH (short motor terminals)
        if (pins_[i].dir_a_pin > 0) {
            digitalWrite(pins_[i].dir_a_pin, HIGH);
        }
        if (pins_[i].dir_b_pin > 0) {
            digitalWrite(pins_[i].dir_b_pin, HIGH);
        }
        
        // Disable
        if (pins_[i].enable_pin > 0) {
            digitalWrite(pins_[i].enable_pin, LOW);
        }
        
        enabled_[i] = false;
        current_velocities_[i] = 0.0f;
        target_velocities_[i] = 0.0f;
    }
}

// ============================================================================
// TARGET SETTING
// ============================================================================

bool DCMotorController::setTarget(uint8_t axis, float position, float velocity) {
    if (axis >= NUM_JOINTS) return false;
    if (!enabled_[axis]) return false;
    
    // Constrain position to limits (SAFETY)
    float constrained = constrainAngle(axis, position);
    
    if (constrained != position) {
        Serial.printf("[DC] Axis %d: Position %.1f constrained to %.1f\n", 
                      axis, position, constrained);
    }
    
    target_positions_[axis] = constrained;
    
    if (velocity > 0.0f) {
        target_velocities_[axis] = velocity;
    }
    
    return true;
}

bool DCMotorController::setAllTargets(const float* positions, float velocity) {
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

float DCMotorController::getPosition(uint8_t axis) {
    if (axis >= NUM_JOINTS) return 0.0f;
    return current_positions_[axis];
}

float DCMotorController::getVelocity(uint8_t axis) {
    if (axis >= NUM_JOINTS) return 0.0f;
    return current_velocities_[axis];
}

void DCMotorController::getAllPositions(float* positions) {
    if (positions == nullptr) return;
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        positions[i] = current_positions_[i];
    }
}

void DCMotorController::getAllVelocities(float* velocities) {
    if (velocities == nullptr) return;
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        velocities[i] = current_velocities_[i];
    }
}

bool DCMotorController::isEnabled(uint8_t axis) {
    if (axis >= NUM_JOINTS) return false;
    return enabled_[axis];
}

MotorStatus DCMotorController::getStatus(uint8_t axis) {
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
    status.target_velocity = target_velocities_[axis];
    status.position_error = target_positions_[axis] - current_positions_[axis];
    
    status.at_limit = (current_positions_[axis] <= config.min_position + 0.5f) ||
                      (current_positions_[axis] >= config.max_position - 0.5f);
    
    status.fault = false;
    status.fault_code = 0;
    
    return status;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

bool DCMotorController::setControlMode(uint8_t axis, MotorControlMode mode) {
    if (axis >= NUM_JOINTS) return false;
    
    // DC motors support all modes
    control_mode_[axis] = mode;
    
    // Reset appropriate PID on mode change
    if (mode == MotorControlMode::POSITION) {
        position_pid_[axis].reset();
    } else if (mode == MotorControlMode::VELOCITY) {
        velocity_pid_[axis].reset();
    }
    
    return true;
}

MotorType DCMotorController::getType() {
    return MotorType::DC_ENCODER;
}

// ============================================================================
// UPDATE LOOP (CALLED FROM CONTROL TASK)
// ============================================================================

void DCMotorController::update() {
    uint32_t now = millis();
    
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        if (!enabled_[i]) continue;
        
        // Calculate time delta
        float dt = (now - last_update_time_[i]) / 1000.0f;
        if (dt < 0.001f) continue;  // Skip if too fast
        
        // Update position from encoder
        updatePositionFromEncoder(i);
        
        // Calculate velocity
        float velocity = (current_positions_[i] - last_positions_[i]) / dt;
        current_velocities_[i] = velocity;
        last_positions_[i] = current_positions_[i];
        
        // Run control loop based on mode
        float output = 0.0f;
        
        switch (control_mode_[i]) {
            case MotorControlMode::POSITION:
                output = position_pid_[i].compute(
                    target_positions_[i], 
                    current_positions_[i], 
                    dt
                );
                break;
                
            case MotorControlMode::VELOCITY:
                output = velocity_pid_[i].compute(
                    target_velocities_[i], 
                    current_velocities_[i], 
                    dt
                );
                break;
                
            case MotorControlMode::TORQUE:
                // Torque mode: output directly proportional to target
                // (simplified - real implementation would use current sensing)
                output = target_velocities_[i];  // Reuse velocity as torque command
                break;
        }
        
        // Apply motor output
        setMotorPWM(i, output);
        
        last_update_time_[i] = now;
    }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void DCMotorController::setMotorPWM(uint8_t axis, float output) {
    if (axis >= NUM_JOINTS) return;
    
    // Determine direction
    bool forward = (output >= 0);
    float pwm_value = fabs(output);
    
    // Constrain PWM to valid range
    if (pwm_value > 255.0f) pwm_value = 255.0f;
    
    // Set direction pins
    if (pins_[axis].dir_a_pin > 0 && pins_[axis].dir_b_pin > 0) {
        if (forward) {
            digitalWrite(pins_[axis].dir_a_pin, HIGH);
            digitalWrite(pins_[axis].dir_b_pin, LOW);
        } else {
            digitalWrite(pins_[axis].dir_a_pin, LOW);
            digitalWrite(pins_[axis].dir_b_pin, HIGH);
        }
    }
    
    // Set PWM
    if (pins_[axis].pwm_pin > 0) {
        analogWrite(pins_[axis].pwm_pin, (int)pwm_value);
    }
}

void DCMotorController::updatePositionFromEncoder(uint8_t axis) {
    if (axis >= NUM_JOINTS) return;
    
    // Get encoder counts from EncoderReader
    int32_t counts = EncoderReader::getInstance().getCount(axis);
    
    // Convert to angle
    current_positions_[axis] = encoderToAngle(axis, counts);
}

float DCMotorController::encoderToAngle(uint8_t axis, int32_t counts) {
    AxisMotorConfig& config = MotorConfigManager::getInstance().getAxisConfig(axis);
    
    // counts / counts_per_rev * 360 / gear_ratio
    float motor_revs = (float)counts / config.encoder_counts_per_rev;
    float output_revs = motor_revs / config.gear_ratio;
    float angle = output_revs * 360.0f;
    
    if (config.invert_encoder) {
        angle = -angle;
    }
    
    return angle;
}

float DCMotorController::constrainAngle(uint8_t axis, float angle) {
    AxisMotorConfig& config = MotorConfigManager::getInstance().getAxisConfig(axis);
    
    if (angle < config.min_position) return config.min_position;
    if (angle > config.max_position) return config.max_position;
    
    return angle;
}
