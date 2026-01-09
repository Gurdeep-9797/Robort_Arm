/**
 * @file MotorInterface.h
 * @brief Common interface for all motor controller types
 * 
 * SAFETY-CRITICAL: This interface defines the contract for motor control.
 * All implementations MUST validate limits and fail safely.
 * 
 * @note No dynamic memory allocation - all arrays are statically sized
 * @note Deterministic behavior required for real-time control
 */

#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <Arduino.h>
#include "../RobotCore.h"

// ============================================================================
// MOTOR TYPE ENUMERATION
// ============================================================================

enum class MotorType : uint8_t {
    SERVO = 0,      // PWM-based servo motor (PCA9685)
    DC_ENCODER = 1  // DC motor with encoder feedback
};

// ============================================================================
// MOTOR STATUS STRUCTURE
// ============================================================================

struct MotorStatus {
    bool enabled;
    float current_position;     // Degrees
    float current_velocity;     // Degrees/second
    float target_position;      // Degrees
    float target_velocity;      // Degrees/second
    float position_error;       // Degrees
    bool at_limit;
    bool fault;
    uint8_t fault_code;
};

// ============================================================================
// MOTOR CONTROL MODE
// ============================================================================

enum class MotorControlMode : uint8_t {
    POSITION = 0,   // Position control (default for servos)
    VELOCITY = 1,   // Velocity control
    TORQUE = 2      // Torque/current control (DC motors only)
};

// ============================================================================
// MOTOR CONTROLLER INTERFACE
// ============================================================================

/**
 * @interface IMotorController
 * @brief Abstract interface for motor controllers
 * 
 * Implementations:
 * - ServoMotorController (PCA9685-based)
 * - DCMotorController (Encoder + H-bridge)
 */
class IMotorController {
public:
    virtual ~IMotorController() = default;
    
    // ========== Lifecycle ==========
    
    /**
     * @brief Initialize the motor controller
     * @return true if initialization successful
     */
    virtual bool begin() = 0;
    
    // ========== Enable/Disable ==========
    
    /**
     * @brief Enable a specific motor axis
     * @param axis Joint index (0 to NUM_JOINTS-1)
     */
    virtual void enable(uint8_t axis) = 0;
    
    /**
     * @brief Disable a specific motor axis
     * @param axis Joint index (0 to NUM_JOINTS-1)
     */
    virtual void disable(uint8_t axis) = 0;
    
    /**
     * @brief Enable all motor axes
     */
    virtual void enableAll() = 0;
    
    /**
     * @brief Disable all motor axes
     */
    virtual void disableAll() = 0;
    
    // ========== Emergency Stop (SAFETY-CRITICAL) ==========
    
    /**
     * @brief Immediately stop all motors and disable
     * CRITICAL: Must execute within 1ms
     * Must NOT throw exceptions or block
     */
    virtual void emergencyStop() = 0;
    
    // ========== Target Setting ==========
    
    /**
     * @brief Set target position for an axis
     * @param axis Joint index
     * @param position Target position in degrees
     * @param velocity Maximum velocity in degrees/second (0 = default)
     * @return true if target accepted (within limits)
     */
    virtual bool setTarget(uint8_t axis, float position, float velocity = 0) = 0;
    
    /**
     * @brief Set target positions for all axes
     * @param positions Array of NUM_JOINTS positions in degrees
     * @param velocity Maximum velocity in degrees/second
     * @return true if all targets accepted
     */
    virtual bool setAllTargets(const float* positions, float velocity = 0) = 0;
    
    // ========== State Queries ==========
    
    /**
     * @brief Get current position of an axis
     * @param axis Joint index
     * @return Current position in degrees
     */
    virtual float getPosition(uint8_t axis) = 0;
    
    /**
     * @brief Get current velocity of an axis
     * @param axis Joint index
     * @return Current velocity in degrees/second
     */
    virtual float getVelocity(uint8_t axis) = 0;
    
    /**
     * @brief Get all current positions
     * @param positions Output array of NUM_JOINTS positions
     */
    virtual void getAllPositions(float* positions) = 0;
    
    /**
     * @brief Get all current velocities
     * @param velocities Output array of NUM_JOINTS velocities
     */
    virtual void getAllVelocities(float* velocities) = 0;
    
    /**
     * @brief Check if an axis is enabled
     * @param axis Joint index
     * @return true if enabled
     */
    virtual bool isEnabled(uint8_t axis) = 0;
    
    /**
     * @brief Get detailed status of an axis
     * @param axis Joint index
     * @return MotorStatus structure
     */
    virtual MotorStatus getStatus(uint8_t axis) = 0;
    
    // ========== Configuration ==========
    
    /**
     * @brief Set control mode for an axis
     * @param axis Joint index
     * @param mode Control mode
     * @return true if mode supported and set
     */
    virtual bool setControlMode(uint8_t axis, MotorControlMode mode) = 0;
    
    /**
     * @brief Get motor type
     * @return MotorType enum value
     */
    virtual MotorType getType() = 0;
    
    // ========== Update Loop ==========
    
    /**
     * @brief Called from control loop to update motor state
     * CRITICAL: Must be deterministic and complete within timing budget
     */
    virtual void update() = 0;
};

// ============================================================================
// MOTOR MANAGER (FACTORY / SELECTOR)
// ============================================================================

/**
 * @class MotorManager
 * @brief Singleton managing active motor controller
 * 
 * Provides runtime switching between motor types without code changes.
 * UI can request motor type change; MotorManager handles safe transition.
 */
class MotorManager {
public:
    static MotorManager& getInstance();
    
    /**
     * @brief Initialize motor manager with default motor type
     * @param defaultType Initial motor type to use
     * @return true if initialization successful
     */
    bool begin(MotorType defaultType = MotorType::SERVO);
    
    /**
     * @brief Get active motor controller
     * @return Pointer to current IMotorController implementation
     */
    IMotorController* getActiveController();
    
    /**
     * @brief Get current motor type
     * @return Active MotorType
     */
    MotorType getActiveType();
    
    /**
     * @brief Request motor type change
     * @param newType Desired motor type
     * @return true if change successful
     * 
     * SAFETY: Type change only allowed when motion is stopped
     */
    bool setMotorType(MotorType newType);
    
    /**
     * @brief Check if motor type change is safe
     * @return true if motors are stopped and change is safe
     */
    bool canChangeMotorType();
    
private:
    MotorManager();
    
    MotorType active_type_;
    IMotorController* active_controller_;
    IMotorController* servo_controller_;
    IMotorController* dc_controller_;
    SemaphoreHandle_t mutex_;
};

#endif // MOTOR_INTERFACE_H
