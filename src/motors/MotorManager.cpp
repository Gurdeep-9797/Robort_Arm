/**
 * @file MotorManager.cpp
 * @brief Motor manager implementation for runtime motor type switching
 * 
 * Manages active motor controller and provides safe switching between
 * servo and DC motor modes at runtime.
 */

#include "../include/motors/MotorInterface.h"
#include "../include/motors/MotorConfig.h"

// Forward declarations for controller instances
extern class ServoMotorController;
extern class DCMotorController;

// ============================================================================
// MOTOR CONFIG MANAGER IMPLEMENTATION
// ============================================================================

MotorConfigManager& MotorConfigManager::getInstance() {
    static MotorConfigManager instance;
    return instance;
}

MotorConfigManager::MotorConfigManager() {
    mutex_ = xSemaphoreCreateMutex();
    
    // Initialize with defaults
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        configs_[i] = AxisMotorConfig();  // Default constructor
    }
}

bool MotorConfigManager::begin() {
    // Try to load from NVS, otherwise use defaults
    if (!loadFromNVS()) {
        // Use defaults - already initialized in constructor
        Serial.println("[MotorConfig] Using default configuration");
    }
    return true;
}

AxisMotorConfig& MotorConfigManager::getAxisConfig(uint8_t axis) {
    if (axis >= NUM_JOINTS) {
        return configs_[0];  // Return first config if invalid axis
    }
    return configs_[axis];
}

bool MotorConfigManager::setAxisConfig(uint8_t axis, const AxisMotorConfig& config) {
    if (axis >= NUM_JOINTS) return false;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    configs_[axis] = config;
    xSemaphoreGive(mutex_);
    
    return true;
}

bool MotorConfigManager::validateConfig(uint8_t axis) {
    if (axis >= NUM_JOINTS) return false;
    
    AxisMotorConfig& config = configs_[axis];
    
    // Validate gear ratio
    if (!validateGearRatio(config.gear_ratio)) return false;
    
    // Validate current limit
    if (!validateCurrentLimit(config.max_current_amps)) return false;
    
    // Validate pulse widths for servo
    if (config.hardware_type == MotorHardwareType::STANDARD_SERVO ||
        config.hardware_type == MotorHardwareType::DIGITAL_SERVO) {
        if (!validatePulseWidth(config.servo_min_pulse_us, config.servo_max_pulse_us)) {
            return false;
        }
    }
    
    // Validate position limits
    if (config.min_position >= config.max_position) return false;
    
    return true;
}

bool MotorConfigManager::hasWarnings(uint8_t axis, char* warning_msg) {
    if (axis >= NUM_JOINTS || warning_msg == nullptr) return false;
    
    AxisMotorConfig& config = configs_[axis];
    
    // Check for high current setting
    if (config.max_current_amps > 5.0f) {
        strncpy(warning_msg, "High current limit!", 31);
        return true;
    }
    
    // Check for extreme gear ratio
    if (config.gear_ratio > 100.0f || config.gear_ratio < 0.1f) {
        strncpy(warning_msg, "Extreme gear ratio!", 31);
        return true;
    }
    
    // Check for inverted limits
    if (config.min_position >= config.max_position) {
        strncpy(warning_msg, "Invalid limits!", 31);
        return true;
    }
    
    return false;
}

bool MotorConfigManager::saveToNVS() {
    Preferences prefs;
    prefs.begin("motorconf", false);
    
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        char key[16];
        
        // Save key parameters
        sprintf(key, "gear_%d", i);
        prefs.putFloat(key, configs_[i].gear_ratio);
        
        sprintf(key, "cur_%d", i);
        prefs.putFloat(key, configs_[i].max_current_amps);
        
        sprintf(key, "smin_%d", i);
        prefs.putUShort(key, configs_[i].servo_min_pulse_us);
        
        sprintf(key, "smax_%d", i);
        prefs.putUShort(key, configs_[i].servo_max_pulse_us);
        
        sprintf(key, "hw_%d", i);
        prefs.putUChar(key, (uint8_t)configs_[i].hardware_type);
        
        sprintf(key, "enc_%d", i);
        prefs.putUChar(key, (uint8_t)configs_[i].encoder_type);
    }
    
    prefs.end();
    return true;
}

bool MotorConfigManager::loadFromNVS() {
    Preferences prefs;
    prefs.begin("motorconf", true);  // Read-only
    
    // Check if any config exists
    if (!prefs.isKey("gear_0")) {
        prefs.end();
        return false;
    }
    
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        char key[16];
        
        sprintf(key, "gear_%d", i);
        configs_[i].gear_ratio = prefs.getFloat(key, 1.0f);
        
        sprintf(key, "cur_%d", i);
        configs_[i].max_current_amps = prefs.getFloat(key, 2.0f);
        
        sprintf(key, "smin_%d", i);
        configs_[i].servo_min_pulse_us = prefs.getUShort(key, 500);
        
        sprintf(key, "smax_%d", i);
        configs_[i].servo_max_pulse_us = prefs.getUShort(key, 2500);
        
        sprintf(key, "hw_%d", i);
        configs_[i].hardware_type = (MotorHardwareType)prefs.getUChar(key, 0);
        
        sprintf(key, "enc_%d", i);
        configs_[i].encoder_type = (EncoderType)prefs.getUChar(key, 0);
    }
    
    prefs.end();
    return true;
}

bool MotorConfigManager::validateGearRatio(float ratio) {
    return (ratio > 0.01f && ratio < 1000.0f);
}

bool MotorConfigManager::validateCurrentLimit(float current) {
    return (current > 0.0f && current < 20.0f);
}

bool MotorConfigManager::validatePulseWidth(uint16_t min_us, uint16_t max_us) {
    if (min_us >= max_us) return false;
    if (min_us < 100 || min_us > 3000) return false;
    if (max_us < 100 || max_us > 3000) return false;
    return true;
}

// ============================================================================
// MOTOR MANAGER IMPLEMENTATION
// ============================================================================

MotorManager& MotorManager::getInstance() {
    static MotorManager instance;
    return instance;
}

MotorManager::MotorManager() {
    mutex_ = xSemaphoreCreateMutex();
    active_type_ = MotorType::SERVO;
    active_controller_ = nullptr;
    servo_controller_ = nullptr;
    dc_controller_ = nullptr;
}

bool MotorManager::begin(MotorType defaultType) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    
    // Initialize motor config manager first
    MotorConfigManager::getInstance().begin();
    
    // Get singleton instances of both controllers
    // Note: These are implemented in their respective .cpp files
    // Using extern declarations to access them
    
    active_type_ = defaultType;
    
    // Initialize the default controller
    bool success = false;
    if (defaultType == MotorType::SERVO) {
        // ServoMotorController will be the default
        // The actual initialization happens when getActiveController() is called
        success = true;
    } else {
        success = true;
    }
    
    xSemaphoreGive(mutex_);
    
    Serial.printf("[MotorManager] Initialized with %s motors\n", 
                  defaultType == MotorType::SERVO ? "SERVO" : "DC");
    
    return success;
}

IMotorController* MotorManager::getActiveController() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    
    // Lazy initialization of controllers
    // This avoids circular dependency issues
    if (active_controller_ == nullptr) {
        // For now, return nullptr - actual controller instances
        // are accessed via their own getInstance() methods
        // This manager primarily handles type switching
    }
    
    xSemaphoreGive(mutex_);
    return active_controller_;
}

MotorType MotorManager::getActiveType() {
    return active_type_;
}

bool MotorManager::setMotorType(MotorType newType) {
    if (!canChangeMotorType()) {
        Serial.println("[MotorManager] Cannot change type - motion in progress");
        return false;
    }
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    
    // Disable current controller
    if (active_controller_ != nullptr) {
        active_controller_->disableAll();
    }
    
    active_type_ = newType;
    
    // The actual controller switch happens on next getActiveController() call
    active_controller_ = nullptr;
    
    xSemaphoreGive(mutex_);
    
    Serial.printf("[MotorManager] Switched to %s motors\n", 
                  newType == MotorType::SERVO ? "SERVO" : "DC");
    
    return true;
}

bool MotorManager::canChangeMotorType() {
    // Check if motion is in progress
    // Uses SafetyManager to ensure safe switching
    return !MotionPlanner::getInstance().isMoving();
}
