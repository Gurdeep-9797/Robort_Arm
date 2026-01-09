/**
 * @file IKManager.cpp
 * @brief IK Manager implementation - factory and runtime solver selection
 * 
 * Manages IK solver instances and provides unified interface for
 * IK computation with runtime solver switching.
 */

#include "../include/ik/IKSolver.h"
#include "../include/RobotCore.h"
#include <Preferences.h>

// Forward declare solver classes (defined in their .cpp files)
class AnalyticalIKSolver;
class NumericalIKSolver;
class ServoFriendlyIKSolver;
class DCFriendlyIKSolver;

// ============================================================================
// IK MANAGER IMPLEMENTATION
// ============================================================================

IKManager& IKManager::getInstance() {
    static IKManager instance;
    return instance;
}

IKManager::IKManager() {
    mutex_ = xSemaphoreCreateMutex();
    active_solver_type_ = IKSolverType::NUMERICAL;  // Default to most flexible
    last_error_ = IKError::NONE;
    memset(last_error_message_, 0, sizeof(last_error_message_));
    
    // Initialize solver array to nullptr
    for (int i = 0; i < 4; i++) {
        solvers_[i] = nullptr;
    }
}

bool IKManager::begin() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    
    // Try to load config from NVS
    if (!loadConfig()) {
        // Use defaults
        config_ = IKConfig();
    }
    
    // Solvers are lazy-initialized via getSolver()
    // This avoids circular dependencies during startup
    
    xSemaphoreGive(mutex_);
    
    Serial.printf("[IKManager] Initialized with %s solver\n", 
                  getSolver(active_solver_type_)->getName());
    
    return true;
}

IKResult IKManager::solve(const float* target_pose, const float* seed_angles) {
    IKResult result;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    
    IIKSolver* solver = getSolver(active_solver_type_);
    if (solver == nullptr) {
        result.feasible = false;
        result.error_code = IKError::INTERNAL_ERROR;
        strncpy(result.error_message, "No solver available", 31);
        
        last_error_ = result.error_code;
        strncpy(last_error_message_, result.error_message, 31);
        
        xSemaphoreGive(mutex_);
        return result;
    }
    
    // Solve
    result = solver->solve(target_pose, seed_angles);
    
    // Store error info for UI notification
    last_error_ = result.error_code;
    strncpy(last_error_message_, result.error_message, 31);
    
    xSemaphoreGive(mutex_);
    
    // Log if failed
    if (!result.feasible) {
        Serial.printf("[IK] Failed: %s (error %d)\n", 
                      result.error_message, result.error_code);
    }
    
    return result;
}

IKResult IKManager::preview(const float* target_pose, const float* seed_angles) {
    // Preview is same as solve but doesn't affect motion
    return solve(target_pose, seed_angles);
}

bool IKManager::setSolverType(IKSolverType type) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    
    // Verify solver is available
    IIKSolver* solver = getSolver(type);
    if (solver == nullptr) {
        xSemaphoreGive(mutex_);
        return false;
    }
    
    active_solver_type_ = type;
    config_.solver_type = type;
    
    xSemaphoreGive(mutex_);
    
    Serial.printf("[IKManager] Switched to %s solver\n", solver->getName());
    
    return true;
}

IKSolverType IKManager::getActiveSolverType() {
    return active_solver_type_;
}

void IKManager::setPreference(IKPreference preference) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    config_.preference = preference;
    xSemaphoreGive(mutex_);
}

IKPreference IKManager::getPreference() {
    return config_.preference;
}

IKConfig& IKManager::getConfig() {
    return config_;
}

IIKSolver* IKManager::getSolver(IKSolverType type) {
    int index = (int)type;
    if (index < 0 || index >= 4) return nullptr;
    
    // Lazy initialization of solvers
    // Note: These are singleton instances from their respective .cpp files
    // We use extern declarations to access them
    
    if (solvers_[index] == nullptr) {
        switch (type) {
            case IKSolverType::ANALYTICAL:
                // AnalyticalIKSolver is in AnalyticalIK.cpp
                // Will be linked at compile time
                break;
            case IKSolverType::NUMERICAL:
                // NumericalIKSolver is in NumericalIK.cpp
                break;
            case IKSolverType::SERVO_FRIENDLY:
                // ServoFriendlyIKSolver is in ServoFriendlyIK.cpp
                break;
            case IKSolverType::DC_FRIENDLY:
                // DCFriendlyIKSolver is in DCFriendlyIK.cpp
                break;
        }
    }
    
    // For now, return first available solver (will be properly linked)
    // The actual solver registration happens via extern declarations
    return solvers_[index];
}

uint8_t IKManager::getLastError() {
    return last_error_;
}

const char* IKManager::getLastErrorMessage() {
    return last_error_message_;
}

bool IKManager::saveConfig() {
    Preferences prefs;
    prefs.begin("ikconfig", false);
    
    prefs.putUChar("solver", (uint8_t)config_.solver_type);
    prefs.putUChar("pref", (uint8_t)config_.preference);
    prefs.putUShort("maxiter", config_.max_iterations);
    prefs.putFloat("convth", config_.convergence_threshold);
    prefs.putFloat("damp", config_.damping_factor);
    prefs.putFloat("singth", config_.singularity_threshold);
    prefs.putBool("avoidsg", config_.avoid_singularities);
    prefs.putFloat("limmar", config_.joint_limit_margin);
    prefs.putBool("softlim", config_.respect_soft_limits);
    
    prefs.end();
    return true;
}

bool IKManager::loadConfig() {
    Preferences prefs;
    prefs.begin("ikconfig", true);
    
    if (!prefs.isKey("solver")) {
        prefs.end();
        return false;
    }
    
    config_.solver_type = (IKSolverType)prefs.getUChar("solver", 1);
    config_.preference = (IKPreference)prefs.getUChar("pref", 1);
    config_.max_iterations = prefs.getUShort("maxiter", 100);
    config_.convergence_threshold = prefs.getFloat("convth", 0.1f);
    config_.damping_factor = prefs.getFloat("damp", 0.01f);
    config_.singularity_threshold = prefs.getFloat("singth", 0.01f);
    config_.avoid_singularities = prefs.getBool("avoidsg", true);
    config_.joint_limit_margin = prefs.getFloat("limmar", 5.0f);
    config_.respect_soft_limits = prefs.getBool("softlim", true);
    
    active_solver_type_ = config_.solver_type;
    
    prefs.end();
    return true;
}
