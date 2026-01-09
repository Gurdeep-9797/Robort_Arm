/**
 * @file IKSolver.h
 * @brief Inverse Kinematics solver interface and result structures
 * 
 * SAFETY-CRITICAL: IK validation is required before any motion execution.
 * Invalid IK must block motion - no exceptions.
 * 
 * Provides pluggable IK architecture with:
 * - Analytical solver (closed-form)
 * - Numerical solver (Jacobian-based)
 * - Servo-friendly solver (position-dominant)
 * - DC-friendly solver (velocity/torque aware)
 */

#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include <Arduino.h>
#include "../RobotCore.h"

// ============================================================================
// IK SOLVER TYPE ENUMERATION
// ============================================================================

enum class IKSolverType : uint8_t {
    ANALYTICAL = 0,     // Closed-form solution (fastest, limited applicability)
    NUMERICAL = 1,      // Jacobian-based iterative (most flexible)
    SERVO_FRIENDLY = 2, // Position-dominant (ignores velocity constraints)
    DC_FRIENDLY = 3     // Velocity/torque aware (respects dynamics)
};

// ============================================================================
// IK PREFERENCE ENUMERATION
// ============================================================================

enum class IKPreference : uint8_t {
    SPEED = 0,      // Minimize computation time
    ACCURACY = 1,   // Minimize position error
    STABILITY = 2   // Avoid singularities and joint limits
};

// ============================================================================
// IK RESULT STRUCTURE
// ============================================================================

/**
 * @struct IKResult
 * @brief Complete result from IK computation
 * 
 * Contains joint angles, velocity suggestions, and validation flags.
 * Motion should ONLY proceed if feasible == true.
 */
struct IKResult {
    // ===== Solution =====
    float joint_angles[NUM_JOINTS];         // Solution joint angles (degrees)
    float velocity_suggestions[NUM_JOINTS]; // Suggested velocities (deg/s)
    
    // ===== Feasibility =====
    bool feasible;              // True if solution is valid and reachable
    float confidence;           // 0.0 - 1.0, confidence in solution
    float error_metric;         // Position error in mm (lower is better)
    
    // ===== Warnings (do not block motion, but inform user) =====
    bool singularity_warning;   // Near a singular configuration
    bool joint_limit_warning;   // Near joint limits
    uint8_t limiting_joint;     // Which joint triggered warning (0-5)
    float singularity_metric;   // 0.0 = singular, 1.0 = far from singular
    
    // ===== Error Info (if feasible == false) =====
    uint8_t error_code;         // Error code for debugging
    char error_message[32];     // Human-readable error
    
    // Default constructor
    IKResult() {
        for (int i = 0; i < NUM_JOINTS; i++) {
            joint_angles[i] = 0.0f;
            velocity_suggestions[i] = 30.0f;  // Default 30 deg/s
        }
        feasible = false;
        confidence = 0.0f;
        error_metric = 9999.0f;
        singularity_warning = false;
        joint_limit_warning = false;
        limiting_joint = 0;
        singularity_metric = 1.0f;
        error_code = 0;
        memset(error_message, 0, 32);
    }
};

// ============================================================================
// IK ERROR CODES
// ============================================================================

namespace IKError {
    constexpr uint8_t NONE = 0;
    constexpr uint8_t OUT_OF_REACH = 1;
    constexpr uint8_t SINGULARITY = 2;
    constexpr uint8_t JOINT_LIMIT = 3;
    constexpr uint8_t NO_CONVERGENCE = 4;
    constexpr uint8_t INVALID_INPUT = 5;
    constexpr uint8_t INTERNAL_ERROR = 6;
}

// ============================================================================
// IK SOLVER INTERFACE
// ============================================================================

/**
 * @interface IIKSolver
 * @brief Abstract interface for IK solvers
 * 
 * All IK solvers must:
 * - Validate joint limits
 * - Detect singularities
 * - Return confidence metrics
 * - Never return invalid solutions as feasible
 */
class IIKSolver {
public:
    virtual ~IIKSolver() = default;
    
    /**
     * @brief Solve inverse kinematics
     * @param target_pose Cartesian target [x, y, z, rx, ry, rz] (mm, degrees)
     * @param seed_angles Initial guess for joint angles (degrees)
     * @return IKResult with solution and validation
     */
    virtual IKResult solve(const float* target_pose, const float* seed_angles) = 0;
    
    /**
     * @brief Validate if joint angles are within limits
     * @param angles Array of joint angles (degrees)
     * @return true if all joints within limits
     */
    virtual bool validateJointLimits(const float* angles) = 0;
    
    /**
     * @brief Calculate singularity metric for given configuration
     * @param angles Array of joint angles (degrees)
     * @return 0.0 = singular, 1.0 = far from singular
     */
    virtual float getSingularityMetric(const float* angles) = 0;
    
    /**
     * @brief Get solver type
     * @return IKSolverType enum value
     */
    virtual IKSolverType getType() = 0;
    
    /**
     * @brief Get solver name for UI display
     * @return Solver name string
     */
    virtual const char* getName() = 0;
};

// ============================================================================
// IK CONFIGURATION STRUCTURE
// ============================================================================

struct IKConfig {
    IKSolverType solver_type;
    IKPreference preference;
    
    // Numerical solver parameters
    uint16_t max_iterations;
    float convergence_threshold;    // mm
    float damping_factor;           // For damped least squares
    
    // Singularity handling
    float singularity_threshold;    // Jacobian condition number threshold
    bool avoid_singularities;       // Enable singularity avoidance
    
    // Joint limit handling
    float joint_limit_margin;       // Degrees from hard limit to trigger warning
    bool respect_soft_limits;       // Use soft limits for IK
    
    // Default constructor
    IKConfig() {
        solver_type = IKSolverType::NUMERICAL;
        preference = IKPreference::ACCURACY;
        max_iterations = 100;
        convergence_threshold = 0.1f;   // 0.1mm
        damping_factor = 0.01f;
        singularity_threshold = 0.01f;
        avoid_singularities = true;
        joint_limit_margin = 5.0f;      // 5 degrees
        respect_soft_limits = true;
    }
};

// ============================================================================
// IK MANAGER (FACTORY / SELECTOR)
// ============================================================================

/**
 * @class IKManager
 * @brief Singleton managing IK solver selection and execution
 * 
 * Provides:
 * - Runtime solver switching
 * - IK preview without motion
 * - IK failure notification to UI
 */
class IKManager {
public:
    static IKManager& getInstance();
    
    /**
     * @brief Initialize IK manager
     * @return true if successful
     */
    bool begin();
    
    /**
     * @brief Solve IK with active solver
     * @param target_pose Cartesian target [x, y, z, rx, ry, rz]
     * @param seed_angles Initial guess
     * @return IKResult
     */
    IKResult solve(const float* target_pose, const float* seed_angles);
    
    /**
     * @brief Preview IK result without executing motion
     * @param target_pose Cartesian target
     * @param seed_angles Initial guess
     * @return IKResult for preview in UI
     */
    IKResult preview(const float* target_pose, const float* seed_angles);
    
    /**
     * @brief Set active solver type
     * @param type Solver type to use
     * @return true if solver available
     */
    bool setSolverType(IKSolverType type);
    
    /**
     * @brief Get active solver type
     * @return Current IKSolverType
     */
    IKSolverType getActiveSolverType();
    
    /**
     * @brief Set IK preference
     * @param preference Speed/Accuracy/Stability
     */
    void setPreference(IKPreference preference);
    
    /**
     * @brief Get current preference
     * @return IKPreference
     */
    IKPreference getPreference();
    
    /**
     * @brief Get IK configuration
     * @return Reference to current config
     */
    IKConfig& getConfig();
    
    /**
     * @brief Get solver by type
     * @param type Solver type
     * @return Pointer to solver (or nullptr if not available)
     */
    IIKSolver* getSolver(IKSolverType type);
    
    /**
     * @brief Get last IK error (for UI notification)
     * @return Last error code
     */
    uint8_t getLastError();
    
    /**
     * @brief Get last error message
     * @return Error message string
     */
    const char* getLastErrorMessage();
    
    /**
     * @brief Save IK config to NVS
     * @return true if successful
     */
    bool saveConfig();
    
    /**
     * @brief Load IK config from NVS
     * @return true if successful
     */
    bool loadConfig();

private:
    IKManager();
    
    IKConfig config_;
    IKSolverType active_solver_type_;
    
    // Static solver instances (no dynamic allocation)
    IIKSolver* solvers_[4];  // One for each solver type
    
    // Last error for UI notification
    uint8_t last_error_;
    char last_error_message_[32];
    
    SemaphoreHandle_t mutex_;
};

// ============================================================================
// FORWARD DECLARATIONS FOR SOLVER IMPLEMENTATIONS
// ============================================================================

class AnalyticalIKSolver;
class NumericalIKSolver;
class ServoFriendlyIKSolver;
class DCFriendlyIKSolver;

#endif // IK_SOLVER_H
