/**
 * @file NumericalIK.cpp
 * @brief Numerical IK solver using Jacobian-based iteration
 * 
 * Uses damped least squares (Levenberg-Marquardt) for robustness
 * near singularities. Most flexible solver but slower than analytical.
 */

#include "../include/ik/IKSolver.h"
#include "../include/RobotCore.h"
#include <math.h>

// ============================================================================
// NUMERICAL IK SOLVER IMPLEMENTATION
// ============================================================================

class NumericalIKSolver : public IIKSolver {
public:
    static NumericalIKSolver& getInstance() {
        static NumericalIKSolver instance;
        return instance;
    }
    
    IKResult solve(const float* target_pose, const float* seed_angles) override;
    bool validateJointLimits(const float* angles) override;
    float getSingularityMetric(const float* angles) override;
    IKSolverType getType() override { return IKSolverType::NUMERICAL; }
    const char* getName() override { return "Numerical"; }
    
    // Configuration
    void setMaxIterations(uint16_t iterations) { max_iterations_ = iterations; }
    void setConvergenceThreshold(float threshold) { convergence_threshold_ = threshold; }
    void setDampingFactor(float damping) { damping_factor_ = damping; }

private:
    NumericalIKSolver();
    
    // Solver parameters
    uint16_t max_iterations_;
    float convergence_threshold_;
    float damping_factor_;
    
    // Static arrays for computation (no dynamic allocation)
    float jacobian_[6][NUM_JOINTS];
    float jacobian_transpose_[NUM_JOINTS][6];
    float jtj_[NUM_JOINTS][NUM_JOINTS];
    float jtj_damped_[NUM_JOINTS][NUM_JOINTS];
    float jte_[NUM_JOINTS];
    float delta_theta_[NUM_JOINTS];
    
    // Helper functions
    void computeJacobian(const float* angles);
    void computeForwardKinematics(const float* angles, float* pose);
    float computeError(const float* current, const float* target);
    bool solveLinearSystem(float delta[NUM_JOINTS], const float jtj[NUM_JOINTS][NUM_JOINTS], 
                          const float jte[NUM_JOINTS]);
    float normalizeAngle(float angle);
    void clampToLimits(float* angles);
};

// ============================================================================
// CONSTRUCTOR
// ============================================================================

NumericalIKSolver::NumericalIKSolver() {
    max_iterations_ = 100;
    convergence_threshold_ = 0.1f;   // 0.1mm
    damping_factor_ = 0.01f;
    
    // Initialize arrays to zero
    memset(jacobian_, 0, sizeof(jacobian_));
    memset(jacobian_transpose_, 0, sizeof(jacobian_transpose_));
    memset(jtj_, 0, sizeof(jtj_));
    memset(jtj_damped_, 0, sizeof(jtj_damped_));
    memset(jte_, 0, sizeof(jte_));
    memset(delta_theta_, 0, sizeof(delta_theta_));
}

// ============================================================================
// SOLVE IMPLEMENTATION
// ============================================================================

IKResult NumericalIKSolver::solve(const float* target_pose, const float* seed_angles) {
    IKResult result;
    
    if (target_pose == nullptr) {
        result.feasible = false;
        result.error_code = IKError::INVALID_INPUT;
        strncpy(result.error_message, "Null target pose", 31);
        return result;
    }
    
    // Initialize with seed angles or zeros
    float current_angles[NUM_JOINTS];
    if (seed_angles != nullptr) {
        memcpy(current_angles, seed_angles, sizeof(float) * NUM_JOINTS);
    } else {
        memset(current_angles, 0, sizeof(current_angles));
    }
    
    float current_pose[6];
    float error[6];
    float last_error = 9999.0f;
    bool converged = false;
    
    // ========== Iterative Solution ==========
    for (uint16_t iter = 0; iter < max_iterations_; iter++) {
        // Compute forward kinematics
        computeForwardKinematics(current_angles, current_pose);
        
        // Compute error
        for (int i = 0; i < 6; i++) {
            error[i] = target_pose[i] - current_pose[i];
        }
        
        float current_error = computeError(current_pose, target_pose);
        
        // Check convergence
        if (current_error < convergence_threshold_) {
            converged = true;
            break;
        }
        
        // Check for divergence
        if (current_error > last_error * 2.0f && iter > 5) {
            // Diverging - try reducing step size
            damping_factor_ *= 2.0f;
            if (damping_factor_ > 1.0f) {
                // Cannot converge
                result.feasible = false;
                result.error_code = IKError::NO_CONVERGENCE;
                strncpy(result.error_message, "IK diverged", 31);
                return result;
            }
        }
        
        last_error = current_error;
        
        // Compute Jacobian
        computeJacobian(current_angles);
        
        // Compute J^T * J
        for (int i = 0; i < NUM_JOINTS; i++) {
            for (int j = 0; j < NUM_JOINTS; j++) {
                jtj_[i][j] = 0.0f;
                for (int k = 0; k < 6; k++) {
                    jtj_[i][j] += jacobian_[k][i] * jacobian_[k][j];
                }
                // Add damping to diagonal (Levenberg-Marquardt)
                if (i == j) {
                    jtj_damped_[i][j] = jtj_[i][j] + damping_factor_;
                } else {
                    jtj_damped_[i][j] = jtj_[i][j];
                }
            }
        }
        
        // Compute J^T * error
        for (int i = 0; i < NUM_JOINTS; i++) {
            jte_[i] = 0.0f;
            for (int k = 0; k < 6; k++) {
                jte_[i] += jacobian_[k][i] * error[k];
            }
        }
        
        // Solve (J^T*J + λI) * Δθ = J^T * e
        if (!solveLinearSystem(delta_theta_, jtj_damped_, jte_)) {
            result.feasible = false;
            result.error_code = IKError::SINGULARITY;
            strncpy(result.error_message, "Singular matrix", 31);
            return result;
        }
        
        // Update angles
        for (int i = 0; i < NUM_JOINTS; i++) {
            current_angles[i] += delta_theta_[i];
            current_angles[i] = normalizeAngle(current_angles[i]);
        }
        
        // Clamp to joint limits
        clampToLimits(current_angles);
    }
    
    // ========== Validate Result ==========
    if (!converged) {
        result.feasible = false;
        result.error_code = IKError::NO_CONVERGENCE;
        strncpy(result.error_message, "Max iterations reached", 31);
        result.error_metric = last_error;
        memcpy(result.joint_angles, current_angles, sizeof(float) * NUM_JOINTS);
        return result;
    }
    
    // Copy solution
    memcpy(result.joint_angles, current_angles, sizeof(float) * NUM_JOINTS);
    
    // Validate joint limits
    if (!validateJointLimits(result.joint_angles)) {
        result.feasible = false;
        result.error_code = IKError::JOINT_LIMIT;
        strncpy(result.error_message, "Joint limit exceeded", 31);
        result.joint_limit_warning = true;
        
        for (int i = 0; i < NUM_JOINTS; i++) {
            JointLimits limits = RobotConfig::getInstance().getJointLimits(i);
            if (result.joint_angles[i] < limits.min_position ||
                result.joint_angles[i] > limits.max_position) {
                result.limiting_joint = i;
                break;
            }
        }
        return result;
    }
    
    // Set success
    result.feasible = true;
    result.error_metric = last_error;
    result.confidence = 1.0f - (last_error / convergence_threshold_) * 0.1f;
    if (result.confidence < 0.5f) result.confidence = 0.5f;
    if (result.confidence > 1.0f) result.confidence = 1.0f;
    
    // Check singularity
    result.singularity_metric = getSingularityMetric(result.joint_angles);
    if (result.singularity_metric < 0.1f) {
        result.singularity_warning = true;
    }
    
    // Velocity suggestions based on joint movement
    for (int i = 0; i < NUM_JOINTS; i++) {
        float movement = fabs(result.joint_angles[i] - 
                             (seed_angles ? seed_angles[i] : 0.0f));
        result.velocity_suggestions[i] = max(30.0f, min(120.0f, movement * 2.0f));
    }
    
    return result;
}

// ============================================================================
// JACOBIAN COMPUTATION
// ============================================================================

void NumericalIKSolver::computeJacobian(const float* angles) {
    // Numerical Jacobian using finite differences
    const float delta = 0.01f;  // Small angle increment (degrees)
    float pose_plus[6], pose_minus[6];
    float angles_test[NUM_JOINTS];
    
    memcpy(angles_test, angles, sizeof(float) * NUM_JOINTS);
    
    for (int j = 0; j < NUM_JOINTS; j++) {
        // Positive perturbation
        angles_test[j] = angles[j] + delta;
        computeForwardKinematics(angles_test, pose_plus);
        
        // Negative perturbation
        angles_test[j] = angles[j] - delta;
        computeForwardKinematics(angles_test, pose_minus);
        
        // Reset
        angles_test[j] = angles[j];
        
        // Finite difference
        for (int i = 0; i < 6; i++) {
            jacobian_[i][j] = (pose_plus[i] - pose_minus[i]) / (2.0f * delta);
        }
    }
}

// ============================================================================
// FORWARD KINEMATICS
// ============================================================================

void NumericalIKSolver::computeForwardKinematics(const float* angles, float* pose) {
    // Use existing Kinematics class
    Kinematics::getInstance().forwardKinematics(angles, pose);
}

// ============================================================================
// ERROR COMPUTATION
// ============================================================================

float NumericalIKSolver::computeError(const float* current, const float* target) {
    float pos_error = 0.0f;
    for (int i = 0; i < 3; i++) {
        float diff = target[i] - current[i];
        pos_error += diff * diff;
    }
    return sqrt(pos_error);
}

// ============================================================================
// LINEAR SYSTEM SOLVER (Gauss-Jordan)
// ============================================================================

bool NumericalIKSolver::solveLinearSystem(float delta[NUM_JOINTS], 
                                          const float jtj[NUM_JOINTS][NUM_JOINTS],
                                          const float jte[NUM_JOINTS]) {
    // Simple Gauss-Jordan elimination
    // Static arrays for augmented matrix
    float aug[NUM_JOINTS][NUM_JOINTS + 1];
    
    // Build augmented matrix
    for (int i = 0; i < NUM_JOINTS; i++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            aug[i][j] = jtj[i][j];
        }
        aug[i][NUM_JOINTS] = jte[i];
    }
    
    // Forward elimination with partial pivoting
    for (int k = 0; k < NUM_JOINTS; k++) {
        // Find pivot
        int max_row = k;
        float max_val = fabs(aug[k][k]);
        for (int i = k + 1; i < NUM_JOINTS; i++) {
            if (fabs(aug[i][k]) > max_val) {
                max_val = fabs(aug[i][k]);
                max_row = i;
            }
        }
        
        // Check for singular matrix
        if (max_val < 1e-10f) {
            return false;
        }
        
        // Swap rows
        if (max_row != k) {
            for (int j = k; j <= NUM_JOINTS; j++) {
                float temp = aug[k][j];
                aug[k][j] = aug[max_row][j];
                aug[max_row][j] = temp;
            }
        }
        
        // Eliminate column
        for (int i = k + 1; i < NUM_JOINTS; i++) {
            float factor = aug[i][k] / aug[k][k];
            for (int j = k; j <= NUM_JOINTS; j++) {
                aug[i][j] -= factor * aug[k][j];
            }
        }
    }
    
    // Back substitution
    for (int i = NUM_JOINTS - 1; i >= 0; i--) {
        delta[i] = aug[i][NUM_JOINTS];
        for (int j = i + 1; j < NUM_JOINTS; j++) {
            delta[i] -= aug[i][j] * delta[j];
        }
        delta[i] /= aug[i][i];
    }
    
    return true;
}

// ============================================================================
// VALIDATION FUNCTIONS
// ============================================================================

bool NumericalIKSolver::validateJointLimits(const float* angles) {
    if (angles == nullptr) return false;
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        JointLimits limits = RobotConfig::getInstance().getJointLimits(i);
        if (angles[i] < limits.min_position || angles[i] > limits.max_position) {
            return false;
        }
    }
    return true;
}

float NumericalIKSolver::getSingularityMetric(const float* angles) {
    // Compute condition number of Jacobian as singularity metric
    computeJacobian(angles);
    
    // Simplified: check diagonal dominance
    float min_diag = 9999.0f;
    float max_diag = 0.0f;
    
    for (int i = 0; i < min(6, NUM_JOINTS); i++) {
        float diag = fabs(jacobian_[i][i]);
        if (diag < min_diag) min_diag = diag;
        if (diag > max_diag) max_diag = diag;
    }
    
    if (max_diag < 1e-6f) return 0.0f;
    
    float ratio = min_diag / max_diag;
    return ratio;  // 0 = singular, 1 = well-conditioned
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

float NumericalIKSolver::normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void NumericalIKSolver::clampToLimits(float* angles) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        JointLimits limits = RobotConfig::getInstance().getJointLimits(i);
        if (angles[i] < limits.min_position) angles[i] = limits.min_position;
        if (angles[i] > limits.max_position) angles[i] = limits.max_position;
    }
}
