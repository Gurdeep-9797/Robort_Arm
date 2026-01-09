/**
 * @file ServoFriendlyIK.cpp
 * @brief Position-dominant IK solver optimized for servo motors
 * 
 * Ignores velocity constraints and focuses on minimizing position error.
 * Ideal for PWM servos that handle their own speed internally.
 */

#include "../include/ik/IKSolver.h"
#include "../include/RobotCore.h"
#include <math.h>

// ============================================================================
// SERVO-FRIENDLY IK SOLVER IMPLEMENTATION
// ============================================================================

class ServoFriendlyIKSolver : public IIKSolver {
public:
    static ServoFriendlyIKSolver& getInstance() {
        static ServoFriendlyIKSolver instance;
        return instance;
    }
    
    IKResult solve(const float* target_pose, const float* seed_angles) override;
    bool validateJointLimits(const float* angles) override;
    float getSingularityMetric(const float* angles) override;
    IKSolverType getType() override { return IKSolverType::SERVO_FRIENDLY; }
    const char* getName() override { return "Servo-Friendly"; }

private:
    ServoFriendlyIKSolver() {}
    
    float normalizeAngle(float angle);
};

// ============================================================================
// SOLVE IMPLEMENTATION
// ============================================================================

IKResult ServoFriendlyIKSolver::solve(const float* target_pose, const float* seed_angles) {
    IKResult result;
    
    if (target_pose == nullptr) {
        result.feasible = false;
        result.error_code = IKError::INVALID_INPUT;
        strncpy(result.error_message, "Null target pose", 31);
        return result;
    }
    
    float x = target_pose[0];
    float y = target_pose[1];
    float z = target_pose[2];
    
    // ========== Use Analytical Solution for Base Computation ==========
    // Joint 0 (Base rotation)
    float theta0 = atan2(y, x) * 180.0f / PI;
    
    // Arm geometry
    float r = sqrt(x*x + y*y);
    float s = z - 150.0f;  // Base height offset
    float D = sqrt(r*r + s*s);
    
    float L1 = 300.0f;  // Shoulder to elbow
    float L2 = 250.0f;  // Elbow to wrist
    
    // Reachability check
    if (D > L1 + L2 * 0.99f || D < fabs(L1 - L2) * 1.01f) {
        result.feasible = false;
        result.error_code = IKError::OUT_OF_REACH;
        strncpy(result.error_message, "Target unreachable", 31);
        return result;
    }
    
    // Elbow angle (cosine law)
    float cos_theta2 = (D*D - L1*L1 - L2*L2) / (2.0f * L1 * L2);
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;
    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    
    float theta2 = atan2(sqrt(1.0f - cos_theta2*cos_theta2), cos_theta2) * 180.0f / PI;
    
    // Shoulder angle
    float alpha = atan2(s, r);
    float beta = atan2(L2 * sin(theta2 * PI / 180.0f), 
                       L1 + L2 * cos(theta2 * PI / 180.0f));
    float theta1 = (alpha - beta) * 180.0f / PI;
    
    // Wrist angles (default to orientation or zero)
    float theta3 = target_pose[3];
    float theta4 = target_pose[4];
    float theta5 = target_pose[5];
    
    // ========== Store Result ==========
    result.joint_angles[0] = normalizeAngle(theta0);
    result.joint_angles[1] = normalizeAngle(theta1);
    result.joint_angles[2] = normalizeAngle(theta2);
    result.joint_angles[3] = normalizeAngle(theta3);
    result.joint_angles[4] = normalizeAngle(theta4);
    result.joint_angles[5] = normalizeAngle(theta5);
    
    // ========== Validate Limits ==========
    if (!validateJointLimits(result.joint_angles)) {
        result.feasible = false;
        result.error_code = IKError::JOINT_LIMIT;
        strncpy(result.error_message, "Joint limit exceeded", 31);
        
        for (int i = 0; i < NUM_JOINTS; i++) {
            JointLimits limits = RobotConfig::getInstance().getJointLimits(i);
            if (result.joint_angles[i] < limits.min_position ||
                result.joint_angles[i] > limits.max_position) {
                result.joint_limit_warning = true;
                result.limiting_joint = i;
                break;
            }
        }
        return result;
    }
    
    // ========== Set Metrics ==========
    result.feasible = true;
    result.confidence = 0.9f;
    
    // Calculate error
    float verify_pose[6];
    Kinematics::getInstance().forwardKinematics(result.joint_angles, verify_pose);
    float dx = verify_pose[0] - x;
    float dy = verify_pose[1] - y;
    float dz = verify_pose[2] - z;
    result.error_metric = sqrt(dx*dx + dy*dy + dz*dz);
    
    // Singularity check
    result.singularity_metric = getSingularityMetric(result.joint_angles);
    if (result.singularity_metric < 0.15f) {
        result.singularity_warning = true;
    }
    
    // ========== Velocity Suggestions (Servo-Friendly: constant speed) ==========
    // Servos have internal speed control, suggest uniform velocity
    for (int i = 0; i < NUM_JOINTS; i++) {
        result.velocity_suggestions[i] = 60.0f;  // Uniform 60 deg/s
    }
    
    return result;
}

// ============================================================================
// VALIDATION FUNCTIONS
// ============================================================================

bool ServoFriendlyIKSolver::validateJointLimits(const float* angles) {
    if (angles == nullptr) return false;
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        JointLimits limits = RobotConfig::getInstance().getJointLimits(i);
        if (angles[i] < limits.min_position || angles[i] > limits.max_position) {
            return false;
        }
    }
    return true;
}

float ServoFriendlyIKSolver::getSingularityMetric(const float* angles) {
    if (angles == nullptr) return 0.0f;
    
    // Check elbow singularity
    float elbow_margin = fabs(sin(angles[2] * PI / 180.0f));
    
    return elbow_margin;
}

float ServoFriendlyIKSolver::normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}
