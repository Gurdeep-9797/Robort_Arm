/**
 * @file AnalyticalIK.cpp
 * @brief Analytical (closed-form) IK solver for 6-axis robot
 * 
 * Uses geometric approach for first 3 joints (position)
 * and analytical solution for wrist joints (orientation).
 * 
 * Fastest solver but may have singular configurations.
 */

#include "../include/ik/IKSolver.h"
#include "../include/RobotCore.h"
#include <math.h>

// ============================================================================
// DH PARAMETERS (must match robot configuration)
// ============================================================================

namespace DHParams {
    // Link lengths in mm (from existing kinematics)
    constexpr float L1 = 150.0f;    // Base height
    constexpr float L2 = 300.0f;    // Shoulder to elbow
    constexpr float L3 = 250.0f;    // Elbow to wrist
    constexpr float L4 = 150.0f;    // Wrist center offset
    constexpr float L5 = 100.0f;    // Tool length
}

// ============================================================================
// ANALYTICAL IK SOLVER IMPLEMENTATION
// ============================================================================

class AnalyticalIKSolver : public IIKSolver {
public:
    static AnalyticalIKSolver& getInstance() {
        static AnalyticalIKSolver instance;
        return instance;
    }
    
    IKResult solve(const float* target_pose, const float* seed_angles) override;
    bool validateJointLimits(const float* angles) override;
    float getSingularityMetric(const float* angles) override;
    IKSolverType getType() override { return IKSolverType::ANALYTICAL; }
    const char* getName() override { return "Analytical"; }

private:
    AnalyticalIKSolver() {}
    
    // Helper functions
    float normalizeAngle(float angle);
    bool checkReachability(float x, float y, float z);
};

// ============================================================================
// SOLVE IMPLEMENTATION
// ============================================================================

IKResult AnalyticalIKSolver::solve(const float* target_pose, const float* seed_angles) {
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
    // Orientation: rx, ry, rz (target_pose[3-5]) - used for wrist
    
    // ========== Step 1: Check Reachability ==========
    if (!checkReachability(x, y, z)) {
        result.feasible = false;
        result.error_code = IKError::OUT_OF_REACH;
        strncpy(result.error_message, "Target out of reach", 31);
        return result;
    }
    
    // ========== Step 2: Solve Joint 0 (Base rotation) ==========
    // theta0 = atan2(y, x)
    float theta0 = atan2(y, x) * 180.0f / PI;
    
    // Check for singularity at x=0, y=0
    if (fabs(x) < 0.01f && fabs(y) < 0.01f) {
        // Overhead singularity - use seed angle
        if (seed_angles != nullptr) {
            theta0 = seed_angles[0];
        }
        result.singularity_warning = true;
        result.singularity_metric = 0.1f;
    }
    
    // ========== Step 3: Solve Joints 1 and 2 (Shoulder and Elbow) ==========
    // Use geometric approach in the plane of the arm
    
    float r = sqrt(x*x + y*y);  // Horizontal distance
    float s = z - DHParams::L1;  // Vertical distance from shoulder
    
    float D = sqrt(r*r + s*s);  // Direct distance to wrist
    
    float L1 = DHParams::L2;  // Shoulder to elbow
    float L2 = DHParams::L3;  // Elbow to wrist
    
    // Check triangle inequality (reachability)
    if (D > L1 + L2 || D < fabs(L1 - L2)) {
        result.feasible = false;
        result.error_code = IKError::OUT_OF_REACH;
        strncpy(result.error_message, "Arm cannot reach", 31);
        return result;
    }
    
    // Cosine law for elbow angle
    float cos_theta2 = (D*D - L1*L1 - L2*L2) / (2.0f * L1 * L2);
    
    // Clamp to valid range
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;
    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    
    // Elbow angle (elbow-up solution by default)
    float theta2 = atan2(sqrt(1.0f - cos_theta2*cos_theta2), cos_theta2) * 180.0f / PI;
    
    // Shoulder angle
    float alpha = atan2(s, r);
    float beta = atan2(L2 * sin(theta2 * PI / 180.0f), 
                       L1 + L2 * cos(theta2 * PI / 180.0f));
    float theta1 = (alpha - beta) * 180.0f / PI;
    
    // ========== Step 4: Solve Wrist Joints (3, 4, 5) ==========
    // For now, set wrist angles to achieve desired orientation
    // Simplified: wrist angles default to 0 or track target orientation
    
    float theta3 = 0.0f;
    float theta4 = 0.0f;
    float theta5 = 0.0f;
    
    // If orientation is specified, compute wrist angles
    if (target_pose[3] != 0.0f || target_pose[4] != 0.0f || target_pose[5] != 0.0f) {
        // Simplified wrist solution
        theta3 = target_pose[3];    // Roll
        theta4 = target_pose[4];    // Pitch
        theta5 = target_pose[5];    // Yaw
    }
    
    // ========== Step 5: Normalize and Store Results ==========
    result.joint_angles[0] = normalizeAngle(theta0);
    result.joint_angles[1] = normalizeAngle(theta1);
    result.joint_angles[2] = normalizeAngle(theta2);
    result.joint_angles[3] = normalizeAngle(theta3);
    result.joint_angles[4] = normalizeAngle(theta4);
    result.joint_angles[5] = normalizeAngle(theta5);
    
    // ========== Step 6: Validate Joint Limits ==========
    if (!validateJointLimits(result.joint_angles)) {
        result.feasible = false;
        result.error_code = IKError::JOINT_LIMIT;
        strncpy(result.error_message, "Joint limit exceeded", 31);
        
        // Find which joint caused the limit
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
    
    // ========== Step 7: Calculate Metrics ==========
    result.feasible = true;
    result.confidence = 0.95f;  // High confidence for analytical solution
    
    // Calculate error by FK verification
    float verify_pose[6];
    Kinematics::getInstance().forwardKinematics(result.joint_angles, verify_pose);
    float dx = verify_pose[0] - x;
    float dy = verify_pose[1] - y;
    float dz = verify_pose[2] - z;
    result.error_metric = sqrt(dx*dx + dy*dy + dz*dz);
    
    // Singularity check
    result.singularity_metric = getSingularityMetric(result.joint_angles);
    if (result.singularity_metric < 0.1f) {
        result.singularity_warning = true;
    }
    
    // Default velocity suggestions
    for (int i = 0; i < NUM_JOINTS; i++) {
        result.velocity_suggestions[i] = 60.0f;  // 60 deg/s default
    }
    
    return result;
}

// ============================================================================
// VALIDATION FUNCTIONS
// ============================================================================

bool AnalyticalIKSolver::validateJointLimits(const float* angles) {
    if (angles == nullptr) return false;
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        JointLimits limits = RobotConfig::getInstance().getJointLimits(i);
        if (angles[i] < limits.min_position || angles[i] > limits.max_position) {
            return false;
        }
    }
    return true;
}

float AnalyticalIKSolver::getSingularityMetric(const float* angles) {
    if (angles == nullptr) return 0.0f;
    
    // Check for common singularities:
    // 1. Overhead: arm stretched straight up
    // 2. Outstretched: arm fully extended
    // 3. Wrist flip: wrist joints aligned
    
    float metric = 1.0f;
    
    // Check elbow angle (singularity when fully extended or folded)
    float elbow_margin = fabs(sin(angles[2] * PI / 180.0f));
    if (elbow_margin < 0.1f) {
        metric = min(metric, elbow_margin);
    }
    
    // Check shoulder angle (singularity at ±90°)
    float shoulder_margin = fabs(cos(angles[1] * PI / 180.0f));
    if (shoulder_margin < 0.1f) {
        metric = min(metric, shoulder_margin);
    }
    
    return metric;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

float AnalyticalIKSolver::normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

bool AnalyticalIKSolver::checkReachability(float x, float y, float z) {
    float distance = sqrt(x*x + y*y + (z - DHParams::L1)*(z - DHParams::L1));
    float max_reach = DHParams::L2 + DHParams::L3;
    float min_reach = fabs(DHParams::L2 - DHParams::L3);
    
    return (distance <= max_reach * 0.95f && distance >= min_reach * 1.05f);
}
