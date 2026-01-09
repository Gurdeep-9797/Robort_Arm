/**
 * @file DCFriendlyIK.cpp
 * @brief Velocity/torque aware IK solver for DC motors
 * 
 * Considers velocity and acceleration constraints when computing IK.
 * Optimal for DC motors with encoder feedback where dynamics matter.
 */

#include "../include/ik/IKSolver.h"
#include "../include/RobotCore.h"
#include "../include/motors/MotorConfig.h"
#include <math.h>

// ============================================================================
// DC-FRIENDLY IK SOLVER IMPLEMENTATION
// ============================================================================

class DCFriendlyIKSolver : public IIKSolver {
public:
    static DCFriendlyIKSolver& getInstance() {
        static DCFriendlyIKSolver instance;
        return instance;
    }
    
    IKResult solve(const float* target_pose, const float* seed_angles) override;
    bool validateJointLimits(const float* angles) override;
    float getSingularityMetric(const float* angles) override;
    IKSolverType getType() override { return IKSolverType::DC_FRIENDLY; }
    const char* getName() override { return "DC-Friendly"; }

private:
    DCFriendlyIKSolver() {}
    
    float normalizeAngle(float angle);
    void computeVelocityProfile(const float* current, const float* target, 
                                float* velocities, float max_time);
};

// ============================================================================
// SOLVE IMPLEMENTATION
// ============================================================================

IKResult DCFriendlyIKSolver::solve(const float* target_pose, const float* seed_angles) {
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
    
    // ========== Geometric Solution (same as other solvers) ==========
    float theta0 = atan2(y, x) * 180.0f / PI;
    
    float r = sqrt(x*x + y*y);
    float s = z - 150.0f;
    float D = sqrt(r*r + s*s);
    
    float L1 = 300.0f;
    float L2 = 250.0f;
    
    if (D > L1 + L2 * 0.99f || D < fabs(L1 - L2) * 1.01f) {
        result.feasible = false;
        result.error_code = IKError::OUT_OF_REACH;
        strncpy(result.error_message, "Target unreachable", 31);
        return result;
    }
    
    float cos_theta2 = (D*D - L1*L1 - L2*L2) / (2.0f * L1 * L2);
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;
    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    
    float theta2 = atan2(sqrt(1.0f - cos_theta2*cos_theta2), cos_theta2) * 180.0f / PI;
    
    float alpha = atan2(s, r);
    float beta = atan2(L2 * sin(theta2 * PI / 180.0f), 
                       L1 + L2 * cos(theta2 * PI / 180.0f));
    float theta1 = (alpha - beta) * 180.0f / PI;
    
    float theta3 = target_pose[3];
    float theta4 = target_pose[4];
    float theta5 = target_pose[5];
    
    // Store angles
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
    
    // ========== DC-Specific: Velocity Profile Computation ==========
    // Compute optimal velocity profile considering motor dynamics
    
    float current_angles[NUM_JOINTS];
    if (seed_angles != nullptr) {
        memcpy(current_angles, seed_angles, sizeof(float) * NUM_JOINTS);
    } else {
        // Get current positions from control loop
        ControlLoop::getInstance().getCurrentPositions(current_angles);
    }
    
    // Calculate movement for each joint
    float max_movement = 0.0f;
    for (int i = 0; i < NUM_JOINTS; i++) {
        float movement = fabs(result.joint_angles[i] - current_angles[i]);
        if (movement > max_movement) {
            max_movement = movement;
        }
    }
    
    // Compute synchronized velocity profile
    // All joints finish at the same time for smooth motion
    if (max_movement > 0.1f) {
        // Get max velocity from config
        float global_max_vel = 120.0f;  // Default max
        
        for (int i = 0; i < NUM_JOINTS; i++) {
            AxisMotorConfig& cfg = MotorConfigManager::getInstance().getAxisConfig(i);
            if (cfg.max_velocity < global_max_vel) {
                global_max_vel = cfg.max_velocity;
            }
        }
        
        // Time for slowest joint at max speed
        float motion_time = max_movement / global_max_vel;
        
        // Compute velocity for each joint to synchronize
        for (int i = 0; i < NUM_JOINTS; i++) {
            float movement = fabs(result.joint_angles[i] - current_angles[i]);
            
            if (movement > 0.1f) {
                // Velocity to complete in motion_time
                float required_vel = movement / motion_time;
                
                // Clamp to joint limit
                JointLimits limits = RobotConfig::getInstance().getJointLimits(i);
                if (required_vel > limits.max_velocity) {
                    required_vel = limits.max_velocity;
                }
                
                result.velocity_suggestions[i] = required_vel;
            } else {
                result.velocity_suggestions[i] = 30.0f;  // Minimum velocity
            }
        }
    } else {
        // Very small movement - use minimum velocity
        for (int i = 0; i < NUM_JOINTS; i++) {
            result.velocity_suggestions[i] = 30.0f;
        }
    }
    
    // ========== Set Metrics ==========
    result.feasible = true;
    result.confidence = 0.85f;
    
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
    
    return result;
}

// ============================================================================
// VALIDATION FUNCTIONS
// ============================================================================

bool DCFriendlyIKSolver::validateJointLimits(const float* angles) {
    if (angles == nullptr) return false;
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        JointLimits limits = RobotConfig::getInstance().getJointLimits(i);
        if (angles[i] < limits.min_position || angles[i] > limits.max_position) {
            return false;
        }
    }
    return true;
}

float DCFriendlyIKSolver::getSingularityMetric(const float* angles) {
    if (angles == nullptr) return 0.0f;
    
    float elbow_margin = fabs(sin(angles[2] * PI / 180.0f));
    return elbow_margin;
}

float DCFriendlyIKSolver::normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void DCFriendlyIKSolver::computeVelocityProfile(const float* current, const float* target,
                                                float* velocities, float max_time) {
    // Compute trapezoidal velocity profile for each joint
    for (int i = 0; i < NUM_JOINTS; i++) {
        float distance = fabs(target[i] - current[i]);
        
        if (distance < 0.1f || max_time < 0.01f) {
            velocities[i] = 30.0f;
            continue;
        }
        
        // Calculate required average velocity
        float avg_velocity = distance / max_time;
        
        // Peak velocity for trapezoidal profile (assume 20% accel/decel time)
        float peak_velocity = avg_velocity / 0.6f;  
        
        // Clamp to limits
        JointLimits limits = RobotConfig::getInstance().getJointLimits(i);
        if (peak_velocity > limits.max_velocity) {
            peak_velocity = limits.max_velocity;
        }
        
        velocities[i] = peak_velocity;
    }
}
