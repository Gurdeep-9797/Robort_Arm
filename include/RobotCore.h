#ifndef ROBOT_CORE_H
#define ROBOT_CORE_H
#include <Adafruit_PWMServoDriver.h> // Add this include
#include <Wire.h>
#include <Arduino.h>

#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#define NUM_JOINTS 6
#define MAX_TAUGHT_POINTS 50

enum class OperatingMode : uint8_t {
    MODE_1_OPEN_LOOP = 1,
    MODE_2_CLOSED_LOOP = 2
};

struct Vector3 {
    float x, y, z;
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct Plane {
    Vector3 origin;
    Vector3 normal;
    char name[32];
};

struct JointLimits {
    float min_position;
    float max_position;
    float max_velocity;
    float max_acceleration;
};

struct RobotState {
    uint32_t timestamp_ms;
    OperatingMode mode;
    float joint_positions[NUM_JOINTS];
    float joint_velocities[NUM_JOINTS];
    bool is_moving;
    bool is_homed;
    bool motion_allowed;
    uint32_t fault_code;
    Vector3 cartesian_pos;
    bool ik_valid;
    Plane active_plane;
    bool plane_active;
};

class PlaneMath {
public:
    static bool planeFrom3Points(const float p1[3], const float p2[3], const float p3[3], float origin[3], float normal[3]);
    static void projectPointOntoPlane(const float origin[3], const float normal[3], const float point[3], float projected[3]);
    static float distanceToPlane(const float origin[3], const float normal[3], const float point[3]);
};

class RobotConfig {
public:
    static RobotConfig& getInstance();
    bool begin();
    
    JointLimits getJointLimits(uint8_t joint);
    bool setJointLimits(uint8_t joint, float min, float max);
    bool setVelocityLimit(uint8_t joint, float max_vel);
    
    OperatingMode getMode();
    bool setMode(OperatingMode mode);
    
    bool getActivePlane(Plane& plane);
    bool definePlaneFrom3Points(const float p1[3], const float p2[3], const float p3[3], const char* name);
    void setPlaneEnabled(bool enabled);
    
    bool saveToNVS();
    bool loadFromNVS();

private:
    RobotConfig();
    JointLimits limits_[NUM_JOINTS];
    OperatingMode current_mode_;
    Plane active_plane_;
    bool plane_enabled_;
    SemaphoreHandle_t mutex_;
    Preferences prefs_;
};


class EncoderReader {
public:
    static EncoderReader& getInstance();
    bool begin();
    
    int32_t getCount(uint8_t joint);
    float getPositionDegrees(uint8_t joint);
    void zero(uint8_t joint);

private:
    EncoderReader();
    volatile int32_t counts_[NUM_JOINTS];
    uint8_t pin_a_[NUM_JOINTS];
    uint8_t pin_b_[NUM_JOINTS];
    
    static void IRAM_ATTR isr0();
    static void IRAM_ATTR isr1();
    static void IRAM_ATTR isr2();
    static void IRAM_ATTR isr3();
    static void IRAM_ATTR isr4();
    static void IRAM_ATTR isr5();
    
    void handleEncoder(uint8_t joint);
};

class Kinematics {
public:
    static Kinematics& getInstance();
    bool begin();
    
    bool forwardKinematics(const float* joint_angles, float* cartesian_pose);
    bool inverseKinematics(const float* cartesian_pose, const float* seed_angles, float* result_angles);

private:
    Kinematics();
};

class SafetyManager {
public:
    static SafetyManager& getInstance();
    bool begin();
    
    bool isHomed();
    void setHomed(bool homed);
    bool canMove();
    uint32_t getFaults();
    void clearFaults();
    void emergencyStop();

private:
    SafetyManager();
    bool is_homed_;
    uint32_t fault_code_;
    bool e_stop_active_;
    SemaphoreHandle_t mutex_;
    static void safetyTask(void* param);
};

class MotionPlanner {
public:
    static MotionPlanner& getInstance();
    bool begin();
    
    bool moveJoint(uint8_t joint, float position, float velocity);
    bool moveAllJoints(const float* positions, float velocity);
    bool moveCartesian(const float* target, float velocity, bool respect_plane);
    bool stopMotion();
    
    bool isMoving();
    float getProgress();

private:
    MotionPlanner();
    QueueHandle_t command_queue_;
    volatile bool is_moving_;
    volatile float progress_;
    static void plannerTask(void* param);
};

class ControlLoop {
public:
    static ControlLoop& getInstance();
    bool begin();
    
    void getCurrentPositions(float* positions);
    void getCurrentVelocities(float* velocities);

private:
    ControlLoop();
    float current_positions_[NUM_JOINTS];
    float current_velocities_[NUM_JOINTS];
    SemaphoreHandle_t mutex_;
    static void controlTask(void* param);
};

class RobotStateManager {
public:
    static RobotStateManager& getInstance();
    bool begin();
    
    RobotState getState();
    void updateState();

private:
    RobotStateManager();
    RobotState cached_state_;
    SemaphoreHandle_t mutex_;
    static void updateTask(void* param);
};
// ... existing code ...

class ServoController {
public:
    static ServoController& getInstance();
    bool begin();
    
    void writePosition(uint8_t joint, float angle);
    void writeAllPositions(const float* angles);
    void disableAll();
    void enableAll();

private:
    ServoController();
    Adafruit_PWMServoDriver pwm_; // Changed from Servo array to PWM driver
    bool enabled_[NUM_JOINTS];
};

#endif