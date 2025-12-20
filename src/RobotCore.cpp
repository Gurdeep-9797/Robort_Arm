#include "RobotCore.h"

bool PlaneMath::planeFrom3Points(const float p1[3], const float p2[3], const float p3[3], float origin[3], float normal[3]) {
    float v1[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
    float v2[3] = {p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]};
    
    normal[0] = v1[1]*v2[2] - v1[2]*v2[1];
    normal[1] = v1[2]*v2[0] - v1[0]*v2[2];
    normal[2] = v1[0]*v2[1] - v1[1]*v2[0];
    
    float len = sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]);
    if (len < 0.01f) return false;
    
    normal[0] /= len;
    normal[1] /= len;
    normal[2] /= len;
    
    origin[0] = (p1[0] + p2[0] + p3[0]) / 3.0f;
    origin[1] = (p1[1] + p2[1] + p3[1]) / 3.0f;
    origin[2] = (p1[2] + p2[2] + p3[2]) / 3.0f;
    
    return true;
}

void PlaneMath::projectPointOntoPlane(const float origin[3], const float normal[3], const float point[3], float projected[3]) {
    float d = (point[0] - origin[0]) * normal[0] + (point[1] - origin[1]) * normal[1] + (point[2] - origin[2]) * normal[2];
    projected[0] = point[0] - d * normal[0];
    projected[1] = point[1] - d * normal[1];
    projected[2] = point[2] - d * normal[2];
}

float PlaneMath::distanceToPlane(const float origin[3], const float normal[3], const float point[3]) {
    return (point[0] - origin[0]) * normal[0] + (point[1] - origin[1]) * normal[1] + (point[2] - origin[2]) * normal[2];
}

RobotConfig& RobotConfig::getInstance() {
    static RobotConfig instance;
    return instance;
}

RobotConfig::RobotConfig() : current_mode_(OperatingMode::MODE_1_OPEN_LOOP), plane_enabled_(false) {
    mutex_ = xSemaphoreCreateMutex();
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        limits_[i].min_position = -180.0f;
        limits_[i].max_position = 180.0f;
        limits_[i].max_velocity = 120.0f;
        limits_[i].max_acceleration = 500.0f;
    }
}

bool RobotConfig::begin() {
    prefs_.begin("robot", false);
    if (prefs_.isKey("mode")) {
        loadFromNVS();
    }
    return true;
}

JointLimits RobotConfig::getJointLimits(uint8_t joint) {
    if (joint >= NUM_JOINTS) return limits_[0];
    xSemaphoreTake(mutex_, portMAX_DELAY);
    JointLimits result = limits_[joint];
    xSemaphoreGive(mutex_);
    return result;
}

bool RobotConfig::setJointLimits(uint8_t joint, float min, float max) {
    if (joint >= NUM_JOINTS || min >= max) return false;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    limits_[joint].min_position = min;
    limits_[joint].max_position = max;
    xSemaphoreGive(mutex_);
    return true;
}

bool RobotConfig::setVelocityLimit(uint8_t joint, float max_vel) {
    if (joint >= NUM_JOINTS || max_vel <= 0) return false;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    limits_[joint].max_velocity = max_vel;
    xSemaphoreGive(mutex_);
    return true;
}

OperatingMode RobotConfig::getMode() {
    return current_mode_;
}

bool RobotConfig::setMode(OperatingMode mode) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    current_mode_ = mode;
    xSemaphoreGive(mutex_);
    return true;
}

bool RobotConfig::getActivePlane(Plane& plane) {
    if (!plane_enabled_) return false;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    plane = active_plane_;
    xSemaphoreGive(mutex_);
    return true;
}

bool RobotConfig::definePlaneFrom3Points(const float p1[3], const float p2[3], const float p3[3], const char* name) {
    float origin[3], normal[3];
    if (!PlaneMath::planeFrom3Points(p1, p2, p3, origin, normal)) return false;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    active_plane_.origin = Vector3(origin[0], origin[1], origin[2]);
    active_plane_.normal = Vector3(normal[0], normal[1], normal[2]);
    if (name) strncpy(active_plane_.name, name, 31);
    plane_enabled_ = true;
    xSemaphoreGive(mutex_);
    return true;
}

void RobotConfig::setPlaneEnabled(bool enabled) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    plane_enabled_ = enabled;
    xSemaphoreGive(mutex_);
}

bool RobotConfig::saveToNVS() {
    prefs_.putUChar("mode", (uint8_t)current_mode_);
    prefs_.putBool("plane_en", plane_enabled_);
    for (int i = 0; i < NUM_JOINTS; i++) {
        char key[16];
        sprintf(key, "min_%d", i);
        prefs_.putFloat(key, limits_[i].min_position);
        sprintf(key, "max_%d", i);
        prefs_.putFloat(key, limits_[i].max_position);
        sprintf(key, "vel_%d", i);
        prefs_.putFloat(key, limits_[i].max_velocity);
    }
    return true;
}

bool RobotConfig::loadFromNVS() {
    current_mode_ = (OperatingMode)prefs_.getUChar("mode", 1);
    plane_enabled_ = prefs_.getBool("plane_en", false);
    for (int i = 0; i < NUM_JOINTS; i++) {
        char key[16];
        sprintf(key, "min_%d", i);
        limits_[i].min_position = prefs_.getFloat(key, -180.0f);
        sprintf(key, "max_%d", i);
        limits_[i].max_position = prefs_.getFloat(key, 180.0f);
        sprintf(key, "vel_%d", i);
        limits_[i].max_velocity = prefs_.getFloat(key, 120.0f);
    }
    return true;
}

ServoController& ServoController::getInstance() {
    static ServoController instance;
    return instance;
}

ServoController::ServoController() {
    pins_[0] = 25;
    pins_[1] = 26;
    pins_[2] = 27;
    pins_[3] = 14;
    pins_[4] = 12;
    pins_[5] = 13;
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        enabled_[i] = false;
    }
}

bool ServoController::begin() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        servos_[i].attach(pins_[i]);
        servos_[i].write(90);
        enabled_[i] = true;
    }
    return true;
}

void ServoController::writePosition(uint8_t joint, float angle) {
    if (joint >= NUM_JOINTS || !enabled_[joint]) return;
    
    JointLimits limits = RobotConfig::getInstance().getJointLimits(joint);
    if (angle < limits.min_position) angle = limits.min_position;
    if (angle > limits.max_position) angle = limits.max_position;
    
    float mapped = map(angle * 10, limits.min_position * 10, limits.max_position * 10, 500, 2500);
    servos_[joint].writeMicroseconds((int)mapped);
}

void ServoController::writeAllPositions(const float* angles) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        writePosition(i, angles[i]);
    }
}

void ServoController::disableAll() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        servos_[i].detach();
        enabled_[i] = false;
    }
}

void ServoController::enableAll() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        servos_[i].attach(pins_[i]);
        enabled_[i] = true;
    }
}

EncoderReader& EncoderReader::getInstance() {
    static EncoderReader instance;
    return instance;
}

EncoderReader::EncoderReader() {
    pin_a_[0] = 32; pin_b_[0] = 33;
    pin_a_[1] = 35; pin_b_[1] = 34;
    pin_a_[2] = 36; pin_b_[2] = 39;
    pin_a_[3] = 21; pin_b_[3] = 22;
    pin_a_[4] = 16; pin_b_[4] = 17;
    pin_a_[5] = 18; pin_b_[5] = 19;
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        counts_[i] = 0;
    }
}

bool EncoderReader::begin() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        pinMode(pin_a_[i], INPUT_PULLUP);
        pinMode(pin_b_[i], INPUT_PULLUP);
    }
    
    attachInterrupt(digitalPinToInterrupt(pin_a_[0]), isr0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pin_a_[1]), isr1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pin_a_[2]), isr2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pin_a_[3]), isr3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pin_a_[4]), isr4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pin_a_[5]), isr5, CHANGE);
    
    return true;
}

void IRAM_ATTR EncoderReader::isr0() { getInstance().handleEncoder(0); }
void IRAM_ATTR EncoderReader::isr1() { getInstance().handleEncoder(1); }
void IRAM_ATTR EncoderReader::isr2() { getInstance().handleEncoder(2); }
void IRAM_ATTR EncoderReader::isr3() { getInstance().handleEncoder(3); }
void IRAM_ATTR EncoderReader::isr4() { getInstance().handleEncoder(4); }
void IRAM_ATTR EncoderReader::isr5() { getInstance().handleEncoder(5); }

void IRAM_ATTR EncoderReader::handleEncoder(uint8_t joint) {
    bool a = digitalRead(pin_a_[joint]);
    bool b = digitalRead(pin_b_[joint]);
    
    static bool last_a[NUM_JOINTS] = {false};
    
    if (a && !last_a[joint]) {
        if (b) counts_[joint]++;
        else counts_[joint]--;
    }
    
    last_a[joint] = a;
}

int32_t EncoderReader::getCount(uint8_t joint) {
    if (joint >= NUM_JOINTS) return 0;
    return counts_[joint];
}

float EncoderReader::getPositionDegrees(uint8_t joint) {
    if (joint >= NUM_JOINTS) return 0.0f;
    return (float)counts_[joint] * 360.0f / 4096.0f;
}

void EncoderReader::zero(uint8_t joint) {
    if (joint >= NUM_JOINTS) return;
    counts_[joint] = 0;
}

Kinematics& Kinematics::getInstance() {
    static Kinematics instance;
    return instance;
}

Kinematics::Kinematics() {}

bool Kinematics::begin() {
    return true;
}

bool Kinematics::forwardKinematics(const float* joint_angles, float* cartesian_pose) {
    float rad[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++) {
        rad[i] = joint_angles[i] * PI / 180.0f;
    }
    
    float x = 300.0f * cos(rad[0]) * cos(rad[1]) + 250.0f * cos(rad[0]) * cos(rad[1] + rad[2]);
    float y = 300.0f * sin(rad[0]) * cos(rad[1]) + 250.0f * sin(rad[0]) * cos(rad[1] + rad[2]);
    float z = 150.0f + 300.0f * sin(rad[1]) + 250.0f * sin(rad[1] + rad[2]);
    
    cartesian_pose[0] = x;
    cartesian_pose[1] = y;
    cartesian_pose[2] = z;
    cartesian_pose[3] = 0.0f;
    cartesian_pose[4] = 0.0f;
    cartesian_pose[5] = rad[0] * 180.0f / PI;
    
    return true;
}

bool Kinematics::inverseKinematics(const float* cartesian_pose, const float* seed_angles, float* result_angles) {
    float x = cartesian_pose[0];
    float y = cartesian_pose[1];
    float z = cartesian_pose[2];
    
    result_angles[0] = atan2(y, x) * 180.0f / PI;
    
    float r = sqrt(x*x + y*y);
    float s = z - 150.0f;
    float D = sqrt(r*r + s*s);
    
    float L1 = 300.0f;
    float L2 = 250.0f;
    
    float cos_theta2 = (D*D - L1*L1 - L2*L2) / (2.0f * L1 * L2);
    if (cos_theta2 < -1.0f || cos_theta2 > 1.0f) return false;
    
    result_angles[2] = atan2(sqrt(1.0f - cos_theta2*cos_theta2), cos_theta2) * 180.0f / PI;
    
    float alpha = atan2(s, r);
    float beta = atan2(L2 * sin(result_angles[2] * PI / 180.0f), L1 + L2 * cos(result_angles[2] * PI / 180.0f));
    result_angles[1] = (alpha - beta) * 180.0f / PI;
    
    result_angles[3] = 0.0f;
    result_angles[4] = 0.0f;
    result_angles[5] = 0.0f;
    
    return true;
}

SafetyManager& SafetyManager::getInstance() {
    static SafetyManager instance;
    return instance;
}

SafetyManager::SafetyManager() : is_homed_(false), fault_code_(0), e_stop_active_(false) {
    mutex_ = xSemaphoreCreateMutex();
}

bool SafetyManager::begin() {
    pinMode(4, INPUT_PULLUP);
    xTaskCreatePinnedToCore(safetyTask, "Safety", 4096, NULL, 4, NULL, 1);
    return true;
}

void SafetyManager::safetyTask(void* param) {
    while (true) {
        SafetyManager& sm = getInstance();
        
        if (digitalRead(4) == LOW) {
            sm.e_stop_active_ = true;
            sm.fault_code_ |= (1 << 5);
            ServoController::getInstance().disableAll();
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

bool SafetyManager::isHomed() {
    return is_homed_;
}

void SafetyManager::setHomed(bool homed) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    is_homed_ = homed;
    xSemaphoreGive(mutex_);
}

bool SafetyManager::canMove() {
    return !e_stop_active_ && fault_code_ == 0;
}

uint32_t SafetyManager::getFaults() {
    return fault_code_;
}

void SafetyManager::clearFaults() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    fault_code_ = 0;
    e_stop_active_ = false;
    xSemaphoreGive(mutex_);
    ServoController::getInstance().enableAll();
}

void SafetyManager::emergencyStop() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    e_stop_active_ = true;
    fault_code_ |= (1 << 5);
    xSemaphoreGive(mutex_);
    ServoController::getInstance().disableAll();
}

MotionPlanner& MotionPlanner::getInstance() {
    static MotionPlanner instance;
    return instance;
}

MotionPlanner::MotionPlanner() : is_moving_(false), progress_(0.0f) {
    command_queue_ = xQueueCreate(10, sizeof(uint8_t) * 100);
}

bool MotionPlanner::begin() {
    xTaskCreatePinnedToCore(plannerTask, "Planner", 8192, NULL, 2, NULL, 1);
    return true;
}

void MotionPlanner::plannerTask(void* param) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

bool MotionPlanner::moveJoint(uint8_t joint, float position, float velocity) {
    if (joint >= NUM_JOINTS || !SafetyManager::getInstance().canMove()) return false;
    
    is_moving_ = true;
    
    float current_pos;
    ControlLoop::getInstance().getCurrentPositions(&current_pos);
    
    float distance = abs(position - current_pos);
    float step = (position > current_pos) ? 1.0f : -1.0f;
    
    for (float pos = current_pos; abs(pos - position) > 0.5f; pos += step) {
        ServoController::getInstance().writePosition(joint, pos);
        progress_ = abs(pos - current_pos) / distance;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    ServoController::getInstance().writePosition(joint, position);
    is_moving_ = false;
    progress_ = 1.0f;
    
    return true;
}

bool MotionPlanner::moveAllJoints(const float* positions, float velocity) {
    if (!SafetyManager::getInstance().canMove()) return false;
    
    is_moving_ = true;
    
    float current[NUM_JOINTS];
    ControlLoop::getInstance().getCurrentPositions(current);
    
    float max_distance = 0;
    for (int i = 0; i < NUM_JOINTS; i++) {
        float dist = abs(positions[i] - current[i]);
        if (dist > max_distance) max_distance = dist;
    }
    
    int steps = (int)(max_distance / 1.0f);
    if (steps < 1) steps = 1;
    
    for (int step = 0; step <= steps; step++) {
        float t = (float)step / (float)steps;
        float interpolated[NUM_JOINTS];
        
        for (int i = 0; i < NUM_JOINTS; i++) {
            interpolated[i] = current[i] + t * (positions[i] - current[i]);
        }
        
        ServoController::getInstance().writeAllPositions(interpolated);
        progress_ = t;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    is_moving_ = false;
    progress_ = 1.0f;
    
    return true;
}

bool MotionPlanner::moveCartesian(const float* target, float velocity, bool respect_plane) {
    if (!SafetyManager::getInstance().canMove()) return false;
    
    float current_joints[NUM_JOINTS];
    ControlLoop::getInstance().getCurrentPositions(current_joints);
    
    float target_joints[NUM_JOINTS];
    if (!Kinematics::getInstance().inverseKinematics(target, current_joints, target_joints)) {
        return false;
    }
    
    if (respect_plane) {
        Plane plane;
        if (RobotConfig::getInstance().getActivePlane(plane)) {
            float origin[3] = {plane.origin.x, plane.origin.y, plane.origin.z};
            float normal[3] = {plane.normal.x, plane.normal.y, plane.normal.z};
            float projected[3];
            PlaneMath::projectPointOntoPlane(origin, normal, target, projected);
            
            float corrected_target[6];
            corrected_target[0] = projected[0];
            corrected_target[1] = projected[1];
            corrected_target[2] = projected[2];
            corrected_target[3] = target[3];
            corrected_target[4] = target[4];
            corrected_target[5] = target[5];
            
            if (!Kinematics::getInstance().inverseKinematics(corrected_target, current_joints, target_joints)) {
                return false;
            }
        }
    }
    
    return moveAllJoints(target_joints, velocity);
}

bool MotionPlanner::stopMotion() {
    is_moving_ = false;
    return true;
}

bool MotionPlanner::isMoving() {
    return is_moving_;
}

float MotionPlanner::getProgress() {
    return progress_;
}

ControlLoop& ControlLoop::getInstance() {
    static ControlLoop instance;
    return instance;
}

ControlLoop::ControlLoop() {
    mutex_ = xSemaphoreCreateMutex();
    for (int i = 0; i < NUM_JOINTS; i++) {
        current_positions_[i] = 0.0f;
        current_velocities_[i] = 0.0f;
    }
}

bool ControlLoop::begin() {
    xTaskCreatePinnedToCore(controlTask, "Control", 8192, NULL, 3, NULL, 1);
    return true;
}

void ControlLoop::controlTask(void* param) {
    ControlLoop& cl = getInstance();
    
    while (true) {
        if (RobotConfig::getInstance().getMode() == OperatingMode::MODE_2_CLOSED_LOOP) {
            xSemaphoreTake(cl.mutex_, portMAX_DELAY);
            for (int i = 0; i < NUM_JOINTS; i++) {
                cl.current_positions_[i] = EncoderReader::getInstance().getPositionDegrees(i);
            }
            xSemaphoreGive(cl.mutex_);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ControlLoop::getCurrentPositions(float* positions) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    memcpy(positions, current_positions_, sizeof(float) * NUM_JOINTS);
    xSemaphoreGive(mutex_);
}

void ControlLoop::getCurrentVelocities(float* velocities) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    memcpy(velocities, current_velocities_, sizeof(float) * NUM_JOINTS);
    xSemaphoreGive(mutex_);
}

RobotStateManager& RobotStateManager::getInstance() {
    static RobotStateManager instance;
    return instance;
}

RobotStateManager::RobotStateManager() {
    mutex_ = xSemaphoreCreateMutex();
}

bool RobotStateManager::begin() {
    xTaskCreatePinnedToCore(updateTask, "StateUpdate", 4096, NULL, 1, NULL, 0);
    return true;
}

void RobotStateManager::updateTask(void* param) {
    while (true) {
        getInstance().updateState();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void RobotStateManager::updateState() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    
    cached_state_.timestamp_ms = millis();
    cached_state_.mode = RobotConfig::getInstance().getMode();
    
    ControlLoop::getInstance().getCurrentPositions(cached_state_.joint_positions);
    ControlLoop::getInstance().getCurrentVelocities(cached_state_.joint_velocities);
    
    cached_state_.is_moving = MotionPlanner::getInstance().isMoving();
    cached_state_.is_homed = SafetyManager::getInstance().isHomed();
    cached_state_.motion_allowed = SafetyManager::getInstance().canMove();
    cached_state_.fault_code = SafetyManager::getInstance().getFaults();
    
    Kinematics::getInstance().forwardKinematics(cached_state_.joint_positions, (float*)&cached_state_.cartesian_pos);
    
    RobotConfig::getInstance().getActivePlane(cached_state_.active_plane);
    cached_state_.plane_active = false;
    
    xSemaphoreGive(mutex_);
}

RobotState RobotStateManager::getState() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    RobotState state = cached_state_;
    xSemaphoreGive(mutex_);
    return state;
}