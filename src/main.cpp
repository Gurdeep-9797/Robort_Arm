#include <Arduino.h>
#include "RobotCore.h"
#include "RobotUI.h"

const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n========================================");
    Serial.println("ESP32 Robotic Arm Firmware v1.0");
    Serial.println("========================================\n");
    
    Serial.println("[1/9] Initializing Configuration...");
    if (!RobotConfig::getInstance().begin()) {
        Serial.println("ERROR: Config initialization failed!");
        while(1) delay(1000);
    }
    Serial.println("✓ Config OK");
    
    Serial.println("[2/9] Initializing Servo Controller...");
    if (!ServoController::getInstance().begin()) {
        Serial.println("ERROR: Servo initialization failed!");
        while(1) delay(1000);
    }
    Serial.println("✓ Servos OK");
    
    Serial.println("[3/9] Initializing Encoder Reader...");
    if (!EncoderReader::getInstance().begin()) {
        Serial.println("WARNING: Encoders may not be connected");
    }
    Serial.println("✓ Encoders OK");
    
    Serial.println("[4/9] Initializing Kinematics...");
    if (!Kinematics::getInstance().begin()) {
        Serial.println("ERROR: Kinematics initialization failed!");
        while(1) delay(1000);
    }
    Serial.println("✓ Kinematics OK");
    
    Serial.println("[5/9] Initializing Safety Manager...");
    if (!SafetyManager::getInstance().begin()) {
        Serial.println("ERROR: Safety manager initialization failed!");
        while(1) delay(1000);
    }
    Serial.println("✓ Safety Manager OK");
    
    Serial.println("[6/9] Initializing Control Loop...");
    if (!ControlLoop::getInstance().begin()) {
        Serial.println("ERROR: Control loop initialization failed!");
        while(1) delay(1000);
    }
    Serial.println("✓ Control Loop OK");
    
    Serial.println("[7/9] Initializing Motion Planner...");
    if (!MotionPlanner::getInstance().begin()) {
        Serial.println("ERROR: Motion planner initialization failed!");
        while(1) delay(1000);
    }
    Serial.println("✓ Motion Planner OK");
    
    Serial.println("[8/9] Initializing State Manager...");
    if (!RobotStateManager::getInstance().begin()) {
        Serial.println("ERROR: State manager initialization failed!");
        while(1) delay(1000);
    }
    Serial.println("✓ State Manager OK");
    
    Serial.println("[9/9] Starting Web Server...");
    if (!WebServer::getInstance().begin(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("WARNING: Web server not available");
    } else {
        Serial.println("✓ Web Server OK");
        Serial.print("\n>>> Open browser to: http://");
        Serial.println(WiFi.localIP());
    }
    
    Serial.println("\n========================================");
    Serial.println("System Ready!");
    Serial.println("========================================\n");
    
    Serial.println("Current Status:");
    Serial.printf("- Mode: %s\n", 
        (RobotConfig::getInstance().getMode() == OperatingMode::MODE_1_OPEN_LOOP) ? 
        "MODE 1 (Open Loop)" : "MODE 2 (Closed Loop)");
    Serial.printf("- Homed: %s\n", SafetyManager::getInstance().isHomed() ? "Yes" : "No");
    Serial.printf("- Motion Allowed: %s\n", SafetyManager::getInstance().canMove() ? "Yes" : "No");
    Serial.println("\nNOTE: Robot must be homed before MODE 2 operation");
    Serial.println("TIP: Use web interface to control the robot\n");
}

void loop() {
    static uint32_t last_status = 0;
    static uint32_t last_broadcast = 0;
    
    uint32_t now = millis();
    
    if (now - last_broadcast > 100) {
        last_broadcast = now;
        WebServer::getInstance().broadcastState();
    }
    
    if (now - last_status > 5000) {
        last_status = now;
        
        RobotState state = RobotStateManager::getInstance().getState();
        
        Serial.println("\n=== System Status ===");
        Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
        Serial.printf("Mode: %s\n", 
            (state.mode == OperatingMode::MODE_1_OPEN_LOOP) ? 
            "MODE 1" : "MODE 2");
        Serial.printf("Homed: %s\n", state.is_homed ? "Yes" : "No");
        Serial.printf("Moving: %s\n", state.is_moving ? "Yes" : "No");
        Serial.printf("Faults: 0x%08X\n", state.fault_code);
        
        Serial.print("Joint Positions: ");
        for (int i = 0; i < NUM_JOINTS; i++) {
            Serial.printf("%.1f° ", state.joint_positions[i]);
        }
        Serial.println();
        
        Serial.printf("Cartesian: X=%.1f Y=%.1f Z=%.1f\n", 
            state.cartesian_pos.x, state.cartesian_pos.y, state.cartesian_pos.z);
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("WiFi: Connected (%s)\n", WiFi.localIP().toString().c_str());
        } else {
            Serial.println("WiFi: Disconnected");
        }
        
        Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
        Serial.println("====================\n");
    }
    
    delay(10);
}