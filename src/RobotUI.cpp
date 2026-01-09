#include "RobotUI.h"

WebServer& WebServer::getInstance() {
    static WebServer instance;
    return instance;
}

WebServer::WebServer() : server_(nullptr), ws_(nullptr), last_broadcast_(0) {}

bool WebServer::begin(const char* ssid, const char* password) {
    WiFi.begin(ssid, password);
    
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi connection failed!");
        return false;
    }
    
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
        return false;
    }
    
    server_ = new AsyncWebServer(80);
    ws_ = new AsyncWebSocket("/ws");
    
    setupWebSocket();
    setupRoutes();
    
    server_->begin();
    Serial.println("Web server started");
    
    return true;
}

void WebServer::setupWebSocket() {
    ws_->onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
        this->onWebSocketEvent(server, client, type, arg, data, len);
    });
    server_->addHandler(ws_);
}

void WebServer::onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WebSocket client #%u connected\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
    }
}

void WebServer::setupRoutes() {
    server_->on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
    
    server_->on("/app.js", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(SPIFFS, "/app.js", "application/javascript");
    });
    
    server_->on("/style.css", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(SPIFFS, "/style.css", "text/css");
    });
    
    server_->on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handleGetStatus(request);
    });
    
    server_->on("/api/motion/joint", HTTP_POST, 
        [](AsyncWebServerRequest* request) {}, 
        NULL,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            this->handleMoveJoint(request, data, len, index, total);
        }
    );
    
    server_->on("/api/motion/cartesian", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            this->handleMoveCartesian(request, data, len, index, total);
        }
    );
    
    server_->on("/api/motion/stop", HTTP_POST, [this](AsyncWebServerRequest* request) {
        this->handleStopMotion(request);
    });
    
    server_->on("/api/safety/estop", HTTP_POST, [this](AsyncWebServerRequest* request) {
        this->handleEmergencyStop(request);
    });
    
    server_->on("/api/safety/clear", HTTP_POST, [this](AsyncWebServerRequest* request) {
        this->handleClearFault(request);
    });
    
    server_->on("/api/mode", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            this->handleSetMode(request, data, len, index, total);
        }
    );
    
    server_->on("/api/plane/define", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            this->handleDefinePlane(request, data, len, index, total);
        }
    );
    
    // ========== IK API Routes ==========
    
    server_->on("/api/ik/config", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handleGetIKConfig(request);
    });
    
    server_->on("/api/ik/configure", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            this->handleSetIKConfig(request, data, len, index, total);
        }
    );
    
    server_->on("/api/ik/preview", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            this->handleIKPreview(request, data, len, index, total);
        }
    );
    
    // ========== Motor API Routes ==========
    
    server_->on("/api/motors/config", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handleGetMotorConfig(request);
    });
    
    server_->on("/api/motors/configure", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            this->handleSetMotorConfig(request, data, len, index, total);
        }
    );
    
    // ========== Alignment & Diagnostics Routes ==========
    
    server_->on("/api/alignment/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handleGetAlignmentStatus(request);
    });
    
    server_->on("/api/diagnostics/history", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handleGetDiagnosticsHistory(request);
    });
    
    server_->on("/api/diagnostics/clear", HTTP_POST, [this](AsyncWebServerRequest* request) {
        this->handleClearDiagnostics(request);
    });
    
    // ========== Static files for new JS modules ==========
    
    server_->on("/ik_config.js", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(SPIFFS, "/ik_config.js", "application/javascript");
    });
    
    server_->on("/motor_config.js", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(SPIFFS, "/motor_config.js", "application/javascript");
    });
    
    server_->on("/diagnostics.js", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(SPIFFS, "/diagnostics.js", "application/javascript");
    });
    
    server_->on("/robot_canvas.js", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(SPIFFS, "/robot_canvas.js", "application/javascript");
    });
    
    // ========== SIMULATOR MODE API ==========
    
    server_->on("/api/mode/simulator", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            this->handleSimulatorModeChange(request, data, len, index, total);
        }
    );
    
    server_->on("/api/mode/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handleGetModeStatus(request);
    });
    
    server_->on("/api/system/graph", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handleGetSystemGraph(request);
    });
    
    // Setup WebSocket for real-time updates
    ws_ = new AsyncWebSocket("/ws");
    ws_->onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client, 
                        AwsEventType type, void* arg, uint8_t* data, size_t len) {
        this->onWsEvent(server, client, type, arg, data, len);
    });
    server_->addHandler(ws_);
}

void WebServer::handleGetStatus(AsyncWebServerRequest* request) {
    StaticJsonDocument<2048> doc;
    
    RobotState state = RobotStateManager::getInstance().getState();
    
    doc["timestamp_ms"] = state.timestamp_ms;
    doc["mode"] = (state.mode == OperatingMode::MODE_1_OPEN_LOOP) ? "MODE_1_OPEN_LOOP" : "MODE_2_CLOSED_LOOP";
    doc["is_moving"] = state.is_moving;
    doc["is_homed"] = state.is_homed;
    doc["motion_allowed"] = state.motion_allowed;
    doc["fault_code"] = state.fault_code;
    
    JsonArray joints = doc.createNestedArray("joints");
    for (int i = 0; i < NUM_JOINTS; i++) {
        JsonObject joint = joints.createNestedObject();
        joint["id"] = i;
        joint["position"] = state.joint_positions[i];
        joint["velocity"] = state.joint_velocities[i];
    }
    
    JsonObject cart = doc.createNestedObject("cartesian");
    cart["x"] = state.cartesian_pos.x;
    cart["y"] = state.cartesian_pos.y;
    cart["z"] = state.cartesian_pos.z;
    
    sendJSON(request, doc);
}

void WebServer::handleMoveJoint(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, data, len);
    
    if (error) {
        sendError(request, "Invalid JSON");
        return;
    }
    
    uint8_t joint = doc["joint"];
    float position = doc["position"];
    float velocity = doc["velocity"] | 30.0f;
    
    if (MotionPlanner::getInstance().moveJoint(joint, position, velocity)) {
        StaticJsonDocument<256> response;
        response["success"] = true;
        response["message"] = "Motion started";
        sendJSON(request, response);
    } else {
        sendError(request, "Motion failed");
    }
}

void WebServer::handleMoveCartesian(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, data, len);
    
    if (error) {
        sendError(request, "Invalid JSON");
        return;
    }
    
    float target[6];
    target[0] = doc["target"]["x"];
    target[1] = doc["target"]["y"];
    target[2] = doc["target"]["z"];
    target[3] = doc["target"]["roll"] | 0.0f;
    target[4] = doc["target"]["pitch"] | 0.0f;
    target[5] = doc["target"]["yaw"] | 0.0f;
    
    float velocity = doc["velocity"] | 50.0f;
    bool respect_plane = doc["respect_plane"] | false;
    
    if (MotionPlanner::getInstance().moveCartesian(target, velocity, respect_plane)) {
        StaticJsonDocument<256> response;
        response["success"] = true;
        response["message"] = "Cartesian motion started";
        sendJSON(request, response);
    } else {
        sendError(request, "IK failed or motion not allowed");
    }
}

void WebServer::handleStopMotion(AsyncWebServerRequest* request) {
    MotionPlanner::getInstance().stopMotion();
    
    StaticJsonDocument<256> doc;
    doc["success"] = true;
    doc["message"] = "Motion stopped";
    sendJSON(request, doc);
}

void WebServer::handleEmergencyStop(AsyncWebServerRequest* request) {
    SafetyManager::getInstance().emergencyStop();
    
    StaticJsonDocument<256> doc;
    doc["success"] = true;
    doc["message"] = "Emergency stop activated";
    sendJSON(request, doc);
}

void WebServer::handleClearFault(AsyncWebServerRequest* request) {
    SafetyManager::getInstance().clearFaults();
    
    StaticJsonDocument<256> doc;
    doc["success"] = true;
    doc["message"] = "Faults cleared";
    sendJSON(request, doc);
}

void WebServer::handleSetMode(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, data, len);
    
    if (error) {
        sendError(request, "Invalid JSON");
        return;
    }
    
    const char* mode_str = doc["mode"];
    OperatingMode mode = (strcmp(mode_str, "MODE_2_CLOSED_LOOP") == 0) ? 
        OperatingMode::MODE_2_CLOSED_LOOP : OperatingMode::MODE_1_OPEN_LOOP;
    
    RobotConfig::getInstance().setMode(mode);
    
    StaticJsonDocument<256> response;
    response["success"] = true;
    response["mode"] = mode_str;
    sendJSON(request, response);
}

void WebServer::handleDefinePlane(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, data, len);
    
    if (error) {
        sendError(request, "Invalid JSON");
        return;
    }
    
    float p1[3], p2[3], p3[3];
    p1[0] = doc["p1"][0]; p1[1] = doc["p1"][1]; p1[2] = doc["p1"][2];
    p2[0] = doc["p2"][0]; p2[1] = doc["p2"][1]; p2[2] = doc["p2"][2];
    p3[0] = doc["p3"][0]; p3[1] = doc["p3"][1]; p3[2] = doc["p3"][2];
    
    const char* name = doc["name"] | "Plane";
    
    if (RobotConfig::getInstance().definePlaneFrom3Points(p1, p2, p3, name)) {
        StaticJsonDocument<256> response;
        response["success"] = true;
        response["message"] = "Plane defined";
        sendJSON(request, response);
    } else {
        sendError(request, "Invalid plane (collinear points)");
    }
}

void WebServer::sendJSON(AsyncWebServerRequest* request, JsonDocument& doc) {
    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
}

void WebServer::sendError(AsyncWebServerRequest* request, const char* message) {
    StaticJsonDocument<256> doc;
    doc["success"] = false;
    doc["error"] = message;
    sendJSON(request, doc);
}

// ============================================================================
// SIMULATOR MODE FLAG (Firmware Authoritative)
// ============================================================================

static bool simulator_mode_ = false;

// ============================================================================
// SIMULATOR MODE HANDLERS
// ============================================================================

void WebServer::handleSimulatorModeChange(AsyncWebServerRequest* request, 
    uint8_t* data, size_t len, size_t index, size_t total) {
    
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, data, len);
    
    if (error) {
        sendError(request, "Invalid JSON");
        return;
    }
    
    const char* target_mode = doc["target_mode"];
    bool to_simulator = strcmp(target_mode, "SIMULATOR") == 0;
    
    StaticJsonDocument<256> response;
    response["version"] = 1;
    response["type"] = "MODE_CHANGE_RESPONSE";
    
    // Validation rules
    RobotState state = RobotStateManager::getInstance().getState();
    
    if (to_simulator) {
        // Rules to enter simulator mode
        if (state.is_moving) {
            response["success"] = false;
            response["current_mode"] = simulator_mode_ ? "SIMULATOR" : "HARDWARE";
            response["rejection_reason"] = "Motion in progress";
            response["rejection_code"] = 1;
        } else {
            // Disable motors and enter simulator mode
            MotorManager::getInstance().emergencyStop();
            simulator_mode_ = true;
            response["success"] = true;
            response["current_mode"] = "SIMULATOR";
            Serial.println("[WEBSERVER] Entered SIMULATOR mode");
        }
    } else {
        // Rules to enter hardware mode
        uint32_t faults = SafetyManager::getInstance().getFaults();
        AlignmentStatus align = AlignmentChecker::getInstance().getLastStatus();
        
        if (SafetyManager::getInstance().isEStopped()) {
            response["success"] = false;
            response["current_mode"] = "SIMULATOR";
            response["rejection_reason"] = "E-Stop active";
            response["rejection_code"] = 3;
        } else if (align.critical) {
            response["success"] = false;
            response["current_mode"] = "SIMULATOR";
            response["rejection_reason"] = "Alignment fault";
            response["rejection_code"] = 5;
        } else if (faults != 0) {
            response["success"] = false;
            response["current_mode"] = "SIMULATOR";
            response["rejection_reason"] = "Uncleared faults";
            response["rejection_code"] = 6;
        } else {
            simulator_mode_ = false;
            response["success"] = true;
            response["current_mode"] = "HARDWARE";
            Serial.println("[WEBSERVER] Entered HARDWARE mode - ARMED");
        }
    }
    
    sendJSON(request, response);
}

void WebServer::handleGetModeStatus(AsyncWebServerRequest* request) {
    StaticJsonDocument<256> doc;
    doc["version"] = 1;
    doc["type"] = "MODE_STATUS";
    doc["payload"]["mode"] = simulator_mode_ ? "SIMULATOR" : "HARDWARE";
    doc["payload"]["simulator_allowed"] = true;
    doc["payload"]["hardware_armed"] = !simulator_mode_;
    doc["payload"]["estop_active"] = SafetyManager::getInstance().isEStopped();
    
    sendJSON(request, doc);
}

void WebServer::handleGetSystemGraph(AsyncWebServerRequest* request) {
    StaticJsonDocument<2048> doc;
    doc["version"] = 1;
    doc["type"] = "SYSTEM_GRAPH";
    
    RobotState state = RobotStateManager::getInstance().getState();
    AlignmentStatus align = AlignmentChecker::getInstance().getLastStatus();
    uint32_t faults = SafetyManager::getInstance().getFaults();
    
    JsonArray nodes = doc.createNestedArray("nodes");
    
    // Add node status
    JsonObject core = nodes.createNestedObject();
    core["id"] = "core";
    core["status"] = state.is_moving ? "ACTIVE" : "IDLE";
    core["uptime_ms"] = millis();
    
    JsonObject ik_mgr = nodes.createNestedObject();
    ik_mgr["id"] = "ik_mgr";
    ik_mgr["status"] = "ACTIVE";
    ik_mgr["solver"] = IKManager::getInstance().getActiveSolverName();
    
    JsonObject motor_mgr = nodes.createNestedObject();
    motor_mgr["id"] = "motor_mgr";
    motor_mgr["status"] = state.is_moving ? "ACTIVE" : "IDLE";
    motor_mgr["type"] = MotorManager::getInstance().getActiveType() == MotorType::SERVO ? "SERVO" : "DC";
    
    JsonObject safety = nodes.createNestedObject();
    safety["id"] = "safety";
    safety["status"] = faults != 0 ? "FAULT" : "ACTIVE";
    safety["fault_code"] = faults;
    
    JsonObject alignment = nodes.createNestedObject();
    alignment["id"] = "align";
    alignment["status"] = align.critical ? "FAULT" : (align.warning ? "WARNING" : "ACTIVE");
    alignment["max_error"] = align.max_joint_error;
    
    sendJSON(request, doc);
}

// ============================================================================
// WEBSOCKET EVENT HANDLER
// ============================================================================

void WebServer::onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                          AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("[WS] Client %u connected\n", client->id());
            ws_client_count_++;
            break;
            
        case WS_EVT_DISCONNECT:
            Serial.printf("[WS] Client %u disconnected\n", client->id());
            ws_client_count_--;
            break;
            
        case WS_EVT_DATA:
            // Handle incoming WebSocket messages
            handleWsMessage(client, data, len);
            break;
            
        case WS_EVT_ERROR:
            Serial.printf("[WS] Error from client %u\n", client->id());
            break;
            
        default:
            break;
    }
}

void WebServer::handleWsMessage(AsyncWebSocketClient* client, uint8_t* data, size_t len) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, data, len);
    
    if (error) {
        Serial.println("[WS] Invalid JSON received");
        return;
    }
    
    const char* type = doc["type"];
    
    if (strcmp(type, "IK_PREVIEW_REQUEST") == 0) {
        // Handle IK preview in WebSocket for lower latency
        JsonArray target = doc["payload"]["target_mm"];
        float target_pose[6] = {
            target[0], target[1], target[2],
            doc["payload"]["orientation_deg"][0] | 0,
            doc["payload"]["orientation_deg"][1] | 0,
            doc["payload"]["orientation_deg"][2] | 0
        };
        
        IKResult result = IKManager::getInstance().solve(target_pose, nullptr);
        
        StaticJsonDocument<512> response;
        response["version"] = 1;
        response["timestamp_ms"] = millis();
        response["type"] = "IK_PREVIEW_RESULT";
        response["payload"]["feasible"] = result.feasible;
        
        if (result.feasible) {
            JsonArray joints = response["payload"].createNestedArray("joints_deg");
            for (int i = 0; i < NUM_JOINTS; i++) {
                joints.add(result.joint_angles[i]);
            }
            response["payload"]["confidence"] = result.confidence;
            response["payload"]["error_mm"] = result.error_metric;
            response["payload"]["singularity_warning"] = result.singularity_warning;
        } else {
            response["payload"]["error_code"] = (int)result.error_code;
            response["payload"]["error_message"] = result.error_message;
        }
        
        String output;
        serializeJson(response, output);
        client->text(output);
    }
    else if (strcmp(type, "MODE_CHANGE_REQUEST") == 0) {
        // Handle mode change request via WebSocket
        bool to_sim = strcmp(doc["payload"]["target_mode"], "SIMULATOR") == 0;
        
        StaticJsonDocument<256> response;
        response["version"] = 1;
        response["timestamp_ms"] = millis();
        response["type"] = "MODE_CHANGE_RESPONSE";
        
        RobotState state = RobotStateManager::getInstance().getState();
        
        if (to_sim && !state.is_moving) {
            MotorManager::getInstance().emergencyStop();
            simulator_mode_ = true;
            response["payload"]["success"] = true;
            response["payload"]["current_mode"] = "SIMULATOR";
        } else if (!to_sim && !SafetyManager::getInstance().isEStopped()) {
            simulator_mode_ = false;
            response["payload"]["success"] = true;
            response["payload"]["current_mode"] = "HARDWARE";
        } else {
            response["payload"]["success"] = false;
            response["payload"]["current_mode"] = simulator_mode_ ? "SIMULATOR" : "HARDWARE";
            response["payload"]["rejection_reason"] = state.is_moving ? "Motion in progress" : "E-Stop active";
        }
        
        String output;
        serializeJson(response, output);
        client->text(output);
    }
}

// ============================================================================
// ENHANCED STATE BROADCAST
// ============================================================================

void WebServer::broadcastState() {
    uint32_t now = millis();
    if (now - last_broadcast_ < 100) return;  // 10Hz
    last_broadcast_ = now;
    
    if (ws_client_count_ == 0) return;  // No clients, skip
    
    StaticJsonDocument<1536> doc;
    doc["version"] = 1;
    doc["timestamp_ms"] = now;
    doc["type"] = "STATE_UPDATE";
    
    RobotState state = RobotStateManager::getInstance().getState();
    AlignmentStatus align = AlignmentChecker::getInstance().getLastStatus();
    
    // Mode
    doc["payload"]["mode"] = (state.mode == OperatingMode::MODE_1_OPEN_LOOP) ? 
                             "MODE_1_OPEN_LOOP" : "MODE_2_CLOSED_LOOP";
    doc["payload"]["is_moving"] = state.is_moving;
    doc["payload"]["is_homed"] = state.is_homed;
    doc["payload"]["motion_allowed"] = SafetyManager::getInstance().isMotionAllowed();
    doc["payload"]["fault_code"] = SafetyManager::getInstance().getFaults();
    
    // Joints
    JsonObject joints = doc["payload"].createNestedObject("joints");
    JsonArray pos = joints.createNestedArray("positions_deg");
    JsonArray vel = joints.createNestedArray("velocities_deg_s");
    JsonArray tgt = joints.createNestedArray("targets_deg");
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        pos.add(state.joint_positions[i]);
        vel.add(state.joint_velocities[i]);
        tgt.add(state.joint_targets[i]);
    }
    
    // TCP
    JsonObject tcp = doc["payload"].createNestedObject("tcp");
    JsonArray tcp_pos = tcp.createNestedArray("position_mm");
    JsonArray tcp_ori = tcp.createNestedArray("orientation_deg");
    
    tcp_pos.add(state.tcp_position[0]);
    tcp_pos.add(state.tcp_position[1]);
    tcp_pos.add(state.tcp_position[2]);
    tcp_ori.add(state.tcp_orientation[0]);
    tcp_ori.add(state.tcp_orientation[1]);
    tcp_ori.add(state.tcp_orientation[2]);
    
    // Motor type
    doc["payload"]["motor_type"] = MotorManager::getInstance().getActiveType() == MotorType::SERVO ? "SERVO" : "DC";
    doc["payload"]["active_frame"] = "BASE";
    
    String output;
    serializeJson(doc, output);
    ws_->textAll(output);
    
    // Broadcast alignment status at 2Hz
    static uint32_t last_align_broadcast = 0;
    if (now - last_align_broadcast >= 500) {
        last_align_broadcast = now;
        broadcastAlignmentStatus();
    }
}

void WebServer::broadcastAlignmentStatus() {
    StaticJsonDocument<512> doc;
    doc["version"] = 1;
    doc["timestamp_ms"] = millis();
    doc["type"] = "ALIGNMENT_STATUS";
    
    AlignmentStatus status = AlignmentChecker::getInstance().getLastStatus();
    
    doc["payload"]["aligned"] = status.aligned;
    doc["payload"]["warning"] = status.warning;
    doc["payload"]["critical"] = status.critical;
    doc["payload"]["speed_modifier"] = AlignmentChecker::getInstance().getSpeedModifier();
    doc["payload"]["max_joint_error_deg"] = status.max_joint_error;
    doc["payload"]["worst_joint"] = status.worst_joint;
    doc["payload"]["tcp_error_mm"] = status.fk_vs_tcp_error;
    doc["payload"]["fault_code"] = (int)status.fault_code;
    doc["payload"]["fault_message"] = status.fault_message;
    
    String output;
    serializeJson(doc, output);
    ws_->textAll(output);
}