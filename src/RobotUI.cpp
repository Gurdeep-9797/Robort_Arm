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

void WebServer::broadcastState() {
    uint32_t now = millis();
    if (now - last_broadcast_ < 100) return;
    last_broadcast_ = now;
    
    StaticJsonDocument<1024> doc;
    RobotState state = RobotStateManager::getInstance().getState();
    
    doc["type"] = "state_update";
    doc["is_moving"] = state.is_moving;
    doc["progress"] = MotionPlanner::getInstance().getProgress();
    
    JsonArray joints = doc.createNestedArray("joints");
    for (int i = 0; i < NUM_JOINTS; i++) {
        joints.add(state.joint_positions[i]);
    }
    
    String output;
    serializeJson(doc, output);
    ws_->textAll(output);
}