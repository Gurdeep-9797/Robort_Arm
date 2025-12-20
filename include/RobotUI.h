#ifndef ROBOT_UI_H
#define ROBOT_UI_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "RobotCore.h"

class WebServer {
public:
    static WebServer& getInstance();
    bool begin(const char* ssid, const char* password);
    void broadcastState();

private:
    WebServer();
    AsyncWebServer* server_;
    AsyncWebSocket* ws_;
    uint32_t last_broadcast_;
    
    void setupRoutes();
    void setupWebSocket();
    
    void handleGetStatus(AsyncWebServerRequest* request);
    void handleMoveJoint(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total);
    void handleMoveCartesian(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total);
    void handleStopMotion(AsyncWebServerRequest* request);
    void handleEmergencyStop(AsyncWebServerRequest* request);
    void handleClearFault(AsyncWebServerRequest* request);
    void handleSetMode(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total);
    void handleDefinePlane(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total);
    
    void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
    
    void sendJSON(AsyncWebServerRequest* request, JsonDocument& doc);
    void sendError(AsyncWebServerRequest* request, const char* message);
};

#endif