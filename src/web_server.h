#pragma once

#include <Arduino.h>
#include "../lib/RcDriver/inc/RcDriverStrategy.h"
#include "vl53l0x_manager.h"
#include <ESPAsyncWebServer.h>  // Cambio de AsyncWebServer.h a ESPAsyncWebServer.h

struct OrientationWebData {
    float pitch;
    float roll;
    float yaw;
    float accelX;
    float accelY;
    float accelZ;
    unsigned long timestamp;
};

// Define the structure to hold all sensor data for the web page
typedef struct {
    // RC data
    RcInfo rc;
    
    // VL53L0X sensor data
    uint16_t sensorDistances[5];
    bool sensorStatus[5];
    
    // Gyroscope data (placeholders)
    float gyroRoll;
    float gyroPitch;
    float gyroYaw;
    
    // Timestamps
    unsigned long lastRcUpdate;
    unsigned long lastSensorUpdate;
    OrientationWebData orientation;
} WebPageData;

// Add this to your web_server.h file




// Function to initialize the web server system
void initWebServer(AsyncWebServer* server, AsyncWebSocket* ws, const char* ssid, const char* password);

// Function to update data on the web server
void updateWebServerData(const WebPageData& newData);

// Helper function to convert sensor position to string for web display
const char* sensorPositionToStringWeb(SensorPosition pos);

// Función para iniciar tarea periódica de actualizaciones web
void startWebUpdateTask(int updateIntervalMs);

// Expose HTML content
extern const char index_html[] PROGMEM;