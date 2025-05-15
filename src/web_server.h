#pragma once

#include <Arduino.h>
#include "../lib/RcDriver/inc/RcDriverStrategy.h"
#include "vl53l0x_manager.h"

// Define the structure to hold all sensor data for the web page
typedef struct {
    // RC data
    RcInfo rc;
    
    // VL53L0X sensor data
    uint16_t sensorDistances[COUNT_SENSORS];
    bool sensorStatus[COUNT_SENSORS];
    
    // Gyroscope data (placeholders)
    float gyroRoll;
    float gyroPitch;
    float gyroYaw;
    
    // Timestamps
    unsigned long lastRcUpdate;
    unsigned long lastSensorUpdate;
} WebPageData;

// Function to start the web server task
void startWebServerTask(const char* ssid, const char* password);

// Function to update data on the web server
void updateWebServerData(const WebPageData& newData);

// Helper function to convert sensor position to string for web display
const char* sensorPositionToStringWeb(SensorPosition pos);