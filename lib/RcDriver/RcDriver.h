#pragma once

#include "inc/RcDriverStrategy.h"
#include <Arduino.h>

/**
 * For now, we will use Serial2 for RX2/TX2 pins
 * In a future version, different serials pins will 
 * be used in case of 2-way communication with RC and FC
*/
#define RX2_PIN 16  // ESP32 RX2 pin (GPIO16)
#define TX2_PIN 17  // ESP32 TX2 pin (GPIO17)

#define DEBUG 0// Set to 1 to enable debug output

class RcDriver {
private:
    RcDriverStrategy* strategy;
    unsigned long lastPacketTime;
    uint16_t* auxBuffer;  // Pre-allocated buffer for aux channels
    
public:
    // Constructor/Destructor
    RcDriver();
    ~RcDriver();
    
    // Initialize serial communication
    void begin(int baudRate = 115200);
    
    // Set strategy method (for changing protocols)
    void setStrategy(RcDriverStrategy* newStrategy);
    
    // Process incoming data from serial port
    void update();
    
    // Get the latest RC data from the strategy
    // Memory for aux channels is managed internally
    void getRcInfo(RcInfo& info);
    
    // Send RC data (if supported by protocol)
    int sendRcInfo(const RcInfo& info);
    
    // Get time since last received packet (delegates to strategy)
    unsigned long getTimeSinceLastRx();
};
