#pragma once

#include "inc/RcDriverStrategy.h"
#include <Arduino.h>

#define RX2_PIN 16  // ESP32 RX2 pin (GPIO16)
#define TX2_PIN 17  // ESP32 TX2 pin (GPIO17)

class RcDriver {
private:
    RcDriverStrategy* strategy;
    unsigned long lastPacketTime;
    
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
    RcInfo getRcInfo();
    
    // Send RC data (if supported by protocol)
    bool sendRcInfo(const RcInfo& info);
    
    // Get time since last received packet (delegates to strategy)
    unsigned long getTimeSinceLastRx();
};
