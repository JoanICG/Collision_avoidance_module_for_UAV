#ifndef RC_DRIVER_H
#define RC_DRIVER_H

#include "inc/RcDriverStrategy.h"
#include <Arduino.h>

// Data structure for RC channels
struct RcChannelData {
    static const int MAX_CHANNELS = 16;
    int channels[MAX_CHANNELS] = {0};
    unsigned long lastUpdateTime = 0;
    bool isValid = false;
};

class RcDriver {
private:
    RcDriverStrategy* strategy;
    RcChannelData channelData;
    
    // Buffer for serial data
    static const int BUFFER_SIZE = 64;
    uint8_t serialBuffer[BUFFER_SIZE];
    int bufferIndex = 0;

public:
    RcDriver();
    ~RcDriver();
    
    void setStrategy(RcDriverStrategy* strategy);
    void encode();
    void decode();
    
    // Data access methods
    const RcChannelData* getData() const { return &channelData; }
    
    // Serial handling
    void beginSerial(int baudRate = 115200);
};

#endif // RC_DRIVER_H