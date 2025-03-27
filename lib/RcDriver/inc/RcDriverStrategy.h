#pragma once

#include <Arduino.h>

// Define State enum
typedef enum State{
    WAITING_HEADER,
    WAITING_DATA,
    WAITING_CHECKSUM
}State;

// Telemetry Info structure
typedef struct TelemetryInfo{
    uint8_t battery;
    uint8_t temperature;
    uint8_t altitude;
    uint8_t speed;
}TelemetryInfo;

// RC Info structure
typedef struct RcInfo{
    uint8_t throttle;
    uint8_t yaw;
    uint8_t pitch;
    uint8_t roll;
    uint8_t Naux;
    uint8_t* aux;
    TelemetryInfo telemetry;
    unsigned long timestamp;
}RcInfo;

class RcDriverStrategy {
public:
    RcDriverStrategy() = default;
    virtual ~RcDriverStrategy() = default;
    
    // Process incoming byte from UART
    virtual void processByte(uint8_t byte) = 0;
    
    // Encode RC data - returns 0 if success, <0 if error
    virtual int encode() = 0;

    // Decode RC data - returns 0 if success, <0 if error
    virtual int decode() = 0;
    
    // Get the decoded RC info
    virtual RcInfo getRcInfo() = 0;
    
    // Set new RC data
    virtual void setRcInfo(const RcInfo& info) = 0;
    
    // Get the encoded buffer for transmission
    virtual uint8_t* getEncodedBuffer() = 0;
    
    // Get the size of the encoded buffer
    virtual size_t getEncodedBufferSize() = 0;
    
    // Check if enough time has passed to send the next frame
    virtual bool canTransmitNow() = 0;
    
    // Mark that a transmission has occurred now
    virtual void markTransmitted() = 0;
    
    // Get time since last received packet
    virtual unsigned long getTimeSinceLastRx() = 0;
};
