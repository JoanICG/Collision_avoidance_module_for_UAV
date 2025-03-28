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
    uint16_t throttle;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;
    uint8_t Naux;
    uint16_t* aux;      // The strategy will NOT allocate memory for this
                        // RcDriver class is responsible for memory management
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
    
    // Fill the provided RcInfo structure with current data
    // IMPORTANT: The strategy must not allocate or deallocate memory for aux
    virtual void getRcInfo(RcInfo& info) = 0;
    
    // Update internal data from the provided RcInfo
    // IMPORTANT: The strategy must not take ownership of the aux pointer
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
