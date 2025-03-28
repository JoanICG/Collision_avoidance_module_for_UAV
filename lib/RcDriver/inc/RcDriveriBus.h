#pragma once

#include "RcDriverStrategy.h"

// IBUS Protocol Constants
#define IBUS_HEADER_BYTE1 0x20
#define IBUS_HEADER_BYTE2 0x40
#define IBUS_MAX_VALUE 0x7D0
#define IBUS_MIN_VALUE 0x3E8
#define IBUS_FRAME_LENGTH 32  // Standard IBUS frame length (header + data + checksum)
#define IBUS_DATA_LENGTH 28   // Length of data part (without header + checksum)
#define IBUS_MAX_CHANNELS 14  // Maximum number of channels in IBUS
#define IBUS_TX_INTERVAL 7    // Minimum ms between transmissions (143Hz max)

class RcDriveriBus : public RcDriverStrategy {
private:
    uint8_t buffer[IBUS_FRAME_LENGTH];
    uint8_t bufferIndex;
    State currentState;
    RcInfo rcInfo;
    uint8_t txBuffer[IBUS_FRAME_LENGTH];
    size_t txBufferSize;
    
    // Timing variables
    unsigned long lastRxTime;
    unsigned long lastTxTime;

    // Verify checksum for received data
    bool verifyChecksum();
    
public:
    RcDriveriBus();
    ~RcDriveriBus() override;
    
    // Process incoming byte from UART
    void processByte(uint8_t byte) override;
    
    // Encode RC data to IBUS format - returns 0 on success
    int encode() override;
    
    // Decode IBUS data to RC format - returns 0 on success
    int decode() override;
    
    // Get the decoded RC info
    void getRcInfo(RcInfo& info) override;
    
    // Set new RC data
    void setRcInfo(const RcInfo& info) override;
    
    // Get the encoded buffer for transmission
    uint8_t* getEncodedBuffer() override;
    
    // Get the size of the encoded buffer
    size_t getEncodedBufferSize() override;
    
    // Check if enough time has passed to send the next frame
    bool canTransmitNow() override;
    
    // Mark that a transmission has occurred now
    void markTransmitted() override;
    
    // Get time since last received packet
    unsigned long getTimeSinceLastRx() override;
};