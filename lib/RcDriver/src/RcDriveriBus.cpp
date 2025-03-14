#include "../inc/RcDriveriBus.h"
#include <Arduino.h>

void RcDriveriBus::decode() {
    Serial.println("Decoding iBus data");
    // This is now mainly used for debugging
}

void RcDriveriBus::encode() {
    Serial.println("Encoding iBus data");
    // Implementation specific to iBus protocol
}

void RcDriveriBus::decodeBuffer(uint8_t* buffer, int length, RcChannelData* data) {
    // Verify we have a valid iBus packet (32 bytes, starts with 0x20 0x40)
    if (length < 32 || buffer[0] != 0x20 || buffer[1] != 0x40) {
        data->isValid = false;
        return;
    }
    
    // Calculate checksum (iBus uses a simple sum of all bytes)
    uint16_t chksum = 0xFFFF - (buffer[0] + buffer[1]);
    for (int i = 2; i < 30; i++) {
        chksum -= buffer[i];
    }
    
    uint16_t receivedChksum = buffer[30] | (buffer[31] << 8);
    
    // Verify checksum
    if (chksum != receivedChksum) {
        data->isValid = false;
        return;
    }
    
    // Decode channel data
    for (int i = 0; i < 14; i++) { // iBus typically has 14 channels
        if (i < RcChannelData::MAX_CHANNELS) {
            // Each channel is 2 bytes, little endian
            data->channels[i] = buffer[2 + i*2] | (buffer[3 + i*2] << 8);
        }
    }
    
    data->isValid = true;
}
