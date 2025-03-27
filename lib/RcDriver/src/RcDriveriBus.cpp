#include "../inc/RcDriveriBus.h"
#include <Arduino.h>

RcDriveriBus::RcDriveriBus() {
    // Initialize buffer and state
    bufferIndex = 0;
    currentState = WAITING_HEADER;
    
    // Initialize timing variables
    lastRxTime = 0;
    lastTxTime = 0;
    
    // Initialize RcInfo structure
    rcInfo.throttle = 0;
    rcInfo.yaw = 0;
    rcInfo.pitch = 0;
    rcInfo.roll = 0;
    rcInfo.Naux = IBUS_MAX_CHANNELS - 4;  // 4 main channels, rest are aux
    rcInfo.aux = new uint8_t[rcInfo.Naux];
    
    // Initialize aux channels
    for (int i = 0; i < rcInfo.Naux; i++) {
        rcInfo.aux[i] = 0;
    }
    
    // Initialize telemetry data
    rcInfo.telemetry.battery = 0;
    rcInfo.telemetry.temperature = 0;
    rcInfo.telemetry.altitude = 0;
    rcInfo.telemetry.speed = 0;
    
    rcInfo.timestamp = 0;
    txBufferSize = 0;
}

RcDriveriBus::~RcDriveriBus() {
    if (rcInfo.aux != nullptr) {
        delete[] rcInfo.aux;
        rcInfo.aux = nullptr;
    }
}

void RcDriveriBus::processByte(uint8_t byte) {
    switch (currentState) {
        case WAITING_HEADER:
            // Check for first header byte
            if (byte == IBUS_HEADER_BYTE1) {
                buffer[0] = byte;
                bufferIndex = 1;
                // Continue to check next byte immediately
                return;
            }
            
            // Check for second header byte if we already received the first
            if (bufferIndex == 1) {
                if (byte == IBUS_HEADER_BYTE2) {
                    buffer[bufferIndex++] = byte;
                    currentState = WAITING_DATA;
                } else {
                    // Reset if header not complete
                    bufferIndex = 0;
                }
            }
            break;
            
        case WAITING_DATA:
            // Collect data bytes
            buffer[bufferIndex++] = byte;
            
            // Check if we collected all data
            if (bufferIndex >= IBUS_FRAME_LENGTH) {
                currentState = WAITING_CHECKSUM;
                
                // Proceed to checksum validation
                if (verifyChecksum()) {
                    // Update last rx time on valid packet
                    lastRxTime = millis();
                    decode();
                } else {
                    Serial.println("Invalid IBus checksum");
                }
                
                // Reset for next frame
                currentState = WAITING_HEADER;
                bufferIndex = 0;
            }
            break;
            
        case WAITING_CHECKSUM:
            // This state is handled in WAITING_DATA when buffer is full
            currentState = WAITING_HEADER;
            bufferIndex = 0;
            break;
    }
}

bool RcDriveriBus::verifyChecksum() {
    // IBus checksum is calculated as 0xFFFF - sum of all data bytes
    uint16_t calculatedChecksum = 0xFFFF;
    for (int i = 0; i < IBUS_FRAME_LENGTH - 2; i++) {
        calculatedChecksum -= buffer[i];
    }
    
    // Extract the checksum from the last two bytes (little endian)
    uint16_t receivedChecksum = buffer[IBUS_FRAME_LENGTH - 1] << 8 | buffer[IBUS_FRAME_LENGTH - 2];
    
    return calculatedChecksum == receivedChecksum;
}

int RcDriveriBus::encode() {
    // Create an IBus frame in txBuffer
    
    // Header bytes
    txBuffer[0] = IBUS_HEADER_BYTE1;
    txBuffer[1] = IBUS_HEADER_BYTE2;
    
    // Channel data (16-bit values in little-endian format)
    // Map 0-255 back to 1000-2000 range for RC servos
    uint16_t throttle = map(rcInfo.throttle, 0, 255, 1000, 2000);
    uint16_t yaw = map(rcInfo.yaw, 0, 255, 1000, 2000);
    uint16_t pitch = map(rcInfo.pitch, 0, 255, 1000, 2000);
    uint16_t roll = map(rcInfo.roll, 0, 255, 1000, 2000);
    
    // Store main channels
    txBuffer[2] = throttle & 0xFF;
    txBuffer[3] = (throttle >> 8) & 0xFF;
    
    txBuffer[4] = yaw & 0xFF;
    txBuffer[5] = (yaw >> 8) & 0xFF;
    
    txBuffer[6] = pitch & 0xFF;
    txBuffer[7] = (pitch >> 8) & 0xFF;
    
    txBuffer[8] = roll & 0xFF;
    txBuffer[9] = (roll >> 8) & 0xFF;
    
    // Store aux channels
    for (int i = 0; i < rcInfo.Naux && i < (IBUS_MAX_CHANNELS - 4); i++) {
        uint16_t auxValue = map(rcInfo.aux[i], 0, 255, 1000, 2000);
        int bufferIndex = 10 + (i * 2);
        
        txBuffer[bufferIndex] = auxValue & 0xFF;
        txBuffer[bufferIndex + 1] = (auxValue >> 8) & 0xFF;
    }
    
    // Calculate checksum
    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < IBUS_FRAME_LENGTH - 2; i++) {
        checksum -= txBuffer[i];
    }
    
    // Store checksum (little-endian)
    txBuffer[IBUS_FRAME_LENGTH - 2] = checksum & 0xFF;
    txBuffer[IBUS_FRAME_LENGTH - 1] = (checksum >> 8) & 0xFF;
    
    // Set the buffer size
    txBufferSize = IBUS_FRAME_LENGTH;
    
    return 0; // Success
}

int RcDriveriBus::decode() {
    // Update timestamp
    rcInfo.timestamp = millis();
    
    // Extract channel values from buffer
    // Channels in IBus are 16-bit values starting at buffer[2]
    rcInfo.throttle = map((buffer[2] | (buffer[3] << 8)), 1000, 2000, 0, 255);
    rcInfo.yaw = map((buffer[4] | (buffer[5] << 8)), 1000, 2000, 0, 255);
    rcInfo.pitch = map((buffer[6] | (buffer[7] << 8)), 1000, 2000, 0, 255);
    rcInfo.roll = map((buffer[8] | (buffer[9] << 8)), 1000, 2000, 0, 255);
    
    // Fill auxiliary channels
    for (int i = 0; i < rcInfo.Naux; i++) {
        int bufferIndex = 10 + (i * 2);  // Start after the 4 main channels
        
        // Ensure we don't read beyond buffer boundary
        if (bufferIndex < IBUS_FRAME_LENGTH - 2) {
            uint16_t rawValue = buffer[bufferIndex] | (buffer[bufferIndex + 1] << 8);
            rcInfo.aux[i] = map(rawValue, 1000, 2000, 0, 255);
        }
    }
    
    #ifdef DEBUG
    Serial.print("Decoded IBus frame - Throttle: ");
    Serial.print(rcInfo.throttle);
    Serial.print(", Yaw: ");
    Serial.print(rcInfo.yaw);
    Serial.print(", Pitch: ");
    Serial.print(rcInfo.pitch);
    Serial.print(", Roll: ");
    Serial.println(rcInfo.roll);
    #endif
    
    return 0; // Success
}

RcInfo RcDriveriBus::getRcInfo() {
    return rcInfo;
}

void RcDriveriBus::setRcInfo(const RcInfo& info) {
    // Copy the data
    rcInfo.throttle = info.throttle;
    rcInfo.yaw = info.yaw;
    rcInfo.pitch = info.pitch;
    rcInfo.roll = info.roll;
    
    // Copy aux channels
    int auxToCopy = min(rcInfo.Naux, info.Naux);
    for (int i = 0; i < auxToCopy; i++) {
        rcInfo.aux[i] = info.aux[i];
    }
    
    // Copy telemetry
    rcInfo.telemetry = info.telemetry;
    rcInfo.timestamp = info.timestamp;
}

uint8_t* RcDriveriBus::getEncodedBuffer() {
    return txBuffer;
}

size_t RcDriveriBus::getEncodedBufferSize() {
    return txBufferSize;
}

bool RcDriveriBus::canTransmitNow() {
    // Check if enough time has passed since last transmission
    return (millis() - lastTxTime >= IBUS_TX_INTERVAL);
}

void RcDriveriBus::markTransmitted() {
    lastTxTime = millis();
}

unsigned long RcDriveriBus::getTimeSinceLastRx() {
    return millis() - lastRxTime;
}

