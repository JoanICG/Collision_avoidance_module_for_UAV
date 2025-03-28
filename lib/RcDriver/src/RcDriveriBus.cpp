#include "../inc/RcDriveriBus.h"
#include <Arduino.h>

RcDriveriBus::RcDriveriBus() {
    //TODO, preguntar si aixo del debug es bona idea
    #ifdef DEBUG
    Serial.println("Initializing RcDriveriBus...");
    #endif

    // Initialize buffer and state
    bufferIndex = 0;
    currentState = WAITING_HEADER;
    
    // Initialize timing variables
    lastRxTime = 0;
    lastTxTime = 0;
    
    // Initialize RcInfo structure
    rcInfo.throttle = IBUS_MIN_VALUE;  // Initialize to min value (1000)
    rcInfo.yaw = IBUS_MIN_VALUE;
    rcInfo.pitch = IBUS_MIN_VALUE;
    rcInfo.roll = IBUS_MIN_VALUE;
    rcInfo.Naux = IBUS_MAX_CHANNELS - 4;  // 4 main channels, rest are aux
    rcInfo.aux = new uint16_t[rcInfo.Naux];  // Changed from uint8_t to uint16_t
    
    // Initialize aux channels
    for (int i = 0; i < rcInfo.Naux; i++) {
        rcInfo.aux[i] = IBUS_MIN_VALUE;  // Initialize to min value
    }
    
    // Initialize telemetry data
    rcInfo.telemetry.battery = 0;
    rcInfo.telemetry.temperature = 0;
    rcInfo.telemetry.altitude = 0;
    rcInfo.telemetry.speed = 0;
    
    rcInfo.timestamp = 0;
    txBufferSize = 0;

    #ifdef DEBUG
    Serial.println("RcDriveriBus initialized.");
    #endif
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
    // Store as-is without mapping
    
    // Transformacio a little endian
    // Store main channels
    txBuffer[2] = rcInfo.throttle & 0xFF;
    txBuffer[3] = (rcInfo.throttle >> 8) & 0xFF;
    
    txBuffer[4] = rcInfo.yaw & 0xFF;
    txBuffer[5] = (rcInfo.yaw >> 8) & 0xFF;
    
    txBuffer[6] = rcInfo.pitch & 0xFF;
    txBuffer[7] = (rcInfo.pitch >> 8) & 0xFF;
    
    txBuffer[8] = rcInfo.roll & 0xFF;
    txBuffer[9] = (rcInfo.roll >> 8) & 0xFF;
    
    // Store aux channels
    for (int i = 0; i < rcInfo.Naux && i < (IBUS_MAX_CHANNELS - 4); i++) {
        int bufferIndex = 10 + (i * 2);
        
        txBuffer[bufferIndex] = rcInfo.aux[i] & 0xFF;
        txBuffer[bufferIndex + 1] = (rcInfo.aux[i] >> 8) & 0xFF;
    }
    
    // Calculate checksum
    //TODO, preguntar si es millor fer el checksum al final, 
    // o fer-ho a mida que es va enviant
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
    
    // Extract channel values from buffer directly without mapping
    // Channels in IBus are 16-bit values starting at buffer[2]
    rcInfo.throttle = buffer[2] | (buffer[3] << 8);
    rcInfo.yaw = buffer[4] | (buffer[5] << 8);
    rcInfo.pitch = buffer[6] | (buffer[7] << 8);
    rcInfo.roll = buffer[8] | (buffer[9] << 8);
    
    // Fill auxiliary channels
    for (int i = 0; i < rcInfo.Naux; i++) {
        int bufferIndex = 10 + (i * 2);  // Start after the 4 main channels
        
        // Ensure we don't read beyond buffer boundary
        if (bufferIndex < IBUS_FRAME_LENGTH - 2) {
            rcInfo.aux[i] = buffer[bufferIndex] | (buffer[bufferIndex + 1] << 8);
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

//TODO, mirar si es millor fer un memcpy, o copiar un a un
void RcDriveriBus::getRcInfo(RcInfo& info) {
    // Copy the main channel data
    info.throttle = rcInfo.throttle;
    info.yaw = rcInfo.yaw;
    info.pitch = rcInfo.pitch;
    info.roll = rcInfo.roll;
    info.Naux = rcInfo.Naux;
    
    // Copy aux channel data if a buffer has been provided
    if (info.aux != nullptr && rcInfo.aux != nullptr && info.Naux > 0) {
        for (int i = 0; i < info.Naux; i++) {
            info.aux[i] = rcInfo.aux[i];
        }
    }
    
    // Copy telemetry and timestamp
    info.telemetry = rcInfo.telemetry;
    info.timestamp = rcInfo.timestamp;
}

//Data is copied one by one,so in a future version you could check for problems or errors
//TODO, mirar si realment fa falta, o simplement es pot fer un memcpy
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

