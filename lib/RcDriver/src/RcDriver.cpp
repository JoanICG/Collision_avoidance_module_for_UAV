#include "../RcDriver.h"
#include <Arduino.h>

RcDriver::RcDriver() : strategy(nullptr) {
    // Initialize buffers
    memset(serialBuffer, 0, BUFFER_SIZE);
}

RcDriver::~RcDriver() {
    // Clean up if needed
}

void RcDriver::beginSerial(int baudRate) {
    // Initialize Serial2 for RX2 pin on ESP32
    Serial2.begin(baudRate, SERIAL_8N1, 16, 17);  // RX2=16, TX2=17 on ESP32
}

void RcDriver::setStrategy(RcDriverStrategy* strategy) {
    this->strategy = strategy;
}

void RcDriver::encode() {
    Serial.println("Encoding data");
    if (this->strategy) {
        this->strategy->encode();
    }
}

void RcDriver::decode() {
    if (!this->strategy) {
        return;
    }
    
    // Check for available data from Serial2
    while (Serial2.available()) {
        uint8_t data = Serial2.read();
        
        // Add to buffer
        if (bufferIndex < BUFFER_SIZE) {
            serialBuffer[bufferIndex++] = data;
        }
        
        // Process the buffer if we have enough data
        if (bufferIndex >= 32) { // Assuming iBus packet size of 32 bytes
            // Let the strategy decode the data into channel values
            this->strategy->decodeBuffer(serialBuffer, bufferIndex, &channelData);
            
            // Update timestamp
            channelData.lastUpdateTime = millis();
            
            // Reset buffer index for next packet
            bufferIndex = 0;
        }
    }
}
