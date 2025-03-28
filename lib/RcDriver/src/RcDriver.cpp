#include "../RcDriver.h"
#include "../inc/RcDriveriBus.h"

RcDriver::RcDriver() {
    strategy = nullptr;
    lastPacketTime = 0;
    
    // Pre-allocate aux array with maximum possible size
    auxBuffer = new uint16_t[IBUS_MAX_CHANNELS];
}

RcDriver::~RcDriver() {
    if (strategy != nullptr) {
        delete strategy;
        strategy = nullptr;
    }
    
    // Free the pre-allocated aux buffer
    if (auxBuffer != nullptr) {
        delete[] auxBuffer;
        auxBuffer = nullptr;
    }
}

void RcDriver::begin(int baudRate) {
    // Initialize Serial2 for RX2/TX2 pins
    Serial2.begin(baudRate, SERIAL_8N1, RX2_PIN, TX2_PIN);
    
    // Set default strategy to IBus if none is set
    if (strategy == nullptr) {
        Serial.println("No strategy set, using default iBus strategy.");
        strategy = new RcDriveriBus();
    }
    
    Serial.println("RcDriver initialized with Serial2 on pins RX2(" + String(RX2_PIN) + ")/TX2(" + String(TX2_PIN) + ")");
}

void RcDriver::setStrategy(RcDriverStrategy* newStrategy) {
    if (strategy != nullptr) {
        delete strategy;
    }
    strategy = newStrategy;
}

void RcDriver::update() {
    // Process all available bytes in the serial buffer
    while (Serial2.available() > 0) {
        uint8_t incomingByte = Serial2.read();
        
        // Let the strategy handle the byte
        if (strategy != nullptr) {
            strategy->processByte(incomingByte);
        }
    }
}

void RcDriver::getRcInfo(RcInfo& info) {
    if (strategy != nullptr) {
        // Make sure info.aux points to our managed buffer before passing to strategy
        info.aux = auxBuffer;
        
        // Let the strategy fill in the values
        strategy->getRcInfo(info);
    } else {
        // Initialize with default values if no strategy is set
        info.throttle = 0;
        info.yaw = 0;
        info.pitch = 0;
        info.roll = 0;
        info.Naux = 0;
        info.aux = auxBuffer;  // Still provide the buffer, but indicate 0 channels
        info.timestamp = 0;
        info.telemetry = {0};
    }
}

int RcDriver::sendRcInfo(const RcInfo& info) {
    if (strategy == nullptr) {
        return -1; // No strategy set
    }
    
    // Check if we can transmit according to the protocol's timing requirements
    if (!strategy->canTransmitNow()) {
        return -2; // Not ready to transmit yet
    }
    
    // Update strategy with new data
    strategy->setRcInfo(info);
    
    // Encode data
    int result = strategy->encode();
    if (result != 0) {
        return -3; // Encoding failed
    }
    
    // Get the encoded buffer and send it
    uint8_t* buffer = strategy->getEncodedBuffer();
    size_t bufferSize = strategy->getEncodedBufferSize();
    
    if (buffer == nullptr || bufferSize == 0) {
        return -4; // Invalid buffer
    }
    
    // Send the data through Serial2
    Serial2.write(buffer, bufferSize);
    
    // Mark that transmission has occurred
    strategy->markTransmitted();
    
    return 0;
}

unsigned long RcDriver::getTimeSinceLastRx() {
    if (strategy != nullptr) {
        return strategy->getTimeSinceLastRx();
    }
    return 0; // Return 0 if no strategy is set or no packets received
}