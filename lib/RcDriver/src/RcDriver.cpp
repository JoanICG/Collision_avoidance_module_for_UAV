#include "../RcDriver.h"
#include "../inc/RcDriveriBus.h"

RcDriver::RcDriver() {
    strategy = nullptr;
    lastPacketTime = 0;
}

RcDriver::~RcDriver() {
    if (strategy != nullptr) {
        delete strategy;
        strategy = nullptr;
    }
}

void RcDriver::begin(int baudRate) {
    // Initialize Serial2 for RX2/TX2 pins
    Serial2.begin(baudRate, SERIAL_8N1, RX2_PIN, TX2_PIN);
    
    // Set default strategy to IBus if none is set
    if (strategy == nullptr) {
        strategy = new RcDriveriBus();
    }
    
    Serial.println("RcDriver initialized with Serial2 on pins RX2(16)/TX2(17)");
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

RcInfo RcDriver::getRcInfo() {
    if (strategy != nullptr) {
        return strategy->getRcInfo();
    }
    
    // Return empty RcInfo if no strategy is set
    RcInfo emptyInfo = {};
    return emptyInfo;
}

bool RcDriver::sendRcInfo(const RcInfo& info) {
    if (strategy == nullptr) {
        return false;
    }
    
    // Check if we can transmit according to the protocol's timing requirements
    if (!strategy->canTransmitNow()) {
        return false;
    }
    
    // Update strategy with new data
    strategy->setRcInfo(info);
    
    // Encode data
    int result = strategy->encode();
    if (result != 0) {
        return false;
    }
    
    // Get the encoded buffer and send it
    uint8_t* buffer = strategy->getEncodedBuffer();
    size_t bufferSize = strategy->getEncodedBufferSize();
    
    if (buffer == nullptr || bufferSize == 0) {
        return false;
    }
    
    // Send the data through Serial2
    Serial2.write(buffer, bufferSize);
    
    // Mark that transmission has occurred
    strategy->markTransmitted();
    
    return true;
}

unsigned long RcDriver::getTimeSinceLastRx() {
    if (strategy != nullptr) {
        return strategy->getTimeSinceLastRx();
    }
    return 0; // Return 0 if no strategy is set or no packets received
}