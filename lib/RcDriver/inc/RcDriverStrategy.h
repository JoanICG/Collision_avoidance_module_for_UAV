#ifndef RC_DRIVER_STRATEGY_H
#define RC_DRIVER_STRATEGY_H

#pragma once
#include <Arduino.h>

// Forward declaration
struct RcChannelData;

class RcDriverStrategy {
public:
    RcDriverStrategy() = default;
    virtual ~RcDriverStrategy() = default;
    
    virtual void encode() = 0;
    virtual void decode() = 0;
    
    // Added method to decode from buffer
    virtual void decodeBuffer(uint8_t* buffer, int length, RcChannelData* data) = 0;
};

#endif // RC_DRIVER_STRATEGY_H