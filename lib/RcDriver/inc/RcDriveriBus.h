#pragma once
#include "RcDriverStrategy.h"
#include "../RcDriver.h"  // For RcChannelData definition

class RcDriveriBus : public RcDriverStrategy {
public:
    RcDriveriBus() = default;
    void encode() override;
    void decode() override;
    void decodeBuffer(uint8_t* buffer, int length, RcChannelData* data) override;
};