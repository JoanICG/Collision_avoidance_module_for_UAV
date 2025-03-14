#include "../RcDriver.h"
#include <Arduino.h>

void RcDriver::setStrategy(RcDriverStrategy* strategy)
{
    this->strategy = strategy;
}

void RcDriver::encode()
{
    this->strategy->encode();
}

void RcDriver::decode()
{
    this->strategy->decode();
}
