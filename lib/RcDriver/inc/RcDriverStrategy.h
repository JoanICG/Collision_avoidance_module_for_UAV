#ifndef RC_DRIVER_STRATEGY_H
#define RC_DRIVER_STRATEGY_H

#pragma once

class RcDriverStrategy
{
public:
    virtual void decode() = 0;
    virtual void encode() = 0;
};

#endif // RC_DRIVER_STRATEGY_H