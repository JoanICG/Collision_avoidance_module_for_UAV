#ifndef RC_DRIVER_IBUS_H
#define RC_DRIVER_IBUS_H

#include "RcDriverStrategy.h"

class RcDriveriBus : public RcDriverStrategy
{
    void decode() override;
    void encode() override;
};

#endif // RC_DRIVER_IBUS_H