#ifndef RC_DRIVER_H
#define RC_DRIVER_H

#include "inc/RcDriverStrategy.h"

class RcDriver {
private:
    RcDriverStrategy* strategy; // Current strategy

public:
    // Constructor
    RcDriver(RcDriverStrategy* strategy = nullptr) : strategy(strategy) {}

    // Destructor
    ~RcDriver() {}

    // Set the strategy
    void setStrategy(RcDriverStrategy* strategy);
    // Execute strategy operations
    void encode();
    void decode();
};

#endif // RC_DRIVER_H