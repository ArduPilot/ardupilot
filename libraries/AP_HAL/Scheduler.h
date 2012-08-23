
#ifndef __AP_HAL_SCHEDULER_H__
#define __AP_HAL_SCHEDULER_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::Scheduler {
public:
    Scheduler() {}
    virtual void          init() = 0;
    virtual void          delay(unsigned long ms) = 0;
    virtual unsigned long millis() = 0;
    virtual unsigned long micros() = 0;
    virtual void          delayMicroseconds(unsigned int us) = 0;
};

#endif // __AP_HAL_SCHEDULER_H__

