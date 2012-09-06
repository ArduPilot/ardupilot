
#ifndef __AP_HAL_SCHEDULER_H__
#define __AP_HAL_SCHEDULER_H__

#include "AP_HAL_Namespace.h"

#include <stdint.h>

class AP_HAL::Scheduler {
public:
    Scheduler() {}
    virtual void     init() = 0;
    virtual void     delay(uint32_t ms) = 0;
    virtual uint32_t millis() = 0;
    virtual uint32_t micros() = 0;
    virtual void     delay_microseconds(uint16_t us) = 0;
    virtual void     register_delay_callback(void(*cb)(void));
};

#endif // __AP_HAL_SCHEDULER_H__

