
#ifndef __AP_HAL_SCHEDULER_H__
#define __AP_HAL_SCHEDULER_H__

#include "AP_HAL_Namespace.h"

#include <stdint.h>
#include <AP_Progmem.h>

class AP_HAL::Scheduler {
public:
    Scheduler() {}
    virtual void     init(void* implspecific) = 0;
    virtual void     delay(uint16_t ms) = 0;
    virtual uint32_t millis() = 0;
    virtual uint32_t micros() = 0;
    virtual void     delay_microseconds(uint16_t us) = 0;
    virtual void     register_delay_callback(AP_HAL::Proc,
                        uint16_t min_time_ms) = 0;

    virtual void     register_timer_process(AP_HAL::TimedProc) = 0;
    virtual void     suspend_timer_procs() = 0;
    virtual void     resume_timer_procs() = 0;
    
    virtual void     register_timer_failsafe(AP_HAL::TimedProc,
                        uint32_t period_us) = 0;

    virtual void     panic(const prog_char_t *errormsg) = 0;
    virtual void     reboot() = 0;
};

#endif // __AP_HAL_SCHEDULER_H__

