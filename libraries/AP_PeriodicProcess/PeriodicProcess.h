/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __PERIODICPROCESS_H__
#define __PERIODICPROCESS_H__

#include <stdint.h>

// the callback type for periodic processes. They are passed the time
// in microseconds since boot
typedef void (*ap_procedure)(uint32_t );

class AP_PeriodicProcess
{
public:
    virtual void        register_process(ap_procedure proc) = 0;
    virtual void        set_failsafe(ap_procedure proc) = 0;
    virtual bool        queue_process(ap_procedure proc) = 0;   // queue process to run as soon as possible after any currently running ap_processes complete.  returns true if it ran immediately
    virtual void        suspend_timer(void) = 0;
    virtual void        resume_timer(void) = 0;
    virtual bool        running() = 0;
};

#endif // __PERIODICPROCESS_H__
