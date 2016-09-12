#pragma once

#include <stdint.h>

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Boards.h"
#include "AP_HAL_Namespace.h"


class AP_HAL::Scheduler {
public:
    Scheduler() {}
    virtual void     init() = 0;
    virtual void     delay(uint16_t ms) = 0;

    /*
      delay for the given number of microseconds. This needs to be as
      accurate as possible - preferably within 100 microseconds.
     */
    virtual void     delay_microseconds(uint16_t us) = 0;

    /*
      delay for the given number of microseconds. On supported
      platforms this boosts the priority of the main thread for a
      short time when the time completes. The aim is to ensure the
      main thread runs at a higher priority than drivers for the start
      of each loop
     */
    virtual void     delay_microseconds_boost(uint16_t us) { delay_microseconds(us); }

    virtual void     register_delay_callback(AP_HAL::Proc,
                                             uint16_t min_time_ms) = 0;

    // register a high priority timer task
    virtual void     register_timer_process(AP_HAL::MemberProc) = 0;
    virtual bool     register_timer_process(AP_HAL::MemberProc proc, uint8_t freq_div)
    {
        register_timer_process(proc);
        return false;
    }

    // register a low priority IO task
    virtual void     register_io_process(AP_HAL::MemberProc) = 0;

    // suspend and resume both timer and IO processes
    virtual void     suspend_timer_procs() = 0;
    virtual void     resume_timer_procs() = 0;

    virtual bool     in_timerprocess() = 0;
    
    virtual void     register_timer_failsafe(AP_HAL::Proc,
                                             uint32_t period_us) = 0;

    virtual void     system_initialized() = 0;

    virtual void     reboot(bool hold_in_bootloader) = 0;

    /**
       optional function to stop clock at a given time, used by log replay
     */
    virtual void     stop_clock(uint64_t time_usec) {}
};
