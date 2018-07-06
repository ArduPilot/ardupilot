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

    /*
      end the priority boost from delay_microseconds_boost()
     */
    virtual void     boost_end(void) {}

    // register a function to be called by the scheduler if it needs
    // to sleep for more than min_time_ms
    virtual void     register_delay_callback(AP_HAL::Proc,
                                             uint16_t min_time_ms);
    // returns true if the Scheduler has called the delay callback
    // function.  If you are on the main thread that means your call
    // stack has the scheduler on it somewhere.
    virtual bool     in_delay_callback() const { return _in_delay_callback; }

    // register a high priority timer task
    virtual void     register_timer_process(AP_HAL::MemberProc) = 0;

    // register a low priority IO task
    virtual void     register_io_process(AP_HAL::MemberProc) = 0;

    virtual void     register_timer_failsafe(AP_HAL::Proc,
                                             uint32_t period_us) = 0;

    virtual void     system_initialized() = 0;

    virtual void     reboot(bool hold_in_bootloader) = 0;

    /**
       optional function to stop clock at a given time, used by log replay
     */
    virtual void     stop_clock(uint64_t time_usec) {}

    virtual bool     in_main_thread() const = 0;

    virtual void create_uavcan_thread() {};

    /*
      disable interrupts and return a context that can be used to
      restore the interrupt state. This can be used to protect
      critical regions

      Warning: may not be implemented on all HALs
     */
    virtual void *disable_interrupts_save(void) { return nullptr; }

    /*
      restore interrupt state from disable_interrupts_save()
     */
    virtual void restore_interrupts(void *) {}

    // called by subclasses when they need to delay for some time
    virtual void call_delay_cb();
    uint16_t _min_delay_cb_ms;

    /*
      priority_base is used to select what the priority for a new
      thread is relative to
     */
    enum priority_base {
        PRIORITY_BOOST,
        PRIORITY_MAIN,
        PRIORITY_SPI,
        PRIORITY_I2C,
        PRIORITY_CAN,
        PRIORITY_TIMER,
        PRIORITY_RCIN,
        PRIORITY_IO,
        PRIORITY_UART,
        PRIORITY_STORAGE
    };
    
    /*
      create a new thread
     */
    virtual bool thread_create(AP_HAL::MemberProc proc, const char *name,
                               uint32_t stack_size, priority_base base, int8_t priority) {
        return false;
    }

private:

    AP_HAL::Proc _delay_cb;
    bool _in_delay_callback : 1;

};
