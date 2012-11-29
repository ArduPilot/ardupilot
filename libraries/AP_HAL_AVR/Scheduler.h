
#ifndef __AP_HAL_AVR_SCHEDULER_H__
#define __AP_HAL_AVR_SCHEDULER_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

#define AVR_SCHEDULER_MAX_TIMER_PROCS 4

class AP_HAL_AVR::AVRScheduler : public AP_HAL::Scheduler {
public:
    AVRScheduler();
    /* AP_HAL::Scheduler methods */

    /* init: implementation-specific void* argument expected to be an
     * AP_HAL_AVR::ISRRegistry*. */
    void     init(void *isrregistry);
    void     delay(uint32_t ms);
    uint32_t millis();
    uint32_t micros();
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc, uint16_t min_time_ms);
    void     register_timer_process(AP_HAL::TimedProc);
    bool     defer_timer_process(AP_HAL::TimedProc);
    void     register_timer_failsafe(AP_HAL::TimedProc, uint32_t period_us);
    void     suspend_timer_procs();
    void     resume_timer_procs();
    void     begin_atomic();
    void     end_atomic();

private:
    /* Implementation specific methods: */
    /* timer_event() is static so it can be called from an interrupt.
     * (This is effectively a singleton class.)
     * _prefix: this method must be public */
    static void _timer_event();

    /* _micros() is the implementation of micros() as a static private method
     * so we can use it from inside _timer_event() without virtual dispatch. */
    static uint32_t _micros();

    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    static AP_HAL::TimedProc _failsafe;

    static volatile bool _timer_suspended;
    static AP_HAL::TimedProc _timer_proc[AVR_SCHEDULER_MAX_TIMER_PROCS];
    static AP_HAL::TimedProc _defered_timer_proc;
    static uint8_t _num_timer_procs;
    static bool    _in_timer_proc;

};
#endif // __AP_HAL_AVR_SCHEDULER_H__

