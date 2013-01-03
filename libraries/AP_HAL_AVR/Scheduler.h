
#ifndef __AP_HAL_AVR_SCHEDULER_H__
#define __AP_HAL_AVR_SCHEDULER_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

#define AVR_SCHEDULER_MAX_TIMER_PROCS 4

/* Class for managing the AVR Timers: */
class AP_HAL_AVR::AVRTimer {
public:
    static void     init();
    static uint32_t millis();
    static uint32_t micros();
    static void     delay_microseconds(uint16_t us);
};

/* Scheduler implementation: */
class AP_HAL_AVR::AVRScheduler : public AP_HAL::Scheduler {
    /* AVRSemaphore gets access to _in_timer_proc */
    friend class AVRSemaphore;
public:
    AVRScheduler();
    /* AP_HAL::Scheduler methods */

    /* init: implementation-specific void* argument expected to be an
     * AP_HAL_AVR::ISRRegistry*. */
    void     init(void *isrregistry);
    void     delay(uint16_t ms);
    uint32_t millis();
    uint32_t micros();
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc, uint16_t min_time_ms);
    void     register_timer_process(AP_HAL::TimedProc);
    void     register_timer_failsafe(AP_HAL::TimedProc, uint32_t period_us);
    void     suspend_timer_procs();
    void     resume_timer_procs();
    void     begin_atomic();
    void     end_atomic();
    void     panic(const prog_char_t *errormsg);
    void     reboot();

protected:
    static volatile bool _in_timer_proc;

private:
    static AVRTimer _timer;

    /* timer_event() is static so it can be called from an interrupt.
     * (This is effectively a singleton class.)
     * _prefix: this method must be public */
    static void _timer_event();

    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    static AP_HAL::TimedProc _failsafe;

    static volatile bool _timer_suspended;
    static AP_HAL::TimedProc _timer_proc[AVR_SCHEDULER_MAX_TIMER_PROCS];
    static uint8_t _num_timer_procs;

    uint8_t _nested_atomic_ctr;

};
#endif // __AP_HAL_AVR_SCHEDULER_H__

