
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

    void     register_timer_process(AP_HAL::MemberProc);
    void     register_io_process(AP_HAL::MemberProc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();

    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);

    bool     system_initializing();
    void     system_initialized();

    void     panic(const prog_char_t *errormsg);
    void     reboot(bool hold_in_bootloader);

    void     set_timer_speed(uint16_t timer_hz);

private:
    static AVRTimer _timer;

    static volatile bool _in_timer_proc;

    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    bool _initialized;

    /* _timer_isr_event() and _run_timer_procs are static so they can be
     * called from an interrupt. */
    static void _timer_isr_event();
    static void _run_timer_procs(bool called_from_isr);

    static AP_HAL::Proc _failsafe;

    static volatile bool _timer_suspended;
    static volatile bool _timer_event_missed;
    static AP_HAL::MemberProc _timer_proc[AVR_SCHEDULER_MAX_TIMER_PROCS];
    static uint8_t _num_timer_procs;
    static volatile uint8_t _timer2_reset_value;

};
#endif // __AP_HAL_AVR_SCHEDULER_H__

