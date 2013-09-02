
#ifndef __AP_HAL_SITL_SCHEDULER_H__
#define __AP_HAL_SITL_SCHEDULER_H__

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#include "AP_HAL_AVR_SITL_Namespace.h"
#include <sys/time.h>

#define SITL_SCHEDULER_MAX_TIMER_PROCS 4

/* Scheduler implementation: */
class AVR_SITL::SITLScheduler : public AP_HAL::Scheduler {
public:
    SITLScheduler();
    /* AP_HAL::Scheduler methods */

    void     init(void *unused);
    void     delay(uint16_t ms);
    uint32_t millis();
    uint32_t micros();
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc, uint16_t min_time_ms);

    void     register_timer_process(AP_HAL::TimedProc);
    void     register_io_process(AP_HAL::TimedProc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();

    void     register_timer_failsafe(AP_HAL::TimedProc, uint32_t period_us);

    bool     system_initializing();
    void     system_initialized();

    void     reboot(bool hold_in_bootloader);
    void     panic(const prog_char_t *errormsg);

    bool     interrupts_are_blocked(void) { return _nested_atomic_ctr != 0; }

    void     sitl_begin_atomic() { _nested_atomic_ctr++; }
    void     sitl_end_atomic();

    // callable from interrupt handler
    static uint32_t _micros();
    static void timer_event() { _run_timer_procs(true); _run_io_procs(true); }

private:
    uint8_t _nested_atomic_ctr;
    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    static struct timeval _sketch_start_time;
    static AP_HAL::TimedProc _failsafe;

    static void _run_timer_procs(bool called_from_isr);
    static void _run_io_procs(bool called_from_isr);

    static volatile bool _timer_suspended;
    static volatile bool _timer_event_missed;
    static AP_HAL::TimedProc _timer_proc[SITL_SCHEDULER_MAX_TIMER_PROCS];
    static AP_HAL::TimedProc _io_proc[SITL_SCHEDULER_MAX_TIMER_PROCS];
    static uint8_t _num_timer_procs;
    static uint8_t _num_io_procs;
    static bool    _in_timer_proc;
    static bool    _in_io_proc;
#ifdef __CYGWIN__
    static double _cyg_freq;
    static long _cyg_start;
    static double _cyg_sec();
#endif

    bool _initialized;

};
#endif
#endif // __AP_HAL_SITL_SCHEDULER_H__


