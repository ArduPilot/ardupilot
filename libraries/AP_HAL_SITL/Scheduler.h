
#ifndef __AP_HAL_SITL_SCHEDULER_H__
#define __AP_HAL_SITL_SCHEDULER_H__

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_HAL_SITL_Namespace.h"
#include <sys/time.h>

#define SITL_SCHEDULER_MAX_TIMER_PROCS 4

/* Scheduler implementation: */
class HALSITL::SITLScheduler : public AP_HAL::Scheduler {
public:
    SITLScheduler(SITL_State *sitlState);
    static SITLScheduler *from(AP_HAL::Scheduler *scheduler) {
        return static_cast<SITLScheduler*>(scheduler);
    }

    /* AP_HAL::Scheduler methods */

    void     init(void *unused);
    void     delay(uint16_t ms);
    uint32_t millis();
    uint32_t micros();
    uint64_t millis64();
    uint64_t micros64();
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

    void     reboot(bool hold_in_bootloader);
    void     panic(const prog_char_t *errormsg) NORETURN;

    bool     interrupts_are_blocked(void) {
        return _nested_atomic_ctr != 0;
    }

    void     sitl_begin_atomic() {
        _nested_atomic_ctr++;
    }
    void     sitl_end_atomic();

    // callable from interrupt handler
    static uint64_t _micros64();
    static void timer_event() {
        _run_timer_procs(true);
        _run_io_procs(true);
    }

private:
    SITL_State *_sitlState;
    uint8_t _nested_atomic_ctr;
    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    static struct timeval _sketch_start_time;
    static AP_HAL::Proc _failsafe;

    static void _run_timer_procs(bool called_from_isr);
    static void _run_io_procs(bool called_from_isr);

    static volatile bool _timer_suspended;
    static volatile bool _timer_event_missed;
    static AP_HAL::MemberProc _timer_proc[SITL_SCHEDULER_MAX_TIMER_PROCS];
    static AP_HAL::MemberProc _io_proc[SITL_SCHEDULER_MAX_TIMER_PROCS];
    static uint8_t _num_timer_procs;
    static uint8_t _num_io_procs;
    static bool    _in_timer_proc;
    static bool    _in_io_proc;

    void stop_clock(uint64_t time_usec);

    bool _initialized;
    uint64_t stopped_clock_usec;
};
#endif
#endif // __AP_HAL_SITL_SCHEDULER_H__


