
#ifndef __AP_HAL_PX4_SCHEDULER_H__
#define __AP_HAL_PX4_SCHEDULER_H__

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "AP_HAL_PX4_Namespace.h"
#include <sys/time.h>
#include <signal.h>

#define PX4_SCHEDULER_MAX_TIMER_PROCS 4

/* Scheduler implementation: */
class PX4::PX4Scheduler : public AP_HAL::Scheduler {
public:
    PX4Scheduler();
    /* AP_HAL::Scheduler methods */

    void     init(void *unused);
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
    void     reboot();
    void     panic(const prog_char_t *errormsg);
    bool     interrupts_are_blocked(void) { return _nested_atomic_ctr != 0; }

private:
    uint8_t _nested_atomic_ctr;
    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    static struct timeval _sketch_start_time;
    static AP_HAL::TimedProc _failsafe;

    static volatile bool _timer_suspended;
    static AP_HAL::TimedProc _timer_proc[PX4_SCHEDULER_MAX_TIMER_PROCS];
    static uint8_t _num_timer_procs;
    static bool    _in_timer_proc;

    // callable from interrupt handler
    static uint32_t _micros();
    static void _timer_event(int signo, siginfo_t *info, void *ucontext);

};
#endif
#endif // __AP_HAL_PX4_SCHEDULER_H__


