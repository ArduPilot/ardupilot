#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_HAL_SITL_Namespace.h"
#include <sys/time.h>

#define SITL_SCHEDULER_MAX_TIMER_PROCS 4

/* Scheduler implementation: */
class HALSITL::Scheduler : public AP_HAL::Scheduler {
public:
    explicit Scheduler(SITL_State *sitlState);
    static Scheduler *from(AP_HAL::Scheduler *scheduler) {
        return static_cast<HALSITL::Scheduler*>(scheduler);
    }

    /* AP_HAL::Scheduler methods */

    void init();
    void delay(uint16_t ms);
    void delay_microseconds(uint16_t us);

    void register_timer_process(AP_HAL::MemberProc);
    void register_io_process(AP_HAL::MemberProc);

    void register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);

    bool in_main_thread() const override;
    void system_initialized();

    void reboot(bool hold_in_bootloader);

    bool interrupts_are_blocked(void) {
        return _nested_atomic_ctr != 0;
    }

    void sitl_begin_atomic() {
        _nested_atomic_ctr++;
    }
    void sitl_end_atomic();

    static void timer_event() {
        _run_timer_procs();
        _run_io_procs();
    }

    uint64_t stopped_clock_usec() const { return _stopped_clock_usec; }

    static void _run_io_procs();
    static bool _should_reboot;

    /*
      create a new thread
     */
    bool thread_create(AP_HAL::MemberProc, const char *name,
                       uint32_t stack_size, priority_base base, int8_t priority) override;
    
private:
    SITL_State *_sitlState;
    uint8_t _nested_atomic_ctr;
    static AP_HAL::Proc _failsafe;

    static void _run_timer_procs();

    static volatile bool _timer_event_missed;
    static AP_HAL::MemberProc _timer_proc[SITL_SCHEDULER_MAX_TIMER_PROCS];
    static AP_HAL::MemberProc _io_proc[SITL_SCHEDULER_MAX_TIMER_PROCS];
    static uint8_t _num_timer_procs;
    static uint8_t _num_io_procs;
    static bool _in_timer_proc;
    static bool _in_io_proc;

    void stop_clock(uint64_t time_usec);

    static void *thread_create_trampoline(void *ctx);
    
    bool _initialized;
    uint64_t _stopped_clock_usec;
    uint64_t _last_io_run;
    pthread_t _main_ctx;
};
#endif  // CONFIG_HAL_BOARD
