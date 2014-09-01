
#ifndef __AP_HAL_VRBRAIN_SCHEDULER_H__
#define __AP_HAL_VRBRAIN_SCHEDULER_H__

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AP_HAL_VRBRAIN_Namespace.h"
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>
#include <systemlib/perf_counter.h>

#define VRBRAIN_SCHEDULER_MAX_TIMER_PROCS 8

#define APM_MAIN_PRIORITY    180
#define APM_TIMER_PRIORITY   181
#define APM_UART_PRIORITY     60
#define APM_IO_PRIORITY       59
#define APM_OVERTIME_PRIORITY 10
#define APM_STARTUP_PRIORITY  10

/* Scheduler implementation: */
class VRBRAIN::VRBRAINScheduler : public AP_HAL::Scheduler {
public:
	VRBRAINScheduler();
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
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);
    void     suspend_timer_procs();
    void     resume_timer_procs();
    void     reboot(bool hold_in_bootloader);
    void     panic(const prog_char_t *errormsg);

    bool     in_timerprocess();
    bool     system_initializing();
    void     system_initialized();
    void     hal_initialized() { _hal_initialized = true; }
    
private:
    bool _initialized;
    volatile bool _hal_initialized;
    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    AP_HAL::Proc _failsafe;
    volatile bool _timer_pending;

    volatile bool _timer_suspended;

    AP_HAL::MemberProc _timer_proc[VRBRAIN_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    volatile bool _in_timer_proc;

    AP_HAL::MemberProc _io_proc[VRBRAIN_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_io_procs;
    volatile bool _in_io_proc;

    volatile bool _timer_event_missed;

    pid_t _main_task_pid;
    pthread_t _timer_thread_ctx;
    pthread_t _io_thread_ctx;
    pthread_t _uart_thread_ctx;

    void *_timer_thread(void);
    void *_io_thread(void);
    void *_uart_thread(void);

    void _run_timers(bool called_from_timer_thread);
    void _run_io(void);

    void delay_microseconds_semaphore(uint16_t us);

    perf_counter_t  _perf_timers;
    perf_counter_t  _perf_io_timers;
    perf_counter_t  _perf_delay;
};
#endif
#endif // __AP_HAL_VRBRAIN_SCHEDULER_H__


