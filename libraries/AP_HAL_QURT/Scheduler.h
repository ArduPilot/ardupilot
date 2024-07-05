#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include "AP_HAL_QURT_Namespace.h"
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>

#define QURT_SCHEDULER_MAX_TIMER_PROCS 8

#define APM_MAIN_PRIORITY       180
#define APM_TIMER_PRIORITY      181
#define APM_UART_PRIORITY        60
#define APM_IO_PRIORITY          58

/* Scheduler implementation: */
class QURT::Scheduler : public AP_HAL::Scheduler
{
public:
    Scheduler();
    /* AP_HAL::Scheduler methods */

    void     init() override;
    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;
    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    void     suspend_timer_procs();
    void     resume_timer_procs();
    void     reboot(bool hold_in_bootloader) override;

    bool     in_main_thread() const override;
    void     hal_initialized();

    void     set_system_initialized() override;
    bool     is_system_initialized() override
    {
        return _initialized;
    }

    bool thread_create(AP_HAL::MemberProc proc, const char *name,
                       uint32_t stack_size, priority_base base, int8_t priority) override;

private:
    bool _initialized;
    volatile bool _hal_initialized;
    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    AP_HAL::Proc _failsafe;

    volatile bool _timer_suspended;

    AP_HAL::MemberProc _timer_proc[QURT_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    volatile bool _in_timer_proc;

    AP_HAL::MemberProc _io_proc[QURT_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_io_procs;
    volatile bool _in_io_proc;

    volatile bool _timer_event_missed;

    pthread_t _main_thread_ctx;
    pthread_t _timer_thread_ctx;
    pthread_t _io_thread_ctx;
    pthread_t _storage_thread_ctx;
    pthread_t _uart_thread_ctx;

    uint8_t calculate_thread_priority(priority_base base, int8_t priority) const;

    static void *_timer_thread(void *arg);
    static void *_io_thread(void *arg);
    static void *_storage_thread(void *arg);
    static void *_uart_thread(void *arg);

    void _run_timers(bool called_from_timer_thread);
    void _run_io(void);
};
#endif



