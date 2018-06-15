#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AP_HAL_VRBRAIN_Namespace.h"
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>
#include <systemlib/perf_counter.h>

#define VRBRAIN_SCHEDULER_MAX_TIMER_PROCS 8

#define APM_MAIN_PRIORITY_BOOST 241
#define APM_MAIN_PRIORITY       180
#define APM_TIMER_PRIORITY      181
#define APM_SPI_PRIORITY        242
#define APM_CAN_PRIORITY        179
#define APM_I2C_PRIORITY        178
#define APM_UART_PRIORITY        60
#define APM_STORAGE_PRIORITY     59
#define APM_IO_PRIORITY          58
#define APM_SHELL_PRIORITY       57
#define APM_OVERTIME_PRIORITY    10
#define APM_STARTUP_PRIORITY     10

/* how long to boost priority of the main thread for each main
   loop. This needs to be long enough for all interrupt-level drivers
   (mostly SPI drivers) to run, and for the main loop of the vehicle
   code to start the AHRS update.

   Priority boosting of the main thread in delay_microseconds_boost()
   avoids the problem that drivers in hpwork all happen to run right
   at the start of the period where the main vehicle loop is calling
   wait_for_sample(). That causes main loop timing jitter, which
   reduces performance. Using the priority boost the main loop
   temporarily runs at a priority higher than hpwork and the timer
   thread, which results in much more consistent loop timing. 
*/
#define APM_MAIN_PRIORITY_BOOST_USEC 150

#define APM_MAIN_THREAD_STACK_SIZE 8192

/* Scheduler implementation: */
class VRBRAIN::VRBRAINScheduler : public AP_HAL::Scheduler {
public:
	VRBRAINScheduler();
    /* AP_HAL::Scheduler methods */

    void     init();
    void     delay(uint16_t ms);
    void     delay_microseconds(uint16_t us);
    void     delay_microseconds_boost(uint16_t us);
    void     register_timer_process(AP_HAL::MemberProc);
    void     register_io_process(AP_HAL::MemberProc);
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);
    void     reboot(bool hold_in_bootloader);

    bool     in_main_thread() const override;
    void     system_initialized();
    void     hal_initialized() { _hal_initialized = true; }

    void create_uavcan_thread() override;

private:
    bool _initialized;
    volatile bool _hal_initialized;
    AP_HAL::Proc _failsafe;

    AP_HAL::MemberProc _timer_proc[VRBRAIN_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    volatile bool _in_timer_proc;

    AP_HAL::MemberProc _io_proc[VRBRAIN_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_io_procs;
    volatile bool _in_io_proc;

    pid_t _main_task_pid;
    pthread_t _timer_thread_ctx;
    pthread_t _io_thread_ctx;
    pthread_t _storage_thread_ctx;
    pthread_t _uart_thread_ctx;
    pthread_t _uavcan_thread_ctx;

    struct _uavcan_thread_arg {
        VRBRAINScheduler *sched;
        uint8_t uavcan_number;
    };

    static void *_timer_thread(void *arg);
    static void *_io_thread(void *arg);
    static void *_storage_thread(void *arg);
    static void *_uart_thread(void *arg);
    static void *_uavcan_thread(void *arg);

    void _run_timers();
    void _run_io(void);

    void delay_microseconds_semaphore(uint16_t us);

    perf_counter_t  _perf_timers;
    perf_counter_t  _perf_io_timers;
    perf_counter_t  _perf_storage_timer;
    perf_counter_t  _perf_delay;
};
#endif
