
#ifndef __AP_HAL_LINUX_SCHEDULER_H__
#define __AP_HAL_LINUX_SCHEDULER_H__

#include <AP_HAL_Linux.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <sys/time.h>
#include <pthread.h>

#define LINUX_SCHEDULER_MAX_TIMER_PROCS 10

class Linux::LinuxScheduler : public AP_HAL::Scheduler {
public:
    LinuxScheduler();
    void     init(void* machtnichts);
    void     delay(uint16_t ms);
    uint32_t millis();
    uint32_t micros();
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc,
                uint16_t min_time_ms);

    void     register_timer_process(AP_HAL::MemberProc);
    void     register_io_process(AP_HAL::MemberProc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();

    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);

    void     begin_atomic();
    void     end_atomic();

    bool     system_initializing();
    void     system_initialized();

    void     panic(const prog_char_t *errormsg);
    void     reboot(bool hold_in_bootloader);

    void     time_shift(uint32_t shift_ms);

private:
    struct timespec _sketch_start_time;    
    void _timer_handler(int signum);
    void _microsleep(uint32_t usec);

    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;

    AP_HAL::Proc _failsafe;

    bool _initialized;
    volatile bool _timer_pending;

    volatile bool _timer_suspended;

    AP_HAL::MemberProc _timer_proc[LINUX_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    volatile bool _in_timer_proc;

    AP_HAL::MemberProc _io_proc[LINUX_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_io_procs;
    volatile bool _in_io_proc;

    volatile bool _timer_event_missed;

    pthread_t _timer_thread_ctx;
    pthread_t _io_thread_ctx;
    pthread_t _uart_thread_ctx;

    void *_timer_thread(void);
    void *_io_thread(void);
    void *_uart_thread(void);

    void _run_timers(bool called_from_timer_thread);
    void _run_io(void);
    void _setup_realtime(uint32_t size);
};

#endif // CONFIG_HAL_BOARD

#endif // __AP_HAL_LINUX_SCHEDULER_H__
