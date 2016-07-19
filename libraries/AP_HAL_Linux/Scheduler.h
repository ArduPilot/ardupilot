#pragma once

#include <pthread.h>

#include "AP_HAL_Linux.h"
#include "Semaphores.h"
#include "Thread.h"

#define LINUX_SCHEDULER_MAX_TIMER_PROCS 10
#define LINUX_SCHEDULER_MAX_TIMESLICED_PROCS 10
#define LINUX_SCHEDULER_MAX_IO_PROCS 10

#define AP_LINUX_SENSORS_STACK_SIZE  256 * 1024
#define AP_LINUX_SENSORS_SCHED_POLICY  SCHED_FIFO
#define AP_LINUX_SENSORS_SCHED_PRIO 12

namespace Linux {

class Scheduler : public AP_HAL::Scheduler {
public:
    Scheduler();

    static Scheduler *from(AP_HAL::Scheduler *scheduler) {
        return static_cast<Scheduler*>(scheduler);
    }

    void     init();
    void     delay(uint16_t ms);
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc,
                uint16_t min_time_ms);

    void     register_timer_process(AP_HAL::MemberProc);
    bool     register_timer_process(AP_HAL::MemberProc, uint8_t);
    void     register_io_process(AP_HAL::MemberProc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();

    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);

    void     system_initialized();

    void     reboot(bool hold_in_bootloader);

    void     stop_clock(uint64_t time_usec);

    uint64_t stopped_clock_usec() const { return _stopped_clock_usec; }

    void microsleep(uint32_t usec);

private:
    class SchedulerThread : public PeriodicThread {
    public:
        SchedulerThread(Thread::task_t t, Scheduler &sched)
            : PeriodicThread(t)
            , _sched(sched)
        { }

    protected:
        bool _run() override;

        Scheduler &_sched;
    };

    void _wait_all_threads();

    void     _debug_stack();

    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;

    AP_HAL::Proc _failsafe;

    bool _initialized;
    pthread_barrier_t _initialized_barrier;

    AP_HAL::MemberProc _timer_proc[LINUX_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    volatile bool _in_timer_proc;
    uint8_t _timeslices_count;

    struct timesliced_proc {
        AP_HAL::MemberProc proc;
        uint8_t timeslot;
        uint8_t freq_div;
    };
    timesliced_proc _timesliced_proc[LINUX_SCHEDULER_MAX_TIMESLICED_PROCS];
    uint8_t _num_timesliced_procs;
    uint8_t _max_freq_div;

    AP_HAL::MemberProc _io_proc[LINUX_SCHEDULER_MAX_IO_PROCS];
    uint8_t _num_io_procs;

    SchedulerThread _timer_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_timer_task, void), *this};
    SchedulerThread _io_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_io_task, void), *this};
    SchedulerThread _rcin_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_rcin_task, void), *this};
    SchedulerThread _uart_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_uart_task, void), *this};
    SchedulerThread _tonealarm_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_tonealarm_task, void), *this};

    void _timer_task();
    void _io_task();
    void _rcin_task();
    void _uart_task();
    void _tonealarm_task();

    void _run_io();
    void _run_uarts();
    bool _register_timesliced_proc(AP_HAL::MemberProc, uint8_t);

    uint64_t _stopped_clock_usec;
    uint64_t _last_stack_debug_msec;

    Semaphore _timer_semaphore;
    Semaphore _io_semaphore;
};

}
