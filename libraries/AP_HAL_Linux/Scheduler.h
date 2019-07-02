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

    void     init() override;
    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;

    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;

    bool     in_main_thread() const override;

    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;

    void     system_initialized() override;

    void     reboot(bool hold_in_bootloader) override;

    void     stop_clock(uint64_t time_usec) override;

    uint64_t stopped_clock_usec() const { return _stopped_clock_usec; }

    void microsleep(uint32_t usec);

    void teardown();

    /*
      create a new thread
     */
    bool thread_create(AP_HAL::MemberProc, const char *name, uint32_t stack_size, priority_base base, int8_t priority) override;
    
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

    void     init_realtime();

    void _wait_all_threads();

    void     _debug_stack();

    AP_HAL::Proc _failsafe;

    bool _initialized;
    pthread_barrier_t _initialized_barrier;

    AP_HAL::MemberProc _timer_proc[LINUX_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    volatile bool _in_timer_proc;

    AP_HAL::MemberProc _io_proc[LINUX_SCHEDULER_MAX_IO_PROCS];
    uint8_t _num_io_procs;

    SchedulerThread _timer_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_timer_task, void), *this};
    SchedulerThread _io_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_io_task, void), *this};
    SchedulerThread _rcin_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_rcin_task, void), *this};
    SchedulerThread _uart_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_uart_task, void), *this};

    void _timer_task();
    void _io_task();
    void _rcin_task();
    void _uart_task();

    void _run_io();
    void _run_uarts();

    uint64_t _stopped_clock_usec;
    uint64_t _last_stack_debug_msec;
    pthread_t _main_ctx;

    Semaphore _io_semaphore;
};

}
