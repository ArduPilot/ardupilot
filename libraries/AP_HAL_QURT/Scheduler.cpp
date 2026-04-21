#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_QURT.h"
#include "Scheduler.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <sched.h>
#include <errno.h>
#include <stdio.h>
#include <pthread.h>

#include "UARTDriver.h"
#include "Storage.h"
#include "RCOutput.h"
#include "RCInput.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include "Thread.h"

using namespace QURT;

extern const AP_HAL::HAL& hal;

Scheduler::Scheduler()
{
}

void Scheduler::init()
{
    _main_thread_ctx = pthread_self();

    // setup the timer thread - this will call tasks at 1kHz
    pthread_attr_t thread_attr;
    struct sched_param param;

    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 16000);

    param.sched_priority = APM_TIMER_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);

    pthread_create(&_timer_thread_ctx, &thread_attr, &Scheduler::_timer_thread, this);

    // the UART thread runs at a medium priority
    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 16000);

    param.sched_priority = APM_UART_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);

    pthread_create(&_uart_thread_ctx, &thread_attr, &Scheduler::_uart_thread, this);

    // the IO thread runs at lower priority
    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 16000);

    param.sched_priority = APM_IO_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);

    pthread_create(&_io_thread_ctx, &thread_attr, &Scheduler::_io_thread, this);
}

#define APM_QURT_MAX_PRIORITY          (200 + 20)
#define APM_QURT_TIMER_PRIORITY        (200 + 15)
#define APM_QURT_UART_PRIORITY         (200 + 14)
#define APM_QURT_NET_PRIORITY          (200 + 14)
#define APM_QURT_RCIN_PRIORITY         (200 + 13)
#define APM_QURT_MAIN_PRIORITY         (200 + 12)
#define APM_QURT_IO_PRIORITY           (200 + 10)
#define APM_QURT_SCRIPTING_PRIORITY    (200 + 1)
#define AP_QURT_SENSORS_SCHED_PRIO     (200 + 12)

uint8_t Scheduler::calculate_thread_priority(priority_base base, int8_t priority) const
{
    uint8_t thread_priority = APM_QURT_IO_PRIORITY;
    static const struct {
        priority_base base;
        uint8_t p;
    } priority_map[] = {
        { PRIORITY_BOOST, APM_QURT_MAIN_PRIORITY},
        { PRIORITY_MAIN, APM_QURT_MAIN_PRIORITY},
        { PRIORITY_SPI, AP_QURT_SENSORS_SCHED_PRIO+1},
        { PRIORITY_I2C, AP_QURT_SENSORS_SCHED_PRIO},
        { PRIORITY_CAN, APM_QURT_TIMER_PRIORITY},
        { PRIORITY_TIMER, APM_QURT_TIMER_PRIORITY},
        { PRIORITY_RCIN, APM_QURT_RCIN_PRIORITY},
        { PRIORITY_IO, APM_QURT_IO_PRIORITY},
        { PRIORITY_UART, APM_QURT_UART_PRIORITY},
        { PRIORITY_STORAGE, APM_QURT_IO_PRIORITY},
        { PRIORITY_SCRIPTING, APM_QURT_SCRIPTING_PRIORITY},
        { PRIORITY_NET, APM_QURT_NET_PRIORITY},
    };
    for (const auto &m : priority_map) {
        if (m.base == base) {
            thread_priority = constrain_int16(m.p + priority, 1, APM_QURT_MAX_PRIORITY);
            break;
        }
    }

    return thread_priority;
}

/*
  create a new thread
*/
bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name, uint32_t stack_size, priority_base base, int8_t priority)
{
    Thread *thread = new Thread{(Thread::task_t)proc};
    if (thread == nullptr) {
        return false;
    }

    const uint8_t thread_priority = calculate_thread_priority(base, priority);

    stack_size = MAX(stack_size, 8192U);

    // Setting the stack size too large can cause odd behavior!!!
    thread->set_stack_size(stack_size);

    /*
     * We should probably store the thread handlers and join() when exiting,
     * but let's the thread manage itself for now.
     */
    thread->set_auto_free(true);

    DEV_PRINTF("Starting thread %s: Priority %u", name, thread_priority);

    if (!thread->start(name, SCHED_FIFO, thread_priority)) {
        delete thread;
        return false;
    }

    return true;
}

void Scheduler::delay_microseconds(uint16_t usec)
{
    qurt_timer_sleep(usec);
}

void Scheduler::delay(uint16_t ms)
{
    uint64_t start = AP_HAL::micros64();

    while ((AP_HAL::micros64() - start)/1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            if (in_main_thread()) {
                const auto old_task = hal.util->persistent_data.scheduler_task;
                hal.util->persistent_data.scheduler_task = -4;
                call_delay_cb();
                hal.util->persistent_data.scheduler_task = old_task;
            }
        }
    }
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < QURT_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < QURT_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::suspend_timer_procs()
{
    _timer_suspended = true;
}

void Scheduler::resume_timer_procs()
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timers(false);
        _timer_event_missed = false;
    }
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    HAP_PRINTF("**** REBOOT REQUESTED ****");
    // delay for printf to appear on USB monitor
    qurt_timer_sleep(10000);

    // tell host we want to reboot
    struct qurt_rpc_msg msg {};
    msg.msg_id = QURT_MSG_ID_REBOOT;
    qurt_rpc_send(msg);

    // wait for RPC to get through
    qurt_timer_sleep(10000);
    exit(1);
}

void Scheduler::_run_timers(bool called_from_timer_thread)
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i]) {
                _timer_proc[i]();
            }
        }
    } else if (called_from_timer_thread) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

extern bool qurt_ran_overtime;

void *Scheduler::_timer_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;

    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);

        // run registered timers
        sched->_run_timers(true);
    }
    return nullptr;
}

void Scheduler::_run_io(void)
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    if (!_timer_suspended) {
        // now call the IO based drivers
        for (int i = 0; i < _num_io_procs; i++) {
            if (_io_proc[i]) {
                _io_proc[i]();
            }
        }
    }

    _in_io_proc = false;
}

void *Scheduler::_uart_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;

    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(200);

        // process any pending serial bytes
        for (uint8_t i = 0; i < hal.num_serial; i++) {
            auto *p = hal.serial(i);
            if (p != nullptr) {
                p->_timer_tick();
            }
        }
    }
    return nullptr;
}

void *Scheduler::_io_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;

    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);

        // run registered IO processes
        sched->_run_io();

        // update storage
        hal.storage->_timer_tick();

        // update RC input
        ((QURT::RCInput *)hal.rcin)->_timer_tick();
    }
    return nullptr;
}

bool Scheduler::in_main_thread() const
{
    return pthread_equal(pthread_self(), _main_thread_ctx);
}

void Scheduler::set_system_initialized()
{
    _main_thread_ctx = pthread_self();
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called"
                      "more than once");
    }
    _initialized = true;
}

void Scheduler::hal_initialized(void)
{
    HAP_PRINTF("HAL is initialised");
    _hal_initialized = true;
}
