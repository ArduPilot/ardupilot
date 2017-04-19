#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QURT

#include "AP_HAL_QURT.h"
#include "Scheduler.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <sched.h>
#include <errno.h>
#include <stdio.h>
#include <dspal/include/pthread.h>
#include <dspal_types.h>

#include "UARTDriver.h"
#include "Storage.h"
#include "RCOutput.h"
#include <AP_Scheduler/AP_Scheduler.h>

using namespace QURT;

extern const AP_HAL::HAL& hal;

Scheduler::Scheduler()
{
}

void Scheduler::init()
{
    _main_task_pid = getpid();

    // setup the timer thread - this will call tasks at 1kHz
	pthread_attr_t thread_attr;
	struct sched_param param;

	pthread_attr_init(&thread_attr);
	pthread_attr_setstacksize(&thread_attr, 40960);

	param.sched_priority = APM_TIMER_PRIORITY;
	(void)pthread_attr_setschedparam(&thread_attr, &param);

	pthread_create(&_timer_thread_ctx, &thread_attr, &Scheduler::_timer_thread, this);

    // the UART thread runs at a medium priority
	pthread_attr_init(&thread_attr);
	pthread_attr_setstacksize(&thread_attr, 40960);

	param.sched_priority = APM_UART_PRIORITY;
	(void)pthread_attr_setschedparam(&thread_attr, &param);

	pthread_create(&_uart_thread_ctx, &thread_attr, &Scheduler::_uart_thread, this);

    // the IO thread runs at lower priority
	pthread_attr_init(&thread_attr);
	pthread_attr_setstacksize(&thread_attr, 40960);

	param.sched_priority = APM_IO_PRIORITY;
	(void)pthread_attr_setschedparam(&thread_attr, &param);

	pthread_create(&_io_thread_ctx, &thread_attr, &Scheduler::_io_thread, this);
}

void Scheduler::delay_microseconds(uint16_t usec) 
{
    //pthread_yield();
    usleep(usec);
}

void Scheduler::delay(uint16_t ms)
{
    if (in_timerprocess()) {
        ::printf("ERROR: delay() from timer process\n");
        return;
    }
	uint64_t start = AP_HAL::micros64();
    uint64_t now;
    
    while (((now=AP_HAL::micros64()) - start)/1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
}

void Scheduler::register_delay_callback(AP_HAL::Proc proc,
                                            uint16_t min_time_ms) 
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
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
    usleep(2000000);
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
    uint32_t last_ran_overtime = 0;

    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);

        // run registered timers
        sched->_run_timers(true);

        // process any pending RC output requests
        ((RCOutput *)hal.rcout)->timer_update();

        if (qurt_ran_overtime && AP_HAL::millis() - last_ran_overtime > 2000) {
            last_ran_overtime = AP_HAL::millis();
            printf("Overtime in task %d\n", (int)AP_Scheduler::current_task);
            hal.console->printf("Overtime in task %d\n", (int)AP_Scheduler::current_task);
        }
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
        sched->delay_microseconds(1000);

        // process any pending serial bytes
        //((UARTDriver *)hal.uartA)->timer_tick();
        ((UARTDriver *)hal.uartB)->timer_tick();
        ((UARTDriver *)hal.uartC)->timer_tick();
        ((UARTDriver *)hal.uartD)->timer_tick();
        ((UARTDriver *)hal.uartE)->timer_tick();
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
    }
    return nullptr;
}

bool Scheduler::in_timerprocess() 
{
    return getpid() != _main_task_pid;
}

void Scheduler::system_initialized() {
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
#endif
