/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "AP_HAL_PX4.h"
#include "Scheduler.h"

#include <unistd.h>
#include <stdlib.h>
#include <sched.h>
#include <errno.h>
#include <stdio.h>
#include <drivers/drv_hrt.h>
#include <nuttx/arch.h>
#include <systemlib/systemlib.h>
#include <pthread.h>
#include <poll.h>

#include "UARTDriver.h"
#include "AnalogIn.h"
#include "Storage.h"
#include "RCOutput.h"
#include "RCInput.h"

using namespace PX4;

extern const AP_HAL::HAL& hal;

extern bool _px4_thread_should_exit;

PX4Scheduler::PX4Scheduler() :
    _perf_timers(perf_alloc(PC_ELAPSED, "APM_timers")),
    _perf_io_timers(perf_alloc(PC_ELAPSED, "APM_IO_timers")),
	_perf_delay(perf_alloc(PC_ELAPSED, "APM_delay"))
{}

void PX4Scheduler::init(void *unused) 
{
    _sketch_start_time = hrt_absolute_time();

    _main_task_pid = getpid();

    // setup the timer thread - this will call tasks at 1kHz
	pthread_attr_t thread_attr;
	struct sched_param param;

	pthread_attr_init(&thread_attr);
	pthread_attr_setstacksize(&thread_attr, 2048);

	param.sched_priority = APM_TIMER_PRIORITY;
	(void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

	pthread_create(&_timer_thread_ctx, &thread_attr, (pthread_startroutine_t)&PX4::PX4Scheduler::_timer_thread, this);

    // the UART thread runs at a medium priority
	pthread_attr_init(&thread_attr);
	pthread_attr_setstacksize(&thread_attr, 2048);

	param.sched_priority = APM_UART_PRIORITY;
	(void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

	pthread_create(&_uart_thread_ctx, &thread_attr, (pthread_startroutine_t)&PX4::PX4Scheduler::_uart_thread, this);

    // the IO thread runs at lower priority
	pthread_attr_init(&thread_attr);
	pthread_attr_setstacksize(&thread_attr, 2048);

	param.sched_priority = APM_IO_PRIORITY;
	(void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

	pthread_create(&_io_thread_ctx, &thread_attr, (pthread_startroutine_t)&PX4::PX4Scheduler::_io_thread, this);
}

uint32_t PX4Scheduler::micros() 
{
    return (uint32_t)(hrt_absolute_time() - _sketch_start_time);
}

uint32_t PX4Scheduler::millis() 
{
    return hrt_absolute_time() / 1000;
}

/**
   delay for a specified number of microseconds using a semaphore wait
 */
void PX4Scheduler::delay_microseconds_semaphore(uint16_t usec) 
{
    sem_t wait_semaphore;
    struct hrt_call wait_call;
    sem_init(&wait_semaphore, 0, 0);
    hrt_call_after(&wait_call, usec, (hrt_callout)sem_post, &wait_semaphore);
    sem_wait(&wait_semaphore);
}

void PX4Scheduler::delay_microseconds(uint16_t usec) 
{
    perf_begin(_perf_delay);
    if (usec >= 500) {
        delay_microseconds_semaphore(usec);
        perf_end(_perf_delay);
        return;
    }
	uint32_t start = micros();
    uint32_t dt;
	while ((dt=(micros() - start)) < usec) {
		up_udelay(usec - dt);
	}
    perf_end(_perf_delay);
}

void PX4Scheduler::delay(uint16_t ms)
{
    if (in_timerprocess()) {
        ::printf("ERROR: delay() from timer process\n");
        return;
    }
    perf_begin(_perf_delay);
	uint64_t start = hrt_absolute_time();
    
    while ((hrt_absolute_time() - start)/1000 < ms && 
           !_px4_thread_should_exit) {
        delay_microseconds_semaphore(1000);
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
    perf_end(_perf_delay);
    if (_px4_thread_should_exit) {
        exit(1);
    }
}

void PX4Scheduler::register_delay_callback(AP_HAL::Proc proc,
                                            uint16_t min_time_ms) 
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void PX4Scheduler::register_timer_process(AP_HAL::MemberProc proc) 
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < PX4_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

void PX4Scheduler::register_io_process(AP_HAL::MemberProc proc) 
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < PX4_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}

void PX4Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us) 
{
    _failsafe = failsafe;
}

void PX4Scheduler::suspend_timer_procs() 
{
    _timer_suspended = true;
}

void PX4Scheduler::resume_timer_procs() 
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timers(false);
        _timer_event_missed = false;
    }
}

void PX4Scheduler::reboot(bool hold_in_bootloader) 
{
	systemreset(hold_in_bootloader);
}

void PX4Scheduler::_run_timers(bool called_from_timer_thread)
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i] != NULL) {
                _timer_proc[i]();
            }
        }
    } else if (called_from_timer_thread) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe();
    }

    // process analog input
    ((PX4AnalogIn *)hal.analogin)->_timer_tick();

    _in_timer_proc = false;
}

void *PX4Scheduler::_timer_thread(void)
{
    while (!_hal_initialized) {
        poll(NULL, 0, 1);        
    }
    while (!_px4_thread_should_exit) {
        delay_microseconds_semaphore(1000);

        // run registered timers
        perf_begin(_perf_timers);
        _run_timers(true);
        perf_end(_perf_timers);

        // process any pending RC output requests
        ((PX4RCOutput *)hal.rcout)->_timer_tick();

        // process any pending RC input requests
        ((PX4RCInput *)hal.rcin)->_timer_tick();
    }
    return NULL;
}

void PX4Scheduler::_run_io(void)
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    if (!_timer_suspended) {
        // now call the IO based drivers
        for (int i = 0; i < _num_io_procs; i++) {
            if (_io_proc[i] != NULL) {
                _io_proc[i]();
            }
        }
    }

    _in_io_proc = false;
}

void *PX4Scheduler::_uart_thread(void)
{
    while (!_hal_initialized) {
        poll(NULL, 0, 1);        
    }
    while (!_px4_thread_should_exit) {
        delay_microseconds_semaphore(1000);

        // process any pending serial bytes
        ((PX4UARTDriver *)hal.uartA)->_timer_tick();
        ((PX4UARTDriver *)hal.uartB)->_timer_tick();
        ((PX4UARTDriver *)hal.uartC)->_timer_tick();
        ((PX4UARTDriver *)hal.uartD)->_timer_tick();
        ((PX4UARTDriver *)hal.uartE)->_timer_tick();
    }
    return NULL;
}

void *PX4Scheduler::_io_thread(void)
{
    while (!_hal_initialized) {
        poll(NULL, 0, 1);        
    }
    while (!_px4_thread_should_exit) {
        poll(NULL, 0, 1);

        // process any pending storage writes
        ((PX4Storage *)hal.storage)->_timer_tick();

        // run registered IO processes
        perf_begin(_perf_io_timers);
        _run_io();
        perf_end(_perf_io_timers);
    }
    return NULL;
}

void PX4Scheduler::panic(const prog_char_t *errormsg) 
{
    write(1, errormsg, strlen(errormsg));
    write(1, "\n", 1);
    hal.scheduler->delay_microseconds(10000);
    _px4_thread_should_exit = true;
    exit(1);
}

bool PX4Scheduler::in_timerprocess() 
{
    return getpid() != _main_task_pid;
}

bool PX4Scheduler::system_initializing() {
    return !_initialized;
}

void PX4Scheduler::system_initialized() {
    if (_initialized) {
        panic(PSTR("PANIC: scheduler::system_initialized called"
                   "more than once"));
    }
    _initialized = true;
}

#endif
