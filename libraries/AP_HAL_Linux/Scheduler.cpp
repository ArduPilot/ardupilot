#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "Scheduler.h"
#include "Storage.h"
#include "UARTDriver.h"
#include <unistd.h>
#include <sys/time.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/mman.h>

using namespace Linux;

extern const AP_HAL::HAL& hal;

#define APM_LINUX_TIMER_PRIORITY    13
#define APM_LINUX_UART_PRIORITY     12
#define APM_LINUX_MAIN_PRIORITY     11
#define APM_LINUX_IO_PRIORITY       10

LinuxScheduler::LinuxScheduler()
{}

typedef void *(*pthread_startroutine_t)(void *);

/*
  setup for realtime. Lock all of memory in the thread and pre-fault
  the given stack size, so stack faults don't cause timing jitter
 */
void LinuxScheduler::_setup_realtime(uint32_t size) 
{
        uint8_t dummy[size];
        mlockall(MCL_CURRENT|MCL_FUTURE);
        memset(dummy, 0, sizeof(dummy));
}

void LinuxScheduler::init(void* machtnichts)
{
    clock_gettime(CLOCK_MONOTONIC, &_sketch_start_time);

    _setup_realtime(32768);

    pthread_attr_t thread_attr;
    struct sched_param param;

    memset(&param, 0, sizeof(param));

    param.sched_priority = APM_LINUX_MAIN_PRIORITY;
    sched_setscheduler(0, SCHED_FIFO, &param);

    param.sched_priority = APM_LINUX_TIMER_PRIORITY;
    pthread_attr_init(&thread_attr);
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_timer_thread_ctx, &thread_attr, (pthread_startroutine_t)&Linux::LinuxScheduler::_timer_thread, this);

    // the UART thread runs at a medium priority
    pthread_attr_init(&thread_attr);
    param.sched_priority = APM_LINUX_UART_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_uart_thread_ctx, &thread_attr, (pthread_startroutine_t)&Linux::LinuxScheduler::_uart_thread, this);
    
    // the IO thread runs at lower priority
    pthread_attr_init(&thread_attr);
    param.sched_priority = APM_LINUX_IO_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);
    
    pthread_create(&_io_thread_ctx, &thread_attr, (pthread_startroutine_t)&Linux::LinuxScheduler::_io_thread, this);
}

void LinuxScheduler::_microsleep(uint32_t usec)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}

void LinuxScheduler::delay(uint16_t ms)
{
    if (stopped_clock_usec) {
        stopped_clock_usec += 1000UL*ms;
        return;
    }
    uint32_t start = millis();
    
    while ((millis() - start) < ms) {
        // this yields the CPU to other apps
        _microsleep(1000);
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
}

uint32_t LinuxScheduler::millis() 
{
    if (stopped_clock_usec) {
        return stopped_clock_usec/1000;
    }
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - 
                  (_sketch_start_time.tv_sec +
                   (_sketch_start_time.tv_nsec*1.0e-9)));
}

uint32_t LinuxScheduler::micros() 
{
    if (stopped_clock_usec) {
        return stopped_clock_usec;
    }
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - 
                  (_sketch_start_time.tv_sec +
                   (_sketch_start_time.tv_nsec*1.0e-9)));
}

void LinuxScheduler::delay_microseconds(uint16_t us)
{
    _microsleep(us);
}

void LinuxScheduler::register_delay_callback(AP_HAL::Proc proc,
                                             uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void LinuxScheduler::register_timer_process(AP_HAL::MemberProc proc) 
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < LINUX_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

void LinuxScheduler::register_io_process(AP_HAL::MemberProc proc) 
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < LINUX_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}

void LinuxScheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void LinuxScheduler::suspend_timer_procs()
{
    _timer_suspended = true;
    while (_in_timer_proc) {
        usleep(1);
    }
}

void LinuxScheduler::resume_timer_procs()
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timers(false);
        _timer_event_missed = false;
    }
}

void LinuxScheduler::_run_timers(bool called_from_timer_thread)
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

    _in_timer_proc = false;
}

void *LinuxScheduler::_timer_thread(void)
{
    _setup_realtime(32768);
    while (system_initializing()) {
        poll(NULL, 0, 1);        
    }
    while (true) {
        _microsleep(5000);

        // run registered timers
        _run_timers(true);

    }
    return NULL;
}

void LinuxScheduler::_run_io(void)
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

void *LinuxScheduler::_uart_thread(void)
{
    _setup_realtime(32768);
    while (system_initializing()) {
        poll(NULL, 0, 1);        
    }
    while (true) {
        _microsleep(10000);

        // process any pending serial bytes
        ((LinuxUARTDriver *)hal.uartA)->_timer_tick();
        ((LinuxUARTDriver *)hal.uartB)->_timer_tick();
        ((LinuxUARTDriver *)hal.uartC)->_timer_tick();
    }
    return NULL;
}

void *LinuxScheduler::_io_thread(void)
{
    _setup_realtime(32768);
    while (system_initializing()) {
        poll(NULL, 0, 1);        
    }
    while (true) {
        _microsleep(20000);

        // process any pending storage writes
        ((LinuxStorage *)hal.storage)->_timer_tick();

        // run registered IO processes
        _run_io();
    }
    return NULL;
}

void LinuxScheduler::panic(const prog_char_t *errormsg) 
{
    write(1, errormsg, strlen(errormsg));
    write(1, "\n", 1);
    hal.scheduler->delay_microseconds(10000);
    exit(1);
}

bool LinuxScheduler::in_timerprocess() 
{
    return _in_timer_proc;
}

void LinuxScheduler::begin_atomic()
{}

void LinuxScheduler::end_atomic()
{}

bool LinuxScheduler::system_initializing() {
    return !_initialized;
}

void LinuxScheduler::system_initialized()
{
    if (_initialized) {
        panic("PANIC: scheduler::system_initialized called more than once");
    }
    _initialized = true;
}

void LinuxScheduler::reboot(bool hold_in_bootloader) 
{
    for(;;);
}

void LinuxScheduler::stop_clock(uint64_t time_usec)
{
    stopped_clock_usec = time_usec;
}

#endif // CONFIG_HAL_BOARD
