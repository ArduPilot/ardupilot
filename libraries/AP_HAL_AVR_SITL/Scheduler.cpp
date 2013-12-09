/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include "AP_HAL_AVR_SITL.h"
#include "Scheduler.h"
#include <sys/time.h>
#include <unistd.h>

#ifdef __CYGWIN__
#include <stdio.h> 
#include <stdlib.h> 
#include <assert.h> 
#include <windows.h> 
#include <string.h> 
#include <memory.h> 
#include <process.h> 
#include <time.h>
#endif

using namespace AVR_SITL;

extern const AP_HAL::HAL& hal;


AP_HAL::Proc SITLScheduler::_failsafe = NULL;
volatile bool SITLScheduler::_timer_suspended = false;
volatile bool SITLScheduler::_timer_event_missed = false;

AP_HAL::MemberProc SITLScheduler::_timer_proc[SITL_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
uint8_t SITLScheduler::_num_timer_procs = 0;
bool SITLScheduler::_in_timer_proc = false;

AP_HAL::MemberProc SITLScheduler::_io_proc[SITL_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
uint8_t SITLScheduler::_num_io_procs = 0;
bool SITLScheduler::_in_io_proc = false;

struct timeval SITLScheduler::_sketch_start_time;

#ifdef __CYGWIN__
double SITLScheduler::_cyg_freq = 0;
long SITLScheduler::_cyg_start = 0;
#endif

SITLScheduler::SITLScheduler()
{}

void SITLScheduler::init(void *unused) 
{
	gettimeofday(&_sketch_start_time,NULL);

#ifdef __CYGWIN__
	LARGE_INTEGER lFreq, lCnt;
	QueryPerformanceFrequency(&lFreq);
	_cyg_freq = (double)lFreq.LowPart;
	QueryPerformanceCounter(&lCnt);
	_cyg_start = lCnt.LowPart;
#endif

}
#ifdef __CYGWIN__
double SITLScheduler::_cyg_sec()
{
	LARGE_INTEGER lCnt;
	long tcnt;
	QueryPerformanceCounter(&lCnt);
 	tcnt = lCnt.LowPart - _cyg_start;
 	return ((double)tcnt) / _cyg_freq;
}
#endif

uint32_t SITLScheduler::_micros() 
{
#ifdef __CYGWIN__
	return (uint32_t)(_cyg_sec() * 1.0e6);
#else   
	struct timeval tp;
	gettimeofday(&tp,NULL);
	return 1.0e6*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
		      (_sketch_start_time.tv_sec +
		       (_sketch_start_time.tv_usec*1.0e-6)));
#endif
}

uint32_t SITLScheduler::micros() 
{
    return _micros();
}

uint32_t SITLScheduler::millis() 
{
#ifdef __CYGWIN__
	// 1000 ms in a second
	return (uint32_t)(_cyg_sec() * 1000);
#else
	struct timeval tp;
	gettimeofday(&tp,NULL);
	return 1.0e3*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
		      (_sketch_start_time.tv_sec +
		       (_sketch_start_time.tv_usec*1.0e-6)));
#endif
}

void SITLScheduler::delay_microseconds(uint16_t usec) 
{
	uint32_t start = micros();
	while (micros() - start < usec) {
		usleep(usec - (micros() - start));
	}
}

void SITLScheduler::delay(uint16_t ms)
{
	uint32_t start = micros();
    
    while (ms > 0) {
        while ((micros() - start) >= 1000) {
            ms--;
            if (ms == 0) break;
            start += 1000;
        }
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
}

void SITLScheduler::register_delay_callback(AP_HAL::Proc proc,
                                            uint16_t min_time_ms) 
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void SITLScheduler::register_timer_process(AP_HAL::MemberProc proc) 
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < SITL_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    }

}

void SITLScheduler::register_io_process(AP_HAL::MemberProc proc) 
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < SITL_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    }

}

void SITLScheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us) 
{
    _failsafe = failsafe;
}

void SITLScheduler::suspend_timer_procs() {
    _timer_suspended = true;
}

void SITLScheduler::resume_timer_procs() {
    _timer_suspended = false;
    if (_timer_event_missed) {
        _timer_event_missed = false;
        _run_timer_procs(false);
    }
}

bool SITLScheduler::in_timerprocess() {
    return _in_timer_proc || _in_io_proc;
}

bool SITLScheduler::system_initializing() {
    return !_initialized;
}

void SITLScheduler::system_initialized() {
    if (_initialized) {
        panic(
            PSTR("PANIC: scheduler system initialized called more than once"));
    }
    _initialized = true;
}

void SITLScheduler::sitl_end_atomic() {
    if (_nested_atomic_ctr == 0) 
        hal.uartA->println_P(PSTR("NESTED ATOMIC ERROR"));
    else
        _nested_atomic_ctr--;
}   

void SITLScheduler::reboot(bool hold_in_bootloader) 
{
    hal.uartA->println_P(PSTR("REBOOT NOT IMPLEMENTED\r\n"));
}

void SITLScheduler::_run_timer_procs(bool called_from_isr) 
{
    if (_in_timer_proc) {
        // the timer calls took longer than the period of the
        // timer. This is bad, and may indicate a serious
        // driver failure. We can't just call the drivers
        // again, as we could run out of stack. So we only
        // call the _failsafe call. It's job is to detect if
        // the drivers or the main loop are indeed dead and to
        // activate whatever failsafe it thinks may help if
        // need be.  We assume the failsafe code can't
        // block. If it does then we will recurse and die when
        // we run out of stack
        if (_failsafe != NULL) {
            _failsafe();
        }
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
    } else if (called_from_isr) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        //_failsafe(NULL);
    }

    _in_timer_proc = false;
}

void SITLScheduler::_run_io_procs(bool called_from_isr) 
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
    } else if (called_from_isr) {
        _timer_event_missed = true;
    }

    _in_io_proc = false;
}

void SITLScheduler::panic(const prog_char_t *errormsg) {
    hal.console->println_P(errormsg);
    for(;;);
}

#endif
