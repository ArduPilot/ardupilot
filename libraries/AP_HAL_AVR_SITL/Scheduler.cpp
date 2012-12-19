/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include "AP_HAL_AVR_SITL.h"
#include "Scheduler.h"
#include <sys/time.h>
#include <unistd.h>

using namespace AVR_SITL;

extern const AP_HAL::HAL& hal;


AP_HAL::TimedProc SITLScheduler::_failsafe = NULL;
volatile bool SITLScheduler::_timer_suspended = false;
AP_HAL::TimedProc SITLScheduler::_timer_proc[SITL_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
AP_HAL::TimedProc SITLScheduler::_defered_timer_proc = NULL;
uint8_t SITLScheduler::_num_timer_procs = 0;
bool SITLScheduler::_in_timer_proc = false;
struct timeval SITLScheduler::_sketch_start_time;

SITLScheduler::SITLScheduler()
{}

void SITLScheduler::init(void *unused) 
{
	gettimeofday(&_sketch_start_time,NULL);
}

uint32_t SITLScheduler::_micros() 
{
	struct timeval tp;
	gettimeofday(&tp,NULL);
	return 1.0e6*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
		      (_sketch_start_time.tv_sec +
		       (_sketch_start_time.tv_usec*1.0e-6)));
}

uint32_t SITLScheduler::micros() 
{
    return _micros();
}

uint32_t SITLScheduler::millis() 
{
	struct timeval tp;
	gettimeofday(&tp,NULL);
	return 1.0e3*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
		      (_sketch_start_time.tv_sec +
		       (_sketch_start_time.tv_usec*1.0e-6)));
}

void SITLScheduler::delay_microseconds(uint16_t usec) 
{
	uint32_t start = micros();
	while (micros() - start < usec) {
		usleep(usec - (micros() - start));
	}
}

void SITLScheduler::delay(uint32_t ms)
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

void SITLScheduler::register_timer_process(AP_HAL::TimedProc proc) 
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

bool SITLScheduler::defer_timer_process(AP_HAL::TimedProc proc) 
{
    if ( _in_timer_proc || _timer_suspended ) {
        _defered_timer_proc = proc;
        return false;
    } else {
        _timer_suspended = true;
        proc(micros());
        _timer_suspended = false;
        return true;
    }
}

void SITLScheduler::register_timer_failsafe(AP_HAL::TimedProc failsafe, uint32_t period_us) 
{
    _failsafe = failsafe;
}

void SITLScheduler::suspend_timer_procs() {
    _timer_suspended = true;
}

void SITLScheduler::resume_timer_procs() {
    _timer_suspended = false;
}

void SITLScheduler::begin_atomic() {
    _nested_atomic_ctr++;
}

void SITLScheduler::end_atomic() {
	if (_nested_atomic_ctr == 0) {
        hal.uartA->println_P(PSTR("ATOMIC NESTING ERROR"));
        return;
    }
    _nested_atomic_ctr--;
}

void SITLScheduler::reboot() 
{
    hal.uartA->println_P(PSTR("REBOOT NOT IMPLEMENTED\r\n"));
}

void SITLScheduler::timer_event() 
{
    uint32_t tnow = _micros();
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
            _failsafe(tnow);
        }
        return;
    }
    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i] != NULL) {
                _timer_proc[i](tnow);
            }
        }
    }

    /* Run any defered procedures, if they exist.*/
    /* Atomic read and clear: */
    AP_HAL::TimedProc defered = _defered_timer_proc;
    _defered_timer_proc = NULL;
    if (defered != NULL) {
        _timer_suspended = true;
        defered(tnow);
        _timer_suspended = false;
    }

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe(tnow);
    }

    _in_timer_proc = false;
}

void SITLScheduler::panic(const prog_char_t *errormsg) {
    hal.console->println_P(errormsg);
    for(;;);
}

#endif
