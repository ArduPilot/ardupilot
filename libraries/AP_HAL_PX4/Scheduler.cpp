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
#include <poll.h>

using namespace PX4;

extern const AP_HAL::HAL& hal;

uint64_t PX4Scheduler::_sketch_start_time;

AP_HAL::TimedProc PX4Scheduler::_failsafe = NULL;
volatile bool PX4Scheduler::_timer_suspended = false;
AP_HAL::TimedProc PX4Scheduler::_timer_proc[PX4_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
uint8_t PX4Scheduler::_num_timer_procs = 0;
bool PX4Scheduler::_in_timer_proc = false;
uint8_t PX4Scheduler::_nested_atomic_ctr;
bool PX4Scheduler::_timer_pending;

PX4Scheduler::PX4Scheduler()
{}

void PX4Scheduler::init(void *unused) 
{
    _sketch_start_time = hrt_absolute_time();

    // setup a 1kHz timer
	memset(&_call, 0, sizeof(_call));
	hrt_call_every(&_call, 1000, 1000, _timer_event, NULL);
}

uint32_t PX4Scheduler::_micros() 
{
    return (uint32_t)(hrt_absolute_time() - _sketch_start_time);
}

uint32_t PX4Scheduler::micros() 
{
    return _micros();
}

uint32_t PX4Scheduler::millis() 
{
    return hrt_absolute_time() / 1000;
}

void PX4Scheduler::delay_microseconds(uint16_t usec) 
{
	uint32_t start = micros();
	while (micros() - start < usec) {
		up_udelay(usec - (micros() - start));
	}
}

void PX4Scheduler::delay(uint16_t ms)
{
	uint64_t start = hrt_absolute_time();
    
    while ((hrt_absolute_time() - start)/1000 < ms) {
        // this yields the CPU to other apps
        poll(NULL, 0, 1);
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
}

void PX4Scheduler::register_delay_callback(AP_HAL::Proc proc,
                                            uint16_t min_time_ms) 
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void PX4Scheduler::register_timer_process(AP_HAL::TimedProc proc) 
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < PX4_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    }

}

void PX4Scheduler::register_timer_failsafe(AP_HAL::TimedProc failsafe, uint32_t period_us) 
{
    hal.console->printf("Not registering failsafe handler\n");
//    _failsafe = failsafe;
}

void PX4Scheduler::suspend_timer_procs() {
    _timer_suspended = true;
}

void PX4Scheduler::resume_timer_procs() {
    _timer_suspended = false;
}

void PX4Scheduler::begin_atomic() {
    _nested_atomic_ctr++;
}

void PX4Scheduler::end_atomic() {
	if (_nested_atomic_ctr == 0) {
        hal.uartA->println_P(PSTR("ATOMIC NESTING ERROR"));
        return;
    }
    _nested_atomic_ctr--;
    if (_nested_atomic_ctr == 0 && _timer_pending) {
        // a timer went off during an atomic operation - run it now
        _timer_pending = false;
        _timer_event(NULL);
    }
}

void PX4Scheduler::reboot() 
{
	up_systemreset();
}

void PX4Scheduler::_timer_event(void *arg)
{
    if (_nested_atomic_ctr != 0) {
        _timer_pending = true;
        return;
    }

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

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe(tnow);
    }

    _in_timer_proc = false;
}

void PX4Scheduler::panic(const prog_char_t *errormsg) {
    write(1, errormsg, strlen(errormsg));
    hal.scheduler->delay_microseconds(10000);
    exit(1);
}

#endif
