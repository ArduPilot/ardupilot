/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "Scheduler.h"
#include "UARTDriver.h"
#include <sys/time.h>
#include <unistd.h>
#include <fenv.h>

using namespace HALSITL;

extern const AP_HAL::HAL& hal;


AP_HAL::Proc Scheduler::_failsafe = NULL;
volatile bool Scheduler::_timer_suspended = false;
volatile bool Scheduler::_timer_event_missed = false;

AP_HAL::MemberProc Scheduler::_timer_proc[SITL_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
uint8_t Scheduler::_num_timer_procs = 0;
bool Scheduler::_in_timer_proc = false;

AP_HAL::MemberProc Scheduler::_io_proc[SITL_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
uint8_t Scheduler::_num_io_procs = 0;
bool Scheduler::_in_io_proc = false;

Scheduler::Scheduler(SITL_State *sitlState) :
    _sitlState(sitlState),
    _stopped_clock_usec(0)
{
}

void Scheduler::init()
{
}

void Scheduler::delay_microseconds(uint16_t usec)
{
    uint64_t start = AP_HAL::micros64();
    uint64_t dtime;
    while ((dtime=(AP_HAL::micros64() - start) < usec)) {
        if (_stopped_clock_usec) {
            _sitlState->wait_clock(start+usec);
        } else {
            usleep(usec - dtime);
        }
    }
}

void Scheduler::delay(uint16_t ms)
{
    while (ms > 0) {
        delay_microseconds(1000);
        ms--;
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

    if (_num_timer_procs < SITL_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    }

}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
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

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::suspend_timer_procs() {
    _timer_suspended = true;
}

void Scheduler::resume_timer_procs() {
    _timer_suspended = false;
    if (_timer_event_missed) {
        _timer_event_missed = false;
        _run_timer_procs(false);
    }
}

bool Scheduler::in_timerprocess() {
    return _in_timer_proc || _in_io_proc;
}

void Scheduler::system_initialized() {
    if (_initialized) {
        AP_HAL::panic(
            "PANIC: scheduler system initialized called more than once");
    }
    int exceptions = FE_OVERFLOW | FE_DIVBYZERO;
#ifndef __i386__
    // i386 with gcc doesn't work with FE_INVALID
    exceptions |= FE_INVALID;
#endif
    if (_sitlState->_sitl == NULL || _sitlState->_sitl->float_exception) {
        feenableexcept(exceptions);
    } else {
        feclearexcept(exceptions);
    }
    _initialized = true;
}

void Scheduler::sitl_end_atomic() {
    if (_nested_atomic_ctr == 0)
        hal.uartA->println("NESTED ATOMIC ERROR");
    else
        _nested_atomic_ctr--;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    hal.uartA->println("REBOOT NOT IMPLEMENTED\r\n");
}

void Scheduler::_run_timer_procs(bool called_from_isr)
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
            if (_timer_proc[i]) {
                _timer_proc[i]();
            }
        }
    } else if (called_from_isr) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void Scheduler::_run_io_procs(bool called_from_isr)
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
    } else if (called_from_isr) {
        _timer_event_missed = true;
    }

    _in_io_proc = false;

    UARTDriver::from(hal.uartA)->_timer_tick();
    UARTDriver::from(hal.uartB)->_timer_tick();
    UARTDriver::from(hal.uartC)->_timer_tick();
    UARTDriver::from(hal.uartD)->_timer_tick();
    UARTDriver::from(hal.uartE)->_timer_tick();
}

/*
  set simulation timestamp
 */
void Scheduler::stop_clock(uint64_t time_usec)
{
    _stopped_clock_usec = time_usec;
    _run_io_procs(false);
}

#endif
