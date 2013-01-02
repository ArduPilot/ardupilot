/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "AP_HAL_PX4.h"
#include "Scheduler.h"
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>
#include <stdio.h>

#define MAIN_TIMER_SIGNAL 17

using namespace PX4;

extern const AP_HAL::HAL& hal;


AP_HAL::TimedProc PX4Scheduler::_failsafe = NULL;
volatile bool PX4Scheduler::_timer_suspended = false;
AP_HAL::TimedProc PX4Scheduler::_timer_proc[PX4_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
uint8_t PX4Scheduler::_num_timer_procs = 0;
bool PX4Scheduler::_in_timer_proc = false;
struct timeval PX4Scheduler::_sketch_start_time;

PX4Scheduler::PX4Scheduler()
{}

void PX4Scheduler::init(void *unused) 
{
    sigset_t           sigset;
    struct sigaction   act;
    struct sigaction   oact;
    struct sigevent    notify;
    struct itimerspec  timer;
    timer_t            timerid;
    int                status;

	gettimeofday(&_sketch_start_time,NULL);

    /* setup a posix timer at 1kHz */
    (void)sigemptyset(&sigset);
    (void)sigaddset(&sigset, MAIN_TIMER_SIGNAL);
    status = sigprocmask(SIG_UNBLOCK, &sigset, NULL);
    if (status != OK) goto failed;

    act.sa_sigaction = _timer_event;
    act.sa_flags  = SA_SIGINFO;

    (void)sigfillset(&act.sa_mask);
    (void)sigdelset(&act.sa_mask, MAIN_TIMER_SIGNAL);
    status = sigaction(MAIN_TIMER_SIGNAL, &act, &oact);
    if (status != OK) goto failed;

    notify.sigev_notify          = SIGEV_SIGNAL;
    notify.sigev_signo           = MAIN_TIMER_SIGNAL;
    notify.sigev_value.sival_int = 0;

    status = timer_create(CLOCK_REALTIME, &notify, &timerid);
    if (status != OK) goto failed;

    /* Start the POSIX timer */

    timer.it_value.tv_sec     = 0;
    timer.it_value.tv_nsec    = 1000000;
    timer.it_interval.tv_sec  = 0;
    timer.it_interval.tv_nsec = 1000000;
    
    status = timer_settime(timerid, 0, &timer, NULL);
    if (status != OK) goto failed;
    
    return;

failed:
    panic("Failed to setup PX4 1kHz timer");
}

uint32_t PX4Scheduler::_micros() 
{
	struct timeval tp;
	gettimeofday(&tp,NULL);
	return 1.0e6*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
		      (_sketch_start_time.tv_sec +
		       (_sketch_start_time.tv_usec*1.0e-6)));
}

uint32_t PX4Scheduler::micros() 
{
    return _micros();
}

uint32_t PX4Scheduler::millis() 
{
	struct timeval tp;
	gettimeofday(&tp,NULL);
	return 1.0e3*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
		      (_sketch_start_time.tv_sec +
		       (_sketch_start_time.tv_usec*1.0e-6)));
}

void PX4Scheduler::delay_microseconds(uint16_t usec) 
{
	uint32_t start = micros();
	while (micros() - start < usec) {
		usleep(usec - (micros() - start));
	}
}

void PX4Scheduler::delay(uint16_t ms)
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
    _failsafe = failsafe;
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
}

void PX4Scheduler::reboot() 
{
    hal.uartA->println_P(PSTR("REBOOT NOT IMPLEMENTED\r\n"));
}

void PX4Scheduler::_timer_event(int signo, siginfo_t *info, void *ucontext) 
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

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe(tnow);
    }

    _in_timer_proc = false;
}

void PX4Scheduler::panic(const prog_char_t *errormsg) {
    hal.console->println_P(errormsg);
    for(;;);
}

#endif
