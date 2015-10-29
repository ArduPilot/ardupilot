/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include "AP_HAL_AVR_SITL.h"
#include "Scheduler.h"
#include <sys/time.h>
#include <unistd.h>
#include <fenv.h>
#include "fenv_polyfill.h"
#include <signal.h>
#include <pthread.h>

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

static void sigcont_handler(int)
{
}

SITLScheduler::SITLScheduler(SITL_State *sitlState) :
    _sitlState(sitlState),
    stopped_clock_usec(0)
{
    signal(SIGCONT, sigcont_handler);
    pthread_mutex_init(&clock_barrier.m, NULL);
    pthread_cond_init(&clock_barrier.cv, NULL);
    clock_barrier.state = CLOCK_WAIT_INIT;
}

void SITLScheduler::init(void *unused) 
{
	gettimeofday(&_sketch_start_time,NULL);
}

uint64_t SITLScheduler::_micros64() 
{
	struct timeval tp;
	gettimeofday(&tp,NULL);
	uint64_t ret = 1.0e6*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
                          (_sketch_start_time.tv_sec +
                           (_sketch_start_time.tv_usec*1.0e-6)));
    return ret;
}

uint64_t SITLScheduler::micros64() 
{
    if (stopped_clock_usec) {
        return stopped_clock_usec;
    }
    return _micros64();
}

uint32_t SITLScheduler::micros() 
{
    return micros64() & 0xFFFFFFFF;
}

uint64_t SITLScheduler::millis64() 
{
    if (stopped_clock_usec) {
        return stopped_clock_usec/1000;
    }
	struct timeval tp;
	gettimeofday(&tp,NULL);
	uint64_t ret = 1.0e3*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
                          (_sketch_start_time.tv_sec +
                           (_sketch_start_time.tv_usec*1.0e-6)));
    return ret;
}

uint32_t SITLScheduler::millis() 
{
    return millis64() & 0xFFFFFFFF;
}

extern AVR_SITL::SITL_State *g_state;

/*
  implement a barrier wait for SITL lock-step scheduling. We don't use
  pthread_barrier_* as it is not available in cygwin
 */
void SITLScheduler::clock_barrier_wait(void)
{
    pthread_mutex_lock(&clock_barrier.m);

    while (clock_barrier.state == CLOCK_WAIT_TWO || 
           clock_barrier.state == CLOCK_WAIT_THREE) {
        pthread_cond_wait(&clock_barrier.cv, &clock_barrier.m);
    }

    if (clock_barrier.state == CLOCK_WAIT_INIT) {
        clock_barrier.state = CLOCK_WAIT_ONE;
		pthread_cond_signal(&clock_barrier.cv);
		while (clock_barrier.state != CLOCK_WAIT_TWO) {
			pthread_cond_wait(&clock_barrier.cv, &clock_barrier.m);
		}
        clock_barrier.state = CLOCK_WAIT_THREE;
		pthread_cond_signal(&clock_barrier.cv);
    } else {
        clock_barrier.state = CLOCK_WAIT_TWO;
		pthread_cond_signal(&clock_barrier.cv);
		while (clock_barrier.state != CLOCK_WAIT_THREE) {
			pthread_cond_wait(&clock_barrier.cv, &clock_barrier.m);
		}
        clock_barrier.state = CLOCK_WAIT_INIT;
		pthread_cond_signal(&clock_barrier.cv);
	}
    pthread_mutex_unlock(&clock_barrier.m);
}

void SITLScheduler::delay_microseconds(uint16_t usec) 
{
	uint64_t start = micros64();
    uint64_t dtime;
	while ((dtime=(micros64() - start) < usec)) {
        if (stopped_clock_usec) {
            /*
              we are using a synthetic clock. We want to wait until
              the stop_clock() call advances the clock
             */
            clock_barrier_wait();
        } else {
            usleep(usec - dtime);
        }
	}
}

void SITLScheduler::delay(uint16_t ms)
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
        _failsafe();
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

/*
  set simulation timestamp
 */
void SITLScheduler::stop_clock(uint64_t time_usec)
{
    if (time_usec == 1) {
        // special case for initialisation in synthetic clock mode
        stopped_clock_usec = time_usec;
        return;
    }

    if (stopped_clock_usec != 0) {
        /*
          wait until the main thread is waiting for us. This ensures
          that any processing is complete before we advance the clock
         */
        clock_barrier_wait();
    }
    stopped_clock_usec = time_usec;
    /*
      wait again to ensure the main thread can't get behind the FDM
     */
    clock_barrier_wait();
    _run_io_procs(false);
}

#endif
