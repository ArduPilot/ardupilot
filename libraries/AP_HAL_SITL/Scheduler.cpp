#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_SITL.h"
#include "Scheduler.h"
#include "UARTDriver.h"
#include <sys/time.h>
#include <fenv.h>
#include <pthread.h>

using namespace HALSITL;

extern const AP_HAL::HAL& hal;


AP_HAL::Proc Scheduler::_failsafe = nullptr;

AP_HAL::MemberProc Scheduler::_timer_proc[SITL_SCHEDULER_MAX_TIMER_PROCS] = {nullptr};
uint8_t Scheduler::_num_timer_procs = 0;
bool Scheduler::_in_timer_proc = false;

AP_HAL::MemberProc Scheduler::_io_proc[SITL_SCHEDULER_MAX_TIMER_PROCS] = {nullptr};
uint8_t Scheduler::_num_io_procs = 0;
bool Scheduler::_in_io_proc = false;
bool Scheduler::_should_reboot = false;

Scheduler::Scheduler(SITL_State *sitlState) :
    _sitlState(sitlState),
    _stopped_clock_usec(0)
{
}

void Scheduler::init()
{
    _main_ctx = pthread_self();
}

bool Scheduler::in_main_thread() const
{
    if (!_in_timer_proc && !_in_io_proc && pthread_self() == _main_ctx) {
        return true;
    }
    return false;
}

void Scheduler::delay_microseconds(uint16_t usec)
{
    uint64_t start = AP_HAL::micros64();
    do {
        uint64_t dtime = AP_HAL::micros64() - start;
        if (dtime >= usec) {
            break;
        }
        _sitlState->wait_clock(start + usec);
    } while (true);
}

void Scheduler::delay(uint16_t ms)
{
    uint32_t start = AP_HAL::millis();
    uint32_t now = start;
    do {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= (ms - (now - start))) {
            if (in_main_thread()) {
                call_delay_cb();
            }
        }
        now = AP_HAL::millis();
    } while (now - start < ms);
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
    if (_sitlState->_sitl == nullptr || _sitlState->_sitl->float_exception) {
        feenableexcept(exceptions);
    } else {
        feclearexcept(exceptions);
    }
    _initialized = true;
}

void Scheduler::sitl_end_atomic() {
    if (_nested_atomic_ctr == 0) {
        hal.uartA->printf("NESTED ATOMIC ERROR\n");
    } else {
        _nested_atomic_ctr--;
    }
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    _should_reboot = true;
}

void Scheduler::_run_timer_procs()
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
        if (_failsafe != nullptr) {
            _failsafe();
        }
        return;
    }
    _in_timer_proc = true;

    // now call the timer based drivers
    for (int i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void Scheduler::_run_io_procs()
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    // now call the IO based drivers
    for (int i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }

    _in_io_proc = false;

    hal.uartA->_timer_tick();
    hal.uartB->_timer_tick();
    hal.uartC->_timer_tick();
    hal.uartD->_timer_tick();
    hal.uartE->_timer_tick();
    hal.uartF->_timer_tick();
    hal.uartG->_timer_tick();
}

/*
  set simulation timestamp
 */
void Scheduler::stop_clock(uint64_t time_usec)
{
    _stopped_clock_usec = time_usec;
    if (time_usec - _last_io_run > 10000) {
        _last_io_run = time_usec;
        _run_io_procs();
    }
}

/*
  trampoline for thread create
*/
void *Scheduler::thread_create_trampoline(void *ctx)
{
    AP_HAL::MemberProc *t = (AP_HAL::MemberProc *)ctx;
    (*t)();
    free(t);
    return nullptr;
}


/*
  create a new thread
*/
bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name, uint32_t stack_size, priority_base base, int8_t priority)
{
    // take a copy of the MemberProc, it is freed after thread exits
    AP_HAL::MemberProc *tproc = (AP_HAL::MemberProc *)malloc(sizeof(proc));
    if (!tproc) {
        return false;
    }
    *tproc = proc;
    pthread_t thread {};
    if (pthread_create(&thread, NULL, thread_create_trampoline, tproc) != 0) {
        free(tproc);
        return false;
    }
    return true;
}

