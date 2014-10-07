#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "Scheduler.h"

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

YUNEECTimer YUNEECScheduler::_timer;

AP_HAL::Proc YUNEECScheduler::_failsafe = NULL;
volatile bool YUNEECScheduler::_timer_suspended = false;
volatile bool YUNEECScheduler::_timer_event_missed = false;
volatile bool YUNEECScheduler::_in_timer_proc = false;
AP_HAL::MemberProc YUNEECScheduler::_timer_proc[YUNEEC_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
uint8_t YUNEECScheduler::_num_timer_procs = 0;

YUNEECScheduler::YUNEECScheduler() :
	_delay_cb(NULL),
	_min_delay_cb_ms(65535),
	_initialized(false)
{}

void YUNEECScheduler::init(void* machtnichts) {
    // _timer: sets up timer hardware to implement millis & micros
    _timer.init();
    _timer.attachInterrupt(_timer_isr_event, _timer_failsafe_event);
}

void YUNEECScheduler::delay(uint16_t ms) {
	uint32_t start = _timer.millis();
    while (ms > 0) {
        while ((_timer.millis() - start) > 0) {
            ms--;
            if (ms == 0) break;
            start ++;
        }
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
}

uint32_t YUNEECScheduler::millis() {
	return _timer.millis();
}

uint32_t YUNEECScheduler::micros() {
	return _timer.micros();
}

void YUNEECScheduler::delay_microseconds(uint16_t us) {
    _timer.delay_microseconds(us);
}

void YUNEECScheduler::register_delay_callback(AP_HAL::Proc proc, uint16_t min_time_ms) {
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void YUNEECScheduler::register_timer_process(AP_HAL::MemberProc proc) {
    for (int i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < YUNEEC_SCHEDULER_MAX_TIMER_PROCS) {
        __disable_irq();
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
        __enable_irq();
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

void YUNEECScheduler::register_io_process(AP_HAL::MemberProc proc)
{
    // IO processes not supported on YUNEEC
}

void YUNEECScheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us) {
    /* XXX Assert period_us == 1000 */
	_failsafe = failsafe;
}

void YUNEECScheduler::suspend_timer_procs() {
	__disable_irq();
    _timer_suspended = true;
    __enable_irq();
}

void YUNEECScheduler::resume_timer_procs() {
	__disable_irq();
    _timer_suspended = false;
    __enable_irq();
    if (_timer_event_missed == true) {
        _run_timer_procs(false);
    	__disable_irq();
        _timer_event_missed = false;
        __enable_irq();
    }
}

bool YUNEECScheduler::in_timerprocess() {
    return _in_timer_proc;
}

bool YUNEECScheduler::system_initializing() {
    return !_initialized;
}

void YUNEECScheduler::system_initialized() {
    if (_initialized) {
        panic(PSTR("PANIC: scheduler::system_initialized called"
                   "more than once"));
    }
    _initialized = true;
}

void YUNEECScheduler::panic(const prog_char_t *errormsg) {
    /* Suspend timer processes. We still want the timer event to go off
     * to run the _failsafe code, however. */
    _timer_suspended = true;
    hal.console->println_P(errormsg);
    for(;;);
}

void YUNEECScheduler::reboot(bool hold_in_bootloader) {
    hal.uartA->println_P(PSTR("GOING DOWN FOR A REBOOT\r\n"));
    hal.scheduler->delay(100);
    NVIC_SystemReset();
}

void YUNEECScheduler::_timer_isr_event() {
    _run_timer_procs(true);
}

void YUNEECScheduler::_timer_failsafe_event() {
    if (_failsafe != NULL)
    	_failsafe();
}

void YUNEECScheduler::_run_timer_procs(bool called_from_isr) {
    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (uint8_t i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i] != NULL) {
                _timer_proc[i]();
            }
        }
    } else if (called_from_isr) {
        _timer_event_missed = true;
    }

    _in_timer_proc = false;
}

#endif
