/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <avr/io.h>
#include <avr/interrupt.h>

#include "HAL_AVR.h"
#include "Scheduler.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

/* AVRScheduler timer interrupt period is controlled by TCNT2.
 * 256-62 gives a 1kHz period. */
#define RESET_TCNT2_VALUE (256 - 62)

/* Static AVRScheduler variables: */
AVRTimer AVRScheduler::_timer;

AP_HAL::TimedProc AVRScheduler::_failsafe = NULL;
volatile bool AVRScheduler::_timer_suspended = false;
AP_HAL::TimedProc AVRScheduler::_timer_proc[AVR_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
AP_HAL::TimedProc AVRScheduler::_defered_timer_proc = NULL;
uint8_t AVRScheduler::_num_timer_procs = 0;
bool AVRScheduler::_in_timer_proc = false;


AVRScheduler::AVRScheduler() :
    _delay_cb(NULL),
    _min_delay_cb_ms(65535),
    _nested_atomic_ctr(0)
{}

void AVRScheduler::init(void* _isrregistry) {
    ISRRegistry* isrregistry = (ISRRegistry*) _isrregistry;

    /* _timer: sets up timer hardware to Arduino defaults, and
     * uses TIMER0 to implement millis & micros */
    _timer.init();

    /* TIMER2: Setup the overflow interrupt to occur at 1khz. */
    TIMSK2 = 0;                     /* Disable timer interrupt */
    TCCR2A = 0;                     /* Normal counting mode */
    TCCR2B = _BV(CS21) | _BV(CS22); /* Prescaler to clk/256 */
    TCNT2 = 0;                      /* Set count to 0 */
    TIFR2 = _BV(TOV2);              /* Clear pending interrupts */
    TIMSK2 = _BV(TOIE2);            /* Enable overflow interrupt*/
    /* Register _timer_event to trigger on overflow */
    isrregistry->register_signal(ISR_REGISTRY_TIMER2_OVF, _timer_event);   
}

uint32_t AVRScheduler::micros() {
    return _timer.micros();
}

uint32_t AVRScheduler::millis() {
    return _timer.millis();
}

void AVRScheduler::delay_microseconds(uint16_t us) {
    _timer.delay_microseconds(us);
}

void AVRScheduler::delay(uint32_t ms)
{
	uint32_t start = _timer.micros();
    
    while (ms > 0) {
        while ((_timer.micros() - start) >= 1000) {
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

void AVRScheduler::register_delay_callback(AP_HAL::Proc proc,
        uint16_t min_time_ms) {
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void AVRScheduler::register_timer_process(AP_HAL::TimedProc proc) {
    for (int i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < AVR_SCHEDULER_MAX_TIMER_PROCS) {
        /* this write to _timer_proc can be outside the critical section
         * because that memory won't be used until _num_timer_procs is
         * incremented. */
        _timer_proc[_num_timer_procs] = proc;
        /* _num_timer_procs is used from interrupt, and multiple bytes long. */
        cli();
        _num_timer_procs++;
        sei();
    }

}

bool AVRScheduler::defer_timer_process(AP_HAL::TimedProc proc) {
    if ( _in_timer_proc || _timer_suspended ) {
        _defered_timer_proc = proc;
        return false;
    } else {
        _timer_suspended = true;
        sei();
        proc(_timer.micros());
        _timer_suspended = false;
        return true;
    }
}

void AVRScheduler::register_timer_failsafe(
        AP_HAL::TimedProc failsafe, uint32_t period_us) {
    /* XXX Assert period_us == 1000 */
    _failsafe = failsafe;
}

void AVRScheduler::suspend_timer_procs() {
    _timer_suspended = true;
}

void AVRScheduler::resume_timer_procs() {
    _timer_suspended = false;
}

void AVRScheduler::_timer_event() {
    // we enable the interrupt again immediately and also enable
    // interrupts. This allows other time critical interrupts to
    // run (such as the serial receive interrupt). We catch the
    // timer calls taking too long using _in_timer_call.
    // This approach also gives us a nice uniform spacing between
    // timer calls

    TCNT2 = RESET_TCNT2_VALUE;
    sei();

    uint32_t tnow = _timer.micros();
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
    cli();
    /* Atomic read and clear: */
    AP_HAL::TimedProc defered = _defered_timer_proc;
    _defered_timer_proc = NULL;
    sei();
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

void AVRScheduler::begin_atomic() {
    _nested_atomic_ctr++;
    cli();
}

void AVRScheduler::end_atomic() {
    _nested_atomic_ctr--;
    if (_nested_atomic_ctr == 0) {
        sei();
    }
}

void AVRScheduler::reboot() {
    hal.uartA->println_P(PSTR("GOING DOWN FOR A REBOOT\r\n"));
    hal.scheduler->delay(100);

    cli();
    /* Making a null pointer call will cause all AVRs to reboot
     * but they may not come back alive properly - we need to setup
     * the IO the way the bootloader would.
     * pch will go back and fix this.
     */
    void (*fn)(void) = NULL;
    fn();

    for(;;);
}

#endif
