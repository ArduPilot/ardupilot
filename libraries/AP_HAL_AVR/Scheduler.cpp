/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "Scheduler.h"
#include "utility/ISRRegistry.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

/* AVRScheduler timer interrupt period is controlled by TCNT2.
 * 256-62 gives a 1kHz period. */
#define RESET_TCNT2_VALUE (256 - 62)

/* Static AVRScheduler variables: */
AVRTimer AVRScheduler::_timer;

AP_HAL::TimedProc AVRScheduler::_failsafe = NULL;
volatile bool AVRScheduler::_timer_suspended = false;
volatile bool AVRScheduler::_timer_event_missed = false;
volatile bool AVRScheduler::_in_timer_proc = false;
AP_HAL::TimedProc AVRScheduler::_timer_proc[AVR_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
uint8_t AVRScheduler::_num_timer_procs = 0;


AVRScheduler::AVRScheduler() :
    _delay_cb(NULL),
    _min_delay_cb_ms(65535),
    _initialized(false)
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
    /* Register _timer_isr_event to trigger on overflow */
    isrregistry->register_signal(ISR_REGISTRY_TIMER2_OVF, _timer_isr_event);   
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

void AVRScheduler::delay(uint16_t ms)
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

void AVRScheduler::register_io_process(AP_HAL::TimedProc proc) 
{
    // IO processes not supported on AVR
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
    if (_timer_event_missed == true) {
        _run_timer_procs(false);
        _timer_event_missed = false;
    }
}

bool AVRScheduler::in_timerprocess() {
    return _in_timer_proc;
}

void AVRScheduler::_timer_isr_event() {
    // we enable the interrupt again immediately and also enable
    // interrupts. This allows other time critical interrupts to
    // run (such as the serial receive interrupt). We catch the
    // timer calls taking too long using _in_timer_call.
    // This approach also gives us a nice uniform spacing between
    // timer calls

    TCNT2 = RESET_TCNT2_VALUE;
    sei();
    _run_timer_procs(true);
}

void AVRScheduler::_run_timer_procs(bool called_from_isr) {

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
    } else if (called_from_isr) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe(tnow);
    }

    _in_timer_proc = false;
}

bool AVRScheduler::system_initializing() {
    return !_initialized;
}

void AVRScheduler::system_initialized() {
    if (_initialized) {
        panic(PSTR("PANIC: scheduler::system_initialized called"
                   "more than once"));
    }
    _initialized = true;
}

void AVRScheduler::panic(const prog_char_t* errormsg) {
    /* Suspend timer processes. We still want the timer event to go off
     * to run the _failsafe code, however. */
    _timer_suspended = true;
    /* Print the error message on both ports */
    hal.uartA->println_P(errormsg);
    hal.uartC->println_P(errormsg);
    /* Spin forever. */
    for(;;);
}

void AVRScheduler::reboot(bool hold_in_bootloader) {
    hal.uartA->println_P(PSTR("GOING DOWN FOR A REBOOT\r\n"));
    hal.scheduler->delay(100);
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    /* The APM2 bootloader will reset the watchdog shortly after
     * starting, so we can use the watchdog to force a reboot
     */
    cli();
    wdt_enable(WDTO_15MS);
    for(;;);
#else
    cli();
    /* Making a null pointer call will cause all AVRs to reboot
     * but they may not come back alive properly - we need to setup
     * the IO the way the bootloader would.
     */
    void (*fn)(void) = NULL;
    fn();
    for(;;);
#endif

}

#endif
