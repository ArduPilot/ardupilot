/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_TimerProcess.h"

extern "C" {
#include <inttypes.h>
#include <stdint.h>
#include <avr/interrupt.h>
}
#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WConstants.h"
#endif

uint8_t AP_TimerProcess::_period;
ap_procedure AP_TimerProcess::_proc[AP_TIMERPROCESS_MAX_PROCS];
ap_procedure AP_TimerProcess::_failsafe;
ap_procedure AP_TimerProcess::_queued_proc = NULL;
bool AP_TimerProcess::_in_timer_call;
uint8_t AP_TimerProcess::_pidx = 0;
bool AP_TimerProcess::_suspended;

AP_TimerProcess::AP_TimerProcess(uint8_t period)
{
    _period = period;
}

void AP_TimerProcess::init( Arduino_Mega_ISR_Registry * isr_reg )
{
    // Enable Timer2 Overflow interrupt to trigger process.
    TIMSK2 = 0;                     // Disable interrupts
    TCCR2A = 0;                     // normal counting mode
    TCCR2B = _BV(CS21) | _BV(CS22);     // Set prescaler of clk/256
    TCNT2  = 0;                     // Set count to zero, so it goes off right away.
    TIFR2  = _BV(TOV2);                 // clear pending interrupts;
    TIMSK2 = _BV(TOIE2);            // enable the overflow interrupt

    _failsafe = NULL;
    _suspended = false;
    _in_timer_call = false;

    for (uint8_t i = 0; i < AP_TIMERPROCESS_MAX_PROCS; i++)
        _proc[i] = NULL;

    isr_reg->register_signal( ISR_REGISTRY_TIMER2_OVF, AP_TimerProcess::run);
}

/*
 *  register a process to be called at the timer interrupt rate
 */
void AP_TimerProcess::register_process(ap_procedure proc)
{
    // see if its already registered (due to double initialisation
    // of a driver)
    for (uint8_t i=0; i<_pidx; i++) {
        if (_proc[i] == proc) return;
    }
    cli();
    if (_pidx < AP_TIMERPROCESS_MAX_PROCS)
        _proc[_pidx++] = proc;
    sei();
}

void AP_TimerProcess::set_failsafe(ap_procedure proc)
{
    _failsafe = proc;
}

/*
 *  queue a process to be run as soon as any currently running ap_processes complete
 */
bool AP_TimerProcess::queue_process(ap_procedure proc)
{
    // check if we are running any ap_processes
    if( _in_timer_call || _suspended ) {
         // queue the process to run after current processes finish
        _queued_proc = proc;
        return false;
    }else{
        // run process immediately
        _suspended = true;
        proc(micros());
        _suspended = false;
        return true;
    }
}

void AP_TimerProcess::suspend_timer(void)
{
    _suspended = true;
}

void AP_TimerProcess::resume_timer(void)
{
    _suspended = false;
}

void AP_TimerProcess::run(void)
{
    // we enable the interrupt again immediately and also enable
    // interrupts. This allows other time critical interrupts to
    // run (such as the serial receive interrupt). We catch the
    // timer calls taking too long using _in_timer_call.
    // This approach also gives us a nice uniform spacing between
    // timer calls
    TCNT2 = _period;
    sei();

    uint32_t tnow = micros();

    if (_in_timer_call) {
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
    _in_timer_call = true;

    if (!_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _pidx; i++) {
            if (_proc[i] != NULL) {
                _proc[i](tnow);
            }
        }

        // run any queued processes
        cli();
        ap_procedure qp = _queued_proc;
        _queued_proc = NULL;
        sei();
        if( qp != NULL ) {
            _suspended = true;
            qp(tnow);
            _suspended = false;
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe(tnow);
    }

    _in_timer_call = false;
}
