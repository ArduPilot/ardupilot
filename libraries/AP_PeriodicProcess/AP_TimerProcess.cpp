
#include "AP_TimerProcess.h"

extern "C" {
#include <inttypes.h>
#include <stdint.h>
#include "WConstants.h"
}

int AP_TimerProcess::_period;
ap_procedure AP_TimerProcess::_proc[AP_TIMERPROCESS_MAX_PROCS];
int AP_TimerProcess::_pidx = 0;

AP_TimerProcess::AP_TimerProcess(int period)
{
    _period = period;
}

void AP_TimerProcess::init( Arduino_Mega_ISR_Registry * isr_reg )
{
	// Enable Timer2 Overflow interrupt to trigger process.
	TIMSK2 = 0;                 // Disable interrupts
	TCCR2A = 0;                 // normal counting mode
	TCCR2B = _BV(CS21) | _BV(CS22);	// Set prescaler of clk/256
	TCNT2  = 0;                 // Set count to zero, so it goes off right away.
	TIFR2  = _BV(TOV2);	        // clear pending interrupts;
	TIMSK2 = _BV(TOIE2);        // enable the overflow interrupt

    for (int i = 0; i < AP_TIMERPROCESS_MAX_PROCS; i++)
        _proc[i] = NULL;

    isr_reg->register_signal( ISR_REGISTRY_TIMER2_OVF, AP_TimerProcess::run);
}

void AP_TimerProcess::register_process(void (*proc)(void) )
{
    if (_pidx < AP_TIMERPROCESS_MAX_PROCS)
        _proc[_pidx++] = proc;
}

void AP_TimerProcess::run(void)
{
    TCNT2 = _period;
    for (int i = 0; i < _pidx; i++) {
        if (_proc[i] != NULL)
            _proc[i]();
    }
}
