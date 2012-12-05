/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <avr/io.h>
#include <avr/interrupt.h>

#include "HAL_AVR.h"
#include "Scheduler.h"
using namespace AP_HAL_AVR;

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

extern const AP_HAL::HAL& hal;

static volatile uint32_t timer0_overflow_count = 0;
static volatile uint32_t timer0_millis = 0;
static uint8_t timer0_fract = 0;

/* AVRScheduler timer interrupt period is controlled by TCNT2.
 * 256-62 gives a 1kHz period. */
#define RESET_TCNT2_VALUE (256 - 62)

/* Static AVRScheduler variables: */
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

    // this needs to be called before setup() or some functions won't
    // work there
    sei();
 
    // set timer 0 prescale factor to 64
    // this combination is for the standard 168/328/1280/2560
    sbi(TCCR0B, CS01);
    sbi(TCCR0B, CS00);
    // enable timer 0 overflow interrupt
    sbi(TIMSK0, TOIE0);

    // timers 1 and 2 are used for phase-correct hardware pwm
    // this is better for motors as it ensures an even waveform
    // note, however, that fast pwm mode can achieve a frequency of up
    // 8 MHz (with a 16 MHz clock) at 50% duty cycle

    TCCR1B = 0;

    // set timer 1 prescale factor to 64
    sbi(TCCR1B, CS11);
    sbi(TCCR1B, CS10);
    // put timer 1 in 8-bit phase correct pwm mode
    sbi(TCCR1A, WGM10);

    sbi(TCCR3B, CS31);      // set timer 3 prescale factor to 64
    sbi(TCCR3B, CS30);
    sbi(TCCR3A, WGM30);     // put timer 3 in 8-bit phase correct pwm mode

    sbi(TCCR4B, CS41);      // set timer 4 prescale factor to 64
    sbi(TCCR4B, CS40);
    sbi(TCCR4A, WGM40);     // put timer 4 in 8-bit phase correct pwm mode

    sbi(TCCR5B, CS51);      // set timer 5 prescale factor to 64
    sbi(TCCR5B, CS50);
    sbi(TCCR5A, WGM50);     // put timer 5 in 8-bit phase correct pwm mode

    // set a2d prescale factor to 128
    // 16 MHz / 128 = 125 KHz, inside the desired 50-200 KHz range.
    // XXX: this will not work properly for other clock speeds, and
    // this code should use F_CPU to determine the prescale factor.
    sbi(ADCSRA, ADPS2);
    sbi(ADCSRA, ADPS1);
    sbi(ADCSRA, ADPS0);

    // enable a2d conversions
    sbi(ADCSRA, ADEN);

    // the bootloader connects pins 0 and 1 to the USART; disconnect them
    // here so they can be used as normal digital i/o; they will be
    // reconnected in Serial.begin()
    UCSR0B = 0;

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

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)


SIGNAL(TIMER0_OVF_vect)
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	uint32_t m = timer0_millis;
	uint8_t f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

uint32_t AVRScheduler::millis()
{
	uint32_t m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	SREG = oldSREG;

	return m;
}

/* micros() is essentially a static method, but we need it to be available
 * via virtual dispatch through the hal. */
uint32_t AVRScheduler::micros() {
    return _micros();
}

/* _micros() is the implementation of micros() as a static private method
 * so we can use it from inside _timer_event() without virtual dispatch. */
uint32_t AVRScheduler::_micros() {
	uint32_t m;
    uint8_t t;
	
	uint8_t oldSREG = SREG;
	cli();

	m = timer0_overflow_count;
	t = TCNT0;
  
	if ((TIFR0 & _BV(TOV0)) && (t < 255))
		m++;

	SREG = oldSREG;
	
	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

void AVRScheduler::delay(uint32_t ms)
{
	uint32_t start = _micros();
    
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

/* Delay for the given number of microseconds.  Assumes a 16 MHz clock. */
void AVRScheduler::delay_microseconds(uint16_t us)
{
	// for the 16 MHz clock on most Arduino boards
	// for a one-microsecond delay, simply return.  the overhead
	// of the function call yields a delay of approximately 1 1/8 us.
	if (--us == 0)
		return;

	// the following loop takes a quarter of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2;

	// account for the time taken in the preceeding commands.
	us -= 2;

	// busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
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
        proc(micros());
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
