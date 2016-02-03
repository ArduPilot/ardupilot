// Scheduler.cpp
//
// Flymaple Scheduler.
// We use systick interrupt for the 1kHz ordinary timers.
// We use a slightly higher priority HardwareTimer 2 for the failsafe callbacks
// so a hung timer wont prevent the failsafe timer interrupt running
//
// Use of noInterrupts()/interrupts() on FLymaple ARM processor.
// Please see the notes in FlymaplePortingNotes.txt in this directory for 
// information about disabling interrupts on Flymaple

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include "Scheduler.h"

#include <stdarg.h>

#include "FlymapleWirish.h"

// Flymaple: Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects (eg SerialUSB) that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

// Not declared in any libmaple headers :-(
extern "C" 
{ 
    void systick_attach_callback(void (*callback)(void)); 
};

// Use Maple hardware timer for 1khz failsafe timer
// Caution, this must agree with the interrupt number passed to
// nvic_irq_set_priority
static HardwareTimer _failsafe_timer(2);

using namespace AP_HAL_FLYMAPLE_NS;

extern const AP_HAL::HAL& hal;

AP_HAL::Proc FLYMAPLEScheduler::_failsafe = NULL;
volatile bool     FLYMAPLEScheduler::_timer_suspended = false;
volatile bool     FLYMAPLEScheduler::_timer_event_missed = false;
volatile bool     FLYMAPLEScheduler::_in_timer_proc = false;
AP_HAL::MemberProc FLYMAPLEScheduler::_timer_proc[FLYMAPLE_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
uint8_t           FLYMAPLEScheduler::_num_timer_procs = 0;

FLYMAPLEScheduler::FLYMAPLEScheduler() :
    _delay_cb(NULL),
    _min_delay_cb_ms(65535),
    _initialized(false)
{}

void FLYMAPLEScheduler::init()
{
    delay_us(2000000); // Wait for startup so we have time to connect a new USB console
    // 1kHz interrupts from systick for normal timers
    systick_attach_callback(_timer_procs_timer_event);

    // Set up Maple hardware timer for 1khz failsafe timer
    // ref: http://leaflabs.com/docs/lang/api/hardwaretimer.html#lang-hardwaretimer
    _failsafe_timer.pause();
    _failsafe_timer.setPeriod(1000); // 1000us = 1kHz
    _failsafe_timer.setChannelMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);// Set up an interrupt on channel 1
    _failsafe_timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
    _failsafe_timer.attachInterrupt(TIMER_CH1, _failsafe_timer_event);
    _failsafe_timer.refresh();// Refresh the timer's count, prescale, and overflow
    _failsafe_timer.resume(); // Start the timer counting
    // We run this timer at a higher priority, so that a broken timer handler (ie one that hangs)
    // will not prevent the failsafe timer interrupt.
    // Caution: the timer number must agree with the HardwareTimer number
    nvic_irq_set_priority(NVIC_TIMER2, 0x14);
}

// This function may calls the _delay_cb to use up time
void FLYMAPLEScheduler::delay(uint16_t ms)
{    
    uint32_t start = AP_HAL::micros();
    
    while (ms > 0) {
        while ((AP_HAL::micros() - start) >= 1000) {
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

void FLYMAPLEScheduler::delay_microseconds(uint16_t us)
{ 
    delay_us(us);
}

void FLYMAPLEScheduler::register_delay_callback(AP_HAL::Proc proc, uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void FLYMAPLEScheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (int i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < FLYMAPLE_SCHEDULER_MAX_TIMER_PROCS) {
        /* this write to _timer_proc can be outside the critical section
         * because that memory won't be used until _num_timer_procs is
         * incremented. */
        _timer_proc[_num_timer_procs] = proc;
        /* _num_timer_procs is used from interrupt, and multiple bytes long. */
        noInterrupts();
        _num_timer_procs++;
        interrupts();
    }
}

void FLYMAPLEScheduler::register_io_process(AP_HAL::MemberProc k)
{
    // IO processes not supported on FLYMAPLE
}

void FLYMAPLEScheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    /* XXX Assert period_us == 1000 */
    _failsafe = failsafe;
}

void FLYMAPLEScheduler::suspend_timer_procs()
{
    _timer_suspended = true;
}

void FLYMAPLEScheduler::resume_timer_procs()
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timer_procs(false);
        _timer_event_missed = false;
    }
}

bool FLYMAPLEScheduler::in_timerprocess() {
    return _in_timer_proc;
}

void FLYMAPLEScheduler::_timer_procs_timer_event() {
    _run_timer_procs(true);
}

// Called by HardwareTimer when a failsafe timer event occurs
void FLYMAPLEScheduler::_failsafe_timer_event() 
{
    // run the failsafe, if one is setup
    if (_failsafe != NULL)
        _failsafe();
}

void FLYMAPLEScheduler::_run_timer_procs(bool called_from_isr) 
{
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

    _in_timer_proc = false;
}

void FLYMAPLEScheduler::system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called"
                   "more than once");
    }
    _initialized = true;
}

void FLYMAPLEScheduler::reboot(bool hold_in_bootloader) {
    hal.uartA->println("GOING DOWN FOR A REBOOT\r\n");
    hal.scheduler->delay(100);
    nvic_sys_reset();
}

#endif
