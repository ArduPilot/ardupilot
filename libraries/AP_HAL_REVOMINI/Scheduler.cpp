
#include "Scheduler.h"
#include <delay.h>
#include <timer.h>
#include <HardwareTimer.h>
#include <systick.h>
#include <AP_Notify/AP_Notify.h>
#include "GPIO.h"

using namespace REVOMINI;

extern const AP_HAL::HAL& hal;

AP_HAL::Proc REVOMINIScheduler::_failsafe = NULL;
volatile bool REVOMINIScheduler::_timer_suspended = false;
volatile bool REVOMINIScheduler::_timer_event_missed = false;
volatile bool REVOMINIScheduler::_in_timer_proc = false;
AP_HAL::MemberProc REVOMINIScheduler::_timer_proc[REVOMINI_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
uint8_t REVOMINIScheduler::_num_timer_procs = 0;
uint32 REVOMINIScheduler::_scheduler_last_call = 0;
uint32 REVOMINIScheduler::_armed_last_call = 0;
uint16_t REVOMINIScheduler::_scheduler_led = 0;

REVOMINIScheduler::REVOMINIScheduler()
:
	    _delay_cb(NULL),
	    _min_delay_cb_ms(65535),
	    _initialized(false)
{}

void REVOMINIScheduler::init()
{

    uint32_t period = (2000000UL / 1000) - 1; // 1000 Hz = 1KHz
    uint32_t prescaler =  (uint16_t) ((SystemCoreClock /2) / 2000000) - 1; //2MHz 0.5us ticks

    timer_pause(TIMER7);
    timer_set_prescaler(TIMER7,prescaler);
    timer_set_count(TIMER7,0);
    timer_set_reload(TIMER7,period);
    timer_attach_interrupt(TIMER7, TIMER_UPDATE_INTERRUPT, _timer_isr_event);
    NVIC_SetPriority(TIM7_IRQn,5);
    timer_resume(TIMER7);

    //systick_attach_callback(_timer_isr_event);
}

void REVOMINIScheduler::delay(uint16_t ms)
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

uint32_t REVOMINIScheduler::millis() {
    return systick_uptime();
}

uint32_t REVOMINIScheduler::micros() {
    uint32 fms, lms;
    uint32 cycle_cnt;
    uint32 res;
    do {
        // make sure millis() return the same value before and after
        // getting the systick count
        fms = millis();
        cycle_cnt = systick_get_count();
        lms = millis();
    } while (lms != fms);

#define US_PER_MS               1000
    /* SYSTICK_RELOAD_VAL is 1 less than the number of cycles it
       actually takes to complete a SysTick reload */
    res = (fms * US_PER_MS) +
        (SYSTICK_RELOAD_VAL + 1 - cycle_cnt) / CYCLES_PER_MICROSECOND;

    return res;
#undef US_PER_MS
}

/*
uint64_t REVOMINIScheduler::millis64()
{
    return systick_uptime();
}
uint64_t REVOMINIScheduler::micros64()
{
    uint64 fms, lms;
    uint32 cycle_cnt;
    uint64 res;
    do {
        // make sure millis() return the same value before and after
        // getting the systick count
        fms = systick_uptime();
        cycle_cnt = systick_get_count();
        lms = systick_uptime();
    } while (lms != fms);

#define US_PER_MS               1000
    //  SYSTICK_RELOAD_VAL is 1 less than the number of cycles it    actually takes to complete a SysTick reload 
    res = (fms * US_PER_MS) +
        (SYSTICK_RELOAD_VAL + 1 - cycle_cnt) / CYCLES_PER_MICROSECOND;

    return res;
#undef US_PER_MS

}

*/
void REVOMINIScheduler::delay_microseconds(uint16_t us)
{
    delay_us((uint32_t)us);
}

void REVOMINIScheduler::register_delay_callback(AP_HAL::Proc proc,
            uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void REVOMINIScheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (int i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < REVOMINI_SCHEDULER_MAX_TIMER_PROCS) {
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

void REVOMINIScheduler::register_io_process(AP_HAL::MemberProc proc)
{
    // IO processes not supported on AVR
}

void REVOMINIScheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us) {
    /* XXX Assert period_us == 1000 */
    _failsafe = failsafe;
}
void REVOMINIScheduler::suspend_timer_procs()
{
    _timer_suspended = true;
}


void REVOMINIScheduler::resume_timer_procs()
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timer_procs(false);
        _timer_event_missed = false;
    }
}

bool REVOMINIScheduler::in_timerprocess()
{
    return _in_timer_proc;
}

#define LED_GRN (*((unsigned long int *) 0x42408294)) // PB5
#define LED_YLW (*((unsigned long int *) 0x42408298)) // PB6 // Not included
#define LED_RED (*((unsigned long int *) 0x42408290)) // PB4


void REVOMINIScheduler::_timer_isr_event() {

    uint32 fms=systick_uptime();

    if(fms - _scheduler_last_call >= 100)
	{
	 if (_scheduler_led == 1)
	     {
	     LED_YLW=0;
	     _scheduler_led=0;
	     }
	     else
	     {
             LED_YLW=1;
             _scheduler_led=1;
	     }
	 _scheduler_last_call = fms;
	}
    // replaced from AP_HAL_Notify (LED always ON to fast flashing)
    if (AP_Notify::flags.armed) {
	if(fms - _armed_last_call >= 100){
	    hal.gpio->toggle(HAL_GPIO_A_LED_PIN);
	    _armed_last_call = fms;
	}
    }

    _run_timer_procs(true);
}

void REVOMINIScheduler::_run_timer_procs(bool called_from_isr) {

    if (_in_timer_proc) {
        return;
    }
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

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe();
    }

    _in_timer_proc = false;
}



void REVOMINIScheduler::system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called"
                   "more than once");
    }
    _initialized = true;
}

/*
void REVOMINIScheduler::panic(const char* errormsg) {
   hal.console->println_P(errormsg);
    //for(;;);
}
*/

void REVOMINIScheduler::reboot(bool hold_in_bootloader) {
    return;
}

