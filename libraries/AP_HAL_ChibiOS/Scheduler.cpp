/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include "AP_HAL_ChibiOS.h"
#include "Scheduler.h"
#include "Util.h"

#include <AP_HAL_ChibiOS/UARTDriver.h>
#include <AP_HAL_ChibiOS/AnalogIn.h>
#include <AP_HAL_ChibiOS/Storage.h>
#include <AP_HAL_ChibiOS/RCOutput.h>
#include <AP_HAL_ChibiOS/RCInput.h>
#include <AP_HAL_ChibiOS/CAN.h>

#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "shared_dma.h"

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

#if TEST_IDLE
THD_WORKING_AREA(_test_thread_wa, 256);
#endif
THD_WORKING_AREA(_timer_thread_wa, 2048);
THD_WORKING_AREA(_rcin_thread_wa, 512);
#ifdef HAL_PWM_ALARM
THD_WORKING_AREA(_toneAlarm_thread_wa, 512);
#endif
THD_WORKING_AREA(_io_thread_wa, 2048);
THD_WORKING_AREA(_storage_thread_wa, 2048);
#if HAL_WITH_UAVCAN
THD_WORKING_AREA(_uavcan_thread_wa, 4096);
#endif

Scheduler::Scheduler()
{}

void Scheduler::init()
{
#if TEST_IDLE
    // setup the test thread - this will just burn cpu
    _test_thread_ctx = chThdCreateStatic(_test_thread_wa,
                     sizeof(_test_thread_wa),
                     APM_TEST_PRIORITY,         /* Initial priority.    */
                     _test_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
#endif
    // setup the timer thread - this will call tasks at 1kHz
    _timer_thread_ctx = chThdCreateStatic(_timer_thread_wa,
                     sizeof(_timer_thread_wa),
                     APM_TIMER_PRIORITY,        /* Initial priority.    */
                     _timer_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */

    // setup the uavcan thread - this will call tasks at 1kHz
#if HAL_WITH_UAVCAN
    _uavcan_thread_ctx = chThdCreateStatic(_uavcan_thread_wa,
                     sizeof(_uavcan_thread_wa),
                     APM_UAVCAN_PRIORITY,        /* Initial priority.    */
                     _uavcan_thread,            /* Thread function.     */
                     this);                     /* Thread parameter.    */
#endif
    // setup the RCIN thread - this will call tasks at 1kHz
    _rcin_thread_ctx = chThdCreateStatic(_rcin_thread_wa,
                     sizeof(_rcin_thread_wa),
                     APM_RCIN_PRIORITY,        /* Initial priority.    */
                     _rcin_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */

    // the toneAlarm thread runs at a medium priority
#ifdef HAL_PWM_ALARM
    _toneAlarm_thread_ctx = chThdCreateStatic(_toneAlarm_thread_wa,
                     sizeof(_toneAlarm_thread_wa),
                     APM_TONEALARM_PRIORITY,        /* Initial priority.    */
                     _toneAlarm_thread,             /* Thread function.     */
                     this);                    /* Thread parameter.    */
#endif
    // the IO thread runs at lower priority
    _io_thread_ctx = chThdCreateStatic(_io_thread_wa,
                     sizeof(_io_thread_wa),
                     APM_IO_PRIORITY,        /* Initial priority.      */
                     _io_thread,             /* Thread function.       */
                     this);                  /* Thread parameter.      */

    // the storage thread runs at just above IO priority
    _storage_thread_ctx = chThdCreateStatic(_storage_thread_wa,
                     sizeof(_storage_thread_wa),
                     APM_STORAGE_PRIORITY,        /* Initial priority.      */
                     _storage_thread,             /* Thread function.       */
                     this);                  /* Thread parameter.      */
}

void Scheduler::get_stats(void) {
    thread_t* tp = chRegFirstThread();
    uint32_t idle_time = 0;
    uint32_t total_time = 0;
    uint8_t idx = 0;
    const uint8_t max_tasks = 20;
    static uint64_t last_time[max_tasks];
    uint32_t task_usec[max_tasks];
    const char *names[max_tasks];
    uint8_t prio[max_tasks];
    uint8_t states[max_tasks];
    const char *state_names[] = { CH_STATE_NAMES };

    static uint64_t then = 0;
    uint64_t now = AP_HAL::micros();

    hal.console->printf("%12s %4s %12s %5s %12s\n",
                        "name", "PRIO", "state", "cpu", "ticks");
    do {
        uint64_t cumulative = tp->stats.cumulative;
        uint64_t delta = cumulative - last_time[idx];
        last_time[idx] = cumulative;
        task_usec[idx] = delta;
        names[idx] = tp->name;
        prio[idx] = tp->prio;
        states[idx] = tp->state;
        if (tp->prio == 1)  {
            idle_time = delta;
        }
        total_time += delta;
        tp = chRegNextThread(tp);
        idx++;
    } while ((tp != NULL) && (idx < max_tasks));

    for (uint8_t i=0; i<idx; i++) {
        uint32_t pct = (100*uint64_t(task_usec[i])) / total_time;
        hal.console->printf("%12s %4u %12s %4u%% %12u\n",
                            names[i], (unsigned)prio[i], state_names[states[i]],
                            (unsigned)pct, (unsigned)task_usec[i]);
    }
    
    _busy_percent = 100 * (1.0f - (float)idle_time / total_time);

    uint64_t dt = now - then;
    if (then) {
        hal.console->printf("busy: %2d%%, nirq: %lu tick rate: %.3f MHz\n",
                            _busy_percent, ch.kernel_stats.n_irq, (float)total_time/dt);
    }
    then = now;
}


void Scheduler::delay_microseconds(uint16_t usec)
{
    if (usec == 0) { //chibios faults with 0us sleep
        return;
    }
    uint32_t ticks;
    if (usec >= 4096) {
        // we need to use 64 bit calculations for tick conversions
        ticks = US2ST64(usec);
    } else {
        ticks = US2ST(usec);
    }
    if (ticks == 0) {
        // calling with ticks == 0 causes a hard fault on ChibiOS
        ticks = 1;
    }
    chThdSleep(ticks); //Suspends Thread for desired microseconds
}

/*
  wrapper around sem_post that boosts main thread priority
 */
static void set_high_priority()
{
#if APM_MAIN_PRIORITY_BOOST != APM_MAIN_PRIORITY
    hal_chibios_set_priority(APM_MAIN_PRIORITY_BOOST);
#endif
}

/*
  return the main thread to normal priority
 */
static void set_normal_priority()
{
#if APM_MAIN_PRIORITY_BOOST != APM_MAIN_PRIORITY
    hal_chibios_set_priority(APM_MAIN_PRIORITY);
#endif
}

/*
  a variant of delay_microseconds that boosts priority to
  APM_MAIN_PRIORITY_BOOST for APM_MAIN_PRIORITY_BOOST_USEC
  microseconds when the time completes. This significantly improves
  the regularity of timing of the main loop as it takes
 */
void Scheduler::delay_microseconds_boost(uint16_t usec)
{
    set_high_priority();
    delay_microseconds(usec); //Suspends Thread for desired microseconds
    set_normal_priority();
    _called_boost = true;
}

/*
  return true if delay_microseconds_boost() has been called since last check
 */
bool Scheduler::check_called_boost(void)
{
    if (!_called_boost) {
        return false;
    }
    _called_boost = false;
    return true;
}

void Scheduler::delay(uint16_t ms)
{
    if (!in_main_thread()) {
        //chprintf("ERROR: delay() from timer process\n");
        return;
    }
    uint64_t start = AP_HAL::micros64();

    while ((AP_HAL::micros64() - start)/1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
}

void Scheduler::register_delay_callback(AP_HAL::Proc proc,
                                            uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < CHIBIOS_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < CHIBIOS_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::suspend_timer_procs()
{
    _timer_suspended = true;
}

void Scheduler::resume_timer_procs()
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timers(false);
        _timer_event_missed = false;
    }
}
extern void Reset_Handler();
void Scheduler::reboot(bool hold_in_bootloader)
{
    // disarm motors to ensure they are off during a bootloader upload
    hal.rcout->force_safety_on();
    hal.rcout->force_safety_no_wait();

    // lock all shared DMA channels. This has the effect of waiting
    // till the sensor buses are idle
    Shared_DMA::lock_all();
    
    // delay to ensure the async force_saftey operation completes
    delay(500);

    // disable interrupts during reboot
    chSysDisable();

    // reboot
    NVIC_SystemReset();
}

void Scheduler::_run_timers(bool called_from_timer_thread)
{
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
    } else if (called_from_timer_thread) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    // process analog input
    ((AnalogIn *)hal.analogin)->_timer_tick();

    _in_timer_proc = false;
}

void Scheduler::_timer_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    sched->_timer_thread_ctx->name = "apm_timer";

    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);

        // run registered timers
        sched->_run_timers(true);

        // process any pending RC output requests
        hal.rcout->timer_tick();
    }
}

#if TEST_IDLE
void Scheduler::_test_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    sched->_test_thread_ctx->name = "test_thread";

    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    uint64_t Fnm1 = 0;
    uint64_t Fn = 1;
    while (true) {
        uint64_t tmp = Fn;
        Fn = Fn + Fnm1;
        Fnm1 = tmp;
    }
}
#endif

#if HAL_WITH_UAVCAN
void Scheduler::_uavcan_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    sched->_rcin_thread_ctx->name = "apm_uavcan";
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(20000);
    }
    while (true) {
        sched->delay_microseconds(1000);
        for (int i = 0; i < MAX_NUMBER_OF_CAN_INTERFACES; i++) {
            if(hal.can_mgr[i] != nullptr) {
                CANManager::from(hal.can_mgr[i])->_timer_tick();
            }
        }
    }
}
#endif

void Scheduler::_rcin_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    sched->_rcin_thread_ctx->name = "apm_rcin";
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(20000);
    }
    while (true) {
        sched->delay_microseconds(2500);
        ((RCInput *)hal.rcin)->_timer_tick();
    }
}
#ifdef HAL_PWM_ALARM

void Scheduler::_toneAlarm_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    sched->_toneAlarm_thread_ctx->name = "toneAlarm";
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(20000);
    }
    while (true) {
        sched->delay_microseconds(20000);

        // process tone command
        Util::from(hal.util)->_toneAlarm_timer_tick();
    }
}
#endif
void Scheduler::_run_io(void)
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    if (!_timer_suspended) {
        // now call the IO based drivers
        for (int i = 0; i < _num_io_procs; i++) {
            if (_io_proc[i]) {
                _io_proc[i]();
            }
        }
    }

    _in_io_proc = false;
}

void Scheduler::_io_thread(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    sched->_io_thread_ctx->name = "apm_io";
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);

        // run registered IO processes
        sched->_run_io();
    }
}

void Scheduler::_storage_thread(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    sched->_storage_thread_ctx->name = "apm_storage";
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(10000);
    }
    while (true) {
        sched->delay_microseconds(10000);

        // process any pending storage writes
        hal.storage->_timer_tick();
    }
}

bool Scheduler::in_main_thread() const
{
    return get_main_thread() == chThdGetSelfX();
}

void Scheduler::system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called"
                      "more than once");
    }
    _initialized = true;
}

#endif
