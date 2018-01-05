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

#include <AP_HAL_ChibiOS/UARTDriver.h>
#include <AP_HAL_ChibiOS/AnalogIn.h>
#include <AP_HAL_ChibiOS/Storage.h>
#include <AP_HAL_ChibiOS/RCOutput.h>
#include <AP_HAL_ChibiOS/RCInput.h>

#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;
THD_WORKING_AREA(_timer_thread_wa, 2048);
THD_WORKING_AREA(_io_thread_wa, 2048);
THD_WORKING_AREA(_storage_thread_wa, 2048);
THD_WORKING_AREA(_uart_thread_wa, 2048);

#if HAL_WITH_IO_MCU
extern ChibiOS::ChibiUARTDriver uart_io;
#endif

ChibiScheduler::ChibiScheduler()
{}

void ChibiScheduler::init()
{
    // setup the timer thread - this will call tasks at 1kHz
    _timer_thread_ctx = chThdCreateStatic(_timer_thread_wa,
                     sizeof(_timer_thread_wa),
                     APM_TIMER_PRIORITY,        /* Initial priority.    */
                     _timer_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */

    // the UART thread runs at a medium priority
    _uart_thread_ctx = chThdCreateStatic(_uart_thread_wa,
                     sizeof(_uart_thread_wa),
                     APM_UART_PRIORITY,        /* Initial priority.    */
                     _uart_thread,             /* Thread function.     */
                     this);                    /* Thread parameter.    */

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

void ChibiScheduler::delay_microseconds(uint16_t usec)
{
    if (usec == 0) { //chibios faults with 0us sleep
        return;
    }
    chThdSleepMicroseconds(usec); //Suspends Thread for desired microseconds
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
void ChibiScheduler::delay_microseconds_boost(uint16_t usec)
{
    delay_microseconds(usec); //Suspends Thread for desired microseconds
    set_high_priority();
    delay_microseconds(APM_MAIN_PRIORITY_BOOST_USEC);
    set_normal_priority();
}

void ChibiScheduler::delay(uint16_t ms)
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

void ChibiScheduler::register_delay_callback(AP_HAL::Proc proc,
                                            uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void ChibiScheduler::register_timer_process(AP_HAL::MemberProc proc)
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

void ChibiScheduler::register_io_process(AP_HAL::MemberProc proc)
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

void ChibiScheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void ChibiScheduler::suspend_timer_procs()
{
    _timer_suspended = true;
}

void ChibiScheduler::resume_timer_procs()
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timers(false);
        _timer_event_missed = false;
    }
}
extern void Reset_Handler();
void ChibiScheduler::reboot(bool hold_in_bootloader)
{
    // disarm motors to ensure they are off during a bootloader upload
    hal.rcout->force_safety_on();
    hal.rcout->force_safety_no_wait();

    // delay to ensure the async force_saftey operation completes
    delay(500);

    // disable interrupts during reboot
    chSysDisable();

    // reboot
    NVIC_SystemReset();
}

void ChibiScheduler::_run_timers(bool called_from_timer_thread)
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
    ((ChibiAnalogIn *)hal.analogin)->_timer_tick();

    _in_timer_proc = false;
}

void ChibiScheduler::_timer_thread(void *arg)
{
    ChibiScheduler *sched = (ChibiScheduler *)arg;
    sched->_timer_thread_ctx->name = "apm_timer";

    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);

        // run registered timers
        sched->_run_timers(true);

        // process any pending RC output requests
        //hal.rcout->timer_tick();

        // process any pending RC input requests
        ((ChibiRCInput *)hal.rcin)->_timer_tick();
    }
}

void ChibiScheduler::_run_io(void)
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

void ChibiScheduler::_uart_thread(void* arg)
{
    ChibiScheduler *sched = (ChibiScheduler *)arg;
    sched->_uart_thread_ctx->name = "apm_uart";
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);

        // process any pending serial bytes
        ((ChibiUARTDriver *)hal.uartA)->_timer_tick();
        ((ChibiUARTDriver *)hal.uartB)->_timer_tick();
        ((ChibiUARTDriver *)hal.uartC)->_timer_tick();
        /*((ChibiUARTDriver *)hal.uartD)->_timer_tick();
        ((ChibiUARTDriver *)hal.uartE)->_timer_tick();
        ((ChibiUARTDriver *)hal.uartF)->_timer_tick();*/
#if HAL_WITH_IO_MCU
        uart_io._timer_tick();
#endif
    }
}

void ChibiScheduler::_io_thread(void* arg)
{
    ChibiScheduler *sched = (ChibiScheduler *)arg;
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

void ChibiScheduler::_storage_thread(void* arg)
{
    ChibiScheduler *sched = (ChibiScheduler *)arg;
    sched->_storage_thread_ctx->name = "apm_storage";
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(10000);
    }
    while (true) {
        sched->delay_microseconds(10000);

        // process any pending storage writes
        ((ChibiStorage *)hal.storage)->_timer_tick();
    }
}

bool ChibiScheduler::in_main_thread() const
{
    return get_main_thread() == chThdGetSelfX();
}

void ChibiScheduler::system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called"
                      "more than once");
    }
    _initialized = true;
}

#endif
