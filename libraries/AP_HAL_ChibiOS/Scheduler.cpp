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

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SCHEDULER_ENABLED
#define HAL_SCHEDULER_ENABLED 1
#endif

#if HAL_SCHEDULER_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <hal.h>
#include "AP_HAL_ChibiOS.h"
#include "Scheduler.h"
#include "Util.h"
#include "GPIO.h"

#include <AP_HAL_ChibiOS/UARTDriver.h>
#include <AP_HAL_ChibiOS/AnalogIn.h>
#include <AP_HAL_ChibiOS/Storage.h>
#include <AP_HAL_ChibiOS/RCOutput.h>
#include <AP_HAL_ChibiOS/RCInput.h>
#include <AP_HAL_ChibiOS/CANIface.h>
#include <AP_InternalError/AP_InternalError.h>

#if CH_CFG_USE_DYNAMIC == TRUE

#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "hwdef/common/stm32_util.h"
#include "hwdef/common/flash.h"
#include "hwdef/common/watchdog.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include "shared_dma.h"
#include <AP_Common/ExpandingString.h>

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;
#ifndef HAL_NO_TIMER_THREAD
THD_WORKING_AREA(_timer_thread_wa, TIMER_THD_WA_SIZE);
#endif
#ifndef HAL_NO_RCOUT_THREAD
THD_WORKING_AREA(_rcout_thread_wa, RCOUT_THD_WA_SIZE);
#endif
#ifndef HAL_NO_RCIN_THREAD
THD_WORKING_AREA(_rcin_thread_wa, RCIN_THD_WA_SIZE);
#endif
#ifndef HAL_USE_EMPTY_IO
THD_WORKING_AREA(_io_thread_wa, IO_THD_WA_SIZE);
#endif
#ifndef HAL_USE_EMPTY_STORAGE
THD_WORKING_AREA(_storage_thread_wa, STORAGE_THD_WA_SIZE);
#endif
#ifndef HAL_NO_MONITOR_THREAD
THD_WORKING_AREA(_monitor_thread_wa, MONITOR_THD_WA_SIZE);
#endif

Scheduler::Scheduler()
{
}

void Scheduler::init()
{
    chBSemObjectInit(&_timer_semaphore, false);
    chBSemObjectInit(&_io_semaphore, false);

#ifndef HAL_NO_MONITOR_THREAD
    // setup the monitor thread - this is used to detect software lockups
    _monitor_thread_ctx = chThdCreateStatic(_monitor_thread_wa,
                     sizeof(_monitor_thread_wa),
                     APM_MONITOR_PRIORITY,        /* Initial priority.    */
                     _monitor_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
#endif

#ifndef HAL_NO_TIMER_THREAD
    // setup the timer thread - this will call tasks at 1kHz
    _timer_thread_ctx = chThdCreateStatic(_timer_thread_wa,
                     sizeof(_timer_thread_wa),
                     APM_TIMER_PRIORITY,        /* Initial priority.    */
                     _timer_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
#endif

#ifndef HAL_NO_RCOUT_THREAD
    // setup the RCOUT thread - this will call tasks at 1kHz
    _rcout_thread_ctx = chThdCreateStatic(_rcout_thread_wa,
                     sizeof(_rcout_thread_wa),
                     APM_RCOUT_PRIORITY,        /* Initial priority.    */
                     _rcout_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
#endif

#ifndef HAL_NO_RCIN_THREAD
    // setup the RCIN thread - this will call tasks at 1kHz
    _rcin_thread_ctx = chThdCreateStatic(_rcin_thread_wa,
                     sizeof(_rcin_thread_wa),
                     APM_RCIN_PRIORITY,        /* Initial priority.    */
                     _rcin_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
#endif
#ifndef HAL_USE_EMPTY_IO
    // the IO thread runs at lower priority
    _io_thread_ctx = chThdCreateStatic(_io_thread_wa,
                     sizeof(_io_thread_wa),
                     APM_IO_PRIORITY,        /* Initial priority.      */
                     _io_thread,             /* Thread function.       */
                     this);                  /* Thread parameter.      */
#endif

#ifndef HAL_USE_EMPTY_STORAGE
    // the storage thread runs at just above IO priority
    _storage_thread_ctx = chThdCreateStatic(_storage_thread_wa,
                     sizeof(_storage_thread_wa),
                     APM_STORAGE_PRIORITY,        /* Initial priority.      */
                     _storage_thread,             /* Thread function.       */
                     this);                  /* Thread parameter.      */
#endif

}

void Scheduler::delay_microseconds(uint16_t usec)
{
    if (usec == 0) { //chibios faults with 0us sleep
        return;
    }
    uint32_t ticks;
    ticks = chTimeUS2I(usec);
    if (ticks == 0) {
        // calling with ticks == 0 causes a hard fault on ChibiOS
        ticks = 1;
    }
    chThdSleep(MAX(ticks,CH_CFG_ST_TIMEDELTA)); //Suspends Thread for desired microseconds
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
void Scheduler::boost_end(void)
{
#if APM_MAIN_PRIORITY_BOOST != APM_MAIN_PRIORITY
    if (in_main_thread() && _priority_boosted) {
        _priority_boosted = false;
        hal_chibios_set_priority(APM_MAIN_PRIORITY);
    }
#endif
}

/*
  a variant of delay_microseconds that boosts priority to
  APM_MAIN_PRIORITY_BOOST for APM_MAIN_PRIORITY_BOOST_USEC
  microseconds when the time completes. This significantly improves
  the regularity of timing of the main loop
 */
void Scheduler::delay_microseconds_boost(uint16_t usec)
{
    if (!_priority_boosted && in_main_thread()) {
        set_high_priority();
        _priority_boosted = true;
        _called_boost = true;
    }
    delay_microseconds(usec); //Suspends Thread for desired microseconds
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
    uint64_t start = AP_HAL::micros64();

    while ((AP_HAL::micros64() - start)/1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            if (in_main_thread()) {
                call_delay_cb();
            }
        }
    }
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    chBSemWait(&_timer_semaphore);
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            chBSemSignal(&_timer_semaphore);
            return;
        }
    }

    if (_num_timer_procs < CHIBIOS_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        DEV_PRINTF("Out of timer processes\n");
    }
    chBSemSignal(&_timer_semaphore);
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    chBSemWait(&_io_semaphore);
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            chBSemSignal(&_io_semaphore);
            return;
        }
    }

    if (_num_io_procs < CHIBIOS_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        DEV_PRINTF("Out of IO processes\n");
    }
    chBSemSignal(&_io_semaphore);
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    // disarm motors to ensure they are off during a bootloader upload
    hal.rcout->force_safety_on();

#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled()) {
        iomcu.shutdown();
    }
#endif

#if HAL_LOGGING_ENABLED
    //stop logging
    if (AP_Logger::get_singleton()) {
        AP::logger().StopLogging();
    }

    // unmount filesystem, if active
    AP::FS().unmount();
#endif

#if AP_FASTBOOT_ENABLED
    // setup RTC for fast reboot
    set_fast_reboot(hold_in_bootloader?RTC_BOOT_HOLD:RTC_BOOT_FAST);
#endif

    // disable all interrupt sources
    port_disable();

    // reboot
    NVIC_SystemReset();
}

void Scheduler::_run_timers()
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    int num_procs = 0;
    chBSemWait(&_timer_semaphore);
    num_procs = _num_timer_procs;
    chBSemSignal(&_timer_semaphore);
    // now call the timer based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

#if HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)
    // process analog input
    ((AnalogIn *)hal.analogin)->_timer_tick();
#endif

    _in_timer_proc = false;
}

void Scheduler::_timer_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    chRegSetThreadName("timer");

    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);

        // run registered timers
        sched->_run_timers();

        if (sched->in_expected_delay()) {
            sched->watchdog_pat();
        }
    }
}

void Scheduler::_rcout_thread(void *arg)
{
#ifndef HAL_NO_RCOUT_THREAD
    Scheduler *sched = (Scheduler *)arg;
    chRegSetThreadName("rcout");

    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
#if HAL_USE_PWM == TRUE
    // trampoline into the rcout thread
    ((RCOutput*)hal.rcout)->rcout_thread();
#endif
#endif
}

/*
  return true if we are in a period of expected delay. This can be
  used to suppress error messages
*/
bool Scheduler::in_expected_delay(void) const
{
    if (!_initialized) {
        // until setup() is complete we expect delays
        return true;
    }
    if (expect_delay_start != 0) {
        uint32_t now = AP_HAL::millis();
        if (now - expect_delay_start <= expect_delay_length) {
            return true;
        }
    }
#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_BOOTLOADER_BUILD)
    if (stm32_flash_recent_erase()) {
        return true;
    }
#endif
    return false;
}

#ifndef HAL_NO_MONITOR_THREAD
void Scheduler::_monitor_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    chRegSetThreadName("monitor");

    while (!sched->_initialized) {
        sched->delay(100);
    }
    bool using_watchdog = AP_BoardConfig::watchdog_enabled();
#if HAL_LOGGING_ENABLED
    uint8_t log_wd_counter = 0;
#endif

    while (true) {
        sched->delay(100);
        if (using_watchdog) {
            stm32_watchdog_save((uint32_t *)&hal.util->persistent_data, (sizeof(hal.util->persistent_data)+3)/4);
        }

        // if running memory guard then check all allocations
        malloc_check(nullptr);

        uint32_t now = AP_HAL::millis();
        uint32_t loop_delay = now - sched->last_watchdog_pat_ms;
        if (loop_delay >= 200) {
            // the main loop has been stuck for at least
            // 200ms. Starting logging the main loop state
#if HAL_LOGGING_ENABLED
            const AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
            if (AP_Logger::get_singleton()) {
                const struct log_MON mon{
                    LOG_PACKET_HEADER_INIT(LOG_MON_MSG),
                    time_us               : AP_HAL::micros64(),
                    loop_delay            : loop_delay,
                    current_task          : pd.scheduler_task,
                    internal_error_mask   : pd.internal_errors,
                    internal_error_count  : pd.internal_error_count,
                    internal_error_line   : pd.internal_error_last_line,
                    mavmsg                : pd.last_mavlink_msgid,
                    mavcmd                : pd.last_mavlink_cmd,
                    semline               : pd.semaphore_line,
                    spicnt                : pd.spi_count,
                    i2ccnt                : pd.i2c_count
                };
                AP::logger().WriteCriticalBlock(&mon, sizeof(mon));
            }
#endif
        }
        if (loop_delay >= 500 && !sched->in_expected_delay()) {
            // at 500ms we declare an internal error
            INTERNAL_ERROR(AP_InternalError::error_t::main_loop_stuck);
        }

#if HAL_LOGGING_ENABLED
    if (log_wd_counter++ == 10 && hal.util->was_watchdog_reset()) {
        log_wd_counter = 0;
        // log watchdog message once a second
        const AP_HAL::Util::PersistentData &pd = hal.util->last_persistent_data;
        struct log_WDOG wdog{
            LOG_PACKET_HEADER_INIT(LOG_WDOG_MSG),
            time_us                  : AP_HAL::micros64(),
            scheduler_task           : pd.scheduler_task,
            internal_errors          : pd.internal_errors,
            internal_error_count     : pd.internal_error_count,
            internal_error_last_line : pd.internal_error_last_line,
            last_mavlink_msgid       : pd.last_mavlink_msgid,
            last_mavlink_cmd         : pd.last_mavlink_cmd,
            semaphore_line           : pd.semaphore_line,
            fault_line               : pd.fault_line,
            fault_type               : pd.fault_type,
            fault_addr               : pd.fault_addr,
            fault_thd_prio           : pd.fault_thd_prio,
            fault_icsr               : pd.fault_icsr,
            fault_lr                 : pd.fault_lr
        };
        memcpy(wdog.thread_name4, pd.thread_name4, ARRAY_SIZE(wdog.thread_name4));

        AP::logger().WriteCriticalBlock(&wdog, sizeof(wdog));
    }
#endif // HAL_LOGGING_ENABLED

#ifndef IOMCU_FW
    // setup GPIO interrupt quotas
    hal.gpio->timer_tick();
#endif
    }
}
#endif // HAL_NO_MONITOR_THREAD

void Scheduler::_rcin_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    chRegSetThreadName("rcin");
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(20000);
    }
    while (true) {
        sched->delay_microseconds(1000);
        ((RCInput *)hal.rcin)->_timer_tick();
    }
}

void Scheduler::_run_io(void)
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    int num_procs = 0;
    chBSemWait(&_io_semaphore);
    num_procs = _num_io_procs;
    chBSemSignal(&_io_semaphore);
    // now call the IO based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }

    _in_io_proc = false;
}

void Scheduler::_io_thread(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    chRegSetThreadName("io");
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
#if HAL_LOGGING_ENABLED
    uint32_t last_sd_start_ms = AP_HAL::millis();
#endif
#if CH_DBG_ENABLE_STACK_CHECK == TRUE
    uint32_t last_stack_check_ms = 0;
#endif
    while (true) {
        sched->delay_microseconds(1000);

        // run registered IO processes
        sched->_run_io();

#if HAL_LOGGING_ENABLED || CH_DBG_ENABLE_STACK_CHECK == TRUE
        uint32_t now = AP_HAL::millis();
#endif

#if HAL_LOGGING_ENABLED
        if (!hal.util->get_soft_armed()) {
            // if sdcard hasn't mounted then retry it every 3s in the IO
            // thread when disarmed
            if (now - last_sd_start_ms > 3000) {
                last_sd_start_ms = now;
                AP::FS().retry_mount();
            }
        }
#endif
#if CH_DBG_ENABLE_STACK_CHECK == TRUE
        if (now - last_stack_check_ms > 1000) {
            last_stack_check_ms = now;
            sched->check_stack_free();
        }
#endif
    }
}

#if defined(STM32H7)
/*
  the H7 has 64k of ITCM memory at address zero. We reserve 1k of it
  to prevent nullptr being valid. This function checks that memory is
  always zero
 */
void Scheduler::check_low_memory_is_zero()
{
    const uint32_t *lowmem = nullptr;
    // we start at address 0x1 as reading address zero causes a fault
    for (uint16_t i=1; i<256; i++) {
        if (lowmem[i] != 0) {
            // re-use memory guard internal error
            AP_memory_guard_error(1023);
            break;
        }
    }
    // we can't do address 0, but can check next 3 bytes
    const uint8_t *addr0 = (const uint8_t *)0;
    for (uint8_t i=1; i<4; i++) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
        if (addr0[i] != 0) {
            AP_memory_guard_error(1023);
            break;
        }
#pragma GCC diagnostic pop
    }
}
#endif // STM32H7

void Scheduler::_storage_thread(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    chRegSetThreadName("storage");
    while (!sched->_hal_initialized) {
        sched->delay_microseconds(10000);
    }
#if defined STM32H7
    uint16_t memcheck_counter=0;
#endif
    while (true) {
        sched->delay_microseconds(1000);

        // process any pending storage writes
        hal.storage->_timer_tick();

#if defined STM32H7
        if (memcheck_counter++ % 500 == 0) {
            // run check at 2Hz
            sched->check_low_memory_is_zero();
        }
#endif
    }
}

void Scheduler::set_system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::set_system_initialized called"
                      "more than once");
    }
    _initialized = true;
}

/*
  disable interrupts and return a context that can be used to
  restore the interrupt state. This can be used to protect
  critical regions
*/
void *Scheduler::disable_interrupts_save(void)
{
    return (void *)(uintptr_t)chSysGetStatusAndLockX();
}

/*
  restore interrupt state from disable_interrupts_save()
*/
void Scheduler::restore_interrupts(void *state)
{
    chSysRestoreStatusX((syssts_t)(uintptr_t)state);
}

/*
  trampoline for thread create
*/
void Scheduler::thread_create_trampoline(void *ctx)
{
    AP_HAL::MemberProc *t = (AP_HAL::MemberProc *)ctx;
    (*t)();
    free(t);
}

// calculates an integer to be used as the priority for a newly-created thread
uint8_t Scheduler::calculate_thread_priority(priority_base base, int8_t priority) const
{
    uint8_t thread_priority = APM_IO_PRIORITY;
    static const struct {
        priority_base base;
        uint8_t p;
    } priority_map[] = {
        { PRIORITY_BOOST, APM_MAIN_PRIORITY_BOOST},
        { PRIORITY_MAIN, APM_MAIN_PRIORITY},
        { PRIORITY_SPI, APM_SPI_PRIORITY},
        { PRIORITY_I2C, APM_I2C_PRIORITY},
        { PRIORITY_CAN, APM_CAN_PRIORITY},
        { PRIORITY_TIMER, APM_TIMER_PRIORITY},
        { PRIORITY_RCOUT, APM_RCOUT_PRIORITY},
        { PRIORITY_RCIN, APM_RCIN_PRIORITY},
        { PRIORITY_IO, APM_IO_PRIORITY},
        { PRIORITY_UART, APM_UART_PRIORITY},
        { PRIORITY_STORAGE, APM_STORAGE_PRIORITY},
        { PRIORITY_SCRIPTING, APM_SCRIPTING_PRIORITY},
    };
    for (uint8_t i=0; i<ARRAY_SIZE(priority_map); i++) {
        if (priority_map[i].base == base) {
            thread_priority = constrain_int16(priority_map[i].p + priority, LOWPRIO, HIGHPRIO);
            break;
        }
    }
    return thread_priority;
}

/*
  create a new thread
*/
bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name, uint32_t stack_size, priority_base base, int8_t priority)
{
    // take a copy of the MemberProc, it is freed after thread exits
    AP_HAL::MemberProc *tproc = (AP_HAL::MemberProc *)malloc(sizeof(proc));
    if (!tproc) {
        return false;
    }
    *tproc = proc;

    const uint8_t thread_priority = calculate_thread_priority(base, priority);

    thread_t *thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(stack_size),
                                               name,
                                               thread_priority,
                                               thread_create_trampoline,
                                               tproc);
    if (thread_ctx == nullptr) {
        free(tproc);
        return false;
    }
    return true;
}

/*
  inform the scheduler that we are calling an operation from the
  main thread that may take an extended amount of time. This can
  be used to prevent watchdog reset during expected long delays
  A value of zero cancels the previous expected delay
*/
void Scheduler::_expect_delay_ms(uint32_t ms)
{
    if (!in_main_thread()) {
        // only for main thread
        return;
    }

    // pat once immediately
    watchdog_pat();

    WITH_SEMAPHORE(expect_delay_sem);

    if (ms == 0) {
        if (expect_delay_nesting > 0) {
            expect_delay_nesting--;
        }
        if (expect_delay_nesting == 0) {
            expect_delay_start = 0;
        }
    } else {
        uint32_t now = AP_HAL::millis();
        if (expect_delay_start != 0) {
            // we already have a delay running, possibly extend it
            uint32_t done = now - expect_delay_start;
            if (expect_delay_length > done) {
                ms = MAX(ms, expect_delay_length - done);
            }
        }
        expect_delay_start = now;
        expect_delay_length = ms;
        expect_delay_nesting++;

        // also put our priority below timer thread if we are boosted
        boost_end();
    }
}

/*
  this is _expect_delay_ms() with check that we are in the main thread
 */
void Scheduler::expect_delay_ms(uint32_t ms)
{
    if (!in_main_thread()) {
        // only for main thread
        return;
    }
    _expect_delay_ms(ms);
}

// pat the watchdog
void Scheduler::watchdog_pat(void)
{
    stm32_watchdog_pat();
    last_watchdog_pat_ms = AP_HAL::millis();
}

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
/*
  check we have enough stack free on all threads and the ISR stack
 */
void Scheduler::check_stack_free(void)
{
    // we raise an internal error stack_overflow when the available
    // stack on any thread or the ISR stack drops below this
    // threshold. This means we get an overflow error when we haven't
    // yet completely run out of stack. This gives us a good
    // pre-warning when we are getting too close
#if defined(STM32F1)
    const uint32_t min_stack = 32;
#else
    const uint32_t min_stack = 64;
#endif

    if (stack_free(&__main_stack_base__) < min_stack) {
        // use "line number" of 0xFFFF for ISR stack low
        AP::internalerror().error(AP_InternalError::error_t::stack_overflow, 0xFFFF);
    }

    for (thread_t *tp = chRegFirstThread(); tp; tp = chRegNextThread(tp)) {
        if (stack_free(tp->wabase) < min_stack) {
            // use task priority for line number. This allows us to
            // identify the task fairly reliably
            AP::internalerror().error(AP_InternalError::error_t::stack_overflow, tp->realprio);
        }
    }
}
#endif // CH_DBG_ENABLE_STACK_CHECK == TRUE

#endif // CH_CFG_USE_DYNAMIC

#endif  // HAL_SCHEDULER_ENABLED
