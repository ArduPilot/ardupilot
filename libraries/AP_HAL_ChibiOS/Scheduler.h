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
#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "AP_HAL_ChibiOS_Namespace.h"

#define CHIBIOS_SCHEDULER_MAX_TIMER_PROCS 8

#define APM_MONITOR_PRIORITY    184
#define APM_MAIN_PRIORITY       180
#define APM_TIMER_PRIORITY      181
#define APM_RCOUT_PRIORITY      183
#define APM_LED_PRIORITY         60
#define APM_UART_PRIORITY        60
#define APM_NET_PRIORITY         60
#define APM_UART_UNBUFFERED_PRIORITY 181
#define APM_STORAGE_PRIORITY     59
#define APM_IO_PRIORITY          58
#define APM_STARTUP_PRIORITY     10
#define APM_SCRIPTING_PRIORITY  LOWPRIO

/*
  boost priority handling
 */
#ifndef APM_MAIN_PRIORITY_BOOST
#define APM_MAIN_PRIORITY_BOOST 182
#endif

#ifndef APM_RCIN_PRIORITY
#define APM_RCIN_PRIORITY      177
#endif

#ifndef APM_SPI_PRIORITY
// SPI priority needs to be above main priority to ensure fast sampling of IMUs can keep up
// with the data rate
#define APM_SPI_PRIORITY        181
#endif

#ifndef APM_CAN_PRIORITY
#define APM_CAN_PRIORITY        178
#endif

#ifndef APM_I2C_PRIORITY
#define APM_I2C_PRIORITY        176
#endif

#ifndef TIMER_THD_WA_SIZE
#define TIMER_THD_WA_SIZE   1536
#endif

#ifndef RCOUT_THD_WA_SIZE
#define RCOUT_THD_WA_SIZE    512
#endif

#ifndef RCIN_THD_WA_SIZE
#define RCIN_THD_WA_SIZE    1024
#endif

#ifndef IO_THD_WA_SIZE
#define IO_THD_WA_SIZE      2048
#endif

#ifndef STORAGE_THD_WA_SIZE
#define STORAGE_THD_WA_SIZE 1024
#endif

#ifndef MONITOR_THD_WA_SIZE
#define MONITOR_THD_WA_SIZE 1024
#endif

/* Scheduler implementation: */
class ChibiOS::Scheduler : public AP_HAL::Scheduler {
public:
    Scheduler();
    /* AP_HAL::Scheduler methods */


    void     init() override;
    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;
    void     delay_microseconds_boost(uint16_t us) override;
    void     boost_end(void) override;
    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    void     reboot(bool hold_in_bootloader) override;

    bool     in_main_thread() const override { return get_main_thread() == chThdGetSelfX(); }

    void     set_system_initialized() override;
    bool     is_system_initialized() override { return _initialized; };
    void     hal_initialized() { _hal_initialized = true; }

    bool     check_called_boost(void);

    /*
      inform the scheduler that we are calling an operation from the
      main thread that may take an extended amount of time. This can
      be used to prevent watchdog reset during expected long delays
      A value of zero cancels the previous expected delay
     */
    void     _expect_delay_ms(uint32_t ms);
    void     expect_delay_ms(uint32_t ms) override;

    /*
      return true if we are in a period of expected delay. This can be
      used to suppress error messages
     */
    bool in_expected_delay(void) const override;
    
    /*
      disable interrupts and return a context that can be used to
      restore the interrupt state. This can be used to protect
      critical regions
     */
    void *disable_interrupts_save(void) override;

    /*
      restore interrupt state from disable_interrupts_save()
     */
    void restore_interrupts(void *) override;

    /*
      create a new thread
     */
    bool thread_create(AP_HAL::MemberProc, const char *name, uint32_t stack_size, priority_base base, int8_t priority) override;

    // pat the watchdog
    void watchdog_pat(void);

private:
    bool _initialized;
    volatile bool _hal_initialized;
    AP_HAL::Proc _failsafe;
    bool _called_boost;
    bool _priority_boosted;
    uint32_t expect_delay_start;
    uint32_t expect_delay_length;
    uint32_t expect_delay_nesting;
    HAL_Semaphore expect_delay_sem;

    AP_HAL::MemberProc _timer_proc[CHIBIOS_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    volatile bool _in_timer_proc;

    AP_HAL::MemberProc _io_proc[CHIBIOS_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_io_procs;
    volatile bool _in_io_proc;
    uint32_t last_watchdog_pat_ms;

    thread_t* _timer_thread_ctx;
    thread_t* _rcout_thread_ctx;
    thread_t* _rcin_thread_ctx;
    thread_t* _io_thread_ctx;
    thread_t* _storage_thread_ctx;
    thread_t* _monitor_thread_ctx;

#if CH_CFG_USE_SEMAPHORES == TRUE
    binary_semaphore_t _timer_semaphore;
    binary_semaphore_t _io_semaphore;
#endif

    // calculates an integer to be used as the priority for a newly-created thread
    uint8_t calculate_thread_priority(priority_base base, int8_t priority) const;

    static void _timer_thread(void *arg);
    static void _rcout_thread(void *arg);
    static void _rcin_thread(void *arg);
    static void _io_thread(void *arg);
    static void _storage_thread(void *arg);
    static void _uart_thread(void *arg);
    static void _monitor_thread(void *arg);

    void _run_timers();
    void _run_io(void);
    static void thread_create_trampoline(void *ctx);

#if defined STM32H7
    void check_low_memory_is_zero();
#endif

    // check for free stack space
    void check_stack_free(void);

#if defined(HAL_GPIO_PIN_EXT_WDOG)
    // external watchdog GPIO support
    void ext_watchdog_pat(uint32_t now_ms);
    uint32_t last_ext_watchdog_ms;
#endif

    static void try_force_mutex(void);
};
#endif
