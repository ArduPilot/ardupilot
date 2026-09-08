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
 * Code by @davidbuzz and Claude
 */
#pragma once

#include <AP_HAL/Scheduler.h>
#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
#include <zephyr/kernel.h>
#endif

/* max registered callbacks. */
/*
  ChibiOS gets by with 8 of each because its UARTs run on a dedicated thread.
  This port's UARTs register two timer procs per port, so five active UARTs
  already consume 10 slots - found completely full on hardware 2026-08-08.
 */
#define ZEPHYR_SCHED_MAX_TIMER_PROCS    16
#define ZEPHYR_SCHED_MAX_IO_PROCS       16

/* THREAD STACK SIZES: every one of these DIVERGES FROM ChibiOS, because Zephyr
 * stacks carry the kernel's own frame and guard region on top of AP's usage. */
#define ZEPHYR_TIMER_THREAD_STACK_SZ    4096
#define ZEPHYR_IO_THREAD_STACK_SZ       8192  // bytes
#define ZEPHYR_MONITOR_THREAD_STACK_SZ  2048  // bytes
#define ZEPHYR_RCIN_THREAD_STACK_SZ     4096  // bytes
#define ZEPHYR_RCOUT_THREAD_STACK_SZ    4096  // bytes
#define ZEPHYR_STORAGE_THREAD_STACK_SZ  4096  // bytes

/* User thread pool. ChibiOS has no equivalent constant: its thread_create() takes
 * the stack from the caller, while Zephyr needs it declared up front. */
/* 14 -> 12 on 2026-08-08 to fit DTCM into 5 FlexRAM banks (160 KB) so ITCM
   could take the 11th bank (see the &flexram node in the board DTS). ~7 user
   threads actually run today (3 SPI + 2 I2C buses, log_io, the lazy FTP
   worker), so 12 keeps 5 spare; thread_create() fails loudly, not silently,
   if this ever runs out. */
/* Boards can override both in hwdef.dat (hwdef.h is included ahead of this
   header via board/zephyr.h): the 12 x 8 KB default is sized for the RT1176's
   FlexRAM and overflows small-RAM parts - ESP32-S3's dram0_0_seg overflowed
   by 11.6 KB with it. */
#ifndef ZEPHYR_MAX_USER_THREADS
#define ZEPHYR_MAX_USER_THREADS         12
#endif
#ifndef ZEPHYR_USER_THREAD_STACK_SZ
#define ZEPHYR_USER_THREAD_STACK_SZ     8192  // bytes
#endif

/* Zephyr preemptible thread priorities (lower number = higher priority). */

/* Thread priorities - the AP_HAL_ChibiOS values, translated.
 * AP_HAL_ChibiOS/Scheduler.h is the source of truth; the numbering convention
 * inverts because ChibiOS counts up and Zephyr preemptive counts down.
 * 
 * ChibiOS                        value  ->  Zephyr
 * APM_MONITOR_PRIORITY             183      PREEMPT(0)
 * APM_MAIN_PRIORITY_BOOST          182      PREEMPT(1)
 * APM_TIMER_PRIORITY               181      PREEMPT(2)
 * APM_RCOUT_PRIORITY               181      PREEMPT(2)
 * APM_SPI_PRIORITY                 181      PREEMPT(2)
 * APM_UART_UNBUFFERED_PRIORITY     181      PREEMPT(2)
 * APM_MAIN_PRIORITY                180      PREEMPT(3)
 * APM_CAN_PRIORITY                 178      PREEMPT(4)
 * APM_RCIN_PRIORITY                177      PREEMPT(5)
 * APM_I2C_PRIORITY                 176      PREEMPT(6)
 * APM_LED_PRIORITY                  60      PREEMPT(7)
 * APM_UART_PRIORITY                 60      PREEMPT(7)
 * APM_NET_PRIORITY                  60      PREEMPT(7)
 * APM_STORAGE_PRIORITY              59      PREEMPT(8)
 * APM_IO_PRIORITY                   58      PREEMPT(9)
 * APM_STARTUP_PRIORITY              10      PREEMPT(10)
 * APM_SCRIPTING_PRIORITY       LOWPRIO      PREEMPT(11)
 * SPI(2) sits ABOVE main(3) and I2C(6) BELOW it. That is deliberate in ChibiOS: */
/* READ THIS BEFORE CHANGING A NUMBER: the values below are ranks derived from the
 * ChibiOS table above, not free parameters. Change the rank, not the number. */
/* ORDERING FIX 2026-08-05: these four were inverted against BOTH the ChibiOS
 * source and the table below. */
#define APM_MONITOR_PRIORITY          K_PRIO_PREEMPT(0)   /* most urgent    */
#define APM_MAIN_PRIORITY_BOOST       K_PRIO_PREEMPT(1)   /* MUST beat timer+SPI */
#define APM_TIMER_PRIORITY            K_PRIO_PREEMPT(2)   /* beats main    */
#define APM_RCOUT_PRIORITY            K_PRIO_PREEMPT(4)   /* beats main    */
#define APM_SPI_PRIORITY              K_PRIO_PREEMPT(2)   /* beats main - IMU feeds the loop */
#define APM_UART_UNBUFFERED_PRIORITY  K_PRIO_PREEMPT(2)   /* beats main    */
#define APM_MAIN_PRIORITY             K_PRIO_PREEMPT(8)   /* <== the flight loop */
#define APM_CAN_PRIORITY              K_PRIO_PREEMPT(4)   /* beats main    */
#define APM_RCIN_PRIORITY             K_PRIO_PREEMPT(4)   /* beats main    */
/* BELOW main, matching ChibiOS. This was K_PRIO_PREEMPT(1), SEVEN levels ABOVE
 * main, which starved the main loop. */
#define APM_I2C_PRIORITY              K_PRIO_PREEMPT(9)
/* LED/UART/NET/SCRIPTING: RECONCILED 2026-08-13 to the user-thread values. */
#define APM_LED_PRIORITY              K_PRIO_PREEMPT(14)
#define APM_UART_PRIORITY             K_PRIO_PREEMPT(9)
#define APM_NET_PRIORITY              K_PRIO_PREEMPT(13)
/* Overridable per board (hwdef.dat `define APM_STORAGE_PRIORITY n`). */
#ifndef APM_STORAGE_PRIORITY
#define APM_STORAGE_PRIORITY          K_PRIO_PREEMPT(12)
#define AP_ZEPHYR_STORAGE_BELOW_MAIN 1
#endif
#define APM_IO_PRIORITY               K_PRIO_PREEMPT(5)
#define APM_STARTUP_PRIORITY          K_PRIO_PREEMPT(10)
#define APM_SCRIPTING_PRIORITY        K_PRIO_PREEMPT(13)

/* ORDER, not numbers. Two priority inversions have shipped in this file, both
 * because a number was edited without re-checking its rank against the table. */
static_assert(APM_MONITOR_PRIORITY < APM_MAIN_PRIORITY_BOOST,
              "monitor must outrank the boosted main loop, or it cannot observe a stuck main thread");
static_assert(APM_MAIN_PRIORITY_BOOST < APM_TIMER_PRIORITY,
              "the main-loop boost exists to beat the timer thread (ChibiOS: 182 > 181)");
static_assert(APM_MAIN_PRIORITY_BOOST < APM_SPI_PRIORITY,
              "the main-loop boost exists to beat the SPI bus threads (ChibiOS: 182 > 181)");
static_assert(APM_SPI_PRIORITY < APM_MAIN_PRIORITY,
              "SPI must preempt the loop it feeds (ChibiOS: SPI 181 > main 180)");
static_assert(APM_MAIN_PRIORITY < APM_I2C_PRIORITY,
              "I2C must yield to the flight loop (ChibiOS: main 180 > I2C 176). "
              "Inverting this held the loop at 20 Hz - see the note above APM_I2C_PRIORITY");
/* main sits BELOW rcin/rcout/io on purpose and against ChibiOS - ArduPilot's
   main loop runs at ~100% CPU under Zephyr and never yields to the io thread
   that drains the MAVLink queue. Measured twice; see the block comment above. */
static_assert(APM_IO_PRIORITY < APM_MAIN_PRIORITY,
              "io must preempt main under Zephyr or MAVLink never drains - measured 2026-08-02");
#ifdef AP_ZEPHYR_STORAGE_BELOW_MAIN
static_assert(APM_MAIN_PRIORITY < APM_STORAGE_PRIORITY,
              "storage must yield to the flight loop (ChibiOS: main 180 > storage 59); "
              "work-limited boards override APM_STORAGE_PRIORITY in hwdef.dat instead");
#endif
static_assert(APM_MAIN_PRIORITY < APM_UART_PRIORITY &&
              APM_MAIN_PRIORITY < APM_NET_PRIORITY &&
              APM_MAIN_PRIORITY < APM_LED_PRIORITY &&
              APM_MAIN_PRIORITY < APM_SCRIPTING_PRIORITY,
              "UART/NET/LED/scripting all yield to the flight loop "
              "(ChibiOS: main 180 > all four at <=60)");

/* numeric forms, for the thread_create() offset arithmetic and prj.conf sync */
#define APM_MAIN_PRIORITY_NUM   8

namespace Zephyr {

class Scheduler : public AP_HAL::Scheduler {
public:
    /* AP_HAL::Scheduler interface */
    void init() override;
    void delay(uint16_t ms) override;
    void delay_microseconds(uint16_t us) override;
    void delay_microseconds_boost(uint16_t us) override;
    void boost_end() override;
    void expect_delay_ms(uint32_t ms) override;
    bool in_expected_delay() const override;
    void register_timer_process(AP_HAL::MemberProc) override;
    void register_io_process(AP_HAL::MemberProc) override;
    void register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    void set_system_initialized() override;
    bool is_system_initialized() override;
    void reboot(bool hold_in_bootloader = false) override;
    bool in_main_thread() const override;
    bool thread_create(AP_HAL::MemberProc proc, const char *name,
                       uint32_t stack_size, priority_base base,
                       int8_t priority) override;
    void *disable_interrupts_save(void) override;
    void restore_interrupts(void *) override;

#ifdef CONFIG_AP_DELAY_CB_PROFILE
    /* Throttled summary of delay-callback timings. Measurement only - it
       never alters control flow, so builds with and without the option
       behave identically. See CONFIG_AP_DELAY_CB_PROFILE in zephyr/Kconfig. */
    void profile_report(void);
#endif

    /* extra public API mirroring ChibiOS HAL */
    void watchdog_pat();
    bool check_called_boost();
    void hal_initialized() { _hal_initialized = true; }

    /* Persistent crash/watchdog forensics (ChibiOS parity). */
    void save_persistent_data();
    void restore_persistent_data();

private:
    bool _initialized      = false;
    bool _hal_initialized  = false;
    bool _called_boost     = false;
    bool _priority_boosted = false;
    int  _saved_priority   = 8;   /* mirrors CONFIG_MAIN_THREAD_PRIORITY */

    /* timer callbacks */
    AP_HAL::MemberProc _timer_procs[ZEPHYR_SCHED_MAX_TIMER_PROCS];
    uint8_t            _num_timer_procs = 0;
    volatile bool      _in_timer_proc   = false;

    /* IO callbacks */
    AP_HAL::MemberProc _io_procs[ZEPHYR_SCHED_MAX_IO_PROCS];
    uint8_t            _num_io_procs = 0;
    volatile bool      _in_io_proc   = false;

    /* failsafe */
    AP_HAL::Proc _failsafe_proc      = nullptr;
    uint32_t     _failsafe_period_us = 0;
    uint64_t     _last_failsafe_us   = 0;

    /* watchdog / expected-delay state */
    volatile uint32_t _last_watchdog_pat_ms = 0;
    uint32_t          _expect_delay_start   = 0;
    uint32_t          _expect_delay_length  = 0;
    uint32_t          _expect_delay_nesting = 0;

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
    k_tid_t _main_tid = nullptr;

    /* internal thread control blocks */
    struct k_thread _timer_thread_data;
    struct k_thread _io_thread_data;
    struct k_thread _monitor_thread_data;
    struct k_thread _rcin_thread_data;
    struct k_thread _rcout_thread_data;
    struct k_thread _storage_thread_data;

    /* user thread pool */
    struct UserThread {
        struct k_thread    thread_data;
        AP_HAL::MemberProc proc;
        bool               in_use;
    };
    UserThread _user_threads[ZEPHYR_MAX_USER_THREADS];

    /* internal thread functions */
    void _run_timer_procs();
    void _run_io();
    static void _timer_thread_fn(void *arg, void *, void *);
    static void _io_thread_fn(void *arg, void *, void *);
    static void _monitor_thread_fn(void *arg, void *, void *);
    static void _rcin_thread_fn(void *arg, void *, void *);
    static void _rcout_thread_fn(void *arg, void *, void *);
    static void _storage_thread_fn(void *arg, void *, void *);
    static void _user_thread_fn(void *arg, void *, void *);
    static int  _zephyr_priority(priority_base base, int8_t offset);

    /* health monitoring */
    void check_stack_free();
    static void try_force_mutex();
#endif
};

}  // namespace Zephyr
