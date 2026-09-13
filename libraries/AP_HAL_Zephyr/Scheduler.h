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

/* Thread priorities: AP_HAL_ChibiOS/Scheduler.h:25-61 translated by RANK.
 * ChibiOS counts UP to more urgent (HIGHPRIO 255); Zephyr's preemptible band
 * counts DOWN (K_PRIO_PREEMPT(0) is the most urgent). Every ChibiOS "+1" is a
 * "-1" here, and _zephyr_priority() applies that sign exactly once. Levels are
 * contiguous where ChibiOS's are (183..176, 61..57) so a caller's +/-1 offset
 * lands on the same neighbour it does on ChibiOS. A NEGATIVE value is a
 * cooperative thread that nothing can preempt: the asserts refuse it.
 *
 *  ChibiOS  role                         Zephyr  who runs there
 *   183     MONITOR (HIGHPRIO clamp)       0     AP_monitor; user BOOST+1 (IOMCU)
 *   182     MAIN_BOOST                     1     main during the INS wait; RCOUT+1 (rate thread)
 *   181     TIMER/RCOUT/SPI/UART_UNBUF     2     AP_timer, AP_rcout, SPIn bus threads
 *   180     MAIN                           3     main (== CONFIG_MAIN_THREAD_PRIORITY)
 *   179     -                              4     free (MAIN-1, CAN+1)
 *   178     CAN                            5     DroneCAN/CANSensor user threads
 *   177     RCIN                           6     AP_rcin
 *   176     I2C                            7     I2Cn bus threads
 *    61     NET+1                          8     lwip tcpip, if built
 *    60     LED/UART/NET                   9     LED/UART/NET user threads; Zephyr logging, mcumgr
 *    59     STORAGE                       10     AP_storage; IO+1 (log_io)
 *    58     IO                            11     AP_io; IO+0; unknown-base default
 *    57     IO-1                          12     IO-1 user threads
 *    10     STARTUP                       13     main during setup(), until the first INS wait
 *     2     SCRIPTING (LOWPRIO clamp)     14     scripting; floor for user threads
 *     1     IDLEPRIO                      15     Zephyr idle
 *
 * The three moves that matter against the ladder this replaced (2026-09-12):
 * main 8 -> 3 with rcin and CAN now BELOW it (ChibiOS 177/178 < 180), io 5 -> 11
 * (ChibiOS 58 < 180; the "temporarily above main" of TODO 2.29 is retired, with a
 * hwdef hatch), and the boost is real again (see Scheduler.cpp set_main_priority).
 */
#define APM_HIGHPRIO                  K_PRIO_PREEMPT(0)                 /* ChibiOS HIGHPRIO: clamp ceiling */
#define APM_LOWPRIO                   K_LOWEST_APPLICATION_THREAD_PRIO  /* ChibiOS LOWPRIO: clamp floor (14) */

#define APM_MONITOR_PRIORITY          K_PRIO_PREEMPT(0)
/* Overridable per board, as on ChibiOS. A hwdef that defines
   APM_MAIN_PRIORITY_BOOST equal to APM_MAIN_PRIORITY compiles the boost out,
   exactly as ChibiOS does when the two are equal. */
#ifndef APM_MAIN_PRIORITY_BOOST
#define APM_MAIN_PRIORITY_BOOST       K_PRIO_PREEMPT(1)
#endif
#define APM_TIMER_PRIORITY            K_PRIO_PREEMPT(2)
#define APM_RCOUT_PRIORITY            K_PRIO_PREEMPT(2)
#ifndef APM_SPI_PRIORITY
#define APM_SPI_PRIORITY              K_PRIO_PREEMPT(2)   /* above main: the IMU feeds the loop */
#endif
#define APM_UART_UNBUFFERED_PRIORITY  K_PRIO_PREEMPT(2)
#define APM_MAIN_PRIORITY             K_PRIO_PREEMPT(3)   /* <== the flight loop */
#ifndef APM_CAN_PRIORITY
#define APM_CAN_PRIORITY              K_PRIO_PREEMPT(5)
#endif
#ifndef APM_RCIN_PRIORITY
#define APM_RCIN_PRIORITY             K_PRIO_PREEMPT(6)
#endif
#ifndef APM_I2C_PRIORITY
#define APM_I2C_PRIORITY              K_PRIO_PREEMPT(7)
#endif
#define APM_LED_PRIORITY              K_PRIO_PREEMPT(9)
#define APM_UART_PRIORITY             K_PRIO_PREEMPT(9)
#define APM_NET_PRIORITY              K_PRIO_PREEMPT(9)
/* Two escape hatches for a board whose loop leaves no headroom (hwdef.dat
   `define APM_STORAGE_PRIORITY K_PRIO_PREEMPT(2)`): a thread below main is fed
   only by the INS wait and the 50 us yield, on this HAL and on ChibiOS alike.
   Overriding skips the order assert for that thread only, and puts it on the
   timer/SPI level - there is no free level between main (3) and timer (2) - so
   its work then delays IMU sampling, which ChibiOS never does. */
#ifndef APM_STORAGE_PRIORITY
#define APM_STORAGE_PRIORITY          K_PRIO_PREEMPT(10)
#define AP_ZEPHYR_STORAGE_BELOW_MAIN 1
#endif
#ifndef APM_IO_PRIORITY
#define APM_IO_PRIORITY               K_PRIO_PREEMPT(11)  /* the HAL's own AP_io thread */
#define AP_ZEPHYR_IO_BELOW_MAIN 1
#endif
/* The base for thread_create(PRIORITY_IO, ...) callers - AP_Logger's log_io
   (IO+1), the MAVFTP worker, compass cal. Separate from APM_IO_PRIORITY so a
   board that lifts its AP_io thread with the hatch above does not drag
   AP_Logger's SD writes up to the boost level with it. */
#ifndef APM_IO_USER_PRIORITY
#define APM_IO_USER_PRIORITY          K_PRIO_PREEMPT(11)
#endif
#define APM_STARTUP_PRIORITY          K_PRIO_PREEMPT(13)
#define APM_SCRIPTING_PRIORITY        APM_LOWPRIO

/* ORDER and RANGE, never numbers. Two priority inversions have shipped in this
 * file because a number was edited without re-checking its rank against the
 * table; every relation ChibiOS relies on is asserted here. */
#define AP_ZEPHYR_PREEMPT_OK(p) ((p) >= APM_HIGHPRIO && (p) <= APM_LOWPRIO)
static_assert(AP_ZEPHYR_PREEMPT_OK(APM_MONITOR_PRIORITY) && AP_ZEPHYR_PREEMPT_OK(APM_MAIN_PRIORITY_BOOST) &&
              AP_ZEPHYR_PREEMPT_OK(APM_TIMER_PRIORITY) && AP_ZEPHYR_PREEMPT_OK(APM_RCOUT_PRIORITY) &&
              AP_ZEPHYR_PREEMPT_OK(APM_SPI_PRIORITY) && AP_ZEPHYR_PREEMPT_OK(APM_UART_UNBUFFERED_PRIORITY) &&
              AP_ZEPHYR_PREEMPT_OK(APM_MAIN_PRIORITY) && AP_ZEPHYR_PREEMPT_OK(APM_CAN_PRIORITY) &&
              AP_ZEPHYR_PREEMPT_OK(APM_RCIN_PRIORITY) && AP_ZEPHYR_PREEMPT_OK(APM_I2C_PRIORITY) &&
              AP_ZEPHYR_PREEMPT_OK(APM_LED_PRIORITY) && AP_ZEPHYR_PREEMPT_OK(APM_UART_PRIORITY) &&
              AP_ZEPHYR_PREEMPT_OK(APM_NET_PRIORITY) && AP_ZEPHYR_PREEMPT_OK(APM_STORAGE_PRIORITY) &&
              AP_ZEPHYR_PREEMPT_OK(APM_IO_PRIORITY) && AP_ZEPHYR_PREEMPT_OK(APM_IO_USER_PRIORITY) &&
              AP_ZEPHYR_PREEMPT_OK(APM_STARTUP_PRIORITY) && AP_ZEPHYR_PREEMPT_OK(APM_SCRIPTING_PRIORITY),
              "every APM_* priority must be preemptible: 0..CONFIG_NUM_PREEMPT_PRIORITIES-1. "
              "A negative value is cooperative and freezes every other thread; CONFIG_ASSERT is off so the kernel would not say so");
static_assert(APM_MAIN_PRIORITY == K_PRIO_PREEMPT(CONFIG_MAIN_THREAD_PRIORITY),
              "prj.conf CONFIG_MAIN_THREAD_PRIORITY and APM_MAIN_PRIORITY disagree; Zephyr creates main, so Kconfig would win silently");
static_assert(APM_MONITOR_PRIORITY < APM_MAIN_PRIORITY_BOOST,
              "monitor must outrank the boosted main loop (ChibiOS 183 > 182)");
static_assert(APM_MAIN_PRIORITY_BOOST == APM_MAIN_PRIORITY ||
              (APM_MAIN_PRIORITY_BOOST < APM_TIMER_PRIORITY && APM_MAIN_PRIORITY_BOOST < APM_SPI_PRIORITY &&
               APM_MAIN_PRIORITY_BOOST < APM_RCOUT_PRIORITY),
              "the boost sits strictly above timer/SPI/rcout (ChibiOS 182 > 181), or equals main to compile the boost out");
static_assert(APM_TIMER_PRIORITY == APM_RCOUT_PRIORITY && APM_TIMER_PRIORITY == APM_SPI_PRIORITY &&
              APM_TIMER_PRIORITY == APM_UART_UNBUFFERED_PRIORITY,
              "ChibiOS runs timer, rcout, SPI and unbuffered UART at one level (181)");
static_assert(APM_SPI_PRIORITY < APM_MAIN_PRIORITY,
              "SPI must preempt the loop it feeds (ChibiOS 181 > 180)");
static_assert(APM_MAIN_PRIORITY < APM_CAN_PRIORITY && APM_CAN_PRIORITY < APM_RCIN_PRIORITY &&
              APM_RCIN_PRIORITY < APM_I2C_PRIORITY,
              "ChibiOS: main 180 > CAN 178 > rcin 177 > I2C 176");
static_assert(APM_I2C_PRIORITY < APM_NET_PRIORITY - 1,
              "one free level between I2C and NET for NET+1 (lwip tcpip, ChibiOS 61)");
static_assert(APM_LED_PRIORITY == APM_UART_PRIORITY && APM_UART_PRIORITY == APM_NET_PRIORITY,
              "ChibiOS: LED, UART and NET all at 60");
#ifdef AP_ZEPHYR_STORAGE_BELOW_MAIN
static_assert(APM_NET_PRIORITY < APM_STORAGE_PRIORITY,
              "storage one step under LED/UART/NET (ChibiOS 59 < 60); work-limited boards override APM_STORAGE_PRIORITY in hwdef.dat");
#endif
#ifdef AP_ZEPHYR_IO_BELOW_MAIN
static_assert(APM_MAIN_PRIORITY < APM_IO_PRIORITY && APM_IO_PRIORITY + 1 < APM_STARTUP_PRIORITY,
              "io below main (ChibiOS 58 < 180) with a free level under it for IO-1; boards that cannot feed it override APM_IO_PRIORITY in hwdef.dat");
#endif
static_assert(APM_MAIN_PRIORITY < APM_IO_USER_PRIORITY,
              "user io threads (log_io, FTP) stay below the flight loop even when a board lifts AP_io");
static_assert(APM_STARTUP_PRIORITY < APM_SCRIPTING_PRIORITY,
              "setup() runs above scripting only (ChibiOS 10 > 2)");

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
    /* The main thread's INTENDED priority, owned by the HAL (ChibiOS's realprio).
       set_main_priority() is hal_chibios_set_priority(); reassert_main_priority()
       is what Semaphore::give() calls after k_mutex_unlock(), because Zephyr's
       mutex restores the priority the owner had when it LOCKED. */
    static void set_main_priority(int prio);
    static void reassert_main_priority();
    /* Semaphore::take() bookkeeping: a thread about to block on a mutex main owns
       registers itself, so neither function above drops main below it while it
       is pending (chMtxUnlock() recomputes from every owned mutex's waiters;
       k_mutex_unlock() does not). */
    static void main_waiter_begin(int prio);
    static void main_waiter_end();
    static k_tid_t main_thread_id();
    void hal_initialized() { _hal_initialized = true; }

    /* Persistent crash/watchdog forensics (ChibiOS parity). */
    void save_persistent_data();
    void restore_persistent_data();

private:
    bool _initialized      = false;
    bool _hal_initialized  = false;
    bool _called_boost     = false;
    bool _priority_boosted = false;

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
