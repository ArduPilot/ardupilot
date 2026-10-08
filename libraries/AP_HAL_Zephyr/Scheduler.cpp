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
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "Scheduler.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "RCInput.h"
#include "RCOutput.h"
#if HAL_WITH_IO_MCU && AP_ZEPHYR_IOMCU_ENABLED
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif
#include "DeviceBus.h"
#include "UARTDriver.h"   /* UARTSTAT byte counters in the LOOPRATE report */
#include "zephyr/src/ap_hooks.h"
#include "zephyr/src/rt1176_romapi_flash.h"   /* g_ap_flash_busy/_ops for the WDG line */   /* ap_sysinfo_capture(), ap_persistent_save_fault() */

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_InternalError/AP_InternalError.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
/* Independent hardware watchdog node, per SoC: STM32 = iwdg1, NXP RT11xx =
   wdog1. Whichever exists+okay is used as the backstop. */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(iwdg1), okay)
#define HW_WATCHDOG_NODE DT_NODELABEL(iwdg1)
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(wdog1), okay)
#define HW_WATCHDOG_NODE DT_NODELABEL(wdog1)
#endif
#if defined(HW_WATCHDOG_NODE)
#include <zephyr/drivers/watchdog.h>
#define HAVE_HW_WATCHDOG 1
#endif
#if defined(CONFIG_SOC_SERIES_IMXRT11XX)
#include "zephyr/src/rt1176_snvs_rtc.h"
#endif

/* ── thread stacks (file-scope, required by Zephyr macros) ───────────── */
/* K_THREAD_STACK_DEFINE MUST COME BEFORE THE AP_Logger INCLUDE BELOW, or its
 * macros are redefined and the stacks land in the wrong section. */
/* STACKS IN DTCM (2026-08-06): DTCM is single-cycle and never contends with the
 * FlexSPI XIP path, so stack traffic stops competing with instruction fetch. */
/* 2026-08-08: stacks moved DTCM -> OCRAM (.noinit) to free FlexRAM banks
   for ITCM (128 KB of code beats stack locality: the DTCM stack move earned
   +2.4 Hz in 08-06; the AP_Motors+SRV XIP tax measured ~7.8% of the core at
   373 Hz). ".noinit" is the plain OCRAM no-init section. */
/* 2026-08-08 partial revert: the FULL move of stacks to OCRAM coincided with a
 * regression, so only part of it stands. */
#define AP_STACK_SECTION __noinit
/* Only a board that declares /chosen/zephyr,dtcm has a DTCM to place these in.
   Without one the linker puts __dtcm_* in flash instead of erroring - see the
   note in DeviceBus.cpp. */
#if DT_HAS_CHOSEN(zephyr_dtcm)
#define AP_STACK_SECTION_HOT __dtcm_noinit_section
#else
#define AP_STACK_SECTION_HOT __noinit
#endif

/* DTCM. 1 kHz timer callbacks (Scheduler::_timer_thread_fn), PREEMPT(2). Runs
   above main and fires 1000x/s, so its stack is touched constantly. */
// _zephyr_timer_stack    0x20022180  DTCM ✅
Z_KERNEL_STACK_DEFINE_IN(_zephyr_timer_stack,   ZEPHYR_TIMER_THREAD_STACK_SZ,   AP_STACK_SECTION);
/* DTCM. IO thread, PREEMPT(5): drains the MAVLink send queue and AP_Param
   saves, and renders g_ap_sysinfo. Deepest stack of the AP threads (8 KB) -
   threads.txt measured 7128 B still free, i.e. ~1 KB in use. */
// _zephyr_io_stack       0x20020100  DTCM ✅
Z_KERNEL_STACK_DEFINE_IN(_zephyr_io_stack,      ZEPHYR_IO_THREAD_STACK_SZ,      AP_STACK_SECTION);
/* DTCM. Monitor thread at PREEMPT(0) - the highest priority in the system, so
   it can observe a stuck main thread. Must never be blocked by bus contention. */
// _zephyr_monitor_stack  0x2001f880  DTCM ✅
Z_KERNEL_STACK_DEFINE_IN(_zephyr_monitor_stack, ZEPHYR_MONITOR_THREAD_STACK_SZ, AP_STACK_SECTION);
/* DTCM. RC input decode, PREEMPT(4), ~1 kHz. Measured 0.9-1.0% of the core. */
// _zephyr_rcin_stack     0x2001e800  DTCM ✅
Z_KERNEL_STACK_DEFINE_IN(_zephyr_rcin_stack,    ZEPHYR_RCIN_THREAD_STACK_SZ,    AP_STACK_SECTION);
/* DTCM. PWM output, PREEMPT(4), ~1 kHz. On the motor path, so jitter here is
   directly visible on the outputs. */
// _zephyr_rcout_stack    0x2001d780  DTCM ✅
Z_KERNEL_STACK_DEFINE_IN(_zephyr_rcout_stack,   ZEPHYR_RCOUT_THREAD_STACK_SZ,   AP_STACK_SECTION);
/* DTCM. Storage writes to external NOR via the ROM API, PREEMPT(12) - the
   lowest of the AP threads. Measured 0.0% of the core; here for uniformity
   rather than need, and because it is only 4 KB. */
// _zephyr_storage_stack  0x2001c700  DTCM ✅
Z_KERNEL_STACK_DEFINE_IN(_zephyr_storage_stack, ZEPHYR_STORAGE_THREAD_STACK_SZ, AP_STACK_SECTION);
/* DTCM pool backing hal.scheduler->thread_create(). */
// _zephyr_user_stacks    0x20000000  DTCM ✅
Z_KERNEL_STACK_ARRAY_DEFINE_IN(_zephyr_user_stacks, ZEPHYR_MAX_USER_THREADS,
                               ZEPHYR_USER_THREAD_STACK_SZ, AP_STACK_SECTION);

#if HAL_LOGGING_ENABLED
/* For AP::logger().StopLogging() in reboot(), so a reboot does not truncate a log. */
#include <AP_Logger/AP_Logger.h>
/* For AP::FS().retry_mount() in _run_io() - the SD remount retry. Dead for the
   same reason and surfaced by the same change. */
#include <AP_Filesystem/AP_Filesystem.h>
#endif

/* ── 1 ms semaphore-based wakeup for timer and IO threads ────────────── */
static K_SEM_DEFINE(s_timer_sem, 0, 2);
static K_SEM_DEFINE(s_io_sem,    0, 2);

static void timer_expiry_fn(struct k_timer *) { k_sem_give(&s_timer_sem); }
static void io_expiry_fn(struct k_timer *)    { k_sem_give(&s_io_sem);    }

static K_TIMER_DEFINE(s_hal_timer, timer_expiry_fn, NULL);
static K_TIMER_DEFINE(s_hal_io,    io_expiry_fn,    NULL);

/* Monitor stand-down: uptime-ms until which the main-loop watchdog must
   not warn/reset, settable by subsystems whose bring-up stalls the whole
   system from OUTSIDE main (WiFiDriver.cpp radio start). extern "C" so
   users need no namespace gymnastics. 0 = inactive. */
extern "C" volatile uint32_t ap_zephyr_grace_until_ms;
volatile uint32_t ap_zephyr_grace_until_ms;

namespace AP_HAL {
extern volatile bool _hal_zephyr_panicked;   /* defined in system.cpp */
}

/* Monitor thresholds: how long the main loop may stall before it is reported. */
static constexpr uint32_t MONITOR_WARN_MS  =  500;
static constexpr uint32_t MONITOR_RESET_MS =  1800;
#if defined(HAVE_HW_WATCHDOG)
/* Hardware IWDG timeout. The monitor thread feeds it every 100 ms, so
   this only expires if the monitor is starved for 2 s straight — total
   lockup. Well below the STM32 IWDG max (~32 s) and well above the feed
   period. */
static constexpr uint32_t HW_WDT_TIMEOUT_MS = 2000;  // 2s watchdog
#endif
static constexpr size_t   MIN_STACK_FREE   =  256;



extern const AP_HAL::HAL& hal;

/* Primary instrument: main loop rate. */
extern "C" uint32_t g_ap_loop_count;
uint32_t g_ap_loop_count;

/* ChibiOS keeps two priorities per thread: realprio (what the code asked for)
 * and the effective one, raised by mutex inheritance; hal_chibios_set_priority()
 * writes realprio always and the effective one only when it is not inherited or
 * the new value beats it (HAL_ChibiOS_Class.cpp:193-204). Zephyr keeps only
 * base.prio: k_mutex_lock() snapshots it into the mutex (kernel/mutex.c:125-127)
 * and k_mutex_unlock() writes the snapshot back (mutex.c:275), so a
 * k_thread_priority_set() made while a HAL_Semaphore is held is undone at the
 * give(). Two mutexes are held across the boost every loop: AP_AHRS::update()
 * takes _rsem then calls boost_end() (AP_AHRS.cpp:563-573), and AP_Scheduler
 * re-takes its own _rsem right after wait_for_sample() and gives it at the top
 * of the next loop (AP_Scheduler.cpp:370-372). Either give() would have put main
 * back at the boost level for the rest of the loop, above timer and SPI, which
 * is the strongest candidate for the July 2026 "service threads queued, never
 * scheduled" hang. s_main_intended_prio is this HAL's realprio and
 * Semaphore::give() re-asserts it. The counters make the mechanism visible in
 * the LoopRate line: in a healthy loop reasserts is about twice boosts, and
 * leaks - main found NOT at its intended priority when a loop begins - is 0. */
static k_tid_t s_main_tid;
static int     s_main_intended_prio = APM_MAIN_PRIORITY;
extern "C" uint32_t g_ap_boost_count;           /* delay_microseconds_boost() raises */
extern "C" uint32_t g_ap_main_prio_reasserts;   /* give() had to put main back */
extern "C" uint32_t g_ap_main_prio_leaks;       /* a loop began with main off its intended level */
uint32_t g_ap_boost_count;
uint32_t g_ap_main_prio_reasserts;
uint32_t g_ap_main_prio_leaks;

/* Threads pending on a HAL semaphore that main owns. chMtxUnlock() recomputes
 * the owner's priority from realprio and the head waiter of EVERY mutex it still
 * owns (os/rt/src/chmtx.c); k_mutex_unlock() restores one mutex's snapshot and
 * knows nothing about the others, and a recursive unlock restores nothing. So
 * the HAL keeps the count: Semaphore::take() on a thread that is about to block
 * on a mutex main owns registers here, and while any are pending main is never
 * put below the most urgent of them - not by give() and not by set_main_priority().
 * Without this a give() of an inner semaphore dropped main to its intended level
 * while the monitor thread (0) was still pending on the statustext semaphore
 * (found in review, 2026-09-12). */
static struct k_spinlock s_main_waiters_lock;
static uint8_t  s_main_waiters;        /* pending now */
static int      s_main_waiters_best;   /* most urgent of them (lowest number) */
extern "C" uint32_t g_ap_main_prio_inherit_holds; /* give() left main raised for a pending waiter */
uint32_t g_ap_main_prio_inherit_holds;

void Zephyr::Scheduler::main_waiter_begin(int prio)
{
    k_spinlock_key_t key = k_spin_lock(&s_main_waiters_lock);
    if (s_main_waiters == 0 || prio < s_main_waiters_best) {
        s_main_waiters_best = prio;
    }
    if (s_main_waiters < 255) {
        s_main_waiters++;
    }
    k_spin_unlock(&s_main_waiters_lock, key);
}

void Zephyr::Scheduler::main_waiter_end()
{
    k_spinlock_key_t key = k_spin_lock(&s_main_waiters_lock);
    if (s_main_waiters > 0) {
        s_main_waiters--;
    }
    k_spin_unlock(&s_main_waiters_lock, key);
}

k_tid_t Zephyr::Scheduler::main_thread_id()
{
    return s_main_tid;
}

/* chMtxUnlock()'s newprio: the intended level, or a pending waiter's if that is
   more urgent. The waiter stays counted until it owns the mutex, by which time
   main's unlock has already restored main's own snapshot. */
static int main_target_prio()
{
    int target = s_main_intended_prio;
    k_spinlock_key_t key = k_spin_lock(&s_main_waiters_lock);
    if (s_main_waiters != 0 && s_main_waiters_best < target) {
        target = s_main_waiters_best;
    }
    k_spin_unlock(&s_main_waiters_lock, key);
    return target;
}

/* published by ArduCopter's rate thread (rate_thread.cpp), printed from
   the main thread's LOOPRATE reporter - the rate thread must never touch
   the console itself (poll_out spin at its priority starves WiFi) */
volatile uint32_t ap_zephyr_rate_thread_hz;
volatile uint32_t ap_zephyr_rate_thread_busy_pct;
volatile uint32_t ap_zephyr_rate_thread_maxdt_us;
volatile uint32_t ap_zephyr_rate_thread_qdepth;

/* Included UNCONDITIONALLY and it must stay that way: chain_profile.h defines the
 * no-op macros too, so a conditional include breaks every non-profiling build. */
#include "chain_profile.h"

#if defined(CONFIG_AP_CHAIN_PROFILE) || HAL_ENABLE_THREAD_STATISTICS
/* Read over SWD without halting: find the address with
   arm-none-eabi-nm zephyr.elf | grep g_ap_phase, then
   pyocd commander --connect attach -c "read32 <addr>" in a loop. */
extern "C" volatile uint32_t g_ap_prof[AP_PROF_WORDS];
volatile uint32_t g_ap_prof[AP_PROF_WORDS];

/* Out of line so chain_profile.h stays free of <zephyr/kernel.h> - it is
   included directly by AP_NavEKF3, AP_AHRS, AP_InertialSensor and
   AP_Scheduler, and dragging the kernel headers into those is not worth the
   inlining. See APPhaseScope. */
extern "C" uint32_t ap_prof_cycles(void)
{
    return k_cycle_get_32();
}

#endif

using namespace Zephyr;

/* ─── init ───────────────────────────────────────────────────────────── */

void Scheduler::init()
{

    _main_tid = k_current_get();
    s_main_tid = _main_tid;
    _last_watchdog_pat_ms = (uint32_t)k_uptime_get_32();

    for (auto &t : _user_threads) {
        t.in_use = false;
    }

#if HAL_MONITOR_THREAD_ENABLED
    /* Monitor: above everything, so it can observe a stuck main thread */
    k_thread_create(&_monitor_thread_data, _zephyr_monitor_stack,
                    ZEPHYR_MONITOR_THREAD_STACK_SZ,
                    _monitor_thread_fn, this, nullptr, nullptr,
                    APM_MONITOR_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&_monitor_thread_data, "AP_monitor");

#endif

#ifndef HAL_NO_TIMER_THREAD
    /* Timer: 1000 Hz timer callbacks, above main (ChibiOS 181) */
    k_thread_create(&_timer_thread_data, _zephyr_timer_stack,
                    ZEPHYR_TIMER_THREAD_STACK_SZ,
                    _timer_thread_fn, this, nullptr, nullptr,
                    APM_TIMER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&_timer_thread_data, "AP_timer");
#endif

#ifndef HAL_USE_EMPTY_IO
    /* IO: 1000 Hz IO callbacks, drains AP_Param save_queue; BELOW main
       (ChibiOS 58) - fed by the INS wait and the per-loop yield */
    k_thread_create(&_io_thread_data, _zephyr_io_stack,
                    ZEPHYR_IO_THREAD_STACK_SZ,
                    _io_thread_fn, this, nullptr, nullptr,
                    APM_IO_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&_io_thread_data, "AP_io");
#endif

#if HAL_RCIN_THREAD_ENABLED
    /* RCIN: RC input polling at ~1 kHz, below main (ChibiOS 177) */
    k_thread_create(&_rcin_thread_data, _zephyr_rcin_stack,
                    ZEPHYR_RCIN_THREAD_STACK_SZ,
                    _rcin_thread_fn, this, nullptr, nullptr,
                    APM_RCIN_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&_rcin_thread_data, "AP_rcin");
#endif

#ifndef HAL_NO_RCOUT_THREAD
    /* RCOUT: RC output at ~1 kHz, timer level (ChibiOS 181) */
    k_thread_create(&_rcout_thread_data, _zephyr_rcout_stack,
                    ZEPHYR_RCOUT_THREAD_STACK_SZ,
                    _rcout_thread_fn, this, nullptr, nullptr,
                    APM_RCOUT_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&_rcout_thread_data, "AP_rcout");
#endif

    /* Storage: flash/EEPROM writes, below main (ChibiOS 59) */
#ifndef HAL_USE_EMPTY_STORAGE
    k_thread_create(&_storage_thread_data, _zephyr_storage_stack,
                    ZEPHYR_STORAGE_THREAD_STACK_SZ,
                    _storage_thread_fn, this, nullptr, nullptr,
                    APM_STORAGE_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&_storage_thread_data, "AP_storage");
#endif

    /* Mark hardware ready so rcin/rcout/storage threads start working */
    _hal_initialized = true;

    /* Start 1 ms hardware timers for timer and IO threads. Each only exists to
       wake its thread, so do not run it when that thread was not created. */
#ifndef HAL_NO_TIMER_THREAD
    k_timer_start(&s_hal_timer, K_USEC(1000), K_USEC(1000));
#endif
#ifndef HAL_USE_EMPTY_IO
    k_timer_start(&s_hal_io,    K_USEC(1000), K_USEC(1000));
#endif
#endif
}

/* ─── delay ──────────────────────────────────────────────────────────── */

#ifdef CONFIG_AP_DELAY_CB_PROFILE
/* Timings for the delay-callback profiler. File-scope so they stay readable. */
uint32_t g_delay_cb_calls;        // call_delay_cb() invocations
uint64_t g_delay_cb_total_us;     // summed callback duration
uint32_t g_delay_cb_max_us;       // worst single callback
uint32_t g_delay_calls;           // delay() invocations
uint64_t g_delay_req_us_total;    // summed requested delay
uint64_t g_delay_act_us_total;    // summed actual delay
/* delay_microseconds_boost() is a SEPARATE path from delay(). It is what
   AP_InertialSensor::wait_for_sample() spins on while waiting for IMU
   samples, and that loop has a 100ms absolute timeout per update(). Gyro
   cal calls update() 50x per iteration, so a stalled sample path costs up
   to 5s per iteration - invisible to the delay() counters above. */
uint32_t g_boost_calls;           // delay_microseconds_boost() calls
uint64_t g_boost_total_us;        // summed time spun in boost
#endif

/* DELAYPROF: requested vs actual for SHORT delays, split by thread, because the
   FTP worker's idle loop is hal.scheduler->delay(2) and its measured poll rate
   was ~1.1 Hz where 2 ms implies ~500 Hz. Split main/non-main because the
   deadline loop below issues one k_msleep(1) per millisecond, so each wake has
   to win the CPU again - which costs far more at PRIORITY_IO than on main. */
/* Bucketed by CALLER PRIORITY, not main/other: the deadline loop issues one
   k_msleep(1) per millisecond requested and each wake must re-win the CPU, so
   the same delay(ms) can cost wildly different wall time depending on where the
   caller sits in the ladder. main at PREEMPT(3) measured x1.09; the FTP worker
   at PRIORITY_IO was never sampled because it never reached its idle loop.
   Buckets: 0 = prio<=3 (main and above), 1 = prio 4..9, 2 = prio>=10. */
#define DLY_BUCKETS 3
static volatile uint32_t g_dly_calls[DLY_BUCKETS];
static volatile uint64_t g_dly_req_us[DLY_BUCKETS];
static volatile uint64_t g_dly_act_us[DLY_BUCKETS];
static volatile uint32_t g_dly_max_us[DLY_BUCKETS];

/* Defined in DeviceBus.cpp. extern "C" so the name does not pick up namespace
   Zephyr, and at file scope because a linkage specification is not allowed
   inside a function body. */
extern "C" uint32_t g_buscb_calls[6];
extern "C" uint64_t g_buscb_us[6];
extern "C" uint32_t g_buscb_max_us[6];
/* Defined in I2CDevice.cpp - see the I2CERR comment there. */
extern "C" uint32_t g_i2c_ok[3];
extern "C" uint32_t g_i2c_nak[3];
extern "C" uint32_t g_i2c_timeout[3];
extern "C" uint32_t g_i2c_otherr[3];
extern "C" uint32_t g_i2c_reset[3];
extern "C" int32_t  g_i2c_lasterr[3];
extern "C" uint32_t g_i2c_maxus[3];
extern "C" uint64_t g_i2c_okus[3];
/* Defined in modules/zephyr/drivers/i2c/i2c_mcux_lpi2c.c - the phase split for a
   successful transfer. AP measured 7-21 ms mean successful transfers with zero
   NAKs and zero completion timeouts, so the time must be in the driver's
   K_FOREVER lock or its busy-bus check; these say which. */
extern "C" uint32_t ap_lpi2c_stat_lock_us;
extern "C" uint32_t ap_lpi2c_stat_lock_n;
extern "C" uint32_t ap_lpi2c_stat_busy_us;
extern "C" uint32_t ap_lpi2c_stat_busy_n;
extern "C" uint32_t ap_lpi2c_stat_bbok_us;
/* Defined in AP_HAL_Zephyr/RCOutput.cpp - see the RCOUTDIAG note there. */
extern "C" uint16_t g_rcout_last_pulse[4];
extern "C" int16_t  g_rcout_last_rc[4];
extern "C" uint8_t  g_rcout_safety;
/* Defined in AP_Logger/AP_Logger_File.cpp - reported from HERE, not from
   io_timer(), because that runs on the starved log_io thread. */
/* Defined in AP_HAL_Zephyr/RCOutput.cpp, for the i.MX RT11xx only. extern "C"
   and at FILE scope: a plain extern inside namespace Zephyr resolves to
   Zephyr::ap_flexpwm_dump, and a linkage specification is not allowed inside a
   function body. */
#if defined(CONFIG_SOC_SERIES_IMXRT11XX)
extern "C" void ap_flexpwm_dump(void);
#endif
extern "C" uint32_t g_logdiag_gap_max;
extern "C" uint32_t g_logdiag_snl_max;
extern "C" uint32_t g_logdiag_snl_calls;
extern "C" uint32_t g_logdiag_iot_max;
extern "C" uint32_t g_logdiag_iot_calls;

void Scheduler::delay(uint16_t ms)
{
    /* (The note that used to sit here said micros64() relies on
       k_cyc_to_us_floor64() and returns 0 on Xtensa for short durations. That
       stopped being true: micros64() is hardware-backed on every arch this HAL
       builds for - wrap-extended CCOUNT on Xtensa, TIM5 on STM32H7, GPT on
       IMXRT11XX - see AP_HAL_Zephyr/system.cpp. The loop below depends on
       that, so the stale warning is removed rather than left to argue against
       it.) */
#ifdef CONFIG_AP_DELAY_CB_PROFILE
    const uint64_t prof_entry_us = AP_HAL::micros64();
#endif
    /* DEADLINE loop, as ChibiOS's Scheduler::delay() does. The previous form
       counted exactly ms iterations of a 1 ms sleep, which ADDED the delay
       callback's cost to the wait instead of absorbing it: a 2 ms callback
       made delay(100) take 300 ms. Comparing elapsed time against the target
       means an expensive callback simply consumes iterations.

       micros64() is the clock to compare against, not k_uptime_get(). Both are
       available, and they are not equally trustworthy: micros64() reads a
       hardware counter per SoC (GPT on IMXRT11XX, TIM5 on STM32H7, the
       wrap-extended CCOUNT on Xtensa), independent of the kernel tick, while
       k_uptime_get() is derived FROM the tick. Measured on mr_vmu_rt1176
       2026-09-18, a misconfigured tick left k_uptime under-reporting by 10x
       while micros64() stayed true to host wall-clock - see the note in
       zephyr/prj.conf. */
    const uint64_t start_us = AP_HAL::micros64();
    const uint64_t target_us = (uint64_t)ms * 1000U;
    const bool dly_short = (ms <= 4);
    const int dly_prio = k_thread_priority_get(k_current_get());
    const uint8_t dly_b = (dly_prio <= 3) ? 0 : ((dly_prio <= 9) ? 1 : 2);
    while (AP_HAL::micros64() - start_us < target_us) {
        /* k_msleep(1), NOT delay_microseconds(1000): the latter busy-waits on some paths,
         * which is exactly what this loop must not do. */
        k_msleep(1);
        if (_min_delay_cb_ms <= ms) {
            if (in_main_thread()) {
#ifdef CONFIG_AP_DELAY_CB_PROFILE
                const uint64_t cb_start_us = AP_HAL::micros64();
                call_delay_cb();
                /* micros64() so this stays correct regardless of micros()'s
                   range - the profiler must not depend on what it measures */
                const uint32_t cb_us = (uint32_t)(AP_HAL::micros64() - cb_start_us);
                g_delay_cb_calls++;
                g_delay_cb_total_us += cb_us;
                if (cb_us > g_delay_cb_max_us) {
                    g_delay_cb_max_us = cb_us;
                }
#else
                call_delay_cb();
#endif
            }
        }
    }
    if (dly_short) {
        const uint64_t act = AP_HAL::micros64() - start_us;
        g_dly_calls[dly_b]++;
        g_dly_req_us[dly_b] += target_us;
        g_dly_act_us[dly_b] += act;
        if (act > g_dly_max_us[dly_b]) {
            g_dly_max_us[dly_b] = (uint32_t)act;
        }
    }
#ifdef CONFIG_AP_DELAY_CB_PROFILE
    g_delay_calls++;
    g_delay_req_us_total += (uint64_t)ms * 1000U;   // ms -> us
    g_delay_act_us_total += AP_HAL::micros64() - prof_entry_us;
    profile_report();
#endif
}

#ifdef CONFIG_AP_DELAY_CB_PROFILE
/*
  Throttled one-line summary. Called after the timings above are recorded, so
  the cost of printing is never folded into the numbers it prints - which
  matters here because the console is itself the prime suspect.
*/
void Scheduler::profile_report(void)
{
    static uint32_t last_report_ms;
    const uint32_t now_ms = AP_HAL::millis();

    /* 5000ms: rare enough that the print cannot meaningfully perturb the
       thing being measured, frequent enough to watch it evolve during boot */
    if (now_ms - last_report_ms < 5000U) {
        return;
    }
    last_report_ms = now_ms;

    if (g_delay_cb_calls == 0 || g_delay_calls == 0) {
        return;
    }
    DEV_PRINTF("DLYPROF cb n=%lu avg=%luus max=%luus | delay n=%lu req=%lums act=%lums | boost n=%lu tot=%lums\n",
               (unsigned long)g_delay_cb_calls,
               (unsigned long)(g_delay_cb_total_us / g_delay_cb_calls),
               (unsigned long)g_delay_cb_max_us,
               (unsigned long)g_delay_calls,
               (unsigned long)(g_delay_req_us_total / 1000U),
               (unsigned long)(g_delay_act_us_total / 1000U),
               (unsigned long)g_boost_calls,
               (unsigned long)(g_boost_total_us / 1000U));
}
#endif

void Scheduler::delay_microseconds(uint16_t us)
{
    /* SLEEPS - structurally identical to AP_HAL_ChibiOS::Scheduler.cpp, so the two
     * HALs have the same blocking behaviour rather than one busy-waiting. */
    if (us == 0) {
        return;
    }
    uint32_t ticks = k_us_to_ticks_ceil32(us);
    if (ticks == 0) {
        ticks = 1;
    }
    k_sleep(K_TICKS(ticks));
}

void Scheduler::set_main_priority(int prio)
{
    s_main_intended_prio = prio;
    if (s_main_tid == nullptr) {
        return;
    }
    /* hal_chibios_set_priority() keeps main raised when a mutex waiter raised it
       ((effective == realprio) || (new > effective)). Here the waiters are
       counted, so the level is computed rather than inferred: intended, unless
       a pending waiter on a semaphore main owns is more urgent. */
    const int cur = k_thread_priority_get(s_main_tid);
    const int target = main_target_prio();
    if (cur == target) {
        /* k_thread_priority_set() to the SAME value is not a no-op: the kernel
           re-queues the thread at the tail of its level, i.e. a yield. Skip it. */
        return;
    }
    k_thread_priority_set(s_main_tid, target);
}

void Scheduler::reassert_main_priority()
{
    if (s_main_tid == nullptr || k_current_get() != s_main_tid) {
        return;
    }
    const int cur = k_thread_priority_get(s_main_tid);
    const int target = main_target_prio();
    if (cur != target) {
        g_ap_main_prio_reasserts++;
        k_thread_priority_set(s_main_tid, target);
    } else if (cur != s_main_intended_prio) {
        /* Raised for a waiter still pending on another semaphore main owns; left
           there, as chMtxUnlock() would. Counted so it can be seen. */
        g_ap_main_prio_inherit_holds++;
    }
}

void Scheduler::delay_microseconds_boost(uint16_t us)
{
#if APM_MAIN_PRIORITY_BOOST != APM_MAIN_PRIORITY
    /* Once per loop, as AP_HAL_ChibiOS/Scheduler.cpp:205-213: raised on the first
       INS wait, dropped by boost_end() from AP_AHRS::update() and expect_delay_ms().
       delay_microseconds() below SLEEPS (k_sleep) on every path; a spin here, above
       the SPI thread that produces the sample being waited for, would be a hang.
       Two explicit priority writes per loop, plus one mutex restore and one
       re-assert per HAL semaphore taken across the boost (two today). */
    if (!_priority_boosted && in_main_thread()) {
        if (k_thread_priority_get(s_main_tid) != main_target_prio()) {
            /* The July 2026 condition: a loop starting with main still at some
               other level. Must stay 0; see the counters above. (A level held
               for a pending waiter is the target, not a leak.) */
            g_ap_main_prio_leaks++;
        }
        set_main_priority(APM_MAIN_PRIORITY_BOOST);
        _priority_boosted = true;
        _called_boost = true;
        g_ap_boost_count++;
    }
#endif
#ifdef CONFIG_AP_DELAY_CB_PROFILE
    const uint64_t boost_start_us = AP_HAL::micros64();
    delay_microseconds(us);
    g_boost_calls++;
    g_boost_total_us += AP_HAL::micros64() - boost_start_us;
#else
    delay_microseconds(us);
#endif
}

void Scheduler::boost_end()
{
    g_ap_loop_count++;          /* once per main-loop iteration - see above */
    AP_PROF_TICK(AP_PROF_LOOP_COUNT);
#if APM_MAIN_PRIORITY_BOOST != APM_MAIN_PRIORITY
    if (in_main_thread() && _priority_boosted) {
        _priority_boosted = false;
        set_main_priority(APM_MAIN_PRIORITY);
    }
#endif
}

bool Scheduler::check_called_boost()
{
    if (!_called_boost) {
        return false;
    }
    _called_boost = false;
    return true;
}

/* ─── watchdog / expected delay ──────────────────────────────────────── */

/* Hardware-watchdog pre-reset interrupt. The WDOG raises this about half a
   period before it resets the SoC, which is the only moment the system can be
   looked at while it is still stuck: a watchdog reset takes no exception, so
   the fault record stays empty and AP's WDG line reads FT0 with nothing in it.

   Runs in interrupt context, so it only reads - no locks, no printk. Scheduler
   is already resident in ITCM (see zephyr/itcm_hot_code.ld), which matters
   because the point of this is to work when flash is not readable.

   main_pended is the wait queue the main thread is blocked on; resolving that
   address against the map file names what it is waiting for. main_pc comes
   from main's own stacked exception frame, PC being the seventh word at its
   saved process stack pointer. */
static Scheduler *s_wdt_sched;

/* Mirror of Scheduler::_last_watchdog_pat_ms, written by watchdog_pat(). The
   interrupt below is file-scope and the member is private; a mirror is less
   intrusive than widening the class for a diagnostic. */
static volatile uint32_t s_last_pat_ms;

static void ap_wdt_stall_cb(const struct device *dev, int channel_id)
{
    (void)dev;
    (void)channel_id;

    const uint32_t now = (uint32_t)k_uptime_get_32();
    const uint32_t stall = now - s_last_pat_ms;

    k_tid_t cur = k_current_get();
    const char *name = (cur != nullptr) ? k_thread_name_get(cur) : nullptr;
    const uint32_t cur_prio = (cur != nullptr)
                              ? (uint32_t)(int32_t)k_thread_priority_get(cur) : 0xFFFFFFFFu;

    uint32_t main_state = 0, main_pended = 0, main_pc = 0;
    if (s_main_tid != nullptr) {
        main_state = s_main_tid->base.thread_state;
        main_pended = (uint32_t)(uintptr_t)s_main_tid->base.pended_on;
#if defined(CONFIG_ARM)
        const uint32_t psp = (uint32_t)s_main_tid->callee_saved.psp;
        /* Only dereference a plausible stack pointer: ITCM/DTCM/OCRAM/SDRAM. */
        if (psp >= 0x20000000u && psp < 0x20400000u) {
            main_pc = ((const uint32_t *)(uintptr_t)psp)[6];
        }
#endif
    }

    ap_wdg_record_put(stall, (int32_t)hal.util->persistent_data.scheduler_task,
                      cur_prio, name, main_state, main_pended, main_pc);
}

void Scheduler::watchdog_pat()
{

    _last_watchdog_pat_ms = (uint32_t)k_uptime_get_32();
    s_last_pat_ms = _last_watchdog_pat_ms;
}

/* Persistent crash/watchdog forensics, ChibiOS parity: the record must survive the
 * reset that follows, so it lives in memory the reset does not clear. */
#define AP_PERSISTENT_DATA_MAGIC 0x50444B31U  // arbitrary, distinct from 0x00000000 and 0xFFFFFFFF

struct PersistentDataBackup {
    uint32_t magic;
    AP_HAL::Util::PersistentData data;
};

static struct PersistentDataBackup g_persistent_backup __noinit;

void Scheduler::save_persistent_data()
{
    g_persistent_backup.data = hal.util->persistent_data;
    g_persistent_backup.magic = AP_PERSISTENT_DATA_MAGIC;
}

void Scheduler::restore_persistent_data()
{
    if (!hal.util->was_watchdog_reset()) {
        return;
    }
    if (g_persistent_backup.magic != AP_PERSISTENT_DATA_MAGIC) {
        // watchdog-reset reboot, but no valid save from before it -
        // nothing to restore (e.g. the very first watchdog reset since
        // this feature was added, or .noinit didn't survive this
        // particular reset path - see this function's own header comment).
        return;
    }
    hal.util->persistent_data = g_persistent_backup.data;
    hal.util->last_persistent_data = g_persistent_backup.data;

    /* Fill the fault fields AP's WDG statustext prints from the record that
       survived the reset. They were reaching the screen as FA0 FLR0 FICSR0:
       ap_persistent_save_fault() is handed what the fatal handler was given,
       and on the paths seen on this board the exception frame pointer was
       null, so the PC and LR it would have carried were zero. The __noinit
       record has the same values captured directly, plus the CFSR, and AP
       repeats the WDG line every ~16 s - which is why it arrives when a
       one-shot STATUSTEXT sent 100 ms into the boot does not.
       Only filled where the restored data has nothing, so a genuine record is
       never overwritten. */
    unsigned int reason;
    uint32_t pc, lr, cfsr, icsr, prio, count;
    if (ap_fault_record_peek(&reason, &pc, &lr, &cfsr, &icsr, &prio, &count)) {
        AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
        if (pd.fault_type == 0) {
            pd.fault_type = (uint8_t)reason;
        }
        if (pd.fault_addr == 0) {
            pd.fault_addr = pc;
        }
        if (pd.fault_lr == 0) {
            pd.fault_lr = lr;
        }
        if (pd.fault_icsr == 0) {
            /* CFSR, not ICSR, when ICSR had nothing: which of UNDEFINSTR /
               INVSTATE / INVPC / UNALIGNED fired is what identifies the fault,
               and the slot is otherwise printed as zero. */
            pd.fault_icsr = (icsr != 0) ? icsr : cfsr;
        }
        hal.util->last_persistent_data = pd;
    }
}

/* Crash-forensics bridge, called from the C fatal handler; declared in
   zephyr/src/ap_hooks.h. */
/*
  Stamp the flash-operation state straight into the persistent data and its
  backup, called by the ROM flash path at the start and end of every operation.

  WHY NOT SAMPLE IT: the monitor thread samples every 100 ms, and at roughly
  0.8 flash operations per second each lasting far less than that, it catches one
  about 8% of the time - so "no operation in progress" in the record proved
  nothing. Worse, the freeze being chased stops the monitor too, so the last
  sample is always up to 100 ms STALE and can never show an operation that began
  after it. Writing from the flash path closes both gaps: if the SoC dies inside
  an operation, the surviving record says so because nothing cleared it.

  Called outside the interrupt-locked window at both ends, so XIP is sound and
  the ~50 byte backup copy is safe. At 0.8 operations per second the cost is
  immaterial.
 */
extern "C" void ap_persistent_flash_mark(uint32_t op, uint32_t offset, uint32_t in_flight)
{
    AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
    if (pd.fault_type != 0) {
        // a real fault owns these fields; never overwrite its evidence
        return;
    }
    pd.fault_line = (uint16_t)(((op & 0xFF) << 8) | (in_flight & 0xFF));
    pd.fault_icsr = offset;
    g_persistent_backup.data = pd;
    g_persistent_backup.magic = AP_PERSISTENT_DATA_MAGIC;
}

extern "C" void ap_persistent_save_fault(uint16_t line, uint8_t fault_type,
                                         uint32_t fault_addr, uint32_t fault_lr,
                                         uint32_t fault_icsr)
{
    AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
    if (pd.fault_type == 0) {
        pd.fault_line = line;
        pd.fault_type = fault_type;
        pd.fault_addr = fault_addr;
        pd.fault_lr = fault_lr;
        pd.fault_icsr = fault_icsr;
        k_tid_t tid = k_current_get();
        if (tid != nullptr) {
            pd.fault_thd_prio = (uint8_t)k_thread_priority_get(tid);
            const char *name = k_thread_name_get(tid);
            if (name != nullptr && pd.thread_name4[0] == 0) {
                // first 4 bytes of the name, un-terminated (ChibiOS parity)
                for (uint8_t i = 0; i < sizeof(pd.thread_name4) && name[i]; i++) {
                    pd.thread_name4[i] = name[i];
                }
            }
        }
    }
    g_persistent_backup.data = pd;
    g_persistent_backup.magic = AP_PERSISTENT_DATA_MAGIC;
}

void Scheduler::expect_delay_ms(uint32_t ms)
{

    if (!in_main_thread()) {
        return;
    }
    watchdog_pat();

    if (ms == 0) {
        if (_expect_delay_nesting > 0) {
            _expect_delay_nesting--;
        }
        if (_expect_delay_nesting == 0) {
            _expect_delay_start = 0;
        }
    } else {
        uint32_t now = (uint32_t)k_uptime_get_32();
        if (_expect_delay_start != 0) {
            uint32_t done = now - _expect_delay_start;
            if (_expect_delay_length > done) {
                ms = MAX(ms, _expect_delay_length - done);
            }
        }
        _expect_delay_start  = now;
        _expect_delay_length = ms;
        _expect_delay_nesting++;
        boost_end();
    }
}

bool Scheduler::in_expected_delay() const
{

    if (!_initialized) {
        return true;
    }
    if (_expect_delay_start != 0) {
        uint32_t now = (uint32_t)k_uptime_get_32();
        if ((now - _expect_delay_start) <= _expect_delay_length) {
            return true;
        }
    }
    return false;
}

/* ─── process registration ───────────────────────────────────────────── */

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_procs[i] == proc) {
            return;
        }
    }
    if (_num_timer_procs < ZEPHYR_SCHED_MAX_TIMER_PROCS) {
        _timer_procs[_num_timer_procs++] = proc;
    } else {
        /* printk alone goes to LPUART1, unmonitored on the bench; the
           INTERNAL_ERROR latches into SYS_STATUS where a GCS will see it */
        printk("AP_Zephyr: out of timer processes\n");
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_procs[i] == proc) {
            return;
        }
    }
    if (_num_io_procs < ZEPHYR_SCHED_MAX_IO_PROCS) {
        _io_procs[_num_io_procs++] = proc;
    } else {
        printk("AP_Zephyr: out of IO processes\n");
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc proc, uint32_t period_us)
{
    _failsafe_proc      = proc;
    _failsafe_period_us = period_us;
}

/* ─── system initialisation ──────────────────────────────────────────── */

void Scheduler::set_system_initialized()
{
    _initialized = true;
}

bool Scheduler::is_system_initialized()
{
    return _initialized;
}

/* ─── reboot ─────────────────────────────────────────────────────────── */

void Scheduler::reboot(bool hold_in_bootloader)
{
    /* Same sequence as AP_HAL_ChibiOS/Scheduler.cpp reboot(). */

    // disarm motors to ensure they are off during a bootloader upload
    hal.rcout->force_safety_on();

#if HAL_WITH_IO_MCU && AP_ZEPHYR_IOMCU_ENABLED
    if (AP_BoardConfig::io_enabled()) {
        iomcu.shutdown();
    }
#endif

#if HAL_LOGGING_ENABLED
    // stop logging
    if (AP_Logger::get_singleton()) {
        AP::logger().StopLogging();
    }

    // unmount filesystem, if active
    AP::FS().unmount();
#endif
#if defined(CONFIG_SOC_MIMXRT1176_CM7)
    /* The app reprograms FlexRAM to 15 ITCM / 1 DTCM banks (480/32 KB). */
    *(volatile uint32_t *)0x400AC040u &= ~(1u << 2);
#endif
#if defined(CONFIG_SOC_SERIES_IMXRT11XX)
    if (hold_in_bootloader) {
        /* Ask the bootloader to stay put instead of chain-loading the app. */
        rt1176_snvs_set_boot_signature(0xb0070001u);
    }
#endif
    // disable all interrupt sources, as ChibiOS's port_disable()
    (void)irq_lock();

    sys_reboot(hold_in_bootloader ? SYS_REBOOT_COLD : SYS_REBOOT_WARM);
    for (;;) {}
}

/* ─── threading ──────────────────────────────────────────────────────── */

bool Scheduler::in_main_thread() const
{

    return k_current_get() == _main_tid;
}

bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name,
                               uint32_t stack_size, priority_base base,
                               int8_t priority)
{

    for (uint8_t i = 0; i < ZEPHYR_MAX_USER_THREADS; i++) {
        if (_user_threads[i].in_use) {
            continue;
        }
        if (stack_size > ZEPHYR_USER_THREAD_STACK_SZ) {
            /* Refuse, do not cap. ChibiOS allocates the REQUESTED size
               (thread_create_alloc) and returns false if it cannot, so a
               caller either gets the stack it asked for or a clear failure.
               This used to hand back a smaller stack and return true: Lua
               asks for 17408 B and was given 8192, then ran believing it had
               the larger one. The failure mode for that is a stack overflow
               at some unrelated later moment, which is far harder to diagnose
               than a refused thread at startup.

               The pool's stacks are a compile-time size, so the fix for a
               genuine need is to raise ZEPHYR_USER_THREAD_STACK_SZ, not to
               let the caller proceed on a stack that is too small. */
            printk("AP_Zephyr: thread '%s' needs %u B of stack, pool slots are "
                   "%u B - refusing (raise ZEPHYR_USER_THREAD_STACK_SZ)\n",
                   name, (unsigned)stack_size,
                   (unsigned)ZEPHYR_USER_THREAD_STACK_SZ);
            return false;
        }
        _user_threads[i].proc   = proc;
        _user_threads[i].in_use = true;
        const int prio = _zephyr_priority(base, priority);
        if (prio == APM_MAIN_PRIORITY) {
            /* Legal on ChibiOS too (MAIN+0), but a thread that never blocks at
               main's own level starves the flight loop once timeslicing is off,
               and rotated with it in 20 ms slices while it was on. */
            printk("AP_Zephyr: thread '%s' created at main's priority %d\n", name, prio);
        }
        k_thread_create(&_user_threads[i].thread_data,
                        _zephyr_user_stacks[i], ZEPHYR_USER_THREAD_STACK_SZ,
                        _user_thread_fn, &_user_threads[i].proc,
                        nullptr, nullptr,
                        prio, 0, K_NO_WAIT);
        k_thread_name_set(&_user_threads[i].thread_data, name);
        return true;
    }
    printk("AP_Zephyr: thread pool full, cannot create '%s'\n", name);
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    return false;
}

/* ─── interrupt control ──────────────────────────────────────────────── */

void *Scheduler::disable_interrupts_save()
{

    unsigned int key = irq_lock();
    return (void *)(uintptr_t)key;
}

void Scheduler::restore_interrupts(void *ctx)
{

    irq_unlock((unsigned int)(uintptr_t)ctx);
}

/* ─── private helpers (Zephyr only) ──────────────────────────────────── */


void Scheduler::_run_timer_procs()
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    /* CORRECTED 2026-08-10 (was unconditional): patting the watchdog on every timer
     * tick defeats it - it must only be patted when the main loop is alive. */
    if (in_expected_delay()) {
        watchdog_pat();
    }

    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_procs[i]) {
            _timer_procs[i]();
        }
    }

    if (_failsafe_proc && _failsafe_period_us) {
        uint64_t now_us = AP_HAL::micros64();
        if ((now_us - _last_failsafe_us) >= _failsafe_period_us) {
            _last_failsafe_us = now_us;
            _failsafe_proc();
        }
    }

    _in_timer_proc = false;
}

void Scheduler::_run_io()
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_procs[i]) {
            _io_procs[i]();
        }
    }

#if HAL_LOGGING_ENABLED
    /* retry SD card mount every 3s when disarmed */
    static uint32_t last_sd_retry_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    if (!hal.util->get_soft_armed() && (now_ms - last_sd_retry_ms) > 3000) {
        last_sd_retry_ms = now_ms;
        AP::FS().retry_mount();
    }
#endif

    _in_io_proc = false;
}

void Scheduler::_timer_thread_fn(void *arg, void *, void *)
{
    // this is a DEV_PRINTF style output thats only active on --debug
    BOOT_TRACE("AP: timer thread started\n");
    Scheduler *sched = static_cast<Scheduler *>(arg);
    while (true) {
        k_sem_take(&s_timer_sem, K_FOREVER);
        /* _hal_initialized, not _initialized: ChibiOS's timer thread waits
           only for the HAL and then runs the timer procs through setup(),
           where the sensor drivers that register them are being probed.
           Gating on _initialized held every timer proc back until setup()
           had returned. */
        if (sched->_hal_initialized) {
            sched->_run_timer_procs();
        }
        /* as ChibiOS: pat while a delay is expected, which includes all of
           init until set_system_initialized() */
        if (sched->in_expected_delay()) {
            sched->watchdog_pat();
        }
    }
}

void Scheduler::_io_thread_fn(void *arg, void *, void *)
{
    BOOT_TRACE("AP: io thread started\n");
    Scheduler *sched = static_cast<Scheduler *>(arg);
    uint32_t iter = 0;
    uint32_t last_print_ms = 0;
    while (true) {
        k_sem_take(&s_io_sem, K_FOREVER);
        iter++;
        const uint32_t now_ms = k_uptime_get_32();
#ifdef CONFIG_AP_SCHED_TRACE
        if (iter <= 5 || (now_ms - last_print_ms) >= 1000) {
            last_print_ms = now_ms;
            printk("AP_io: iter=%u initialized=%d\n", (unsigned)iter, (int)sched->_initialized);
        }
#else
        (void)now_ms;
        (void)last_print_ms;
#endif
        /* _hal_initialized, not _initialized: ChibiOS's io thread waits only for
           the HAL to be up and then runs IO continuously, which is what lets
           AP_Param::save_queue drain during setup(). Gating on the
           system-initialized flag instead meant nothing drained that queue
           until after init, so set_and_save() during init spun - which is why
           set_system_initialized() used to be called before setup(). */
        if (sched->_hal_initialized) {
            sched->_run_io();
        }

#if defined(CONFIG_AP_CHAIN_PROFILE) || HAL_ENABLE_THREAD_STATISTICS
        /* Render @SYS/threads.txt and @SYS/tasks.txt into g_ap_sysinfo (and the
           per-file views g_threads_txt / g_tasks_txt) for SWD readback. Also
           on every --enable-stats build, not only chain-profile ones: that is
           the build whose threads.txt carries per-thread load, so it is the
           one anyone reads over a debugger, and without this call nothing
           references the buffer and the linker discards it. Ordinary flight
           builds keep the 8 KB - which is why this tests the macro's VALUE:
           AP_HAL_Boards.h defines HAL_ENABLE_THREAD_STATISTICS as 0 when
           --enable-stats is off, so defined() was true on every build and a
           plain flight image lost 8 KB of heap (PM.Mem, 2026-09-12). */
        static uint32_t last_sysinfo_ms;
        if (now_ms - last_sysinfo_ms >= 2000) {
            last_sysinfo_ms = now_ms;
            ap_sysinfo_capture();
        }
#endif
    }
}

void Scheduler::_monitor_thread_fn(void *arg, void *, void *)
{
    BOOT_TRACE("AP: monitor thread started\n");
    Scheduler *sched = static_cast<Scheduler *>(arg);
    bool warned = false;
    uint32_t last_stack_check_ms = 0;

#if defined(HAVE_HW_WATCHDOG)
    /* Arm the independent hardware watchdog and feed it from this thread.
       It fires only if the monitor thread itself stops running for
       HW_WDT_TIMEOUT_MS — i.e. total starvation (IRQ storm) that the
       software main-loop watchdog cannot catch. Debugger halts freeze it
       (WDT_OPT_PAUSE_HALTED_BY_DBG), so BMP/GDB sessions don't reset. */
    const struct device *wdt = DEVICE_DT_GET(HW_WATCHDOG_NODE);
    int wdt_channel = -1;
    /* Armed BELOW, once the system is initialised - not here.
       AP_BoardConfig::watchdog_enabled() reads BRD_OPTIONS, and this thread
       starts long before parameters are loaded, so sampling it here always saw
       the compiled-in default. On a stock config that default has no watchdog
       bit, so the hardware watchdog was never armed on any board no matter
       what the operator had set. ChibiOS evaluates it in
       HAL_ChibiOS_Class.cpp, after g_callbacks->setup() has returned and the
       parameters are up; _initialized is true at exactly that point.

       ChibiOS parity: the watchdog is opt-in via BRD_OPTIONS. With no
       AP_BoardConfig singleton - any Tools/ target, which never creates one -
       watchdog_enabled() is HAL_WATCHDOG_ENABLED_DEFAULT, false. That is how
       ChibiOS keeps a tool from being reset by a facility meant to catch a
       hung flight loop. */
    bool wdt_decided = false;
    /* set when the main loop is judged gone: stops the feed above so the
       hardware watchdog does the reset. */
    bool wdt_starve = false;
#endif

    uint32_t lr_last_ms = 0, lr_last_count = 0;
    uint32_t last_cpu_report_ms = 0;

    while (true) {
        k_msleep(100);

#if defined(HAVE_HW_WATCHDOG)
        /* Decide once, after the parameters are up. _initialized becomes true
           when callbacks->setup() returns, which is where ChibiOS makes the
           same call. */
        if (!wdt_decided && sched->_initialized) {
            wdt_decided = true;
            if (AP_BoardConfig::watchdog_enabled() && device_is_ready(wdt)) {
                struct wdt_timeout_cfg wcfg = {};
                wcfg.window.min = 0U;
                wcfg.window.max = HW_WDT_TIMEOUT_MS;
                s_wdt_sched = sched;
                wcfg.callback = ap_wdt_stall_cb;
                wcfg.flags = WDT_FLAG_RESET_SOC;
                wdt_channel = wdt_install_timeout(wdt, &wcfg);
                /* Prefer freeze-on-debug (STM32 supports it); fall back to no
                   options for SoCs whose driver rejects it (NXP imx-wdog). */
                if (wdt_channel >= 0 &&
                    wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG) < 0 &&
                    wdt_setup(wdt, 0) < 0) {
                    wdt_channel = -1;
                }
                if (wdt_channel < 0) {
                    printk("AP_Zephyr: hardware watchdog unavailable\n");
                } else {
                    printk("AP_Zephyr: hardware watchdog armed (%u ms)\n",
                           (unsigned)HW_WDT_TIMEOUT_MS);
                }
            } else if (!device_is_ready(wdt)) {
                /* Silent before this: device_is_ready()==false skipped both
                   printks, so a missing device looked identical to a working
                   one. */
                printk("AP_Zephyr: hardware watchdog device not ready (check "
                       "CONFIG_WATCHDOG)\n");
            }
        }
#endif

        /* Report the fault record that survived the last reset, once, over
           MAVLink. Needed because this board's console is USB CDC, and
           uart_poll_out() on a CDC endpoint cannot transmit from the halted,
           irq-locked fault context - so k_sys_fatal_error_handler()'s own
           "### FATAL ERROR ### pc=..." dump never leaves the board. The
           persistent-data WDG statustext is no substitute: it records only the
           FIRST fault of a boot and carried neither PC nor CFSR in practice.
           Reported from here rather than at init because the GCS link has to
           be up for a STATUSTEXT to go anywhere. */
        /* REPEATED, not sent once: the first pass through here is ~100 ms into
           the boot, long before a GCS has attached, and a STATUSTEXT with no
           listener is simply dropped. Sending it once lost every report on a
           board that reboots every minute - AP's own WDG line only reaches the
           screen because send_watchdog_reset_statustext() repeats it. Take the
           record on the first pass so nothing can overwrite it, then re-send
           for a minute. */
        {
            static bool record_taken;
            static bool have_fault;
            static unsigned int reason;
            static uint32_t pc, lr, cfsr, icsr, prio, count;
            static bool have_wdg;
            static uint32_t stall_ms, cur_prio, main_state, main_pended, main_pc;
            static int32_t sched_task;
            static char cur_name[12];
            static uint8_t sends_left;
            static uint32_t last_send_ms;

            if (!record_taken) {
                record_taken = true;
                have_fault = ap_fault_record_take(&reason, &pc, &lr, &cfsr, &icsr,
                                                  &prio, &count);
                have_wdg = ap_wdg_record_take(&stall_ms, &sched_task, &cur_prio,
                                              cur_name, sizeof(cur_name),
                                              &main_state, &main_pended, &main_pc);
                if (have_fault || have_wdg) {
                    sends_left = 6;   /* six tries over a minute */
                }
            }
            const uint32_t now_ms = AP_HAL::millis();
            if (sends_left > 0 && (last_send_ms == 0 || now_ms - last_send_ms > 10000)) {
                last_send_ms = now_ms;
                sends_left--;
                if (have_fault) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                                  "FAULT r=%u pc=%08lx lr=%08lx",
                                  reason, (unsigned long)pc, (unsigned long)lr);
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                                  "FAULT cfsr=%08lx icsr=%08lx pri=%ld n=%lu",
                                  (unsigned long)cfsr, (unsigned long)icsr,
                                  (long)(int32_t)prio, (unsigned long)count);
                }
                if (have_wdg) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                                  "WSTALL %lums in %s pri=%ld task=%ld",
                                  (unsigned long)stall_ms, cur_name,
                                  (long)(int32_t)cur_prio, (long)sched_task);
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                                  "WSTALL main state=%02lx pended=%08lx pc=%08lx",
                                  (unsigned long)main_state,
                                  (unsigned long)main_pended,
                                  (unsigned long)main_pc);
                }
            }
        }

        /* GPIO ISR-flood quota refill + disabled-pin retry, ChibiOS parity
           (AP_HAL_ChibiOS/GPIO.cpp's own timer_tick(), same 100ms cadence
           from its own monitor thread). See GPIO.h's override comment. */
        hal.gpio->timer_tick();

#if defined(HAVE_HW_WATCHDOG)
        /* Persistent crash/watchdog forensics, ChibiOS parity. */
        if (wdt_channel >= 0) {
            /* FILL THE WDG LINE'S SPARE FIELDS EVERY PASS, not at the moment a
               stall is detected. The freeze this is chasing stops every thread -
               the monitor included - so code that runs "when it goes wrong"
               never runs at all, which is why a 500 ms recorder and a watchdog
               pre-reset interrupt both produced nothing. What DOES survive is
               the snapshot taken up to 100 ms earlier, because AP repeats the
               WDG statustext on the next boot from restored persistent data.
               So: sample continuously and read the last one.

               Only while fault_type is 0. A real fault fills these same fields
               with better information and must not be overwritten. */
            AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
            if (pd.fault_type == 0) {
                /* FL and FICSR are written by the flash path itself now, via
                   ap_persistent_flash_mark() - see there for why sampling them
                   here was useless. Left alone so this does not clobber them. */
                /* FA: where the main thread is, from its own stacked frame.
                   FLR: the wait queue it is blocked on, 0 if it is runnable. */
                uint32_t main_pc = 0, main_pended = 0;
                if (s_main_tid != nullptr) {
                    main_pended = (uint32_t)(uintptr_t)s_main_tid->base.pended_on;
                    const uint32_t psp = (uint32_t)s_main_tid->callee_saved.psp;
                    if (psp >= 0x20000000u && psp < 0x20400000u) {
                        main_pc = ((const uint32_t *)(uintptr_t)psp)[6];
                    }
                }
                pd.fault_addr = main_pc;
                pd.fault_lr = main_pended;
            }
            sched->save_persistent_data();
        }
#endif

        {
            /* 0.1 Hz loop-rate report. Stopwatch-verifiable: dt_ms should read
               ~10000. the target is loop_hz > 400. */
            const uint32_t now_ms = AP_HAL::millis();
            if (lr_last_ms == 0) {
                lr_last_ms = now_ms;
            } else if (now_ms - lr_last_ms >= 10000U) {
                const uint32_t dt = now_ms - lr_last_ms;
                const uint32_t hz = (g_ap_loop_count - lr_last_count) * 1000U / dt;
                printk("LOOPRATE dt_ms=%lu loop_hz=%lu boost=%lu reassert=%lu leaks=%lu\n",
                       (unsigned long)dt, (unsigned long)hz,
                       (unsigned long)g_ap_boost_count, (unsigned long)g_ap_main_prio_reasserts,
                       (unsigned long)g_ap_main_prio_leaks);
                /* Per-port byte counters for the telem ports (SERIAL1/2),
                   so "is this port actually moving bytes" is answerable
                   without a debugger - see the counters in UARTDriver.h. */
                for (uint8_t sn = 1; sn <= 3; sn++) {   /* 3 = GPS on this board */
                    auto *u = static_cast<Zephyr::UARTDriver *>(hal.serial(sn));
                    if (u == nullptr) {
                        continue;
                    }
                    /* async= is the DMA path flag. The four counters after it
                       are TX ONLY (_dbg_tx_*), so a port that never transmits
                       shows queued=0 dma=0 done=0 and that says nothing about
                       whether RX is on DMA - which is exactly how it was
                       misread once. rx/rxev come only from _async_cb's
                       UART_RX_RDY, so non-zero rx PROVES the async path. */
                    printk("UARTSTAT s%u async=%u queued=%lu dma=%lu done=%lu fail=%lu "
                           "rx=%lu rxev=%lu\n", sn, (unsigned)u->is_dma_enabled(),
                           (unsigned long)u->_dbg_tx_queued,
                           (unsigned long)u->_dbg_tx_dma,
                           (unsigned long)u->_dbg_tx_done,
                           (unsigned long)u->_dbg_tx_fail,
                           (unsigned long)u->_dbg_rx_bytes,
                           (unsigned long)u->_dbg_rx_events);
                }
                /* rate-thread frequency, published by rate_thread.cpp and
                   printed HERE because only main may risk the console
                   poll_out spin (see the comment at the publisher) */
                if (ap_zephyr_rate_thread_hz != 0) {
                    printk("RATETHREAD rate_hz=%lu span_pct=%lu maxdt_us=%lu qmax=%lu\n",
                           (unsigned long)ap_zephyr_rate_thread_hz,
                           (unsigned long)ap_zephyr_rate_thread_busy_pct,
                           (unsigned long)ap_zephyr_rate_thread_maxdt_us,
                           (unsigned long)ap_zephyr_rate_thread_qdepth);
                    ap_zephyr_rate_thread_qdepth = 0;
                }
                /* Also over MAVLink: the console is LPUART1 with no bridge on some boards, so a
                 * console-only message would be invisible. */
                /* b/r: boosts and priority re-asserts in THIS interval (a
                   healthy loop shows r = 2b - one per HAL semaphore held across
                   the boost); leaks: total loops that began with main off its
                   intended level, must stay 0. Deltas, not totals, because a
                   STATUSTEXT is 50 characters and the totals overflowed it on
                   silicon within a minute. The printk above keeps the totals. */
                static uint32_t lr_last_boost, lr_last_reassert, lr_last_hold;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LoopRate: %lu Hz b=%lu r=%lu leaks=%lu i=%lu",
                              (unsigned long)hz,
                              (unsigned long)(g_ap_boost_count - lr_last_boost),
                              (unsigned long)(g_ap_main_prio_reasserts - lr_last_reassert),
                              (unsigned long)g_ap_main_prio_leaks,
                              (unsigned long)(g_ap_main_prio_inherit_holds - lr_last_hold));
                /* I2C health over MAVLink as well as the console I2CERR line:
                   the compass and both baros sit on I2C, and when they report
                   Bad Health the question is always the same - did the device
                   NAK (absent part, or an unpowered vehicle rail) or did the
                   controller time out (bus/driver fault). The console is held by
                   whatever is attached to USB CDC, so a printk alone is not
                   readable while a GCS is connected. One line per bus that saw
                   traffic, kept short because a STATUSTEXT is 50 characters. */
                for (uint8_t b = 0; b < 3; b++) {
                    if ((g_i2c_ok[b] | g_i2c_nak[b] | g_i2c_timeout[b]) == 0) {
                        continue;
                    }
                    /* Label is b, NOT b+1: the slot index IS the AP bus number
                       (I2CDevice uses slot = _bus, and hwdef declares I2C:1 and
                       I2C:2), so b+1 printed bus 1 as "I2C2". The bus thread's
                       own callback count and mean are NOT repeated here - a
                       STATUSTEXT is 50 characters and the BUSCB printk below
                       already carries them per bus. */
                    /* xfer = mean microseconds of a SUCCESSFUL transfer, mx = the
                       worst one. This is the number that splits the two
                       explanations for a 24-29 ms callback with zero failures:
                       a large xfer means the transfer itself is slow (driver /
                       DMA completion), while a small xfer with a low cb count
                       means the bus thread is simply not being run. */
                    const uint32_t okn = g_i2c_ok[b];
                    const uint32_t okmean = (okn != 0U)
                        ? (uint32_t)(g_i2c_okus[b] / okn) : 0U;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                  "I2C%u ok=%lu oth=%lu xfer=%luus mx=%lu",
                                  (unsigned)b,
                                  (unsigned long)okn,
                                  (unsigned long)g_i2c_otherr[b],
                                  (unsigned long)okmean,
                                  (unsigned long)g_i2c_maxus[b]);
                    if (g_i2c_nak[b] || g_i2c_timeout[b] || g_i2c_reset[b]) {
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                      "I2C%u FAIL nak=%lu to=%lu rst=%lu e=%ld",
                                      (unsigned)b,
                                      (unsigned long)g_i2c_nak[b],
                                      (unsigned long)g_i2c_timeout[b],
                                      (unsigned long)g_i2c_reset[b],
                                      (long)g_i2c_lasterr[b]);
                    }
                    /* Sole owner of the reset - see the note in the I2CERR
                       printk block below. */
                    g_i2c_ok[b] = 0; g_i2c_nak[b] = 0; g_i2c_timeout[b] = 0;
                    g_i2c_otherr[b] = 0; g_i2c_reset[b] = 0; g_i2c_maxus[b] = 0;
                    g_i2c_okus[b] = 0;
                }
                lr_last_hold = g_ap_main_prio_inherit_holds;
                lr_last_boost = g_ap_boost_count;
                lr_last_reassert = g_ap_main_prio_reasserts;
                lr_last_count = g_ap_loop_count;
                lr_last_ms = now_ms;
            }
        }

#if defined(HAVE_HW_WATCHDOG)
        /* Feed each iteration while the loop is healthy: the hardware
           watchdog guards "is the monitor thread alive", independent of the
           software main-loop checks below. Once wdt_starve is set we stop,
           and the watchdog reboots the SoC for us - see the stall handling
           further down for why that matters. */
        if (wdt_channel >= 0 && !wdt_starve) {
            wdt_feed(wdt, wdt_channel);
        }
#endif

        if (!sched->_initialized) {
            sched->watchdog_pat();
            warned = false;
            continue;
        }

        if (sched->in_expected_delay()) {
            warned = false;
            continue;
        }

        uint32_t now     = (uint32_t)k_uptime_get_32();

        /* Subsystem-declared stand-down (e.g. WiFi radio start, whose USB
           Serial-JTAG re-enumeration makes console writes stall every
           thread including main). expect_delay_ms() can't cover this: it
           is a MAIN-thread facility, and these stalls are inflicted ON
           main from elsewhere. */
        if (ap_zephyr_grace_until_ms != 0 &&
            (int32_t)(ap_zephyr_grace_until_ms - now) > 0) {
            sched->watchdog_pat();
            warned = false;
            continue;
        }

        uint32_t elapsed = now - sched->_last_watchdog_pat_ms;

        if (elapsed >= MONITOR_RESET_MS) {
            if (AP_HAL::_hal_zephyr_panicked) {
                /* Deliberate AP_HAL::panic() — keep the board up so the
                 * message stays readable on the console; keep the hardware
                 * watchdog fed. */
                sched->watchdog_pat();
                continue;
            }
            if (!AP_BoardConfig::watchdog_enabled()) {
                /* ChibiOS only resets when the watchdog is on; without that
                   guard a tool - or a vehicle with the BRD_OPTIONS bit clear -
                   is rebooted out from under itself for a legitimate delay. */
                sched->watchdog_pat();
                continue;
            }
            printk("AP_Zephyr: WATCHDOG main loop stuck %u ms — resetting\n",
                   (unsigned)elapsed);
#if defined(HAVE_HW_WATCHDOG)
            if (wdt_channel >= 0) {
                /* Let the HARDWARE watchdog do it, by ceasing to feed it.
                   sys_reboot() is a SOFTWARE reset, so the SoC came back
                   reporting RESET_SOFTWARE: hal.util->was_watchdog_reset()
                   stayed false, the persistent data from the crash was never
                   restored, no INTERNAL_ERROR(watchdog_reset) was raised, and
                   every piece of watchdog recovery in ArduPilot sat out the
                   one event it exists for. A watchdog reset has to look like
                   a watchdog reset.

                   This is how ChibiOS does it too - its monitor stops calling
                   watchdog_pat() and the IWDG expires. The reset lands within
                   HW_WDT_TIMEOUT_MS. */
                wdt_starve = true;
                continue;
            }
#endif
            /* No armed hardware watchdog - BRD_OPTIONS asked for one but the
               device was not ready, or this SoC has none. Fall back to the
               software reset so a stuck board still recovers, accepting that
               the reset cause will read RESET_SOFTWARE. */
            sys_reboot(SYS_REBOOT_COLD);

        } else if (elapsed >= MONITOR_WARN_MS && !warned) {
            printk("AP_Zephyr: WARNING main loop stuck %u ms\n",
                   (unsigned)elapsed);
            /* Record it where a reset cannot erase it. The printk above goes to
               the USB CDC console, which drops output when its ring fills or the
               host is not reading, so its absence proved nothing. Writing the
               record here - from the monitor thread, which is still running -
               also settles whether the monitor is alive during the stall at all:
               if the next boot reports a WSTALL, it was; if it never does, then
               nothing below interrupt level is running and main is not merely
               starved. */
            {
                k_tid_t cur = k_current_get();
                uint32_t main_state = 0, main_pended = 0, main_pc = 0;
                if (s_main_tid != nullptr) {
                    main_state = s_main_tid->base.thread_state;
                    main_pended = (uint32_t)(uintptr_t)s_main_tid->base.pended_on;
#if defined(CONFIG_ARM)
                    const uint32_t psp = (uint32_t)s_main_tid->callee_saved.psp;
                    if (psp >= 0x20000000u && psp < 0x20400000u) {
                        main_pc = ((const uint32_t *)(uintptr_t)psp)[6];
                    }
#endif
                }
                ap_wdg_record_put(elapsed,
                                  (int32_t)hal.util->persistent_data.scheduler_task,
                                  (uint32_t)(int32_t)k_thread_priority_get(cur),
                                  k_thread_name_get(cur),
                                  main_state, main_pended, main_pc);
            }
            /* ChibiOS raises this at the same 500 ms (its Scheduler.cpp:462), and
               it is what puts a non-zero IE/IEC on AP's repeating WDG line - the
               one channel that has reliably reached the screen all along. The
               Zephyr monitor was only printing. */
#if AP_INTERNALERROR_ENABLED
            AP::internalerror().error(AP_InternalError::error_t::main_loop_stuck,
                                      hal.util->persistent_data.semaphore_line);
#endif
            /* One-shot thread-state dump on the first stuck warning, so a hang is diagnosable
             * without attaching a debugger. */
#ifdef CONFIG_THREAD_MONITOR
            k_thread_foreach([](const struct k_thread *t, void *) {
                char sbuf[32];
                const char *nm = k_thread_name_get((k_tid_t)t);
                printk("  thr %-16s prio %3d %s\n",
                       nm ? nm : "?", t->base.prio,
                       k_thread_state_str((k_tid_t)t, sbuf, sizeof(sbuf)));
            }, nullptr);
#endif
            warned = true;
            try_force_mutex();

        } else if (elapsed < MONITOR_WARN_MS) {
            warned = false;
        }

#ifdef CONFIG_THREAD_RUNTIME_STATS
        /* Per-thread CPU, once per 10 s. Measured rather than inferred: which
           threads actually consume the core is the question that decides
           whether a starved thread needs a bigger share or the load needs to
           come down, and until now there was no number for it. One line per
           report, so it cannot flood the console. */
        if ((now - last_cpu_report_ms) >= 10000) {
            last_cpu_report_ms = now;
            k_thread_runtime_stats_t tot;
            /* Cycles consumed IN THIS WINDOW, not since boot: a cumulative
               percentage keeps reporting a load the board shed ten minutes
               ago, which is how a thread that stopped running can still look
               busy. Index 0 is the total; the rest follow the print order. */
            static uint64_t prev_cyc[1 + 7 + ZEPHYR_MAX_USER_THREADS];
            uint8_t slot = 0;
            if (k_thread_runtime_stats_all_get(&tot) == 0 &&
                tot.execution_cycles > prev_cyc[0]) {
                const uint64_t all = tot.execution_cycles - prev_cyc[0];
                prev_cyc[0] = tot.execution_cycles;
                char line[460];
                int n = snprintf(line, sizeof(line), "CPU%%");
                const struct { struct k_thread *t; const char *nm; } who[] = {
                    { s_main_tid,             "main" },
                    { &sched->_timer_thread_data,   "tmr" },
                    { &sched->_io_thread_data,      "io"  },
                    { &sched->_rcin_thread_data,    "rcin"},
                    { &sched->_rcout_thread_data,   "rcout"},
                    { &sched->_storage_thread_data, "stor"},
                    { &sched->_monitor_thread_data, "mon" },
                };
                for (uint8_t i = 0; i < ARRAY_SIZE(who) && n > 0 && n < (int)sizeof(line); i++) {
                    k_thread_runtime_stats_t st;
                    slot++;
                    if (who[i].t != nullptr &&
                        k_thread_runtime_stats_get(who[i].t, &st) == 0) {
                        /* State as well as load: a thread at 0% is either
                           blocked on something that never comes or ready and
                           never scheduled, and those need opposite fixes. */
                        char sb[16];
                        const char *stt = k_thread_state_str(who[i].t, sb, sizeof(sb));
                        const uint64_t d = st.execution_cycles - prev_cyc[slot];
                        prev_cyc[slot] = st.execution_cycles;
                        n += snprintf(line + n, sizeof(line) - n, " %s=%u/%s",
                                      who[i].nm, (unsigned)((d * 100U) / all),
                                      stt ? stt : "?");
                    }
                }
                for (uint8_t i = 0; i < ZEPHYR_MAX_USER_THREADS && n > 0 && n < (int)sizeof(line); i++) {
                    if (!sched->_user_threads[i].in_use) {
                        continue;
                    }
                    k_thread_runtime_stats_t st;
                    slot++;
                    if (k_thread_runtime_stats_get(&sched->_user_threads[i].thread_data, &st) == 0) {
                        const char *nm = k_thread_name_get(&sched->_user_threads[i].thread_data);
                        char sb[16];
                        const char *stt = k_thread_state_str(&sched->_user_threads[i].thread_data,
                                                             sb, sizeof(sb));
                        const uint64_t d = st.execution_cycles - prev_cyc[slot];
                        prev_cyc[slot] = st.execution_cycles;
                        n += snprintf(line + n, sizeof(line) - n, " %s=%u/%s",
                                      nm ? nm : "usr", (unsigned)((d * 100U) / all),
                                      stt ? stt : "?");
                    }
                }
                printk("%s\n", line);
            }
            for (uint8_t b = 0; b < DLY_BUCKETS; b++) {
                if (g_dly_calls[b] == 0) {
                    continue;
                }
                static const char *bn[DLY_BUCKETS] = { "p<=3", "p4-9", "p>=10" };
                printk("DELAYPROF %s n=%lu req=%luus mean=%luus max=%luus x%lu.%02lu\n",
                       bn[b], (unsigned long)g_dly_calls[b],
                       (unsigned long)(g_dly_req_us[b]/g_dly_calls[b]),
                       (unsigned long)(g_dly_act_us[b]/g_dly_calls[b]),
                       (unsigned long)g_dly_max_us[b],
                       (unsigned long)(g_dly_req_us[b] ? g_dly_act_us[b]/g_dly_req_us[b] : 0),
                       (unsigned long)(g_dly_req_us[b] ? (g_dly_act_us[b]*100/g_dly_req_us[b])%100 : 0));
                g_dly_calls[b] = 0; g_dly_req_us[b] = 0;
                g_dly_act_us[b] = 0; g_dly_max_us[b] = 0;
            }
            /* EVERY thread, by name, not a fixed list. The CPU% line above
               names a hand-written set plus _user_threads, which together came
               to 78-85% while CPUACCT measured threads=100% - so 15-22% was in
               threads never being reported. The DeviceBus threads (SPI1..3,
               I2C1..3) are created with k_thread_create directly rather than
               through thread_create, so they are in neither list, and they run
               at prio 2, ABOVE main. */
            {
                static uint64_t prev_each[24];
                static uintptr_t known[24];
                k_thread_runtime_stats_t tota;
                if (k_thread_runtime_stats_all_get(&tota) == 0) {
                    struct ea { uint64_t *prev; uintptr_t *known; uint64_t all; char *buf; int off; int cap; };
                    static char eline[420];
                    int eoff = snprintf(eline, sizeof(eline), "CPUALL");
                    static uint64_t prev_tot;
                    const uint64_t dall = (prev_tot && tota.execution_cycles > prev_tot)
                                          ? tota.execution_cycles - prev_tot : 0;
                    prev_tot = tota.execution_cycles;
                    if (dall > 0) {
                        struct ea acc { prev_each, known, dall, eline, eoff, (int)sizeof(eline) };
                        k_thread_foreach_unlocked([](const struct k_thread *th, void *ud) {
                            auto *e = (struct ea *)ud;
                            k_thread_runtime_stats_t st;
                            if (k_thread_runtime_stats_get((k_tid_t)th, &st) != 0) {
                                return;
                            }
                            int slot = -1;
                            for (int i = 0; i < 24; i++) {
                                if (e->known[i] == (uintptr_t)th) { slot = i; break; }
                                if (e->known[i] == 0) { e->known[i] = (uintptr_t)th; slot = i; break; }
                            }
                            if (slot < 0) { return; }
                            const uint64_t d = (st.execution_cycles >= e->prev[slot])
                                               ? st.execution_cycles - e->prev[slot] : 0;
                            e->prev[slot] = st.execution_cycles;
                            const unsigned pct = (unsigned)(d * 100U / e->all);
                            if (pct < 2U) { return; }
                            const char *nm = k_thread_name_get((k_tid_t)th);
                            if (e->off > 0 && e->off < e->cap - 1) {
                                e->off += snprintf(e->buf + e->off, e->cap - e->off,
                                                   " %s(p%d)=%u%%", nm ? nm : "?",
                                                   (int)th->base.prio, pct);
                            }
                        }, &acc);
                        printk("%s\n", eline);
                        /* The same top consumers over MAVLINK: a clean boot now
                           measures 310-325 Hz where it measured 423-443 Hz, and
                           the printk is unreadable while a GCS holds the USB CDC
                           console. Without this, "what is eating the CPU" is not
                           answerable at all from a connected GCS. */
                        {
                            struct top { char nm[10]; unsigned pct; int prio; } t1{{0},0,0}, t2{{0},0,0}, t3{{0},0,0};
                            struct tacc { top *a; top *b; top *c; uint64_t all; uint64_t *prev; uintptr_t *known; };
                            static uint64_t tprev[24]; static uintptr_t tknown[24];
                            static uint64_t tprev_tot;
                            k_thread_runtime_stats_t tt;
                            if (k_thread_runtime_stats_all_get(&tt) == 0 && tprev_tot != 0 &&
                                tt.execution_cycles > tprev_tot) {
                                const uint64_t dall = tt.execution_cycles - tprev_tot;
                                struct tacc ta { &t1, &t2, &t3, dall, tprev, tknown };
                                k_thread_foreach_unlocked([](const struct k_thread *th, void *ud) {
                                    auto *e = (struct tacc *)ud;
                                    k_thread_runtime_stats_t st;
                                    if (k_thread_runtime_stats_get((k_tid_t)th, &st) != 0) { return; }
                                    int slot = -1;
                                    for (int i = 0; i < 24; i++) {
                                        if (e->known[i] == (uintptr_t)th) { slot = i; break; }
                                        if (e->known[i] == 0) { e->known[i] = (uintptr_t)th; slot = i; break; }
                                    }
                                    if (slot < 0) { return; }
                                    const uint64_t d = (st.execution_cycles >= e->prev[slot])
                                                       ? st.execution_cycles - e->prev[slot] : 0;
                                    e->prev[slot] = st.execution_cycles;
                                    const unsigned pct = (unsigned)(d * 100U / e->all);
                                    const char *nm = k_thread_name_get((k_tid_t)th);
                                    top cand{{0}, pct, (int)th->base.prio};
                                    strncpy(cand.nm, nm ? nm : "?", sizeof(cand.nm) - 1);
                                    if (pct > e->a->pct)      { *e->c = *e->b; *e->b = *e->a; *e->a = cand; }
                                    else if (pct > e->b->pct) { *e->c = *e->b; *e->b = cand; }
                                    else if (pct > e->c->pct) { *e->c = cand; }
                                }, &ta);
                                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CPU %s%u=%u %s%u=%u %s%u=%u",
                                              t1.nm, t1.prio, t1.pct,
                                              t2.nm, t2.prio, t2.pct,
                                              t3.nm, t3.prio, t3.pct);
                            }
                            if (k_thread_runtime_stats_all_get(&tt) == 0) {
                                tprev_tot = tt.execution_cycles;
                            }
                        }
                    }
                }
            }
            /* IDLE vs ISR. A thread reported "queued" (runnable) while the
               idle thread is getting CPU would be a scheduler fault outright.
               If idle is 0 and the named threads do not add up to 100%, the
               missing time is interrupt context, which no thread can have.
               k_thread_runtime_stats does not attribute ISR time to any
               thread, so the only way to see it is total minus the sum over
               ALL threads - not the sum over the ones we happen to name. */
            {
                /* Own baseline for the total. prev_cyc[0] is NOT usable here:
                   the CPU% block above runs first in the same pass and has
                   already advanced it to the current total, so reusing it makes
                   the denominator ~0 and the percentages absurd. */
                static uint64_t prev_sum, prev_idle, prev_all;
                struct acc { uint64_t sum; uint64_t idle; } a { 0, 0 };
                k_thread_foreach_unlocked([](const struct k_thread *th, void *ud) {
                    auto *p = (struct acc *)ud;
                    k_thread_runtime_stats_t st;
                    if (k_thread_runtime_stats_get((k_tid_t)th, &st) != 0) {
                        return;
                    }
                    p->sum += st.execution_cycles;
                    const char *nm = k_thread_name_get((k_tid_t)th);
                    if (th->base.prio >= K_IDLE_PRIO ||
                        (nm != nullptr && strncmp(nm, "idle", 4) == 0)) {
                        p->idle += st.execution_cycles;
                    }
                }, &a);
                k_thread_runtime_stats_t tot2;
                if (k_thread_runtime_stats_all_get(&tot2) == 0 &&
                    a.sum >= prev_sum && tot2.execution_cycles > prev_all &&
                    prev_all != 0) {
                    const uint64_t all2 = tot2.execution_cycles - prev_all;
                    const uint64_t dsum = a.sum - prev_sum;
                    const uint64_t didle = (a.idle >= prev_idle) ? a.idle - prev_idle : 0;
                    /* Over MAVLINK too. The top-3 CPU line sums to only ~75%
                       while the loop rate fell ~30% with no thread showing a
                       rise, and k_thread_runtime_stats attributes NO interrupt
                       time to any thread - so the missing quarter is either idle
                       (the loop is waiting on something) or ISR context. Those
                       two have completely different fixes, and this line is the
                       only thing that separates them. */
                    /* sf: 0=SAFETY_DISARMED (pulses FORCED TO ZERO), 1=ARMED.
                       p1..p4: the pulse in us actually handed to pwm_set() for
                       motors 1-4. rc: the driver's return code, 0 = accepted. */
                    /* Per-submodule FlexPWM registers: the only direct
                       evidence of what the PINS do, as opposed to what the HAL
                       wrote. See ap_flexpwm_dump() in RCOutput.cpp - it reads
                       absolute i.MX RT11xx addresses, so it exists only there. */
#if defined(CONFIG_SOC_SERIES_IMXRT11XX)
                    ap_flexpwm_dump();
#endif
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RCOUT sf%u p%u,%u,%u,%u rc%d",
                           (unsigned)g_rcout_safety,
                           (unsigned)g_rcout_last_pulse[0], (unsigned)g_rcout_last_pulse[1],
                           (unsigned)g_rcout_last_pulse[2], (unsigned)g_rcout_last_pulse[3],
                           (int)g_rcout_last_rc[0]);
#if HAL_LOGGING_ENABLED
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LOGDIAG gap%lu snl%lu/%lu iot%lu n%lu",
                           (unsigned long)g_logdiag_gap_max,
                           (unsigned long)g_logdiag_snl_max,
                           (unsigned long)g_logdiag_snl_calls,
                           (unsigned long)g_logdiag_iot_max,
                           (unsigned long)g_logdiag_iot_calls);
                    g_logdiag_gap_max = 0; g_logdiag_snl_max = 0;
                    g_logdiag_iot_max = 0; g_logdiag_iot_calls = 0;
#endif
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ACCT thr%u idle%u isr%u",
                           (unsigned)(all2 ? dsum * 100U / all2 : 0),
                           (unsigned)(all2 ? didle * 100U / all2 : 0),
                           (unsigned)(all2 && dsum <= all2 ? (all2 - dsum) * 100U / all2 : 0));
                    printk("CPUACCT threads=%u%% idle=%u%% isr_or_unattributed=%u%%\n",
                           (unsigned)(all2 ? dsum * 100U / all2 : 0),
                           (unsigned)(all2 ? didle * 100U / all2 : 0),
                           (unsigned)(all2 && dsum <= all2 ? (all2 - dsum) * 100U / all2 : 0));
                }
                prev_sum = a.sum; prev_idle = a.idle;
                if (k_thread_runtime_stats_all_get(&tot2) == 0) {
                    prev_all = tot2.execution_cycles;
                }
            }
            /* I2CERR: which failure mode is costing the I2C bus threads their
               24-29 ms. nak means the device did not answer (absent part, or an
               unpowered rail - both baros and the compass sit on the vehicle
               rail); timeout means the controller never completed the transfer,
               which is a bus or driver fault, not a device one. reset counts the
               ap_lpi2c_hard_reset() after every attempt failed. */
            {
                char il[300];
                int io2 = snprintf(il, sizeof(il), "I2CERR");
                bool anyi = false;
                for (uint8_t b = 0; b < 3; b++) {
                    const uint32_t tot = g_i2c_ok[b] + g_i2c_nak[b] +
                                         g_i2c_timeout[b] + g_i2c_otherr[b];
                    if (tot == 0) {
                        continue;
                    }
                    anyi = true;
                    if (io2 > 0 && io2 < (int)sizeof(il) - 1) {
                        io2 += snprintf(il + io2, sizeof(il) - io2,
                                        " b%u ok=%lu nak=%lu to=%lu oth=%lu rst=%lu"
                                        " err=%ld max=%luus",
                                        (unsigned)(b + 1),
                                        (unsigned long)g_i2c_ok[b],
                                        (unsigned long)g_i2c_nak[b],
                                        (unsigned long)g_i2c_timeout[b],
                                        (unsigned long)g_i2c_otherr[b],
                                        (unsigned long)g_i2c_reset[b],
                                        (long)g_i2c_lasterr[b],
                                        (unsigned long)g_i2c_maxus[b]);
                    }
                    /* Deliberately does NOT zero: the I2C STATUSTEXT above owns
                       the reset. Two readers zeroing one counter set made the
                       STATUSTEXT report a shrinking fraction of each window
                       (ok=59 -> 26 -> 3) that looked exactly like the buses
                       dying, and was not. */
                }
                if (anyi) {
                    printk("%s\n", il);
                }
                /* Driver phase split: lk is the mean wait on the per-device
                   K_FOREVER lock, bb counts busy-bus rejections (-EBUSY, which
                   lands in the oth= bucket above), bbok is time spent in the
                   busy check when it passed. */
#if defined(CONFIG_I2C_MCUX_LPI2C)
                if (ap_lpi2c_stat_lock_n != 0U) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                  "LPI2C lk=%luus n=%lu bb=%lu bbok=%luus",
                                  (unsigned long)(ap_lpi2c_stat_lock_us /
                                                  ap_lpi2c_stat_lock_n),
                                  (unsigned long)ap_lpi2c_stat_lock_n,
                                  (unsigned long)ap_lpi2c_stat_busy_n,
                                  (unsigned long)ap_lpi2c_stat_bbok_us);
                    ap_lpi2c_stat_lock_us = 0; ap_lpi2c_stat_lock_n = 0;
                    ap_lpi2c_stat_busy_us = 0; ap_lpi2c_stat_busy_n = 0;
                    ap_lpi2c_stat_bbok_us = 0;
                }
#endif
            }
            /* BUSCB: SPI2 is 20% of the machine at PREEMPT(2), above main and
               above the whole p>=10 band, so it is the largest single block of
               CPU that could become free slack. calls tells us the rate, mean
               tells us the body cost. Counters live in DeviceBus.cpp. */
            {
                static const char *bnm[6] = { "SPI1", "SPI2", "SPI3",
                                              "I2C1", "I2C2", "I2C3" };
                char bl[300];
                int bo = snprintf(bl, sizeof(bl), "BUSCB");
                bool any = false;
                for (uint8_t i = 0; i < 6; i++) {
                    const uint32_t c = g_buscb_calls[i];
                    if (c == 0) {
                        continue;
                    }
                    const uint64_t t = g_buscb_us[i];
                    const uint32_t mx = g_buscb_max_us[i];
                    g_buscb_calls[i] = 0; g_buscb_us[i] = 0; g_buscb_max_us[i] = 0;
                    any = true;
                    if (bo > 0 && bo < (int)sizeof(bl) - 1) {
                        bo += snprintf(bl + bo, sizeof(bl) - bo,
                                       " %s n=%lu tot=%luus mean=%luus max=%luus",
                                       bnm[i], (unsigned long)c,
                                       (unsigned long)t,
                                       (unsigned long)(t / c),
                                       (unsigned long)mx);
                    }
                }
                if (any) {
                    printk("%s\n", bl);
                }
            }
            /* CPUBAND: where the CPU goes, split by position in the priority
               ladder, which is what says whether prio>=10 is starved by RANK or
               simply by a saturated machine.
                 main        main itself, its own column - lumping it into the
                             p<=6 band reported 97.5% and said nothing
                 hi_nonmain  the non-main p<=6 threads, which are the ones that
                             could take main's slack before prio>=10 sees it
                 mid         p 7..9 (the I2C baro/compass band)
                 lo          p>=10 (log_io, storage, the FTP worker)
               lo ~0 with hi_nonmain large -> rank IS the mechanism.
               lo ~0 with the window nearly all main -> capacity, not rank.
               lo substantial -> the band IS scheduled and FTP's problem is not
               starvation at all. */
            {
                static uint64_t prev_lo, prev_hi, prev_mid, prev_main, prev_all_b;
                struct bacc { uint64_t lo; uint64_t hi; uint64_t mid; uint64_t mn; } b { 0, 0, 0, 0 };
                k_thread_foreach_unlocked([](const struct k_thread *th, void *ud) {
                    auto *p = (struct bacc *)ud;
                    k_thread_runtime_stats_t st;
                    if (k_thread_runtime_stats_get((k_tid_t)th, &st) != 0) {
                        return;
                    }
                    if (th->base.prio >= K_IDLE_PRIO) {
                        return;         /* idle is in none of the bands */
                    }
                    /* main MUST be its own column. Lumping it into p<=6 made the
                       first run report hi=97.5%, which is mostly main itself and
                       says nothing about who takes the handback. The claim under
                       test is specifically that the NON-MAIN p<=6 threads absorb
                       main's handback before anything at p>=10 can be chosen. */
                    const char *nm = k_thread_name_get((k_tid_t)th);
                    if (nm != nullptr && strcmp(nm, "main") == 0) {
                        p->mn += st.execution_cycles;
                    } else if (th->base.prio >= 10) {
                        p->lo += st.execution_cycles;
                    } else if (th->base.prio <= 6) {
                        p->hi += st.execution_cycles;
                    } else {
                        p->mid += st.execution_cycles;
                    }
                }, &b);
                k_thread_runtime_stats_t tb;
                if (k_thread_runtime_stats_all_get(&tb) == 0 && prev_all_b != 0 &&
                    tb.execution_cycles > prev_all_b) {
                    const uint32_t cyc_per_us =
                        CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000U;
                    const uint64_t win = tb.execution_cycles - prev_all_b;
                    const uint64_t dlo = (b.lo >= prev_lo) ? b.lo - prev_lo : 0;
                    const uint64_t dhi = (b.hi >= prev_hi) ? b.hi - prev_hi : 0;
                    const uint64_t dmid = (b.mid >= prev_mid) ? b.mid - prev_mid : 0;
                    const uint64_t dmn = (b.mn >= prev_main) ? b.mn - prev_main : 0;
                    printk("CPUBAND main=%luus hi_nonmain=%luus "
                           "mid=%luus lo=%luus | win=%luus\n",
                           (unsigned long)(dmn / cyc_per_us),
                           (unsigned long)(dhi / cyc_per_us),
                           (unsigned long)(dmid / cyc_per_us),
                           (unsigned long)(dlo / cyc_per_us),
                           (unsigned long)(win / cyc_per_us));
                }
                prev_lo = b.lo; prev_hi = b.hi; prev_mid = b.mid; prev_main = b.mn;
                if (k_thread_runtime_stats_all_get(&tb) == 0) {
                    prev_all_b = tb.execution_cycles;
                }
            }
            /* The FTP worker's state WHILE PARKED is what separates the two
               explanations: Zephyr reports a mutex/semaphore waiter as
               "pending" with pended_on set, and a ready-but-starved thread as
               "queued" with pended_on null. */
            k_thread_foreach_unlocked([](const struct k_thread *th, void *) {
                const char *nm = k_thread_name_get((k_tid_t)th);
                if (nm == nullptr || strcmp(nm, "FTP") != 0) {
                    return;
                }
                char sb[16];
                printk("FTPTHREAD state=%s prio=%d pended_on=%p\n",
                       k_thread_state_str((k_tid_t)th, sb, sizeof(sb)),
                       (int)th->base.prio, (void *)th->base.pended_on);
            }, nullptr);
#if defined(CONFIG_SOC_SERIES_IMXRT11XX)
            ap_pcprofile_report();
#endif
        }
#endif

        /* stack health check once per second */
        if ((now - last_stack_check_ms) >= 1000) {
            last_stack_check_ms = now;
            sched->check_stack_free();
        }
    }
}

void Scheduler::_rcin_thread_fn(void *arg, void *, void *)
{
    BOOT_TRACE("AP: rcin thread started\n");
    Scheduler *sched = static_cast<Scheduler *>(arg);

    while (!sched->_hal_initialized) {
        k_msleep(20);
    }
    while (true) {
        k_msleep(1);
        ((RCInput *)hal.rcin)->_update();
    }
}

void Scheduler::_rcout_thread_fn(void *arg, void *, void *)
{
    BOOT_TRACE("AP: rcout thread started\n");
    Scheduler *sched = static_cast<Scheduler *>(arg);

    while (!sched->_hal_initialized) {
        k_msleep(20);
    }
    while (true) {
        k_msleep(1);
#if AP_ZEPHYR_DSHOT_ENABLED
        /* ~1kHz DShot frame stream (ChibiOS sends from its own rcout
           thread for the same reason: DShot ESCs disarm on signal loss,
           so frames must flow continuously, not only on write()).
           No-op until ArduPilot's output params select a DShot protocol
           via set_output_mode(). */
        ((RCOutput *)hal.rcout)->dshot_tick();
#endif
    }
}

void Scheduler::_storage_thread_fn(void *arg, void *, void *)
{
    BOOT_TRACE("AP: storage thread started\n");
    Scheduler *sched = static_cast<Scheduler *>(arg);

    while (!sched->_hal_initialized) {
        k_msleep(10);
    }
    BOOT_TRACE("AP_thread: AP_storage hal_initialized, entering tick loop\n");
    uint32_t tick = 0;
    while (true) {
        k_msleep(1);
        tick++;
#ifdef CONFIG_AP_SCHED_TRACE
        if (tick <= 5 || tick % 1000 == 0) {
            printk("AP_storage: tick=%u\n", (unsigned)tick);
        }
#endif
        hal.storage->_timer_tick();
    }
}

void Scheduler::_user_thread_fn(void *arg, void *, void *)
{
    AP_HAL::MemberProc *proc = static_cast<AP_HAL::MemberProc *>(arg);
    (*proc)();
}

/* Declared in Zephyr's kernel_internal.h, which is not on the application
   include path; both symbols are exported by the kernel (CONFIG_INIT_STACKS). */
extern "C" int z_stack_space_get(const uint8_t *stack_start, size_t size, size_t *unused_ptr);
K_KERNEL_STACK_ARRAY_DECLARE(z_interrupt_stacks, CONFIG_MP_MAX_NUM_CPUS, CONFIG_ISR_STACK_SIZE);

/* As AP_HAL_ChibiOS/Scheduler.cpp check_stack_free(): every thread AND the
   interrupt stack, and a stack_overflow INTERNAL ERROR - the thread priority
   as the "line number", 0xFFFF for the interrupt stack - so the condition
   reaches the GCS and the log rather than a printk nobody is watching.
   ChibiOS walks the thread registry; Zephyr has none without
   CONFIG_THREAD_MONITOR, so the threads are enumerated: the main thread, the
   HAL's own, every DeviceBus thread and the user threads. */
void Scheduler::check_stack_free()
{
    const auto report = [](const char *name, int prio, size_t unused) {
        printk("AP_Zephyr: stack LOW %s: only %u B free\n", name, (unsigned)unused);
#if AP_INTERNALERROR_ENABLED
        AP::internalerror().error(AP_InternalError::error_t::stack_overflow, (uint16_t)prio);
#endif
    };
    const auto check = [&report](struct k_thread *t, const char *name) {
        size_t unused = 0;
        if (t != nullptr && k_thread_stack_space_get(t, &unused) == 0 &&
            unused < MIN_STACK_FREE) {
            report(name, k_thread_priority_get(t), unused);
        }
    };

    check(_main_tid, "main");

    struct {
        struct k_thread *thd;
        const char      *name;
    } const known[] = {
        { &_timer_thread_data,   "timer"   },
        { &_io_thread_data,      "io"      },
        { &_monitor_thread_data, "monitor" },
        { &_rcin_thread_data,    "rcin"    },
        { &_rcout_thread_data,   "rcout"   },
        { &_storage_thread_data, "storage" },
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(known); i++) {
        check(known[i].thd, known[i].name);
    }

    for (DeviceBus *b = DeviceBus::first_bus(); b != nullptr; b = b->next) {
        check(b->thread(), "devbus");
    }

    for (uint8_t i = 0; i < ZEPHYR_MAX_USER_THREADS; i++) {
        if (_user_threads[i].in_use) {
            check(&_user_threads[i].thread_data, "user");
        }
    }

    // the interrupt stack, "line number" 0xFFFF as ChibiOS
    size_t unused = 0;
    // K_KERNEL_STACK_BUFFER + K_KERNEL_STACK_SIZEOF, as Zephyr's own
    // "kernel stacks" shell command reads the interrupt stack
    if (z_stack_space_get((const uint8_t *)K_KERNEL_STACK_BUFFER(z_interrupt_stacks[0]),
                          K_KERNEL_STACK_SIZEOF(z_interrupt_stacks[0]), &unused) == 0 &&
        unused < MIN_STACK_FREE) {
        report("isr", 0xFFFF, unused);
    }
}

void Scheduler::try_force_mutex()
{
    /* Zephyr has no ChibiOS-equivalent forced mutex release, so there is
       nothing to force - only report. Say what is actually known: main has not
       patted. It may be blocked, or legitimately busy, as any Tools/ target
       running a long measurement inside one loop() call is. Claiming a mutex
       deadlock here was misleading; so was promising a reset, which only
       happens when AP_BoardConfig::watchdog_enabled(). */
    printk("AP_Zephyr: main loop has not patted for over %u ms\n",
           (unsigned)MONITOR_WARN_MS);
}

int Scheduler::_zephyr_priority(priority_base base, int8_t offset)
{
    /* AP_HAL_ChibiOS/Scheduler.cpp:690-716 with the sign handled ONCE: ChibiOS ADDS
       the offset because a bigger number is more urgent there; Zephyr SUBTRACTS it
       because a smaller number is more urgent here. Nothing else differs: the table
       holds the same APM_* constants the HAL's own threads are created with (io
       uses the user base, see Scheduler.h), an unknown base becomes the io level
       with no offset, and the result is clamped to the preemptible band
       (ChibiOS: constrain(LOWPRIO, HIGHPRIO)). */
    static const struct { priority_base base; int8_t prio; } priority_map[] = {
        { PRIORITY_BOOST,     APM_MAIN_PRIORITY_BOOST },
        { PRIORITY_MAIN,      APM_MAIN_PRIORITY },
        { PRIORITY_SPI,       APM_SPI_PRIORITY },
        { PRIORITY_I2C,       APM_I2C_PRIORITY },
        { PRIORITY_CAN,       APM_CAN_PRIORITY },
        { PRIORITY_TIMER,     APM_TIMER_PRIORITY },
        { PRIORITY_RCOUT,     APM_RCOUT_PRIORITY },
        { PRIORITY_LED,       APM_LED_PRIORITY },
        { PRIORITY_RCIN,      APM_RCIN_PRIORITY },
        { PRIORITY_IO,        APM_IO_USER_PRIORITY },
        { PRIORITY_UART,      APM_UART_PRIORITY },
        { PRIORITY_STORAGE,   APM_STORAGE_PRIORITY },
        { PRIORITY_SCRIPTING, APM_SCRIPTING_PRIORITY },
        { PRIORITY_NET,       APM_NET_PRIORITY },
    };
    int prio = APM_IO_USER_PRIORITY;
    for (const auto &m : priority_map) {
        if (m.base == base) {
            prio = (int)m.prio - (int)offset;
            break;
        }
    }
    if (prio < APM_HIGHPRIO) { prio = APM_HIGHPRIO; }
    if (prio > APM_LOWPRIO)  { prio = APM_LOWPRIO;  }
    return prio;
}



#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
