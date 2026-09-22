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
#include "zephyr/src/ap_hooks.h"   /* ap_sysinfo_capture(), ap_persistent_save_fault() */

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

#if defined(__riscv)
/* RISC-V ONLY: restore Zephyr's MAX macro for the stack definitions below. */
#pragma push_macro("MAX")
#undef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
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

#if defined(__riscv)
/* restore AP_Math's MAX template for the rest of this file */
#pragma pop_macro("MAX")
#endif

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

void Scheduler::watchdog_pat()
{

    _last_watchdog_pat_ms = (uint32_t)k_uptime_get_32();
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
}

/* Crash-forensics bridge, called from the C fatal handler; declared in
   zephyr/src/ap_hooks.h. */
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
                wcfg.callback = nullptr;
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

        /* GPIO ISR-flood quota refill + disabled-pin retry, ChibiOS parity
           (AP_HAL_ChibiOS/GPIO.cpp's own timer_tick(), same 100ms cadence
           from its own monitor thread). See GPIO.h's override comment. */
        hal.gpio->timer_tick();

#if defined(HAVE_HW_WATCHDOG)
        /* Persistent crash/watchdog forensics, ChibiOS parity. */
        if (wdt_channel >= 0) {
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
                    printk("UARTSTAT s%u queued=%lu dma=%lu done=%lu fail=%lu "
                           "rx=%lu rxev=%lu\n", sn,
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
