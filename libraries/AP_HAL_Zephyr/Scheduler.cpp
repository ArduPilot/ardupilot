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
#include "UARTDriver.h"   /* UARTSTAT byte counters in the LOOPRATE report */

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

#ifdef CONFIG_AP_CHAIN_PROFILE
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

/* Defined in Util.cpp. Declared HERE at file scope, not inside the io thread
   function: an extern "C" linkage specification is only valid at namespace
   scope, and a plain `extern void ...;` written inside a Zephyr:: member would
   bind to Zephyr::ap_sysinfo_capture(), which is not what Util.cpp defines. */
extern "C" void ap_sysinfo_capture(void);
#endif

using namespace Zephyr;

/* ─── init ───────────────────────────────────────────────────────────── */

void Scheduler::init()
{

    _main_tid = k_current_get();
    _last_watchdog_pat_ms = (uint32_t)k_uptime_get_32();

    for (auto &t : _user_threads) {
        t.in_use = false;
    }

#if HAL_MONITOR_THREAD_ENABLED
    /* Monitor: priority 0 — always able to observe a stuck main thread */
    k_thread_create(&_monitor_thread_data, _zephyr_monitor_stack,
                    ZEPHYR_MONITOR_THREAD_STACK_SZ,
                    _monitor_thread_fn, this, nullptr, nullptr,
                    APM_MONITOR_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&_monitor_thread_data, "AP_monitor");

#endif

#ifndef HAL_NO_TIMER_THREAD
    /* Timer: priority 1 — 1000 Hz timer callbacks */
    k_thread_create(&_timer_thread_data, _zephyr_timer_stack,
                    ZEPHYR_TIMER_THREAD_STACK_SZ,
                    _timer_thread_fn, this, nullptr, nullptr,
                    APM_TIMER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&_timer_thread_data, "AP_timer");
#endif

#ifndef HAL_USE_EMPTY_IO
    /* IO: priority 5 — 1000 Hz IO callbacks, drains AP_Param save_queue */
    k_thread_create(&_io_thread_data, _zephyr_io_stack,
                    ZEPHYR_IO_THREAD_STACK_SZ,
                    _io_thread_fn, this, nullptr, nullptr,
                    APM_IO_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&_io_thread_data, "AP_io");
#endif

#if HAL_RCIN_THREAD_ENABLED
    /* RCIN: priority 4 — RC input polling at ~1 kHz */
    k_thread_create(&_rcin_thread_data, _zephyr_rcin_stack,
                    ZEPHYR_RCIN_THREAD_STACK_SZ,
                    _rcin_thread_fn, this, nullptr, nullptr,
                    APM_RCIN_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&_rcin_thread_data, "AP_rcin");
#endif

#ifndef HAL_NO_RCOUT_THREAD
    /* RCOUT: priority 4 — RC output at ~1 kHz */
    k_thread_create(&_rcout_thread_data, _zephyr_rcout_stack,
                    ZEPHYR_RCOUT_THREAD_STACK_SZ,
                    _rcout_thread_fn, this, nullptr, nullptr,
                    APM_RCOUT_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&_rcout_thread_data, "AP_rcout");
#endif

    /* Storage: priority 12 — low-priority flash/EEPROM writes */
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
    /* k_uptime_get() is always correct (kernel tick counter).
     * AP_HAL::micros64() relies on k_cyc_to_us_floor64() which returns 0 on
     * ESP32-S3 Xtensa for short durations (system timer at 16 MHz vs
     * CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=240 MHz mismatch). */
#ifdef CONFIG_AP_DELAY_CB_PROFILE
    const uint64_t prof_entry_us = AP_HAL::micros64();
#endif
    /* COUNTED loop: exactly ms iterations of a 1 ms sleep. Counting iterations rather
     * than comparing a deadline keeps it correct across a clock that quantises. */
    for (uint16_t i = 0; i < ms; i++) {
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

void Scheduler::delay_microseconds_boost(uint16_t us)
{
    /* No priority boost on Zephyr. Flipping the RUNNING main thread's priority races
     * with IRQ-driven ready-queue inserts - see the SCHED_MULTIQ note in prj.conf. */
    _called_boost = true;
#ifdef CONFIG_AP_MAIN_PRIORITY_BOOST
    /* Boost ONCE per loop, as ChibiOS does - cleared in boost_end(), which
       AP_AHRS::update() calls once per iteration. The guard matters: without it
       this flips priority on every delay rather than twice per loop.
       PREEMPT(2) stays BELOW the bus threads at PREEMPT(1), so IMU sample
       delivery still preempts the main loop. */
    if (!_priority_boosted && in_main_thread()) {
        k_thread_priority_set(k_current_get(), APM_MAIN_PRIORITY_BOOST);
        _priority_boosted = true;
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
#ifdef CONFIG_AP_MAIN_PRIORITY_BOOST
    /* Restore. Without this main stays above io (5) and storage (12)
       permanently, starving them - the failure the original attempt produced. */
    if (_priority_boosted && in_main_thread()) {
        _priority_boosted = false;
        k_thread_priority_set(k_current_get(), APM_MAIN_PRIORITY);
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

/* Crash-forensics bridge, called from the C fatal handler. */
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

#if AP_RCOUTPUT_ENABLED
    if (hal.rcout != nullptr) {
        hal.rcout->force_safety_on();
    }
#endif
#if HAL_LOGGING_ENABLED
    AP::logger().StopLogging();
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
            printk("AP_Zephyr: thread '%s' wants %u B stack, capped at %u B\n",
                   name, (unsigned)stack_size,
                   (unsigned)ZEPHYR_USER_THREAD_STACK_SZ);
        }
        _user_threads[i].proc   = proc;
        _user_threads[i].in_use = true;
        k_thread_create(&_user_threads[i].thread_data,
                        _zephyr_user_stacks[i], ZEPHYR_USER_THREAD_STACK_SZ,
                        _user_thread_fn, &_user_threads[i].proc,
                        nullptr, nullptr,
                        _zephyr_priority(base, priority), 0, K_NO_WAIT);
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
    printk("AP: timer thread started\n");
    Scheduler *sched = static_cast<Scheduler *>(arg);
    while (true) {
        k_sem_take(&s_timer_sem, K_FOREVER);
        if (sched->_initialized) {
            sched->_run_timer_procs();
        } else {
            /* pat watchdog during early init so the monitor doesn't reset us */
            sched->watchdog_pat();
        }
    }
}

void Scheduler::_io_thread_fn(void *arg, void *, void *)
{
    printk("AP: io thread started\n");
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
        if (sched->_initialized) {
            sched->_run_io();
        }

#ifdef CONFIG_AP_CHAIN_PROFILE
        /* Render @SYS/threads.txt and @SYS/tasks.txt into g_ap_sysinfo for SWD readback. */
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
    printk("AP: monitor thread started\n");
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
    /* ChibiOS parity (AP_HAL_ChibiOS/Scheduler.cpp): the watchdog is opt-in,
       via BRD_OPTIONS. With no AP_BoardConfig singleton - any Tools/ target,
       which never creates one - this is HAL_WATCHDOG_ENABLED_DEFAULT, false.
       That is how ChibiOS keeps a tool from being reset by a facility meant
       to catch a hung flight loop. */
    if (AP_BoardConfig::watchdog_enabled() && device_is_ready(wdt)) {
        struct wdt_timeout_cfg wcfg = {};
        wcfg.window.min = 0U;
        wcfg.window.max = HW_WDT_TIMEOUT_MS;
        wcfg.callback = nullptr;
        wcfg.flags = WDT_FLAG_RESET_SOC;
        wdt_channel = wdt_install_timeout(wdt, &wcfg);
        /* Prefer freeze-on-debug (STM32 supports it); fall back to no options
           for SoCs whose driver rejects it (e.g. NXP imx-wdog). */
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
    } else {
        /* Silent before this: device_is_ready()==false skipped both printks, so a missing
         * device looked identical to a working one. */
        printk("AP_Zephyr: hardware watchdog device not ready (check "
               "CONFIG_WATCHDOG)\n");
    }
#endif

    uint32_t lr_last_ms = 0, lr_last_count = 0;

    while (true) {
        k_msleep(100);

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
                printk("LOOPRATE dt_ms=%lu loop_hz=%lu\n",
                       (unsigned long)dt, (unsigned long)hz);
                /* Per-port byte counters for the telem ports (SERIAL1/2),
                   so "is this port actually moving bytes" is answerable
                   without a debugger - see the counters in UARTDriver.h. */
                for (uint8_t sn = 1; sn <= 2; sn++) {
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
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LoopRate: %lu Hz",
                              (unsigned long)hz);
                lr_last_count = g_ap_loop_count;
                lr_last_ms = now_ms;
            }
        }

#if defined(HAVE_HW_WATCHDOG)
        /* Feed unconditionally each iteration: the hardware watchdog
           guards "is the monitor thread alive", independent of the
           software main-loop checks below. */
        if (wdt_channel >= 0) {
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
    printk("AP: rcin thread started\n");
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
    printk("AP: rcout thread started\n");
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
    printk("AP: storage thread started\n");
    Scheduler *sched = static_cast<Scheduler *>(arg);

    while (!sched->_hal_initialized) {
        k_msleep(10);
    }
    printk("AP_thread: AP_storage hal_initialized, entering tick loop\n");
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

void Scheduler::check_stack_free()
{
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
        size_t unused = 0;
        if (k_thread_stack_space_get(known[i].thd, &unused) == 0) {
            if (unused < MIN_STACK_FREE) {
                printk("AP_Zephyr: stack LOW %s: only %u B free\n",
                       known[i].name, (unsigned)unused);
            }
        }
    }
    for (uint8_t i = 0; i < ZEPHYR_MAX_USER_THREADS; i++) {
        if (_user_threads[i].in_use) {
            size_t unused = 0;
            if (k_thread_stack_space_get(
                    &_user_threads[i].thread_data, &unused) == 0) {
                if (unused < MIN_STACK_FREE) {
                    printk("AP_Zephyr: stack LOW user[%u]: only %u B free\n",
                           i, (unsigned)unused);
                }
            }
        }
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
    /* Mirrors AP_HAL_ChibiOS::Scheduler::calculate_thread_priority(). */
    /* RECONCILED 2026-08-13 against the APM_* defines in Scheduler.h - one table, not
     * two, so the priorities cannot drift apart again. */
    int base_prio;
    switch (base) {
    case PRIORITY_BOOST:     base_prio = 2;  break;                       /* one below APM_MAIN_PRIORITY_BOOST(1) */
    case PRIORITY_TIMER:     base_prio = 3;  break;                       /* one below APM_TIMER_PRIORITY(2) */
    case PRIORITY_RCOUT:     base_prio = APM_RCOUT_PRIORITY;    break;
    case PRIORITY_SPI:       base_prio = 6;  break;                       /* below the APM_SPI_PRIORITY(2) bus threads */
    case PRIORITY_MAIN:      base_prio = APM_MAIN_PRIORITY;     break;
    case PRIORITY_CAN:       base_prio = 5;  break;                       /* CAN-bench-validated; see note above */
    case PRIORITY_RCIN:      base_prio = APM_RCIN_PRIORITY;     break;
    case PRIORITY_I2C:       base_prio = APM_I2C_PRIORITY;      break;    /* was 7 = ABOVE main, the same inversion
                                                                             the 2026-08-05 bus-thread fix removed;
                                                                             zero users existed, so no runtime change */
    case PRIORITY_LED:       base_prio = APM_LED_PRIORITY;      break;
    case PRIORITY_UART:      base_prio = APM_UART_PRIORITY;     break;
    case PRIORITY_NET:       base_prio = APM_NET_PRIORITY;      break;
    case PRIORITY_STORAGE:   base_prio = APM_STORAGE_PRIORITY;  break;
    case PRIORITY_IO:        base_prio = 7;  break;                       /* below the APM_IO_PRIORITY(5) io thread */
    case PRIORITY_SCRIPTING: base_prio = APM_SCRIPTING_PRIORITY; break;
    default:                 base_prio = 10; break;                       /* ChibiOS defaults to IO */
    }
    int prio = base_prio - (int)offset;
    if (prio < 0)  { prio = 0;  }
    if (prio > 14) { prio = 14; }
    return prio;
}



#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
