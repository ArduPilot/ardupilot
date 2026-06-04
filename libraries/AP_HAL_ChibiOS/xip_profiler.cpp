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
 */

/*
 * RP2350 per-thread XIP cache hit-rate profiler.
 *
 * ap_xip_cs_hook() is registered in rp2350_ramfunc2_registry.txt so the
 * linker places it in SRAM — it must not be in XIP flash itself or it would
 * disturb the very counters it reads.
 *
 * Both Laurel and Pico2 run ChibiOS Full SMP (CH_CFG_SMP_MODE TRUE).
 * Core 0 runs most ArduPilot threads.  Core 1 runs a ChibiOS SMP instance
 * (ch1) hosting the rate controller thread (pinned via
 * thread_create_alloc_affinity).  CH_CFG_CONTEXT_SWITCH_HOOK fires on
 * whichever core the context switch occurs on, so the rate thread's XIP
 * counts are correctly attributed here along with all Core 0 threads.
 *
 * Call ap_xip_profiler_append_thread_info() from Util::thread_info() to emit
 * the accumulated stats via @SYS/threads.txt.
 */

#include <stdint.h>
#include <ch.h>
#include "hal.h"

#include "xip_profiler.h"

#if defined(RP2350) && defined(AP_XIP_PROFILER_ENABLED)

/* Maximum number of threads tracked.  ArduPilot on RP2350 uses around 15. */
#define XIP_PROFILER_MAX_THREADS 24U

struct xip_thread_entry {
    const void *ptr;      /* thread_t pointer used as key */
    uint64_t    acc;      /* accumulated access count     */
    uint64_t    hit;      /* accumulated hit count        */
    uint32_t    windows;  /* number of switch-out events  */
};

static struct xip_thread_entry _xip_threads[XIP_PROFILER_MAX_THREADS];
static uint8_t _xip_thread_count;

/*
 * ap_xip_cs_hook — lives in SRAM via rp2350_ramfunc2_registry.txt.
 *
 * Called from CH_CFG_CONTEXT_SWITCH_HOOK(ntp, otp) in sys_lock state, on
 * whichever core the context switch occurs (Core 0 or Core 1).
 * Must not call any function that resides in XIP flash.
 * Only uses:
 *   - hardware register reads/writes (memory-mapped, not XIP)
 *   - BSS array access (_xip_threads[], _xip_thread_count)
 *   - simple integer arithmetic
 */
extern "C" void ap_xip_cs_hook(const void *ntp, const void *otp)
{
    (void)ntp;

    /* Snapshot and reset 21-bit saturating counters.  Writing 0 resets. */
    const uint32_t hit = XIP_CTRL->CTR_HIT;
    const uint32_t acc = XIP_CTRL->CTR_ACC;
    XIP_CTRL->CTR_HIT = 0U;
    XIP_CTRL->CTR_ACC = 0U;

    /* Locate or allocate a slot for the outgoing thread. */
    struct xip_thread_entry *slot = nullptr;
    for (uint8_t i = 0; i < _xip_thread_count; i++) {
        if (_xip_threads[i].ptr == otp) {
            slot = &_xip_threads[i];
            break;
        }
    }
    if (slot == nullptr && _xip_thread_count < XIP_PROFILER_MAX_THREADS) {
        slot = &_xip_threads[_xip_thread_count];
        slot->ptr     = otp;
        slot->acc     = 0U;
        slot->hit     = 0U;
        slot->windows = 0U;
        _xip_thread_count++;
    }

    if (slot != nullptr) {
        slot->acc += acc;
        slot->hit += hit;
        slot->windows++;
    }
}

/*
 * ap_xip_profiler_append_thread_info — normal C++ code, may be in XIP.
 *
 * Appends per-thread XIP hit-rate rows to str.  The format mirrors the
 * existing thread_info() output so log parsers can easily correlate.
 * Also shows the current (in-flight) counters for the calling thread.
 *
 * Snapshots are a best-effort read without taking a critical section;
 * slight inconsistency is acceptable for a profiling tool.
 */
void ap_xip_profiler_append_thread_info(ExpandingString &str)
{
    str.printf("XIPCacheV1\n");

    /* Current in-flight window counters (not yet attributed to any thread). */
    str.printf("%-13.13s HIT=%7lu ACC=%7lu (live)\n",
               "current",
               (unsigned long)XIP_CTRL->CTR_HIT,
               (unsigned long)XIP_CTRL->CTR_ACC);

    for (uint8_t i = 0; i < _xip_thread_count; i++) {
        const thread_t *tp = (const thread_t *)_xip_threads[i].ptr;
        const char *name   = (tp != nullptr && tp->name != nullptr) ? tp->name : "?";
        const uint64_t acc = _xip_threads[i].acc;
        const uint64_t hit = _xip_threads[i].hit;
        const uint32_t pct = (acc > 0U) ? (uint32_t)((hit * 100ULL) / acc) : 0U;
        str.printf("%-13.13s HIT=%7lu ACC=%7lu (%3lu%%) W=%lu\n",
                   name,
                   (unsigned long)hit,
                   (unsigned long)acc,
                   (unsigned long)pct,
                   (unsigned long)_xip_threads[i].windows);
    }
}

#endif /* RP2350 && AP_XIP_PROFILER_ENABLED */
