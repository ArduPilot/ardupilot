#pragma once
/*
 * RP2350 per-thread XIP cache hit-rate profiler.
 *
 * Enable with:  define AP_XIP_PROFILER_ENABLED 1  in the board's hwdef.dat
 *
 * Design notes:
 *  - RP2350 XIP_CTRL has two 21-bit saturating counters (CTR_HIT, CTR_ACC)
 *    that count cache hits and total accesses respectively.  They are global
 *    (no per-master or per-core filter exists in hardware).
 *  - Both Laurel and Pico2 run ChibiOS Full SMP (CH_CFG_SMP_MODE TRUE).
 *    Core 0 runs most ArduPilot threads.  Core 1 runs a ChibiOS SMP instance
 *    (ch1) with the rate controller thread pinned to it via
 *    thread_create_alloc_affinity().
 *  - CH_CFG_CONTEXT_SWITCH_HOOK fires on whichever core the context switch
 *    occurs on, so ap_xip_cs_hook() correctly attributes XIP counts to the
 *    rate thread on Core 1 as well as all Core 0 threads.
 *  - ISR XIP accesses are charged to the thread that was interrupted; this is
 *    acceptable for a tuning tool.
 *  - Counters are reset on every context switch, so 21-bit saturation only
 *    affects one window (~1 ms quantum) and does not corrupt the accumulator.
 */
#if defined(RP2350) && defined(AP_XIP_PROFILER_ENABLED)

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Called from CH_CFG_CONTEXT_SWITCH_HOOK(ntp, otp) — runs in sys_lock state.
 * Snapshots CTR_HIT / CTR_ACC, resets them, and attributes the counts to the
 * outgoing thread (otp).
 */
void ap_xip_cs_hook(const void *ntp, const void *otp);

#ifdef __cplusplus
}

#include <AP_Common/ExpandingString.h>

/*
 * Appends per-thread XIP cache hit-rate statistics to str.
 * Call from Util::thread_info() so the data appears in @SYS/threads.txt.
 */
void ap_xip_profiler_append_thread_info(ExpandingString &str);

#endif /* __cplusplus */
#endif /* RP2350 && AP_XIP_PROFILER_ENABLED */
