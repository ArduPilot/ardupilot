/*
 * Statistical PC profiler that needs no debug probe.
 *
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

/* WHY THIS EXISTS: Tools/zephyr/zephyr_pcsr_sample.py answers the same question
 * - which code the CPU is really in, and whether it is fetching that code from
 * ITCM or from external NOR over FlexSPI - but it needs an SWD probe attached.
 * This does it from inside the firmware so the question can be answered with
 * nothing but the USB console.
 *
 * A k_timer callback runs in the system clock interrupt. PSP still points at
 * the exception frame the hardware pushed for the thread that was interrupted,
 * so the stacked PC at PSP+24 (r0,r1,r2,r3,r12,lr,pc,xpsr) is a sample of that
 * thread's execution. Interrupts are therefore invisible to this profiler: a
 * sample taken while another ISR was running reports the thread underneath it.
 * That is the intended bias - the question here is where thread time goes.
 *
 * Needs >=2000 samples before any single line of output means anything.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <cmsis_core.h>

#include <fsl_clock.h>

#include "ap_hooks.h"

/* 64-byte buckets: fine enough to separate functions, coarse enough that a
 * hot loop lands in one or two buckets instead of smearing over dozens. */
#define PCP_SHIFT   6u
#define PCP_BINS    512u           /* power of two, open addressed */
#define PCP_PROBES  8u
#define PCP_TOP     10u
#define PCP_PERIOD_US 997          /* prime-ish: no aliasing with 1 kHz work */

static uint32_t pcp_key[PCP_BINS];
static uint32_t pcp_cnt[PCP_BINS];
static uint32_t pcp_n, pcp_miss, pcp_bad;
static uint32_t pcp_reg[5];        /* itcm, dtcm, ocram, xip, other */
static uint32_t pcp_band[4];       /* prio <=2, ==3 (main), 4..9, >=10 */
static struct k_timer pcp_timer;

static inline uint32_t pcp_region(uint32_t pc)
{
    if (pc < 0x00080000u) {
        return 0;                                  /* ITCM */
    }
    if (pc >= 0x20000000u && pc < 0x20080000u) {
        return 1;                                  /* DTCM */
    }
    if (pc >= 0x20200000u && pc < 0x20300000u) {
        return 2;                                  /* OCRAM, .ramfunc */
    }
    if (pc >= 0x30000000u && pc < 0x34000000u) {
        return 3;                                  /* external NOR, XIP */
    }
    return 4;
}

static void pcp_tick(struct k_timer *t)
{
    ARG_UNUSED(t);
    const uint32_t psp = __get_PSP();

    /* every thread stack on this board is in DTCM or OCRAM; anything else means
     * there is no thread frame to read. */
    if (psp < 0x20000000u || psp >= 0x20300000u) {
        return;
    }
    const uint32_t pc = *(volatile uint32_t *)(psp + 24u);
    const uint32_t xpsr = *(volatile uint32_t *)(psp + 28u);

    /* Reject anything that is not a real exception frame. PSP is only the
     * frame the hardware pushed when the interrupted context was a thread; at
     * other moments - inside a context switch, or when a thread has been
     * switched out and PSP holds saved callee registers - PSP+24 is ordinary
     * stack data. Without this check those reads land on literal pools and
     * .rodata, which read as plausible code addresses and quietly inflate the
     * share attributed to whichever region they fall in. The T bit in the
     * stacked xPSR is always set on Cortex-M, so a frame without it is not
     * one. */
    if ((xpsr & 0x01000000u) == 0u || (pc & 1u) != 0u ||
        pcp_region(pc) == 4u) {
        pcp_bad++;
        return;
    }

    pcp_n++;
    pcp_reg[pcp_region(pc)]++;

    const int prio = k_thread_priority_get(k_current_get());
    if (prio <= 2) {
        pcp_band[0]++;
    } else if (prio == 3) {
        pcp_band[1]++;
    } else if (prio <= 9) {
        pcp_band[2]++;
    } else {
        pcp_band[3]++;
    }

    const uint32_t bucket = pc >> PCP_SHIFT;
    uint32_t h = (bucket * 2654435761u) & (PCP_BINS - 1u);
    for (uint32_t i = 0; i < PCP_PROBES; i++) {
        if (pcp_cnt[h] == 0u) {
            pcp_key[h] = bucket;
            pcp_cnt[h] = 1u;
            return;
        }
        if (pcp_key[h] == bucket) {
            pcp_cnt[h]++;
            return;
        }
        h = (h + 1u) & (PCP_BINS - 1u);
    }
    pcp_miss++;
}

/* One-shot: the two things that make every instruction on this part slow or
 * fast, reported as facts rather than assumed from Kconfig. CONFIG_ICACHE=y
 * only means the build asked for the cache; CCR says whether it is on. */
static void pcp_report_env(void)
{
    printk("PCPROF env ccr=%08x ic=%u dc=%u mpuctrl=%08x m7=%uHz flexspi1=%uHz\n",
           (unsigned)SCB->CCR,
           (unsigned)((SCB->CCR & SCB_CCR_IC_Msk) ? 1u : 0u),
           (unsigned)((SCB->CCR & SCB_CCR_DC_Msk) ? 1u : 0u),
           (unsigned)MPU->CTRL,
           (unsigned)CLOCK_GetRootClockFreq(kCLOCK_Root_M7),
           (unsigned)CLOCK_GetRootClockFreq(kCLOCK_Root_Flexspi1));
}

void ap_pcprofile_report(void)
{
    static bool env_done;
    const uint32_t n = pcp_n;

    if (!env_done) {
        env_done = true;
        pcp_report_env();
    }
    if (n < 500u) {
        return;
    }
    printk("PCPROF n=%u itcm=%u%% ocram=%u%% xip=%u%% dtcm=%u%% oth=%u%% "
           "miss=%u bad=%u | hi=%u%% main=%u%% mid=%u%% lo=%u%%\n",
           n,
           pcp_reg[0] * 100u / n, pcp_reg[2] * 100u / n, pcp_reg[3] * 100u / n,
           pcp_reg[1] * 100u / n, pcp_reg[4] * 100u / n, pcp_miss, pcp_bad,
           pcp_band[0] * 100u / n, pcp_band[1] * 100u / n,
           pcp_band[2] * 100u / n, pcp_band[3] * 100u / n);

    /* top buckets, selection sort over the table - runs once per report */
    char line[200];
    int off = snprintf(line, sizeof(line), "PCPROF top");
    for (uint32_t k = 0; k < PCP_TOP; k++) {
        uint32_t best = 0, bi = PCP_BINS;
        for (uint32_t i = 0; i < PCP_BINS; i++) {
            if (pcp_cnt[i] > best) {
                best = pcp_cnt[i];
                bi = i;
            }
        }
        if (bi == PCP_BINS || best == 0u) {
            break;
        }
        int w = snprintf(line + off, sizeof(line) - off, " %08x:%u",
                         pcp_key[bi] << PCP_SHIFT, best);
        pcp_cnt[bi] = 0u;              /* consumed: also clears for next window */
        if (w <= 0 || off + w >= (int)sizeof(line) - 1) {
            break;
        }
        off += w;
    }
    printk("%s\n", line);

    for (uint32_t i = 0; i < PCP_BINS; i++) {
        pcp_cnt[i] = 0u;
    }
    pcp_n = 0u;
    pcp_miss = 0u;
    pcp_bad = 0u;
    for (uint32_t i = 0; i < 5; i++) {
        pcp_reg[i] = 0u;
    }
    for (uint32_t i = 0; i < 4; i++) {
        pcp_band[i] = 0u;
    }
}

/* Off by default since 2026-09-27. The sampler is a 1 kHz k_timer, and with
 * CONFIG_TICKLESS_KERNEL on a 1 MHz tick every expiry reprograms the hardware
 * timer - so it costs far more than the sample itself. It has already given its
 * answer on this board (itcm=42% xip=57%, hottest bucket the AP_InternalError
 * region at 5.2%), and the CPU it frees is needed by the below-main threads.
 * Set AP_ZEPHYR_PCPROFILE_ENABLED to 1 to profile again. */
#ifndef AP_ZEPHYR_PCPROFILE_ENABLED
#define AP_ZEPHYR_PCPROFILE_ENABLED 0
#endif

static int ap_pcprofile_init(void)
{
#if AP_ZEPHYR_PCPROFILE_ENABLED
    k_timer_init(&pcp_timer, pcp_tick, NULL);
    k_timer_start(&pcp_timer, K_USEC(PCP_PERIOD_US), K_USEC(PCP_PERIOD_US));
#endif
    return 0;
}

SYS_INIT(ap_pcprofile_init, APPLICATION, 90);
