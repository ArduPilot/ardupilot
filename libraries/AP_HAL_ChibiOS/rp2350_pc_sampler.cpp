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
 * RP2350 timer-driven statistical PC sampler. See rp2350_pc_sampler.h for the
 * mechanism and rationale.
 *
 * Samples are kept at full PC resolution in a per-core open-addressing hash
 * table, not a bucketed array: the LTO build packs functions into many small
 * (tens of bytes) sections, so coarse address bins cannot be attributed to a
 * single function. Exact PCs map one-to-one to functions offline via nm.
 *
 * The alarm IRQs are ChibiOS zero-latency interrupts (priority 0), so the
 * kernel BASEPRI cannot mask them: sampling is uniform across thread, ISR and
 * critical-section time. A priority equal to or above the kernel would be
 * blocked inside every critical section and pile the deferred samples onto
 * chSysUnlock, badly biasing the profile. Each handler is a bare naked stub
 * (no OS calls) that reads the interrupted PC from whichever stack the
 * exception used (EXC_RETURN bit 2 selects MSP vs PSP) and returns straight to
 * the interrupted context. The ST LLD ISRs for these two alarms are suppressed
 * (ST_TIMER_ALARM2/3_SUPPRESS_ISR) so these strong symbols own the vectors.
 */

#include <ch.h>
#include "hal.h"

// Included after the ChibiOS headers so RP2350 and the enable macro (from
// hwdef.h via board.h) are defined when the header applies C linkage to the
// declarations below; otherwise the definitions would be C++-mangled and the
// extern "C" callers in ArduCopter would not resolve.
#include "rp2350_pc_sampler.h"

#if defined(RP2350) && defined(AP_RP2350_PC_SAMPLER_ENABLED)

#include <AP_Common/ExpandingString.h>

/*
 * Sampling period in TIMER0 ticks (1 MHz). A prime, non-harmonic with the main
 * (~250 Hz), rate (~335 Hz) and EKF loops to avoid aliasing. ~5.08 kHz.
 */
#define PROF_INTERVAL_US   197u

/* Per-core PC hash table. 2048 slots/core keeps the BSS to 24 KB total, which
 * matters because runtime free heap is tight (~20 KB after EKF init). Colder
 * PCs beyond the table capacity drop (counted), but the hot set we rank stays
 * resident. */
#define PROF_HASH_BITS     11u
#define PROF_HASH_SIZE     (1u << PROF_HASH_BITS)
#define PROF_HASH_MASK     (PROF_HASH_SIZE - 1u)
#define PROF_PROBE_MAX     8u

/* Start of XIP text, from the linker rather than a literal: the app base moves
 * with FLASH_RESERVE_START_KB, and a stale value silently misattributes every
 * flash sample and shifts the reported offsets. */
extern uint32_t __vectors_base__[];

/* Code regions, used only to classify PCs for the summary line. Bases:
 * F=XIP flash text, S=SRAM ramfunc, C=scratch banks (ram4/ram5). */
static const struct {
    uint32_t base;
    uint32_t span;
    char     tag;
} prof_regions[] = {
    { (uint32_t)(uintptr_t)__vectors_base__, 0x160000u, 'F' },
    { 0x20000000u, 0x020000u, 'S' },
    { 0x20080000u, 0x002000u, 'C' },
};
#define PROF_NREGIONS   (sizeof(prof_regions) / sizeof(prof_regions[0]))

#define PROF_DUMP_MAX   16u

struct prof_core {
    uint32_t pc[PROF_HASH_SIZE];   /* key; 0 = empty slot (no code PC is 0) */
    uint16_t cnt[PROF_HASH_SIZE];  /* saturating sample count               */
    uint32_t total;                /* samples taken                         */
    uint32_t dropped;              /* not-code PC or probe chain full       */
};

static struct prof_core prof[2];
static bool inited_core0;
static bool inited_core1;

/*
 * Record one sample and re-arm the alarm. Called (tail-branched) from the naked
 * vector stubs with the interrupted PC in the first argument and the alarm
 * index in the second; on entry LR still holds EXC_RETURN, so this function's
 * ordinary return performs the exception return to the interrupted context.
 *
 * SRAM-resident (.ramtext) so this ~5 kHz path never stalls on an XIP miss; it
 * only uses inlined intrinsics and memory-mapped registers - no flash calls.
 */
extern "C" __attribute__((used, noinline, section(".ramtext")))
void rp2350_pc_sampler_sink(uint32_t addr, uint32_t alarm)
{
    /* Ack this alarm and schedule the next sample (INTE stays set). */
    TIMER0->INTR = (1u << alarm);
    TIMER0->ALARM[alarm] = TIMER0->TIMERAWL + PROF_INTERVAL_US;

    const unsigned core = (unsigned)port_get_core_id() & 1u;
    struct prof_core *pc = &prof[core];
    pc->total++;

    addr &= ~1u;                       /* drop the thumb bit */
    if (addr < 0x10000000u) {          /* ROM/poison: not attributable code */
        pc->dropped++;
        return;
    }

    uint32_t idx = (addr * 2654435761u) >> (32u - PROF_HASH_BITS);
    for (unsigned probe = 0; probe < PROF_PROBE_MAX; probe++) {
        const uint32_t slot = (idx + probe) & PROF_HASH_MASK;
        const uint32_t key = pc->pc[slot];
        if (key == addr) {
            if (pc->cnt[slot] != 0xFFFFu) {
                pc->cnt[slot]++;
            }
            return;
        }
        if (key == 0u) {
            pc->pc[slot] = addr;
            pc->cnt[slot] = 1u;
            return;
        }
    }
    pc->dropped++;
}

/*
 * Bare zero-latency handlers for the two free TIMER0 alarms (IRQ2 -> Vector48
 * on core0, IRQ3 -> Vector4C on core1). EXC_RETURN bit 2 selects the stack the
 * exception frame is on; the stacked PC is at frame offset 0x18 (fixed even
 * with FPU lazy stacking). Tail-call the sink with (PC, alarm); LR is untouched
 * so the sink's return is the exception return.
 */
extern "C" __attribute__((naked, used, section(".ramtext"))) void Vector48(void)
{
    __asm volatile(
        "tst    lr, #4\n"
        "ite    eq\n"
        "mrseq  r0, msp\n"
        "mrsne  r0, psp\n"
        "ldr    r0, [r0, #0x18]\n"
        "mov    r1, #2\n"
        "b      rp2350_pc_sampler_sink\n");
}
extern "C" __attribute__((naked, used, section(".ramtext"))) void Vector4C(void)
{
    __asm volatile(
        "tst    lr, #4\n"
        "ite    eq\n"
        "mrseq  r0, msp\n"
        "mrsne  r0, psp\n"
        "ldr    r0, [r0, #0x18]\n"
        "mov    r1, #3\n"
        "b      rp2350_pc_sampler_sink\n");
}

/* Arm one free alarm (INTE via the atomic SET alias to avoid a cross-core RMW
 * race on the shared register). */
static void sampler_arm(unsigned alarm)
{
    TIMER0->INTR = (1u << alarm);
    TIMER0->ALARM[alarm] = TIMER0->TIMERAWL + PROF_INTERVAL_US;
    TIMER0->SET.INTE = (1u << alarm);
}

void rp2350_pc_sampler_init_core0(void)
{
    if (inited_core0) {
        return;
    }
    inited_core0 = true;
    sampler_arm(2);
    /* Priority 0: a zero-latency IRQ, unmaskable by the kernel BASEPRI. */
    nvicEnableVector(RP_TIMER0_IRQ2_NUMBER, 0);
}

void rp2350_pc_sampler_init_core1(void)
{
    if (inited_core1) {
        return;
    }
    inited_core1 = true;
    sampler_arm(3);
    nvicEnableVector(RP_TIMER0_IRQ3_NUMBER, 0);
}

/* Region tag for an address, and its offset from the region base, for the
 * dump tokens. Returns 'o' (other) when outside every code region. */
static char region_of(uint32_t addr, uint32_t *off)
{
    for (unsigned r = 0; r < PROF_NREGIONS; r++) {
        if (addr - prof_regions[r].base < prof_regions[r].span) {
            *off = addr - prof_regions[r].base;
            return prof_regions[r].tag;
        }
    }
    *off = addr;
    return 'o';
}

/* Minimal append helpers - keep one byte spare for the terminating NUL. */
static uint32_t put_str(char *b, uint32_t u, uint32_t cap, const char *s)
{
    while (*s != '\0' && u + 1u < cap) {
        b[u++] = *s++;
    }
    return u;
}
static uint32_t put_ch(char *b, uint32_t u, uint32_t cap, char c)
{
    if (u + 1u < cap) {
        b[u++] = c;
    }
    return u;
}
static uint32_t put_dec(char *b, uint32_t u, uint32_t cap, uint32_t v)
{
    char t[10];
    unsigned n = 0;
    do {
        t[n++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v != 0 && n < sizeof(t));
    while (n != 0 && u + 1u < cap) {
        b[u++] = t[--n];
    }
    return u;
}
static uint32_t put_hex(char *b, uint32_t u, uint32_t cap, uint32_t v)
{
    char t[8];
    unsigned n = 0;
    do {
        const unsigned d = v & 0xFu;
        t[n++] = (char)(d < 10u ? '0' + d : 'a' + d - 10u);
        v >>= 4;
    } while (v != 0 && n < sizeof(t));
    while (n != 0 && u + 1u < cap) {
        b[u++] = t[--n];
    }
    return u;
}

uint32_t rp2350_pc_sampler_dump(unsigned core, unsigned maxn,
                                char *buf, uint32_t buflen)
{
    if (core > 1 || buf == nullptr || buflen == 0) {
        return 0;
    }
    if (maxn > PROF_DUMP_MAX) {
        maxn = PROF_DUMP_MAX;
    }
    const struct prof_core *pc = &prof[core];

    /* Single pass over the hash table: per-region sample sums (F/S) and a
     * descending top-N of the hottest individual PCs. */
    uint32_t fsum = 0, ssum = 0;
    uint32_t top_pc[PROF_DUMP_MAX];
    uint16_t top_cnt[PROF_DUMP_MAX];
    unsigned ntop = 0;
    for (unsigned i = 0; i < PROF_HASH_SIZE; i++) {
        const uint32_t addr = pc->pc[i];
        const uint16_t c = pc->cnt[i];
        if (addr == 0u || c == 0u) {
            continue;
        }
        if (addr - prof_regions[0].base < prof_regions[0].span) {
            fsum += c;
        } else if (addr - prof_regions[1].base < prof_regions[1].span) {
            ssum += c;
        }
        if (ntop < maxn || (maxn > 0 && c > top_cnt[maxn - 1])) {
            unsigned j = (ntop < maxn) ? ntop++ : (maxn - 1);
            while (j > 0 && top_cnt[j - 1] < c) {
                top_cnt[j] = top_cnt[j - 1];
                top_pc[j] = top_pc[j - 1];
                j--;
            }
            top_cnt[j] = c;
            top_pc[j] = addr;
        }
    }

    const uint32_t total = pc->total ? pc->total : 1;
    const uint32_t osum = (total > fsum + ssum) ? (total - fsum - ssum) : 0;
    uint32_t used = 0;
    used = put_str(buf, used, buflen, "n=");
    used = put_dec(buf, used, buflen, pc->total);
    used = put_str(buf, used, buflen, " F=");
    used = put_dec(buf, used, buflen, (uint32_t)((uint64_t)fsum * 100u / total));
    used = put_str(buf, used, buflen, "% S=");
    used = put_dec(buf, used, buflen, (uint32_t)((uint64_t)ssum * 100u / total));
    used = put_str(buf, used, buflen, "% o=");
    used = put_dec(buf, used, buflen, (uint32_t)((uint64_t)osum * 100u / total));
    used = put_ch(buf, used, buflen, '%');

    /* Hot PCs as region-tagged offset:count tokens (full resolution), wrapped
     * near 40 chars so each line fits a MAVLink STATUSTEXT. */
    uint32_t line = 999;  /* force a newline before the first token */
    for (unsigned k = 0; k < ntop && used + 20u < buflen; k++) {
        uint32_t off = 0;
        const char tag = region_of(top_pc[k], &off);
        if (line >= 40) {
            used = put_ch(buf, used, buflen, '\n');
            line = 0;
        } else {
            used = put_ch(buf, used, buflen, ' ');
            line += 1;
        }
        const uint32_t start = used;
        used = put_ch(buf, used, buflen, tag);
        used = put_hex(buf, used, buflen, off);
        used = put_ch(buf, used, buflen, ':');
        used = put_dec(buf, used, buflen, top_cnt[k]);
        line += used - start;
    }
    buf[used] = '\0';
    return used;
}

/*
 * Runtime free heap is tight (~20 KB after EKF init), so dump only the hottest
 * PROF_FULL_TOPN PCs rather than the whole table - 32x the STATUSTEXT top-16,
 * enough to rank functions, but a few KB. Selection buffers are static (BSS),
 * not on the FTP thread's stack.
 */
#define PROF_FULL_TOPN 512u
static uint32_t full_pc[PROF_FULL_TOPN];
static uint16_t full_cnt[PROF_FULL_TOPN];

void rp2350_pc_sampler_dump_full(ExpandingString &str, unsigned core)
{
    if (core > 1) {
        return;
    }
    const struct prof_core *pc = &prof[core];

    unsigned ntop = 0;
    for (unsigned i = 0; i < PROF_HASH_SIZE; i++) {
        const uint32_t addr = pc->pc[i];
        const uint16_t c = pc->cnt[i];
        if (addr == 0u || c == 0u) {
            continue;
        }
        if (ntop < PROF_FULL_TOPN || c > full_cnt[PROF_FULL_TOPN - 1]) {
            unsigned j = (ntop < PROF_FULL_TOPN) ? ntop++ : (PROF_FULL_TOPN - 1);
            while (j > 0 && full_cnt[j - 1] < c) {
                full_cnt[j] = full_cnt[j - 1];
                full_pc[j] = full_pc[j - 1];
                j--;
            }
            full_cnt[j] = c;
            full_pc[j] = addr;
        }
    }

    str.printf("PROF c%u n=%lu drop=%lu top=%u\n", core,
               (unsigned long)pc->total, (unsigned long)pc->dropped, ntop);

    /* Region-tagged offset:count tokens, ~12 per line; the offline tool
     * aggregates by function. */
    unsigned per_line = 0;
    for (unsigned k = 0; k < ntop; k++) {
        uint32_t off = 0;
        const char tag = region_of(full_pc[k], &off);
        if (per_line == 0) {
            str.printf("PROF");
        }
        str.printf(" %c%lx:%u", tag, (unsigned long)off, (unsigned)full_cnt[k]);
        if (++per_line >= 12) {
            str.printf("\n");
            per_line = 0;
        }
    }
    if (per_line != 0) {
        str.printf("\n");
    }
}

#endif /* RP2350 && AP_RP2350_PC_SAMPLER_ENABLED */
