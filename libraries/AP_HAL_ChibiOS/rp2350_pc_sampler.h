/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   RP2350 statistical PC sampler.
 */
#pragma once
/*
 * RP2350 timer-driven statistical PC sampler (SWD-free, in-flight).
 *
 * Enable with:  define AP_RP2350_PC_SAMPLER_ENABLED 1  in the board hwdef.dat
 *
 * Purpose: decide which functions to relocate into SRAM (see
 * rp2350_ramfunc2_registry.txt) by measuring where each core actually spends
 * its time, while the vehicle runs. Complements xip_profiler.cpp, which
 * attributes XIP cache hit rate per thread; this attributes CPU time per code
 * address so the hot XIP-resident functions can be identified for relocation.
 *
 * Mechanism: the SMP tickless scheduler binds TIMER0 ALARM0 to core0 and
 * ALARM1 to core1 for its per-core ticks, leaving ALARM2 and ALARM3 free. We
 * take ALARM2 for core0 and ALARM3 for core1, each fired at a fixed
 * non-harmonic rate on its own core's NVIC. The alarm ISR reads the stacked
 * (interrupted) PC at offset 0x18 of the exception frame, on the MSP or PSP
 * as EXC_RETURN selects, and increments a per-core address histogram.
 * Attribution to functions is done offline with the ELF symbol table
 * (Tools/debug/rp2350_pc_profiler.py).
 *
 * The alarm IRQs run at priority 0, which the kernel BASEPRI cannot mask, so
 * thread, ISR and critical-section time are all sampled uniformly.
 */
#include <AP_HAL/AP_HAL_Boards.h>

#if defined(RP2350) && AP_RP2350_PC_SAMPLER_ENABLED

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

/* Arm ALARM2 and enable its IRQ on the calling core (core0). Idempotent. */
void rp2350_pc_sampler_init_core0(void);

/* Arm ALARM3 and enable its IRQ on the calling core (core1). Idempotent.
 * Must be called from a thread that runs on core1 (e.g. the rate thread). */
void rp2350_pc_sampler_init_core1(void);

/*
 * Format the top-N buckets for one core (0 or 1) into buf as newline-separated
 * lines, each kept short enough for a MAVLink STATUSTEXT. The first line is a
 * summary (total samples and the flash/sram/other split); the remaining lines
 * list the hottest buckets as region-tagged offset:count tokens (F=flash from
 * __vectors_base__, S=sram from 0x20000000, C=scratch from 0x20080000).
 * Returns the number of bytes written (excluding the NUL).
 */
uint32_t rp2350_pc_sampler_dump(unsigned core, unsigned maxn,
                                char *buf, uint32_t buflen);

#ifdef __cplusplus
}

/*
 * Dump the hottest 512 PCs of the per-core hash into str as
 * PROF-prefixed token lines, for bulk readout over MAVLink FTP (@SYS/pcprof.txt)
 * and offline attribution with rp2350_pc_profiler.py --histogram. The top-N
 * STATUSTEXT dump only covers the concentrated hotspots; the spread EKF math
 * needs the longer list.
 */
class ExpandingString;
void rp2350_pc_sampler_dump_full(ExpandingString &str, unsigned core);
#endif  // __cplusplus

#endif /* RP2350 && AP_RP2350_PC_SAMPLER_ENABLED */
