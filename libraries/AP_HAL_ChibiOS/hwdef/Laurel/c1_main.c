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
 * @file Laurel/c1_main.c
 * RP2350 Core1 entry point for ChibiOS Full SMP mode.
 * Initialises the ch1 OS instance so threads created with affinity &ch1
 * (e.g. the rate thread) are scheduled on core1.
 */
#include "ch.h"
#include <stdint.h>

/*
 * This translation unit is added directly by the Laurel board makefile and the generated CRT0 trampoline expects c1_main() to exist whenever the extra-core startup path is selected.
 */

/*
 * RP2350 SIO register block (section 2.3.1 of the RP2350 datasheet).
 * Only the FIFO registers needed here.
 */
#define SIO_BASE        0xD0000000UL
#define SIO_FIFO_ST     (*(volatile uint32_t *)(SIO_BASE + 0x050))
#define SIO_FIFO_WR     (*(volatile uint32_t *)(SIO_BASE + 0x054))
#define SIO_FIFO_RD     (*(volatile uint32_t *)(SIO_BASE + 0x058))

#define FIFO_ST_VLD     (1u << 0)   /* RX FIFO has data */
#define FIFO_ST_RDY     (1u << 1)   /* TX FIFO has space */
#define FIFO_ST_ROE     (1u << 3)   /* RX overflow error */
#define FIFO_ST_WOF     (1u << 2)   /* TX underflow error */


/*
 * Core-Private NVIC registers (PPB address space, banked per-core on RP2350).
 * RP2350 has 52 external IRQs, covered by two 32-bit words (ICER0/ICER1).
 */
#define NVIC_ICER0  (*(volatile uint32_t *)0xE000E180U)
#define NVIC_ICER1  (*(volatile uint32_t *)0xE000E184U)
#define NVIC_ICPR0  (*(volatile uint32_t *)0xE000E280U)
#define NVIC_ICPR1  (*(volatile uint32_t *)0xE000E284U)

/*
 * SCB registers used in __c1_cpu_init() to configure Core 1's exception handling.
 * All PPB addresses are per-core on Cortex-M33 / RP2350.
 */
#define SCB_VTOR    (*(volatile uint32_t *)0xE000ED08U)  /* Vector Table Offset Register       */
#define SCB_SHCSR   (*(volatile uint32_t *)0xE000ED24U)  /* System Handler Control and State   */
/* SHCSR enable bits for configurable-priority system exceptions */
#define SHCSR_MEMFAULTENA  (1u << 16)  /* enable MemManage fault handler     */
#define SHCSR_BUSFAULTENA  (1u << 17)  /* enable BusFault handler            */
#define SHCSR_USGFAULTENA  (1u << 18)  /* enable UsageFault handler          */

/*
 * TIMER0 TIMERAWL.
 * Unlike TIMELR, reading TIMERAWL doesn't latch TIMERAWH so it is safe to read from Core1 concurrently with Core0 without coordination.
 */
#define TIMER0_TIMERAWL  (*(volatile uint32_t *)0x400B0028U)

/* Diagnostic: captures Core 1's VTOR at init time — readable via OpenOCD/GDB. */
volatile uint32_t c1_vtor_at_boot = 0U;

volatile uint32_t c1_boot_stage = 0xDEAD0000U;
void __c1_cpu_init(void)
{
/*
 * Scrub Core 1's NVIC before anything else runs.
 * The RP2350 ROM's Core 1 launch protocol (6-step FIFO handshake) leaves SPARE_IRQ_1 (IRQ 47, VectorFC) enabled in Core 1's banked NVIC registers.
 */
    NVIC_ICER0 = 0xFFFFFFFFU;  /* disable IRQs  0..31 on Core 1 */
    NVIC_ICER1 = 0xFFFFFFFFU;  /* disable IRQs 32..51 on Core 1 (covers SPARE_IRQ_1=47) */
    NVIC_ICPR0 = 0xFFFFFFFFU;  /* clear pending IRQs  0..31 on Core 1 */
    NVIC_ICPR1 = 0xFFFFFFFFU;  /* clear pending IRQs 32..51 on Core 1 */
    /* DSB+ISB ensure the writes complete before any instruction that could
     * re-enable IRQs or take an exception sees the updated NVIC state. */
    __asm volatile ("dsb sy\n isb" ::: "memory");

/*
 * Re-assert VTOR explicitly on Core 1.
 * We repeat it here belt-and-suspenders style to guard against any ROM or bootloader artefact that may have left VTOR at an invalid address on Core 1.
 * Explicitly setting VTOR here breaks that cycle.
 */
    extern uint32_t _vectors[];
    SCB_VTOR = (uint32_t)_vectors;
    c1_vtor_at_boot = (uint32_t)_vectors;  /* readable via OpenOCD for diagnosis */

/*
 * Enable configurable-priority fault handlers on Core 1.
 * VECTTBL), causing a double-fault lockup i.e. opaque to the debugger.
 * The handlers (at VTOR[4..6]) loop safely via save_fault_watchdog, and OpenOCD can halt Core 1 to read CFSR/MMFAR/BFAR for diagnosis.
 */
    SCB_SHCSR |= SHCSR_MEMFAULTENA | SHCSR_BUSFAULTENA | SHCSR_USGFAULTENA;
    __asm volatile ("dsb sy\n isb" ::: "memory");

    c1_boot_stage = 1U;
}

void __c1_late_init(void)
{
    c1_boot_stage = 2U;
}

/*
 * Core1 entry point — ChibiOS Full SMP mode.
 * Called from _crt0_c1_entry after stack/FPU init.
 * Initialises ch1 so the OS can schedule threads pinned to core1.
 */
void c1_main(void) {
    c1_boot_stage = 3U;
#if CH_CFG_SMP_MODE == TRUE
    /* Wait for core0's chSysInit() to complete. */
    chSysWaitSystemState(ch_sys_running);

    /* Initialise core1's OS instance; ch1.rlist.current is now valid. */
    chInstanceObjectInit(&ch1, &ch_core1_cfg);

    /* Release the I-Lock — scheduling on core1 starts here. */
    chSysUnlock();

    while (true) {
        chThdSleepMilliseconds(1000U);
    }
#else
    while (true) {}
#endif
}