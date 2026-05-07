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
 * @file Laurel/c1_main.c @brief RP2350 core1 bare-metal function-pointer dispatcher for Laurel.
 * Protocol (SIO FIFO, 32-bit words): Core0 → Core1 (core1's RX FIFO): function pointer cast to uint32_t Core1 → Core0 (core0's RX FIFO): 1 = task completed, 0 = idle ping
 */

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

/*
 * Accumulated microseconds Core1 has spent executing dispatched functions.
 * Core0 reads this every 10 s to compute Core1 CPU utilisation.
 */
volatile uint32_t c1_busy_us __attribute__((used, externally_visible)) = 0U;

volatile uint32_t c1_boot_stage = 0xDEAD0000U;
/*
 * Side-channel async dispatch.
 * Two independent zero-mutex, zero-FIFO-protocol side-channels let Core0 fire jobs on Core1 without touching the SIO FIFO done-token stream used by the synchronous dispatchers (c1_run_sync / c1_run_sync_locked / c1_try_run_sync).
 * collected at the start of the next update_flight_mode cycle (~8 ms gap → ~0 µs wait).
 * Having a separate channel means UpdateFilter() doesn't need to drain the attitude side-channel before dispatching covariance.
 * eliminating the ~92 µs extra barrier wait that regressed the loop rate when both jobs shared the same channel.
 * Core1 detects the non-zero fn in its WFE idle loop, clears the pointer, calls the function, accumulates busy time, sets done=1, calls SEV.
 * Core0 barrier spins on the done variable (3 ms timeout).
 */
volatile uint32_t c1_att_fn_sidechan  __attribute__((used, externally_visible)) = 0U;
volatile uint8_t  c1_att_sidechan_done __attribute__((used, externally_visible)) = 0U;
/* Covariance side-channel — separate from attitude so UpdateFilter() can
 * dispatch covariance without first draining the attitude side-channel.     */
volatile uint32_t c1_cov_fn_sidechan  __attribute__((used, externally_visible)) = 0U;
volatile uint8_t  c1_cov_sidechan_done __attribute__((used, externally_visible)) = 0U;
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
 * Core1 entry point.
 * called from _crt0_c1_entry after stack/FPU init.
 * Runs entirely bare-metal.
 * Core1 sits in a WFE loop waiting for core0 to push a function pointer via the SIO inter-core FIFO.
 * // core0 side: SIO->FIFO_WR = (uint32_t)(c1_task_fn)my_function
 * // core1 will call my_function() then write 1 back.
 */
void c1_main(void)
{
    c1_boot_stage = 3U;

/*
 * Re-enable maskable interrupts on Core 1.
 * The ChibiOS CRT1 startup sets PRIMASK=1 (cpsid i) at entry to give the startup code a stable environment.
 * bad VTOR → HFSR.VECTTBL), the core goes to lockup with no recoverable state.
 * We have already disabled all 52 NVIC IRQs via NVIC_ICER, so clearing PRIMASK here is safe: no peripheral interrupt can fire.
 * This survives a single fault without double-faulting.
 */
    __asm volatile ("cpsie i" ::: "memory");

    SIO_FIFO_ST = FIFO_ST_ROE | FIFO_ST_WOF;
    while (SIO_FIFO_ST & FIFO_ST_VLD) {
        (void)SIO_FIFO_RD;
    }

    while (1) {
        c1_boot_stage = 0x4DU;

        // wait for a function pointer from core0, execute it, then signal completion by writing 1 back to core0.  If we receive 0 instead of a valid pointer, it's a ping from core0 — write 0 back and wait for the next message.
        while (!(SIO_FIFO_ST & FIFO_ST_VLD)) {
/*
 * Side-channel: check for async jobs queued by Core0.
 * We consume the pointer (clear before calling to prevent re-entry), run it, accumulate busy time, signal done, call SEV to wake Core0.
 * Done is signalled via the dedicated done flag, NOT via the SIO FIFO, to avoid corrupting the sync-dispatch done-token stream.
 */
            typedef void (*c1_task_fn)(void);

            /* --- attitude side-channel --- */
            uint32_t att_fn_raw = c1_att_fn_sidechan;
            if (att_fn_raw != 0U) {
                c1_att_fn_sidechan = 0U;   /* consume before calling (prevents re-entry) */
                __asm volatile ("dsb sy\n isb" ::: "memory");
                const uint32_t t0 = TIMER0_TIMERAWL;
                ((c1_task_fn)att_fn_raw)();
                c1_busy_us += TIMER0_TIMERAWL - t0;
                __asm volatile ("dmb sy" ::: "memory");
                c1_att_sidechan_done = 1U; /* signal completion to Core0 */
                __asm volatile ("sev");    /* wake Core0 if it is WFEing in c1_att_barrier */
            }

            /* --- covariance side-channel (separate from attitude) --- */
            uint32_t cov_fn_raw = c1_cov_fn_sidechan;
            if (cov_fn_raw != 0U) {
                c1_cov_fn_sidechan = 0U;   /* consume before calling (prevents re-entry) */
                __asm volatile ("dsb sy\n isb" ::: "memory");
                const uint32_t tc0 = TIMER0_TIMERAWL;
                ((c1_task_fn)cov_fn_raw)();
                c1_busy_us += TIMER0_TIMERAWL - tc0;
                __asm volatile ("dmb sy" ::: "memory");
                c1_cov_sidechan_done = 1U; /* signal completion to Core0 */
                __asm volatile ("sev");    /* wake Core0 if it is WFEing in c1_cov_barrier */
            }

            __asm volatile ("wfe"); // wait for event — low-power sleep until core0 writes to the FIFO and signals with SEV
            // small delay.
            for (volatile int i = 0; i < 100; i++) {
                __asm volatile ("nop");
            }
        }

        uint32_t msg = SIO_FIFO_RD;

        // if msg is 0, it's a ping from core0.  Write 0 back and wait for the next message.
        if (msg == 0U) {
            /* Null pointer = keepalive ping, just acknowledge. */
            while (!(SIO_FIFO_ST & FIFO_ST_RDY)) { /* spin */ }
            SIO_FIFO_WR = 0U;
            // small delay
            for (volatile int i = 0; i < 100; i++) {
                __asm volatile ("nop");
            }
            continue;
        }

        c1_boot_stage = 0x50U;  /* running task */

        // treat the message as a function pointer and call it.  The function is responsible for doing its own DMB if it needs to ensure memory visibility of its actions to core0 before signaling completion.
        typedef void (*c1_task_fn)(void);
        const uint32_t _c1_t0 = TIMER0_TIMERAWL;
        ((c1_task_fn)msg)();
        c1_busy_us += TIMER0_TIMERAWL - _c1_t0;  /* accumulate Core1 busy time in µs */

        c1_boot_stage = 0x51U; /* task returned, sending done */

        // notify core0 that task is complete
        __asm volatile ("dmb sy" ::: "memory");

        /* Signal completion to core0.  Spin-wait if TX FIFO is full
         * (should never happen in normal operation). */
        while (!(SIO_FIFO_ST & FIFO_ST_RDY)) { /* spin */ }
        SIO_FIFO_WR = 1U;  /* done */

        // 'sev' is a ARM instruction that signals an event to all cores. It sets the internal "event register" on every core in the system.
        // It's the counterpart to WFE (Wait For Event), which puts a core to sleep until its event register is set 
        __asm volatile ("sev");
    }
}