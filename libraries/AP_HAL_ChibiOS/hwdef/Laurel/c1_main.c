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

/* NVIC Set-Enable registers (banked per-core); used by XIP lockout handler. */
#define NVIC_ISER0  (*(volatile uint32_t *)0xE000E100U)
#define NVIC_ISER1  (*(volatile uint32_t *)0xE000E104U)

/* Core1's SIO doorbell IN_CLR — de-asserts IRQ (banked; Core1 view = 0xD000018C). */
#define SIO_DOORBELL_IN_CLR  (*(volatile uint32_t *)0xD000018CU)

/* WATCHDOG SCRATCH[0..5]: survive warm reset. [2..5] = Core1 fault diagnostics
 * (sentinel/CFSR/HFSR/VTOR). [0..1] reserved for bootloader handoff. */
#define WD_SCRATCH0     (*(volatile uint32_t *)0x400D800CU)
#define WD_SCRATCH1     (*(volatile uint32_t *)0x400D8010U)
#define WD_SCRATCH2     (*(volatile uint32_t *)0x400D8014U)
#define WD_SCRATCH3     (*(volatile uint32_t *)0x400D8018U)
#define WD_SCRATCH4     (*(volatile uint32_t *)0x400D801CU)
#define WD_SCRATCH5     (*(volatile uint32_t *)0x400D8020U)


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

/* XIP lockout: 0=free, 1=Core0 requested, 2=Core1 parked. Defined in board_rp2350.c.
   In the bootloader build board_rp2350.c is not linked, so define them here. */
#ifdef HAL_BOOTLOADER_BUILD
volatile uint32_t c1_xip_lock = 0U;
volatile uint32_t c1_xip_lock_ready = 0U;
#else
extern volatile uint32_t c1_xip_lock;
extern volatile uint32_t c1_xip_lock_ready;
#endif

volatile uint32_t c1_boot_stage = 0xDEAD0000U;
volatile uint32_t c1_heartbeat = 0U;

/* c1_fault_info[8]: GDB "p c1_fault_info" — [0]CFSR [1]HFSR [2]MMFAR [3]SFSR [4]SFAR [5]PSP [6]MSP [7]VTOR */
volatile uint32_t c1_fault_info[8];

/* Core1 vector table in SRAM9 (0x20081000, 256-byte aligned). Non-striped bank
 * avoids IBUSERR when Core0 hits the same bank during Core1's vector fetch. */
#define c1_vtable ((volatile uint32_t *)0x20081000U)

/* XIP lockout handler (IRQ26/SIO_BELL, .ramtext): de-asserts doorbell, saves/disables
 * Core1 NVIC, signals Core0 (lock=2), spins until lock=0, then restores NVIC. */
__attribute__((section(".ramtext"), noinline, used))
static void c1_xip_lockout_handler(void)
{
    /* De-assert the doorbell IRQ line so it doesn't re-fire after we return */
    SIO_DOORBELL_IN_CLR = 0xFFU;
    __asm volatile ("dsb sy\n isb" ::: "memory");

    /* Save and disable ALL Core1 IRQs (NVIC banked per-core on RP2350) */
    const uint32_t saved_iser0 = NVIC_ISER0;
    const uint32_t saved_iser1 = NVIC_ISER1;
    NVIC_ICER0 = 0xFFFFFFFFU;
    NVIC_ICER1 = 0xFFFFFFFFU;
    __asm volatile ("dsb sy\n isb" ::: "memory");

    /* Signal Core0: Core1 is parked, XIP-off is now safe */
    c1_xip_lock = 2U;
    __asm volatile ("dsb sy" ::: "memory");

    /* Spin in SRAM — no instruction fetch from flash while XIP is disabled */
    while (c1_xip_lock != 0U) {
        __asm volatile ("" ::: "memory");
    }

    /* XIP restored by Core0; re-enable the IRQs that were active before */
    NVIC_ISER0 = saved_iser0;
    NVIC_ISER1 = saved_iser1;
    __asm volatile ("dsb sy\n isb" ::: "memory");
    /* Returns via EXC_RETURN — hardware restores pre-interrupt register state */
}

/* SRAM fault handler (.ramtext, used): saves CFSR/HFSR/etc to c1_fault_info[] + WD_SCRATCH, spins for GDB. */
__attribute__((section(".ramtext"), noinline, used))
static void c1_sram_fault_handler(void)
{
    /* Sentinel to WD_SCRATCH[2] first — survives SYSRESETREQ, readable after reset. */
    WD_SCRATCH2 = 0xC1FA0001U;

    /* Also set the SRAM sentinel for GDB sessions that catch the crash live */
    c1_boot_stage = 0xFA000001U;

    /* Fault status registers — PPB is always accessible in privileged mode */
    uint32_t cfsr = *(volatile uint32_t *)0xE000ED28U; /* CFSR  */
    uint32_t hfsr = *(volatile uint32_t *)0xE000ED2CU; /* HFSR  */
    uint32_t vtor = *(volatile uint32_t *)0xE000ED08U; /* VTOR at fault */

    /* Mirror to WATCHDOG SCRATCH (survive reset) and SRAM (readable live) */
    WD_SCRATCH3 = cfsr;
    WD_SCRATCH4 = hfsr;
    WD_SCRATCH5 = vtor;

    c1_fault_info[0] = cfsr;
    c1_fault_info[1] = hfsr;
    c1_fault_info[2] = *(volatile uint32_t *)0xE000ED34U; /* MMFAR */
    c1_fault_info[3] = *(volatile uint32_t *)0xE000EDE4U; /* SFSR  */
    c1_fault_info[4] = *(volatile uint32_t *)0xE000EDE8U; /* SFAR  */

    uint32_t psp, msp;
    __asm volatile ("mrs %0, PSP" : "=r"(psp));
    __asm volatile ("mrs %0, MSP" : "=r"(msp));
    c1_fault_info[5] = psp;
    c1_fault_info[6] = msp;
    c1_fault_info[7] = vtor;

    /* Write-to-clear fault status so we don't re-enter immediately */
    *(volatile uint32_t *)0xE000ED28U = cfsr; /* clear CFSR */
    *(volatile uint32_t *)0xE000ED2CU = hfsr; /* clear HFSR */

    /* Spin here — attach GDB and read c1_fault_info[] on rp2350.dap.core1 */
    while (1) {}
}

void __c1_cpu_init(void)
{
    /* Clear SCRATCH[0..5] on each boot so all fields are fresh. */
    WD_SCRATCH0 = 0U;
    WD_SCRATCH1 = 0U;
    WD_SCRATCH2 = 0U;
    WD_SCRATCH3 = 0U;
    WD_SCRATCH4 = 0U;
    WD_SCRATCH5 = 0U;

/*
 * Scrub Core 1's NVIC before anything else runs.
 * The RP2350 ROM's Core 1 launch protocol (6-step FIFO handshake) leaves SPARE_IRQ_1 (IRQ 47, VectorFC) enabled in Core 1's banked NVIC registers.
 */
    NVIC_ICER0 = 0xFFFFFFFFU;  /* disable IRQs  0..31 on Core 1 */
    NVIC_ICER1 = 0xFFFFFFFFU;  /* disable IRQs 32..51 on Core 1 (covers SPARE_IRQ_1=47) */
    NVIC_ICPR0 = 0xFFFFFFFFU;  /* clear pending IRQs  0..31 on Core 1 */
    NVIC_ICPR1 = 0xFFFFFFFFU;  /* clear pending IRQs 32..51 on Core 1 */
    __asm volatile ("dsb sy\n isb" ::: "memory");

/* Disable MPU before ChibiOS reinitialises it; ROM may leave regions active. */
    {
        volatile uint32_t *mpu_ctrl = (volatile uint32_t *)0xE000ED94U;
        *mpu_ctrl = 0U;
    }
    __asm volatile ("dsb sy\n isb" ::: "memory");

/*
 * Re-assert VTOR on Core 1.
 */
    extern uint32_t _vectors[];
    SCB_VTOR = (uint32_t)_vectors;
    c1_vtor_at_boot = (uint32_t)_vectors;

/* Copy vector table to c1_vtable (SRAM9); redirect fault handlers [3..7] to
 * c1_sram_fault_handler and IRQ26 to c1_xip_lockout_handler. */
    {
        volatile uint32_t *orig = (volatile uint32_t *)_vectors;
        for (unsigned i = 0; i < 64U; i++) {
            c1_vtable[i] = orig[i];
        }
        uint32_t h = (uint32_t)c1_sram_fault_handler | 1U; /* set Thumb bit */
        c1_vtable[3] = h;  /* HardFault   */
        c1_vtable[4] = h;  /* MemManage   */
        c1_vtable[5] = h;  /* BusFault    */
        c1_vtable[6] = h;  /* UsageFault  */
        c1_vtable[7] = h;  /* SecureFault */
        /* XIP lockout: VectorA8 = SIO_BELL = IRQ26 = c1_vtable[42] */
        c1_vtable[42] = (uint32_t)c1_xip_lockout_handler | 1U;
        __asm volatile ("dsb sy\n isb" ::: "memory");
        SCB_VTOR = (uint32_t)c1_vtable;
    }

    /* IRQ26 at CORTEX_MINIMUM_PRIORITY (0xF0): masked by BASEPRI_KERNEL during kernel locks.
     * Without this, lockout firing while Core1 holds the SMP spinlock causes deadlock.
     * NVIC_IPR6 bits[23:16] cover IRQ26 (byte 2 of IPR6). */
    {
        volatile uint32_t *nvic_ipr6 = (volatile uint32_t *)0xE000E418U;
        const uint32_t min_prio = CORTEX_PRIO_MASK(CORTEX_MINIMUM_PRIORITY); /* 0xF0 */
        *nvic_ipr6 = (*nvic_ipr6 & ~(0xFFU << 16U)) | (min_prio << 16U);
        __asm volatile ("dsb sy\n isb" ::: "memory");
    }

/*
 * Enable configurable-priority fault handlers on Core 1.
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
    WD_SCRATCH1 = 0xBB000003U;  /* milestone: c1_main entered */
#if CH_CFG_SMP_MODE == TRUE
    /* Wait for core0's chSysInit() to complete. */
    chSysWaitSystemState(ch_sys_running);
    c1_boot_stage = 0x31U;
    WD_SCRATCH1 = 0xBB000031U;  /* milestone: after chSysWaitSystemState */

    /* Initialise core1's OS instance; ch1.rlist.current is now valid. */
    chInstanceObjectInit(&ch1, &ch_core1_cfg);
    c1_boot_stage = 0x32U;
    WD_SCRATCH1 = 0xBB000032U;  /* milestone: after chInstanceObjectInit */

    /* Arm XIP lockout IRQ26 (SIO_BELL) — must be after chInstanceObjectInit. */
    NVIC_ISER0 |= (1U << 26);   /* enable IRQ26 (SIO_BELL) on Core1 */
    __asm volatile ("dsb sy" ::: "memory");
    c1_xip_lock_ready = 1U;     /* signal Core0: lockout protocol is armed */

    /* Capture VTOR + c1_vtable[41] right after port_init() to detect changes. */
    c1_fault_info[7] = SCB_VTOR;
    WD_SCRATCH1 = 0xBB000033U;  /* milestone: after VTOR capture */

    /* Rename the main thread while still holding the I-Lock. */
    ch1.mainthread.name = "c1_main";

    /* Snapshot c1_vtable addr and c1_vtable[41] before unlocking. */
    c1_fault_info[6] = c1_vtable[41];  /* what's the SIO handler entry? */

    /* Release the I-Lock — scheduling on core1 starts here. */
    WD_SCRATCH1 = 0xBB000034U;  /* milestone: about to chSysUnlock */
    chSysUnlock();
    WD_SCRATCH1 = 0xBB000035U;  /* milestone: after chSysUnlock */
    c1_boot_stage = 0x33U;

    while (true) {
        chThdSleepMilliseconds(100U);
        c1_heartbeat++;
    }
#else
    while (true) {}
#endif
}