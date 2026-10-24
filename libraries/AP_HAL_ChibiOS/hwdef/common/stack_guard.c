/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "hwdef.h"
#include "stack_guard.h"
#if HAL_STACK_GUARD_ENABLED
#include "ch.h"
#include "hal.h"
#include "stack_guard_fault.h"

/* Region 7 must override the RAM cache-attribute regions. The null-pointer
 * guard uses region 3 when this facility is enabled. No thread may share a
 * guard region with another live allocation. */
void ap_stack_guard_switch(void *next)
{
    const struct AP_StackGuard *guard = ((thread_t *)next)->stack_guard;
    __DSB();
    if (guard != NULL) {
        MPU->RBAR = guard->base | MPU_RBAR_VALID_Msk | HAL_STACK_GUARD_MPU_REGION;
        MPU->RASR = MPU_RASR_ATTR_AP_NA_NA | MPU_RASR_ATTR_XN | MPU_RASR_SIZE_1K | MPU_RASR_ENABLE;
    } else {
        MPU->RBAR = MPU_RBAR_VALID_Msk | HAL_STACK_GUARD_MPU_REGION;
        MPU->RASR = 0;
    }
    __DSB();
    __ISB();
}

static void __attribute__((noreturn)) stack_fault_exit(struct AP_StackGuard *guard)
{
    /* Back in thread mode on a fresh stack, not in the exception handler.
     * No unwinding or destructor calls into the discarded module frames. */
    guard->fault(guard->context);

    /* Unlocking a contended mutex normally reschedules from within
     * chMtxUnlockAllS(). Avoid suspending this recovery frame in the middle of
     * teardown: terminate it in the same locked scheduler transaction that
     * makes the waiters ready. The temporary maximum priority suppresses the
     * internal reschedule; chThdExitS() then selects the next runnable thread.
     */
    chSysLock();
    thread_t *tp = chThdGetSelfX();
    const tprio_t realprio = tp->realprio;
    tp->realprio = HIGHPRIO;
    tp->hdr.pqueue.prio = HIGHPRIO;
    chMtxUnlockAllS();
    tp->realprio = realprio;
    tp->hdr.pqueue.prio = realprio;
    chThdExitS(MSG_RESET);
    __builtin_unreachable();
}

/* Never touch FP registers here: doing so can trigger lazy stacking onto the
 * very PSP that just overflowed. No logging, allocation or RTOS locking. */
uint32_t __attribute__((target("general-regs-only"), used, noinline))
ap_stack_guard_recover(uint32_t exc_return, uintptr_t psp)
{
    /* Do not inspect RTOS state until exception provenance is established. */
    if ((exc_return != 0xFFFFFFFDU && exc_return != 0xFFFFFFEDU) ||
        __get_PRIMASK() != 0 || __get_BASEPRI() != 0 || __get_FAULTMASK() != 0) {
        return 0;
    }
    thread_t *tp = chThdGetSelfX();
    if (tp == NULL) {
        return 0;
    }
    struct AP_StackGuard *guard = tp->stack_guard;
    if (guard == NULL || guard->faulted || tp->state != CH_STATE_CURRENT) {
        return 0;
    }
    const struct AP_StackFault fault = {
        exc_return, psp, __get_PRIMASK(), __get_BASEPRI(), __get_FAULTMASK(),
        SCB->ICSR, SCB->CFSR, SCB->HFSR, SCB->MMFAR, FPU->FPCCR, FPU->FPCAR,
        guard->base, guard->top
    };
    if (!ap_stack_fault_matches(&fault, HAL_STACK_GUARD_BYTES)) {
        return 0;
    }

    /* The old exception frame may be incomplete (MSTKERR). Do not read it.
     * The thread descriptor sits immediately ABOVE the usable stack. */
    volatile struct port_extctx *frame = (volatile struct port_extctx *)(guard->top - sizeof(struct port_extctx));
    volatile uint32_t *words = (volatile uint32_t *)frame;
    for (unsigned i = 0; i < sizeof(*frame) / sizeof(*words); i++) {
        words[i] = 0;
    }
    frame->r0 = (uintptr_t)guard;
    frame->pc = (uintptr_t)stack_fault_exit & ~1U;
    frame->xpsr = 1U << 24; /* Thumb, thread mode, no stack alignment word */
#if CORTEX_USE_FPU
    frame->fpscr = FPU->FPDSCR;
#endif
    guard->faulted = true;
    // Same cancellation used for deleted tasks by Arm CMSIS-RTX's ARMv7-M
    // SVC_FP_LazyState path. The interrupted task will never resume.
    FPU->FPCCR &= ~FPU_FPCCR_LSPACT_Msk;
#if CORTEX_USE_FPU
    // The ChibiOS ARMv7-M port always discards sizeof(port_extctx) in its
    // SVC/IRQ return path. Keep FPCA active so subsequent exceptions create
    // the extended frame that invariant requires.
    __set_CONTROL(__get_CONTROL() | CONTROL_FPCA_Msk);
    __ISB();
#endif
    __set_PSP((uintptr_t)frame);
    // Escalation can leave MemManage pending. Do not alter MEMFAULTACT: the
    // exception-return machinery, not software, completes the active handler.
    SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTPENDED_Msk;
    SCB->CFSR = fault.cfsr;
    SCB->HFSR = SCB_HFSR_FORCED_Msk;
    __DSB();
    __ISB();
    // ChibiOS uses sizeof(port_extctx) in its interrupt/SVC return paths.
    // Returning with a basic frame on an FPU build clears CONTROL.FPCA and
    // breaks the next preemption, even if this trampoline never uses FP.
#if CORTEX_USE_FPU
    return 0xFFFFFFEDU;
#else
    return 0xFFFFFFFDU;
#endif
}

/* The fatal dump scans memory, including the guard. Disable it before falling
 * through from MemManage (where the MPU is still active). */
void __attribute__((target("general-regs-only"), used, noinline)) ap_stack_guard_disable(void)
{
    mpuConfigureRegion(HAL_STACK_GUARD_MPU_REGION, 0, 0);
    __DSB();
    __ISB();
}

/* USB debug owns the fault vectors when enabled and attempts recovery before
 * inspecting an application exception frame or completing lazy FP stacking. */
#if !defined(AP_USB_DEBUG_ENABLED) || !AP_USB_DEBUG_ENABLED
/* Preserve EXC_RETURN and the original MSP for the ordinary fatal path. */
#define STACK_FAULT_ENTRY(fallback) \
    __asm volatile("push {r4, lr}\n" \
                   "mov r0, lr\n" \
                   "mrs r1, psp\n" \
                   "bl ap_stack_guard_recover\n" \
                   "cbz r0, 1f\n" \
                   "mov lr, r0\n" \
                   "ldr r4, [sp]\n" \
                   "add sp, sp, #8\n" \
                   "bx lr\n" \
                   "1: bl ap_stack_guard_disable\n" \
                   "pop {r4, lr}\n" \
                   "b " fallback "\n")

void __attribute__((naked)) MemManage_Handler(void)
{
    STACK_FAULT_ENTRY("AP_StackGuard_MemManage_Fallback");
}

void __attribute__((naked)) HardFault_Handler(void)
{
    STACK_FAULT_ENTRY("AP_StackGuard_HardFault_Fallback");
}
#endif
#endif
