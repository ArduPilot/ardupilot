/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Cortex-M7 register snapshot. Kept independent of CMSIS/ChibiOS so the
 * fail-closed classification can also be exhaustively tested on the host. */
struct AP_StackFault {
    uint32_t exc_return, psp, primask, basepri, faultmask;
    uint32_t icsr, cfsr, hfsr, mmfar, fpccr, fpcar;
    uint32_t base, top;
};

static inline bool ap_stack_fault_matches(const struct AP_StackFault *f, uint32_t guard_bytes)
{
    const uint32_t daccviol = 1U << 1;
    const uint32_t mstkerr = 1U << 4;
    const uint32_t mmarvalid = 1U << 7;
    const uint32_t permitted = daccviol | mstkerr | mmarvalid;
    if ((f->exc_return != 0xFFFFFFFDU && f->exc_return != 0xFFFFFFEDU) ||
        f->primask != 0 || f->basepri != 0 || f->faultmask != 0 ||
        (f->icsr & (1U << 11)) == 0 || /* RETTOBASE: no suspended handler */
        (f->cfsr & ~permitted) != 0 || (f->cfsr & (daccviol | mstkerr)) == 0 ||
        (f->hfsr & ~(1U << 30)) != 0 || /* only FORCED escalation */
        f->base > UINT32_MAX - guard_bytes || f->base + guard_bytes > UINT32_MAX - 256U ||
        f->psp < f->base || f->psp > f->base + guard_bytes + 256U) {
        return false;
    }
    if (f->cfsr & mmarvalid) {
        if (f->mmfar < f->base || f->mmfar >= f->base + guard_bytes) {
            return false;
        }
    } else if (!(f->cfsr & mstkerr) || f->psp >= f->base + guard_bytes) {
        return false;
    }
    /* Pending FP preservation must belong to the abandoned thread. */
    if ((f->fpccr & 1U) &&
        ((f->exc_return & 0x10U) || f->fpcar < f->base || f->fpcar >= f->top)) {
        return false;
    }
    return true;
}
