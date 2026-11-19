/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

/* Independent HAL facility: no dependency on the optional module runtime.
 * Opt-in until fault entry/return has also been validated on physical H7s. */
#ifndef HAL_STACK_GUARD_ENABLED
#define HAL_STACK_GUARD_ENABLED 0
#endif

#if HAL_STACK_GUARD_ENABLED
#if !defined(STM32H7) || defined(HAL_BOOTLOADER_BUILD) || defined(IOMCU_FW)
#error Stack fault recovery currently requires an H7 application build
#endif
#define HAL_STACK_GUARD_BYTES 1024U
#define HAL_STACK_GUARD_MPU_REGION 7

#ifndef __ASSEMBLER__
#include <stdint.h>
#include <stdbool.h>
struct AP_StackGuard {
    uintptr_t base;
    uintptr_t top;
    void *context;
    void (*fault)(void *context);
    bool faulted;
};
#ifdef __cplusplus
extern "C" {
#endif
void ap_stack_guard_switch(void *next);
void ap_stack_guard_disable(void);
#ifdef __cplusplus
}
#endif
#endif
#endif
