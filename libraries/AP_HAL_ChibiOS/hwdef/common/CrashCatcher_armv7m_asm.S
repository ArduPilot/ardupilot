/*
 * Copyright (C) 2019 Adam Green (https://github.com/adamgreen)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Adapted for ArduPilot.
 */

#include "CrashCatcher.h"

    .text
    .syntax unified
    .arch armv7-m

    .global HardFault_Handler
    .type HardFault_Handler, %function
    .thumb_func
HardFault_Handler:
    /*
     * Save the exception registers before switching MSP to the dedicated
     * CrashCatcher stack. The resulting layout is CrashCatcherExceptionRegisters.
     */
    mrs     r3, xpsr
    mrs     r2, psp
    mrs     r1, msp
    ldr     sp, =(g_crashCatcherStack + 4 * CRASH_CATCHER_STACK_WORD_COUNT)
    push.w  {r1-r11,lr}

    mov     r0, sp
    bl      CrashCatcher_Entry

    /* Only breakpoints which request CRASH_CATCHER_EXIT return here. */
    pop.w   {r1-r11,lr}
    mov     sp, r1
    bx      lr

    .pool
    .size   HardFault_Handler, .-HardFault_Handler

    .global CrashCatcher_CopyAllFloatingPointRegisters
    .type CrashCatcher_CopyAllFloatingPointRegisters, %function
    .thumb_func
CrashCatcher_CopyAllFloatingPointRegisters:
#if CRASH_CATCHER_WITH_FPU
    vmrs        r1, fpscr
    vstmia.32   r0!, {s0-s31}
    str         r1, [r0]
#endif
    bx          lr

    .pool
    .size CrashCatcher_CopyAllFloatingPointRegisters, .-CrashCatcher_CopyAllFloatingPointRegisters

    .global CrashCatcher_CopyUpperFloatingPointRegisters
    .type CrashCatcher_CopyUpperFloatingPointRegisters, %function
    .thumb_func
CrashCatcher_CopyUpperFloatingPointRegisters:
#if CRASH_CATCHER_WITH_FPU
    vstmia.32   r0!, {s16-s31}
#endif
    bx          lr

    .pool
    .size CrashCatcher_CopyUpperFloatingPointRegisters, .-CrashCatcher_CopyUpperFloatingPointRegisters

    .end
