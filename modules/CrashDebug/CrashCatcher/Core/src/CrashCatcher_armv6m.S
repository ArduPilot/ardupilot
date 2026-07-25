/* Copyright (C) 2019  Adam Green (https://github.com/adamgreen)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include "CrashCatcherPriv.h"

/* Implementation of ARMv6-M assembly language code to trap exceptions and call CrashCatcher_Entry(). */
    .text
    .syntax unified
    .arch armv6-m

    /* Called on Hard Fault exception.  Stacks important registers and calls CrashCatcher_Entry().

        extern "C" void HardFault_Handler(void);
    */
    .global HardFault_Handler
    .type HardFault_Handler, %function
    .thumb_func
HardFault_Handler:
    /* Push the following onto the stack (see CrashCatcherExceptionRegisters structure). The g_crashCatcherStack buffer
       is reserved for use as the stack while CrashCatcher is running.
        msp
        psp
        exceptionPSR
        r4
        r5
        r6
        r7
        r8
        r9
        r10
        r11
        exceptionLR */
    mrs     r12, msp
    ldr     r0, =(g_crashCatcherStack + 4 * CRASH_CATCHER_STACK_WORD_COUNT)
    mov     sp, r0
    mov     r0, r8
    mov     r1, r9
    mov     r2, r10
    mov     r3, r11
    push    {r0-r3, lr}
    mrs     r3, xpsr
    mrs     r2, psp
    mov     r1, r12
    push    {r1-r7}

    // Call CrashCatcher_Entry with first argument pointing to registers that were just stacked.
    mov     r0, sp
    bl      CrashCatcher_Entry

    // Only make it back here when CrashCatcher_DumpEnd() returns CRASH_CATCHER_EXIT to indicate that it would like
    // execution to be restored back to the faulting code (typically a hard coded breakpoint).
    // Restore non-volatile registers and SP to values they had upon entry to fault handler before resuming execution
    // at point of fault.
    pop     {r1-r7}
    mov     r12, r1
    pop     {r0-r3}
    mov     r11, r3
    mov     r10, r2
    mov     r9, r1
    mov     r8, r0
    pop     {r0}
    mov     sp, r12
    bx      r0

    // Let assembler know that we have hit the end of the HardFault_Handler function.
    .pool
    .size   HardFault_Handler, .-HardFault_Handler



    /* Called from CrashCatcher core to copy all floating point registers to supplied buffer. The supplied buffer must
       be large enough to contain 33 32-bit values (S0-S31 & FPSCR).

        void CrashCatcher_CopyAllFloatingPointRegisters(uint32_t* pBuffer);
    */
    .global CrashCatcher_CopyAllFloatingPointRegisters
    .type CrashCatcher_CopyAllFloatingPointRegisters, %function
    .thumb_func
CrashCatcher_CopyAllFloatingPointRegisters:
    // Just return on ARMv6-M as it doesn't support floating point.
    bx      lr

    .pool
    .size   CrashCatcher_CopyAllFloatingPointRegisters, .-CrashCatcher_CopyAllFloatingPointRegisters



    /* Called from CrashCatcher core to copy upper 16 floating point registers to supplied buffer. The supplied buffer
       must be large enough to contain 16 32-bit values (S16-S31).

        void CrashCatcher_CopyUpperFloatingPointRegisters(uint32_t* pBuffer);
    */
    .global CrashCatcher_CopyUpperFloatingPointRegisters
    .type CrashCatcher_CopyUpperFloatingPointRegisters, %function
    .thumb_func
CrashCatcher_CopyUpperFloatingPointRegisters:
    // Just return on ARMv6-M as it doesn't support floating point.
    bx      lr

    .pool
    .size   CrashCatcher_CopyUpperFloatingPointRegisters, .-CrashCatcher_CopyUpperFloatingPointRegisters


    .end