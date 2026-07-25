/* Copyright 2017 Adam Green (http://mbed.org/users/AdamGreen/)

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
/* Implementation of Cortex-M architecture assembly language routines to be used by mri debug monitor. */
    .text
    .code 16
    .syntax unified

    #include "armv7-m.h"

    .equ CFSR,                   0xE000ED28
    .equ CFSR_STACK_ERROR_BITS,  0x00001818
    .equ CFSR_BUS_IMPRECISE_BIT, 0x00000400
    .equ PSR_STACK_ALIGN,        0x00000200
    /* Bit in LR to indicate whether PSP was used for automatic stacking of registers during exception entry. */
    .equ LR_PSP,                 (1 << 2)
    /* Bit in LR set to 0 when automatic stacking of floating point registers occurs during exception handling. */
    .equ LR_FLOAT_STACK,         (1 << 4)
    /* Coprocessor Access Control Register. */
    .equ CPACR,                 0xE000ED88
    /* Bits set in CPACR if FPU is enabled. */
    .equ CPACR_FPU_BITS,        (5 << 20)

    .global __mriExceptionHandler
    .type __mriExceptionHandler, function
    .thumb_func
    /* Saves task SP in R1 and g_MriTaskSP before saving context and calling main mri debugger entry point. */
__mriExceptionHandler:
    ldr     r0, =mriSaveRestoreContext
    b       mriGetSPAndCallHandler
    

    /* Like MriExceptionHandler above but it also checks and handles stacking/unstacking faults. */
mriFaultHandler:
    ldr     r0, =checkForStackError
    b       mriGetSPAndCallHandler
    
    
    /* Determines whether task context was saved in PSP or MSP and places the correct stack pointer address into
       the R1 register before calling the caller provided fault handler pointed to by the r0 register.
    */
mriGetSPAndCallHandler:
    /* Task may have been using PSP or MSP so retrieve the correct one into R1. */
    tst     lr, #LR_PSP
    ite     eq
    mrseq   r1, msp
    mrsne   r1, psp
    /* Save away task SP so that C code can query it later. */
    ldr     r2, =(__mriCortexMState + CORTEXM_STATE_TASK_SP_OFFSET)
    str     r1, [r2]
    /* Call specified handler exception handler routine */
    mov     pc, r0
    
    
    .type checkForStackError, function
    .thumb_func
    /* Handles hard/memory/bus fault exceptions by checking for stacking/unstacking errors before calling the main
       MriExceptionHandler.  Sets r1 register to point to a fake stack frame in FLASH when a stacking/unstacking
       error occurs rather than pointing to the interrupted process's stack pointer as usual.
    */
checkForStackError:
    /* See if a stacking/unstacking error is flagged in the memory or bus fault registers. */
    ldr     r0, =CFSR
    ldr     r2, =CFSR_STACK_ERROR_BITS
    ldr     r0, [r0]
    tst     r0, r2
    beq     mriSaveRestoreContext
    /* Get here when there is a bus/memory stacking/unstacking error. */
    ldr     r1, =__mriCortexMFakeStack
    b       mriSaveRestoreContext


    .type mriSaveRestoreContext, function
    /* Handles exception invokations for MRI.  Its main job is to store the interrupted task's context, call into
       the C based __mriDebugException, and then restore the task's context (which may have been modified by the
       debugger.
       
       Layout of the registers which are pushed on the stack automatically by Cortex-M3 curing exception execution:
            SP      Value in MSP or PSP task was interrupted.
        PSR SP - 4
        PC  SP - 8
        LR  SP - 12
        R12 SP - 16
        R3  SP - 20
        R2  SP - 24
        R1  SP - 28
        R0  SP - 32  Value in MSP or PSP when handler is started.
        
       Layout of the SContext record used by gdb for 'g' and 'G' commands.
        typedef struct
        {
            unsigned int    R0;
            unsigned int    R1;
            unsigned int    R2;
            unsigned int    R3;
            unsigned int    R4;
            unsigned int    R5;
            unsigned int    R6;
            unsigned int    R7;
            unsigned int    R8;
            unsigned int    R9;
            unsigned int    R10;
            unsigned int    R11;
            unsigned int    R12;
            unsigned int    SP;
            unsigned int    LR;
            unsigned int    PC;
            unsigned int    CPSR;
        } SContext;
    */
mriSaveRestoreContext:
    /**** Detect fault encountered during debug and in such cases, only set flag and skip overwrite of existing context. */
    ldr     r0, =(__mriCortexMState + CORTEXM_STATE_FLAGS_OFFSET)
    ldr     r2, [r0]
    /* If already debugging, then set flag and jump to code which will advance PC before resuming execution. */
    tst     r2, #CORTEXM_FLAGS_ACTIVE_DEBUG
    ittt    ne
    orrne   r2, #CORTEXM_FLAGS_FAULT_DURING_DEBUG
    strne   r2, [r0]
    bne     mriAdvancePCAndReturn
    /* Set the debugger active flag so that subsequent exceptions will be caught. */
    orr     r2, #CORTEXM_FLAGS_ACTIVE_DEBUG
    str     r2, [r0]
    
    /* Fill the debugger stack with 0xDEADBEEF. */
    movs    r0, #CORTEXM_DEBUGGER_STACK_SIZE
    ldr     r12, =(__mriCortexMState + CORTEXM_STATE_DEBUGGER_STACK_OFFSET + CORTEXM_DEBUGGER_STACK_SIZE_IN_BYTES)
    ldr     r2, =CORTEXM_DEBUGGER_STACK_FILL
    mov     r3, r2
1$:
    strd    r2, r3, [r12, #-8]!
    subs    r0, #1
    bne     1$

    /**** Save current MSP into global and then switch to special debugger stack. */
    mrs     r2, msp
    ldr     r0, =(__mriCortexMState + CORTEXM_STATE_SAVED_MSP_OFFSET)
    str     r2, [r0]
    ldr     r0, =(__mriCortexMState + CORTEXM_STATE_DEBUGGER_STACK_OFFSET + CORTEXM_DEBUGGER_STACK_SIZE_IN_BYTES)
    msr     msp, r0
    
    /**** Copy R0-R3, R12, LR, PC, and PSR from running task's stack and place in MRI context structure. */
    ldr     r0, =(__mriCortexMState + CORTEXM_STATE_CONTEXT_OFFSET)
    /* Now that R1 contains the SP, we can push away the current LR (and R12 to just keep 8-byte alignment.) */
    push    {r12,lr}
    /* R0 is now destination pointer to beginning of context record.
       R1 is now source pointer to end of auto stacking area. */
    /* Copy R0 - R3 from stacked area to context. */
    ldmia   r1!, {r2,r3,r12,lr}
    stmia   r0!, {r2,r3,r12,lr}
    /* Store R4 - R11 to context. */
    stmia   r0!, {r4-r11}
    /* Load R12, LR, PC, and PSR from stacked area. */
    ldmia   r1!, {r2-r5}
    /* Store R12 to context. */
    str     r2, [r0], #8      /* Skip over SP which will be saved in context just before calling __mriDebugException. */
    /* Store LR, PC, and PSR to context. */
    stmia   r0!, {r3, r4, r5}
    /* Skip over MSP since it was already saved in context. */
    adds    r0, #4
    /* Store PSP, PRIMASK, BASEPRI, FAULTMASK, and CONTROL to context. */
    mrs     r2, psp
    mrs     r3, primask
    mrs     r4, basepri
    mrs     r6, faultmask /* Don't use r5 since later code expects it to still contain PSR. */
    mrs     r7, control
    stmia   r0!, {r2-r4, r6-r7}

    /* Fetch the stacked lr back into r2 and check for auto-stacked FPU registers. */
    ldr     r2, [sp, #4]
    tst     r2, #LR_FLOAT_STACK

#if !MRI_DEVICE_HAS_FPU
    /* Advance R1 past the auto-stacked FPU registers if LR_FLOAT_STACK bit in LR is 0. */
    /* NOTE: This code only runs when device has FPU but MRI was compiled without FPU support. */
    it      eq
    addeq   r1, r1, #0x48
#else /* MRI_DEVICE_HAS_FPU */
    bne     3$

    /* If LR_FLOAT_STACK bit in LR is 0 then pull auto-stacked S0-S15 and FPSCR registers off of exception stack. */
    vldmia.32 r1!, {s0-s15}
    ldr       r2, [r1], #8    /* FPSCR->R2 and 8-byte alignment padding. */

    /* Write all FPU registers to context. FPSCR should be in R2. */
2$: vstmia.32   r0!, {s0-s31}
    str         r2, [r0], #4
    b           5$

    /* Get here if no floating point registers were auto-stacked. */
    /* If FPU is enabled then move current FPSCR into R2 and then save all FPU registers to context. */
3$: ldr     r2, =CPACR
    ldr     r2, [r2]
    and     r2, #CPACR_FPU_BITS
    cmp     r2, #CPACR_FPU_BITS
    itt     eq
    vmrseq  r2, fpscr
    beq     2$

    /* Get here if FPU isn't enabled so zero out FPU registers in context. */
    mov     r2, #0
    mov     r3, #33
4$: str     r2, [r0], #4
    subs    r3, #1
    bne     4$

5$: /* Set R0 context pointer back to beginning of FPU registers where subsequent code expects it. */
    subs    r0, #(33*4)
#endif /* _FPU_PRESENT */

    /* R1 will now be pointing to the task's SP at interrupt time unless the stack was 8-byte aligned. */
    tst     r5, #PSR_STACK_ALIGN
    it      ne
    addne   r1, r1, #4
    /* Jump back past the CONTROL, FAULTMASK, BASEPRI, PRIMASK, PSP, MSP, PSR, PC and LR that were placed in context 
       to store the potentially updated SP. */
    str     r1, [r0, #-40]


    /**** Run the C routine which allows for debugging of exceptions. */
    bl      __mriDebugException


    /**** Restore the tasks' registers which may have been modified by debugger. */
    /* Point context source pointer to location of PSR data. */
    ldr     r0, =(__mriCortexMState + CORTEXM_STATE_CONTEXT_OFFSET + 16*4)
    /* Retrieve value of task's SP at time of interrupt.  May have been changed by debug client. */
    ldr     r1, [r0, #-(3*4)]

#if !MRI_DEVICE_HAS_FPU
    /* Advance down past the auto-stacked FPU registers if necessary. */
    /* Fetch the stacked lr back into r2. */
    ldr     r2, [sp, #4]
    tst     r2, #LR_FLOAT_STACK
    it      eq
    subeq   r1, r1, #0x48
#else /* MRI_DEVICE_HAS_FPU */
    /* See if the FPU is enabled. */
    ldr     r2, =CPACR
    ldr     r2, [r2]
    and     r2, #CPACR_FPU_BITS
    cmp     r2, #CPACR_FPU_BITS
    bne     6$

    /* If FPU is enabled then pull all FPU register values out of the debug context. */
    /* R0 is currently pointing at the XPSR value.  Advance to point to S0 instead. */
    adds        r0, #(7 * 4)
    vldmia.32   r0!, {s0-s31}
    ldr         r2, [r0]
    vmsr        fpscr, r2
    /* Point R0 back to the PSR in the debug context. */
    subs        r0, #(39 * 4)

    /* Copy S0-S15,FPSCR to auto-stacked area if the auto-stack bit is set in exception LR. */
    /* Fetch the stacked lr back into r3 and check for the auto-stack bit. */
6$: ldr     r3, [sp, #4]
    tst     r3, #LR_FLOAT_STACK
    itt     eq
    /* Store FPSR and LR (as padding word) to exception stack. */
    stmdbeq     r1!, {r2-r3}
    /* Store S0-S15 to exception stack. */
    vstmdbeq.32 r1!, {s0-s15}
#endif /* MRI_DEVICE_HAS_FPU */

    /* Force double word stack alignment off in PSR. */
    ldr     r2, [r0]
    bic     r2, #PSR_STACK_ALIGN
    /* Store PSR on task stack. */
    str     r2, [r1, #-4]!
    /* Copy PC and LR from context to task stack. */
    ldmdb   r0!, {r2,r3}
    stmdb   r1!, {r2,r3}
    /* Copy r12 from context to stack area.  Skip over SP in context as it was already handled above. */
    ldr     r12, [r0, #-8]!
    str     r12, [r1, #-4]!
    /* Load R4-R11 from context. */
    ldmdb   r0!, {r4-r11}
    /* Copy R0 - R3 from context to task stack. */
    ldmdb   r0!, {r2,r3,r12,lr}
    stmdb   r1!, {r2,r3,r12,lr}
    /* Restore lr from the debugger stack. */
    pop     {r12,lr}
    /* Restore the MSP back to what it was before switching it to the debugger stack. */
    ldr     r0, =(__mriCortexMState + CORTEXM_STATE_SAVED_MSP_OFFSET)
    ldr     r2, [r0]
    msr     msp, r2
    /* Task may have been using PSP or MSP so store the new SP (after auto stacking) in appropriate register. */
    tst     lr, #0x4
    ite     eq
    msreq   msp,r1
    msrne   psp,r1
    /* Clear the debugger active flag. */
    ldr     r0, =(__mriCortexMState + CORTEXM_STATE_FLAGS_OFFSET)
    ldr     r1, [r0]
    bic     r1, #CORTEXM_FLAGS_ACTIVE_DEBUG
    str     r1, [r0]

    /**** Make sure that any modifications to the instruction stream by data writes are seen. */
    dsb
    isb
    
    /**** Return to task after clearing fault status bits. */
    b       mriClearFaultStatusBitsAndReturn


    .type mriAdvancePCAndReturn, function
    .thumb_func
    /* Branched to from MriException handler if the debugger is already active.  Getting an exception in such a
       scenario typically means that a memory fault occurred while attempting to access an invalid memory address so
       it should be flagged as such and then the offending instruction skipped.  The debug monitor can then check the
       flag to see if its memory access caused a fault or not. */
mriAdvancePCAndReturn:
    /* Can skip the advance if the fault was an imprecise data write since the faulting instruction was in the past. */
    ldr     r0, =CFSR
    ldr     r0, [r0]
    tst     r0, #CFSR_BUS_IMPRECISE_BIT
    bne     mriClearFaultStatusBitsAndReturn
    /* Load the current value of the PC into R0 and place the instruction code in R2. */
    ldr     r0, [r1, #24]
    ldrh    r2, [r0]
    /* Add 2 or 4 to the PC value depending on whether the instruction is 16 or 32-bit in length. */
    and     r2, r2, #0xF800
    cmp     r2, #0xE800
    beq     1$  /* instructionIs32Bit */
    cmp     r2, #0xF000
    beq     1$  /* instructionIs32Bit */
    cmp     r2, #0xF800
    beq     1$  /* instructionIs32Bit */
    /* instructionIs16Bit */
    add     r0, r0, #2
    b       2$  /* storePCAndReturn */
    /* instructionIs32Bit */
1$:
    add     r0, r0, #4
    /* storePCAndReturn */
2$:
    str     r0, [r1, #24]
    b       mriClearFaultStatusBitsAndReturn


    /* Clears the mem/bus/usage/hard fault status bits before returning to the debuggee. */
mriClearFaultStatusBitsAndReturn:
    ldr     r0, =CFSR
    ldr     r1, [r0]
    str     r1, [r0]
    ldr     r1, [r0, 4]
    str     r1, [r0, 4]
    bx      lr


    .global HardFault_Handler
    .type HardFault_Handler, function
    .thumb_func
    /* extern "C" void HardFault_Handler(void);
       Override Hard Faults and send to MriExceptionHandler.
    */
HardFault_Handler:
    b       mriFaultHandler
    
 
    .global MemManage_Handler
    .type MemManage_Handler, function
    .thumb_func
    /* extern "C" void MemManage_Handler(void);
       Override MPU Memory Faults and send to MriExceptionHandler.
    */
MemManage_Handler:
    b       mriFaultHandler
    

    .global BusFault_Handler
    .type BusFault_Handler, function
    .thumb_func
    /* extern "C" void BusFault_Handler(void);
       Override Bus Faults and send to MriExceptionHandler.
    */
BusFault_Handler:
    b       mriFaultHandler
    

    .global UsageFault_Handler
    .type UsageFault_Handler, function
    .thumb_func
    /* extern "C" void UsageFault_Handler(void);
       Override Instruction Usage Faults and send to MriExceptionHandler.
    */
UsageFault_Handler:
    b       __mriExceptionHandler
    

    .global DebugMon_Handler
    .type DebugMon_Handler, function
    .thumb_func
    /* extern "C" void DebugMon_Handler(void);
       Override Debug Monintor exceptions and send to MriExceptionHandler.
    */
DebugMon_Handler:
    b       __mriExceptionHandler
