/* *****************************************************************************
 * The MIT License
 *
 exception handling and task switcher, (C) 2017 night_ghost@ykoctpa.ru

  adedd some useful info to __error handler and debugger, dual stacks support and task switching code
 
 
    based on:
 
 
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ****************************************************************************/

# On an exception, push a fake stack thread mode stack frame and redirect
# thread execution to a thread mode error handler

# From RM008:
# The SP is decremented by eight words by the completion of the stack push.
# Figure 5-1 shows the contents of the stack after an exception pre-empts the
# current program flow.
#
# Old SP--> <previous>
#           xPSR        28
#           PC          24
#           LR          20
#           r12         16
#           r3          12
#           r2          8
#           r1          4
#    SP-->  r0          0

  .syntax unified
  .cpu cortex-m4
.text
.globl HardFault_Handler
.globl NMI_Handler
.globl MemManage_Handler
.globl BusFault_Handler
.globl UsageFault_Handler
.globl __default_exc
.globl __do_context_switch

.code 16
.thumb_func
HardFault_Handler:
    mov r0, #2
    b __default_exc

.thumb_func
MemManage_Handler:
    mov r0, #3
    b __default_exc

.thumb_func
BusFault_Handler:
    mov r0, #4
    b __default_exc

.thumb_func
UsageFault_Handler:
    mov r0, #5
    b __default_exc

.thumb_func
FLASH_IRQHandler:
    mov r0, #6
    b __default_exc


.thumb_func
__default_exc:
    cpsid i                     @ Disable global interrupts

    ldr r1, BFAR                @ for debug, to see what happens
    ldr r1, [r1]
    ldr r1, CFSR
    ldr r1, [r1]
    ldr r1, HFSR
    ldr r1, [r1]
    ldr r1, DFSR
    ldr r1, [r1]
    ldr r1, AFSR
    ldr r1, [r1]
    ldr r1, SHCSR
    ldr r1, [r1]
    ldr r1, MMFAR
    ldr r1, [r1]

    ldr r2, NVIC_CCR            @ Enable returning to thread mode even if there are
    mov r1 ,#1                  @ pending exceptions. See flag NONEBASETHRDENA - http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0337e/Cihcbadd.html
    str r1, [r2]

    tst lr, #4
    ite eq
    mrseq r1, msp
    mrsne r1, psp 
 
    ldr r3, [r1, #24]           @ PC of exception - if access to wrong address
    ldr r2, [r1, #20]           @ LR of exception - if call to wrong address

    str r0, [r1]                @ exception number - to R0
    str r3, [r1, #4]            @ PC of exception  - to R1
    str r2, [r1, #8]            @ LR of exception  - to R2
    str lr, [r1, #12]           @ exception code   - to R3

    mov r3, #0
    ldr r2, MPU_CTRL            @ disable MPU
    str r3, [r2]

    ldr r2, CPSR_MASK           @ Set CPSR to default
    str r2, [r1,#28]

    ldr r2, TARGET_PC           @ Set target pc
    str r2, [r1,#24]

    DSB
    ISB
    
    bx lr                       @ Exception exit



.thumb_func
__do_context_switch: @ we already in interrupt so all interrupts with higher priority will use MSP - so dont need to disable interrupts

    MRS       R0, PSP           @  PSP is process stack pointer
    TST       LR, #0x10         @  exc_return[4]=0? (it means that current process
    IT        EQ                @  has active floating point context)
    VSTMDBEQ  R0!, {S16-S31}    @  if so - save it.
    STMDB     R0!, {R4-R11, LR} @  save remaining regs r4-11 and LR (EXC_RETURN) on process stack

        @ At this point, entire context of process has been saved
    LDR     R2, px_running      @ address of s_running
    LDR     R1, [R2]            @ value of s_running - address of old task_t
    STR     R0, [R1]            @ store stack pointer
    
    LDR     R3, px_nextTask     @ address of next_task
    LDR     R1, [R3]            @ value of next_task - address of new task_t
    STR     R1, [R2]            @ save to s_running
    LDR     R0, [R1]            @ R0 is new process SP
        @load context of new process
    LDMIA     R0!, {R4-R11, LR} @  Restore r4-11 and LR from new process stack
    TST       LR, #0x10         @  exc_return[4]=0? (it means that new process
    IT        EQ                @  has active floating point context)
    VLDMIAEQ  R0!, {S16-S31}    @  if so - restore it.
    MSR       PSP, R0           @  Load PSP with new process SP
    BX        LR                @  Return to saved exc_return. Exception return will restore remaining context                              

.align 4
CPSR_MASK:     .word 0x61000000
EXC_RETURN:    .word 0xFFFFFFF9
EXC_RETURN_PSP: .word 0xFFFFFFFD
TARGET_PC:     .word __error
NVIC_CCR:      .word 0xE000ED14    @ NVIC configuration control register
SYSTICK_CSR:   .word 0xE000E010    @ Systick control register
MPU_CTRL:      .word 0xE000ED94    @ MPU Control register

BFAR:          .word 0xE000ED38
CFSR:          .word 0xE000ED28
HFSR:          .word 0xE000ED2C
DFSR:          .word 0xE000ED30
AFSR:          .word 0xE000ED3C
SHCSR:         .word 0xE000ED24
MMFAR:         .word 0xE000ED34   @ MemManage Fault Address register

px_running:     .word s_running    
px_nextTask:    .word next_task
