/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

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

/**
 * @file    ARMCMx/compilers/IAR/cstartup.s
 * @brief   Generic IAR Cortex-Mx startup file.
 *
 * @addtogroup ARMCMx_IAR_STARTUP
 * @{
 */

#if !defined(__DOXYGEN__) 

#define SCB_VTOR                            0xE000ED08

 /**  
  * @brief   VTOR special register initialization.  
  * @details VTOR is initialized to point to the vectors table.     
  * @note    IAR assembler #if directive conditions do not work like C/C++ conditions.
  * @details Set to 0 to disable the function, 1 to enable
  */  
#ifndef CRT0_VTOR_INIT 
#define CRT0_VTOR_INIT                      1
#endif 
/**
 * @brief   Stack segments initialization value.
 */
#ifndef CRT0_STACKS_FILL_PATTERN 
#define CRT0_STACKS_FILL_PATTERN            0x55555555
#endif

/**
 * @brief   Stack segments initialization switch.
  * @details Set to 0 to disable the function, 1 to enable
 */
#ifndef CRT0_INIT_STACKS
#define CRT0_INIT_STACKS                    1
#endif

/**
 * @brief   Heap segment initialization value.
 */
#ifndef CRT0_HEAP_FILL_PATTERN 
#define CRT0_HEAP_FILL_PATTERN              0xCCCCCCCC
#endif

/**
 * @brief   Heap segment initialization switch.
 * @details Set to 0 to disable the function, 1 to enable
 */
#ifndef CRT0_INIT_HEAP
#define CRT0_INIT_HEAP                      1
#endif


        MODULE  ?cstartup

CONTROL_MODE_PRIVILEGED SET 0
CONTROL_MODE_UNPRIVILEGED SET 1
CONTROL_USE_MSP SET 0
CONTROL_USE_PSP SET 2

        AAPCS INTERWORK, VFP_COMPATIBLE, ROPI
        PRESERVE8
        
        SECTION HEAP:DATA:NOROOT(3)
        PUBLIC  __heap_base__
__heap_base__:             /* Note: heap section defines sysheap base */

        SECTION SYSHEAP:DATA:NOROOT(3)
        PUBLIC  __heap_end__
__heap_end__:              /* Note: sysheap section defines sysheap end */

        PUBLIC  __iar_program_start
        EXTWEAK __iar_init_core
        EXTWEAK __iar_init_vfp
        EXTERN  __cmain
        EXTERN  __vector_table
        EXTERN  __main_stack_base__
        EXTERN  __main_stack_end__
        EXTERN  __process_stack_base__
        EXTERN  __process_stack_end__        

        SECTION IRQSTACK:DATA:NOROOT(3)
        SECTION CSTACK:DATA:NOROOT(3)
        SECTION .text:CODE:REORDER(2)
        THUMB
        
__iar_program_start:
        cpsid   i
        ldr     r0, =SFE(IRQSTACK)
        msr     MSP, r0
        ldr     r0, =SFE(CSTACK)
        msr     PSP, r0
        movs    r0, #CONTROL_MODE_PRIVILEGED | CONTROL_USE_PSP
        msr     CONTROL, r0
        isb
        
#if (CRT0_VTOR_INIT)
        ldr     r0, =__vector_table  
        movw    r1, #SCB_VTOR & 0xFFFF  
        movt    r1, #SCB_VTOR >> 16  
        str     r0, [r1]  
#endif

#if (CRT0_INIT_STACKS)
        ldr     r0, =CRT0_STACKS_FILL_PATTERN
        /* Main Stack initialization. Note, it assumes that the stack size
           is a multiple of 4 so the linker file must ensure this.*/
        ldr     r1, =__main_stack_base__
        ldr     r2, =__main_stack_end__
msloop:
        cmp     r1, r2
        itt     lo
        strlo   r0, [r1], #4
        blo     msloop

        /* Process Stack initialization. Note, it assumes that the stack size
           is a multiple of 4 so the linker file must ensure this.*/
        ldr     r1, =__process_stack_base__
        ldr     r2, =__process_stack_end__
psloop:
        cmp     r1, r2
        itt     lo
        strlo   r0, [r1], #4
        blo     psloop
#endif

#if (CRT0_INIT_HEAP)
        ldr     r0, =CRT0_HEAP_FILL_PATTERN
        /* Sys Heap initialization. Note, it assumes that the heap size
           is a multiple of 4 so the linker file must ensure this.*/
        ldr     r1, =__heap_base__
        ldr     r2, =__heap_end__
hloop:
        cmp     r1, r2
        itt     lo
        strlo   r0, [r1], #4
        blo     hloop
#endif

        bl      __early_init
        bl      __iar_init_core
        bl      __iar_init_vfp
        b       __cmain

        SECTION .text:CODE:NOROOT:REORDER(2)
        PUBWEAK __early_init
__early_init:
        bx      lr

        END

#endif /* !defined(__DOXYGEN__) */

/**< @} */
