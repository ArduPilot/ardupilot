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
 * @file    ARMC6/cstartup.s
 * @brief   Generic KEIL ARMv7-M startup file.
 *
 * @addtogroup ARMV7M_ARMC6_STARTUP
 * @{
 */

#if !defined(CORTEX_USE_FPU)
#error "CORTEX_USE_FPU not defined"
#endif

#if !defined(__DOXYGEN__)

;/* <<< Use Configuration Wizard in Context Menu >>> */

;// <h> Main Stack Configuration (IRQ Stack)
;//   <o> Main Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;// </h>
                .set    main_stack_size, 0x00000400

;// <h> Process Stack Configuration
;//   <o> Process Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;// </h>
                .set    proc_stack_size, 0x00000400

;// <h> C-runtime heap size
;//   <o> C-runtime heap size (in Bytes) <0x0-0xFFFFFFFF:8>
;// </h>
                .set    heap_size, 0x00000400

                .section    ".0mstack", "aw"
                .balign     8
                .global     __main_stack_base__, __main_stack_end__
__main_stack_base__:
                .org        __main_stack_base__ + main_stack_size
__main_stack_end__:

                .section    ".1pstack", "aw"
                .balign     8
                .global     __process_stack_base__, __process_stack_end__
                .global     __main_thread_stack_base__, __main_thread_stack_end__
__process_stack_base__:
__main_thread_stack_base__:
                .org        __process_stack_base__ + proc_stack_size
__process_stack_end__:
__main_thread_stack_end__:

                .section    ".2heap", "aw"
                .balign     8
                .global     __c_heap_base__, __c_heap_end__
__c_heap_base__:
                .org        __c_heap_base__ + heap_size
__c_heap_end__:

                .set        CONTROL_MODE_PRIVILEGED, 0
                .set        CONTROL_MODE_UNPRIVILEGED, 1
                .set        CONTROL_USE_MSP, 0
                .set        CONTROL_USE_PSP, 2

                .thumb
                .text
                .eabi_attribute Tag_ABI_align_preserved, 1

                .global     Reset_Handler
                .type       Reset_Handler, "function"    
Reset_Handler:
                cpsid       i
                ldr         r0, =__main_stack_end__
                msr         msp, r0
                ldr         r0, =__process_stack_end__
                msr         psp, r0
                bl          __early_init
                movs        r0, #(CONTROL_MODE_PRIVILEGED | CONTROL_USE_PSP)
                msr         control, r0
                isb

#if CORTEX_USE_FPU
                ldr         r0, =0xE000ED88
                ldr         r1, [r0]
                orr         r1, r1, #(0xF << 20)
                str         r1, [r0]
#endif

                ldr         r0, =__main
                bx          r0

                .weak       __early_init
                .type       __early_init, "function"    
__early_init:
                bx          lr

                .global     __use_two_region_memory
                .global     __user_initial_stackheap
__user_initial_stackheap:
                ldr         r0, =__c_heap_base__
                ldr         r2, =__c_heap_end__
                bx          lr

                .end

#endif /* !defined(__DOXYGEN__) */

/**< @} */
