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
 * @file    ARMCMx/compilers/GCC/crt0.S
 * @brief   Generic ARM startup file.
 *
 * @addtogroup ARM_GCC_STARTUP
 * @{
 */

#if !defined(__DOXYGEN__)

                .set    MODE_USR, 0x10
                .set    MODE_FIQ, 0x11
                .set    MODE_IRQ, 0x12
                .set    MODE_SVC, 0x13
                .set    MODE_ABT, 0x17
                .set    MODE_UND, 0x1B
                .set    MODE_SYS, 0x1F

                .set    I_BIT, 0x80
                .set    F_BIT, 0x40

                .text
                .code   32
                .balign 4

/*
 * Reset handler.
 */
                .global Reset_Handler
Reset_Handler:
                /*
                 * Stack pointers initialization.
                 */
                ldr     r0, =__stacks_end__
                /* Undefined */
                msr     CPSR_c, #MODE_UND | I_BIT | F_BIT
                mov     sp, r0
                ldr     r1, =__und_stack_size__
                sub     r0, r0, r1
                /* Abort */
                msr     CPSR_c, #MODE_ABT | I_BIT | F_BIT
                mov     sp, r0
                ldr     r1, =__abt_stack_size__
                sub     r0, r0, r1
                /* FIQ */
                msr     CPSR_c, #MODE_FIQ | I_BIT | F_BIT
                mov     sp, r0
                ldr     r1, =__fiq_stack_size__
                sub     r0, r0, r1
                /* IRQ */
                msr     CPSR_c, #MODE_IRQ | I_BIT | F_BIT
                mov     sp, r0
                ldr     r1, =__irq_stack_size__
                sub     r0, r0, r1
                /* Supervisor */
                msr     CPSR_c, #MODE_SVC | I_BIT | F_BIT
                mov     sp, r0
                ldr     r1, =__svc_stack_size__
                sub     r0, r0, r1
                /* System */
                msr     CPSR_c, #MODE_SYS | I_BIT | F_BIT
                mov     sp, r0
//                ldr     r1, =__sys_stack_size__
//                sub     r0, r0, r1
                /* Core initialization.*/
                bl      __cpu_init
                /*
                 * Early initialization.
                 */
                bl      __early_init

                /*
                 * Data initialization.
                 * NOTE: It assumes that the DATA size is a multiple of 4.
                 */
                ldr     r1, =__textdata_base__
                ldr     r2, =__data_base__
                ldr     r3, =__data_end__
.Ldataloop:
                cmp     r2, r3
                ldrlo   r0, [r1], #4
                strlo   r0, [r2], #4
                blo     .Ldataloop
                /*
                 * BSS initialization.
                 * NOTE: It assumes that the BSS size is a multiple of 4.
                 */
                mov     r0, #0
                ldr     r1, =__bss_base__
                ldr     r2, =__bss_end__
.Lbssloop:
                cmp     r1, r2
                strlo   r0, [r1], #4
                blo     .Lbssloop
                /*
                 * Late initialization.
                 */
                bl      __late_init

                /*
                 * Main program invocation.
                 */
                bl      main
                b       __default_exit

#endif /* !defined(__DOXYGEN__) */

/** @} */
