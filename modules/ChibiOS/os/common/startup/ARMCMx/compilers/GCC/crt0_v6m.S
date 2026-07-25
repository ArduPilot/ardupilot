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
 * @file    crt0_v6m.S
 * @brief   Generic ARMv6-M (Cortex-M0/M1) startup file for ChibiOS.
 *
 * @addtogroup ARMCMx_GCC_STARTUP_V6M
 * @{
 */

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#if !defined(FALSE) || defined(__DOXYGEN__)
#define FALSE                               0
#endif

#if !defined(TRUE) || defined(__DOXYGEN__)
#define TRUE                                1
#endif

#define CONTROL_MODE_PRIVILEGED             0
#define CONTROL_MODE_UNPRIVILEGED           1
#define CONTROL_USE_MSP                     0
#define CONTROL_USE_PSP                     2

#define SCB_VTOR                            0xE000ED08

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Enforces initialization of MSP.
 * @note    This is required if the boot process is not reliable for whatever
 *          reason (bad ROMs, bad bootloaders, bad debuggers=.
 */
#if !defined(CRT0_FORCE_MSP_INIT) || defined(__DOXYGEN__)
#define CRT0_FORCE_MSP_INIT                 TRUE
#endif

/**
 * @brief   VTOR special register initialization.
 * @details VTOR is initialized to point to the vectors table.
 * @note    This option can only be enabled on Cortex-M0+ cores.
 */
#if !defined(CRT0_VTOR_INIT) || defined(__DOXYGEN__)
#define CRT0_VTOR_INIT                      FALSE
#endif

/**
 * @brief   Control special register initialization value.
 * @details The system is setup to run in privileged mode using the PSP
 *          stack (dual stack mode).
 */
#if !defined(CRT0_CONTROL_INIT) || defined(__DOXYGEN__)
#define CRT0_CONTROL_INIT                   (CONTROL_USE_PSP |              \
                                             CONTROL_MODE_PRIVILEGED)
#endif

/**
 * @brief   Core initialization switch.
 */
#if !defined(CRT0_INIT_CORE) || defined(__DOXYGEN__)
#define CRT0_INIT_CORE                      TRUE
#endif

/**
 * @brief   Stack segments initialization switch.
 */
#if !defined(CRT0_STACKS_FILL_PATTERN) || defined(__DOXYGEN__)
#define CRT0_STACKS_FILL_PATTERN            0x55555555
#endif

/**
 * @brief   Stack segments initialization switch.
 */
#if !defined(CRT0_INIT_STACKS) || defined(__DOXYGEN__)
#define CRT0_INIT_STACKS                    TRUE
#endif

/**
 * @brief   Vectors table initialization.
 * @details Vectors are copied in RAM on startup.
 */
#if !defined(CRT0_INIT_VECTORS) || defined(__DOXYGEN__)
#define CRT0_INIT_VECTORS                   FALSE
#endif

/**
 * @brief   DATA segment initialization switch.
 */
#if !defined(CRT0_INIT_DATA) || defined(__DOXYGEN__)
#define CRT0_INIT_DATA                      TRUE
#endif

/**
 * @brief   BSS segment initialization switch.
 */
#if !defined(CRT0_INIT_BSS) || defined(__DOXYGEN__)
#define CRT0_INIT_BSS                       TRUE
#endif

/**
 * @brief   RAM areas initialization switch.
 */
#if !defined(CRT0_INIT_RAM_AREAS) || defined(__DOXYGEN__)
#define CRT0_INIT_RAM_AREAS                 TRUE
#endif

/**
 * @brief   Constructors invocation switch.
 */
#if !defined(CRT0_CALL_CONSTRUCTORS) || defined(__DOXYGEN__)
#define CRT0_CALL_CONSTRUCTORS              TRUE
#endif

/**
 * @brief   Destructors invocation switch.
 */
#if !defined(CRT0_CALL_DESTRUCTORS) || defined(__DOXYGEN__)
#define CRT0_CALL_DESTRUCTORS               TRUE
#endif

/**
 * @brief   Number of extra cores.
 */
#if !defined(CRT0_EXTRA_CORES_NUMBER) || defined(__DOXYGEN__)
#define CRT0_EXTRA_CORES_NUMBER             0
#endif

/*===========================================================================*/
/* Code section.                                                             */
/*===========================================================================*/

#if !defined(__DOXYGEN__)

                .cpu    cortex-m0
                .fpu    softvfp
                .syntax unified
                .thumb
                .text

/*
 * CRT0 entry point.
 */
                .align  2
                .thumb_func
                .global _crt0_entry
_crt0_entry:
                /* Interrupts are globally masked initially.*/
                cpsid   i

#if CRT0_FORCE_MSP_INIT == TRUE
                /* MSP stack pointers initialization.*/
                ldr     r0, =__main_stack_end__
                msr     MSP, r0
#endif

                /* PSP stack pointers initialization.*/
                ldr     r0, =__process_stack_end__
                msr     PSP, r0

                /* CPU mode initialization as configured.*/
                movs    r0, #CRT0_CONTROL_INIT
                msr     CONTROL, r0
                isb

#if CRT0_VTOR_INIT == TRUE
                ldr     r0, =_vectors
                ldr     r1, =SCB_VTOR
                str     r0, [r1]
#endif

#if CRT0_INIT_CORE == TRUE
                /* Core initialization.*/
                bl      __cpu_init
#endif

                /* Early initialization..*/
                bl      __early_init

#if CRT0_INIT_STACKS == TRUE
                ldr     r0, =CRT0_STACKS_FILL_PATTERN
                /* Main Stack initialization. Note, it assumes that the
                   stack size is a multiple of 4 so the linker file must
                   ensure this.*/
                ldr     r1, =__main_stack_base__
                ldr     r2, =__main_stack_end__
.Lmsloop:
                cmp     r1, r2
                bge     .Lendmsloop
                str     r0, [r1]
                adds    r1, #4
                b       .Lmsloop
.Lendmsloop:
                /* Process Stack initialization. Note, it assumes that the
                   stack size is a multiple of 4 so the linker file must
                   ensure this.*/
                ldr     r1, =__process_stack_base__
                ldr     r2, =__process_stack_end__
.Lpsloop:
                cmp     r1, r2
                bge     .Lendpsloop
                str     r0, [r1]
                adds    r1, #4
                b       .Lpsloop
.Lendpsloop:
#endif

#if CRT0_INIT_VECTORS == TRUE
                /* Vectors initialization. Note, it assumes that the vectors
                   size is a multiple of 4 so the linker file must ensure
                   this.*/
                ldr     r1, =__textvectors_base__
                ldr     r2, =__vectors_base__
                ldr     r3, =__vectors_end__
                mov     r4, r2
.Lvloop:
                cmp     r2, r3
                bge     .Lendvloop
                ldr     r0, [r1]
                str     r0, [r2]
                adds    r1, #4
                adds    r2, #4
                b       .Lvloop
.Lendvloop:

                /* VTOR now pointing to the RAM table.*/
                ldr     r1, =SCB_VTOR
                str     r4, [r1]
#endif

#if CRT0_INIT_DATA == TRUE
                /* Data initialization. Note, it assumes that the DATA
                   size is a multiple of 4 so the linker file must ensure
                   this.*/
                ldr     r1, =__textdata_base__
                ldr     r2, =__data_base__
                ldr     r3, =__data_end__
.Ldloop:
                cmp     r2, r3
                bge     .Lenddloop
                ldr     r0, [r1]
                str     r0, [r2]
                adds    r1, #4
                adds    r2, #4
                b       .Ldloop
.Lenddloop:
#endif

#if CRT0_INIT_BSS == TRUE
                /* BSS initialization. Note, it assumes that the DATA
                   size is a multiple of 4 so the linker file must ensure
                   this.*/
                movs    r0, #0
                ldr     r1, =__bss_base__
                ldr     r2, =__bss_end__
.Lbloop:
                cmp     r1, r2
                bge     .Lendbloop
                str     r0, [r1]
                adds    r1, #4
                b       .Lbloop
.Lendbloop:
#endif

#if CRT0_INIT_RAM_AREAS == TRUE
                /* RAM areas initialization.*/
                bl      __init_ram_areas
#endif

                /* Late initialization..*/
                bl      __late_init

#if CRT0_CALL_CONSTRUCTORS == TRUE
                /* Constructors invocation.*/
                ldr     r4, =__init_array_base__
                ldr     r5, =__init_array_end__
.Linitloop:
                cmp     r4, r5
                bge     .Lendinitloop
                ldr     r1, [r4]
                blx     r1
                adds    r4, #4
                b       .Linitloop
.Lendinitloop:
#endif

                /* Main program invocation, r0 contains the returned value.*/
                bl      main

#if CRT0_CALL_DESTRUCTORS == TRUE
                /* Destructors invocation.*/
                ldr     r4, =__fini_array_base__
                ldr     r5, =__fini_array_end__
.Lfiniloop:
                cmp     r4, r5
                bge     .Lendfiniloop
                ldr     r1, [r4]
                blx     r1
                adds    r4, #4
                b       .Lfiniloop
.Lendfiniloop:
#endif

                /* Branching to the defined exit handler.*/
                ldr     r1, =__default_exit
                bx      r1

#endif

                .macro  CRT0_EXTRA_CORE_ENTRY core
                .align  2
                .thumb_func
                .global _crt0_c\core\()_entry
_crt0_c\core\()_entry:
                /* Interrupts are globally masked initially.*/
                cpsid   i

#if CRT0_FORCE_MSP_INIT == TRUE
                /* MSP stack pointers initialization.*/
                ldr     r0, =__c\core\()_main_stack_end__
                msr     MSP, r0
#endif

                /* PSP stack pointers initialization.*/
                ldr     r0, =__c\core\()_process_stack_end__
                msr     PSP, r0

                /* CPU mode initialization as configured.*/
                movs    r0, #CRT0_CONTROL_INIT
                msr     CONTROL, r0
                isb

#if CRT0_VTOR_INIT == TRUE
                ldr     r0, =_vectors
                ldr     r1, =SCB_VTOR
                str     r0, [r1]
#endif

#if CRT0_INIT_CORE == TRUE
                /* Core initialization.*/
                bl      __c\core\()_cpu_init
#endif

                /* Early initialization..*/
                bl      __c\core\()_early_init

#if CRT0_INIT_STACKS == TRUE
                ldr     r0, =CRT0_STACKS_FILL_PATTERN
                /* Main Stack initialization. Note, it assumes that the
                   stack size is a multiple of 4 so the linker file must
                   ensure this.*/
                ldr     r1, =__c\core\()_main_stack_base__
                ldr     r2, =__c\core\()_main_stack_end__
.Lc\core\()msloop\@:
                cmp     r1, r2
                bge     .Lc\core\()endmsloop\@
                str     r0, [r1]
                adds    r1, #4
                b       .Lc\core\()msloop\@
.Lc\core\()endmsloop\@:
                /* Process Stack initialization. Note, it assumes that the
                   stack size is a multiple of 4 so the linker file must
                   ensure this.*/
                ldr     r1, =__c\core\()_process_stack_base__
                ldr     r2, =__c\core\()_process_stack_end__
.Lc\core\()psloop\@:
                cmp     r1, r2
                bge     .Lc\core\()endpsloop\@
                str     r0, [r1]
                adds    r1, #4
                b       .Lc\core\()psloop\@
.Lc\core\()endpsloop\@:
#endif

                /* Late initialization..*/
                bl      __c\core\()_late_init

                /* Main program invocation, r0 contains the returned value.*/
                bl      c\core\()_main

                /* Branching to the defined exit handler.*/
                ldr     r1, =__c\core\()_default_exit
                bx      r1
                .endm

#if CRT0_EXTRA_CORES_NUMBER > 0
                CRT0_EXTRA_CORE_ENTRY 1
#endif

#if CRT0_EXTRA_CORES_NUMBER > 1
                CRT0_EXTRA_CORE_ENTRY 2
#endif

#if CRT0_EXTRA_CORES_NUMBER > 2
                CRT0_EXTRA_CORE_ENTRY 3
#endif

#if CRT0_EXTRA_CORES_NUMBER > 3
#error "unsupported number of extra cores"
#endif

/** @} */
