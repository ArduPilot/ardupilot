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
 * @file    CW/crt0.s
 * @brief   Generic PowerPC startup file for CodeWarrior.
 *
 * @addtogroup PPC_CW_CORE
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

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

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
 * @brief   Constructors invocation switch.
 */
#if !defined(CRT0_CALL_CONSTRUCTORS) || defined(__DOXYGEN__)
#define CRT0_CALL_CONSTRUCTORS              FALSE
#endif

/**
 * @brief   Destructors invocation switch.
 */
#if !defined(CRT0_CALL_DESTRUCTORS) || defined(__DOXYGEN__)
#define CRT0_CALL_DESTRUCTORS               FALSE
#endif

/*===========================================================================*/
/* Code section.                                                             */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
        .extern     __sdata2_start__
        .extern     __sdata_start__
        .extern     __bss_start__
        .extern     __bss_end__
        .extern     __irq_stack_base__
        .extern     __irq_stack_end__
        .extern     __process_stack_end__
        .extern     __process_stack_base__
        .extern     __romdata_start__
        .extern     __data_start__
        .extern     __data_end__
        .extern     __init_array_start
        .extern     __init_array_end
        .extern     __fini_array_start
        .extern     __fini_array_end
        
        .extern     main

        .section    .crt0, text_vle
        .align		16
        .globl      _boot_address
        .type       _boot_address, @function
_boot_address:
        /* Stack setup.*/
        e_lis       r1, __process_stack_end__@h
        e_or2i      r1, __process_stack_end__@l
        se_li       r0, 0
        e_stwu      r0, -8(r1)

        /* Small sections registers initialization.*/
        e_lis       r2, __sdata2_start__@h
        e_or2i      r2, __sdata2_start__@l
        e_lis       r13, __sdata_start__@h
        e_or2i      r13, __sdata_start__@l

        /* Early initialization.*/
        e_bl        __early_init

#if CRT0_INIT_STACKS == TRUE
        /* Stacks fill pattern.*/
        e_lis       r7, CRT0_STACKS_FILL_PATTERN@h
        e_or2i      r7, CRT0_STACKS_FILL_PATTERN@l

       /* IRQ Stack initialization. Note, the architecture does not use this
          stack, the size is usually zero. An OS can have special SW handling
          and require this. A 4 bytes alignment is assumed and required.*/
        e_lis       r4, __irq_stack_base__@h
        e_or2i      r4, __irq_stack_base__@l
        e_lis       r5, __irq_stack_end__@h
        e_or2i      r5, __irq_stack_end__@l
.irqsloop:
        se_cmpl     r4, r5
        se_bge      .irqsend
        se_stw      r7, 0(r4)
        se_addi     r4, 4
        se_b        .irqsloop
.irqsend:

        /* Process Stack initialization. Note, does not overwrite the already
           written EABI frame. A 4 bytes alignment is assumed and required.*/
        e_lis       r4, __process_stack_base__@h
        e_or2i      r4, __process_stack_base__@l
        e_lis       r5, (__process_stack_end__ - 8)@h
        e_or2i      r5, (__process_stack_end__ - 8)@l
.prcsloop:
        se_cmpl     r4, r5
        se_bge      .prcsend
        se_stw      r7, 0(r4)
        se_addi     r4, 4
        se_b        .prcsloop
.prcsend:
#endif

#if CRT0_INIT_BSS == TRUE
        /* BSS clearing.*/
        e_lis       r4, __bss_start__@h
        e_or2i      r4, __bss_start__@l
        e_lis       r5, __bss_end__@h
        e_or2i      r5, __bss_end__@l
        se_li       r7, 0
.bssloop:
        se_cmpl     r4, r5
        se_bge      .bssend
        se_stw      r7, 0(r4)
        se_addi     r4, 4
        se_b        .bssloop
.bssend:
#endif

#if CRT0_INIT_DATA == TRUE
        /* DATA initialization.*/
        e_lis       r4, __romdata_start__@h
        e_or2i      r4, __romdata_start__@l
        e_lis       r5, __data_start__@h
        e_or2i      r5, __data_start__@l
        e_lis       r6, __data_end__@h
        e_or2i      r6, __data_end__@l
.dataloop:
        se_cmpl     r5, r6
        se_bge      .dataend
        se_lwz      r7, 0(r4)
        se_addi     r4, 4
        se_stw      r7, 0(r5)
        se_addi     r5, 4
        se_b        .dataloop
.dataend:
#endif

        /* Late initialization.*/
        e_bl        __late_init

#if CRT0_CALL_CONSTRUCTORS == TRUE
        /* Constructors invocation.*/
        e_lis       r4, __init_array_start@h
        e_or2i      r4, __init_array_start@l
        e_lis       r5, __init_array_end@h
        e_or2i      r5, __init_array_end@l
.iniloop:
        se_cmpl     r4, r5
        se_bge      .iniend
        se_lwz      r6, 0(r4)
        se_mtctr    r6
        se_addi     r4, 4
        se_bctrl
        se_b        .iniloop
.iniend:
#endif

        /* Main program invocation.*/
        e_bl        main

#if CRT0_CALL_DESTRUCTORS == TRUE
        /* Destructors invocation.*/
        e_lis       r4, __fini_array_start@h
        e_or2i      r4, __fini_array_start@l
        e_lis       r5, __fini_array_end@h
        e_or2i      r5, __fini_array_end@l
.finiloop:
        se_cmpl     r4, r5
        se_bge      .finiend
        se_lwz      r6, 0(r4)
        se_mtctr    r6
        se_addi     r4, 4
        se_bctrl
        se_b        .finiloop
.finiend:
#endif

        /* Branching to the defined exit handler.*/
        e_b         __default_exit

#endif /* !defined(__DOXYGEN__) */

        .section    .text_vle
        .align      4

        /* Default main exit code, infinite loop.*/
        .weak       __default_exit
__default_exit:
        e_b         __default_exit

        /* Default early initialization code, none.*/
        .weak       __early_init
        se_blr

        /* Default late initialization code, none.*/
        .weak       __late_init
__late_init:
        se_blr

/** @} */
