/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    ARMv7-M/compilers/GCC/chcoreasm.S
 * @brief   ARMv7-M port low level code.
 *
 * @addtogroup ARMV7M_GCC_CORE
 * @{
 */

#if !defined(FALSE) || defined(__DOXYGEN__)
#define FALSE   0
#endif

#if !defined(TRUE) || defined(__DOXYGEN__)
#define TRUE    1
#endif

#define _FROM_ASM_
#include "chlicense.h"
#include "chconf.h"
#include "chcore.h"

#if !defined(__DOXYGEN__)

/*
 * RTOS-specific context offset.
 */
#if defined(_CHIBIOS_RT_CONF_)
#define CONTEXT_OFFSET  12

#elif defined(_CHIBIOS_NIL_CONF_)
#define CONTEXT_OFFSET  0

#else
#error "invalid chconf.h"
#endif

/* MPU-related constants.*/
#define MPU_RBAR        0xE000ED9C

/* Other constants.*/
#define SCB_ICSR        0xE000ED04
#define ICSR_PENDSVSET  0x10000000

                .syntax unified
                .cpu    cortex-m4
#if CORTEX_USE_FPU
                .fpu    fpv4-sp-d16
#else
                .fpu    softvfp
#endif

                .thumb
                .text

/*--------------------------------------------------------------------------*
 * Performs a context switch between two threads.
 *--------------------------------------------------------------------------*/
                .thumb_func
                .globl  __port_switch
__port_switch:
                push    {r4, r5, r6, r7, r8, r9, r10, r11, lr}
#if CORTEX_USE_FPU
                /* Saving FPU context.*/
                vpush   {s16-s31}
#endif

#if PORT_SWITCHED_REGIONS_NUMBER > 0
                /* Saving MPU context.*/
                ldr     r2, =MPU_RBAR
#if PORT_SWITCHED_REGIONS_NUMBER >= 1
                mov     r3, #0
                str     r3, [r2, #-4]   /* RNR */
                ldm     r2, {r4, r5}    /* RBAR, RASR */
#endif
#if PORT_SWITCHED_REGIONS_NUMBER >= 2
                add     r3, #1
                str     r3, [r2, #-4]   /* RNR */
                ldm     r2, {r6, r7}    /* RBAR, RASR */
#endif
#if PORT_SWITCHED_REGIONS_NUMBER >= 3
                add     r3, #1
                str     r3, [r2, #-4]   /* RNR */
                ldm     r2, {r8, r9}    /* RBAR, RASR */
#endif
#if PORT_SWITCHED_REGIONS_NUMBER >= 4
                add     r3, #1
                str     r3, [r2, #-4]   /* RNR */
                ldm     r2, {r10, r11}  /* RBAR, RASR */
#endif
#if PORT_SWITCHED_REGIONS_NUMBER == 1
                push    {r4, r5}
#endif
#if PORT_SWITCHED_REGIONS_NUMBER == 2
                push    {r4, r5, r6, r7}
#endif
#if PORT_SWITCHED_REGIONS_NUMBER == 3
                push    {r4, r5, r6, r7, r8, r9}
#endif
#if PORT_SWITCHED_REGIONS_NUMBER == 4
                push    {r4, r5, r6, r7, r8, r9, r10, r11}
#endif
#endif

                str     sp, [r1, #CONTEXT_OFFSET]
#if (CORTEX_SIMPLIFIED_PRIORITY == FALSE) &&                                \
    ((CORTEX_MODEL == 3) || (CORTEX_MODEL == 4))
                /* Workaround for ARM errata 752419, only applied if
                   condition exists for it to be triggered.*/
                ldr     r3, [r0, #CONTEXT_OFFSET]
                mov     sp, r3
#else
                ldr     sp, [r0, #CONTEXT_OFFSET]
#endif

#if PORT_SWITCHED_REGIONS_NUMBER > 0
                /* Restoring MPU context.*/
#if PORT_SWITCHED_REGIONS_NUMBER == 1
                pop     {r4, r5}
#endif
#if PORT_SWITCHED_REGIONS_NUMBER == 2
                pop     {r4, r5, r6, r7}
#endif
#if PORT_SWITCHED_REGIONS_NUMBER == 3
                pop     {r4, r5, r6, r7, r8, r9}
#endif
#if PORT_SWITCHED_REGIONS_NUMBER == 4
                pop     {r4, r5, r6, r7, r8, r9, r10, r11}
#endif
#if PORT_SWITCHED_REGIONS_NUMBER >= 1
                mov     r3, #0
                str     r3, [r2, #-4]   /* RNR */
                stm     r2, {r4, r5}    /* RBAR, RASR */
#endif
#if PORT_SWITCHED_REGIONS_NUMBER >= 2
                add     r3, #1
                str     r3, [r2, #-4]   /* RNR */
                stm     r2, {r6, r7}    /* RBAR, RASR */
#endif
#if PORT_SWITCHED_REGIONS_NUMBER >= 3
                add     r3, #1
                str     r3, [r2, #-4]   /* RNR */
                stm     r2, {r8, r9}    /* RBAR, RASR */
#endif
#if PORT_SWITCHED_REGIONS_NUMBER >= 4
                add     r3, #1
                str     r3, [r2, #-4]   /* RNR */
                stm     r2, {r10, r11}  /* RBAR, RASR */
#endif
#endif

#if CORTEX_USE_FPU
                /* Restoring FPU context.*/
                vpop    {s16-s31}
#endif
                pop     {r4, r5, r6, r7, r8, r9, r10, r11, pc}

/*--------------------------------------------------------------------------*
 * Start a thread by invoking its work function.
 *
 * Threads execution starts here, the code leaves the system critical zone
 * and then jumps into the thread function passed in register R4. The
 * register R5 contains the thread parameter. The function chThdExit() is
 * called on thread function return.
 *--------------------------------------------------------------------------*/
                .thumb_func
                .globl  __port_thread_start
__port_thread_start:
#if CH_DBG_ENABLE_STACK_CHECK && PORT_ENABLE_GUARD_PAGES
                bl      __port_set_region
#endif
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      __dbg_check_unlock
#endif
#if CH_DBG_STATISTICS
                bl      __stats_stop_measure_crit_thd
#endif
#if CORTEX_SIMPLIFIED_PRIORITY
                cpsie   i
#else
                movs    r3, #0              /* CORTEX_BASEPRI_DISABLED */
                msr     BASEPRI, r3
#endif
                mov     r0, r5
                blx     r4
                movs    r0, #0              /* MSG_OK */
                bl      chThdExit
.zombies:       b       .zombies

/*--------------------------------------------------------------------------*
 * Post-IRQ switch code.
 *
 * Exception handlers return here for context switching.
 *--------------------------------------------------------------------------*/
                .thumb_func
                .globl  __port_switch_from_isr
__port_switch_from_isr:
#if CH_DBG_STATISTICS
                bl      __stats_start_measure_crit_thd
#endif
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      __dbg_check_lock
#endif
                bl      chSchDoPreemption
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      __dbg_check_unlock
#endif
#if CH_DBG_STATISTICS
                bl      __stats_stop_measure_crit_thd
#endif
                .globl  __port_exit_from_isr
__port_exit_from_isr:
#if CORTEX_SIMPLIFIED_PRIORITY
                movw    r3, #:lower16:SCB_ICSR
                movt    r3, #:upper16:SCB_ICSR
                mov     r2, ICSR_PENDSVSET
                str     r2, [r3, #0]
                cpsie   i
#else /* !CORTEX_SIMPLIFIED_PRIORITY */
                svc     #0
#endif /* !CORTEX_SIMPLIFIED_PRIORITY */
.L1:            b       .L1

#endif /* !defined(__DOXYGEN__) */

/** @} */
