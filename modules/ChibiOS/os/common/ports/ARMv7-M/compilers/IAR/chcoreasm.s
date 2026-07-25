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
 * @file    ARMv7-M/compilers/IAR/chcoreasm.s
 * @brief   ARMv7-M port low level code.
 *
 * @addtogroup ARMV7M_IAR_CORE
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

                MODULE  ?chcoreasm_v7m

                AAPCS INTERWORK, VFP_COMPATIBLE
                PRESERVE8

SCB_ICSR        SET 0xE000ED04
ICSR_PENDSVSET  SET 0x10000000

                SECTION .text:CODE:NOROOT(2)

                EXTERN  chThdExit
                EXTERN  chSchDoPreemption
#if CH_DBG_ENABLE_STACK_CHECK && PORT_ENABLE_GUARD_PAGES
                EXTERN  __port_set_region
#endif
#if CH_DBG_STATISTICS
                EXTERN  __stats_start_measure_crit_thd
                EXTERN  __stats_stop_measure_crit_thd
#endif
#if CH_DBG_SYSTEM_STATE_CHECK
                EXTERN  __dbg_check_unlock
                EXTERN  __dbg_check_lock
#endif

                THUMB

/*
 * Performs a context switch between two threads.
 */
                PUBLIC  __port_switch
__port_switch:
                push    {r4, r5, r6, r7, r8, r9, r10, r11, lr}
#if CORTEX_USE_FPU
                vpush   {s16-s31}
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

#if CORTEX_USE_FPU
                vpop    {s16-s31}
#endif
                pop     {r4, r5, r6, r7, r8, r9, r10, r11, pc}

/*
 * Start a thread by invoking its work function.
 * If the work function returns @p chThdExit() is automatically invoked.
 */
                PUBLIC  __port_thread_start
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

/*
 * Post-IRQ switch code.
 * Exception handlers return here for context switching.
 */
                PUBLIC  __port_switch_from_isr
                PUBLIC  __port_exit_from_isr
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
__port_exit_from_isr:
#if CORTEX_SIMPLIFIED_PRIORITY
                mov     r3, #LWRD SCB_ICSR
                movt    r3, #HWRD SCB_ICSR
                mov	r2, #ICSR_PENDSVSET
                str	r2, [r3]
                cpsie   i
#else
                svc     #0
#endif
.L3:            b       .L3

                END

#endif /* !defined(__DOXYGEN__) */

/** @} */
