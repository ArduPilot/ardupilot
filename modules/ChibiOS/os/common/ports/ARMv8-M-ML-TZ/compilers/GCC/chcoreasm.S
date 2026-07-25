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
 * @file    ARMv8-M-ML-TZ/compilers/GCC/chcoreasm.S
 * @brief   ARMv8-M MainLine architecture port low level code.
 *
 * @addtogroup ARMV8M_ML_TZ_GCC_CORE
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
#define CURRENT_OFFSET  12
#define CONTEXT_OFFSET  12

#elif defined(_CHIBIOS_NIL_CONF_)
#define CURRENT_OFFSET  0           /* nil.current */
#define CONTEXT_OFFSET  0

#else
#error "invalid chconf.h"
#endif

/* MPU-related constants.*/
#define MPU_RBAR        0xE000ED9C

/* Other constants.*/
#define SCB_ICSR        0xE000ED04
#define SCB_VTOR_NS     0xE002ED08
#define ICSR_PENDSVSET  0x10000000

                .syntax unified
                .cpu    cortex-m33
#if CORTEX_USE_FPU
                .fpu    fpv5-sp-d16
#else
                .fpu    softvfp
#endif

                .thumb
                .text

/*--------------------------------------------------------------------------*
 * Context switch macros depending on various options.
 *--------------------------------------------------------------------------*/

#if PORT_KERNEL_MODE == PORT_KERNEL_MODE_HOST

#if !CH_DBG_ENABLE_STACK_CHECK
                .macro  PORT_STORE_INTEGER_CONTEXT
                mrs     r1, BASEPRI_NS
                mrs     r3, BASEPRI
                stmia   r0!, {r1-r11,lr}
                .endm

                .macro  PORT_RESTORE_INTEGER_CONTEXT
                msr     BASEPRI_NS, r1
                ldmia   r0!, {r1-r11, lr}
                msr     PSP, r2
                msr     BASEPRI, r3
                .endm
#else /* CH_DBG_ENABLE_STACK_CHECK */
                .macro  PORT_STORE_INTEGER_CONTEXT
                mrs     r1, BASEPRI_NS
                mrs     r3, BASEPRI
                mrs     r12, PSPLIM
                stmia   r0!, {r1-r12,lr}
                .endm

                .macro  PORT_RESTORE_INTEGER_CONTEXT
                ldmia   r0!, {r1-r12, lr}
                /* Note the following is not required because this sentence
                   in the ARMv8-M architecture manual:
                       Updates to the stack pointer by the MSR instruction
                       targeting SP_NS are subject to stack limit checking.
                       Updates to the stack pointer and stack pointer limit
                       by any other MSR instruction are not subject to
                       stack limit checking.*/
//                movs    r1, #0
//                msr     PSPLIM, r1      /* Temporarily disabling stack check.*/
                msr     PSP, r2
                msr     BASEPRI_NS, r1
                msr     BASEPRI, r3
                msr     PSPLIM, r12
                .endm
#endif /* CH_DBG_ENABLE_STACK_CHECK */

#else /* PORT_KERNEL_MODE != PORT_KERNEL_MODE_HOST */

#if !CH_DBG_ENABLE_STACK_CHECK
                .macro  PORT_STORE_INTEGER_CONTEXT
                mrs     r3, BASEPRI
                stmia   r0!, {r2-r11,lr}
                .endm

                .macro  PORT_RESTORE_INTEGER_CONTEXT
                ldmia   r0!, {r2-r11, lr}
                msr     PSP, r2
                msr     BASEPRI, r3
                .endm
#else /* CH_DBG_ENABLE_STACK_CHECK */
                .macro  PORT_STORE_INTEGER_CONTEXT
                mrs     r3, BASEPRI
                mrs     r12, PSPLIM
                stmia   r0!, {r2-r12,lr}
                .endm

                .macro  PORT_RESTORE_INTEGER_CONTEXT
                ldmia   r0!, {r2-r12, lr}
                /* Note the following is not required because this sentence
                   in the ARMv8-M architecture manual:
                       Updates to the stack pointer by the MSR instruction
                       targeting SP_NS are subject to stack limit checking.
                       Updates to the stack pointer and stack pointer limit
                       by any other MSR instruction are not subject to
                       stack limit checking.*/
//                movs    r1, #0
//                msr     PSPLIM, r1      /* Temporarily disabling stack check.*/
                msr     PSP, r2
                msr     BASEPRI, r3
                msr     PSPLIM, r12
                .endm
#endif /* CH_DBG_ENABLE_STACK_CHECK */

#endif /* PORT_KERNEL_MODE != PORT_KERNEL_MODE_HOST */

#if CORTEX_USE_FPU
                .macro  PORT_STORE_FLOAT_CONTEXT
                vstmia  r0!, {s16-s31}
                .endm

                .macro  PORT_RESTORE_FLOAT_CONTEXT
                vldmia  r0!, {s16-s31}
                .endm
#else
                .macro  PORT_STORE_FLOAT_CONTEXT
                .endm

                .macro  PORT_RESTORE_FLOAT_CONTEXT
                .endm
#endif

/*--------------------------------------------------------------------------*
 * Performs a context switch between two threads using SVC.
 *--------------------------------------------------------------------------*/
                .thumb_func
                .globl  SVC_Handler
SVC_Handler:
                /* Saving callee context of thread being swapped out.*/
                mrs     r2, PSP
                ldr     r0, [r2, #4]        /* R1 on SVC entry (otp).   */
                adds    r0, #CONTEXT_OFFSET

                /* Storing integer and control context.*/
                PORT_STORE_INTEGER_CONTEXT

                /* Storing float context.*/
                PORT_STORE_FLOAT_CONTEXT

                /* Restoring calle context of thread being swapped in.*/
                ldr     r0, [r2, #0]        /* R0 on SVC entry (ntp).   */
                adds    r0, #CONTEXT_OFFSET

                /* Restoring integer and control context through R0.*/
                PORT_RESTORE_INTEGER_CONTEXT

                /* Restoring float context.*/
                PORT_RESTORE_FLOAT_CONTEXT

                bx      lr

/*--------------------------------------------------------------------------*
 * Tail preemption check using PENDSV.
 *--------------------------------------------------------------------------*/
                .thumb_func
                .globl  PendSV_Handler
PendSV_Handler:
#if PORT_KERNEL_MODE == PORT_KERNEL_MODE_HOST
                /* The following check is required in case PendSV-S preempted
                   a NS exception, in that case a reschedule cannot be
                   performed immediately.*/
                ldr     r0, =SCB_ICSR
                tst     r0, #(1 << 11)
                beq.w   port_delay_reschedule   /* Tail call on far address.*/
#endif
                /* Pointer to the current thread.*/
                ldr     r0, =ch0
//                movw    r0, #:lower16:ch
//                movt    r0, #:upper16:ch
                ldr     r0, [r0, #CURRENT_OFFSET]

                /* Saving callee context of thread being swapped out.*/
                adds    r0, #CONTEXT_OFFSET

                /* Storing integer and control context.*/
                mrs     r2, PSP
                PORT_STORE_INTEGER_CONTEXT

                /* Storing float context.*/
                PORT_STORE_FLOAT_CONTEXT

                /* Selecting the thread to be swapped in, R0 points to it.*/
                bl      port_schedule_next

                /* Restoring calle context of thread being swapped in.*/
                adds    r0, #CONTEXT_OFFSET

                /* Restoring integer and control context.*/
                PORT_RESTORE_INTEGER_CONTEXT

                /* Restoring float context.*/
                PORT_RESTORE_FLOAT_CONTEXT

                bx      lr

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
#if CH_DBG_STATISTICS
                bl      _stats_stop_measure_crit_thd
#endif
                movs    r3, #CORTEX_BASEPRI_DISABLED
                msr     BASEPRI, r3
                mov     r0, r5
                blx     r4
                movs    r0, #0              /* MSG_OK */
                bl      chThdExit
.zombies:       b       .zombies

#if PORT_KERNEL_MODE == PORT_KERNEL_MODE_HOST
/*--------------------------------------------------------------------------*
 * Safe jump in non-secure code zone. All registers are cleared.
 *--------------------------------------------------------------------------*/
                .thumb_func
                .globl  __port_ns_boot
__port_ns_boot:
                movs    r3, #0
                movs    r4, #0
                movs    r5, #0
                movs    r6, #0
                movs    r7, #0
                movs    r8, #0
                movs    r9, #0
                movs    r10, #0
                movs    r11, #0
                movs    r12, #0
                movs    lr, #0
#if CORTEX_USE_FPU
                vmov    s0, s1, r3, r3
                vmov    s2, s3, r3, r3
                vmov    s4, s5, r3, r3
                vmov    s6, s7, r3, r3
                vmov    s8, s9, r3, r3
                vmov    s10, s11, r3, r3
                vmov    s12, s13, r3, r3
                vmov    s14, s15, r3, r3
                vmov    s16, s17, r3, r3
                vmov    s18, s19, r3, r3
                vmov    s20, s21, r3, r3
                vmov    s22, s23, r3, r3
                vmov    s24, s25, r3, r3
                vmov    s26, s27, r3, r3
                vmov    s28, s29, r3, r3
                vmov    s30, s31, r3, r3
#endif
                msr     APSR_nzcvqg, r3
                msr     BASEPRI_NS, r3  /* Allowing NS-PendSV in NS state.*/
                msr     PSPLIM_NS, r3
                msr     MSPLIM_NS, r3
                ldr     r1, = SCB_VTOR_NS
                str     r0, [r1, #0]
                ldr     r1, [r0, #0]
                msr     MSP_NS, r1
                ldr     r2, [r0, #4]
                bic     r2, #1
                blxns   r2
                b       chThdExit
#endif

#endif /* !defined(__DOXYGEN__) */

/** @} */
