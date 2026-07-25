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
 * @file    ivor.s
 * @brief   Kernel ISRs.
 *
 * @addtogroup PPC_CORE
 * @{
 */

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#if !defined(FALSE) || defined(__DOXYGEN__)
#define FALSE   0
#endif

#if !defined(TRUE) || defined(__DOXYGEN__)
#define TRUE    1
#endif

/*===========================================================================*/
/* Code section.                                                             */
/*===========================================================================*/

/*
 * Imports the PPC configuration headers.
 */
#define _FROM_ASM_
#include "chlicense.h"
#include "chconf.h"
#include "chcore.h"

#if !defined(__DOXYGEN__)

        .extern      _stats_start_measure_crit_thd
        .extern      _stats_stop_measure_crit_thd
        .extern      _dbg_check_lock
        .extern      _dbg_check_unlock
        .extern      chSchIsPreemptionRequired
        .extern      chSchDoReschedule
        .extern      chSysTimerHandlerI

        .section    .handlers, text_vle

#if PPC_USE_VLE == TRUE

#if PPC_SUPPORTS_DECREMENTER
        /*
         * _IVOR10 handler (Book-E decrementer).
         */
        .align      16
        .globl      _IVOR10
        .type       _IVOR10, @function
_IVOR10:
        /* Saving the external context (port_extctx structure).*/
        e_stwu      r1, -80(r1)
        e_stmvsrrw  8(r1)                  /* Saves PC, MSR.               */
        e_stmvsprw  16(r1)                 /* Saves CR, LR, CTR, XER.      */
        e_stmvgprw  32(r1)                 /* Saves GPR0, GPR3...GPR12.    */

        /* Increasing the SPGR0 register.*/
        mfspr       r0, 272
        se_addi     r0, 1
        mtspr       272, r0

        /* Reset DIE bit in TSR register.*/
        e_lis       r3, 0x0800             /* DIS bit mask.                */
        mtspr       336, r3                /* TSR register.                */

        /* Restoring pre-IRQ MSR register value.*/
        mfSRR1      r0
#if !PPC_USE_IRQ_PREEMPTION
        /* No preemption, keeping EE disabled.*/
        se_bclri    r0, 16                 /* EE = bit 16.                 */
#endif
        mtMSR       r0

#if CH_DBG_SYSTEM_STATE_CHECK
        bl          _dbg_check_enter_isr
        bl          _dbg_check_lock_from_isr
#endif
        /* System tick handler invocation.*/
        e_bl        chSysTimerHandlerI
#if CH_DBG_SYSTEM_STATE_CHECK
        bl          _dbg_check_unlock_from_isr
        bl          _dbg_check_leave_isr
#endif

#if PPC_USE_IRQ_PREEMPTION
        /* Prevents preemption again.*/
        wrteei      0
#endif

        /* Jumps to the common IVOR epilogue code.*/
        se_b        _ivor_exit
#endif /* PPC_SUPPORTS_DECREMENTER */

        /*
         * _IVOR4 handler (Book-E external interrupt).
         */
        .align      16
        .globl      _IVOR4
        .type       _IVOR4, @function
_IVOR4:
        /* Saving the external context (port_extctx structure).*/
        e_stwu      r1, -80(r1)
        e_stmvsrrw  8(r1)                  /* Saves PC, MSR.               */
        e_stmvsprw  16(r1)                 /* Saves CR, LR, CTR, XER.      */
        e_stmvgprw  32(r1)                 /* Saves GPR0, GPR3...GPR12.    */

        /* Increasing the SPGR0 register.*/
        mfspr       r0, 272
        se_addi     r0, 1
        mtspr       272, r0

        /* Software vector address from the INTC register.*/
        e_lis       r3, INTC_IACKR_ADDR@h
        e_or2i      r3, INTC_IACKR_ADDR@l  /* IACKR register address.      */
        se_lwz      r3, 0(r3)              /* IACKR register value.        */
        se_lwz      r3, 0(r3)
        mtCTR       r3                     /* Software handler address.    */

        /* Restoring pre-IRQ MSR register value.*/
        mfSRR1      r0
#if !PPC_USE_IRQ_PREEMPTION
        /* No preemption, keeping EE disabled.*/
        se_bclri    r0, 16                 /* EE = bit 16.                 */
#endif
        mtMSR       r0

        /* Exectes the software handler.*/
        se_bctrl

#if PPC_USE_IRQ_PREEMPTION
        /* Prevents preemption again.*/
        wrteei      0
#endif

        /* Informs the INTC that the interrupt has been served.*/
        mbar        0
        e_lis       r3, INTC_EOIR_ADDR@h
        e_or2i      r3, INTC_EOIR_ADDR@l
        se_stw      r3, 0(r3)             /* Writing any value should do. */

        /* Common IVOR epilogue code, context restore.*/
        .globl      _ivor_exit
_ivor_exit:
        /* Decreasing the SPGR0 register.*/
        mfspr       r0, 272
        se_subi     r0, 1
        mtspr       272, r0

#if CH_DBG_STATISTICS
        e_bl        __stats_start_measure_crit_thd
#endif
#if CH_DBG_SYSTEM_STATE_CHECK
        e_bl        __dbg_check_lock
#endif
        e_bl        chSchIsPreemptionRequired
        e_cmpli     cr0, r3, 0
        se_beq      .noresch
        e_bl        chSchDoReschedule
.noresch:
#if CH_DBG_SYSTEM_STATE_CHECK
        e_bl        __dbg_check_unlock
#endif
#if CH_DBG_STATISTICS
        e_bl        __stats_stop_measure_crit_thd
#endif

        /* Restoring the external context.*/
        e_lmvgprw   32(r1)                 /* Restores GPR0, GPR3...GPR12. */
        e_lmvsprw   16(r1)                 /* Restores CR, LR, CTR, XER.   */
        e_lmvsrrw   8(r1)                  /* Restores PC, MSR.            */
        e_addi      r1, r1, 80             /* Back to the previous frame.  */
        se_rfi

#else /* PPC_USE_VLE == FALSE */

#error "non-VLE mode not yet implemented"

#endif /* PPC_USE_VLE == FALSE */

#endif /* !defined(__DOXYGEN__) */

/** @} */
