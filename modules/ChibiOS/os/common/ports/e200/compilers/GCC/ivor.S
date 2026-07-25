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
 * @file    GCC/ivor.S
 * @brief   Kernel ISRs.
 *
 * @addtogroup PPC_CORE
 * @{
 */

#if !defined(FALSE) || defined(__DOXYGEN__)
#define FALSE   0
#endif

#if !defined(TRUE) || defined(__DOXYGEN__)
#define TRUE    1
#endif

/*
 * Imports the PPC configuration headers.
 */
#define _FROM_ASM_
#include "chlicense.h"
#include "chconf.h"
#include "chcore.h"

#if defined(__HIGHTEC__)
#define se_beq beq
#endif

#if !defined(__DOXYGEN__)

        .section    .handlers, "ax"

#if PPC_SUPPORTS_DECREMENTER
        /*
         * _IVOR10 handler (Book-E decrementer).
         */
        .align      4
        .globl      _IVOR10
        .type       _IVOR10, @function
_IVOR10:
        /* Saving the external context (port_extctx structure).*/
        e_stwu       sp, -80(sp)
#if PPC_USE_VLE && PPC_SUPPORTS_VLE_MULTI
        e_stmvsrrw  8(sp)                  /* Saves PC, MSR.               */
        e_stmvsprw  16(sp)                 /* Saves CR, LR, CTR, XER.      */
        e_stmvgprw  32(sp)                 /* Saves GPR0, GPR3...GPR12.    */
#else /* !(PPC_USE_VLE && PPC_SUPPORTS_VLE_MULTI) */
        se_stw      r0, 32(sp)             /* Saves GPR0.                  */
        mfSRR0      r0
        se_stw      r0, 8(sp)              /* Saves PC.                    */
        mfSRR1      r0
        se_stw      r0, 12(sp)             /* Saves MSR.                   */
        mfCR        r0
        se_stw      r0, 16(sp)             /* Saves CR.                    */
        mfLR        r0
        se_stw      r0, 20(sp)             /* Saves LR.                    */
        mfCTR       r0
        se_stw      r0, 24(sp)             /* Saves CTR.                   */
        mfXER       r0
        se_stw      r0, 28(sp)             /* Saves XER.                   */
        se_stw      r3, 36(sp)             /* Saves GPR3...GPR12.          */
        se_stw      r4, 40(sp)
        se_stw      r5, 44(sp)
        se_stw      r6, 48(sp)
        se_stw      r7, 52(sp)
        e_stw       r8, 56(sp)
        e_stw       r9, 60(sp)
        e_stw       r10, 64(sp)
        e_stw       r11, 68(sp)
        e_stw       r12, 72(sp)
#endif /* !(PPC_USE_VLE && PPC_SUPPORTS_VLE_MULTI) */

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
        e_bl        _dbg_check_enter_isr
        e_bl        _dbg_check_lock_from_isr
#endif
        /* System tick handler invocation.*/
        e_bl        chSysTimerHandlerI
#if CH_DBG_SYSTEM_STATE_CHECK
        e_bl        _dbg_check_unlock_from_isr
        e_bl        _dbg_check_leave_isr
#endif

#if PPC_USE_IRQ_PREEMPTION
        /* Prevents preemption again.*/
        wrteei      0
#endif

        /* Jumps to the common IVOR epilogue code.*/
        e_b         _ivor_exit
#endif /* PPC_SUPPORTS_DECREMENTER */

        /*
         * _IVOR4 handler (Book-E external interrupt).
         */
        .align      4
        .globl      _IVOR4
        .type       _IVOR4, @function
_IVOR4:
        /* Saving the external context (port_extctx structure).*/
        e_stwu      sp, -80(sp)
#if PPC_USE_VLE && PPC_SUPPORTS_VLE_MULTI
        e_stmvsrrw  8(sp)                  /* Saves PC, MSR.               */
        e_stmvsprw  16(sp)                 /* Saves CR, LR, CTR, XER.      */
        e_stmvgprw  32(sp)                 /* Saves GPR0, GPR3...GPR12.    */
#else /* !(PPC_USE_VLE && PPC_SUPPORTS_VLE_MULTI) */
        se_stw      r0, 32(sp)             /* Saves GPR0.                  */
        mfSRR0      r0
        se_stw      r0, 8(sp)              /* Saves PC.                    */
        mfSRR1      r0
        se_stw      r0, 12(sp)             /* Saves MSR.                   */
        mfCR        r0
        se_stw      r0, 16(sp)             /* Saves CR.                    */
        mfLR        r0
        se_stw      r0, 20(sp)             /* Saves LR.                    */
        mfCTR       r0
        se_stw      r0, 24(sp)             /* Saves CTR.                   */
        mfXER       r0
        se_stw      r0, 28(sp)             /* Saves XER.                   */
        se_stw      r3, 36(sp)             /* Saves GPR3...GPR12.          */
        se_stw      r4, 40(sp)
        se_stw      r5, 44(sp)
        se_stw      r6, 48(sp)
        se_stw      r7, 52(sp)
        e_stw       r8, 56(sp)
        e_stw       r9, 60(sp)
        e_stw       r10, 64(sp)
        e_stw       r11, 68(sp)
        e_stw       r12, 72(sp)
#endif /* !(PPC_USE_VLE && PPC_SUPPORTS_VLE_MULTI) */

        /* Increasing the SPGR0 register.*/
        mfspr       r0, 272
        se_addi     r0, 1
        mtspr       272, r0

        /* Software vector address from the INTC register.*/
        e_lis       r3, INTC_IACKR_ADDR@h
        e_or2i      r3, INTC_IACKR_ADDR@l   /* IACKR register address.      */
        e_lwz       r3, 0(r3)               /* IACKR register value.        */
        e_lwz       r3, 0(r3)
        mtCTR       r3                      /* Software handler address.    */

        /* Restoring pre-IRQ MSR register value.*/
        mfSRR1      r0
#if !PPC_USE_IRQ_PREEMPTION
        /* No preemption, keeping EE disabled.*/
        se_bclri    r0, 16                  /* EE = bit 16.                 */
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
        se_stw      r3, 0(r3)               /* Writing any value should do. */

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
        se_cmpi     r3, 0
        se_beq      .noresch
        e_bl        chSchDoPreemption
.noresch:
#if CH_DBG_SYSTEM_STATE_CHECK
        e_bl        __dbg_check_unlock
#endif
#if CH_DBG_STATISTICS
        e_bl        __stats_stop_measure_crit_thd
#endif

        /* Restoring the external context.*/
#if PPC_USE_VLE && PPC_SUPPORTS_VLE_MULTI
        e_lmvgprw   32(sp)                 /* Restores GPR0, GPR3...GPR12. */
        e_lmvsprw   16(sp)                 /* Restores CR, LR, CTR, XER.   */
        e_lmvsrrw   8(sp)                  /* Restores PC, MSR.            */
#else /*!(PPC_USE_VLE && PPC_SUPPORTS_VLE_MULTI) */
        se_lwz      r3, 36(sp)             /* Restores GPR3...GPR12.       */
        se_lwz      r4, 40(sp)
        se_lwz      r5, 44(sp)
        se_lwz      r6, 48(sp)
        se_lwz      r7, 52(sp)
        e_lwz       r8, 56(sp)
        e_lwz       r9, 60(sp)
        e_lwz       r10, 64(sp)
        e_lwz       r11, 68(sp)
        e_lwz       r12, 72(sp)
        se_lwz      r0, 8(sp)
        mtSRR0      r0                     /* Restores PC.                 */
        se_lwz      r0, 12(sp)
        mtSRR1      r0                     /* Restores MSR.                */
        se_lwz      r0, 16(sp)
        mtCR        r0                     /* Restores CR.                 */
        se_lwz      r0, 20(sp)
        mtLR        r0                     /* Restores LR.                 */
        se_lwz      r0, 24(sp)
        mtCTR       r0                     /* Restores CTR.                */
        se_lwz      r0, 28(sp)
        mtXER       r0                     /* Restores XER.                */
        se_lwz         r0, 32(sp)             /* Restores GPR0.               */
#endif /* !(PPC_USE_VLE && PPC_SUPPORTS_VLE_MULTI) */
        e_addi      sp, sp, 80             /* Back to the previous frame.  */
        se_rfi

#endif /* !defined(__DOXYGEN__) */

/** @} */
