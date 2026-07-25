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
 * @file    rt/src/chdebug.c
 * @brief   Debug support code.
 *
 * @addtogroup checks_assertions
 * @details Debug APIs and services:
 *          - Runtime system state and call protocol check. The following
 *            panic messages can be generated:
 *            - SV#1, misplaced @p chSysDisable().
 *              - Called from an ISR.
 *              - Called from a critical zone.
 *              .
 *            - SV#2, misplaced @p chSysSuspend()
 *              - Called from an ISR.
 *              - Called from a critical zone.
 *              .
 *            - SV#3, misplaced @p chSysEnable().
 *              - Called from an ISR.
 *              - Called from a critical zone.
 *              .
 *            - SV#4, misplaced @p chSysLock().
 *              - Called from an ISR.
 *              - Called from a critical zone.
 *              .
 *            - SV#5, misplaced @p chSysUnlock().
 *              - Called from an ISR.
 *              - Not called from a critical zone.
 *              .
 *            - SV#6, misplaced @p chSysLockFromISR().
 *              - Not called from an ISR.
 *              - Called from a critical zone.
 *              .
 *            - SV#7, misplaced @p chSysUnlockFromISR().
 *              - Not called from an ISR.
 *              - Not called from a critical zone.
 *              .
 *            - SV#8, misplaced @p CH_IRQ_PROLOGUE().
 *              - Not called at ISR begin.
 *              - Called from a critical zone.
 *              .
 *            - SV#9, misplaced @p CH_IRQ_EPILOGUE().
 *              - @p CH_IRQ_PROLOGUE() missing.
 *              - Not called at ISR end.
 *              - Called from a critical zone.
 *              .
 *            - SV#10, misplaced I-class function.
 *              - I-class function not called from within a critical zone.
 *              .
 *            - SV#11, misplaced S-class function.
 *              - S-class function not called from within a critical zone.
 *              - Called from an ISR.
 *            .
 *          - Parameters check.
 *          - Kernel assertions.
 *          .
 * @note    Stack checks are not implemented in this module but in the port
 *          layer in an architecture-dependent way.
 * @{
 */

#include "ch.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

#if (CH_DBG_SYSTEM_STATE_CHECK == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Guard code for @p chSysDisable().
 *
 * @notapi
 */
void __dbg_check_disable(void) {
  os_instance_t *oip = currcore;

  if (unlikely((oip->dbg.isr_cnt != (cnt_t)0) ||
               (oip->dbg.lock_cnt != (cnt_t)0))) {
    chSysHalt("SV#1");
  }
}

/**
 * @brief   Guard code for @p chSysSuspend().
 *
 * @notapi
 */
void __dbg_check_suspend(void) {
  os_instance_t *oip = currcore;

  if (unlikely((oip->dbg.isr_cnt != (cnt_t)0) ||
               (oip->dbg.lock_cnt != (cnt_t)0))) {
    chSysHalt("SV#2");
  }
}

/**
 * @brief   Guard code for @p chSysEnable().
 *
 * @notapi
 */
void __dbg_check_enable(void) {
  os_instance_t *oip = currcore;

  if (unlikely((oip->dbg.isr_cnt != (cnt_t)0) ||
               (oip->dbg.lock_cnt != (cnt_t)0))) {
    chSysHalt("SV#3");
  }
}

/**
 * @brief   Guard code for @p chSysLock().
 *
 * @notapi
 */
void __dbg_check_lock(void) {
  os_instance_t *oip = currcore;

  if (unlikely((oip->dbg.isr_cnt != (cnt_t)0) ||
               (oip->dbg.lock_cnt != (cnt_t)0))) {
    chSysHalt("SV#4");
  }
  oip->dbg.lock_cnt = (cnt_t)1;
}

/**
 * @brief   Guard code for @p chSysUnlock().
 *
 * @notapi
 */
void __dbg_check_unlock(void) {
  os_instance_t *oip = currcore;

  if (unlikely((oip->dbg.isr_cnt != (cnt_t)0) ||
               (oip->dbg.lock_cnt <= (cnt_t)0))) {
    chSysHalt("SV#5");
  }
  oip->dbg.lock_cnt = (cnt_t)0;
}

/**
 * @brief   Guard code for @p chSysLockFromIsr().
 *
 * @notapi
 */
void __dbg_check_lock_from_isr(void) {
  os_instance_t *oip = currcore;

  if (unlikely((oip->dbg.isr_cnt <= (cnt_t)0) ||
               (oip->dbg.lock_cnt != (cnt_t)0))) {
    chSysHalt("SV#6");
  }
  oip->dbg.lock_cnt = (cnt_t)1;
}

/**
 * @brief   Guard code for @p chSysUnlockFromIsr().
 *
 * @notapi
 */
void __dbg_check_unlock_from_isr(void) {
  os_instance_t *oip = currcore;

  if (unlikely((oip->dbg.isr_cnt <= (cnt_t)0) ||
               (oip->dbg.lock_cnt <= (cnt_t)0))) {
    chSysHalt("SV#7");
  }
  oip->dbg.lock_cnt = (cnt_t)0;
}

/**
 * @brief   Guard code for @p CH_IRQ_PROLOGUE().
 *
 * @notapi
 */
void __dbg_check_enter_isr(void) {
  os_instance_t *oip = currcore;

  port_lock_from_isr();
  if (unlikely((oip->dbg.isr_cnt < (cnt_t)0) ||
               (oip->dbg.lock_cnt != (cnt_t)0))) {
    chSysHalt("SV#8");
  }
  oip->dbg.isr_cnt++;
  port_unlock_from_isr();
}

/**
 * @brief   Guard code for @p CH_IRQ_EPILOGUE().
 *
 * @notapi
 */
void __dbg_check_leave_isr(void) {
  os_instance_t *oip = currcore;

  port_lock_from_isr();
  if (unlikely((oip->dbg.isr_cnt <= (cnt_t)0) ||
               (oip->dbg.lock_cnt != (cnt_t)0))) {
    chSysHalt("SV#9");
  }
  oip->dbg.isr_cnt--;
  port_unlock_from_isr();
}

/**
 * @brief   I-class functions context check.
 * @details Verifies that the system is in an appropriate state for invoking
 *          an I-class API function. A panic is generated if the state is
 *          not compatible.
 *
 * @api
 */
void chDbgCheckClassI(void) {
  os_instance_t *oip = currcore;

  if (unlikely((oip->dbg.isr_cnt < (cnt_t)0) ||
               (oip->dbg.lock_cnt <= (cnt_t)0))) {
    chSysHalt("SV#10");
  }
}

/**
 * @brief   S-class functions context check.
 * @details Verifies that the system is in an appropriate state for invoking
 *          an S-class API function. A panic is generated if the state is
 *          not compatible.
 *
 * @api
 */
void chDbgCheckClassS(void) {
  os_instance_t *oip = currcore;

  if (unlikely((oip->dbg.isr_cnt != (cnt_t)0) ||
               (oip->dbg.lock_cnt <= (cnt_t)0))) {
    chSysHalt("SV#11");
  }
}

#endif /* CH_DBG_SYSTEM_STATE_CHECK == TRUE */

/** @} */
