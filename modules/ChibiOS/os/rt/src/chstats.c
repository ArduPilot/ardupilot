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
 * @file    rt/src/chstats.c
 * @brief   Statistics module code.
 *
 * @addtogroup statistics
 * @details Statistics services.
 * @{
 */

#include "ch.h"

#if (CH_DBG_STATISTICS == TRUE) || defined(__DOXYGEN__)

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

/**
 * @brief   Increases the IRQ counter.
 */
void __stats_increase_irq(void) {

  port_lock_from_isr();
  currcore->kernel_stats.n_irq++;
  port_unlock_from_isr();
}

/**
 * @brief   Updates context switch related statistics.
 *
 * @param[in] ntp       the thread to be switched in
 * @param[in] otp       the thread to be switched out
 */
void __stats_ctxswc(thread_t *ntp, thread_t *otp) {

  currcore->kernel_stats.n_ctxswc++;
  chTMChainMeasurementToX(&otp->stats, &ntp->stats);
}

/**
 * @brief   Starts the measurement of a thread critical zone.
 */
void __stats_start_measure_crit_thd(void) {

  chTMStartMeasurementX(&currcore->kernel_stats.m_crit_thd);
}

/**
 * @brief   Stops the measurement of a thread critical zone.
 */
void __stats_stop_measure_crit_thd(void) {

  chTMStopMeasurementX(&currcore->kernel_stats.m_crit_thd);
}

/**
 * @brief   Starts the measurement of an ISR critical zone.
 */
void __stats_start_measure_crit_isr(void) {

  chTMStartMeasurementX(&currcore->kernel_stats.m_crit_isr);
}

/**
 * @brief   Stops the measurement of an ISR critical zone.
 */
void __stats_stop_measure_crit_isr(void) {

  chTMStopMeasurementX(&currcore->kernel_stats.m_crit_isr);
}

#endif /* CH_DBG_STATISTICS == TRUE */

/** @} */
