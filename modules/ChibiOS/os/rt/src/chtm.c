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
 * @file    rt/src/chtm.c
 * @brief   Time Measurement module code.
 *
 * @addtogroup time_measurement
 * @details Time Measurement APIs and services.
 * @{
 */

#include "ch.h"

#if (CH_CFG_USE_TM == TRUE) || defined(__DOXYGEN__)

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

static inline void tm_stop(time_measurement_t *tmp,
                           rtcnt_t now,
                           rtcnt_t offset) {

  tmp->n++;
  tmp->last = (now - tmp->last) - offset;
  tmp->cumulative += (rttime_t)tmp->last;
  if (tmp->last > tmp->worst) {
    tmp->worst = tmp->last;
  }
  if (tmp->last < tmp->best) {
    tmp->best = tmp->last;
  }
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes a @p TimeMeasurement object.
 *
 * @param[out] tmp      pointer to a @p TimeMeasurement structure
 *
 * @init
 */
void chTMObjectInit(time_measurement_t *tmp) {

  tmp->best       = (rtcnt_t)-1;
  tmp->worst      = (rtcnt_t)0;
  tmp->last       = (rtcnt_t)0;
  tmp->n          = (ucnt_t)0;
  tmp->cumulative = (rttime_t)0;
}

/**
 * @brief   Starts a measurement.
 * @pre     The @p time_measurement_t structure must be initialized.
 *
 * @param[in,out] tmp   pointer to a @p TimeMeasurement structure
 *
 * @xclass
 */
NOINLINE void chTMStartMeasurementX(time_measurement_t *tmp) {

  tmp->last = chSysGetRealtimeCounterX();
}

/**
 * @brief   Stops a measurement.
 * @pre     The @p time_measurement_t structure must be initialized.
 *
 * @param[in,out] tmp   pointer to a @p time_measurement_t structure
 *
 * @xclass
 */
NOINLINE void chTMStopMeasurementX(time_measurement_t *tmp) {

  tm_stop(tmp, chSysGetRealtimeCounterX(), ch_system.tmc.offset);
}

/**
 * @brief   Stops a measurement and chains to the next one using the same time
 *          stamp.
 *
 * @param[in,out] tmp1  pointer to the @p time_measurement_t structure to be
 *                      stopped
 * @param[in,out] tmp2  pointer to the @p time_measurement_t structure to be
 *                      started
 *
 *
 * @xclass
 */
NOINLINE void chTMChainMeasurementToX(time_measurement_t *tmp1,
                                      time_measurement_t *tmp2) {

  /* Starts new measurement.*/
  tmp2->last = chSysGetRealtimeCounterX();

  /* Stops previous measurement using the same time stamp.*/
  tm_stop(tmp1, tmp2->last, (rtcnt_t)0);
}

#endif /* CH_CFG_USE_TM == TRUE */

/** @} */
