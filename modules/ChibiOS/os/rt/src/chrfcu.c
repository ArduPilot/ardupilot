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
 * @file    rt/src/chrfcu.c
 * @brief   Runtime Faults Collection Unit code.
 *
 * @addtogroup rfcu
 * @details Runtime Faults Collection Unit service.
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

/**
 * @brief   Adds fault flags to the current mask.
 *
 * @param[in] mask      fault flags to be added
 */
void chRFCUCollectFaultsI(rfcu_mask_t mask) {

#if CH_CFG_SMP_MODE == FALSE
  currcore->rfcu.mask |= mask;
#else
  ch_system.rfcu.mask |= mask;
#endif

  CH_CFG_RUNTIME_FAULTS_HOOK(mask);
}

/**
 * @brief   Returns the current faults mask clearing it.
 *
 * @param[in] mask      mask of faults to be read and cleared
 * @return              The current faults mask.
 * @retval 0            if no faults were collected since last call to this
 *                      function.
 */
rfcu_mask_t chRFCUGetAndClearFaultsI(rfcu_mask_t mask) {
  rfcu_mask_t m;

#if CH_CFG_SMP_MODE == FALSE
  os_instance_t *oip = currcore;

  m = oip->rfcu.mask & mask;
  oip->rfcu.mask &= ~m;
#else

  m = ch_system.rfcu.mask & mask;
  ch_system.rfcu.mask &= ~m;
#endif

  return m;
}
/** @} */
