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
 * @file    hal_wdg.c
 * @brief   WDG Driver code.
 *
 * @addtogroup WDG
 * @{
 */

#include "hal.h"

#if (HAL_USE_WDG == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   WDG Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void wdgInit(void) {

  wdg_lld_init();
}

/**
 * @brief   Configures and activates the WDG peripheral.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 * @param[in] config    pointer to the @p WDGConfig object
 * @return              The operation status.
 *
 * @api
 */
msg_t wdgStart(WDGDriver *wdgp, const WDGConfig *config) {
  msg_t msg;

  osalDbgCheck((wdgp != NULL) && (config != NULL));

  osalSysLock();
  osalDbgAssert((wdgp->state == WDG_STOP) || (wdgp->state == WDG_READY),
                "invalid state");

  wdgp->config = config;

#if defined(WDG_LLD_ENHANCED_API)
  msg = wdg_lld_start(wdgp);
  if (msg == HAL_RET_SUCCESS) {
    wdgp->state = WDG_READY;
  }
  else {
    wdgp->state = WDG_STOP;
  }
#else
  wdg_lld_start(wdgp);
  wdgp->state = WDG_READY;
  msg = HAL_RET_SUCCESS;
#endif

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Deactivates the WDG peripheral.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @api
 */
void wdgStop(WDGDriver *wdgp) {

  osalDbgCheck(wdgp != NULL);

  osalSysLock();

  osalDbgAssert((wdgp->state == WDG_STOP) || (wdgp->state == WDG_READY),
                "invalid state");

  wdg_lld_stop(wdgp);
  wdgp->config = NULL;
  wdgp->state  = WDG_STOP;

  osalSysUnlock();
}

/**
 * @brief   Resets WDG's counter.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @api
 */
void wdgReset(WDGDriver *wdgp) {

  osalDbgCheck(wdgp != NULL);

  osalSysLock();
  osalDbgAssert(wdgp->state == WDG_READY, "not ready");
  wdgResetI(wdgp);
  osalSysUnlock();
}

#endif /* HAL_USE_WDG == TRUE */

/** @} */
