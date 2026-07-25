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
 * @file    hal_wdg.h
 * @brief   WDG Driver macros and structures.
 *
 * @addtogroup WDG
 * @{
 */

#ifndef HAL_WDG_H
#define HAL_WDG_H

#if (HAL_USE_WDG == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  WDG_UNINIT = 0,                   /**< Not initialized.                   */
  WDG_STOP = 1,                     /**< Stopped.                           */
  WDG_READY = 2                     /**< Ready.                             */
} wdgstate_t;

#include "hal_wdg_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Resets WDG's counter.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @iclass
 */
#define wdgResetI(wdgp) wdg_lld_reset(wdgp)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void wdgInit(void);
  msg_t wdgStart(WDGDriver *wdgp, const WDGConfig * config);
  void wdgStop(WDGDriver *wdgp);
  void wdgReset(WDGDriver *wdgp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_WDG == TRUE */

#endif /* HAL_WDG_H */

/** @} */
