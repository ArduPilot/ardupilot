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
 * @file    xWDGv1/hal_wdg_lld.c
 * @brief   WDG Driver subsystem low level driver source.
 *
 * @addtogroup WDG
 * @{
 */

#include "hal.h"

#if (HAL_USE_WDG == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define KR_KEY_RELOAD                       0xAAAAU
#define KR_KEY_ENABLE                       0xCCCCU
#define KR_KEY_WRITE                        0x5555U
#define KR_KEY_PROTECT                      0x0000U

#if !defined(IWDG) && defined(IWDG1)
#define IWDG                                IWDG1
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if STM32_WDG_USE_IWDG || defined(__DOXYGEN__)
WDGDriver WDGD1;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level WDG driver initialization.
 *
 * @notapi
 */
void wdg_lld_init(void) {

#if STM32_WDG_USE_IWDG
  WDGD1.state = WDG_STOP;
  WDGD1.wdg   = IWDG;
#endif
}

/**
 * @brief   Configures and activates the WDG peripheral.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @notapi
 */
void wdg_lld_start(WDGDriver *wdgp) {

  /* Enable IWDG and unlock for write.*/
  wdgp->wdg->KR   = KR_KEY_ENABLE;
  wdgp->wdg->KR   = KR_KEY_WRITE;

  /* Write configuration.*/
  wdgp->wdg->PR   = wdgp->config->pr;
  wdgp->wdg->RLR  = wdgp->config->rlr;

  /* Wait the registers to be updated.*/
  while (wdgp->wdg->SR != 0)
    ;

#if STM32_IWDG_IS_WINDOWED
  /* This also triggers a refresh.*/
  wdgp->wdg->WINR = wdgp->config->winr;
#else
  wdgp->wdg->KR   = KR_KEY_RELOAD;
#endif
}

/**
 * @brief   Deactivates the WDG peripheral.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @notapi
 */
void wdg_lld_stop(WDGDriver *wdgp) {

  osalDbgAssert(wdgp->state == WDG_STOP,
                "IWDG cannot be stopped once activated");
}

/**
 * @brief   Reloads WDG's counter.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @notapi
 */
void wdg_lld_reset(WDGDriver * wdgp) {

  wdgp->wdg->KR = KR_KEY_RELOAD;
}

#endif /* HAL_USE_WDG == TRUE */

/** @} */
