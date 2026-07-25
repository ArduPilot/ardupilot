/*
    ChibiOS - Copyright (C) 2016..2018 Theodore Ateba

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
 * @file    hal_wdg_lld.c
 * @brief   AVR WDG Driver subsystem low level driver source.
 *
 * @addtogroup WDG
 * @{
 */

#include "hal.h"

#if (HAL_USE_WDG == TRUE) || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

#if (AVR_WDG_USE_WDT == TRUE) || defined(__DOXYGEN__)
WDGDriver WDTD1;
#endif

/*==========================================================================*/
/* Driver local variables.                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/**
 * @brief   Check if Synchronisation busy flag is set.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 * @return              synchronisazion busy flag value
 * @retval    true      The synchronisation is in process.
 * @retval    false     The synchronisation is done.
 */
static bool wdg_get_sync_busy_flag(WDGDriver *wdgp) {

  if (wdgp->wdg->STATUS & WDT_SYNCBUSY_bm)
    return true;
  else
    return false;
}

/**
 * @brief   Enable Watchdog module.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 */
static void wdg_enable(WDGDriver *wdgp) {

  uint8_t cfg = wdgp->wdg->CTRL | WDT_ENABLE_bm | WDT_CEN_bm;
  CCP = CCP_IOREG_gc;
  wdgp->wdg->CTRL = cfg;

  while (wdg_get_sync_busy_flag(wdgp));
}

/**
 * @brief   Set the watchdog period for the reset operation.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 */
static void wdg_set_period(WDGDriver *wdgp) {

  uint8_t cfg = WDT_ENABLE_bm | WDT_CEN_bm | wdgp->config->ntp;
  CCP = CCP_IOREG_gc;
  wdgp->wdg->CTRL = cfg;

  while (wdg_get_sync_busy_flag(wdgp));
}

/**
 * @brief   Disable Watchdog module.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 */
static void wdg_disable(WDGDriver *wdgp) {

  uint8_t cfg = (wdgp->wdg->CTRL & ~WDT_ENABLE_bm) | WDT_CEN_bm;
  CCP = CCP_IOREG_gc;
  wdgp->wdg->CTRL = cfg;
}

/**
 * @brief Enable watchdog window mode.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 * @return    wd_enble  status of the watchdog before the operation
 * @retval    true      The WD is enabled before enabling window mode.
 * @retval    false     The WD is not enabled before enabling window mode.
 */
static bool wdg_enable_window_mode(WDGDriver *wdgp) {

  uint8_t wdStatus = wdgp->wdg->CTRL & WDT_ENABLE_bm;
  uint8_t cfg = wdgp->wdg->WINCTRL | WDT_WEN_bm | WDT_WCEN_bm;

  CCP = CCP_IOREG_gc;
  wdgp->wdg->WINCTRL = cfg;

  while (wdg_get_sync_busy_flag(wdgp));

  return wdStatus;
}

/**
 * @brief Enable window mode and set period.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 * @return              The satus of the watchdog before this operation
 * @retval    true      The WD is enabled before enabling window mode.
 * @retval    false     The WD is not enabled before enabling window mode.
 */
static bool wdg_set_window_period(WDGDriver *wdgp) {

  uint8_t wdStatus = wdgp->wdg->CTRL & WDT_ENABLE_bm;
  uint8_t cfg = WDT_WEN_bm | WDT_WCEN_bm | wdgp->config->ntp;

  CCP = CCP_IOREG_gc;
  wdgp->wdg->WINCTRL = cfg;

  while (wdg_get_sync_busy_flag(wdgp));

  return wdStatus;
}

/**
 * @brief   Disable window mode.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 */
static void wdg_disable_window_mode(WDGDriver *wdgp) {

  uint8_t cfg = (wdgp->wdg->WINCTRL & ~WDT_WEN_bm) | WDT_WCEN_bm;

  CCP = CCP_IOREG_gc;
  wdgp->wdg->WINCTRL = cfg;
}

/**
 * @brief   Low level reset of the Watchgot timer by software.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 */
static void wdg_reset(WDGDriver *wdgp) {

  (void)wdgp;
  asm("wdr"); /* The reset is made by calling WDR instruction. */
}

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Low level WDG driver initialization.
 *
 * @notapi
 */
void wdg_lld_init(void) {

#if AVR_WDG_USE_WDT
  WDTD1.state = WDG_STOP;
  WDTD1.wdg   = &WDT;
#endif
  /*  TODO: See if the configuration of the watchgod must
   * be done with a default configuration.
   */
}

/**
 * @brief   Configures and activates the WDG peripheral.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @notapi
 */
void wdg_lld_start(WDGDriver *wdgp) {

  /* TODO: Use the wdgp->state to see if it is a start or a restart
   * and look what to do
   */

  if (wdgp->config->mode) {
    wdg_set_window_period(wdgp);
    wdg_enable_window_mode(wdgp);
  }
  else {
    wdg_set_period(wdgp);
    wdg_enable(wdgp);
  }

  while (wdg_get_sync_busy_flag(wdgp));
}

/**
 * @brief   Deactivates the WDG peripheral.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @notapi
 */
void wdg_lld_stop(WDGDriver *wdgp) {

  if (wdgp->config->mode) {
    wdg_disable_window_mode(wdgp);
  }
  else {
    wdg_disable(wdgp);
  }
}

/**
 * @brief   Reloads WDG's counter.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @notapi
 */
void wdg_lld_reset(WDGDriver *wdgp) {

  wdg_reset(wdgp);
}

#endif /* HAL_USE_WDG */

/** @} */
