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
 * @file    hal_icu.c
 * @brief   ICU Driver code.
 *
 * @addtogroup ICU
 * @{
 */

#include "hal.h"

#if (HAL_USE_ICU == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   ICU Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void icuInit(void) {

  icu_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p ICUDriver structure.
 *
 * @param[out] icup     pointer to the @p ICUDriver object
 *
 * @init
 */
void icuObjectInit(ICUDriver *icup) {

  icup->state  = ICU_STOP;
  icup->config = NULL;
}

/**
 * @brief   Configures and activates the ICU peripheral.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @param[in] config    pointer to the @p ICUConfig object
 * @return              The operation status.
 *
 * @api
 */
msg_t icuStart(ICUDriver *icup, const ICUConfig *config) {
  msg_t msg;

  osalDbgCheck((icup != NULL) && (config != NULL));

  osalSysLock();
  osalDbgAssert((icup->state == ICU_STOP) || (icup->state == ICU_READY),
                "invalid state");

  icup->config = config;

#if defined(ICU_LLD_ENHANCED_API)
  msg = icu_lld_start(icup);
  if (msg == HAL_RET_SUCCESS) {
    icup->state = ICU_READY;
  }
  else {
    icup->state = ICU_STOP;
  }
#else
  icu_lld_start(icup);
  icup->state = ICU_READY;
  msg = HAL_RET_SUCCESS;
#endif

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Deactivates the ICU peripheral.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @api
 */
void icuStop(ICUDriver *icup) {

  osalDbgCheck(icup != NULL);

  osalSysLock();

  osalDbgAssert((icup->state == ICU_STOP) || (icup->state == ICU_READY),
                "invalid state");

  icu_lld_stop(icup);
  icup->config = NULL;
  icup->state  = ICU_STOP;

  osalSysUnlock();
}

/**
 * @brief   Starts the input capture.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @api
 */
void icuStartCapture(ICUDriver *icup) {

  osalDbgCheck(icup != NULL);

  osalSysLock();
  osalDbgAssert(icup->state == ICU_READY, "invalid state");
  icuStartCaptureI(icup);
  osalSysUnlock();
}

/**
 * @brief   Waits for a completed capture.
 * @note    The operation could be performed in polled mode depending on.
 * @note    In order to use this function notifications must be disabled.
 * @pre     The driver must be in @p ICU_WAITING or  @p ICU_ACTIVE states.
 * @post    After the capture is available the driver is in @p ICU_ACTIVE
 *          state. If a capture fails then the driver is in @p ICU_WAITING
 *          state.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The capture status.
 * @retval false        if the capture is successful.
 * @retval true         if a timer overflow occurred.
 *
 * @api
 */
bool icuWaitCapture(ICUDriver *icup) {
  bool result;

  osalDbgCheck(icup != NULL);

  osalSysLock();
  osalDbgAssert((icup->state == ICU_WAITING) || (icup->state == ICU_ACTIVE),
                "invalid state");
  osalDbgAssert(icuAreNotificationsEnabledX(icup) == false,
                "notifications enabled");
  result = icu_lld_wait_capture(icup);
  icup->state = result ? ICU_WAITING : ICU_ACTIVE;
  osalSysUnlock();

  return result;
}

/**
 * @brief   Stops the input capture.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @api
 */
void icuStopCapture(ICUDriver *icup) {

  osalDbgCheck(icup != NULL);

  osalSysLock();
  osalDbgAssert((icup->state == ICU_READY) || (icup->state == ICU_WAITING) ||
                (icup->state == ICU_ACTIVE),
                "invalid state");
  icuStopCaptureI(icup);
  osalSysUnlock();
}

/**
 * @brief   Enables notifications.
 * @pre     The ICU unit must have been activated using @p icuStart() and the
 *          capture started using @p icuStartCapture().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @api
 */
void icuEnableNotifications(ICUDriver *icup) {

  osalDbgCheck(icup != NULL);

  osalSysLock();
  osalDbgAssert((icup->state == ICU_WAITING) || (icup->state == ICU_ACTIVE),
                "invalid state");
  icuEnableNotificationsI(icup);
  osalSysUnlock();
}

/**
 * @brief   Disables notifications.
 * @pre     The ICU unit must have been activated using @p icuStart() and the
 *          capture started using @p icuStartCapture().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @api
 */
void icuDisableNotifications(ICUDriver *icup) {

  osalDbgCheck(icup != NULL);

  osalSysLock();
  osalDbgAssert((icup->state == ICU_WAITING) || (icup->state == ICU_ACTIVE),
                "invalid state");
  icuDisableNotificationsI(icup);
  osalSysUnlock();
}

#endif /* HAL_USE_ICU == TRUE */

/** @} */
