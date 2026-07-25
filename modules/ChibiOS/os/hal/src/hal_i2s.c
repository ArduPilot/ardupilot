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
 * @file    hal_i2s.c
 * @brief   I2S Driver code.
 *
 * @addtogroup I2S
 * @{
 */

#include "hal.h"

#if (HAL_USE_I2S == TRUE) || defined(__DOXYGEN__)

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
 * @brief   I2S Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void i2sInit(void) {

  i2s_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p I2SDriver structure.
 *
 * @param[out] i2sp     pointer to the @p I2SDriver object
 *
 * @init
 */
void i2sObjectInit(I2SDriver *i2sp) {

  i2sp->state  = I2S_STOP;
  i2sp->config = NULL;
}

/**
 * @brief   Configures and activates the I2S peripheral.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 * @param[in] config    pointer to the @p I2SConfig object
 * @return              The operation status.
 *
 * @api
 */
msg_t i2sStart(I2SDriver *i2sp, const I2SConfig *config) {
  msg_t msg;

  osalDbgCheck((i2sp != NULL) && (config != NULL));

  osalSysLock();
  osalDbgAssert((i2sp->state == I2S_STOP) || (i2sp->state == I2S_READY),
                "invalid state");

  i2sp->config = config;

#if defined(I2S_LLD_ENHANCED_API)
  msg = i2s_lld_start(i2sp);
  if (msg == HAL_RET_SUCCESS) {
    i2sp->state = I2S_READY;
  }
  else {
    i2sp->state = I2S_STOP;
  }
#else
  i2s_lld_start(i2sp);
  i2sp->state = I2S_READY;
  msg = HAL_RET_SUCCESS;
#endif

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Deactivates the I2S peripheral.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @api
 */
void i2sStop(I2SDriver *i2sp) {

  osalDbgCheck(i2sp != NULL);

  osalSysLock();

  osalDbgAssert((i2sp->state == I2S_STOP) || (i2sp->state == I2S_READY),
                "invalid state");

  i2s_lld_stop(i2sp);
  i2sp->config = NULL;
  i2sp->state  = I2S_STOP;

  osalSysUnlock();
}

/**
 * @brief   Starts a I2S data exchange.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @api
 */
void i2sStartExchange(I2SDriver *i2sp) {

  osalDbgCheck(i2sp != NULL);

  osalSysLock();
  osalDbgAssert(i2sp->state == I2S_READY, "not ready");
  i2sStartExchangeI(i2sp);
  osalSysUnlock();
}

/**
 * @brief   Stops the ongoing data exchange.
 * @details The ongoing data exchange, if any, is stopped, if the driver
 *          was not active the function does nothing.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @api
 */
void i2sStopExchange(I2SDriver *i2sp) {

  osalDbgCheck((i2sp != NULL));

  osalSysLock();
  osalDbgAssert((i2sp->state == I2S_READY) ||
                (i2sp->state == I2S_ACTIVE) ||
                (i2sp->state == I2S_COMPLETE),
                "invalid state");
  i2sStopExchangeI(i2sp);
  osalSysUnlock();
}

#endif /* HAL_USE_I2S == TRUE */

/** @} */
