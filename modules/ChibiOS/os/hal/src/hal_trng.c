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
 * @file    hal_trng.c
 * @brief   TRNG Driver code.
 *
 * @addtogroup TRNG
 * @{
 */

#include "hal.h"

#if (HAL_USE_TRNG == TRUE) || defined(__DOXYGEN__)

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
 * @brief   TRNG Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void trngInit(void) {

  trng_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p TRNGDriver structure.
 *
 * @param[out] trngp     pointer to the @p TRNGDriver object
 *
 * @init
 */
void trngObjectInit(TRNGDriver *trngp) {

  trngp->state  = TRNG_STOP;
  trngp->config = NULL;
}

/**
 * @brief   Configures and activates the TRNG peripheral.
 *
 * @param[in] trngp     pointer to the @p TRNGDriver object
 * @param[in] config    pointer to the @p TRNGConfig object or @p NULL for
 *                      default configuration
 * @return              The operation status.
 *
 * @api
 */
msg_t trngStart(TRNGDriver *trngp, const TRNGConfig *config) {
  msg_t msg;

  osalDbgCheck(trngp != NULL);

  osalSysLock();
  osalDbgAssert((trngp->state == TRNG_STOP) || (trngp->state == TRNG_READY),
              "invalid state");

  trngp->config = config;

#if defined(TRNG_LLD_ENHANCED_API)
  msg = trng_lld_start(trngp);
  if (msg == HAL_RET_SUCCESS) {
    trngp->state = TRNG_READY;
  }
  else {
    trngp->state = TRNG_STOP;
  }
#else
  trng_lld_start(trngp);
  trngp->state = TRNG_READY;
  msg = HAL_RET_SUCCESS;
#endif

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Deactivates the TRNG peripheral.
 *
 * @param[in] trngp      pointer to the @p TRNGDriver object
 *
 * @api
 */
void trngStop(TRNGDriver *trngp) {

  osalDbgCheck(trngp != NULL);

  osalSysLock();

  osalDbgAssert((trngp->state == TRNG_STOP) || (trngp->state == TRNG_READY),
                "invalid state");

  trng_lld_stop(trngp);
  trngp->config = NULL;
  trngp->state  = TRNG_STOP;

  osalSysUnlock();
}

/**
 * @brief   True random numbers generator.
 * @note    The function is blocking and likely performs polled waiting
 *          inside the low level implementation.
 *
 * @param[in] trngp             pointer to the @p TRNGDriver object
 * @param[in] size              size of output buffer
 * @param[out] out              output buffer
 * @return                      The operation status.
 * @retval false                if a random number has been generated.
 * @retval true                 if an HW error occurred.
 *
 * @api
 */
bool trngGenerate(TRNGDriver *trngp, size_t size, uint8_t *out) {
  bool err;

  osalDbgCheck((trngp != NULL) && (out != NULL));

  osalDbgAssert(trngp->state == TRNG_READY, "not ready");

  trngp->state = TRNG_RUNNING;

  err = trng_lld_generate(trngp, size, out);

  trngp->state = TRNG_READY;

  return err;
}

#endif /* HAL_USE_TRNG == TRUE */

/** @} */
