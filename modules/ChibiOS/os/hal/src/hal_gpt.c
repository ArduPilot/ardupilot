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
 * @file    hal_gpt.c
 * @brief   GPT Driver code.
 *
 * @addtogroup GPT
 * @{
 */

#include "hal.h"

#if (HAL_USE_GPT == TRUE) || defined(__DOXYGEN__)

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
 * @brief   GPT Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void gptInit(void) {

  gpt_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p GPTDriver structure.
 *
 * @param[out] gptp     pointer to the @p GPTDriver object
 *
 * @init
 */
void gptObjectInit(GPTDriver *gptp) {

  gptp->state  = GPT_STOP;
  gptp->config = NULL;
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] config    pointer to the @p GPTConfig object
 * @return              The operation status.
 *
 * @api
 */
msg_t gptStart(GPTDriver *gptp, const GPTConfig *config) {
  msg_t msg;

  osalDbgCheck((gptp != NULL) && (config != NULL));

  osalSysLock();

  osalDbgAssert((gptp->state == GPT_STOP) || (gptp->state == GPT_READY),
              "invalid state");

  gptp->config = config;

#if defined(GPT_LLD_ENHANCED_API)
  msg = gpt_lld_start(gptp);
  if (msg == HAL_RET_SUCCESS) {
    gptp->state = GPT_READY;
  }
  else {
    gptp->state = GPT_STOP;
  }
#else
  gpt_lld_start(gptp);
  gptp->state = GPT_READY;
  msg = HAL_RET_SUCCESS;
#endif

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @api
 */
void gptStop(GPTDriver *gptp) {

  osalDbgCheck(gptp != NULL);

  osalSysLock();

  osalDbgAssert((gptp->state == GPT_STOP) || (gptp->state == GPT_READY),
                "invalid state");

  gpt_lld_stop(gptp);
  gptp->config = NULL;
  gptp->state  = GPT_STOP;

  osalSysUnlock();
}

/**
 * @brief   Changes the interval of GPT peripheral.
 * @details This function changes the interval of a running GPT unit.
 * @pre     The GPT unit must be running in continuous mode.
 * @post    The GPT unit interval is changed to the new value.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @param[in] interval  new cycle time in timer ticks
 *
 * @api
 */
void gptChangeInterval(GPTDriver *gptp, gptcnt_t interval) {

  osalDbgCheck(gptp != NULL);

  osalSysLock();
  osalDbgAssert(gptp->state == GPT_CONTINUOUS,
                "invalid state");
  gptChangeIntervalI(gptp, interval);
  osalSysUnlock();
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  period in ticks
 *
 * @api
 */
void gptStartContinuous(GPTDriver *gptp, gptcnt_t interval) {

  osalSysLock();
  gptStartContinuousI(gptp, interval);
  osalSysUnlock();
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  period in ticks
 *
 * @iclass
 */
void gptStartContinuousI(GPTDriver *gptp, gptcnt_t interval) {

  osalDbgCheckClassI();
  osalDbgCheck(gptp != NULL);
  osalDbgAssert(gptp->state == GPT_READY,
                "invalid state");

  gptp->state = GPT_CONTINUOUS;
  gpt_lld_start_timer(gptp, interval);
}

/**
 * @brief   Starts the timer in one shot mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @api
 */
void gptStartOneShot(GPTDriver *gptp, gptcnt_t interval) {

  osalSysLock();
  gptStartOneShotI(gptp, interval);
  osalSysUnlock();
}

/**
 * @brief   Starts the timer in one shot mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @api
 */
void gptStartOneShotI(GPTDriver *gptp, gptcnt_t interval) {

  osalDbgCheckClassI();
  osalDbgCheck(gptp != NULL);
  osalDbgCheck(gptp->config->callback != NULL);
  osalDbgAssert(gptp->state == GPT_READY,
                "invalid state");

  gptp->state = GPT_ONESHOT;
  gpt_lld_start_timer(gptp, interval);
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @api
 */
void gptStopTimer(GPTDriver *gptp) {

  osalSysLock();
  gptStopTimerI(gptp);
  osalSysUnlock();
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @api
 */
void gptStopTimerI(GPTDriver *gptp) {

  osalDbgCheckClassI();
  osalDbgCheck(gptp != NULL);
  osalDbgAssert((gptp->state == GPT_READY) || (gptp->state == GPT_CONTINUOUS) ||
                (gptp->state == GPT_ONESHOT),
                "invalid state");

  gptp->state = GPT_READY;
  gpt_lld_stop_timer(gptp);
}

/**
 * @brief   Starts the timer in one shot mode and waits for completion.
 * @details This function specifically polls the timer waiting for completion
 *          in order to not have extra delays caused by interrupt servicing,
 *          this function is only recommended for short delays.
 * @note    The configured callback is not invoked when using this function.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @api
 */
void gptPolledDelay(GPTDriver *gptp, gptcnt_t interval) {

  osalDbgAssert(gptp->state == GPT_READY,
                "invalid state");

  gptp->state = GPT_ONESHOT;
  gpt_lld_polled_delay(gptp, interval);
  gptp->state = GPT_READY;
}

#endif /* HAL_USE_GPT == TRUE */

/** @} */
