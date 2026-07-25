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
 * @file    hal_gpt.h
 * @brief   GPT Driver macros and structures.
 *
 * @addtogroup GPT
 * @{
 */

#ifndef HAL_GPT_H
#define HAL_GPT_H

#if (HAL_USE_GPT == TRUE) || defined(__DOXYGEN__)

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
  GPT_UNINIT = 0,                   /**< Not initialized.                   */
  GPT_STOP = 1,                     /**< Stopped.                           */
  GPT_READY = 2,                    /**< Ready.                             */
  GPT_CONTINUOUS = 3,               /**< Active in continuous mode.         */
  GPT_ONESHOT = 4                   /**< Active in one shot mode.           */
} gptstate_t;

/**
 * @brief   Type of a structure representing a GPT driver.
 */
typedef struct GPTDriver GPTDriver;

/**
 * @brief   GPT notification callback type.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 */
typedef void (*gptcallback_t)(GPTDriver *gptp);

#include "hal_gpt_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Changes the interval of GPT peripheral.
 * @details This function changes the interval of a running GPT unit.
 * @pre     The GPT unit must be running in continuous mode.
 * @post    The GPT unit interval is changed to the new value.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @param[in] interval  new cycle time in timer ticks
 *
 * @iclass
 */
#define gptChangeIntervalI(gptp, interval) {                                \
  gpt_lld_change_interval(gptp, interval);                                  \
}

/**
 * @brief   Returns the interval of GPT peripheral.
 * @pre     The GPT unit must be running in continuous mode.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @return              The current interval.
 *
 * @xclass
 */
#define gptGetIntervalX(gptp) gpt_lld_get_interval(gptp)

/**
 * @brief   Returns the counter value of GPT peripheral.
 * @pre     The GPT unit must be running in continuous mode.
 * @note    The nature of the counter is not defined, it may count upward
 *          or downward, it could be continuously running or not.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @return              The current counter value.
 *
 * @xclass
 */
#define gptGetCounterX(gptp) gpt_lld_get_counter(gptp)

/**
 * @brief   Common ISR code, GPT period event.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
#define _gpt_isr_invoke_cb(gptp) do {                                       \
  if ((gptp)->state == GPT_ONESHOT) {                                       \
    (gptp)->state = GPT_READY;                                              \
      gpt_lld_stop_timer(gptp);                                             \
    }                                                                       \
    if ((gptp)->config->callback != NULL) {                                 \
      (gptp)->config->callback(gptp);                                       \
    }                                                                       \
} while (0)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void gptInit(void);
  void gptObjectInit(GPTDriver *gptp);
  msg_t gptStart(GPTDriver *gptp, const GPTConfig *config);
  void gptStop(GPTDriver *gptp);
  void gptStartContinuous(GPTDriver *gptp, gptcnt_t interval);
  void gptStartContinuousI(GPTDriver *gptp, gptcnt_t interval);
  void gptChangeInterval(GPTDriver *gptp, gptcnt_t interval);
  void gptStartOneShot(GPTDriver *gptp, gptcnt_t interval);
  void gptStartOneShotI(GPTDriver *gptp, gptcnt_t interval);
  void gptStopTimer(GPTDriver *gptp);
  void gptStopTimerI(GPTDriver *gptp);
  void gptPolledDelay(GPTDriver *gptp, gptcnt_t interval);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_GPT == TRUE */

#endif /* HAL_GPT_H */

/** @} */
