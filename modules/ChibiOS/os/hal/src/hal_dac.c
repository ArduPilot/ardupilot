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
 * @file    hal_dac.c
 * @brief   DAC Driver code.
 *
 * @addtogroup DAC
 * @{
 */

#include "hal.h"

#if (HAL_USE_DAC == TRUE) || defined(__DOXYGEN__)

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
 * @brief   DAC Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void dacInit(void) {

  dac_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p DACDriver structure.
 *
 * @param[out] dacp     pointer to the @p DACDriver object
 *
 * @init
 */
void dacObjectInit(DACDriver *dacp) {

  dacp->state = DAC_STOP;
  dacp->config = NULL;
#if (DAC_USE_SYNCHRONIZATION == TRUE)
  dacp->thread = NULL;
#endif
#if DAC_USE_MUTUAL_EXCLUSION
  osalMutexObjectInit(&dacp->mutex);
#endif
#if defined(DAC_DRIVER_EXT_INIT_HOOK)
  DAC_DRIVER_EXT_INIT_HOOK(dacp);
#endif
}

/**
 * @brief   Configures and activates the DAC peripheral.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] config    pointer to the @p DACConfig object, it can be
 *                      @p NULL if the low level driver implementation
 *                      supports a default configuration
 *
 * @return              The operation status.
 *
 * @api
 */
msg_t dacStart(DACDriver *dacp, const DACConfig *config) {
  msg_t msg;

  osalDbgCheck(dacp != NULL);

  osalSysLock();

  osalDbgAssert((dacp->state == DAC_STOP) || (dacp->state == DAC_READY),
                "invalid state");

  dacp->config = config;

#if defined(DAC_LLD_ENHANCED_API)
  msg = dac_lld_start(dacp);
  if (msg == HAL_RET_SUCCESS) {
    dacp->state = DAC_READY;
  }
  else {
    dacp->state = DAC_STOP;
  }
#else
  dac_lld_start(dacp);
  dacp->state = DAC_READY;
  msg = HAL_RET_SUCCESS;
#endif

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Deactivates the DAC peripheral.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @api
 */
void dacStop(DACDriver *dacp) {

  osalDbgCheck(dacp != NULL);

  osalSysLock();

  osalDbgAssert((dacp->state == DAC_STOP) || (dacp->state == DAC_READY),
                "invalid state");

  dac_lld_stop(dacp);
  dacp->config = NULL;
  dacp->state  = DAC_STOP;

  osalSysUnlock();
}

/**
 * @brief   Outputs a value directly on a DAC channel.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] channel   DAC channel number
 * @param[in] sample    value to be output
 *
 * @xclass
 */
msg_t dacPutChannelX(DACDriver *dacp, dacchannel_t channel, dacsample_t sample) {

  osalDbgCheck(channel < (dacchannel_t)DAC_MAX_CHANNELS);
  osalDbgAssert((dacp->state == DAC_READY)  ||
                (dacp->state == DAC_ACTIVE) ||
                (dacp->state == DAC_COMPLETE), "invalid state");
  msg_t msg;
#if defined(DAC_LLD_ENHANCED_API)
  msg = dac_lld_put_channel(dacp, channel, sample);
#else
  dac_lld_put_channel(dacp, channel, sample);
  msg = HAL_RET_SUCCESS;
#endif
  return msg;
}

/**
 * @brief   Starts a DAC conversion.
 * @details Starts an asynchronous conversion operation.
 * @note    The buffer is organized as a matrix of M*N elements where M is the
 *          channels number configured into the conversion group and N is the
 *          buffer depth. The samples are sequentially written into the buffer
 *          with no gaps.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] grpp      pointer to a @p DACConversionGroup object
 * @param[in] samples   pointer to the samples buffer
 * @param[in] depth     buffer depth (matrix rows number). The buffer depth
 *                      must be one or an even number.
 *
 * @return              The operation status.
 *
 * @api
 */
msg_t dacStartConversion(DACDriver *dacp,
                        const DACConversionGroup *grpp,
                        dacsample_t *samples,
                        size_t depth) {
  msg_t msg;
  osalSysLock();
#if defined(DAC_LLD_ENHANCED_API)
  msg = dacStartConversionI(dacp, grpp, samples, depth);
#else
  (void) dacStartConversionI(dacp, grpp, samples, depth);
  msg = HAL_RET_SUCCESS;
#endif
  osalSysUnlock();
  return msg;
}

/**
 * @brief   Starts a DAC conversion.
 * @details Starts an asynchronous conversion operation.
 * @post    The callbacks associated to the conversion group will be invoked
 *          on buffer complete and error events.
 * @note    DAC conversions are circular/streaming operations, a new
 *          conversion can only be started after stopping the current one.
 * @note    The buffer is organized as a matrix of M*N elements where M is the
 *          channels number configured into the conversion group and N is the
 *          buffer depth. The samples are sequentially organised in the buffer
 *          with no gaps.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] grpp      pointer to a @p DACConversionGroup object
 * @param[in] samples   pointer to the samples buffer
 * @param[in] depth     buffer depth (matrix rows number). The buffer depth
 *                      must be one or an even number.
 *
 * @return              The operation status.
 *
 * @iclass
 */
msg_t dacStartConversionI(DACDriver *dacp,
                         const DACConversionGroup *grpp,
                         dacsample_t *samples,
                         size_t depth) {
  msg_t msg;

  osalDbgCheckClassI();
  osalDbgCheck((dacp != NULL) && (grpp != NULL) && (samples != NULL) &&
               ((depth == 1U) || ((depth & 1U) == 0U)));
  osalDbgAssert((dacp->state == DAC_READY) ||
                (dacp->state == DAC_ERROR),
                "not ready");

  dacp->samples  = samples;
  dacp->depth    = depth;
  dacp->grpp     = grpp;
#if defined(DAC_LLD_ENHANCED_API)
  msg = dac_lld_start_conversion(dacp);
  if (msg == HAL_RET_SUCCESS) {
    dacp->state = DAC_ACTIVE;
  }
  else {
    dacp->grpp = NULL;
  }
#else
  dac_lld_start_conversion(dacp);
  dacp->state = DAC_ACTIVE;
  msg = HAL_RET_SUCCESS;
#endif

  return msg;
}

/**
 * @brief   Stops an ongoing conversion.
 * @details This function stops the currently ongoing conversion and returns
 *          the driver in the @p DAC_READY state. If there was no conversion
 *          being processed then the function does nothing.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @api
 */
void dacStopConversion(DACDriver *dacp) {

  osalDbgCheck(dacp != NULL);

  osalSysLock();

  osalDbgAssert((dacp->state == DAC_READY) ||
                (dacp->state == DAC_ACTIVE),
                "invalid state");

  if (dacp->state != DAC_READY) {
    dac_lld_stop_conversion(dacp);
    dacp->grpp  = NULL;
    dacp->state = DAC_READY;
    _dac_reset_s(dacp);
  }

  osalSysUnlock();
}

/**
 * @brief   Stops an ongoing conversion.
 * @details This function stops the currently ongoing conversion and returns
 *          the driver in the @p DAC_READY state. If there was no conversion
 *          being processed then the function does nothing.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @iclass
 */
void dacStopConversionI(DACDriver *dacp) {

  osalDbgCheckClassI();
  osalDbgCheck(dacp != NULL);
  osalDbgAssert((dacp->state == DAC_READY) ||
                (dacp->state == DAC_ACTIVE) ||
                (dacp->state == DAC_COMPLETE),
                "invalid state");

  if (dacp->state != DAC_READY) {
    dac_lld_stop_conversion(dacp);
    dacp->grpp  = NULL;
    dacp->state = DAC_READY;
    _dac_reset_i(dacp);
  }
}

#if (DAC_USE_SYNCHRONIZATION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Performs a DAC conversion.
 * @details Performs a synchronous conversion operation.
 * @note    The buffer is organized as a matrix of M*N elements where M is the
 *          channels number configured into the conversion group and N is the
 *          buffer depth. The samples are sequentially organised in the buffer
 *          with no gaps.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] grpp      pointer to a @p DACConversionGroup object
 * @param[in] samples   pointer to the samples buffer
 * @param[in] depth     buffer depth (matrix rows number). The buffer depth
 *                      must be one or an even number.
 *
 * @return              The operation result.
 * @retval MSG_OK       Conversion finished.
 * @retval MSG_RESET    The conversion has been stopped using
 *                      @p dacStopConversion() or @p dacStopConversionI().
 * @retval MSG_TIMEOUT  The conversion has been stopped because an hardware
 *                      error.
 * @note    Starting a follow-on conversion from the callback before this
 *          function returns is undefined behavior.
 *
 * @api
 */
msg_t dacConvert(DACDriver *dacp,
                 const DACConversionGroup *grpp,
                 dacsample_t *samples,
                 size_t depth) {
  msg_t msg;

  osalSysLock();

  msg = dacStartConversionI(dacp, grpp, samples, depth);
  if (msg != HAL_RET_SUCCESS) {
    osalSysUnlock();
    return msg;
  }
  msg = osalThreadSuspendS(&dacp->thread);

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Synchronize to a conversion completion.
 * @note    This function can only be called by a single thread at time.
 * @note    Starting a follow-on conversion from the callback before this
 *          function returns is undefined behavior.
 *
 * @param[in] dacp             pointer to the @p DACDriver object
 * @param[in] timeout          wait timeout
 *
 * @return                      The wait result.
 * @retval MSG_OK               if operation completed without errors.
 * @retval MSG_TIMEOUT          if synchronization request timed out.
 * @retval MSG_RESET            if the conversion has been stopped.
 *
 * @sclass
 */
msg_t dacSynchronizeS(DACDriver *dacp, sysinterval_t timeout) {
  msg_t msg;

  osalDbgCheckClassS();
  osalDbgCheck(dacp != NULL);
  osalDbgAssert((dacp->state == DAC_ACTIVE) || (dacp->state == DAC_READY),
                "invalid state");

  if (dacp->state == DAC_ACTIVE) {
    msg = osalThreadSuspendTimeoutS(&dacp->thread, timeout);
  }
  else {
    msg = MSG_OK;
  }

  return msg;
}

/**
 * @brief   Synchronize to a conversion completion.
 * @note    This function can only be called by a single thread at time.
 *
 * @param[in] dacp              pointer to the @p DACDriver object
 * @param[in] timeout           wait timeout
 *
 * @return                      The wait result.
 * @retval MSG_OK               if operation completed without errors.
 * @retval MSG_TIMEOUT          if synchronization request timed out.
 * @retval MSG_RESET            if the conversion has been stopped.
 *
 * @api
 */
msg_t dacSynchronize(DACDriver *dacp, sysinterval_t timeout) {
  msg_t msg;

  osalSysLock();
  msg = dacSynchronizeS(dacp, timeout);
  osalSysUnlock();

  return msg;
}
#endif /* DAC_USE_SYNCHRONIZATION == TRUE */

#if (DAC_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Gains exclusive access to the DAC bus.
 * @details This function tries to gain ownership to the DAC bus, if the bus
 *          is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option @p DAC_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @api
 */
void dacAcquireBus(DACDriver *dacp) {

  osalDbgCheck(dacp != NULL);

  osalMutexLock(&dacp->mutex);
}

/**
 * @brief   Releases exclusive access to the DAC bus.
 * @pre     In order to use this function the option @p DAC_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @api
 */
void dacReleaseBus(DACDriver *dacp) {

  osalDbgCheck(dacp != NULL);

  osalMutexUnlock(&dacp->mutex);
}
#endif /* DAC_USE_MUTUAL_EXCLUSION == TRUE */

#endif /* HAL_USE_DAC == TRUE */

/** @} */
