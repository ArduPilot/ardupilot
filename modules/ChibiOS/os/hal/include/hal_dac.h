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
 * @file    hal_dac.h
 * @brief   DAC Driver macros and structures.
 *
 * @addtogroup DAC
 * @{
 */

#ifndef HAL_DAC_H
#define HAL_DAC_H

#if (HAL_USE_DAC == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    DAC configuration options
 * @{
 */
/**
 * @brief   Support for thread synchronization API.
 */
#if !defined(DAC_USE_SYNCHRONIZATION) || defined(__DOXYGEN__)
#if !defined(DAC_USE_WAIT) || defined(__DOXYGEN__)
#define DAC_USE_SYNCHRONIZATION             FALSE
#else
#define DAC_USE_SYNCHRONIZATION             DAC_USE_WAIT
#endif
#endif

/**
 * @brief   Enables the @p dacAcquireBus() and @p dacReleaseBus() APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(DAC_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define DAC_USE_MUTUAL_EXCLUSION    TRUE
#endif
/** @} */

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
  DAC_UNINIT = 0,                   /**< Not initialized.                   */
  DAC_STOP = 1,                     /**< Stopped.                           */
  DAC_READY = 2,                    /**< Ready.                             */
  DAC_ACTIVE = 3,                   /**< Exchanging data.                   */
  DAC_COMPLETE = 4,                 /**< Circular full buffer callback.     */
  DAC_ERROR = 5                     /**< Error.                             */
} dacstate_t;

/**
 * @brief   Type of a structure representing an DAC driver.
 */
typedef struct hal_dac_driver DACDriver;

/**
 * @brief   Type of a structure representing an DAC driver configuration.
 */
typedef struct hal_dac_config DACConfig;

/**
 * @brief   Type of a DAC conversion group.
 */
typedef struct hal_dac_conversion_group DACConversionGroup;

/* Including the low level driver header, it exports information required
   for completing types.*/
#include "hal_dac_lld.h"

/**
 * @brief   DAC notification callback type.
 *
 * @param[in] dacp      pointer to the @p DACDriver object triggering the
 */
typedef void (*daccallback_t)(DACDriver *dacp);

/**
 * @brief   DAC error callback type.
 *
 * @param[in] dacp      pointer to the @p DACDriver object triggering the
 *                      callback
 * @param[in] err       DAC error code
 */
typedef void (*dacerrorcallback_t)(DACDriver *dacp, dacerror_t err);

/**
 * @brief   DAC Conversion group structure.
 */
struct hal_dac_conversion_group {
  /**
   * @brief   Number of DAC channels.
   */
  uint32_t                  num_channels;
  /**
   * @brief   Operation complete callback or @p NULL.
   * @note    This callback is invoked from ISR context on half buffer and
   *          full buffer events during the ongoing circular conversion. The
   *          driver state is @p DAC_ACTIVE on half buffer callbacks and
   *          @p DAC_COMPLETE on full buffer callbacks. Starting a new
   *          conversion from this callback is not supported.
   */
  daccallback_t             end_cb;
  /**
   * @brief   Error handling callback or @p NULL.
   */
  dacerrorcallback_t        error_cb;
  /* End of the mandatory fields.*/
  dac_lld_conversion_group_fields
};

/**
 * @brief   Driver configuration structure.
 */
struct hal_dac_config {
  /* End of the mandatory fields.*/
  dac_lld_config_fields
};

/**
 * @brief   Structure representing a DAC driver.
 */
struct hal_dac_driver {
  /**
   * @brief   Driver state.
   */
  dacstate_t                state;
  /**
   * @brief   Conversion group.
   */
  const DACConversionGroup  *grpp;
  /**
   * @brief   Samples buffer pointer.
   */
  dacsample_t               *samples;
  /**
   * @brief   Samples buffer size.
   */
  size_t                    depth;
  /**
   * @brief   Current configuration data.
   */
  const DACConfig           *config;
#if (DAC_USE_SYNCHRONIZATION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Waiting thread.
   */
  thread_reference_t        thread;
#endif /* DAC_USE_SYNCHRONIZATION */
#if (DAC_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the bus.
   */
  mutex_t                   mutex;
#endif /* DAC_USE_MUTUAL_EXCLUSION */
#if defined(DAC_DRIVER_EXT_FIELDS)
  DAC_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  dac_lld_driver_fields
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Low level driver helper macros
 * @{
 */
/**
 * @brief   Buffer state.
 * @note    This function is meant to be called from the DAC callback only.
 * @note    This function is meaningful for circular conversion callbacks
 *          only.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @return              The buffer state.
 * @retval              false if the driver filled/sent the first half of the
 *                      buffer.
 * @retval              true if the driver filled/sent the second half of the
 *                      buffer.
 *
 * @special
 */
#define dacIsBufferComplete(dacp) ((bool)((dacp)->state == DAC_COMPLETE))

#if (DAC_USE_SYNCHRONIZATION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Waits for operation completion.
 * @details This function waits for the driver to complete the current
 *          conversion cycle.
 * @pre     An operation must be running while the function is invoked.
 * @note    No more than one thread can wait on a DAC driver using
 *          this function.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
#define _dac_wait_s(dacp) osalThreadSuspendS(&(dacp)->thread)

/**
 * @brief   Resumes a thread waiting for a conversion completion.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
#define _dac_reset_i(dacp) osalThreadResumeI(&(dacp)->thread, MSG_RESET)

/**
 * @brief   Resumes a thread waiting for a conversion completion.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
#define _dac_reset_s(dacp) osalThreadResumeS(&(dacp)->thread, MSG_RESET)

/**
 * @brief   Wakes up the waiting thread.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
#define _dac_wakeup_isr(dacp) {                                             \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(dacp)->thread, MSG_OK);                               \
  osalSysUnlockFromISR();                                                   \
}

/**
 * @brief   Wakes up the waiting thread with a timeout message.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
#define _dac_timeout_isr(dacp) {                                            \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(dacp)->thread, MSG_TIMEOUT);                          \
  osalSysUnlockFromISR();                                                   \
}

#else /* !DAC_USE_SYNCHRONIZATION */
#define _dac_wait_s(dacp)
#define _dac_reset_i(dacp)
#define _dac_reset_s(dacp)
#define _dac_wakeup_isr(dacp)
#define _dac_timeout_isr(dacp)
#endif /* !DAC_USE_SYNCHRONIZATION */

/**
 * @brief   Common ISR code, half buffer event.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
#define _dac_isr_half_code(dacp) {                                          \
  if ((dacp)->grpp->end_cb != NULL) {                                       \
    (dacp)->grpp->end_cb(dacp);                                             \
  }                                                                         \
}

/**
 * @brief   Common ISR code, full buffer event.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Driver state transitions.
 * @details The @p DAC_COMPLETE state is used only as a transient full buffer
 *          callback marker during the ongoing circular conversion.
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
#define _dac_isr_full_code(dacp) {                                          \
  if ((dacp)->grpp->end_cb) {                                               \
    (dacp)->state = DAC_COMPLETE;                                           \
    (dacp)->grpp->end_cb(dacp);                                             \
    if ((dacp)->state == DAC_COMPLETE)                                      \
      (dacp)->state = DAC_ACTIVE;                                           \
  }                                                                         \
  _dac_wakeup_isr(dacp);                                                    \
}

/**
 * @brief   Common ISR code, error event.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread timeout signalling, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] err       platform dependent error code
 *
 * @notapi
 */
#define _dac_isr_error_code(dacp, err) {                                    \
  dac_lld_stop_conversion(dacp);                                            \
  if ((dacp)->grpp->error_cb != NULL) {                                     \
    (dacp)->state = DAC_ERROR;                                              \
    (dacp)->grpp->error_cb(dacp, err);                                      \
    if ((dacp)->state == DAC_ERROR)                                         \
      (dacp)->state = DAC_READY;                                            \
  }                                                                         \
  (dacp)->grpp = NULL;                                                      \
  _dac_timeout_isr(dacp);                                                   \
}
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void  dacInit(void);
  void  dacObjectInit(DACDriver *dacp);
  msg_t dacStart(DACDriver *dacp, const DACConfig *config);
  void  dacStop(DACDriver *dacp);
  msg_t dacPutChannelX(DACDriver *dacp,
                      dacchannel_t channel,
                      dacsample_t sample);
  msg_t dacStartConversion(DACDriver *dacp, const DACConversionGroup *grpp,
                          dacsample_t *samples, size_t depth);
  msg_t dacStartConversionI(DACDriver *dacp, const DACConversionGroup *grpp,
                           dacsample_t *samples, size_t depth);
  void  dacStopConversion(DACDriver *dacp);
  void  dacStopConversionI(DACDriver *dacp);
#if DAC_USE_SYNCHRONIZATION
  msg_t dacConvert(DACDriver *dacp, const DACConversionGroup *grpp,
                   dacsample_t *samples, size_t depth);
  msg_t dacSynchronizeS(DACDriver *dacp, sysinterval_t timeout);
  msg_t dacSynchronize(DACDriver *dacp, sysinterval_t timeout);
#endif /* DAC_USE_SYNCHRONIZATION */
#if DAC_USE_MUTUAL_EXCLUSION
  void  dacAcquireBus(DACDriver *dacp);
  void  dacReleaseBus(DACDriver *dacp);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_DAC == TRUE */

#endif /* HAL_DAC_H */

/** @} */
