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
 * @file    hal_adc.h
 * @brief   ADC Driver macros and structures.
 *
 * @addtogroup ADC
 * @{
 */

#ifndef HAL_ADC_H
#define HAL_ADC_H

#if (HAL_USE_ADC == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    ADC configuration options
 * @{
 */
/**
 * @brief   Enables synchronous APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(ADC_USE_WAIT) || defined(__DOXYGEN__)
#define ADC_USE_WAIT                TRUE
#endif

/**
 * @brief   Enables the @p adcAcquireBus() and @p adcReleaseBus() APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(ADC_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define ADC_USE_MUTUAL_EXCLUSION    TRUE
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
  ADC_UNINIT = 0,                           /**< Not initialized.           */
  ADC_STOP = 1,                             /**< Stopped.                   */
  ADC_READY = 2,                            /**< Ready.                     */
  ADC_ACTIVE = 3,                           /**< Converting.                */
  ADC_COMPLETE = 4,                         /**< Conversion complete.       */
  ADC_ERROR = 5                             /**< Conversion error.          */
} adcstate_t;

/**
 * @brief   Type of a structure representing an ADC driver.
 */
typedef struct hal_adc_driver ADCDriver;

/**
 * @brief   Type of a structure representing an ADC driver configuration.
 */
typedef struct hal_adc_config ADCConfig;

/**
 * @brief   Conversion group configuration structure.
 * @details This implementation-dependent structure describes a conversion
 *          operation.
 * @note    The use of this configuration structure requires knowledge of
 *          STM32 ADC cell registers interface, please refer to the STM32
 *          reference manual for details.
 */
typedef struct hal_adc_configuration_group ADCConversionGroup;

/* Including the low level driver header, it exports information required
   for completing types.*/
#include "hal_adc_lld.h"

/**
 * @brief   Type of an ADC notification callback.
 * @details The callback is invoked from ISR context.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object triggering the
 *                      callback
 */
typedef void (*adccallback_t)(ADCDriver *adcp);

/**
 * @brief   Type of an ADC error callback.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object triggering the
 *                      callback
 * @param[in] err       ADC error code
 */
typedef void (*adcerrorcallback_t)(ADCDriver *adcp, adcerror_t err);

/**
 * @brief   Conversion group configuration structure.
 * @details This implementation-dependent structure describes a conversion
 *          operation.
 * @note    The use of this configuration structure requires knowledge of
 *          STM32 ADC cell registers interface, please refer to the STM32
 *          reference manual for details.
 */
struct hal_adc_configuration_group {
  /**
   * @brief   Enables the circular buffer mode for the group.
   */
  bool                      circular;
  /**
   * @brief   Number of the analog channels belonging to the conversion group.
   */
  adc_channels_num_t        num_channels;
  /**
   * @brief   Callback function associated to the group or @p NULL.
   * @details In linear mode the callback is invoked after the driver returns
   *          to the @p ADC_READY state, this allows chaining another
   *          conversion using I-Class APIs. In circular mode the callback is
   *          invoked in either @p ADC_ACTIVE state (half buffer) or
   *          @p ADC_COMPLETE state (full buffer).
   * @note    If a synchronous API is waiting for completion then starting a
   *          follow-on conversion from this callback is undefined.
   */
  adccallback_t             end_cb;
  /**
   * @brief   Error callback or @p NULL.
   */
  adcerrorcallback_t        error_cb;
  /* End of the mandatory fields.*/
  adc_lld_configuration_group_fields
};

/**
 * @brief   Driver configuration structure.
 */
struct hal_adc_config {
  /* End of the mandatory fields.*/
  adc_lld_config_fields
};

/**
 * @brief   Structure representing an ADC driver.
 */
struct hal_adc_driver {
  /**
   * @brief Driver state.
   */
  adcstate_t                state;
  /**
   * @brief Current configuration data.
   */
  const ADCConfig           *config;
  /**
   * @brief Current samples buffer pointer or @p NULL.
   */
  adcsample_t               *samples;
  /**
   * @brief Current samples buffer depth or @p 0.
   */
  size_t                    depth;
  /**
   * @brief Current conversion group pointer or @p NULL.
   */
  const ADCConversionGroup  *grpp;
#if (ADC_USE_WAIT == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief Waiting thread.
   */
  thread_reference_t        thread;
#endif /* ADC_USE_WAIT == TRUE */
#if (ADC_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief Mutex protecting the peripheral.
   */
  mutex_t                   mutex;
#endif /* ADC_USE_MUTUAL_EXCLUSION == TRUE */
#if defined(ADC_DRIVER_EXT_FIELDS)
  ADC_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  adc_lld_driver_fields
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Buffer state.
 * @note    This function is meant to be called from the ADC callback only.
 * @note    This state is only meaningful for circular conversions, where it
 *          is used to distinguish the full buffer callback from the half
 *          buffer callback.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @return              The buffer state.
 * @retval              false if the driver filled/sent the first half of the
 *                      buffer.
 * @retval              true if the driver filled/sent the second half of the
 *                      buffer.
 *
 * @special
 */
#define adcIsBufferComplete(adcp) ((bool)((adcp)->state == ADC_COMPLETE))
/** @} */

/**
 * @name    Low level driver helper macros
 * @{
 */
#if (ADC_USE_WAIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Resumes a thread waiting for a conversion completion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
#define _adc_reset_i(adcp)                                                  \
  osalThreadResumeI(&(adcp)->thread, MSG_RESET)

/**
 * @brief   Resumes a thread waiting for a conversion completion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
#define _adc_reset_s(adcp)                                                  \
  osalThreadResumeS(&(adcp)->thread, MSG_RESET)

/**
 * @brief   Wakes up the waiting thread.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
#define _adc_wakeup_isr(adcp) {                                             \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(adcp)->thread, MSG_OK);                               \
  osalSysUnlockFromISR();                                                   \
}

/**
 * @brief   Wakes up the waiting thread with a timeout message.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
#define _adc_timeout_isr(adcp) {                                            \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(adcp)->thread, MSG_TIMEOUT);                          \
  osalSysUnlockFromISR();                                                   \
}

#else /* !ADC_USE_WAIT */
#define _adc_reset_i(adcp)
#define _adc_reset_s(adcp)
#define _adc_wakeup_isr(adcp)
#define _adc_timeout_isr(adcp)
#endif /* !ADC_USE_WAIT */

/**
 * @brief   Common ISR code, half buffer event.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
#define _adc_isr_half_code(adcp) {                                          \
  if ((adcp)->grpp->end_cb != NULL) {                                       \
    (adcp)->grpp->end_cb(adcp);                                             \
  }                                                                         \
}

/**
 * @brief   Common ISR code, full buffer event.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
#define _adc_isr_full_code(adcp) {                                          \
  if ((adcp)->grpp->circular) {                                             \
    /* Callback handling.*/                                                 \
    if ((adcp)->grpp->end_cb != NULL) {                                     \
      (adcp)->state = ADC_COMPLETE;                                         \
      (adcp)->grpp->end_cb(adcp);                                           \
      if ((adcp)->state == ADC_COMPLETE) {                                  \
        (adcp)->state = ADC_ACTIVE;                                         \
      }                                                                     \
    }                                                                       \
  }                                                                         \
  else {                                                                    \
    /* End conversion.*/                                                    \
    const ADCConversionGroup *grpp = (adcp)->grpp;                          \
    adc_lld_stop_conversion(adcp);                                          \
    if ((adcp)->grpp->end_cb != NULL) {                                     \
      (adcp)->state = ADC_READY;                                            \
      (adcp)->grpp->end_cb(adcp);                                           \
      if (((adcp)->state == ADC_READY) && ((adcp)->grpp == grpp)) {         \
        (adcp)->grpp = NULL;                                                \
      }                                                                     \
    }                                                                       \
    else {                                                                  \
      (adcp)->state = ADC_READY;                                            \
      (adcp)->grpp = NULL;                                                  \
    }                                                                       \
    _adc_wakeup_isr(adcp);                                                  \
  }                                                                         \
}

/**
 * @brief   Common ISR code, error event.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread timeout signaling, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @param[in] err       platform dependent error code
 *
 * @notapi
 */
#define _adc_isr_error_code(adcp, err) {                                    \
  adc_lld_stop_conversion(adcp);                                            \
  if ((adcp)->grpp->error_cb != NULL) {                                     \
    (adcp)->state = ADC_ERROR;                                              \
    (adcp)->grpp->error_cb(adcp, err);                                      \
    if ((adcp)->state == ADC_ERROR) {                                       \
      (adcp)->state = ADC_READY;                                            \
      (adcp)->grpp = NULL;                                                  \
    }                                                                       \
  }                                                                         \
  else {                                                                    \
    (adcp)->state = ADC_READY;                                              \
    (adcp)->grpp = NULL;                                                    \
  }                                                                         \
  _adc_timeout_isr(adcp);                                                   \
}
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void adcInit(void);
  void adcObjectInit(ADCDriver *adcp);
  msg_t adcStart(ADCDriver *adcp, const ADCConfig *config);
  void adcStop(ADCDriver *adcp);
  void adcStartConversion(ADCDriver *adcp,
                          const ADCConversionGroup *grpp,
                          adcsample_t *samples,
                          size_t depth);
  void adcStartConversionI(ADCDriver *adcp,
                           const ADCConversionGroup *grpp,
                           adcsample_t *samples,
                           size_t depth);
  void adcStopConversion(ADCDriver *adcp);
  void adcStopConversionI(ADCDriver *adcp);
#if ADC_USE_WAIT == TRUE
  msg_t adcConvert(ADCDriver *adcp,
                   const ADCConversionGroup *grpp,
                   adcsample_t *samples,
                   size_t depth);
#endif
#if ADC_USE_MUTUAL_EXCLUSION == TRUE
  void adcAcquireBus(ADCDriver *adcp);
  void adcReleaseBus(ADCDriver *adcp);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ADC == TRUE */

#endif /* HAL_ADC_H */

/** @} */
