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
 * @file    TIMv1/hal_icu_lld.h
 * @brief   AVR/MEGA ICU subsystem low level driver header.
 *
 * @addtogroup ICU
 * @{
 */

#ifndef HAL_ICU_LLD_H
#define HAL_ICU_LLD_H

#if HAL_USE_ICU || defined(__DOXYGEN__)

#include "avr_timers.h"

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   ICU driver enable switch.
 * @details If set to @p TRUE the support for ICU1 is included.
 */
#if !defined(AVR_ICU_USE_TIM1) || defined(__DOXYGEN__)
#define AVR_ICU_USE_TIM1               FALSE
#endif
/**
 * @brief   ICU driver enable switch.
 * @details If set to @p TRUE the support for ICU3 is included.
 */
#if !defined(AVR_ICU_USE_TIM3) || defined(__DOXYGEN__)
#define AVR_ICU_USE_TIM3               FALSE
#endif
/**
 * @brief   ICU driver enable switch.
 * @details If set to @p TRUE the support for ICU4 is included.
 */
#if !defined(AVR_ICU_USE_TIM4) || defined(__DOXYGEN__)
#define AVR_ICU_USE_TIM4               FALSE
#endif
/**
 * @brief   ICU driver enable switch.
 * @details If set to @p TRUE the support for ICU5 is included.
 */
#if !defined(AVR_ICU_USE_TIM5) || defined(__DOXYGEN__)
#define AVR_ICU_USE_TIM5               FALSE
#endif
/** @} */

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @brief ICU driver mode.
 */
typedef enum {
  ICU_INPUT_ACTIVE_HIGH = 0,        /**< Trigger on rising edge.            */
  ICU_INPUT_ACTIVE_LOW = 1,         /**< Trigger on falling edge.           */
} icumode_t;

/**
 * @brief   ICU frequency type.
 */
typedef uint16_t icufreq_t;

/**
 * @brief   ICU counter type.
 */
typedef uint16_t icucnt_t;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Driver mode.
   */
  icumode_t                 mode;
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  icufreq_t                 frequency;
  /**
   * @brief   Callback for pulse width measurement.
   */
  icucallback_t             width_cb;
  /**
   * @brief   Callback for cycle period measurement.
   */
  icucallback_t             period_cb;
  /**
   * @brief   Callback for timer overflow.
   */
  icucallback_t             overflow_cb;
  /* End of the mandatory fields. */
} ICUConfig;

/**
 * @brief   Structure representing an ICU driver.
 */
struct ICUDriver {
  /**
   * @brief Driver state.
   */
  icustate_t                state;
  /**
   * @brief Current configuration data.
   */
  const ICUConfig           *config;
#if defined(ICU_DRIVER_EXT_FIELDS)
  ICU_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields. */
  /**
   * @brief Width value read by ISR.
   */
  icucnt_t                  width;
  /**
   * @brief Period value read by ISR.
   */
  icucnt_t                  period;
};

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/**
 * @brief   Returns the width of the latest pulse.
 * @details The pulse width is defined as number of ticks between the start
 *          edge and the stop edge.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The number of ticks.
 *
 * @notapi
 */
/* #define icu_lld_get_width(icup) 1 */

/**
 * @brief   Returns the width of the latest cycle.
 * @details The cycle width is defined as number of ticks between a start
 *          edge and the next start edge.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The number of ticks.
 *
 * @notapi
 */
/* #define icu_lld_get_period(icup) 1 */

/**
 * @brief   Check on notifications status.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The notifications status.
 * @retval false        if notifications are not enabled.
 * @retval true         if notifications are enabled.
 *
 * @notapi
 */
/* TODO: The following macros must be implement. */
#define icu_lld_are_notifications_enabled(icup)                             \
  (bool)(FALSE)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if AVR_ICU_USE_TIM1 && !defined(__DOXYGEN__)
extern ICUDriver ICUD1;
#endif
#if AVR_ICU_USE_TIM3 && !defined(__DOXYGEN__)
extern ICUDriver ICUD3;
#endif
#if AVR_ICU_USE_TIM4 && !defined(__DOXYGEN__)
extern ICUDriver ICUD4;
#endif
#if AVR_ICU_USE_TIM5 && !defined(__DOXYGEN__)
extern ICUDriver ICUD5;
#endif

#ifdef __cplusplus
extern "C" {
#endif

  void icu_lld_init(void);
  void icu_lld_start(ICUDriver *icup);
  void icu_lld_stop(ICUDriver *icup);
  void icu_lld_start_capture(ICUDriver *icup);
  void icu_lld_stop_capture(ICUDriver *icup);
  bool icu_lld_wait_capture(ICUDriver *icup);           /* TODO: Implement. */
  void icu_lld_enable_notifications(ICUDriver *icup);   /* TODO: Implement. */
  void icu_lld_disable_notifications(ICUDriver *icup);  /* TODO: Implement. */
  icucnt_t icu_lld_get_width(ICUDriver *icup);
  icucnt_t icu_lld_get_period(ICUDriver *icup);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ICU */

#endif /* HAL_ICU_LLD_H */

/** @} */
