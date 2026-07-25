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

/*
   This driver is based on the work done by Matteo Serva available at
   http://github.com/matteoserva/ChibiOS-AVR
*/

/**
 * @file    TIMv1/hal_gpt_lld.h
 * @brief   AVR/MEGA GPT subsystem low level driver header.
 *
 * @addtogroup GPT
 * @{
 */

#ifndef HAL_GPT_LLD_H
#define HAL_GPT_LLD_H

#if HAL_USE_GPT || defined(__DOXYGEN__)

#include "avr_timers.h"

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/**
 * @brief     GPT1 driver enable switch.
 * @details   If set to @p TRUE the support for GPT1 is included.
 * @note      The default is @p FALSE.
 */
#if !defined(AVR_GPT_USE_TIM1)
#define AVR_GPT_USE_TIM1              FALSE
#endif

/**
 * @brief     GPT2 driver enable switch.
 * @details   If set to @p TRUE the support for GPT2 is included.
 * @note      The default is @p FALSE.
 */
#if !defined(AVR_GPT_USE_TIM2)
#define AVR_GPT_USE_TIM2              FALSE
#endif

/**
 * @brief     GPT3 driver enable switch.
 * @details   If set to @p TRUE the support for GPT3 is included.
 * @note      The default is @p FALSE.
 */
#if !defined(AVR_GPT_USE_TIM3)
#define AVR_GPT_USE_TIM3              FALSE
#endif

/**
 * @brief     GPT4 driver enable switch.
 * @details   If set to @p TRUE the support for GPT4 is included.
 * @note      The default is @p FALSE.
 */
#if !defined(AVR_GPT_USE_TIM4)
#define AVR_GPT_USE_TIM4              FALSE
#endif

/**
 * @brief     GPT5 driver enable switch.
 * @details   If set to @p TRUE the support for GPT5 is included.
 * @note      The default is @p FALSE.
 */
#if !defined(AVR_GPT_USE_TIM5)
#define AVR_GPT_USE_TIM5              FALSE
#endif

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @brief   GPT frequency type.
 */
typedef uint32_t gptfreq_t;

/**
 * @brief   GPT counter type.
 */
typedef uint16_t gptcnt_t;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  gptfreq_t                 frequency;
  /**
   * @brief   Timer callback pointer.
   * @note    This callback is invoked on GPT counter events.
   */
  gptcallback_t             callback;
  /* End of the mandatory fields. */
} GPTConfig;

/**
 * @brief   Structure representing a GPT driver.
 */
struct GPTDriver {
  /**
   * @brief Driver state.
   */
  volatile gptstate_t       state;
  /**
   * @brief Current configuration data.
   */
  const GPTConfig           *config;

#if defined(GPT_DRIVER_EXT_FIELDS)
  GPT_DRIVER_EXT_FIELDS
#endif

  /* End of the mandatory fields. */
  /**
   * @brief input clock from prescaler.
   */
  uint8_t                   clock_source;
  /**
   * @brief Lenght of the period in clock ticks.
   */
  gptcnt_t                  period;
  /**
   * @brief Current clock tick.
   */
  gptcnt_t                  counter;
  /**
   * @brief Function called from the interrupt service routine.
   */
  gptcallback_t             callback;
};

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/**
 * @brief     Changes the interval of GPT peripheral.
 * @details   This function changes the interval of a running GPT unit.
 * @pre       The GPT unit must have been activated using @p gptStart().
 * @pre       The GPT unit must have been running in continuous mode using
 *            @p gptStartContinuous().
 * @post      The GPT unit interval is changed to the new value.
 * @note      The function has effect at the next cycle start.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @param[in] interval  new cycle time in timer ticks
 * @notapi
 */

/* FIXME: placeholder to enable compile, should be implemented! */
#define gpt_lld_change_interval(gptp, interval)

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#if AVR_GPT_USE_TIM1 || defined(__DOXYGEN__)
extern GPTDriver GPTD1;
#endif
#if AVR_GPT_USE_TIM2 || defined(__DOXYGEN__)
extern GPTDriver GPTD2;
#endif
#if AVR_GPT_USE_TIM3 || defined(__DOXYGEN__)
extern GPTDriver GPTD3;
#endif
#if AVR_GPT_USE_TIM4 || defined(__DOXYGEN__)
extern GPTDriver GPTD4;
#endif
#if AVR_GPT_USE_TIM5 || defined(__DOXYGEN__)
extern GPTDriver GPTD5;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void gpt_lld_init(void);
  void gpt_lld_start(GPTDriver *gptp);
  void gpt_lld_stop(GPTDriver *gptp);
  void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t interval);
  void gpt_lld_stop_timer(GPTDriver *gptp);
  void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_GPT */

#endif /* HAL_GPT_LLD_H */

/** @} */
