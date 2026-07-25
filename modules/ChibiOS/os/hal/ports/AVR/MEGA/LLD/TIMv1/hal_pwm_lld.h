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
 * @file    TIMv1/hal_pwm_lld.h
 * @brief   AVR/MEGA PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#ifndef HAL_PWM_LLD_H
#define HAL_PWM_LLD_H

#if HAL_USE_PWM || defined(__DOXYGEN__)

#include "avr_timers.h"

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

#if !defined(AVR_PWM_USE_TIM1)
#define AVR_PWM_USE_TIM1 FALSE
#endif
#if !defined(AVR_PWM_USE_TIM2)
#define AVR_PWM_USE_TIM2 FALSE
#endif
#if !defined(AVR_PWM_USE_TIM3)
#define AVR_PWM_USE_TIM3 FALSE
#endif
#if !defined(AVR_PWM_USE_TIM4)
#define AVR_PWM_USE_TIM4 FALSE
#endif
#if !defined(AVR_PWM_USE_TIM5)
#define AVR_PWM_USE_TIM5 FALSE
#endif

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/**
 * @brief   Number of PWM channels per PWM driver.
 */
#if !defined(PWM_CHANNELS) || defined(__DOXYGEN__)
  #if defined(TIMER1_COMPC_vect)
    #define PWM_CHANNELS        3
  #else
    #define PWM_CHANNELS        2
  #endif
#endif

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @brief PWM mode type.
 */
typedef uint8_t pwmmode_t;

/**
 * @brief   PWM channel type.
 */
typedef uint8_t pwmchannel_t;

/**
 * @brief   Type of a channels mask.
 */
typedef uint8_t pwmchnmsk_t;

/**
 * @brief   PWM counter type.
 */
typedef uint16_t pwmcnt_t;

/**
 * @brief   PWM driver channel configuration structure.
 * @note    Some architectures may not be able to support the channel mode
 *          or the callback, in this case the fields are ignored.
 */
typedef struct {
  /**
   * @brief Channel active logic level.
   */
  pwmmode_t                 mode;
  /**
   * @brief Channel callback pointer.
   * @note  This callback is invoked on the channel compare event. If set to
   *        @p NULL then the callback is disabled.
   */
  pwmcallback_t             callback;
  /* End of the mandatory fields. */
} PWMChannelConfig;

/**
 * @brief   Driver configuration structure.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
typedef struct {
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  uint32_t                  frequency;
  /**
   * @brief   PWM period in ticks.
   * @note    The low level can use assertions in order to catch invalid
   *          period specifications.
   */
  pwmcnt_t                  period;
  /**
   * @brief Periodic callback pointer.
   * @note  This callback is invoked on PWM counter reset. If set to
   *        @p NULL then the callback is disabled.
   */
  pwmcallback_t             callback;
  /**
   * @brief Channels configurations.
   */
  PWMChannelConfig          channels[PWM_CHANNELS];
  /* End of the mandatory fields. */
} PWMConfig;

/**
 * @brief   Structure representing an PWM driver.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
struct PWMDriver {
  /**
   * @brief Driver state.
   */
  pwmstate_t                state;
  /**
   * @brief Current configuration data.
   */
  const PWMConfig           *config;
  /**
   * @brief   Current PWM period in ticks.
   */
  pwmcnt_t                  period;
  /**
   * @brief   Mask of the enabled channels.
   */
  pwmchnmsk_t               enabled;
  /**
   * @brief   Number of channels in this instance.
   */
  pwmchannel_t              channels;
#if defined(PWM_DRIVER_EXT_FIELDS)
  PWM_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields. */
};

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#if AVR_PWM_USE_TIM1 || defined(__DOXYGEN__)
extern PWMDriver PWMD1;
#endif
#if AVR_PWM_USE_TIM2 || defined(__DOXYGEN__)
extern PWMDriver PWMD2;
#endif
#if AVR_PWM_USE_TIM3 || defined(__DOXYGEN__)
extern PWMDriver PWMD3;
#endif
#if AVR_PWM_USE_TIM4 || defined(__DOXYGEN__)
extern PWMDriver PWMD4;
#endif
#if AVR_PWM_USE_TIM5 || defined(__DOXYGEN__)
extern PWMDriver PWMD5;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void pwm_lld_init(void);
  void pwm_lld_start(PWMDriver *pwmp);
  void pwm_lld_stop(PWMDriver *pwmp);
  void pwm_lld_change_period(PWMDriver *pwmp, pwmcnt_t period);
  void pwm_lld_enable_channel(PWMDriver *pwmp,
                              pwmchannel_t channel,
                              pwmcnt_t width);
  void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel);
  void pwm_lld_enable_periodic_notification(PWMDriver *pwmp);
  void pwm_lld_disable_periodic_notification(PWMDriver *pwmp);
  void pwm_lld_enable_channel_notification(PWMDriver *pwmp,
                                           pwmchannel_t channel);
  void pwm_lld_disable_channel_notification(PWMDriver *pwmp,
                                            pwmchannel_t channel);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PWM */

#endif /* HAL_PWM_LLD_H */

/** @} */
