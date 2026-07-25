/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

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
   Rewritten by Emil Fresk (1/5 - 2014) for extended input capture
   functionality. And fix for spurious callbacks in the interrupt handler.
*/
/*
   Improved by Uladzimir Pylinsky aka barthess (1/3 - 2015) for support of
   32-bit timers and timers with single capture/compare channels.
*/

#ifndef HAL_EICU_LLD_H
#define HAL_EICU_LLD_H

#include "stm32_tim.h"

#if (HAL_USE_EICU == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   EICUD1 driver enable switch.
 * @details If set to @p TRUE the support for EICUD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM1) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM1                  FALSE
#endif

/**
 * @brief   EICUD2 driver enable switch.
 * @details If set to @p TRUE the support for EICUD2 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM2) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM2                  FALSE
#endif

/**
 * @brief   EICUD3 driver enable switch.
 * @details If set to @p TRUE the support for EICUD3 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM3) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM3                  FALSE
#endif

/**
 * @brief   EICUD4 driver enable switch.
 * @details If set to @p TRUE the support for EICUD4 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM4) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM4                  FALSE
#endif

/**
 * @brief   EICUD5 driver enable switch.
 * @details If set to @p TRUE the support for EICUD5 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM5) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM5                  FALSE
#endif

/**
 * @brief   EICUD8 driver enable switch.
 * @details If set to @p TRUE the support for EICUD8 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM8) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM8                  FALSE
#endif

/**
 * @brief   EICUD9 driver enable switch.
 * @details If set to @p TRUE the support for EICUD9 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM9) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM9                  FALSE
#endif

/**
 * @brief   EICUD12 driver enable switch.
 * @details If set to @p TRUE the support for EICUD12 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM12) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM12                 FALSE
#endif

/**
 * @brief   EICUD10 driver enable switch.
 * @details If set to @p TRUE the support for EICUD10 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM10) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM10                 FALSE
#endif

/**
 * @brief   EICUD11 driver enable switch.
 * @details If set to @p TRUE the support for EICUD11 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM11) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM11                 FALSE
#endif

/**
 * @brief   EICUD13 driver enable switch.
 * @details If set to @p TRUE the support for EICUD13 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM13) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM13                 FALSE
#endif

/**
 * @brief   EICUD14 driver enable switch.
 * @details If set to @p TRUE the support for EICUD14 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM14) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM14                 FALSE
#endif

/**
 * @brief   EICUD1 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM1_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD2 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM2_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD3 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM3_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD4 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM4_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD5 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM5_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM5_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD8 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM8_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM8_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD9 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM9_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM9_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD12 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM12_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM12_IRQ_PRIORITY        7
#endif

/**
 * @brief   EICUD10 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM10_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM10_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD11 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM11_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM11_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD13 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM13_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM13_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD14 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM14_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM14_IRQ_PRIORITY         7
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_EICU_USE_TIM1 && !STM32_HAS_TIM1
#error "TIM1 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM2 && !STM32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM3 && !STM32_HAS_TIM3
#error "TIM3 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM4 && !STM32_HAS_TIM4
#error "TIM4 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM5 && !STM32_HAS_TIM5
#error "TIM5 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM8 && !STM32_HAS_TIM8
#error "TIM8 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM9 && !STM32_HAS_TIM9
#error "TIM9 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM12 && !STM32_HAS_TIM12
#error "TIM12 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM10 && !STM32_HAS_TIM10
#error "TIM10 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM11 && !STM32_HAS_TIM11
#error "TIM11 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM13 && !STM32_HAS_TIM13
#error "TIM13 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM14 && !STM32_HAS_TIM14
#error "TIM14 not present in the selected device"
#endif

#if !STM32_EICU_USE_TIM1  && !STM32_EICU_USE_TIM2  &&                         \
    !STM32_EICU_USE_TIM3  && !STM32_EICU_USE_TIM4  &&                         \
    !STM32_EICU_USE_TIM5  && !STM32_EICU_USE_TIM8  &&                         \
    !STM32_EICU_USE_TIM9  && !STM32_EICU_USE_TIM12 &&                         \
    !STM32_EICU_USE_TIM10 && !STM32_EICU_USE_TIM11 &&                         \
    !STM32_EICU_USE_TIM13 && !STM32_EICU_USE_TIM14
#error "EICU driver activated but no TIM peripheral assigned"
#endif

/* Checks on allocation of TIMx units.*/
#if STM32_EICU_USE_TIM1
#if defined(STM32_TIM1_IS_USED)
#error "EICUD1 requires TIM1 but the timer is already used"
#else
#define STM32_TIM1_IS_USED
#endif
#endif

#if STM32_EICU_USE_TIM2
#if defined(STM32_TIM2_IS_USED)
#error "EICUD2 requires TIM2 but the timer is already used"
#else
#define STM32_TIM2_IS_USED
#endif
#endif

#if STM32_EICU_USE_TIM3
#if defined(STM32_TIM3_IS_USED)
#error "EICUD3 requires TIM3 but the timer is already used"
#else
#define STM32_TIM3_IS_USED
#endif
#endif

#if STM32_EICU_USE_TIM4
#if defined(STM32_TIM4_IS_USED)
#error "EICUD4 requires TIM4 but the timer is already used"
#else
#define STM32_TIM4_IS_USED
#endif
#endif

#if STM32_EICU_USE_TIM5
#if defined(STM32_TIM5_IS_USED)
#error "EICUD5 requires TIM5 but the timer is already used"
#else
#define STM32_TIM5_IS_USED
#endif
#endif

#if STM32_EICU_USE_TIM8
#if defined(STM32_TIM8_IS_USED)
#error "EICUD8 requires TIM8 but the timer is already used"
#else
#define STM32_TIM8_IS_USED
#endif
#endif

#if STM32_EICU_USE_TIM9
#if defined(STM32_TIM9_IS_USED)
#error "EICUD9 requires TIM9 but the timer is already used"
#else
#define STM32_TIM9_IS_USED
#endif
#endif

#if STM32_EICU_USE_TIM10
#if defined(STM32_TIM10_IS_USED)
#error "EICUD10 requires TIM10 but the timer is already used"
#else
#define STM32_TIM10_IS_USED
#endif
#endif

#if STM32_EICU_USE_TIM11
#if defined(STM32_TIM11_IS_USED)
#error "EICUD11 requires TIM11 but the timer is already used"
#else
#define STM32_TIM11_IS_USED
#endif
#endif

#if STM32_EICU_USE_TIM12
#if defined(STM32_TIM12_IS_USED)
#error "EICUD12 requires TIM12 but the timer is already used"
#else
#define STM32_TIM12_IS_USED
#endif
#endif

#if STM32_EICU_USE_TIM13
#if defined(STM32_TIM13_IS_USED)
#error "EICUD13 requires TIM13 but the timer is already used"
#else
#define STM32_TIM13_IS_USED
#endif
#endif

#if STM32_EICU_USE_TIM14
#if defined(STM32_TIM14_IS_USED)
#error "EICUD14 requires TIM14 but the timer is already used"
#else
#define STM32_TIM14_IS_USED
#endif
#endif

#if STM32_EICU_USE_TIM1 &&                                                    \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_EICU_TIM1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM1"
#endif

#if STM32_EICU_USE_TIM2 &&                                                    \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_EICU_TIM2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM2"
#endif

#if STM32_EICU_USE_TIM3 &&                                                    \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_EICU_TIM3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM3"
#endif

#if STM32_EICU_USE_TIM4 &&                                                    \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_EICU_TIM4_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM4"
#endif

#if STM32_EICU_USE_TIM5 &&                                                    \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_EICU_TIM5_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM5"
#endif

#if STM32_EICU_USE_TIM8 &&                                                    \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_EICU_TIM8_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM8"
#endif

#if STM32_EICU_USE_TIM9 &&                                                    \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_EICU_TIM9_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM9"
#endif

#if STM32_EICU_USE_TIM12 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_EICU_TIM12_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM12"
#endif

#if STM32_EICU_USE_TIM10 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_EICU_TIM10_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM10"
#endif

#if STM32_EICU_USE_TIM11 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_EICU_TIM11_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM11"
#endif

#if STM32_EICU_USE_TIM13 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_EICU_TIM13_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM13"
#endif

#if STM32_EICU_USE_TIM14 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_EICU_TIM14_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM14"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   Active level selector.
 */
typedef enum {
  EICU_INPUT_ACTIVE_HIGH,           /**< Trigger on rising edge.            */
  EICU_INPUT_ACTIVE_LOW,            /**< Trigger on falling edge.           */
} eicuactivelevel_t;

/**
 * @brief   Timer registers width in bits.
 */
typedef enum {
  EICU_WIDTH_16,
  EICU_WIDTH_32
} eicutimerwidth_t;

/**
 * @brief   EICU frequency type.
 */
typedef uint32_t eicufreq_t;

/**
 * @brief   EICU counter type.
 */
typedef uint32_t eicucnt_t;

/** 
 * @brief EICU captured width and (or) period.
 */
typedef struct {
  /**
   * @brief   Pulse width.
   */
  eicucnt_t               width;
  /**
   * @brief   Pulse period.
   */
  eicucnt_t               period;
} eicuresult_t;

/**
 * @brief EICU Capture Channel Config structure definition.
 */
typedef struct {
  /**
   * @brief   Specifies the active level of the input signal.
   */
  eicuactivelevel_t       alvl;
  /**
   * @brief   Capture event callback. Used for PWM width, pulse width and
   *          pulse period capture event.
   */
  eicucallback_t          capture_cb;
} EICUChannelConfig;

/** 
 * @brief EICU Capture Channel structure definition.
 */
typedef struct {
  /**
   * @brief   Channel state for the internal state machine.
   */
  eicuchannelstate_t      state;
  /**
   * @brief   Cached value for pulse width calculation.
   */
  eicucnt_t               last_active;
  /**
   * @brief   Cached value for period calculation.
   */
  eicucnt_t               last_idle;
  /**
   * @brief   Pointer to Input Capture channel configuration.
   */
  const EICUChannelConfig *config;
  /**
   * @brief   CCR register pointer for faster access.
   */
  volatile uint32_t       *ccrp;
} EICUChannel;

/**
 * @brief EICU Config structure definition.
 */
typedef struct {
  /**
   * @brief   Specifies the Timer clock in Hz.
   */
  eicufreq_t              frequency;
  /**
   * @brief   Pointer to each Input Capture channel configuration.
   * @note    A NULL parameter indicates the channel as unused. 
   * @note    In PWM mode, only Channel 1 OR Channel 2 may be used.
   */
  const EICUChannelConfig *iccfgp[EICU_CHANNEL_ENUM_END];
  /**
   * @brief   TIM DIER register initialization data.
   */
  uint32_t                dier;
} EICUConfig;

/** 
 * @brief EICU Driver structure definition
 */
struct EICUDriver {
  /**
   * @brief   STM32 timer peripheral for Input Capture.
   */
  stm32_tim_t             *tim;
  /**
   * @brief   Driver state for the internal state machine.
   */
  eicustate_t             state;
  /**
   * @brief   Channels' data structures.
   */
  EICUChannel             channel[EICU_CHANNEL_ENUM_END];
  /**
   * @brief   Timer base clock.
   */
  uint32_t                clock;
  /**
   * @brief   Number of available capture compare channels in timer.
   */
  size_t                  channels;
  /**
   * @brief   Timer registers width in bits.
   */
  eicutimerwidth_t        width;
  /**
   * @brief   Pointer to configuration for the driver.
   */
  const EICUConfig        *config;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
#if STM32_EICU_USE_TIM1 && !defined(__DOXYGEN__)
extern EICUDriver EICUD1;
#endif

#if STM32_EICU_USE_TIM2 && !defined(__DOXYGEN__)
extern EICUDriver EICUD2;
#endif

#if STM32_EICU_USE_TIM3 && !defined(__DOXYGEN__)
extern EICUDriver EICUD3;
#endif

#if STM32_EICU_USE_TIM4 && !defined(__DOXYGEN__)
extern EICUDriver EICUD4;
#endif

#if STM32_EICU_USE_TIM5 && !defined(__DOXYGEN__)
extern EICUDriver EICUD5;
#endif

#if STM32_EICU_USE_TIM8 && !defined(__DOXYGEN__)
extern EICUDriver EICUD8;
#endif

#if STM32_EICU_USE_TIM9 && !defined(__DOXYGEN__)
extern EICUDriver EICUD9;
#endif

#if STM32_EICU_USE_TIM12 && !defined(__DOXYGEN__)
extern EICUDriver EICUD12;
#endif

#if STM32_EICU_USE_TIM10 && !defined(__DOXYGEN__)
extern EICUDriver EICUD10;
#endif

#if STM32_EICU_USE_TIM11 && !defined(__DOXYGEN__)
extern EICUDriver EICUD11;
#endif

#if STM32_EICU_USE_TIM13 && !defined(__DOXYGEN__)
extern EICUDriver EICUD13;
#endif

#if STM32_EICU_USE_TIM14 && !defined(__DOXYGEN__)
extern EICUDriver EICUD14;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void eicu_lld_init(void);
  void eicu_lld_start(EICUDriver *eicup);
  void eicu_lld_stop(EICUDriver *eicup);
  void eicu_lld_enable(EICUDriver *eicup);
  void eicu_lld_disable(EICUDriver *eicup);
  void eicu_lld_serve_interrupt(EICUDriver *eicup);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EICU */

#endif /* HAL_EICU_LLD_H */