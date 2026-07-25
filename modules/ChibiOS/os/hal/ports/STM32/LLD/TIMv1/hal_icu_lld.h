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
 * @brief   STM32 ICU subsystem low level driver header.
 *
 * @addtogroup ICU
 * @{
 */

#ifndef HAL_ICU_LLD_H
#define HAL_ICU_LLD_H

#if HAL_USE_ICU || defined(__DOXYGEN__)

#include "stm32_tim.h"

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
 * @brief   ICUD1 driver enable switch.
 * @details If set to @p TRUE the support for ICUD1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM1) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM1                  FALSE
#endif

/**
 * @brief   ICUD2 driver enable switch.
 * @details If set to @p TRUE the support for ICUD2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM2) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM2                  FALSE
#endif

/**
 * @brief   ICUD3 driver enable switch.
 * @details If set to @p TRUE the support for ICUD3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM3) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM3                  FALSE
#endif

/**
 * @brief   ICUD4 driver enable switch.
 * @details If set to @p TRUE the support for ICUD4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM4) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM4                  FALSE
#endif

/**
 * @brief   ICUD5 driver enable switch.
 * @details If set to @p TRUE the support for ICUD5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM5) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM5                  FALSE
#endif

/**
 * @brief   ICUD8 driver enable switch.
 * @details If set to @p TRUE the support for ICUD8 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM8) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM8                  FALSE
#endif

/**
 * @brief   ICUD9 driver enable switch.
 * @details If set to @p TRUE the support for ICUD9 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM9) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM9                  FALSE
#endif

/**
 * @brief   ICUD10 driver enable switch.
 * @details If set to @p TRUE the support for ICUD10 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM10) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM10                 FALSE
#endif

/**
 * @brief   ICUD11 driver enable switch.
 * @details If set to @p TRUE the support for ICUD11 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM11) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM11                 FALSE
#endif

/**
 * @brief   ICUD12 driver enable switch.
 * @details If set to @p TRUE the support for ICUD12 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM12) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM12                 FALSE
#endif

/**
 * @brief   ICUD13 driver enable switch.
 * @details If set to @p TRUE the support for ICUD13 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM13) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM13                 FALSE
#endif

/**
 * @brief   ICUD14 driver enable switch.
 * @details If set to @p TRUE the support for ICUD14 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM14) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM14                 FALSE
#endif

/**
 * @brief   ICUD15 driver enable switch.
 * @details If set to @p TRUE the support for ICUD15 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM15) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM15                 FALSE
#endif

/**
 * @brief   ICUD20 driver enable switch.
 * @details If set to @p TRUE the support for ICUD20 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM20) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM20                 FALSE
#endif

/**
 * @brief   ICUD21 driver enable switch.
 * @details If set to @p TRUE the support for ICUD21 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM21) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM21                 FALSE
#endif

/**
 * @brief   ICUD22 driver enable switch.
 * @details If set to @p TRUE the support for ICUD22 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ICU_USE_TIM22) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM22                 FALSE
#endif

/**
 * @brief   ICUD1 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM1_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD2 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM2_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD3 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM3_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD4 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM4_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD5 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM5_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM5_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD8 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM8_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM8_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD9 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM9_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM9_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD10 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM10_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM10_IRQ_PRIORITY        7
#endif

/**
 * @brief   ICUD11 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM11_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM11_IRQ_PRIORITY        7
#endif

/**
 * @brief   ICUD12 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM12_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM12_IRQ_PRIORITY        7
#endif

/**
 * @brief   ICUD13 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM13_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM13_IRQ_PRIORITY        7
#endif

/**
 * @brief   ICUD14 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM14_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM14_IRQ_PRIORITY        7
#endif

/**
 * @brief   ICUD15 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM15_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM15_IRQ_PRIORITY        7
#endif

/**
 * @brief   ICUD20 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM20_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM20_IRQ_PRIORITY        7
#endif

/**
 * @brief   ICUD21 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM21_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM21_IRQ_PRIORITY        7
#endif

/**
 * @brief   ICUD22 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM22_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM22_IRQ_PRIORITY        7
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !defined(STM32_HAS_TIM1)
#define STM32_HAS_TIM1                      FALSE
#endif

#if !defined(STM32_HAS_TIM2)
#define STM32_HAS_TIM2                      FALSE
#endif

#if !defined(STM32_HAS_TIM3)
#define STM32_HAS_TIM3                      FALSE
#endif

#if !defined(STM32_HAS_TIM4)
#define STM32_HAS_TIM4                      FALSE
#endif

#if !defined(STM32_HAS_TIM5)
#define STM32_HAS_TIM5                      FALSE
#endif

#if !defined(STM32_HAS_TIM8)
#define STM32_HAS_TIM8                      FALSE
#endif

#if !defined(STM32_HAS_TIM9)
#define STM32_HAS_TIM9                      FALSE
#endif

#if !defined(STM32_HAS_TIM10)
#define STM32_HAS_TIM10                     FALSE
#endif

#if !defined(STM32_HAS_TIM11)
#define STM32_HAS_TIM11                     FALSE
#endif

#if !defined(STM32_HAS_TIM12)
#define STM32_HAS_TIM12                     FALSE
#endif

#if !defined(STM32_HAS_TIM13)
#define STM32_HAS_TIM13                     FALSE
#endif

#if !defined(STM32_HAS_TIM14)
#define STM32_HAS_TIM14                     FALSE
#endif

#if !defined(STM32_HAS_TIM15)
#define STM32_HAS_TIM15                     FALSE
#endif

#if !defined(STM32_HAS_TIM20)
#define STM32_HAS_TIM20                     FALSE
#endif

#if !defined(STM32_HAS_TIM21)
#define STM32_HAS_TIM21                     FALSE
#endif

#if !defined(STM32_HAS_TIM22)
#define STM32_HAS_TIM22                     FALSE
#endif

#if STM32_ICU_USE_TIM1 && !STM32_HAS_TIM1
#error "TIM1 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM2 && !STM32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM3 && !STM32_HAS_TIM3
#error "TIM3 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM4 && !STM32_HAS_TIM4
#error "TIM4 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM5 && !STM32_HAS_TIM5
#error "TIM5 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM8 && !STM32_HAS_TIM8
#error "TIM8 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM9 && !STM32_HAS_TIM9
#error "TIM9 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM10 && !STM32_HAS_TIM10
#error "TIM10 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM11 && !STM32_HAS_TIM11
#error "TIM11 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM12 && !STM32_HAS_TIM12
#error "TIM12 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM13 && !STM32_HAS_TIM13
#error "TIM13 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM14 && !STM32_HAS_TIM14
#error "TIM14 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM15 && !STM32_HAS_TIM15
#error "TIM15 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM20 && !STM32_HAS_TIM20
#error "TIM20 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM21 && !STM32_HAS_TIM21
#error "TIM21 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM22 && !STM32_HAS_TIM22
#error "TIM22 not present in the selected device"
#endif

#if !STM32_ICU_USE_TIM1  && !STM32_ICU_USE_TIM2  &&                         \
    !STM32_ICU_USE_TIM3  && !STM32_ICU_USE_TIM4  &&                         \
    !STM32_ICU_USE_TIM5  && !STM32_ICU_USE_TIM8  &&                         \
    !STM32_ICU_USE_TIM9  && !STM32_ICU_USE_TIM10 &&                         \
    !STM32_ICU_USE_TIM11 && !STM32_ICU_USE_TIM12 &&                         \
    !STM32_ICU_USE_TIM13 && !STM32_ICU_USE_TIM14 &&                         \
    !STM32_ICU_USE_TIM15 && !STM32_ICU_USE_TIM20 &&                         \
    !STM32_ICU_USE_TIM21 && !STM32_ICU_USE_TIM22
#error "ICU driver activated but no TIM peripheral assigned"
#endif

/* Checks on allocation of TIMx units.*/
#if STM32_ICU_USE_TIM1
#if defined(STM32_TIM1_IS_USED)
#error "ICUD1 requires TIM1 but the timer is already used"
#else
#define STM32_TIM1_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM2
#if defined(STM32_TIM2_IS_USED)
#error "ICUD2 requires TIM2 but the timer is already used"
#else
#define STM32_TIM2_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM3
#if defined(STM32_TIM3_IS_USED)
#error "ICUD3 requires TIM3 but the timer is already used"
#else
#define STM32_TIM3_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM4
#if defined(STM32_TIM4_IS_USED)
#error "ICUD4 requires TIM4 but the timer is already used"
#else
#define STM32_TIM4_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM5
#if defined(STM32_TIM5_IS_USED)
#error "ICUD5 requires TIM5 but the timer is already used"
#else
#define STM32_TIM5_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM8
#if defined(STM32_TIM8_IS_USED)
#error "ICUD8 requires TIM8 but the timer is already used"
#else
#define STM32_TIM8_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM9
#if defined(STM32_TIM9_IS_USED)
#error "ICUD9 requires TIM9 but the timer is already used"
#else
#define STM32_TIM9_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM10
#if defined(STM32_TIM10_IS_USED)
#error "ICUD10 requires TIM10 but the timer is already used"
#else
#define STM32_TIM10_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM11
#if defined(STM32_TIM11_IS_USED)
#error "ICUD11 requires TIM11 but the timer is already used"
#else
#define STM32_TIM11_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM12
#if defined(STM32_TIM12_IS_USED)
#error "ICUD12 requires TIM12 but the timer is already used"
#else
#define STM32_TIM12_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM13
#if defined(STM32_TIM13_IS_USED)
#error "ICUD13 requires TIM13 but the timer is already used"
#else
#define STM32_TIM13_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM14
#if defined(STM32_TIM14_IS_USED)
#error "ICUD14 requires TIM14 but the timer is already used"
#else
#define STM32_TIM14_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM15
#if defined(STM32_TIM15_IS_USED)
#error "ICUD15 requires TIM15 but the timer is already used"
#else
#define STM32_TIM15_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM20
#if defined(STM32_TIM20_IS_USED)
#error "ICUD20 requires TIM20 but the timer is already used"
#else
#define STM32_TIM20_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM21
#if defined(STM32_TIM21_IS_USED)
#error "ICUD21 requires TIM21 but the timer is already used"
#else
#define STM32_TIM21_IS_USED
#endif
#endif

#if STM32_ICU_USE_TIM22
#if defined(STM32_TIM22_IS_USED)
#error "ICUD22 requires TIM22 but the timer is already used"
#else
#define STM32_TIM22_IS_USED
#endif
#endif

/* IRQ priority checks.*/
#if STM32_ICU_USE_TIM1 && !defined(STM32_TIM1_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM1"
#endif

#if STM32_ICU_USE_TIM2 && !defined(STM32_TIM2_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM2"
#endif

#if STM32_ICU_USE_TIM3 && !defined(STM32_TIM3_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM3"
#endif

#if STM32_ICU_USE_TIM4 && !defined(STM32_TIM4_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM4_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM4"
#endif

#if STM32_ICU_USE_TIM5 && !defined(STM32_TIM5_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM5_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM5"
#endif

#if STM32_ICU_USE_TIM8 && !defined(STM32_TIM8_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM8_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM8"
#endif

#if STM32_ICU_USE_TIM9 && !defined(STM32_TIM9_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM9_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM9"
#endif

#if STM32_ICU_USE_TIM10 && !defined(STM32_TIM10_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM10_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM10"
#endif

#if STM32_ICU_USE_TIM11 && !defined(STM32_TIM11_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM11_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM11"
#endif

#if STM32_ICU_USE_TIM12 && !defined(STM32_TIM12_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM12_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM12"
#endif

#if STM32_ICU_USE_TIM13 && !defined(STM32_TIM13_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM13_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM13"
#endif

#if STM32_ICU_USE_TIM14 && !defined(STM32_TIM14_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM14_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM14"
#endif

#if STM32_ICU_USE_TIM15 && !defined(STM32_TIM15_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM15_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM15"
#endif

#if STM32_ICU_USE_TIM20 && !defined(STM32_TIM20_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM20_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM20"
#endif

#if STM32_ICU_USE_TIM21 && !defined(STM32_TIM21_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM21_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM21"
#endif

#if STM32_ICU_USE_TIM22 && !defined(STM32_TIM22_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ICU_TIM22_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM22"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   ICU driver mode.
 */
typedef enum {
  ICU_INPUT_ACTIVE_HIGH = 0,        /**< Trigger on rising edge.            */
  ICU_INPUT_ACTIVE_LOW = 1,         /**< Trigger on falling edge.           */
} icumode_t;

/**
 * @brief   ICU frequency type.
 */
typedef uint32_t icufreq_t;

/**
 * @brief   ICU channel type.
 */
typedef enum {
  ICU_CHANNEL_1 = 0,              /**< Use TIMxCH1.      */
  ICU_CHANNEL_2 = 1,              /**< Use TIMxCH2.      */
} icuchannel_t;

/**
 * @brief   ICU counter type.
 */
typedef uint32_t icucnt_t;

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
  /* End of the mandatory fields.*/
  /**
   * @brief   Timer input channel to be used.
   * @note    Only inputs TIMx 1 and 2 are supported.
   */
  icuchannel_t              channel;
  /**
   * @brief TIM DIER register initialization data.
   * @note  The value of this field should normally be equal to zero.
   * @note  Only the DMA-related bits can be specified in this field.
   */
  uint32_t                  dier;
  /**
   * @brief TIM ARR register initialization data.
   * @note  The value of this field should normally be equal to 0xFFFFFFFFU.
   */
  uint32_t                  arr;
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
  /* End of the mandatory fields.*/
  /**
   * @brief Timer base clock.
   */
  uint32_t                  clock;
  /**
   * @brief Pointer to the TIMx registers block.
   */
  stm32_tim_t               *tim;
  /**
   * @brief CCR register used for width capture.
   */
  volatile uint32_t         *wccrp;
  /**
   * @brief CCR register used for period capture.
   */
  volatile uint32_t         *pccrp;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

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
#define icu_lld_get_width(icup) (*((icup)->wccrp) + 1)

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
#define icu_lld_get_period(icup) (*((icup)->pccrp) + 1)

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
#define icu_lld_are_notifications_enabled(icup)                             \
  (bool)(((icup)->tim->DIER & STM32_TIM_DIER_IRQ_MASK) != 0)

/**
 * @brief   Returns the TIM associated with an ICU.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The TIM reference.
 *
 * @notapi
 */
#define icu_lld_get_timer(icup) ((icup)->tim)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_ICU_USE_TIM1 && !defined(__DOXYGEN__)
extern ICUDriver ICUD1;
#endif

#if STM32_ICU_USE_TIM2 && !defined(__DOXYGEN__)
extern ICUDriver ICUD2;
#endif

#if STM32_ICU_USE_TIM3 && !defined(__DOXYGEN__)
extern ICUDriver ICUD3;
#endif

#if STM32_ICU_USE_TIM4 && !defined(__DOXYGEN__)
extern ICUDriver ICUD4;
#endif

#if STM32_ICU_USE_TIM5 && !defined(__DOXYGEN__)
extern ICUDriver ICUD5;
#endif

#if STM32_ICU_USE_TIM8 && !defined(__DOXYGEN__)
extern ICUDriver ICUD8;
#endif

#if STM32_ICU_USE_TIM9 && !defined(__DOXYGEN__)
extern ICUDriver ICUD9;
#endif

#if STM32_ICU_USE_TIM10 && !defined(__DOXYGEN__)
extern ICUDriver ICUD10;
#endif

#if STM32_ICU_USE_TIM11 && !defined(__DOXYGEN__)
extern ICUDriver ICUD11;
#endif

#if STM32_ICU_USE_TIM12 && !defined(__DOXYGEN__)
extern ICUDriver ICUD12;
#endif

#if STM32_ICU_USE_TIM13 && !defined(__DOXYGEN__)
extern ICUDriver ICUD13;
#endif

#if STM32_ICU_USE_TIM14 && !defined(__DOXYGEN__)
extern ICUDriver ICUD14;
#endif

#if STM32_ICU_USE_TIM15 && !defined(__DOXYGEN__)
extern ICUDriver ICUD15;
#endif

#if STM32_ICU_USE_TIM20 && !defined(__DOXYGEN__)
extern ICUDriver ICUD20;
#endif

#if STM32_ICU_USE_TIM21 && !defined(__DOXYGEN__)
extern ICUDriver ICUD21;
#endif

#if STM32_ICU_USE_TIM22 && !defined(__DOXYGEN__)
extern ICUDriver ICUD22;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void icu_lld_init(void);
  void icu_lld_start(ICUDriver *icup);
  void icu_lld_stop(ICUDriver *icup);
  void icu_lld_start_capture(ICUDriver *icup);
  bool icu_lld_wait_capture(ICUDriver *icup);
  void icu_lld_stop_capture(ICUDriver *icup);
  void icu_lld_enable_notifications(ICUDriver *icup);
  void icu_lld_disable_notifications(ICUDriver *icup);
  void icu_lld_serve_interrupt(ICUDriver *icup);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ICU */

#endif /* HAL_ICU_LLD_H */

/** @} */
