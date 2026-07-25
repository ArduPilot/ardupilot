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
 * @file    TIMv1/hal_pwm_lld.h
 * @brief   STM32 PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#ifndef HAL_PWM_LLD_H
#define HAL_PWM_LLD_H

#if HAL_USE_PWM || defined(__DOXYGEN__)

#include "stm32_tim.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Number of PWM channels per PWM driver.
 */
#define PWM_CHANNELS                            STM32_TIM_MAX_CHANNELS

/**
 * @name    STM32-specific PWM complementary output mode macros
 * @{
 */
/**
 * @brief   Complementary output modes mask.
 * @note    This is an STM32-specific setting.
 */
#define PWM_COMPLEMENTARY_OUTPUT_MASK           0xF0

/**
 * @brief   Complementary output not driven.
 * @note    This is an STM32-specific setting.
 */
#define PWM_COMPLEMENTARY_OUTPUT_DISABLED       0x00

/**
 * @brief   Complementary output, active is logic level one.
 * @note    This is an STM32-specific setting.
 * @note    This setting is only available if the timer supports the
 *          BDTR register.
 */
#define PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH    0x10

/**
 * @brief   Complementary output, active is logic level zero.
 * @note    This is an STM32-specific setting.
 * @note    This setting is only available if the timer supports the
 *          BDTR register.
 */
#define PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW     0x20
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   PWMD1 driver enable switch.
 * @details If set to @p TRUE the support for PWMD1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM1) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM1                  FALSE
#endif

/**
 * @brief   PWMD2 driver enable switch.
 * @details If set to @p TRUE the support for PWMD2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM2) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM2                  FALSE
#endif

/**
 * @brief   PWMD3 driver enable switch.
 * @details If set to @p TRUE the support for PWMD3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM3) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM3                  FALSE
#endif

/**
 * @brief   PWMD4 driver enable switch.
 * @details If set to @p TRUE the support for PWMD4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM4) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM4                  FALSE
#endif

/**
 * @brief   PWMD5 driver enable switch.
 * @details If set to @p TRUE the support for PWMD5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM5) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM5                  FALSE
#endif

/**
 * @brief   PWMD8 driver enable switch.
 * @details If set to @p TRUE the support for PWMD8 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM8) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM8                  FALSE
#endif

/**
 * @brief   PWMD9 driver enable switch.
 * @details If set to @p TRUE the support for PWMD9 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM9) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM9                  FALSE
#endif

/**
 * @brief   PWMD10 driver enable switch.
 * @details If set to @p TRUE the support for PWMD10 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM10) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM10                 FALSE
#endif

/**
 * @brief   PWMD11 driver enable switch.
 * @details If set to @p TRUE the support for PWMD11 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM11) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM11                 FALSE
#endif

/**
 * @brief   PWMD12 driver enable switch.
 * @details If set to @p TRUE the support for PWMD12 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM12) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM12                 FALSE
#endif

/**
 * @brief   PWMD13 driver enable switch.
 * @details If set to @p TRUE the support for PWMD13 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM13) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM13                 FALSE
#endif

/**
 * @brief   PWMD14 driver enable switch.
 * @details If set to @p TRUE the support for PWMD14 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM14) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM14                 FALSE
#endif

/**
 * @brief   PWMD15 driver enable switch.
 * @details If set to @p TRUE the support for PWMD15 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM15) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM15                 FALSE
#endif

/**
 * @brief   PWMD16 driver enable switch.
 * @details If set to @p TRUE the support for PWMD16 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM16) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM16                 FALSE
#endif

/**
 * @brief   PWMD17 driver enable switch.
 * @details If set to @p TRUE the support for PWMD17 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM17) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM17                 FALSE
#endif

/**
 * @brief   PWMD20 driver enable switch.
 * @details If set to @p TRUE the support for PWMD20 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM20) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM20                 FALSE
#endif

/**
 * @brief   PWMD21 driver enable switch.
 * @details If set to @p TRUE the support for PWMD21 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM21) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM21                 FALSE
#endif

/**
 * @brief   PWMD22 driver enable switch.
 * @details If set to @p TRUE the support for PWMD22 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_PWM_USE_TIM22) || defined(__DOXYGEN__)
#define STM32_PWM_USE_TIM22                 FALSE
#endif

/**
 * @brief   PWMD1 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM1_IRQ_PRIORITY         7
#endif

/**
 * @brief   PWMD2 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM2_IRQ_PRIORITY         7
#endif

/**
 * @brief   PWMD3 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM3_IRQ_PRIORITY         7
#endif

/**
 * @brief   PWMD4 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM4_IRQ_PRIORITY         7
#endif

/**
 * @brief   PWMD5 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM5_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM5_IRQ_PRIORITY         7
#endif

/**
 * @brief   PWMD8 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM8_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM8_IRQ_PRIORITY         7
#endif

/**
 * @brief   PWMD9 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM9_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM9_IRQ_PRIORITY         7
#endif

/**
 * @brief   PWMD10 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM10_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM10_IRQ_PRIORITY        7
#endif

/**
 * @brief   PWMD11 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM11_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM11_IRQ_PRIORITY        7
#endif

/**
 * @brief   PWMD12 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM12_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM12_IRQ_PRIORITY        7
#endif

/**
 * @brief   PWMD13 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM13_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM13_IRQ_PRIORITY        7
#endif

/**
 * @brief   PWMD14 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM14_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM14_IRQ_PRIORITY        7
#endif

/**
 * @brief   PWMD15 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM15_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM15_IRQ_PRIORITY        7
#endif

/**
 * @brief   PWMD16 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM16_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM16_IRQ_PRIORITY        7
#endif

/**
 * @brief   PWMD17 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM17_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM17_IRQ_PRIORITY        7
#endif

/**
 * @brief   PWMD20 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM20_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM20_IRQ_PRIORITY        7
#endif

/**
 * @brief   PWMD21 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM21_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM21_IRQ_PRIORITY        7
#endif

/**
 * @brief   PWMD22 interrupt priority level setting.
 */
#if !defined(STM32_PWM_TIM22_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_PWM_TIM22_IRQ_PRIORITY        7
#endif
/** @} */

/*===========================================================================*/
/* Configuration checks.                                                     */
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

#if !defined(STM32_HAS_TIM16)
#define STM32_HAS_TIM16                     FALSE
#endif

#if !defined(STM32_HAS_TIM17)
#define STM32_HAS_TIM17                     FALSE
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

#if STM32_PWM_USE_TIM1 && !STM32_HAS_TIM1
#error "TIM1 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM2 && !STM32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM3 && !STM32_HAS_TIM3
#error "TIM3 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM4 && !STM32_HAS_TIM4
#error "TIM4 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM5 && !STM32_HAS_TIM5
#error "TIM5 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM8 && !STM32_HAS_TIM8
#error "TIM8 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM9 && !STM32_HAS_TIM9
#error "TIM9 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM10 && !STM32_HAS_TIM10
#error "TIM10 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM11 && !STM32_HAS_TIM11
#error "TIM11 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM12 && !STM32_HAS_TIM12
#error "TIM12 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM13 && !STM32_HAS_TIM13
#error "TIM13 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM14 && !STM32_HAS_TIM14
#error "TIM14 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM15 && !STM32_HAS_TIM15
#error "TIM15 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM16 && !STM32_HAS_TIM16
#error "TIM16 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM17 && !STM32_HAS_TIM17
#error "TIM17 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM20 && !STM32_HAS_TIM20
#error "TIM20 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM21 && !STM32_HAS_TIM21
#error "TIM21 not present in the selected device"
#endif

#if STM32_PWM_USE_TIM22 && !STM32_HAS_TIM22
#error "TIM22 not present in the selected device"
#endif

#if !STM32_PWM_USE_TIM1  && !STM32_PWM_USE_TIM2  &&                         \
    !STM32_PWM_USE_TIM3  && !STM32_PWM_USE_TIM4  &&                         \
    !STM32_PWM_USE_TIM5  && !STM32_PWM_USE_TIM8  &&                         \
    !STM32_PWM_USE_TIM9  && !STM32_PWM_USE_TIM10 &&                         \
    !STM32_PWM_USE_TIM11 && !STM32_PWM_USE_TIM12 &&                         \
    !STM32_PWM_USE_TIM13 && !STM32_PWM_USE_TIM14 &&                         \
    !STM32_PWM_USE_TIM15 && !STM32_PWM_USE_TIM16 &&                         \
    !STM32_PWM_USE_TIM17 && !STM32_PWM_USE_TIM20 &&                         \
    !STM32_PWM_USE_TIM21 && !STM32_PWM_USE_TIM22
#error "PWM driver activated but no TIM peripheral assigned"
#endif

/* Checks on allocation of TIMx units.*/
#if STM32_PWM_USE_TIM1
#if defined(STM32_TIM1_IS_USED)
#error "PWMD1 requires TIM1 but the timer is already used"
#else
#define STM32_TIM1_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM2
#if defined(STM32_TIM2_IS_USED)
#error "PWMD2 requires TIM2 but the timer is already used"
#else
#define STM32_TIM2_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM3
#if defined(STM32_TIM3_IS_USED)
#error "PWMD3 requires TIM3 but the timer is already used"
#else
#define STM32_TIM3_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM4
#if defined(STM32_TIM4_IS_USED)
#error "PWMD4 requires TIM4 but the timer is already used"
#else
#define STM32_TIM4_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM5
#if defined(STM32_TIM5_IS_USED)
#error "PWMD5 requires TIM5 but the timer is already used"
#else
#define STM32_TIM5_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM8
#if defined(STM32_TIM8_IS_USED)
#error "PWMD8 requires TIM8 but the timer is already used"
#else
#define STM32_TIM8_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM9
#if defined(STM32_TIM9_IS_USED)
#error "PWMD9 requires TIM9 but the timer is already used"
#else
#define STM32_TIM9_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM10
#if defined(STM32_TIM10_IS_USED)
#error "PWMD10 requires TIM10 but the timer is already used"
#else
#define STM32_TIM10_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM11
#if defined(STM32_TIM11_IS_USED)
#error "PWMD11 requires TIM11 but the timer is already used"
#else
#define STM32_TIM11_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM12
#if defined(STM32_TIM12_IS_USED)
#error "PWMD12 requires TIM12 but the timer is already used"
#else
#define STM32_TIM12_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM13
#if defined(STM32_TIM13_IS_USED)
#error "PWMD13 requires TIM13 but the timer is already used"
#else
#define STM32_TIM13_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM14
#if defined(STM32_TIM14_IS_USED)
#error "PWMD14 requires TIM14 but the timer is already used"
#else
#define STM32_TIM14_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM15
#if defined(STM32_TIM15_IS_USED)
#error "PWMD15 requires TIM15 but the timer is already used"
#else
#define STM32_TIM15_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM16
#if defined(STM32_TIM16_IS_USED)
#error "PWMD16 requires TIM16 but the timer is already used"
#else
#define STM32_TIM16_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM17
#if defined(STM32_TIM17_IS_USED)
#error "PWMD17 requires TIM17 but the timer is already used"
#else
#define STM32_TIM17_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM20
#if defined(STM32_TIM20_IS_USED)
#error "PWMD20 requires TIM20 but the timer is already used"
#else
#define STM32_TIM20_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM21
#if defined(STM32_TIM21_IS_USED)
#error "PWMD21 requires TIM21 but the timer is already used"
#else
#define STM32_TIM21_IS_USED
#endif
#endif

#if STM32_PWM_USE_TIM22
#if defined(STM32_TIM22_IS_USED)
#error "PWMD22 requires TIM22 but the timer is already used"
#else
#define STM32_TIM22_IS_USED
#endif
#endif

/* IRQ priority checks.*/
#if STM32_PWM_USE_TIM1 && !defined(STM32_TIM1_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM1"
#endif

#if STM32_PWM_USE_TIM2 && !defined(STM32_TIM2_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM2"
#endif

#if STM32_PWM_USE_TIM3 && !defined(STM32_TIM3_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM3"
#endif

#if STM32_PWM_USE_TIM4 && !defined(STM32_TIM4_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM4_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM4"
#endif

#if STM32_PWM_USE_TIM5 && !defined(STM32_TIM5_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM5_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM5"
#endif

#if STM32_PWM_USE_TIM8 && !defined(STM32_TIM8_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM8_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM8"
#endif

#if STM32_PWM_USE_TIM9 && !defined(STM32_TIM9_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM9_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM9"
#endif

#if STM32_PWM_USE_TIM10 && !defined(STM32_TIM10_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM10_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM10"
#endif

#if STM32_PWM_USE_TIM11 && !defined(STM32_TIM11_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM11_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM11"
#endif

#if STM32_PWM_USE_TIM12 && !defined(STM32_TIM12_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM12_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM12"
#endif

#if STM32_PWM_USE_TIM13 && !defined(STM32_TIM13_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM13_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM13"
#endif

#if STM32_PWM_USE_TIM14 && !defined(STM32_TIM14_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM14_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM14"
#endif

#if STM32_PWM_USE_TIM15 && !defined(STM32_TIM15_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM15_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM15"
#endif

#if STM32_PWM_USE_TIM16 && !defined(STM32_TIM16_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM16_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM16"
#endif

#if STM32_PWM_USE_TIM17 && !defined(STM32_TIM17_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM17_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM17"
#endif

#if STM32_PWM_USE_TIM20 && !defined(STM32_TIM20_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM20_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM20"
#endif

#if STM32_PWM_USE_TIM21 && !defined(STM32_TIM21_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM21_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM21"
#endif

#if STM32_PWM_USE_TIM22 && !defined(STM32_TIM22_SUPPRESS_ISR) &&            \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_PWM_TIM22_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM22"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a PWM mode.
 */
typedef uint32_t pwmmode_t;

/**
 * @brief   Type of a PWM channel.
 */
typedef uint8_t pwmchannel_t;

/**
 * @brief   Type of a channels mask.
 */
typedef uint32_t pwmchnmsk_t;

/**
 * @brief   Type of a PWM counter.
 */
typedef uint32_t pwmcnt_t;

/**
 * @brief   Type of a PWM driver channel configuration structure.
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
  /* End of the mandatory fields.*/
} PWMChannelConfig;

/**
 * @brief   Type of a PWM driver configuration structure.
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
  /* End of the mandatory fields.*/
  /**
   * @brief TIM CR2 register initialization data.
   * @note  The value of this field should normally be equal to zero.
   */
  uint32_t                  cr2;
  /**
   * @brief TIM BDTR (break & dead-time) register initialization data.
   * @note  The value of this field should normally be equal to zero.
   */                                                                     \
   uint32_t                 bdtr;
   /**
    * @brief TIM DIER register initialization data.
    * @note  The value of this field should normally be equal to zero.
    * @note  Only the DMA-related bits can be specified in this field.
    */
   uint32_t                 dier;
} PWMConfig;

/**
 * @brief   Structure representing a PWM driver.
 */
struct PWMDriver {
  /**
   * @brief Driver state.
   */
  pwmstate_t                state;
  /**
   * @brief Current driver configuration data.
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
  /* End of the mandatory fields.*/
  /**
   * @brief Timer base clock.
   */
  uint32_t                  clock;
  /**
   * @brief Presence of BDTR register.
   */
  bool                      has_bdtr;
  /**
   * @brief Pointer to the TIMx registers block.
   */
  stm32_tim_t               *tim;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Changes the period the PWM peripheral.
 * @details This function changes the period of a PWM unit that has already
 *          been activated using @p pwmStart().
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The PWM unit period is changed to the new value.
 * @note    The function has effect at the next cycle start.
 * @note    If a period is specified that is shorter than the pulse width
 *          programmed in one of the channels then the behavior is not
 *          guaranteed.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] period    new cycle time in ticks
 *
 * @notapi
 */
#define pwm_lld_change_period(pwmp, period)                                 \
  ((pwmp)->tim->ARR = ((period) - 1))

/**
 * @brief   Returns the TIM associated with a PWM.
 *
 * @param[in] pwmp      pointer to the @p PWMDriver object
 * @return              The TIM reference.
 *
 * @notapi
 */
#define pwm_lld_get_timer(pwmp) ((pwmp)->tim)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_PWM_USE_TIM1 && !defined(__DOXYGEN__)
extern PWMDriver PWMD1;
#endif

#if STM32_PWM_USE_TIM2 && !defined(__DOXYGEN__)
extern PWMDriver PWMD2;
#endif

#if STM32_PWM_USE_TIM3 && !defined(__DOXYGEN__)
extern PWMDriver PWMD3;
#endif

#if STM32_PWM_USE_TIM4 && !defined(__DOXYGEN__)
extern PWMDriver PWMD4;
#endif

#if STM32_PWM_USE_TIM5 && !defined(__DOXYGEN__)
extern PWMDriver PWMD5;
#endif

#if STM32_PWM_USE_TIM8 && !defined(__DOXYGEN__)
extern PWMDriver PWMD8;
#endif

#if STM32_PWM_USE_TIM9 && !defined(__DOXYGEN__)
extern PWMDriver PWMD9;
#endif

#if STM32_PWM_USE_TIM10 && !defined(__DOXYGEN__)
extern PWMDriver PWMD10;
#endif

#if STM32_PWM_USE_TIM11 && !defined(__DOXYGEN__)
extern PWMDriver PWMD11;
#endif

#if STM32_PWM_USE_TIM12 && !defined(__DOXYGEN__)
extern PWMDriver PWMD12;
#endif

#if STM32_PWM_USE_TIM13 && !defined(__DOXYGEN__)
extern PWMDriver PWMD13;
#endif

#if STM32_PWM_USE_TIM14 && !defined(__DOXYGEN__)
extern PWMDriver PWMD14;
#endif

#if STM32_PWM_USE_TIM15 && !defined(__DOXYGEN__)
extern PWMDriver PWMD15;
#endif

#if STM32_PWM_USE_TIM16 && !defined(__DOXYGEN__)
extern PWMDriver PWMD16;
#endif

#if STM32_PWM_USE_TIM17 && !defined(__DOXYGEN__)
extern PWMDriver PWMD17;
#endif

#if STM32_PWM_USE_TIM20 && !defined(__DOXYGEN__)
extern PWMDriver PWMD20;
#endif

#if STM32_PWM_USE_TIM21 && !defined(__DOXYGEN__)
extern PWMDriver PWMD21;
#endif

#if STM32_PWM_USE_TIM22 && !defined(__DOXYGEN__)
extern PWMDriver PWMD22;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void pwm_lld_init(void);
  void pwm_lld_start(PWMDriver *pwmp);
  void pwm_lld_stop(PWMDriver *pwmp);
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
  void pwm_lld_serve_interrupt(PWMDriver *pwmp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PWM */

#endif /* HAL_PWM_LLD_H */

/** @} */
