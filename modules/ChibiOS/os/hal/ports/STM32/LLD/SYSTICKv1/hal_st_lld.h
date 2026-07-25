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
 * @file    SYSTICKv1/hal_st_lld.h
 * @brief   ST Driver subsystem low level driver header.
 * @details This header is designed to be include-able without having to
 *          include other files from the HAL.
 *
 * @addtogroup ST
 * @{
 */

#ifndef HAL_ST_LLD_H
#define HAL_ST_LLD_H

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
 * @brief   SysTick timer IRQ priority.
 */
#if !defined(STM32_ST_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ST_IRQ_PRIORITY               8
#endif

/**
 * @brief   TIMx unit (by number) to be used for free running operations.
 * @note    You must select a 32 bits timer if a 32 bits @p systick_t type
 *          is required.
 * @note    Timers 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
 *          21 and 22 are supported.
 */
#if !defined(STM32_ST_USE_TIMER) || defined(__DOXYGEN__)
#define STM32_ST_USE_TIMER                  2
#endif

/**
 * @brief   Overrides the number of supported alarms.
 * @note    The default number of alarms is equal to the number of
 *          comparators in the timer, overriding it to one makes
 *          the driver a little faster and smaller. The kernel itself
 *          only needs one alarm, additional features could need more.
 * @note    Zero means do not override.
 * @note    This setting is only meaningful in free running mode, in
 *          tick mode there are no alarms.
 */
#if !defined(STM32_ST_OVERRIDE_ALARMS) || defined(__DOXYGEN__)
#define STM32_ST_OVERRIDE_ALARMS            1
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* This has to go after transition to shared handlers is complete for all
   platforms.*/
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

#if !defined(STM32_HAS_TIM21)
#define STM32_HAS_TIM21                     FALSE
#endif

#if !defined(STM32_HAS_TIM22)
#define STM32_HAS_TIM22                     FALSE
#endif
/* End of checks to be removed.*/

#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING

#if STM32_ST_USE_TIMER == 1

#if defined(STM32_TIM1_IS_USED)
#error "ST requires TIM1 but the timer is already used"
#else
#define STM32_TIM1_IS_USED
#endif

#if defined(STM32_TIM1_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM1
#define ST_LLD_NUM_ALARMS                   STM32_TIM1_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   TRUE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 2

#if defined(STM32_TIM2_IS_USED)
#error "ST requires TIM2 but the timer is already used"
#else
#define STM32_TIM2_IS_USED
#endif

#if defined(STM32_TIM2_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM2
#define ST_LLD_NUM_ALARMS                   STM32_TIM2_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   TRUE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 3

#if defined(STM32_TIM3_IS_USED)
#error "ST requires TIM3 but the timer is already used"
#else
#define STM32_TIM3_IS_USED
#endif

#if defined(STM32_TIM3_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM3
#define ST_LLD_NUM_ALARMS                   STM32_TIM3_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   TRUE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 4

#if defined(STM32_TIM4_IS_USED)
#error "ST requires TIM4 but the timer is already used"
#else
#define STM32_TIM4_IS_USED
#endif

#if defined(STM32_TIM4_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM4
#define ST_LLD_NUM_ALARMS                   STM32_TIM4_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   TRUE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 5

#if defined(STM32_TIM5_IS_USED)
#error "ST requires TIM5 but the timer is already used"
#else
#define STM32_TIM5_IS_USED
#endif

#if defined(STM32_TIM5_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM5
#define ST_LLD_NUM_ALARMS                   STM32_TIM5_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   TRUE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 8

#if defined(STM32_TIM8_IS_USED)
#error "ST requires TIM8 but the timer is already used"
#else
#define STM32_TIM8_IS_USED
#endif

#if defined(STM32_TIM8_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM8
#define ST_LLD_NUM_ALARMS                   STM32_TIM8_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   TRUE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 9

#if defined(STM32_TIM9_IS_USED)
#error "ST requires TIM9 but the timer is already used"
#else
#define STM32_TIM9_IS_USED
#endif

#if defined(STM32_TIM9_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM9
#define ST_LLD_NUM_ALARMS                   STM32_TIM9_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   TRUE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 10

#if defined(STM32_TIM10_IS_USED)
#error "ST requires TIM10 but the timer is already used"
#else
#define STM32_TIM10_IS_USED
#endif

#if defined(STM32_TIM10_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM10
#define ST_LLD_NUM_ALARMS                   STM32_TIM10_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  TRUE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 11

#if defined(STM32_TIM11_IS_USED)
#error "ST requires TIM11 but the timer is already used"
#else
#define STM32_TIM11_IS_USED
#endif

#if defined(STM32_TIM11_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM11
#define ST_LLD_NUM_ALARMS                   STM32_TIM11_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  TRUE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 12

#if defined(STM32_TIM12_IS_USED)
#error "ST requires TIM12 but the timer is already used"
#else
#define STM32_TIM12_IS_USED
#endif

#if defined(STM32_TIM12_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM12
#define ST_LLD_NUM_ALARMS                   STM32_TIM12_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  TRUE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 13

#if defined(STM32_TIM13_IS_USED)
#error "ST requires TIM13 but the timer is already used"
#else
#define STM32_TIM13_IS_USED
#endif

#if defined(STM32_TIM13_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM13
#define ST_LLD_NUM_ALARMS                   STM32_TIM13_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  TRUE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 14

#if defined(STM32_TIM14_IS_USED)
#error "ST requires TIM14 but the timer is already used"
#else
#define STM32_TIM14_IS_USED
#endif

#if defined(STM32_TIM14_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM14
#define ST_LLD_NUM_ALARMS                   STM32_TIM14_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  TRUE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 15

#if defined(STM32_TIM15_IS_USED)
#error "ST requires TIM15 but the timer is already used"
#else
#define STM32_TIM15_IS_USED
#endif

#if defined(STM32_TIM15_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM15
#define ST_LLD_NUM_ALARMS                   STM32_TIM15_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  TRUE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 16

#if defined(STM32_TIM16_IS_USED)
#error "ST requires TIM16 but the timer is already used"
#else
#define STM32_TIM16_IS_USED
#endif

#if defined(STM32_TIM16_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM16
#define ST_LLD_NUM_ALARMS                   STM32_TIM16_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  TRUE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 17

#if defined(STM32_TIM17_IS_USED)
#error "ST requires TIM17 but the timer is already used"
#else
#define STM32_TIM17_IS_USED
#endif

#if defined(STM32_TIM17_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM17
#define ST_LLD_NUM_ALARMS                   STM32_TIM17_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  TRUE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE
#elif STM32_ST_USE_TIMER == 21

#if defined(STM32_TIM21_IS_USED)
#error "ST requires TIM21 but the timer is already used"
#else
#define STM32_TIM21_IS_USED
#endif

#if defined(STM32_TIM21_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM21
#define ST_LLD_NUM_ALARMS                   STM32_TIM21_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  TRUE
#define STM32_ST_USE_TIM22                  FALSE

#elif STM32_ST_USE_TIMER == 22

#if defined(STM32_TIM22_IS_USED)
#error "ST requires TIM22 but the timer is already used"
#else
#define STM32_TIM22_IS_USED
#endif

#if defined(STM32_TIM22_SUPPRESS_ISR)
#define STM32_SYSTICK_SUPPRESS_ISR
#endif

#define STM32_ST_TIM                        STM32_TIM22
#define ST_LLD_NUM_ALARMS                   STM32_TIM22_CHANNELS
#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  TRUE

#else
#error "STM32_ST_USE_TIMER specifies an unsupported timer"
#endif

#if (STM32_ST_OVERRIDE_ALARMS < 0) ||                                       \
    (STM32_ST_OVERRIDE_ALARMS > ST_LLD_NUM_ALARMS)
#error "invalid STM32_ST_OVERRIDE_ALARMS value"
#endif

#if STM32_ST_OVERRIDE_ALARMS > 0
#undef ST_LLD_NUM_ALARMS
#define ST_LLD_NUM_ALARMS                   STM32_ST_OVERRIDE_ALARMS
#endif

#elif OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC

#define STM32_ST_USE_SYSTICK                TRUE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#else

#define STM32_ST_USE_SYSTICK                FALSE
#define STM32_ST_USE_TIM1                   FALSE
#define STM32_ST_USE_TIM2                   FALSE
#define STM32_ST_USE_TIM3                   FALSE
#define STM32_ST_USE_TIM4                   FALSE
#define STM32_ST_USE_TIM5                   FALSE
#define STM32_ST_USE_TIM8                   FALSE
#define STM32_ST_USE_TIM9                   FALSE
#define STM32_ST_USE_TIM10                  FALSE
#define STM32_ST_USE_TIM11                  FALSE
#define STM32_ST_USE_TIM12                  FALSE
#define STM32_ST_USE_TIM13                  FALSE
#define STM32_ST_USE_TIM14                  FALSE
#define STM32_ST_USE_TIM15                  FALSE
#define STM32_ST_USE_TIM16                  FALSE
#define STM32_ST_USE_TIM17                  FALSE
#define STM32_ST_USE_TIM21                  FALSE
#define STM32_ST_USE_TIM22                  FALSE

#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void st_lld_init(void);
  void st_lld_serve_interrupt(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Driver inline functions.                                                  */
/*===========================================================================*/

#if (OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING) || defined(__DOXYGEN__)

/**
 * @brief   Returns the time counter value.
 *
 * @return              The counter value.
 *
 * @notapi
 */
static inline systime_t st_lld_get_counter(void) {

  return (systime_t)STM32_ST_TIM->CNT;
}

/**
 * @brief   Starts the alarm.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 *
 * @param[in] abstime   the time to be set for the first alarm
 *
 * @notapi
 */
static inline void st_lld_start_alarm(systime_t abstime) {

  STM32_ST_TIM->CCR[0] = (uint32_t)abstime;
  STM32_ST_TIM->SR     = 0;
#if ST_LLD_NUM_ALARMS == 1
  STM32_ST_TIM->DIER   = STM32_TIM_DIER_CC1IE;
#else
  STM32_ST_TIM->DIER  |= STM32_TIM_DIER_CC1IE;
#endif
}

/**
 * @brief   Stops the alarm interrupt.
 *
 * @notapi
 */
static inline void st_lld_stop_alarm(void) {

#if ST_LLD_NUM_ALARMS == 1
  STM32_ST_TIM->DIER = 0U;
#else
 STM32_ST_TIM->DIER &= ~STM32_TIM_DIER_CC1IE;
#endif
}

/**
 * @brief   Sets the alarm time.
 *
 * @param[in] abstime   the time to be set for the next alarm
 *
 * @notapi
 */
static inline void st_lld_set_alarm(systime_t abstime) {

  STM32_ST_TIM->CCR[0] = (uint32_t)abstime;
}

/**
 * @brief   Returns the current alarm time.
 *
 * @return              The currently set alarm time.
 *
 * @notapi
 */
static inline systime_t st_lld_get_alarm(void) {

  return (systime_t)STM32_ST_TIM->CCR[0];
}

/**
 * @brief   Determines if the alarm is active.
 *
 * @return              The alarm status.
 * @retval false        if the alarm is not active.
 * @retval true         is the alarm is active
 *
 * @notapi
 */
static inline bool st_lld_is_alarm_active(void) {

  return (bool)((STM32_ST_TIM->DIER & STM32_TIM_DIER_CC1IE) != 0);
}

#if (ST_LLD_NUM_ALARMS > 1) || defined(__DOXYGEN__)
/**
 * @brief   Starts an alarm.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] abstime   the time to be set for the first alarm
 * @param[in] alarm     alarm channel number
 *
 * @notapi
 */
static inline void st_lld_start_alarm_n(unsigned alarm, systime_t abstime) {

  STM32_ST_TIM->CCR[alarm] = (uint32_t)abstime;
  STM32_ST_TIM->SR         = 0;
  STM32_ST_TIM->DIER      |= (STM32_TIM_DIER_CC1IE << alarm);
}

/**
 * @brief   Stops an alarm interrupt.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] alarm     alarm channel number
 *
 * @notapi
 */
static inline void st_lld_stop_alarm_n(unsigned alarm) {

  STM32_ST_TIM->DIER &= ~(STM32_TIM_DIER_CC1IE << alarm);
}

/**
 * @brief   Sets an alarm time.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] alarm     alarm channel number
 * @param[in] abstime   the time to be set for the next alarm
 *
 * @notapi
 */
static inline void st_lld_set_alarm_n(unsigned alarm, systime_t abstime) {

  STM32_ST_TIM->CCR[alarm] = (uint32_t)abstime;
}

/**
 * @brief   Returns an alarm current time.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] alarm     alarm channel number
 * @return              The currently set alarm time.
 *
 * @notapi
 */
static inline systime_t st_lld_get_alarm_n(unsigned alarm) {

  return (systime_t)STM32_ST_TIM->CCR[alarm];
}

/**
 * @brief   Determines if an alarm is active.
 *
 * @param[in] alarm     alarm channel number
 * @return              The alarm status.
 * @retval false        if the alarm is not active.
 * @retval true         is the alarm is active
 *
 * @notapi
 */
static inline bool st_lld_is_alarm_active_n(unsigned alarm) {

  return (bool)((STM32_ST_TIM->DIER & (STM32_TIM_DIER_CC1IE << alarm)) != 0);
}
#endif /* ST_LLD_NUM_ALARMS > 1 */

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#endif /* HAL_ST_LLD_H */

/** @} */
