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
 * @file    STM32G4xx_ALT/stm32_limits.h
 * @brief   STM32G4xx ALT device limits header.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef STM32_LIMITS_H
#define STM32_LIMITS_H

/**
 * @name    Activation times in microseconds
 */
/**
 * @brief   Timeout tolerance.
 */
#define STM32_RELAXED_TIMEOUT_FACTOR    5U

/**
 * @brief   Worst case startup time of HSI16.
 */
#define STM32_HSI_STARTUP_TIME          (4U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Worst case startup time of all oscillators.
 * @note    The value is taken from HSE which is the slowest one according
 *          to datasheet.
 */
#define STM32_OSCILLATORS_STARTUP_TIME  (2000U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Worst case startup time of HSI48.
 */
#define STM32_HSI48_STARTUP_TIME        (6U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Stabilization time of PLL.
 */
#define STM32_PLL_STARTUP_TIME          (40U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Time required for system time switch (RCC_CFGR.SW).
 * @note    This value is estimated and kept "safe", there is no
 *          specification.
 */
#define STM32_SYSCLK_SWITCH_TIME        (50U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Regulators transition time.
 */
#define STM32_REGULATORS_TRANSITION_TIME (40U * STM32_RELAXED_TIMEOUT_FACTOR)
/** @} */

/**
 * @name    System Limits for VOS range 1 with boost
 * @{
 */
#define STM32_BOOST_SYSCLK_MAX          170000000
#define STM32_BOOST_HSECLK_MAX          48000000
#define STM32_BOOST_HSECLK_BYP_MAX      48000000
#define STM32_BOOST_HSECLK_MIN          8000000
#define STM32_BOOST_HSECLK_BYP_MIN      8000000
#define STM32_BOOST_LSECLK_MAX          32768
#define STM32_BOOST_LSECLK_BYP_MAX      1000000
#define STM32_BOOST_LSECLK_MIN          32768
#define STM32_BOOST_LSECLK_BYP_MIN      32768
#define STM32_BOOST_PLLIN_MAX           16000000
#define STM32_BOOST_PLLIN_MIN           2660000
#define STM32_BOOST_PLLVCO_MAX          344000000
#define STM32_BOOST_PLLVCO_MIN          96000000
#define STM32_BOOST_PLLP_MAX            170000000
#define STM32_BOOST_PLLP_MIN            2064500
#define STM32_BOOST_PLLQ_MAX            170000000
#define STM32_BOOST_PLLQ_MIN            8000000
#define STM32_BOOST_PLLR_MAX            170000000
#define STM32_BOOST_PLLR_MIN            8000000
#define STM32_BOOST_PCLK1_MAX           170000000
#define STM32_BOOST_PCLK2_MAX           170000000
#define STM32_BOOST_ADCCLK_MAX          60000000

#define STM32_BOOST_0WS_THRESHOLD       34000000
#define STM32_BOOST_1WS_THRESHOLD       68000000
#define STM32_BOOST_2WS_THRESHOLD       102000000
#define STM32_BOOST_3WS_THRESHOLD       136000000
#define STM32_BOOST_4WS_THRESHOLD       170000000
/** @} */

/**
 * @name    System Limits for VOS range 1 without boost
 * @{
 */
#define STM32_VOS1_SYSCLK_MAX           150000000
#define STM32_VOS1_HSECLK_MAX           48000000
#define STM32_VOS1_HSECLK_BYP_MAX       48000000
#define STM32_VOS1_HSECLK_MIN           8000000
#define STM32_VOS1_HSECLK_BYP_MIN       8000000
#define STM32_VOS1_LSECLK_MAX           32768
#define STM32_VOS1_LSECLK_BYP_MAX       1000000
#define STM32_VOS1_LSECLK_MIN           32768
#define STM32_VOS1_LSECLK_BYP_MIN       32768
#define STM32_VOS1_PLLIN_MAX            16000000
#define STM32_VOS1_PLLIN_MIN            2660000
#define STM32_VOS1_PLLVCO_MAX           344000000
#define STM32_VOS1_PLLVCO_MIN           96000000
#define STM32_VOS1_PLLP_MAX             150000000
#define STM32_VOS1_PLLP_MIN             2064500
#define STM32_VOS1_PLLQ_MAX             150000000
#define STM32_VOS1_PLLQ_MIN             8000000
#define STM32_VOS1_PLLR_MAX             150000000
#define STM32_VOS1_PLLR_MIN             8000000
#define STM32_VOS1_PCLK1_MAX            150000000
#define STM32_VOS1_PCLK2_MAX            150000000
#define STM32_VOS1_ADCCLK_MAX           60000000

#define STM32_VOS1_0WS_THRESHOLD        30000000
#define STM32_VOS1_1WS_THRESHOLD        60000000
#define STM32_VOS1_2WS_THRESHOLD        90000000
#define STM32_VOS1_3WS_THRESHOLD        120000000
#define STM32_VOS1_4WS_THRESHOLD        150000000
/** @} */

/**
 * @name    System Limits for VOS range 2
 * @{
 */
#define STM32_VOS2_SYSCLK_MAX           26000000
#define STM32_VOS2_HSECLK_MAX           26000000
#define STM32_VOS2_HSECLK_BYP_MAX       26000000
#define STM32_VOS2_HSECLK_MIN           8000000
#define STM32_VOS2_HSECLK_BYP_MIN       8000000
#define STM32_VOS2_LSECLK_MAX           32768
#define STM32_VOS2_LSECLK_BYP_MAX       1000000
#define STM32_VOS2_LSECLK_MIN           32768
#define STM32_VOS2_LSECLK_BYP_MIN       32768
#define STM32_VOS2_PLLIN_MAX            16000000
#define STM32_VOS2_PLLIN_MIN            2660000
#define STM32_VOS2_PLLVCO_MAX           128000000
#define STM32_VOS2_PLLVCO_MIN           96000000
#define STM32_VOS2_PLLP_MAX             26000000
#define STM32_VOS2_PLLP_MIN             2064500
#define STM32_VOS2_PLLQ_MAX             26000000
#define STM32_VOS2_PLLQ_MIN             8000000
#define STM32_VOS2_PLLR_MAX             26000000
#define STM32_VOS2_PLLR_MIN             8000000
#define STM32_VOS2_PCLK1_MAX            26000000
#define STM32_VOS2_PCLK2_MAX            26000000
#define STM32_VOS2_ADCCLK_MAX           26000000

#define STM32_VOS2_0WS_THRESHOLD        12000000
#define STM32_VOS2_1WS_THRESHOLD        24000000
#define STM32_VOS2_2WS_THRESHOLD        26000000
#define STM32_VOS2_3WS_THRESHOLD        0
#define STM32_VOS2_4WS_THRESHOLD        0
/** @} */

/* Voltage related limits.*/
#if (STM32_VOS == STM32_VOS_RANGE1) || defined(__DOXYGEN__)
#if STM32_PWR_BOOST || defined(__DOXYGEN__)
#define STM32_SYSCLK_MAX                STM32_BOOST_SYSCLK_MAX
#define STM32_HSECLK_MAX                STM32_BOOST_HSECLK_MAX
#define STM32_HSECLK_BYP_MAX            STM32_BOOST_HSECLK_BYP_MAX
#define STM32_HSECLK_MIN                STM32_BOOST_HSECLK_MIN
#define STM32_HSECLK_BYP_MIN            STM32_BOOST_HSECLK_BYP_MIN
#define STM32_LSECLK_MAX                STM32_BOOST_LSECLK_MAX
#define STM32_LSECLK_BYP_MAX            STM32_BOOST_LSECLK_BYP_MAX
#define STM32_LSECLK_MIN                STM32_BOOST_LSECLK_MIN
#define STM32_LSECLK_BYP_MIN            STM32_BOOST_LSECLK_BYP_MIN
#define STM32_PLLIN_MAX                 STM32_BOOST_PLLIN_MAX
#define STM32_PLLIN_MIN                 STM32_BOOST_PLLIN_MIN
#define STM32_PLLVCO_MAX                STM32_BOOST_PLLVCO_MAX
#define STM32_PLLVCO_MIN                STM32_BOOST_PLLVCO_MIN
#define STM32_PLLP_MAX                  STM32_BOOST_PLLP_MAX
#define STM32_PLLP_MIN                  STM32_BOOST_PLLP_MIN
#define STM32_PLLQ_MAX                  STM32_BOOST_PLLQ_MAX
#define STM32_PLLQ_MIN                  STM32_BOOST_PLLQ_MIN
#define STM32_PLLR_MAX                  STM32_BOOST_PLLR_MAX
#define STM32_PLLR_MIN                  STM32_BOOST_PLLR_MIN
#define STM32_PCLK1_MAX                 STM32_BOOST_PCLK1_MAX
#define STM32_PCLK2_MAX                 STM32_BOOST_PCLK2_MAX
#define STM32_ADCCLK_MAX                STM32_BOOST_ADCCLK_MAX

#define STM32_0WS_THRESHOLD             STM32_BOOST_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD             STM32_BOOST_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD             STM32_BOOST_2WS_THRESHOLD
#define STM32_3WS_THRESHOLD             STM32_BOOST_3WS_THRESHOLD
#define STM32_4WS_THRESHOLD             STM32_BOOST_4WS_THRESHOLD
#define STM32_5WS_THRESHOLD             STM32_BOOST_5WS_THRESHOLD
#define STM32_6WS_THRESHOLD             STM32_BOOST_6WS_THRESHOLD
#define STM32_7WS_THRESHOLD             STM32_BOOST_7WS_THRESHOLD
#define STM32_8WS_THRESHOLD             STM32_BOOST_8WS_THRESHOLD

#else /* !STM32_PWR_BOOST */
#define STM32_SYSCLK_MAX                STM32_VOS1_SYSCLK_MAX
#define STM32_HSECLK_MAX                STM32_VOS1_HSECLK_MAX
#define STM32_HSECLK_BYP_MAX            STM32_VOS1_HSECLK_BYP_MAX
#define STM32_HSECLK_MIN                STM32_VOS1_HSECLK_MIN
#define STM32_HSECLK_BYP_MIN            STM32_VOS1_HSECLK_BYP_MIN
#define STM32_LSECLK_MAX                STM32_VOS1_LSECLK_MAX
#define STM32_LSECLK_BYP_MAX            STM32_VOS1_LSECLK_BYP_MAX
#define STM32_LSECLK_MIN                STM32_VOS1_LSECLK_MIN
#define STM32_LSECLK_BYP_MIN            STM32_VOS1_LSECLK_BYP_MIN
#define STM32_PLLIN_MAX                 STM32_VOS1_PLLIN_MAX
#define STM32_PLLIN_MIN                 STM32_VOS1_PLLIN_MIN
#define STM32_PLLVCO_MAX                STM32_VOS1_PLLVCO_MAX
#define STM32_PLLVCO_MIN                STM32_VOS1_PLLVCO_MIN
#define STM32_PLLP_MAX                  STM32_VOS1_PLLP_MAX
#define STM32_PLLP_MIN                  STM32_VOS1_PLLP_MIN
#define STM32_PLLQ_MAX                  STM32_VOS1_PLLQ_MAX
#define STM32_PLLQ_MIN                  STM32_VOS1_PLLQ_MIN
#define STM32_PLLR_MAX                  STM32_VOS1_PLLR_MAX
#define STM32_PLLR_MIN                  STM32_VOS1_PLLR_MIN
#define STM32_PCLK1_MAX                 STM32_VOS1_PCLK1_MAX
#define STM32_PCLK2_MAX                 STM32_VOS1_PCLK2_MAX
#define STM32_ADCCLK_MAX                STM32_VOS1_ADCCLK_MAX

#define STM32_0WS_THRESHOLD             STM32_VOS1_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD             STM32_VOS1_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD             STM32_VOS1_2WS_THRESHOLD
#define STM32_3WS_THRESHOLD             STM32_VOS1_3WS_THRESHOLD
#define STM32_4WS_THRESHOLD             STM32_VOS1_4WS_THRESHOLD
#define STM32_5WS_THRESHOLD             STM32_VOS1_5WS_THRESHOLD
#define STM32_6WS_THRESHOLD             STM32_VOS1_6WS_THRESHOLD
#define STM32_7WS_THRESHOLD             STM32_VOS1_7WS_THRESHOLD
#define STM32_8WS_THRESHOLD             STM32_VOS1_8WS_THRESHOLD
#endif /* !STM32_PWR_BOOST */

#elif STM32_VOS == STM32_VOS_RANGE2
#define STM32_SYSCLK_MAX                STM32_VOS2_SYSCLK_MAX
#define STM32_HSECLK_MAX                STM32_VOS2_HSECLK_MAX
#define STM32_HSECLK_BYP_MAX            STM32_VOS2_HSECLK_BYP_MAX
#define STM32_HSECLK_MIN                STM32_VOS2_HSECLK_MIN
#define STM32_HSECLK_BYP_MIN            STM32_VOS2_HSECLK_BYP_MIN
#define STM32_LSECLK_MAX                STM32_VOS2_LSECLK_MAX
#define STM32_LSECLK_BYP_MAX            STM32_VOS2_LSECLK_BYP_MAX
#define STM32_LSECLK_MIN                STM32_VOS2_LSECLK_MIN
#define STM32_LSECLK_BYP_MIN            STM32_VOS2_LSECLK_BYP_MIN
#define STM32_PLLIN_MAX                 STM32_VOS2_PLLIN_MAX
#define STM32_PLLIN_MIN                 STM32_VOS2_PLLIN_MIN
#define STM32_PLLVCO_MAX                STM32_VOS2_PLLVCO_MAX
#define STM32_PLLVCO_MIN                STM32_VOS2_PLLVCO_MIN
#define STM32_PLLP_MAX                  STM32_VOS2_PLLP_MAX
#define STM32_PLLP_MIN                  STM32_VOS2_PLLP_MIN
#define STM32_PLLQ_MAX                  STM32_VOS2_PLLQ_MAX
#define STM32_PLLQ_MIN                  STM32_VOS2_PLLQ_MIN
#define STM32_PLLR_MAX                  STM32_VOS2_PLLR_MAX
#define STM32_PLLR_MIN                  STM32_VOS2_PLLR_MIN
#define STM32_PCLK1_MAX                 STM32_VOS2_PCLK1_MAX
#define STM32_PCLK2_MAX                 STM32_VOS2_PCLK2_MAX
#define STM32_ADCCLK_MAX                STM32_VOS2_ADCCLK_MAX

#define STM32_0WS_THRESHOLD             STM32_VOS2_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD             STM32_VOS2_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD             STM32_VOS2_2WS_THRESHOLD
#define STM32_3WS_THRESHOLD             STM32_VOS2_3WS_THRESHOLD
#define STM32_4WS_THRESHOLD             STM32_VOS2_4WS_THRESHOLD
#define STM32_5WS_THRESHOLD             STM32_VOS2_5WS_THRESHOLD
#define STM32_6WS_THRESHOLD             STM32_VOS2_6WS_THRESHOLD
#define STM32_7WS_THRESHOLD             STM32_VOS2_7WS_THRESHOLD
#define STM32_8WS_THRESHOLD             STM32_VOS2_8WS_THRESHOLD

#else
#error "invalid STM32_VOS value specified"
#endif

#endif /* STM32_LIMITS_H */

/** @} */
