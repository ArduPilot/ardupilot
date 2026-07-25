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
 * @file    STM32U0xx/stm32_limits.h
 * @brief   STM32U0xx device limits header.
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
#define STM32_RELAXED_TIMEOUT_FACTOR        5U

/**
 * @brief   Regulators transition time.
 */
#define STM32_REGULATORS_TRANSITION_TIME    (40U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Worst case startup time of HSI, HSE and MSI.
 * @note    The value is taken from HSE which is the slowest one according
 *          to datasheet.
 */
#define STM32_HSI_HSE_MSI_STARTUP_TIME      (2000U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Worst case startup time of HSI48.
 */
#define STM32_HSI48_STARTUP_TIME            (6U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Stabilization time of the MSI PLLs.
 * @note    This value does not assume the MSIPLLFAST bit.
 */
#define STM32_PLL_STARTUP_TIME              (40U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Time required for system time switch (RCC_CFGR1.SW).
 * @note    This value is estimated and kept "safe", there is no
 *          specification.
 */
#define STM32_SYSCLK_SWITCH_TIME            (50U * STM32_RELAXED_TIMEOUT_FACTOR)
/** @} */

/**
 * @name    Device Limits for VCore range 1
 * @{
 */
#define STM32_RANGE1_SYSCLK_MAX             56100000 /*Some tolerance for MSIPLL.*/
#define STM32_RANGE1_HSECLK_MAX             48000000
#define STM32_RANGE1_HSECLK_BYP_MAX         48000000
#define STM32_RANGE1_HSECLK_MIN             4000000
#define STM32_RANGE1_HSECLK_BYP_MIN         8000000
#define STM32_RANGE1_LSECLK_MAX             32768
#define STM32_RANGE1_LSECLK_BYP_MAX         1000000
#define STM32_RANGE1_LSECLK_MIN             32768
#define STM32_RANGE1_LSECLK_BYP_MIN         32768
#define STM32_RANGE1_PLLIN_MAX              16050000 /*Some tolerance for MSIPLL.*/
#define STM32_RANGE1_PLLIN_MIN              2660000
#define STM32_RANGE1_PLLVCO_MAX             344000000
#define STM32_RANGE1_PLLVCO_MIN             96000000
#define STM32_RANGE1_PLLP_MAX               128000000
#define STM32_RANGE1_PLLP_MIN               3090000
#define STM32_RANGE1_PLLQ_MAX               128000000
#define STM32_RANGE1_PLLQ_MIN               12000000
#define STM32_RANGE1_PLLR_MAX               64000000
#define STM32_RANGE1_PLLR_MIN               12000000
#define STM32_RANGE1_PCLK_MAX               64000000
#define STM32_RANGE1_ADCCLK_MAX             35000000

#define STM32_RANGE1_0WS_THRESHOLD          24100000 /*Some tolerance for MSIPLL.*/
#define STM32_RANGE1_1WS_THRESHOLD          48100000 /*Some tolerance for MSIPLL.*/
#define STM32_RANGE1_2WS_THRESHOLD          56100000 /*Some tolerance for MSIPLL.*/
/** @} */

/**
 * @name    Device Limits for VCore range 2
 * @{
 */
#define STM32_RANGE2_SYSCLK_MAX             16050000 /*Some tolerance for MSIPLL.*/
#define STM32_RANGE2_HSECLK_MAX             19000000
#define STM32_RANGE2_HSECLK_BYP_MAX         19000000
#define STM32_RANGE2_HSECLK_MIN             4000000
#define STM32_RANGE2_HSECLK_BYP_MIN         8000000
#define STM32_RANGE2_LSECLK_MAX             32768
#define STM32_RANGE2_LSECLK_BYP_MAX         1000000
#define STM32_RANGE2_LSECLK_MIN             32768
#define STM32_RANGE2_LSECLK_BYP_MIN         32768
#define STM32_RANGE2_PLLIN_MAX              16050000 /*Some tolerance for MSIPLL.*/
#define STM32_RANGE2_PLLIN_MIN              2660000
#define STM32_RANGE2_PLLVCO_MAX             128000000
#define STM32_RANGE2_PLLVCO_MIN             96000000
#define STM32_RANGE2_PLLP_MAX               40000000
#define STM32_RANGE2_PLLP_MIN               3090000
#define STM32_RANGE2_PLLQ_MAX               33000000
#define STM32_RANGE2_PLLQ_MIN               12000000
#define STM32_RANGE2_PLLR_MAX               16000000
#define STM32_RANGE2_PLLR_MIN               12000000
#define STM32_RANGE2_PCLK_MAX               64000000
#define STM32_RANGE2_ADCCLK_MAX             16050000 /*Some tolerance for MSIPLL.*/

#define STM32_RANGE2_0WS_THRESHOLD          8000000
#define STM32_RANGE2_1WS_THRESHOLD          16050000 /*Some tolerance for MSIPLL.*/
#define STM32_RANGE2_2WS_THRESHOLD          18050000 /*Some tolerance for MSIPLL.*/
/** @} */

/**
 * @name    Device Limits for current Vcore settings
 * @{
 */
#if ((STM32_PWR_CR1 & PWR_CR1_VOS_Msk) == PWR_CR1_VOS_RANGE1) || defined(__DOXYGEN__)
#define STM32_SYSCLK_MAX                    STM32_RANGE1_SYSCLK_MAX
#define STM32_HSECLK_MAX                    STM32_RANGE1_HSECLK_MAX
#define STM32_HSECLK_BYP_MAX                STM32_RANGE1_HSECLK_BYP_MAX
#define STM32_HSECLK_MIN                    STM32_RANGE1_HSECLK_MIN
#define STM32_HSECLK_BYP_MIN                STM32_RANGE1_HSECLK_BYP_MIN
#define STM32_LSECLK_MAX                    STM32_RANGE1_LSECLK_MAX
#define STM32_LSECLK_BYP_MAX                STM32_RANGE1_LSECLK_BYP_MAX
#define STM32_LSECLK_MIN                    STM32_RANGE1_LSECLK_MIN
#define STM32_LSECLK_BYP_MIN                STM32_RANGE1_LSECLK_BYP_MIN
#define STM32_PLLIN_MAX                     STM32_RANGE1_PLLIN_MAX
#define STM32_PLLIN_MIN                     STM32_RANGE1_PLLIN_MIN
#define STM32_PLLVCO_MAX                    STM32_RANGE1_PLLVCO_MAX
#define STM32_PLLVCO_MIN                    STM32_RANGE1_PLLVCO_MIN
#define STM32_PLLP_MAX                      STM32_RANGE1_PLLP_MAX
#define STM32_PLLP_MIN                      STM32_RANGE1_PLLP_MIN
#define STM32_PLLQ_MAX                      STM32_RANGE1_PLLQ_MAX
#define STM32_PLLQ_MIN                      STM32_RANGE1_PLLQ_MIN
#define STM32_PLLR_MAX                      STM32_RANGE1_PLLR_MAX
#define STM32_PLLR_MIN                      STM32_RANGE1_PLLR_MIN
#define STM32_PCLK_MAX                      STM32_RANGE1_PCLK_MAX
#define STM32_ADCCLK_MAX                    STM32_RANGE1_ADCCLK_MAX

#define STM32_0WS_THRESHOLD                 STM32_RANGE1_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD                 STM32_RANGE1_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD                 STM32_RANGE1_2WS_THRESHOLD

#elif (STM32_PWR_CR1 & PWR_CR1_VOS_Msk) == PWR_CR1_VOS_RANGE2
#define STM32_SYSCLK_MAX                    STM32_RANGE2_SYSCLK_MAX
#define STM32_HSECLK_MAX                    STM32_RANGE2_HSECLK_MAX
#define STM32_HSECLK_BYP_MAX                STM32_RANGE2_HSECLK_BYP_MAX
#define STM32_HSECLK_MIN                    STM32_RANGE2_HSECLK_MIN
#define STM32_HSECLK_BYP_MIN                STM32_RANGE2_HSECLK_BYP_MIN
#define STM32_LSECLK_MAX                    STM32_RANGE2_LSECLK_MAX
#define STM32_LSECLK_BYP_MAX                STM32_RANGE2_LSECLK_BYP_MAX
#define STM32_LSECLK_MIN                    STM32_RANGE2_LSECLK_MIN
#define STM32_LSECLK_BYP_MIN                STM32_RANGE2_LSECLK_BYP_MIN
#define STM32_PLLIN_MAX                     STM32_RANGE2_PLLIN_MAX
#define STM32_PLLIN_MIN                     STM32_RANGE2_PLLIN_MIN
#define STM32_PLLVCO_MAX                    STM32_RANGE2_PLLVCO_MAX
#define STM32_PLLVCO_MIN                    STM32_RANGE2_PLLVCO_MIN
#define STM32_PLLP_MAX                      STM32_RANGE2_PLLP_MAX
#define STM32_PLLP_MIN                      STM32_RANGE2_PLLP_MIN
#define STM32_PLLQ_MAX                      STM32_RANGE2_PLLQ_MAX
#define STM32_PLLQ_MIN                      STM32_RANGE2_PLLQ_MIN
#define STM32_PLLR_MAX                      STM32_RANGE2_PLLR_MAX
#define STM32_PLLR_MIN                      STM32_RANGE2_PLLR_MIN
#define STM32_PCLK_MAX                      STM32_RANGE2_PCLK_MAX
#define STM32_ADCCLK_MAX                    STM32_RANGE2_ADCCLK_MAX

#define STM32_0WS_THRESHOLD                 STM32_RANGE2_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD                 STM32_RANGE2_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD                 STM32_RANGE2_2WS_THRESHOLD

#else
#error "invalid STM32_PWR_VOSR settings specified"
#endif
/** @} */

/**
 * @name    PLL dividers ranges
 * @{
 */
#define STM32_PLLM_VALUE_MAX                8
#define STM32_PLLM_VALUE_MIN                1

#define STM32_PLLN_VALUE_MAX                127
#define STM32_PLLN_VALUE_MIN                4

#define STM32_PLLP_VALUE_MAX                32
#define STM32_PLLP_VALUE_MIN                2

#define STM32_PLLQ_VALUE_MAX                8
#define STM32_PLLQ_VALUE_MIN                2

#define STM32_PLLR_VALUE_MAX                8
#define STM32_PLLR_VALUE_MIN                2
/** @} */

/**
 * @name    Peripherals limits
 * @{
 */
#define STM32_SDMMC_MAXCLK              STM32_SYSCLK_MAX
/** @} */

#endif /* STM32_LIMITS_H */

/** @} */
