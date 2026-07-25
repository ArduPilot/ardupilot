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
 * @file    STM32U3xx/stm32_limits.h
 * @brief   STM32U3xx device limits header.
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
#define STM32_REGULATORS_TRANSITION_TIME    (21U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Worst case startup time of all oscillators.
 * @note    The value is taken from HSE which is the slowest one according
 *          to datasheet.
 * @note    LSE not included because it is way slower, in the order of
 *          seconds according to datasheet.
 */
#define STM32_OSCILLATORS_STARTUP_TIME      (2000U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Stabilization time of the MSI PLLs.
 * @note    This value does not assume the MSIPLLFAST bit.
 */
#define STM32_MSIPLL_STARTUP_TIME           (800U * STM32_RELAXED_TIMEOUT_FACTOR)

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
#define STM32_RANGE1_BOOSTEN_THRESHOLD      0U
#define STM32_RANGE1_SYSCLK_MAX             96010000
#define STM32_RANGE1_HSECLK_MAX             50000000
#define STM32_RANGE1_HSECLK_BYP_MAX         50000000
#define STM32_RANGE1_HSECLK_MIN             4000000
#define STM32_RANGE1_HSECLK_BYP_MIN         4000000
#define STM32_RANGE1_LSECLK_MAX             32768
#define STM32_RANGE1_LSECLK_BYP_MAX         40000
#define STM32_RANGE1_LSECLK_MIN             32768
#define STM32_RANGE1_LSECLK_BYP_MIN         5000
#define STM32_RANGE1_PCLK1_MAX              96010000
#define STM32_RANGE1_PCLK2_MAX              96010000
#define STM32_RANGE1_PCLK3_MAX              96010000
#define STM32_RANGE1_ADCCLK_MAX             55000000

#if ((STM32_FLASH_ACR & FLASH_ACR_LPM) == 0) || defined(__DOXYGEN__)
#define STM32_RANGE1_0WS_THRESHOLD          32005000
#define STM32_RANGE1_1WS_THRESHOLD          64007500
#define STM32_RANGE1_2WS_THRESHOLD          96010000
#define STM32_RANGE1_3WS_THRESHOLD          0
#define STM32_RANGE1_4WS_THRESHOLD          0
#else
#define STM32_RANGE1_0WS_THRESHOLD          20000000
#define STM32_RANGE1_1WS_THRESHOLD          40000000
#define STM32_RANGE1_2WS_THRESHOLD          60000000
#define STM32_RANGE1_3WS_THRESHOLD          80000000
#define STM32_RANGE1_4WS_THRESHOLD          96010000
#endif
/** @} */

/**
 * @name    Device Limits for VCore range 2
 * @{
 */
#define STM32_RANGE2_BOOSTEN_THRESHOLD      24000000
#define STM32_RANGE2_SYSCLK_MAX             48005000
#define STM32_RANGE2_HSECLK_MAX             48000000
#define STM32_RANGE2_HSECLK_BYP_MAX         48000000
#define STM32_RANGE2_HSECLK_MIN             4000000
#define STM32_RANGE2_HSECLK_BYP_MIN         4000000
#define STM32_RANGE2_LSECLK_MAX             32768
#define STM32_RANGE2_LSECLK_BYP_MAX         40000
#define STM32_RANGE2_LSECLK_MIN             32768
#define STM32_RANGE2_LSECLK_BYP_MIN         5000
#define STM32_RANGE2_PCLK1_MAX              48005000
#define STM32_RANGE2_PCLK2_MAX              48005000
#define STM32_RANGE2_PCLK3_MAX              48005000
#define STM32_RANGE2_ADCCLK_MAX             48005000

#if ((STM32_FLASH_ACR & FLASH_ACR_LPM) == 0) || defined(__DOXYGEN__)
#define STM32_RANGE2_0WS_THRESHOLD          16001250
#define STM32_RANGE2_1WS_THRESHOLD          32002500
#define STM32_RANGE2_2WS_THRESHOLD          48005000
#define STM32_RANGE2_3WS_THRESHOLD          0
#define STM32_RANGE2_4WS_THRESHOLD          0
#else
#define STM32_RANGE2_0WS_THRESHOLD          16001250
#define STM32_RANGE2_1WS_THRESHOLD          32002500
#define STM32_RANGE2_2WS_THRESHOLD          48005000
#define STM32_RANGE2_3WS_THRESHOLD          0
#define STM32_RANGE2_4WS_THRESHOLD          0
#endif
/** @} */

/**
 * @name    Device Limits for current Vcore settings
 * @{
 */
#if ((STM32_PWR_VOSR & PWR_VOSR_RANGE_Msk) == PWR_VOSR_RANGE1) || defined(__DOXYGEN__)
#define STM32_BOOSTEN_THRESHOLD             STM32_RANGE1_BOOSTEN_THRESHOLD
#define STM32_SYSCLK_MAX                    STM32_RANGE1_SYSCLK_MAX
#define STM32_HSECLK_MAX                    STM32_RANGE1_HSECLK_MAX
#define STM32_HSECLK_BYP_MAX                STM32_RANGE1_HSECLK_BYP_MAX
#define STM32_HSECLK_MIN                    STM32_RANGE1_HSECLK_MIN
#define STM32_HSECLK_BYP_MIN                STM32_RANGE1_HSECLK_BYP_MIN
#define STM32_LSECLK_MAX                    STM32_RANGE1_LSECLK_MAX
#define STM32_LSECLK_BYP_MAX                STM32_RANGE1_LSECLK_BYP_MAX
#define STM32_LSECLK_MIN                    STM32_RANGE1_LSECLK_MIN
#define STM32_LSECLK_BYP_MIN                STM32_RANGE1_LSECLK_BYP_MIN
#define STM32_PCLK1_MAX                     STM32_RANGE1_PCLK1_MAX
#define STM32_PCLK2_MAX                     STM32_RANGE1_PCLK2_MAX
#define STM32_PCLK3_MAX                     STM32_RANGE1_PCLK3_MAX
#define STM32_ADCCLK_MAX                    STM32_RANGE1_ADCCLK_MAX

#define STM32_0WS_THRESHOLD                 STM32_RANGE1_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD                 STM32_RANGE1_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD                 STM32_RANGE1_2WS_THRESHOLD
#define STM32_3WS_THRESHOLD                 STM32_RANGE1_3WS_THRESHOLD
#define STM32_4WS_THRESHOLD                 STM32_RANGE1_4WS_THRESHOLD
/** @} */

#elif (STM32_PWR_VOSR & PWR_VOSR_RANGE_Msk) == PWR_VOSR_RANGE2
#define STM32_BOOSTEN_THRESHOLD             STM32_RANGE2_BOOSTEN_THRESHOLD
#define STM32_SYSCLK_MAX                    STM32_RANGE2_SYSCLK_MAX
#define STM32_HSECLK_MAX                    STM32_RANGE2_HSECLK_MAX
#define STM32_HSECLK_BYP_MAX                STM32_RANGE2_HSECLK_BYP_MAX
#define STM32_HSECLK_MIN                    STM32_RANGE2_HSECLK_MIN
#define STM32_HSECLK_BYP_MIN                STM32_RANGE2_HSECLK_BYP_MIN
#define STM32_LSECLK_MAX                    STM32_RANGE2_LSECLK_MAX
#define STM32_LSECLK_BYP_MAX                STM32_RANGE2_LSECLK_BYP_MAX
#define STM32_LSECLK_MIN                    STM32_RANGE2_LSECLK_MIN
#define STM32_LSECLK_BYP_MIN                STM32_RANGE2_LSECLK_BYP_MIN
#define STM32_PCLK1_MAX                     STM32_RANGE2_PCLK1_MAX
#define STM32_PCLK2_MAX                     STM32_RANGE2_PCLK2_MAX
#define STM32_PCLK3_MAX                     STM32_RANGE2_PCLK3_MAX
#define STM32_ADCCLK_MAX                    STM32_RANGE2_ADCCLK_MAX

#define STM32_0WS_THRESHOLD                 STM32_RANGE2_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD                 STM32_RANGE2_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD                 STM32_RANGE2_2WS_THRESHOLD
#define STM32_3WS_THRESHOLD                 STM32_RANGE2_3WS_THRESHOLD
#define STM32_4WS_THRESHOLD                 STM32_RANGE2_4WS_THRESHOLD

#else
#error "invalid STM32_PWR_VOSR settings specified"
#endif

/**
 * @name    Peripherals limits
 * @{
 */
#define STM32_SDMMC_MAXCLK                  STM32_SYSCLK_MAX
/** @} */

#endif /* STM32_LIMITS_H */

/** @} */
