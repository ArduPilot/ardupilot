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
 * @file    STM32H5xx/stm32_limits.h
 * @brief   STM32H5xx device limits header.
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
 * @brief   Worst case startup and stabilization time of HSI oscillator.
 */
#define STM32_HSI_STARTUP_TIME              (8U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   HSIDIV propagation time.
 * @note    This value is estimated and kept "safe", there is no
 *          specification.
 */
#define STM32_HSIDIV_PROPAGATION_TIME       (10U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Worst case startup time of all oscillators controlled in the
 *          RCC_CR register.
 * @note    The value is taken from HSE which is the slowest one according
 *          to datasheet.
 */
#define STM32_OSCILLATORS_STARTUP_TIME      (2000U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Stabilization time of PLLs.
 */
#define STM32_PLL_STARTUP_TIME              (120U * STM32_RELAXED_TIMEOUT_FACTOR)

/**
 * @brief   Time required for system time switch (RCC_CFGR1.SW).
 * @note    This value is estimated and kept "safe", there is no
 *          specification.
 */
#define STM32_SYSCLK_SWITCH_TIME            (50U * STM32_RELAXED_TIMEOUT_FACTOR)
/** @} */

/**
 * @name    Device Limits for VOS range 0
 * @{
 */
#define STM32_VOS0_SYSCLK_MAX           250000000
#define STM32_VOS0_HSECLK_MAX           50000000
#define STM32_VOS0_HSECLK_BYP_MAX       50000000
#define STM32_VOS0_HSECLK_MIN           4000000
#define STM32_VOS0_HSECLK_BYP_MIN       4000000
#define STM32_VOS0_LSECLK_MAX           32768
#define STM32_VOS0_LSECLK_BYP_MAX       1000000
#define STM32_VOS0_LSECLK_MIN           32768
#define STM32_VOS0_LSECLK_BYP_MIN       32768
#define STM32_VOS0_PLLP_MAX             250000000
#define STM32_VOS0_PLLP_MIN             1000000
#define STM32_VOS0_PLLQ_MAX             250000000
#define STM32_VOS0_PLLQ_MIN             1000000
#define STM32_VOS0_PLLR_MAX             250000000
#define STM32_VOS0_PLLR_MIN             1000000
#define STM32_VOS0_PCLK1_MAX            250000000
#define STM32_VOS0_PCLK2_MAX            250000000
#define STM32_VOS0_PCLK3_MAX            250000000
#define STM32_VOS0_ADCCLK_MAX           125000000

#define STM32_VOS0_0WS_THRESHOLD        42000000
#define STM32_VOS0_1WS_THRESHOLD        84000000
#define STM32_VOS0_2WS_THRESHOLD        126000000
#define STM32_VOS0_3WS_THRESHOLD        168000000
#define STM32_VOS0_4WS_THRESHOLD        210000000
#define STM32_VOS0_5WS_THRESHOLD        250000000
/** @} */

/**
 * @name    Device Limits for VOS range 1
 * @{
 */
#define STM32_VOS1_SYSCLK_MAX           200000000
#define STM32_VOS1_HSECLK_MAX           50000000
#define STM32_VOS1_HSECLK_BYP_MAX       50000000
#define STM32_VOS1_HSECLK_MIN           4000000
#define STM32_VOS1_HSECLK_BYP_MIN       4000000
#define STM32_VOS1_LSECLK_MAX           32768
#define STM32_VOS1_LSECLK_BYP_MAX       1000000
#define STM32_VOS1_LSECLK_MIN           32768
#define STM32_VOS1_LSECLK_BYP_MIN       32768
#define STM32_VOS1_PLLP_MAX             200000000
#define STM32_VOS1_PLLP_MIN             1000000
#define STM32_VOS1_PLLQ_MAX             200000000
#define STM32_VOS1_PLLQ_MIN             1000000
#define STM32_VOS1_PLLR_MAX             200000000
#define STM32_VOS1_PLLR_MIN             1000000
#define STM32_VOS1_PCLK1_MAX            200000000
#define STM32_VOS1_PCLK2_MAX            200000000
#define STM32_VOS1_PCLK3_MAX            200000000
#define STM32_VOS1_ADCCLK_MAX           100000000

#define STM32_VOS1_0WS_THRESHOLD        34000000
#define STM32_VOS1_1WS_THRESHOLD        68000000
#define STM32_VOS1_2WS_THRESHOLD        102000000
#define STM32_VOS1_3WS_THRESHOLD        136000000
#define STM32_VOS1_4WS_THRESHOLD        170000000
#define STM32_VOS1_5WS_THRESHOLD        200000000
/** @} */

/**
 * @name    Device Limits for VOS range 2
 * @{
 */
#define STM32_VOS2_SYSCLK_MAX           150000000
#define STM32_VOS2_HSECLK_MAX           50000000
#define STM32_VOS2_HSECLK_BYP_MAX       50000000
#define STM32_VOS2_HSECLK_MIN           4000000
#define STM32_VOS2_HSECLK_BYP_MIN       4000000
#define STM32_VOS2_LSECLK_MAX           32768
#define STM32_VOS2_LSECLK_BYP_MAX       1000000
#define STM32_VOS2_LSECLK_MIN           32768
#define STM32_VOS2_LSECLK_BYP_MIN       32768
#define STM32_VOS2_PLLP_MAX             150000000
#define STM32_VOS2_PLLP_MIN             1000000
#define STM32_VOS2_PLLQ_MAX             150000000
#define STM32_VOS2_PLLQ_MIN             1000000
#define STM32_VOS2_PLLR_MAX             150000000
#define STM32_VOS2_PLLR_MIN             1000000
#define STM32_VOS2_PCLK1_MAX            150000000
#define STM32_VOS2_PCLK2_MAX            150000000
#define STM32_VOS2_PCLK3_MAX            150000000
#define STM32_VOS2_ADCCLK_MAX           75000000

#define STM32_VOS2_0WS_THRESHOLD        30000000
#define STM32_VOS2_1WS_THRESHOLD        60000000
#define STM32_VOS2_2WS_THRESHOLD        90000000
#define STM32_VOS2_3WS_THRESHOLD        120000000
#define STM32_VOS2_4WS_THRESHOLD        150000000
#define STM32_VOS2_5WS_THRESHOLD        150000000
/** @} */

/**
 * @name    Device Limits for VOS range 3
 * @{
 */
#define STM32_VOS3_SYSCLK_MAX           100000000
#define STM32_VOS3_HSECLK_MAX           50000000
#define STM32_VOS3_HSECLK_BYP_MAX       50000000
#define STM32_VOS3_HSECLK_MIN           4000000
#define STM32_VOS3_HSECLK_BYP_MIN       4000000
#define STM32_VOS3_LSECLK_MAX           32768
#define STM32_VOS3_LSECLK_BYP_MAX       1000000
#define STM32_VOS3_LSECLK_MIN           32768
#define STM32_VOS3_LSECLK_BYP_MIN       32768
#define STM32_VOS3_PLLP_MAX             100000000
#define STM32_VOS3_PLLP_MIN             1000000
#define STM32_VOS3_PLLQ_MAX             100000000
#define STM32_VOS3_PLLQ_MIN             1000000
#define STM32_VOS3_PLLR_MAX             100000000
#define STM32_VOS3_PLLR_MIN             1000000
#define STM32_VOS3_PCLK1_MAX            100000000
#define STM32_VOS3_PCLK2_MAX            100000000
#define STM32_VOS3_PCLK3_MAX            100000000
#define STM32_VOS3_ADCCLK_MAX           50000000

#define STM32_VOS3_0WS_THRESHOLD        20000000
#define STM32_VOS3_1WS_THRESHOLD        40000000
#define STM32_VOS3_2WS_THRESHOLD        60000000
#define STM32_VOS3_3WS_THRESHOLD        80000000
#define STM32_VOS3_4WS_THRESHOLD        100000000
#define STM32_VOS3_5WS_THRESHOLD        100000000
/** @} */

/**
 * @name    Device Limits for current VOS settings
 * @{
 */
#if ((STM32_PWR_VOSCR & PWR_VOSCR_VOS_Msk) == PWR_VOSCR_VOS_RANGE0) || defined(__DOXYGEN__)
#define STM32_SYSCLK_MAX                STM32_VOS0_SYSCLK_MAX
#define STM32_HSECLK_MAX                STM32_VOS0_HSECLK_MAX
#define STM32_HSECLK_BYP_MAX            STM32_VOS0_HSECLK_BYP_MAX
#define STM32_HSECLK_MIN                STM32_VOS0_HSECLK_MIN
#define STM32_HSECLK_BYP_MIN            STM32_VOS0_HSECLK_BYP_MIN
#define STM32_LSECLK_MAX                STM32_VOS0_LSECLK_MAX
#define STM32_LSECLK_BYP_MAX            STM32_VOS0_LSECLK_BYP_MAX
#define STM32_LSECLK_MIN                STM32_VOS0_LSECLK_MIN
#define STM32_LSECLK_BYP_MIN            STM32_VOS0_LSECLK_BYP_MIN
#define STM32_PLLP_MAX                  STM32_VOS0_PLLP_MAX
#define STM32_PLLP_MIN                  STM32_VOS0_PLLP_MIN
#define STM32_PLLQ_MAX                  STM32_VOS0_PLLQ_MAX
#define STM32_PLLQ_MIN                  STM32_VOS0_PLLQ_MIN
#define STM32_PLLR_MAX                  STM32_VOS0_PLLR_MAX
#define STM32_PLLR_MIN                  STM32_VOS0_PLLR_MIN
#define STM32_PCLK1_MAX                 STM32_VOS0_PCLK1_MAX
#define STM32_PCLK2_MAX                 STM32_VOS0_PCLK2_MAX
#define STM32_PCLK3_MAX                 STM32_VOS0_PCLK3_MAX
#define STM32_ADCCLK_MAX                STM32_VOS0_ADCCLK_MAX

#define STM32_0WS_THRESHOLD             STM32_VOS0_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD             STM32_VOS0_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD             STM32_VOS0_2WS_THRESHOLD
#define STM32_3WS_THRESHOLD             STM32_VOS0_3WS_THRESHOLD
#define STM32_4WS_THRESHOLD             STM32_VOS0_4WS_THRESHOLD
#define STM32_5WS_THRESHOLD             STM32_VOS0_5WS_THRESHOLD
/** @} */

#elif (STM32_PWR_VOSCR & PWR_VOSCR_VOS_Msk) == PWR_VOSCR_VOS_RANGE1
#define STM32_SYSCLK_MAX                STM32_VOS1_SYSCLK_MAX
#define STM32_HSECLK_MAX                STM32_VOS1_HSECLK_MAX
#define STM32_HSECLK_BYP_MAX            STM32_VOS1_HSECLK_BYP_MAX
#define STM32_HSECLK_MIN                STM32_VOS1_HSECLK_MIN
#define STM32_HSECLK_BYP_MIN            STM32_VOS1_HSECLK_BYP_MIN
#define STM32_LSECLK_MAX                STM32_VOS1_LSECLK_MAX
#define STM32_LSECLK_BYP_MAX            STM32_VOS1_LSECLK_BYP_MAX
#define STM32_LSECLK_MIN                STM32_VOS1_LSECLK_MIN
#define STM32_LSECLK_BYP_MIN            STM32_VOS1_LSECLK_BYP_MIN
#define STM32_PLLP_MAX                  STM32_VOS1_PLLP_MAX
#define STM32_PLLP_MIN                  STM32_VOS1_PLLP_MIN
#define STM32_PLLQ_MAX                  STM32_VOS1_PLLQ_MAX
#define STM32_PLLQ_MIN                  STM32_VOS1_PLLQ_MIN
#define STM32_PLLR_MAX                  STM32_VOS1_PLLR_MAX
#define STM32_PLLR_MIN                  STM32_VOS1_PLLR_MIN
#define STM32_PCLK1_MAX                 STM32_VOS1_PCLK1_MAX
#define STM32_PCLK2_MAX                 STM32_VOS1_PCLK2_MAX
#define STM32_PCLK3_MAX                 STM32_VOS1_PCLK3_MAX
#define STM32_ADCCLK_MAX                STM32_VOS1_ADCCLK_MAX

#define STM32_0WS_THRESHOLD             STM32_VOS1_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD             STM32_VOS1_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD             STM32_VOS1_2WS_THRESHOLD
#define STM32_3WS_THRESHOLD             STM32_VOS1_3WS_THRESHOLD
#define STM32_4WS_THRESHOLD             STM32_VOS1_4WS_THRESHOLD
#define STM32_5WS_THRESHOLD             STM32_VOS1_5WS_THRESHOLD

#elif (STM32_PWR_VOSCR & PWR_VOSCR_VOS_Msk) == PWR_VOSCR_VOS_RANGE2
#define STM32_SYSCLK_MAX                STM32_VOS2_SYSCLK_MAX
#define STM32_HSECLK_MAX                STM32_VOS2_HSECLK_MAX
#define STM32_HSECLK_BYP_MAX            STM32_VOS2_HSECLK_BYP_MAX
#define STM32_HSECLK_MIN                STM32_VOS2_HSECLK_MIN
#define STM32_HSECLK_BYP_MIN            STM32_VOS2_HSECLK_BYP_MIN
#define STM32_LSECLK_MAX                STM32_VOS2_LSECLK_MAX
#define STM32_LSECLK_BYP_MAX            STM32_VOS2_LSECLK_BYP_MAX
#define STM32_LSECLK_MIN                STM32_VOS2_LSECLK_MIN
#define STM32_LSECLK_BYP_MIN            STM32_VOS2_LSECLK_BYP_MIN
#define STM32_PLLP_MAX                  STM32_VOS2_PLLP_MAX
#define STM32_PLLP_MIN                  STM32_VOS2_PLLP_MIN
#define STM32_PLLQ_MAX                  STM32_VOS2_PLLQ_MAX
#define STM32_PLLQ_MIN                  STM32_VOS2_PLLQ_MIN
#define STM32_PLLR_MAX                  STM32_VOS2_PLLR_MAX
#define STM32_PLLR_MIN                  STM32_VOS2_PLLR_MIN
#define STM32_PCLK1_MAX                 STM32_VOS2_PCLK1_MAX
#define STM32_PCLK2_MAX                 STM32_VOS2_PCLK2_MAX
#define STM32_PCLK3_MAX                 STM32_VOS2_PCLK3_MAX
#define STM32_ADCCLK_MAX                STM32_VOS2_ADCCLK_MAX

#define STM32_0WS_THRESHOLD             STM32_VOS2_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD             STM32_VOS2_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD             STM32_VOS2_2WS_THRESHOLD
#define STM32_3WS_THRESHOLD             STM32_VOS2_3WS_THRESHOLD
#define STM32_4WS_THRESHOLD             STM32_VOS2_4WS_THRESHOLD
#define STM32_5WS_THRESHOLD             STM32_VOS2_5WS_THRESHOLD

#elif (STM32_PWR_VOSCR & PWR_VOSCR_VOS_Msk) == PWR_VOSCR_VOS_RANGE3
#define STM32_SYSCLK_MAX                STM32_VOS3_SYSCLK_MAX
#define STM32_HSECLK_MAX                STM32_VOS3_HSECLK_MAX
#define STM32_HSECLK_BYP_MAX            STM32_VOS3_HSECLK_BYP_MAX
#define STM32_HSECLK_MIN                STM32_VOS3_HSECLK_MIN
#define STM32_HSECLK_BYP_MIN            STM32_VOS3_HSECLK_BYP_MIN
#define STM32_LSECLK_MAX                STM32_VOS3_LSECLK_MAX
#define STM32_LSECLK_BYP_MAX            STM32_VOS3_LSECLK_BYP_MAX
#define STM32_LSECLK_MIN                STM32_VOS3_LSECLK_MIN
#define STM32_LSECLK_BYP_MIN            STM32_VOS3_LSECLK_BYP_MIN
#define STM32_PLLP_MAX                  STM32_VOS3_PLLP_MAX
#define STM32_PLLP_MIN                  STM32_VOS3_PLLP_MIN
#define STM32_PLLQ_MAX                  STM32_VOS3_PLLQ_MAX
#define STM32_PLLQ_MIN                  STM32_VOS3_PLLQ_MIN
#define STM32_PLLR_MAX                  STM32_VOS3_PLLR_MAX
#define STM32_PLLR_MIN                  STM32_VOS3_PLLR_MIN
#define STM32_PCLK1_MAX                 STM32_VOS3_PCLK1_MAX
#define STM32_PCLK2_MAX                 STM32_VOS3_PCLK2_MAX
#define STM32_PCLK3_MAX                 STM32_VOS3_PCLK3_MAX
#define STM32_ADCCLK_MAX                STM32_VOS3_ADCCLK_MAX

#define STM32_0WS_THRESHOLD             STM32_VOS3_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD             STM32_VOS3_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD             STM32_VOS3_2WS_THRESHOLD
#define STM32_3WS_THRESHOLD             STM32_VOS3_3WS_THRESHOLD
#define STM32_4WS_THRESHOLD             STM32_VOS3_4WS_THRESHOLD
#define STM32_5WS_THRESHOLD             STM32_VOS3_5WS_THRESHOLD

#else
#error "invalid STM32_VOS value specified"
#endif

/**
 * @name    PLL input ranges
 */
#define STM32_PLLIN_MIN                 2000000
#define STM32_PLLIN_MAX                 16000000
#define STM32_PLLIN_THRESHOLD1          2000000
#define STM32_PLLIN_THRESHOLD2          4000000
#define STM32_PLLIN_THRESHOLD3          8000000
/** @} */

/**
 * @name    PLL output ranges
 */
#define STM32_PLLVCO_MIN                128000000
#define STM32_PLLVCO_MAX                560000000
#define STM32_PLLVCO_WIDE_MIN           128000000
#define STM32_PLLVCO_WIDE_MAX           560000000
#define STM32_PLLVCO_MEDIUM_MIN         150000000
#define STM32_PLLVCO_MEDIUM_MAX         420000000
/** @} */

/**
 * @name    PLL dividers ranges
 * @{
 */
#define STM32_PLL1M_VALUE_MAX           63
#define STM32_PLL1M_VALUE_MIN           1
#define STM32_PLL1N_ODDVALID            TRUE
#define STM32_PLL1N_VALUE_MAX           512
#define STM32_PLL1N_VALUE_MIN           4
#define STM32_PLL1P_ODDVALID            FALSE
#define STM32_PLL1P_VALUE_MAX           128
#define STM32_PLL1P_VALUE_MIN           2
#define STM32_PLL1Q_ODDVALID            TRUE
#define STM32_PLL1Q_VALUE_MAX           128
#define STM32_PLL1Q_VALUE_MIN           1
#define STM32_PLL1R_ODDVALID            TRUE
#define STM32_PLL1R_VALUE_MAX           128
#define STM32_PLL1R_VALUE_MIN           1

#define STM32_PLL2M_VALUE_MAX           63
#define STM32_PLL2M_VALUE_MIN           1
#define STM32_PLL2N_ODDVALID            TRUE
#define STM32_PLL2N_VALUE_MAX           512
#define STM32_PLL2N_VALUE_MIN           4
#define STM32_PLL2P_ODDVALID            TRUE
#define STM32_PLL2P_VALUE_MAX           128
#define STM32_PLL2P_VALUE_MIN           2
#define STM32_PLL2Q_ODDVALID            TRUE
#define STM32_PLL2Q_VALUE_MAX           128
#define STM32_PLL2Q_VALUE_MIN           1
#define STM32_PLL2R_ODDVALID            TRUE
#define STM32_PLL2R_VALUE_MAX           128
#define STM32_PLL2R_VALUE_MIN           1

#define STM32_PLL3M_VALUE_MAX           63
#define STM32_PLL3M_VALUE_MIN           1
#define STM32_PLL3N_ODDVALID            TRUE
#define STM32_PLL3N_VALUE_MAX           512
#define STM32_PLL3N_VALUE_MIN           4
#define STM32_PLL3P_ODDVALID            TRUE
#define STM32_PLL3P_VALUE_MAX           128
#define STM32_PLL3P_VALUE_MIN           2
#define STM32_PLL3Q_ODDVALID            TRUE
#define STM32_PLL3Q_VALUE_MAX           128
#define STM32_PLL3Q_VALUE_MIN           1
#define STM32_PLL3R_ODDVALID            TRUE
#define STM32_PLL3R_VALUE_MAX           128
#define STM32_PLL3R_VALUE_MIN           1
/** @} */

/**
 * @name    Peripherals limits
 * @{
 */
#define STM32_SDMMC_MAXCLK              130000000U
/** @} */

#endif /* STM32_LIMITS_H */

/** @} */
