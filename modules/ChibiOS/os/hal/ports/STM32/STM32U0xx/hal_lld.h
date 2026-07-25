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
 * @file    STM32U0xx/hal_lld.h
 * @brief   STM32U0xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32U031xx.
 *          - STM32U073xx, STM32U083xx.
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "stm32_registry.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Requires use of SPIv2 driver model.
 */
#define HAL_LLD_SELECT_SPI_V2               TRUE

/**
 * @brief   Signals that TIM7 is used internally by the HAL for timeouts.
 */
#define STM32_TIM7_IS_USED

/**
 * @name    Platform identification
 * @{
 */
#if defined(STM32U031xx)
#define PLATFORM_NAME                       "STM32U0 Ultra-low-power Access Line"

#elif defined(STM32U073xx)
#define PLATFORM_NAME                       "STM32U0 Ultra-low-power with USB"

#elif defined(STM32U083xx)
#define PLATFORM_NAME                       "STM32U0 Ultra-low-power with USB and LCD"

#else
#error "STM32U0 device not specified"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(STM32U0XX) || defined(__DOXYGEN__)
#define STM32U0XX
#endif
/** @} */

/**
 * @name   Clock points names
 * @{
 */
#define CLK_SYSCLK                          0U
#define CLK_HSI16                           1U
#define CLK_HSI48                           2U
#define CLK_HSE                             3U
#define CLK_MSI                             4U
#define CLK_PLLP                            5U
#define CLK_PLLQ                            6U
#define CLK_PLLR                            7U
#define CLK_HCLK                            8U
#define CLK_PCLK                            9U
#define CLK_PCLKTIM                         10U
#define CLK_MCO1                            11U
#define CLK_MCO2                            12U
#define CLK_ARRAY_SIZE                      13U
#define CLK_POINT_NAMES                                                     \
  {                                                                         \
    "SYSCLK", "HSI16", "HSI48", "HSE", "MSI", "PLLP", "PLLQ",               \
    "PLLR", "HCLK", "PCLK", "PCLKTIM", "MCO1", "MCO2"                       \
  }
/** @} */

/**
 * @name    PWR_CR1 register helpers
 * @{
 */
#define PWR_CR1_LPMS_FIELD(n)               ((n) << PWR_CR1_LPMS_Pos)
#define PWR_CR1_LPMS_STOP0                  PWR_CR1_LPMS_FIELD(0)
#define PWR_CR1_LPMS_STOP1                  PWR_CR1_LPMS_FIELD(1)
#define PWR_CR1_LPMS_STOP2                  PWR_CR1_LPMS_FIELD(2)

#define PWR_CR1_VOS_FIELD(n)                ((n) << PWR_CR1_VOS_Pos)
#define PWR_CR1_VOS_RANGE1                  PWR_CR1_VOS_FIELD(1U)
#define PWR_CR1_VOS_RANGE2                  PWR_CR1_VOS_FIELD(2U)
/** @} */

/**
 * @name    PWR_CR2 register helpers
 * @{
 */
#define PWR_CR2_PLS_FIELD(n)                ((n) << PWR_CR2_PLS_Pos)
#define PWR_CR2_PLS_2P0V                    PWR_CR2_PLS_FIELD(0)
#define PWR_CR2_PLS_2P2V                    PWR_CR2_PLS_FIELD(1)
#define PWR_CR2_PLS_2P4V                    PWR_CR2_PLS_FIELD(2)
#define PWR_CR2_PLS_2P5V                    PWR_CR2_PLS_FIELD(3)
#define PWR_CR2_PLS_2P6V                    PWR_CR2_PLS_FIELD(4)
#define PWR_CR2_PLS_2P7V                    PWR_CR2_PLS_FIELD(5)
#define PWR_CR2_PLS_2P8V                    PWR_CR2_PLS_FIELD(6)
#define PWR_CR2_PLS_2P9V                    PWR_CR2_PLS_FIELD(7)
/** @} */

/**
 * @name    RCC_CFGR register helpers
 * @{
 */
#define RCC_CFGR_SW_FIELD(n)                ((n) << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_MSI                     RCC_CFGR_SW_FIELD(0U)
#define RCC_CFGR_SW_HSI16                   RCC_CFGR_SW_FIELD(1U)
#define RCC_CFGR_SW_HSE                     RCC_CFGR_SW_FIELD(2U)
#define RCC_CFGR_SW_PLLR                    RCC_CFGR_SW_FIELD(3U)
#define RCC_CFGR_SW_LSI                     RCC_CFGR_SW_FIELD(4U)
#define RCC_CFGR_SW_LSE                     RCC_CFGR_SW_FIELD(5U)

#define RCC_CFGR_MCO2SEL_FIELD(n)           ((n) << RCC_CFGR_MCO2SEL_Pos)
#define RCC_CFGR_MCO2SEL_NOCLOCK            RCC_CFGR_MCO2SEL_FIELD(0)
#define RCC_CFGR_MCO2SEL_SYSCLK             RCC_CFGR_MCO2SEL_FIELD(1)
#define RCC_CFGR_MCO2SEL_MSI                RCC_CFGR_MCO2SEL_FIELD(2)
#define RCC_CFGR_MCO2SEL_HSI16              RCC_CFGR_MCO2SEL_FIELD(3)
#define RCC_CFGR_MCO2SEL_HSE                RCC_CFGR_MCO2SEL_FIELD(4)
#define RCC_CFGR_MCO2SEL_PLLRCLK            RCC_CFGR_MCO2SEL_FIELD(5)
#define RCC_CFGR_MCO2SEL_LSI                RCC_CFGR_MCO2SEL_FIELD(6)
#define RCC_CFGR_MCO2SEL_LSE                RCC_CFGR_MCO2SEL_FIELD(7)
#define RCC_CFGR_MCO2SEL_HSI48              RCC_CFGR_MCO2SEL_FIELD(8)
#define RCC_CFGR_MCO2SEL_RTCCLK             RCC_CFGR_MCO2SEL_FIELD(9)
#define RCC_CFGR_MCO2SEL_RTCWKP             RCC_CFGR_MCO2SEL_FIELD(10)

#define RCC_CFGR_MCO2PRE_FIELD(n)           ((n) << RCC_CFGR_MCO2PRE_Pos)
#define RCC_CFGR_MCO2PRE_DIV1               RCC_CFGR_MCO2PRE_FIELD(0)
#define RCC_CFGR_MCO2PRE_DIV2               RCC_CFGR_MCO2PRE_FIELD(1)
#define RCC_CFGR_MCO2PRE_DIV4               RCC_CFGR_MCO2PRE_FIELD(2)
#define RCC_CFGR_MCO2PRE_DIV8               RCC_CFGR_MCO2PRE_FIELD(3)
#define RCC_CFGR_MCO2PRE_DIV16              RCC_CFGR_MCO2PRE_FIELD(4)
#define RCC_CFGR_MCO2PRE_DIV32              RCC_CFGR_MCO2PRE_FIELD(5)
#define RCC_CFGR_MCO2PRE_DIV64              RCC_CFGR_MCO2PRE_FIELD(6)
#define RCC_CFGR_MCO2PRE_DIV128             RCC_CFGR_MCO2PRE_FIELD(7)
#define RCC_CFGR_MCO2PRE_DIV256             RCC_CFGR_MCO2PRE_FIELD(8)
#define RCC_CFGR_MCO2PRE_DIV512             RCC_CFGR_MCO2PRE_FIELD(9)
#define RCC_CFGR_MCO2PRE_DIV1024            RCC_CFGR_MCO2PRE_FIELD(10)

#define RCC_CFGR_MCO1SEL_FIELD(n)           ((n) << RCC_CFGR_MCO1SEL_Pos)
#define RCC_CFGR_MCO1SEL_NOCLOCK            RCC_CFGR_MCO1SEL_FIELD(0)
#define RCC_CFGR_MCO1SEL_SYSCLK             RCC_CFGR_MCO1SEL_FIELD(1)
#define RCC_CFGR_MCO1SEL_MSI                RCC_CFGR_MCO1SEL_FIELD(2)
#define RCC_CFGR_MCO1SEL_HSI16              RCC_CFGR_MCO1SEL_FIELD(3)
#define RCC_CFGR_MCO1SEL_HSE                RCC_CFGR_MCO1SEL_FIELD(4)
#define RCC_CFGR_MCO1SEL_PLLRCLK            RCC_CFGR_MCO1SEL_FIELD(5)
#define RCC_CFGR_MCO1SEL_LSI                RCC_CFGR_MCO1SEL_FIELD(6)
#define RCC_CFGR_MCO1SEL_LSE                RCC_CFGR_MCO1SEL_FIELD(7)
#define RCC_CFGR_MCO1SEL_HSI48              RCC_CFGR_MCO1SEL_FIELD(8)
#define RCC_CFGR_MCO1SEL_RTCCLK             RCC_CFGR_MCO1SEL_FIELD(9)
#define RCC_CFGR_MCO1SEL_RTCWKP             RCC_CFGR_MCO1SEL_FIELD(10)

#define RCC_CFGR_MCO1PRE_FIELD(n)           ((n) << RCC_CFGR_MCO1PRE_Pos)
#define RCC_CFGR_MCO1PRE_DIV1               RCC_CFGR_MCO1PRE_FIELD(0)
#define RCC_CFGR_MCO1PRE_DIV2               RCC_CFGR_MCO1PRE_FIELD(1)
#define RCC_CFGR_MCO1PRE_DIV4               RCC_CFGR_MCO1PRE_FIELD(2)
#define RCC_CFGR_MCO1PRE_DIV8               RCC_CFGR_MCO1PRE_FIELD(3)
#define RCC_CFGR_MCO1PRE_DIV16              RCC_CFGR_MCO1PRE_FIELD(4)
#define RCC_CFGR_MCO1PRE_DIV32              RCC_CFGR_MCO1PRE_FIELD(5)
#define RCC_CFGR_MCO1PRE_DIV64              RCC_CFGR_MCO1PRE_FIELD(6)
#define RCC_CFGR_MCO1PRE_DIV128             RCC_CFGR_MCO1PRE_FIELD(7)
#define RCC_CFGR_MCO1PRE_DIV256             RCC_CFGR_MCO1PRE_FIELD(8)
#define RCC_CFGR_MCO1PRE_DIV512             RCC_CFGR_MCO1PRE_FIELD(9)
#define RCC_CFGR_MCO1PRE_DIV1024            RCC_CFGR_MCO1PRE_FIELD(10)
/** @} */

/**
 * @name    RCC_PLLCFGR register helpers
 * @{
 */
#define RCC_PLLCFGR_PLLSRC_FIELD(n)         ((n) << RCC_PLLCFGR_PLLSRC_Pos)
#define RCC_PLLCFGR_PLLSRC_NOCLOCK          RCC_PLLCFGR_PLLSRC_FIELD(0U)
#define RCC_PLLCFGR_PLLSRC_MSI              RCC_PLLCFGR_PLLSRC_FIELD(1U)
#define RCC_PLLCFGR_PLLSRC_HSI16            RCC_PLLCFGR_PLLSRC_FIELD(2U)
#define RCC_PLLCFGR_PLLSRC_HSE              RCC_PLLCFGR_PLLSRC_FIELD(3U)
/** @} */

/**
 * @name    RCC_CCIPR register helpers
 * @{
 */
#define RCC_CCIPR_USART1SEL_PCLK            (0U << RCC_CCIPR_USART1SEL_Pos)
#define RCC_CCIPR_USART1SEL_SYSCLK          (1U << RCC_CCIPR_USART1SEL_Pos)
#define RCC_CCIPR_USART1SEL_HSI16           (2U << RCC_CCIPR_USART1SEL_Pos)
#define RCC_CCIPR_USART1SEL_LSE             (3U << RCC_CCIPR_USART1SEL_Pos)

#define RCC_CCIPR_USART2SEL_PCLK            (0U << RCC_CCIPR_USART2SEL_Pos)
#define RCC_CCIPR_USART2SEL_SYSCLK          (1U << RCC_CCIPR_USART2SEL_Pos)
#define RCC_CCIPR_USART2SEL_HSI16           (2U << RCC_CCIPR_USART2SEL_Pos)
#define RCC_CCIPR_USART2SEL_LSE             (3U << RCC_CCIPR_USART2SEL_Pos)

#define RCC_CCIPR_LPUART3SEL_PCLK           (0U << RCC_CCIPR_LPUART3SEL_Pos)
#define RCC_CCIPR_LPUART3SEL_SYSCLK         (1U << RCC_CCIPR_LPUART3SEL_Pos)
#define RCC_CCIPR_LPUART3SEL_HSI16          (2U << RCC_CCIPR_LPUART3SEL_Pos)
#define RCC_CCIPR_LPUART3SEL_LSE            (3U << RCC_CCIPR_LPUART3SEL_Pos)

#define RCC_CCIPR_LPUART2SEL_PCLK           (0U << RCC_CCIPR_LPUART2SEL_Pos)
#define RCC_CCIPR_LPUART2SEL_SYSCLK         (1U << RCC_CCIPR_LPUART2SEL_Pos)
#define RCC_CCIPR_LPUART2SEL_HSI16          (2U << RCC_CCIPR_LPUART2SEL_Pos)
#define RCC_CCIPR_LPUART2SEL_LSE            (3U << RCC_CCIPR_LPUART2SEL_Pos)

#define RCC_CCIPR_LPUART1SEL_PCLK           (0U << RCC_CCIPR_LPUART1SEL_Pos)
#define RCC_CCIPR_LPUART1SEL_SYSCLK         (1U << RCC_CCIPR_LPUART1SEL_Pos)
#define RCC_CCIPR_LPUART1SEL_HSI16          (2U << RCC_CCIPR_LPUART1SEL_Pos)
#define RCC_CCIPR_LPUART1SEL_LSE            (3U << RCC_CCIPR_LPUART1SEL_Pos)

#define RCC_CCIPR_I2C1SEL_PCLK              (0U << RCC_CCIPR_I2C1SEL_Pos)
#define RCC_CCIPR_I2C1SEL_SYSCLK            (1U << RCC_CCIPR_I2C1SEL_Pos)
#define RCC_CCIPR_I2C1SEL_HSI16             (2U << RCC_CCIPR_I2C1SEL_Pos)

#define RCC_CCIPR_I2C3SEL_PCLK              (0U << RCC_CCIPR_I2C3SEL_Pos)
#define RCC_CCIPR_I2C3SEL_SYSCLK            (1U << RCC_CCIPR_I2C3SEL_Pos)
#define RCC_CCIPR_I2C3SEL_HSI16             (2U << RCC_CCIPR_I2C3SEL_Pos)

#define RCC_CCIPR_LPTIM1SEL_PCLK            (0U << RCC_CCIPR_LPTIM1SEL_Pos)
#define RCC_CCIPR_LPTIM1SEL_LSI             (1U << RCC_CCIPR_LPTIM1SEL_Pos)
#define RCC_CCIPR_LPTIM1SEL_HSI16           (2U << RCC_CCIPR_LPTIM1SEL_Pos)
#define RCC_CCIPR_LPTIM1SEL_LSE             (3U << RCC_CCIPR_LPTIM1SEL_Pos)

#define RCC_CCIPR_LPTIM2SEL_PCLK            (0U << RCC_CCIPR_LPTIM2SEL_Pos)
#define RCC_CCIPR_LPTIM2SEL_LSI             (1U << RCC_CCIPR_LPTIM2SEL_Pos)
#define RCC_CCIPR_LPTIM2SEL_HSI16           (2U << RCC_CCIPR_LPTIM2SEL_Pos)
#define RCC_CCIPR_LPTIM2SEL_LSE             (3U << RCC_CCIPR_LPTIM2SEL_Pos)

#define RCC_CCIPR_LPTIM3SEL_PCLK            (0U << RCC_CCIPR_LPTIM3SEL_Pos)
#define RCC_CCIPR_LPTIM3SEL_LSI             (1U << RCC_CCIPR_LPTIM3SEL_Pos)
#define RCC_CCIPR_LPTIM3SEL_HSI16           (2U << RCC_CCIPR_LPTIM3SEL_Pos)
#define RCC_CCIPR_LPTIM3SEL_LSE             (3U << RCC_CCIPR_LPTIM3SEL_Pos)

#define RCC_CCIPR_TIM1SEL_TIMPCLK           (0U << RCC_CCIPR_TIM1SEL_Pos)
#define RCC_CCIPR_TIM1SEL_PLLQCLK           (1U << RCC_CCIPR_TIM1SEL_Pos)

#define RCC_CCIPR_TIM15SEL_TIMPCLK          (0U << RCC_CCIPR_TIM15SEL_Pos)
#define RCC_CCIPR_TIM15SEL_PLLQCLK          (1U << RCC_CCIPR_TIM15SEL_Pos)

#define RCC_CCIPR_CLK48SEL_NOCLOCK          (0U << RCC_CCIPR_CLK48SEL_Pos)
#define RCC_CCIPR_CLK48SEL_MSI              (1U << RCC_CCIPR_CLK48SEL_Pos)
#define RCC_CCIPR_CLK48SEL_PLLQCLK          (2U << RCC_CCIPR_CLK48SEL_Pos)
#define RCC_CCIPR_CLK48SEL_HSI48            (3U << RCC_CCIPR_CLK48SEL_Pos)

#define RCC_CCIPR_ADCSEL_SYSCLK             (0U << RCC_CCIPR_ADCSEL_Pos)
#define RCC_CCIPR_ADCSEL_PLLPCLK            (1U << RCC_CCIPR_ADCSEL_Pos)
#define RCC_CCIPR_ADCSEL_HSI16              (2U << RCC_CCIPR_ADCSEL_Pos)
/** @} */

/**
 * @name    RCC_BDCR register helpers
 * @{
 */
#define RCC_BDCR_RTCSEL_NOCLOCK             (0U << RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_LSE                 (1U << RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_LSI                 (2U << RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_HSEDIV              (3U << RCC_BDCR_RTCSEL_Pos)

#define RCC_BDCR_LSCOSEL_NOCLOCK            0U
#define RCC_BDCR_LSCOSEL_LSI                RCC_BDCR_LSCOEN
#define RCC_BDCR_LSCOSEL_LSE                (RCC_BDCR_LSCOEN | RCC_BDCR_LSCOSEL)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Disables the PWR/RCC initialization in the HAL.
 */
#if !defined(STM32_NO_INIT) || defined(__DOXYGEN__)
#define STM32_NO_INIT                       FALSE
#endif

/**
 * @brief   Enables the dynamic clock handling.
 */
#if !defined(STM32_CLOCK_DYNAMIC) || defined(__DOXYGEN__)
#define STM32_CLOCK_DYNAMIC                 FALSE
#endif

/**
 * @brief   SYSCFG CFGR1 register initialization value.
 * @note    The field MEM_MODE is not touched by this setting.
 */
#if !defined(STM32_SYSCFG_CFGR1) || defined(__DOXYGEN__)
#define STM32_SYSCFG_CFGR1                  (0U)
#endif

/**
 * @brief   PWR CR1 register initialization value.
 */
#if !defined(STM32_PWR_CR1) || defined(__DOXYGEN__)
#define STM32_PWR_CR1                       (PWR_CR1_VOS_RANGE1 | PWR_CR1_FPD_STOP)
#endif

/**
 * @brief   PWR CR2 register initialization value.
 */
#if !defined(STM32_PWR_CR2) || defined(__DOXYGEN__)
#define STM32_PWR_CR2                       (PWR_CR2_USV)
#endif

/**
 * @brief   PWR CR3 register initialization value.
 */
#if !defined(STM32_PWR_CR3) || defined(__DOXYGEN__)
#define STM32_PWR_CR3                       (0U)
#endif

/**
 * @brief   PWR CR4 register initialization value.
 */
#if !defined(STM32_PWR_CR4) || defined(__DOXYGEN__)
#define STM32_PWR_CR4                       (0U)
#endif

/**
 * @brief   PWR PUCRA register initialization value.
 */
#if !defined(STM32_PWR_PUCRA) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRA                     (0U)
#endif

/**
 * @brief   PWR PDCRA register initialization value.
 */
#if !defined(STM32_PWR_PDCRA) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRA                     (0U)
#endif

/**
 * @brief   PWR PUCRB register initialization value.
 */
#if !defined(STM32_PWR_PUCRB) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRB                     (0U)
#endif

/**
 * @brief   PWR PDCRB register initialization value.
 */
#if !defined(STM32_PWR_PDCRB) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRB                     (0U)
#endif

/**
 * @brief   PWR PUCRC register initialization value.
 */
#if !defined(STM32_PWR_PUCRC) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRC                     (0U)
#endif

/**
 * @brief   PWR PDCRC register initialization value.
 */
#if !defined(STM32_PWR_PDCRC) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRC                     (0U)
#endif

/**
 * @brief   PWR PUCRD register initialization value.
 */
#if !defined(STM32_PWR_PUCRD) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRD                     (0U)
#endif

/**
 * @brief   PWR PDCRD register initialization value.
 */
#if !defined(STM32_PWR_PDCRD) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRD                     (0U)
#endif

/**
 * @brief   PWR PUCRE register initialization value.
 */
#if !defined(STM32_PWR_PUCRE) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRE                     (0U)
#endif

/**
 * @brief   PWR PDCRE register initialization value.
 */
#if !defined(STM32_PWR_PDCRE) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRE                     (0U)
#endif

/**
 * @brief   PWR PUCRF register initialization value.
 */
#if !defined(STM32_PWR_PUCRF) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRF                     (0U)
#endif

/**
 * @brief   PWR PDCRF register initialization value.
 */
#if !defined(STM32_PWR_PDCRF) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRF                     (0U)
#endif

/**
 * @brief   FLASH ACR register initialization value.
 * @note    Do not specify the LATENCY bits because those are calculated
 *          depending on other settings and ORed to this value.
 */
#if !defined(STM32_FLASH_ACR) || defined(__DOXYGEN__)
#define STM32_FLASH_ACR                     (FLASH_ACR_DBG_SWEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN)
#endif

/**
 * @brief   Enables or disables the HSI16 clock source.
 */
#if !defined(STM32_HSI16_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI16_ENABLED                 FALSE
#endif

/**
 * @brief   Enables or disables the HSI48 clock source.
 */
#if !defined(STM32_HSI48_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI48_ENABLED                 FALSE
#endif

/**
 * @brief   Enables or disables the HSE clock source.
 */
#if !defined(STM32_HSE_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSE_ENABLED                   FALSE
#endif

/**
 * @brief   Enables or disables the LSI clock source.
 */
#if !defined(STM32_LSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSI_ENABLED                   FALSE
#endif

/**
 * @brief   Enables or disables the LSE clock source.
 */
#if !defined(STM32_LSE_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSE_ENABLED                   FALSE
#endif

/**
 * @brief   Enables or disables the MSI clock source.
 */
#if !defined(STM32_MSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_MSI_ENABLED                   TRUE
#endif

/**
 * @brief   Enables or disables the MSI PLL on LSE clock source.
 */
#if !defined(STM32_MSIPLL_ENABLED) || defined(__DOXYGEN__)
#define STM32_MSIPLL_ENABLED                FALSE
#endif

/**
 * @brief   MSI frequency setting.
 */
#if !defined(STM32_MSIRANGE) || defined(__DOXYGEN__)
#define STM32_MSIRANGE                      RCC_CR_MSIRANGE_4MHz
#endif

/**
 * @brief   MSI frequency setting after standby.
 */
#if !defined(STM32_MSISRANGE) || defined(__DOXYGEN__)
#define STM32_MSISRANGE                     RCC_CR_MSIRANGE_4MHz
#endif

/**
 * @brief   Main clock source selection.
 * @note    If the selected clock source is not the PLL then the PLL is not
 *          initialized and started.
 * @note    The default value is calculated for a 64MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                            RCC_CFGR_SW_PLLR
#endif

/**
 * @brief   Clock source for the PLL.
 * @note    This setting has only effect if the PLL is selected as the
 *          system clock source.
 * @note    The default value is calculated for a 64MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_PLLSRC) || defined(__DOXYGEN__)
#define STM32_PLLSRC                        RCC_PLLCFGR_PLLSRC_MSI
#endif

/**
 * @brief   PLLM divider value.
 * @note    The allowed values are 1..8.
 * @note    The default value is calculated for a 64MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_PLLM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLM_VALUE                    1
#endif

/**
 * @brief   PLLN multiplier value.
 * @note    The allowed values are 8..86.
 * @note    The default value is calculated for a 64MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_PLLN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLN_VALUE                    28
#endif

/**
 * @brief   PLLP divider value.
 * @note    The allowed values are 2..32.
 */
#if !defined(STM32_PLLP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLP_VALUE                    2
#endif

/**
 * @brief   PLLQ divider value.
 * @note    The allowed values are 2..8.
 */
#if !defined(STM32_PLLQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLQ_VALUE                    2
#endif

/**
 * @brief   PLLR divider value.
 * @note    The allowed values are 2..8.
 * @note    The default value is calculated for a 64MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_PLLR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLR_VALUE                    2
#endif

/**
 * @brief   AHB prescaler value.
 * @note    The default value is calculated for a 64MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_HPRE) || defined(__DOXYGEN__)
#define STM32_HPRE                          RCC_CFGR_HPRE_DIV1
#endif

/**
 * @brief   APB prescaler value.
 */
#if !defined(STM32_PPRE) || defined(__DOXYGEN__)
#define STM32_PPRE                          RCC_CFGR_PPRE_DIV1
#endif

/**
 * @brief   MCO1 clock source.
 */
#if !defined(STM32_MCO1SEL) || defined(__DOXYGEN__)
#define STM32_MCO1SEL                       RCC_CFGR_MCO1SEL_NOCLOCK
#endif

/**
 * @brief   MCO1 divider setting.
 */
#if !defined(STM32_MCO1PRE) || defined(__DOXYGEN__)
#define STM32_MCO1PRE                       RCC_CFGR_MCO1PRE_DIV1
#endif

/**
 * @brief   MCO2 clock source.
 */
#if !defined(STM32_MCO2SEL) || defined(__DOXYGEN__)
#define STM32_MCO2SEL                       RCC_CFGR_MCO2SEL_NOCLOCK
#endif

/**
 * @brief   MCO2 divider setting.
 */
#if !defined(STM32_MCO2PRE) || defined(__DOXYGEN__)
#define STM32_MCO2PRE                       RCC_CFGR_MCO2PRE_DIV1
#endif

/**
 * @brief   LSCO clock source.
 */
#if !defined(STM32_LSCOSEL) || defined(__DOXYGEN__)
#define STM32_LSCOSEL                       RCC_BDCR_LSCOSEL_NOCLOCK
#endif

/**
 * @brief   USART1 clock source.
 */
#if !defined(STM32_USART1SEL) || defined(__DOXYGEN__)
#define STM32_USART1SEL                     RCC_CCIPR_USART1SEL_PCLK
#endif

/**
 * @brief   USART2 clock source.
 */
#if !defined(STM32_USART2SEL) || defined(__DOXYGEN__)
#define STM32_USART2SEL                     RCC_CCIPR_USART2SEL_PCLK
#endif

/**
 * @brief   LPUART1 clock source.
 */
#if !defined(STM32_LPUART1SEL) || defined(__DOXYGEN__)
#define STM32_LPUART1SEL                    RCC_CCIPR_LPUART1SEL_PCLK
#endif

/**
 * @brief   LPUART2 clock source.
 */
#if !defined(STM32_LPUART2SEL) || defined(__DOXYGEN__)
#define STM32_LPUART2SEL                    RCC_CCIPR_LPUART2SEL_PCLK
#endif

/**
 * @brief   LPUART3 clock source.
 */
#if !defined(STM32_LPUART3SEL) || defined(__DOXYGEN__)
#define STM32_LPUART3SEL                    RCC_CCIPR_LPUART3SEL_PCLK
#endif

/**
 * @brief   I2C1 clock source.
 */
#if !defined(STM32_I2C1SEL) || defined(__DOXYGEN__)
#define STM32_I2C1SEL                       RCC_CCIPR_I2C1SEL_PCLK
#endif

/**
 * @brief   I2C3 clock source.
 */
#if !defined(STM32_I2C3SEL) || defined(__DOXYGEN__)
#define STM32_I2C3SEL                       RCC_CCIPR_I2C3SEL_PCLK
#endif

/**
 * @brief   LPTIM1 clock source.
 */
#if !defined(STM32_LPTIM1SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM1SEL                     RCC_CCIPR_LPTIM1SEL_PCLK
#endif

/**
 * @brief   LPTIM2 clock source.
 */
#if !defined(STM32_LPTIM2SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM2SEL                     RCC_CCIPR_LPTIM2SEL_PCLK
#endif

/**
 * @brief   LPTIM3 clock source.
 */
#if !defined(STM32_LPTIM3SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM3SEL                     RCC_CCIPR_LPTIM3SEL_PCLK
#endif

/**
 * @brief   TIM1 clock source.
 */
#if !defined(STM32_TIM1SEL) || defined(__DOXYGEN__)
#define STM32_TIM1SEL                       RCC_CCIPR_TIM1SEL_TIMPCLK
#endif

/**
 * @brief   TIM15 clock source.
 */
#if !defined(STM32_TIM15SEL) || defined(__DOXYGEN__)
#define STM32_TIM15SEL                      RCC_CCIPR_TIM15SEL_TIMPCLK
#endif

/**
 * @brief   USB/RNG clock source.
 */
#if !defined(STM32_CLK48SEL) || defined(__DOXYGEN__)
#define STM32_CLK48SEL                      RCC_CCIPR_CLK48SEL_PLLQCLK
#endif

/**
 * @brief   ADC clock source.
 */
#if !defined(STM32_ADCSEL) || defined(__DOXYGEN__)
#define STM32_ADCSEL                        RCC_CCIPR_ADCSEL_SYSCLK
#endif

/**
 * @brief   RTC clock source.
 */
#if !defined(STM32_RTCSEL) || defined(__DOXYGEN__)
#define STM32_RTCSEL                        RCC_BDCR_RTCSEL_NOCLOCK
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Clock handling mode selection.*/
#if STM32_CLOCK_DYNAMIC == TRUE
#define HAL_LLD_USE_CLOCK_MANAGEMENT
#endif

/*
 * Configuration-related checks.
 */
#if !defined(STM32U0xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32U0xx_MCUCONF not defined"
#endif

#if defined(STM32U031xx) && !defined(STM32U031_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32U031_MCUCONF not defined"

#elif defined(STM32U073xx) && !defined(STM32U073_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32U073_MCUCONF not defined"

#elif defined(STM32U083xx) && !defined(STM32U083_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32U083_MCUCONF not defined"

#endif

/*
 * Board files sanity checks.
 */
#if !defined(STM32_LSECLK)
#error "STM32_LSECLK not defined in board.h"
#endif

#if !defined(STM32_LSEDRV)
#error "STM32_LSEDRV not defined in board.h"
#endif

#if !defined(STM32_HSECLK)
#error "STM32_HSECLK not defined in board.h"
#endif

/* Device limits.*/
#include "stm32_limits.h"

/* Clock handlers.*/
#include "stm32_msi.inc"
#include "stm32_lse.inc"
#include "stm32_lsi.inc"
#include "stm32_hsi16.inc"
#include "stm32_hsi48.inc"
#include "stm32_hse.inc"

/*
 * HSI16 related checks.
 */
#if STM32_HSI16_ENABLED
#else /* !STM32_HSI16_ENABLED */

  #if STM32_SW == RCC_CFGR_SW_HSI16
    #error "HSI16 not enabled, required by STM32_SW"
  #endif

  #if (STM32_SW == RCC_CFGR_SW_PLLR) && (STM32_PLLSRC == RCC_PLLCFGR_PLLSRC_HSI16)
    #error "HSI16 not enabled, required by STM32_SW and STM32_PLLSRC"
  #endif

  #if (STM32_MCO1SEL == RCC_CFGR_MCO1SEL_HSI16) ||                          \
      ((STM32_MCO1SEL == RCC_CFGR_MCO1SEL_PLLRCLK) && (STM32_PLLSRC == RCC_PLLCFGR_PLLSRC_HSI16))
    #error "HSI16 not enabled, required by MCO"
  #endif

  #if (STM32_MCO2SEL == RCC_CFGR_MCO2SEL_HSI16) ||                          \
      ((STM32_MCO2SEL == RCC_CFGR_MCO2SEL_PLLRCLK) &&  (STM32_PLLSRC == RCC_PLLCFGR_PLLSRC_HSI16))
    #error "HSI16 not enabled, required by MCO2"
  #endif

  #if (STM32_USART1SEL == RCC_CCIPR_USART1SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_USART1SEL"
  #endif
  #if (STM32_USART2SEL == RCC_CCIPR_USART2SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_USART2SEL"
  #endif
  #if (STM32_LPUART1SEL == RCC_CCIPR_LPUART1SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_LPUART1SEL"
  #endif
  #if (STM32_LPUART2SEL == RCC_CCIPR_LPUART2SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_LPUART2SEL"
  #endif
  #if (STM32_LPUART3SEL == RCC_CCIPR_LPUART3SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_LPUART3SEL"
  #endif

  #if (STM32_I2C1SEL == RCC_CCIPR_I2C1SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_I2C1SEL"
  #endif
  #if (STM32_I2C3SEL == RCC_CCIPR_I2C3SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_I2C3SEL"
  #endif

  #if (STM32_LPTIM1SEL == RCC_CCIPR_LPTIM1SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_LPTIM1SEL"
  #endif
  #if (STM32_LPTIM2SEL == RCC_CCIPR_LPTIM2SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_LPTIM2SEL"
  #endif
  #if (STM32_LPTIM3SEL == RCC_CCIPR_LPTIM3SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_LPTIM3SEL"
  #endif

  #if (STM32_ADCSEL == RCC_CCIPR_ADCSEL_HSI16)
    #error "HSI16 not enabled, required by STM32_ADCSEL"
  #endif

#endif /* !STM32_HSI16_ENABLED */

/*
 * HSI48 related checks.
 */
#if STM32_HSI48_ENABLED
#else /* !STM32_HSI48_ENABLED */

  #if STM32_MCO1SEL == RCC_CFGR_MCO1SEL_HSI48
    #error "HSI48 not enabled, required by STM32_MCO1SEL"
  #endif

  #if STM32_MCO2SEL == RCC_CFGR_MCO2SEL_HSI48
    #error "HSI48 not enabled, required by STM32_MCO2SEL"
  #endif

  #if (STM32_CLK48SEL == RCC_CCIPR_CLK48SEL_HSI48) &&                       \
      ((HAL_USE_USB == TRUE) || (HAL_USE_TRNG == TRUE))
    #error "HSI48 not enabled, required by STM32_CLK48SEL"
  #endif

#endif /* !STM32_HSI48_ENABLED */

/*
 * HSE related checks.
 */
#if STM32_HSE_ENABLED
#else /* !STM32_HSE_ENABLED */

  #if STM32_SW == RCC_CFGR_SW_HSE
    #error "HSE not enabled, required by STM32_SW"
  #endif

  #if (STM32_SW == RCC_CFGR_SW_PLLR) && (STM32_PLLSRC == RCC_PLLCFGR_PLLSRC_HSE)
    #error "HSE not enabled, required by STM32_SW and STM32_PLLSRC"
  #endif

  #if (STM32_MCO1SEL == RCC_CFGR_MCO1SEL_HSE) ||                            \
      ((STM32_MCO1SEL == RCC_CFGR_MCO1SEL_PLLRCLK) &&                       \
       (STM32_PLLSRC == RCC_PLLCFGR_PLLSRC_HSE))
    #error "HSE not enabled, required by STM32_MCO1SEL"
  #endif

  #if (STM32_MCO2SEL == RCC_CFGR_MCO2SEL_HSE) ||                            \
      ((STM32_MCO2SEL == RCC_CFGR_MCO2SEL_PLLRCLK) &&                       \
       (STM32_PLLSRC == RCC_PLLCFGR_PLLSRC_HSE))
    #error "HSE not enabled, required by STM32_MCO2SEL"
  #endif

  #if STM32_RTCSEL == RCC_BDCR_RTCSEL_HSEDIV
    #error "HSE not enabled, required by STM32_RTCSEL"
  #endif

#endif /* !STM32_HSE_ENABLED */

/*
 * LSI related checks.
 */
#if STM32_LSI_ENABLED
#else /* !STM32_LSI_ENABLED */

  #if STM32_SW == RCC_CFGR_SW_LSI
    #error "LSI not enabled, required by STM32_SW"
  #endif

  #if HAL_USE_RTC && (STM32_RTCSEL == RCC_BDCR_RTCSEL_LSI)
    #error "LSI not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_MCO1SEL == RCC_CFGR_MCO1SEL_LSI
    #error "LSI not enabled, required by STM32_MCO1SEL"
  #endif

  #if STM32_MCO2SEL == RCC_CFGR_MCO2SEL_LSI
    #error "LSI not enabled, required by STM32_MCO2SEL"
  #endif

  #if STM32_LSCOSEL == RCC_BDCR_LSCOSEL_LSI
    #error "LSI not enabled, required by STM32_LSCOSEL"
  #endif

  #if (STM32_LPTIM1SEL == RCC_CCIPR_LPTIM1SEL_LSI)
    #error "LSI not enabled, required by STM32_LPTIM1SEL"
  #endif
  #if (STM32_LPTIM2SEL == RCC_CCIPR_LPTIM2SEL_LSI)
    #error "LSI not enabled, required by STM32_LPTIM2SEL"
  #endif
  #if (STM32_LPTIM3SEL == RCC_CCIPR_LPTIM3SEL_LSI)
    #error "LSI not enabled, required by STM32_LPTIM3SEL"
  #endif

#endif /* !STM32_LSI_ENABLED */

/*
 * LSE related checks.
 */
#if STM32_LSE_ENABLED
#else /* !STM32_LSE_ENABLED */

  #if STM32_SW == RCC_CFGR_SW_LSE
    #error "LSE not enabled, required by STM32_SW"
  #endif

  #if STM32_RTCSEL == RCC_BDCR_RTCSEL_LSE
    #error "LSE not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_MCO1SEL == RCC_CFGR_MCO1SEL_LSE
    #error "LSE not enabled, required by STM32_MCO1SEL"
  #endif

  #if STM32_MCO2SEL == RCC_CFGR_MCO2SEL_LSE
    #error "LSE not enabled, required by STM32_MCO2SEL"
  #endif

  #if STM32_LSCOSEL == RCC_BDCR_LSCOSEL_LSE
    #error "LSE not enabled, required by STM32_LSCOSEL"
  #endif

  #if (STM32_USART1SEL == RCC_CCIPR_USART1SEL_LSE)
    #error "LSE not enabled, required by STM32_USART1SEL"
  #endif
  #if (STM32_USART2SEL == RCC_CCIPR_USART2SEL_LSE)
    #error "LSE not enabled, required by STM32_USART2SEL"
  #endif
  #if (STM32_LPUART1SEL == RCC_CCIPR_LPUART1SEL_LSE)
    #error "LSE not enabled, required by STM32_LPUART1SEL"
  #endif
  #if (STM32_LPUART2SEL == RCC_CCIPR_LPUART2SEL_LSE)
    #error "LSE not enabled, required by STM32_LPUART2SEL"
  #endif
  #if (STM32_LPUART3SEL == RCC_CCIPR_LPUART3SEL_LSE)
    #error "LSE not enabled, required by STM32_LPUART3SEL"
  #endif

  #if (STM32_LPTIM1SEL == RCC_CCIPR_LPTIM1SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM1SEL"
  #endif
  #if (STM32_LPTIM2SEL == RCC_CCIPR_LPTIM2SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM2SEL"
  #endif
  #if (STM32_LPTIM3SEL == RCC_CCIPR_LPTIM3SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM3SEL"
  #endif

#endif /* !STM32_LSE_ENABLED */

/**
 * @brief   PLL input clock frequency.
 */
#if (STM32_PLLSRC == RCC_PLLCFGR_PLLSRC_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_PLLCLKIN            0

#elif STM32_PLLSRC == RCC_PLLCFGR_PLLSRC_MSI
  #define STM32_PLLCLKIN            (STM32_MSICLK / STM32_PLLM_VALUE)

#elif STM32_PLLSRC == RCC_PLLCFGR_PLLSRC_HSI16
  #define STM32_PLLCLKIN            (STM32_HSI16CLK / STM32_PLLM_VALUE)

#elif STM32_PLLSRC == RCC_PLLCFGR_PLLSRC_HSE
  #define STM32_PLLCLKIN            (STM32_HSECLK / STM32_PLLM_VALUE)

#else
#error "invalid STM32_PLLSRC value specified"
#endif

/*
 * PLL enable check.
 */
#if (STM32_SW == RCC_CFGR_SW_PLLR) ||                                       \
    (STM32_MCO1SEL == RCC_CFGR_MCO1SEL_PLLRCLK) ||                          \
    (STM32_MCO2SEL == RCC_CFGR_MCO2SEL_PLLRCLK) ||                          \
    (STM32_TIM1SEL == RCC_CCIPR_TIM1SEL_PLLQCLK) ||                         \
    (STM32_TIM15SEL == RCC_CCIPR_TIM15SEL_PLLQCLK) ||                       \
    (STM32_CLK48SEL == RCC_CCIPR_CLK48SEL_PLLQCLK) ||                       \
    (STM32_ADCSEL == RCC_CCIPR_ADCSEL_PLLPCLK) ||                           \
    defined(__DOXYGEN__)
  /**
   * @brief   PLL activation flag.
   */
  #define STM32_ACTIVATE_PLL                TRUE

#else
  #define STM32_ACTIVATE_PLL                FALSE
#endif

  /**
   * @brief   STM32_PLLREN field.
   */
#if (STM32_SW == RCC_CFGR_SW_PLLR) ||                                       \
    (STM32_MCO1SEL == RCC_CFGR_MCO1SEL_PLLRCLK) ||                          \
    (STM32_MCO2SEL == RCC_CFGR_MCO2SEL_PLLRCLK) ||                          \
    defined(__DOXYGEN__)
  #define STM32_PLLREN                      RCC_PLLCFGR_PLLREN

#else
  #define STM32_PLLREN                      0U
#endif

/**
 * @brief   STM32_PLLQEN field.
 */
#if (STM32_TIM1SEL == RCC_CCIPR_TIM1SEL_PLLQCLK) ||                         \
    (STM32_TIM15SEL == RCC_CCIPR_TIM15SEL_PLLQCLK) ||                       \
    (STM32_CLK48SEL == RCC_CCIPR_CLK48SEL_PLLQCLK) ||                       \
    defined(__DOXYGEN__)
  #define STM32_PLLQEN                      RCC_PLLCFGR_PLLQEN

#else
  #define STM32_PLLQEN                      0U
#endif

/**
 * @brief   STM32_PLLPEN field.
 */
#if (STM32_ADCSEL == RCC_CCIPR_ADCSEL_PLLPCLK) ||                           \
    defined(__DOXYGEN__)
  #define STM32_PLLPEN                      RCC_PLLCFGR_PLLPEN

#else
  #define STM32_PLLPEN                      0U
#endif

/* Inclusion of PLL-related checks and calculations.*/
#include <stm32_pll_v2.inc>

/**
 * @brief   System clock source.
 */
#if (STM32_SW == RCC_CFGR_SW_MSI)
  #define STM32_SYSCLK                      STM32_MSICLK

#elif (STM32_SW == RCC_CFGR_SW_HSI16)
  #define STM32_SYSCLK                      STM32_HSI16CLK

#elif (STM32_SW == RCC_CFGR_SW_HSE)
  #define STM32_SYSCLK                      STM32_HSECLK

#elif (STM32_SW == RCC_CFGR_SW_PLLR)
  #define STM32_SYSCLK                      STM32_PLL_R_CLKOUT

#elif (STM32_SW == RCC_CFGR_SW_LSI)
  #define STM32_SYSCLK                      STM32_LSICLK

#elif (STM32_SW == RCC_CFGR_SW_LSE)
  #define STM32_SYSCLK                      STM32_LSECLK

#else
#error "invalid STM32_SW value specified"
#endif

/* Bus handlers.*/
#include "stm32_ahb.inc"
#include "stm32_apb.inc"

/*
 * Compatibility definitions.
 */
#define STM32_PCLK1                         STM32_PCLK
#define STM32_PCLK2                         STM32_PCLK

/**
 * @brief   MCO divider clock source.
 */
#if (STM32_MCO1SEL == RCC_CFGR_MCO1SEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_MCO1DIVCLK                  0

#elif STM32_MCO1SEL == RCC_CFGR_MCO1SEL_SYSCLK
  #define STM32_MCO1DIVCLK                  STM32_SYSCLK

#elif STM32_MCO1SEL == RCC_CFGR_MCO1SEL_MSI
  #define STM32_MCO1DIVCLK                  STM32_MSICLK

#elif STM32_MCO1SEL == RCC_CFGR_MCO1SEL_HSI16
  #define STM32_MCO1DIVCLK                  STM32_HSI16CLK

#elif STM32_MCO1SEL == RCC_CFGR_MCO1SEL_HSE
  #define STM32_MCO1DIVCLK                  STM32_HSECLK

#elif STM32_MCO1SEL == RCC_CFGR_MCO1SEL_PLLRCLK
  #define STM32_MCO1DIVCLK                  STM32_PLL_R_CLKOUT

#elif STM32_MCO1SEL == RCC_CFGR_MCO1SEL_LSI
  #define STM32_MCO1DIVCLK                  STM32_LSICLK

#elif STM32_MCO1SEL == RCC_CFGR_MCO1SEL_LSE
  #define STM32_MCO1DIVCLK                  STM32_LSECLK

#elif STM32_MCO1SEL == RCC_CFGR_MCO1SEL_HSI48
  #define STM32_MCO1DIVCLK                  STM32_HSI48CLK

#elif STM32_MCO1SEL == RCC_CFGR_MCO1SEL_RTCCLK
  #define STM32_MCO1DIVCLK                  STM32_RTCCLK

#elif STM32_MCO1SEL == RCC_CFGR_MCO1SEL_RTCWKP
  #define STM32_MCO1DIVCLK                  0   /* TODO */

#else
  #error "invalid STM32_MCO1SEL value specified"
#endif

/**
 * @brief   MCO output pin clock divider.
 */
#if (STM32_MCO1PRE == RCC_CFGR_MCO1PRE_DIV1) || defined(__DOXYGEN__)
  #define STM32_MCO1CLK                     STM32_MCO1DIVCLK

#elif STM32_MCO1PRE == RCC_CFGR_MCO1PRE_DIV2
  #define STM32_MCO1CLK                     (STM32_MCO1DIVCLK / 2)

#elif STM32_MCO1PRE == RCC_CFGR_MCO1PRE_DIV4
  #define STM32_MCO1CLK                     (STM32_MCO1DIVCLK / 4)

#elif STM32_MCO1PRE == RCC_CFGR_MCO1PRE_DIV8
  #define STM32_MCO1CLK                     (STM32_MCO1DIVCLK / 8)

#elif STM32_MCO1PRE == RCC_CFGR_MCO1PRE_DIV16
  #define STM32_MCO1CLK                     (STM32_MCO1DIVCLK / 16)

#elif STM32_MCO1PRE == RCC_CFGR_MCO1PRE_DIV32
  #define STM32_MCO1CLK                     (STM32_MCO1DIVCLK / 32)

#elif STM32_MCO1PRE == RCC_CFGR_MCO1PRE_DIV64
  #define STM32_MCO1CLK                     (STM32_MCO1DIVCLK / 64)

#elif STM32_MCO1PRE == RCC_CFGR_MCO1PRE_DIV128
  #define STM32_MCO1CLK                     (STM32_MCO1DIVCLK / 128)

#elif STM32_MCO1PRE == RCC_CFGR_MCO1PRE_DIV256
  #define STM32_MCO1CLK                     (STM32_MCO1DIVCLK / 256)

#elif STM32_MCO1PRE == RCC_CFGR_MCO1PRE_DIV512
  #define STM32_MCO1CLK                     (STM32_MCO1DIVCLK / 512)

#elif STM32_MCO1PRE == RCC_CFGR_MCO1PRE_DIV1024
  #define STM32_MCO1CLK                     (STM32_MCO1DIVCLK / 1024)

#else
  #error "invalid STM32_MCO1PRE value specified"
#endif

/**
 * @brief   MCO2 divider clock source.
 */
#if (STM32_MCO2SEL == RCC_CFGR_MCO2SEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_MCO2DIVCLK                  0

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_SYSCLK
  #define STM32_MCO2DIVCLK                  STM32_SYSCLK

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_MSI
  #define STM32_MCO2DIVCLK                  STM32_MSICLK

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_HSI16
  #define STM32_MCO2DIVCLK                  STM32_HSI16CLK

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_HSE
  #define STM32_MCO2DIVCLK                  STM32_HSECLK

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_PLLRCLK
  #define STM32_MCO2DIVCLK                  STM32_PLL_R_CLKOUT

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_LSI
  #define STM32_MCO2DIVCLK                  STM32_LSICLK

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_LSE
  #define STM32_MCO2DIVCLK                  STM32_LSECLK

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_HSI48
  #define STM32_MCO2DIVCLK                  STM32_HSI48CLK

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_RTCCLK
  #define STM32_MCO2DIVCLK                  STM32_RTCCLK

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_RTCWKP
  #define STM32_MCO2DIVCLK                  0   /* TODO */

#else
  #error "invalid STM32_MCO1SEL value specified"
#endif

/**
 * @brief   MCO2 output pin clock divider.
 */
#if (STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV1) || defined(__DOXYGEN__)
  #define STM32_MCO2CLK                     STM32_MCO2DIVCLK

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV2
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 2)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV4
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 4)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV8
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 8)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV16
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 16)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV32
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 32)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV64
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 64)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV128
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 128)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV256
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 256)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV512
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 512)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV1024
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 1024)

#else
  #error "invalid STM32_MCO2PRE value specified"
#endif

/**
 * @brief   LSCO clock source.
 */
#if (STM32_LSCOSEL == RCC_BDCR_LSCOSEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_LSCOCLK                     0

#elif STM32_LSCOSEL == RCC_BDCR_LSCOSEL_LSI
  #define STM32_LSCOCLK                     STM32_LSICLK

#elif STM32_LSCOSEL == RCC_BDCR_LSCOSEL_LSE
  #define STM32_LSCOCLK                     STM32_LSECLK

#else
  #error "invalid STM32_LSCOSEL value specified"
#endif

/**
 * @brief   RTC clock frequency.
 */
#if (STM32_RTCSEL == RCC_BDCR_RTCSEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_RTCCLK                      0

#elif STM32_RTCSEL == RCC_BDCR_RTCSEL_LSE
  #define STM32_RTCCLK                      STM32_LSECLK

#elif STM32_RTCSEL == RCC_BDCR_RTCSEL_LSI
  #define STM32_RTCCLK                      STM32_LSICLK

#elif STM32_RTCSEL == RCC_BDCR_RTCSEL_HSEDIV
  #define STM32_RTCCLK                      (hal_lld_get_clock_point(CLK_HSE) / 32)

#else
  #error "invalid STM32_RTCSEL value specified"
#endif

/**
 * @brief   USART1 clock frequency.
 */
#if (STM32_USART1SEL == RCC_CCIPR_USART1SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_USART1CLK                   hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_USART1SEL == RCC_CCIPR_USART1SEL_SYSCLK
  #define STM32_USART1CLK                   hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_USART1SEL == RCC_CCIPR_USART1SEL_HSI16
  #define STM32_USART1CLK                   hal_lld_get_clock_point(CLK_HSI16)

#elif STM32_USART1SEL == RCC_CCIPR_USART1SEL_LSE
  #define STM32_USART1CLK                   STM32_LSECLK

#else
  #error "invalid source selected for USART1 clock"
#endif

/**
 * @brief   USART2 clock frequency.
 */
#if (STM32_USART2SEL == RCC_CCIPR_USART2SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_USART2CLK                   hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_USART2SEL == RCC_CCIPR_USART2SEL_SYSCLK
  #define STM32_USART2CLK                   hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_USART2SEL == RCC_CCIPR_USART2SEL_HSI16
  #define STM32_USART2CLK                   hal_lld_get_clock_point(CLK_HSI16)

#elif STM32_USART2SEL == RCC_CCIPR_USART2SEL_LSE
  #define STM32_USART2CLK                   STM32_LSECLK

#else
  #error "invalid source selected for USART2 clock"
#endif

/**
 * @brief   LPUART1 clock frequency.
 */
#if (STM32_LPUART1SEL == RCC_CCIPR_LPUART1SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_LPUART1CLK                  hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_LPUART1SEL == RCC_CCIPR_LPUART1SEL_SYSCLK
  #define STM32_LPUART1CLK                  hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_LPUART1SEL == RCC_CCIPR_LPUART1SEL_HSI16
  #define STM32_LPUART1CLK                  hal_lld_get_clock_point(CLK_HSI16)

#elif STM32_LPUART1SEL == RCC_CCIPR_LPUART1SEL_LSE
  #define STM32_LPUART1CLK                  STM32_LSECLK

#else
  #error "invalid source selected for LPUART1 clock"
#endif

/**
 * @brief   LPUART2 clock frequency.
 */
#if (STM32_LPUART2SEL == RCC_CCIPR_LPUART2SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_LPUART2CLK                  hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_LPUART2SEL == RCC_CCIPR_LPUART2SEL_SYSCLK
  #define STM32_LPUART2CLK                  hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_LPUART2SEL == RCC_CCIPR_LPUART2SEL_HSI16
  #define STM32_LPUART2CLK                  hal_lld_get_clock_point(CLK_HSI16)

#elif STM32_LPUART2SEL == RCC_CCIPR_LPUART2SEL_LSE
  #define STM32_LPUART2CLK                  STM32_LSECLK

#else
  #error "invalid source selected for LPUART2 clock"
#endif

/**
 * @brief   LPUART3 clock frequency.
 */
#if (STM32_LPUART3SEL == RCC_CCIPR_LPUART3SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_LPUART3CLK                  hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_LPUART3SEL == RCC_CCIPR_LPUART3SEL_SYSCLK
  #define STM32_LPUART3CLK                  hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_LPUART3SEL == RCC_CCIPR_LPUART3SEL_HSI16
  #define STM32_LPUART3CLK                  hal_lld_get_clock_point(CLK_HSI16)

#elif STM32_LPUART3SEL == RCC_CCIPR_LPUART3SEL_LSE
  #define STM32_LPUART3CLK                  STM32_LSECLK

#else
  #error "invalid source selected for LPUART3 clock"
#endif

/**
 * @brief   I2C1 clock frequency.
 */
#if (STM32_I2C1SEL == RCC_CCIPR_I2C1SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_I2C1CLK                     hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_I2C1SEL == RCC_CCIPR_I2C1SEL_SYSCLK
  #define STM32_I2C1CLK                     hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_I2C1SEL == RCC_CCIPR_I2C1SEL_HSI16
  #define STM32_I2C1CLK                     hal_lld_get_clock_point(CLK_HSI16)

#else
  #error "invalid source selected for I2C1 clock"
#endif

/**
 * @brief   I2C3 clock frequency.
 */
#if (STM32_I2C3SEL == RCC_CCIPR_I2C3SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_I2C3CLK                     hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_I2C3SEL == RCC_CCIPR_I2C3SEL_SYSCLK
  #define STM32_I2C3CLK                     hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_I2C3SEL == RCC_CCIPR_I2C3SEL_HSI16
  #define STM32_I2C3CLK                     hal_lld_get_clock_point(CLK_HSI16)

#else
  #error "invalid source selected for I2C3 clock"
#endif

/**
 * @brief   LPTIM1 clock frequency.
 */
#if (STM32_LPTIM1SEL == RCC_CCIPR_LPTIM1SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_LPTIM1CLK                   hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_LPTIM1SEL == RCC_CCIPR_LPTIM1SEL_LSI
  #define STM32_LPTIM1CLK                   STM32_LSICLK

#elif STM32_LPTIM1SEL == RCC_CCIPR_LPTIM1SEL_HSI16
  #define STM32_LPTIM1CLK                   hal_lld_get_clock_point(CLK_HSI16)

#elif STM32_LPTIM1SEL == RCC_CCIPR_LPTIM1SEL_LSE
  #define STM32_LPTIM1CLK                   STM32_LSECLK

#else
#error "invalid source selected for LPTIM1 clock"
#endif

/**
 * @brief   LPTIM2 clock frequency.
 */
#if (STM32_LPTIM2SEL == RCC_CCIPR_LPTIM2SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_LPTIM2CLK                   hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_LPTIM2SEL == RCC_CCIPR_LPTIM2SEL_LSI
  #define STM32_LPTIM2CLK                   STM32_LSICLK

#elif STM32_LPTIM2SEL == RCC_CCIPR_LPTIM2SEL_HSI16
  #define STM32_LPTIM2CLK                   hal_lld_get_clock_point(CLK_HSI16)

#elif STM32_LPTIM2SEL == RCC_CCIPR_LPTIM2SEL_LSE
  #define STM32_LPTIM2CLK                   STM32_LSECLK

#else
#error "invalid source selected for LPTIM2 clock"
#endif

/**
 * @brief   LPTIM3 clock frequency.
 */
#if (STM32_LPTIM3SEL == RCC_CCIPR_LPTIM3SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_LPTIM3CLK                   hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_LPTIM3SEL == RCC_CCIPR_LPTIM3SEL_LSI
  #define STM32_LPTIM3CLK                   STM32_LSICLK

#elif STM32_LPTIM3SEL == RCC_CCIPR_LPTIM3SEL_HSI16
  #define STM32_LPTIM3CLK                   hal_lld_get_clock_point(CLK_HSI16)

#elif STM32_LPTIM3SEL == RCC_CCIPR_LPTIM3SEL_LSE
  #define STM32_LPTIM3CLK                   STM32_LSECLK

#else
#error "invalid source selected for LPTIM3 clock"
#endif

/**
 * @brief   CLK48 clock frequency.
 */
#if (STM32_CLK48SEL == RCC_CCIPR_CLK48SEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_CLK48CLK                    0

#elif STM32_CLK48SEL == RCC_CCIPR_CLK48SEL_MSI
  #define STM32_CLK48CLK                    hal_lld_get_clock_point(CLK_MSI)

#elif STM32_CLK48SEL == RCC_CCIPR_CLK48SEL_PLLQCLK
  #define STM32_CLK48CLK                    hal_lld_get_clock_point(CLK_PLLQ)

#elif STM32_CLK48SEL == RCC_CCIPR_CLK48SEL_HSI48
  #define STM32_CLK48CLK                    hal_lld_get_clock_point(CLK_HSI48)

#else
#error "invalid source selected for CLK48 clock"
#endif

/**
 * @brief   ADC clock frequency.
 */
#if (STM32_ADCSEL == RCC_CCIPR_ADCSEL_SYSCLK) || defined(__DOXYGEN__)
  #define STM32_ADCCLK                      hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_ADCSEL == RCC_CCIPR_ADCSEL_PLLPCLK
  #define STM32_ADCCLK                      hal_lld_get_clock_point(CLK_PLLP)

#elif STM32_ADCSEL == RCC_CCIPR_ADCSEL_HSI16
  #define STM32_ADCCLK                      hal_lld_get_clock_point(CLK_HSI16)

#else
  #error "invalid source selected for ADC clock"
#endif

/**
 * @brief   TIMPCLK clock frequency.
 */
#if (STM32_PPRE == RCC_CFGR_PPRE_DIV1) || defined(__DOXYGEN__)
  #define STM32_TIMPCLK                     (STM32_PCLK * 1)

#else
  #define STM32_TIMPCLK                     (STM32_PCLK * 2)
#endif

/**
 * @brief   TIM1 clock frequency.
 */
#if (STM32_TIM1SEL == RCC_CCIPR_TIM1SEL_TIMPCLK) || defined(__DOXYGEN__)
  #define STM32_TIM1CLK                     hal_lld_get_clock_point(CLK_PCLKTIM)

#elif STM32_TIM1SEL == RCC_CCIPR_TIM1SEL_PLLQCLK
  #define STM32_TIM1CLK                     hal_lld_get_clock_point(CLK_PLLQ)

#else
  #error "invalid source selected for TIM1 clock"
#endif

/**
 * @brief   TIM15 clock frequency.
 */
#if (STM32_TIM15SEL == RCC_CCIPR_TIM15SEL_TIMPCLK) || defined(__DOXYGEN__)
  #define STM32_TIM15CLK                    hal_lld_get_clock_point(CLK_PCLKTIM)

#elif STM32_TIM15SEL == RCC_CCIPR_TIM15SEL_PLLQCLK
  #define STM32_TIM15CLK                    hal_lld_get_clock_point(CLK_PLLQ)

#else
#error "invalid source selected for TIM15 clock"
#endif

/**
 * @brief   Clock of timers connected to APB1.
 */
#define STM32_TIMCLK1                       hal_lld_get_clock_point(CLK_PCLKTIM)

/**
 * @brief   Clock of timers connected to APB2.
 */
#define STM32_TIMCLK2                       hal_lld_get_clock_point(CLK_PCLKTIM)

/**
 * @brief   USB clock point.
 */
#define STM32_USBCLK                        hal_lld_get_clock_point(CLK_HSI48)

/**
 * @brief   Flash settings.
 */
#if (STM32_HCLK <= STM32_0WS_THRESHOLD) || defined(__DOXYGEN__)
  #define STM32_FLASHBITS                   0

#elif STM32_HCLK <= STM32_1WS_THRESHOLD
  #define STM32_FLASHBITS                   FLASH_ACR_LATENCY_0

#elif STM32_HCLK <= STM32_2WS_THRESHOLD
  #define STM32_FLASHBITS                   FLASH_ACR_LATENCY_1

#else
  #define STM32_FLASHBITS                   (FLASH_ACR_LATENCY_1 | FLASH_ACR_LATENCY_0)
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a clock configuration and switch structure.
 */
typedef struct {
  uint32_t          pwr_cr1;
  uint32_t          rcc_cr;
  uint32_t          rcc_cfgr;
  uint32_t          rcc_pllcfgr;
  uint32_t          rcc_crrcr;
  uint32_t          flash_acr;
} halclkcfg_t;

/**
 * @brief   Type of a timeout counter.
 * @note    16 bits because it must match TIM7 counter size.
 */
typedef uint16_t halcnt_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Real time counter frequency exported to the safety module.
 * @note    The counter is clocked at 1MHz, the prescaled is recalculated
 *          if/when the input frequency changes.
 */
#define HAL_LLD_GET_CNT_FREQUENCY()         1000000U

/**
 * @brief   Real time counter value exported to the safety module.
 */
#define HAL_LLD_GET_CNT_VALUE()             ((halcnt_t)TIM7->CNT)

#if !defined(HAL_LLD_USE_CLOCK_MANAGEMENT)
/**
 * @brief   Returns the frequency of a clock point in Hz.
 * @note    Static implementation.
 *
 * @param[in] clkpt     clock point to be returned
 * @return              The clock point frequency in Hz or zero if the
 *                      frequency is unknown.
 *
 * @notapi
 */
  #define hal_lld_get_clock_point(clkpt)                                    \
    ((clkpt) == CLK_SYSCLK    ? STM32_SYSCLK        :                       \
     (clkpt) == CLK_HSI16     ? STM32_HSI16CLK      :                       \
     (clkpt) == CLK_HSI48     ? STM32_HSI48CLK      :                       \
     (clkpt) == CLK_HSE       ? STM32_HSECLK        :                       \
     (clkpt) == CLK_MSI       ? STM32_MSICLK        :                       \
     (clkpt) == CLK_PLLP      ? STM32_PLL_P_CLKOUT  :                       \
     (clkpt) == CLK_PLLQ      ? STM32_PLL_Q_CLKOUT  :                       \
     (clkpt) == CLK_PLLR      ? STM32_PLL_R_CLKOUT  :                       \
     (clkpt) == CLK_HCLK      ? STM32_HCLK          :                       \
     (clkpt) == CLK_PCLK      ? STM32_PCLK          :                       \
     (clkpt) == CLK_PCLKTIM   ? STM32_TIMPCLK       :                       \
     (clkpt) == CLK_MCO1      ? STM32_MCO1CLK       :                       \
     (clkpt) == CLK_MCO2      ? STM32_MCO2CLK       :                       \
     0U)
#endif /* !defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "stm32_isr.h"
#include "stm32_dma.h"
#include "stm32_exti.h"
#include "stm32_rcc.h"
#include "stm32_tim.h"

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) && !defined(__DOXYGEN__)
extern const halclkcfg_t hal_clkcfg_reset;
extern const halclkcfg_t hal_clkcfg_default;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void stm32_clock_init(void);
#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
  bool hal_lld_clock_switch_mode(const halclkcfg_t *ccp);
  halfreq_t hal_lld_get_clock_point(halclkpt_t clkpt);
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
