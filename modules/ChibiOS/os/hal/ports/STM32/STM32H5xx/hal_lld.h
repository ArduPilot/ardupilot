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
 * @file    STM32H5xx/hal_lld.h
 * @brief   STM32H5xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32H503xx.
 *          - STM32H562xx, STM32H563xx, STM32H573xx.
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
#define HAL_LLD_SELECT_SPI_V2           TRUE

/**
 * @name    Platform identification
 * @{
 */
#if defined(STM32H503xx) || defined(__DOXYGEN__)
  #define PLATFORM_NAME         "STM32H5 High-performance"

#elif defined(STM32H562xx) || defined(STM32H563xx)
  #define PLATFORM_NAME         "STM32H5 High-performance"

#elif defined(STM32H573xx)
  #define PLATFORM_NAME         "STM32H5 High-performance with security"

#else
  #error "STM32H5 device not specified"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(STM32H5XX) || defined(__DOXYGEN__)
  #define STM32H5XX
#endif
/** @} */

/**
 * @name   Clock points names
 * @{
 */
#if STM32_RCC_HAS_PLL3 || defined(__DOXYGEN__)
#define CLK_HSI                 0U
#define CLK_CSI                 1U
#define CLK_HSI48               2U
#define CLK_HSE                 3U
#define CLK_SYSCLK              4U
#define CLK_PLL1PCLK            5U
#define CLK_PLL1QCLK            6U
#define CLK_PLL1RCLK            7U
#define CLK_PLL2PCLK            8U
#define CLK_PLL2QCLK            9U
#define CLK_PLL2RCLK            10U
#define CLK_PLL3PCLK            11U
#define CLK_PLL3QCLK            12U
#define CLK_PLL3RCLK            13U
#define CLK_HCLK                14U
#define CLK_PCLK1               15U
#define CLK_PCLK1TIM            16U
#define CLK_PCLK2               17U
#define CLK_PCLK2TIM            18U
#define CLK_PCLK3               19U
#define CLK_MCO1                20U
#define CLK_MCO2                21U
#define CLK_ARRAY_SIZE          22U

#define CLK_POINT_NAMES                                                     \
  {                                                                         \
    "HSI", "CSI", "HSI48", "HSE", "SYSCLK", "PLL1P", "PLL1Q", "PLL1R",      \
    "PLL2P", "PLL2Q", "PLL2R", "PLL3P", "PLL3Q", "PLL3R", "HCLK",           \
    "PCLK1", "PCLK1TIM", "PCLK2", "PCLK2TIM", "PCLK3", "MCO1", "MCO2"       \
  }
#else
#define CLK_HSI                 0U
#define CLK_CSI                 1U
#define CLK_HSI48               2U
#define CLK_HSE                 3U
#define CLK_SYSCLK              4U
#define CLK_PLL1PCLK            5U
#define CLK_PLL1QCLK            6U
#define CLK_PLL1RCLK            7U
#define CLK_PLL2PCLK            8U
#define CLK_PLL2QCLK            9U
#define CLK_PLL2RCLK            10U
#define CLK_HCLK                11U
#define CLK_PCLK1               12U
#define CLK_PCLK1TIM            13U
#define CLK_PCLK2               14U
#define CLK_PCLK2TIM            15U
#define CLK_PCLK3               16U
#define CLK_MCO1                17U
#define CLK_MCO2                18U
#define CLK_ARRAY_SIZE          19U

#define CLK_POINT_NAMES                                                     \
  {                                                                         \
    "HSI", "CSI", "HSI48", "HSE", "SYSCLK", "PLL1P", "PLL1Q", "PLL1R",      \
    "PLL2P", "PLL2Q", "PLL2R", "HCLK", "PCLK1", "PCLK1TIM", "PCLK2",        \
    "PCLK2TIM", "PCLK3", "MCO1", "MCO2"                                     \
  }
#endif
/** @} */

/*===========================================================================*/
/* RCC/PWR/FLASH configuration aliases.                                      */
/*===========================================================================*/

/* Missing bit position definitions on some headers.*/
#ifndef RCC_CCIPR4_OSPISEL_Pos
#define RCC_CCIPR4_OSPISEL_Pos              0U
#endif

#ifndef RCC_CCIPR5_CKPERSEL_Pos
#define RCC_CCIPR5_CKPERSEL_Pos             30U
#endif

#ifndef RCC_CCIPR1_UART4SEL_Pos
#define RCC_CCIPR1_UART4SEL_Pos             9U
#endif

#ifndef RCC_CCIPR1_UART5SEL_Pos
#define RCC_CCIPR1_UART5SEL_Pos             12U
#endif

#ifndef RCC_CCIPR1_USART6SEL_Pos
#define RCC_CCIPR1_USART6SEL_Pos            15U
#endif

#ifndef RCC_CCIPR1_UART7SEL_Pos
#define RCC_CCIPR1_UART7SEL_Pos             18U
#endif

#ifndef RCC_CCIPR1_UART8SEL_Pos
#define RCC_CCIPR1_UART8SEL_Pos             21U
#endif

#ifndef RCC_CCIPR1_UART9SEL_Pos
#define RCC_CCIPR1_UART9SEL_Pos             24U
#endif

#ifndef RCC_CCIPR1_USART10SEL_Pos
#define RCC_CCIPR1_USART10SEL_Pos           27U
#endif

#ifndef RCC_CCIPR2_USART11SEL_Pos
#define RCC_CCIPR2_USART11SEL_Pos           0U
#endif

#ifndef RCC_CCIPR2_UART12SEL_Pos
#define RCC_CCIPR2_UART12SEL_Pos            4U
#endif

#ifndef RCC_CCIPR2_LPTIM3SEL_Pos
#define RCC_CCIPR2_LPTIM3SEL_Pos            16U
#endif

#ifndef RCC_CCIPR2_LPTIM4SEL_Pos
#define RCC_CCIPR2_LPTIM4SEL_Pos            20U
#endif

#ifndef RCC_CCIPR2_LPTIM5SEL_Pos
#define RCC_CCIPR2_LPTIM5SEL_Pos            24U
#endif

#ifndef RCC_CCIPR2_LPTIM6SEL_Pos
#define RCC_CCIPR2_LPTIM6SEL_Pos            28U
#endif

#ifndef RCC_CCIPR3_SPI4SEL_Pos
#define RCC_CCIPR3_SPI4SEL_Pos              9U
#endif

#ifndef RCC_CCIPR3_SPI5SEL_Pos
#define RCC_CCIPR3_SPI5SEL_Pos              12U
#endif

#ifndef RCC_CCIPR3_SPI6SEL_Pos
#define RCC_CCIPR3_SPI6SEL_Pos              15U
#endif

#ifndef RCC_CCIPR4_SDMMC1SEL_Pos
#define RCC_CCIPR4_SDMMC1SEL_Pos            6U
#endif

#ifndef RCC_CCIPR4_SDMMC2SEL_Pos
#define RCC_CCIPR4_SDMMC2SEL_Pos            7U
#endif

#ifndef RCC_CCIPR4_I2C3SEL_Pos
#define RCC_CCIPR4_I2C3SEL_Pos              20U
#endif

#ifndef RCC_CCIPR4_I2C4SEL_Pos
#define RCC_CCIPR4_I2C4SEL_Pos              22U
#endif

#ifndef RCC_CCIPR5_CECSEL_Pos
#define RCC_CCIPR5_CECSEL_Pos               6U
#endif

#ifndef RCC_CCIPR5_SAI1SEL_Pos
#define RCC_CCIPR5_SAI1SEL_Pos              16U
#endif

#ifndef RCC_CCIPR5_SAI2SEL_Pos
#define RCC_CCIPR5_SAI2SEL_Pos              19U
#endif

/**
 * @name    PWR_VOSCR register helpers
 * @{
 */
#define PWR_VOSCR_VOS_FIELD(n)              ((n) << PWR_VOSCR_VOS_Pos)
#define PWR_VOSCR_VOS_RANGE3                PWR_VOSCR_VOS_FIELD(0U)
#define PWR_VOSCR_VOS_RANGE2                PWR_VOSCR_VOS_FIELD(1U)
#define PWR_VOSCR_VOS_RANGE1                PWR_VOSCR_VOS_FIELD(2U)
#define PWR_VOSCR_VOS_RANGE0                PWR_VOSCR_VOS_FIELD(3U)
/** @} */

/**
 * @name    RCC_CR register helpers
 * @{
 */
#define RCC_CR_HSIDIV_FIELD(n)              ((n) << RCC_CR_HSIDIV_Pos)
#define RCC_CR_HSIDIV_DIV1                  RCC_CR_HSIDIV_FIELD(0U)
#define RCC_CR_HSIDIV_DIV2                  RCC_CR_HSIDIV_FIELD(1U)
#define RCC_CR_HSIDIV_DIV4                  RCC_CR_HSIDIV_FIELD(2U)
#define RCC_CR_HSIDIV_DIV8                  RCC_CR_HSIDIV_FIELD(3U)
/** @} */

/**
 * @name    RCC_CFGR1 register helpers
 * @{
 */
#define RCC_CFGR1_SW_FIELD(n)               ((n) << RCC_CFGR1_SW_Pos)
#define RCC_CFGR1_SW_HSI                    RCC_CFGR1_SW_FIELD(0U)
#define RCC_CFGR1_SW_CSI                    RCC_CFGR1_SW_FIELD(1U)
#define RCC_CFGR1_SW_HSE                    RCC_CFGR1_SW_FIELD(2U)
#define RCC_CFGR1_SW_PLL1P                  RCC_CFGR1_SW_FIELD(3U)

#define RCC_CFGR1_SWS_FIELD(n)              ((n) << RCC_CFGR1_SWS_Pos)
#define RCC_CFGR1_SWS_HSI                   RCC_CFGR1_SWS_FIELD(0U)
#define RCC_CFGR1_SWS_CSI                   RCC_CFGR1_SWS_FIELD(1U)
#define RCC_CFGR1_SWS_HSE                   RCC_CFGR1_SWS_FIELD(2U)
#define RCC_CFGR1_SWS_PLL1P                 RCC_CFGR1_SWS_FIELD(3U)

#define RCC_CFGR1_STOPWUCK_FIELD(n)         ((n) << RCC_CFGR1_STOPWUCK_Pos)
#define RCC_CFGR1_STOPWUCK_HSI              RCC_CFGR1_STOPWUCK_FIELD(0U)
#define RCC_CFGR1_STOPWUCK_CSI              RCC_CFGR1_STOPWUCK_FIELD(1U)

#define RCC_CFGR1_STOPKERWUCK_FIELD(n)      ((n) << RCC_CFGR1_STOPKERWUCK_Pos)
#define RCC_CFGR1_STOPKERWUCK_HSI           RCC_CFGR1_STOPKERWUCK_FIELD(0U)
#define RCC_CFGR1_STOPKERWUCK_CSI           RCC_CFGR1_STOPKERWUCK_FIELD(1U)

#define RCC_CFGR1_RTCPRE_FIELD(n)           ((n) << RCC_CFGR1_RTCPRE_Pos)
#define RCC_CFGR1_RTCPRE_NOCLOCK            RCC_CFGR1_RTCPRE_FIELD(0U)

#define RCC_CFGR1_TIMPRE_FIELD(n)           ((n) << RCC_CFGR1_TIMPRE_Pos)
#define RCC_CFGR1_TIMPRE_LOW                RCC_CFGR1_TIMPRE_FIELD(0U)
#define RCC_CFGR1_TIMPRE_HIGH               RCC_CFGR1_TIMPRE_FIELD(1U)

#define RCC_CFGR1_MCO1SEL_FIELD(n)          ((n) << RCC_CFGR1_MCO1SEL_Pos)
#define RCC_CFGR1_MCO1SEL_HSI               RCC_CFGR1_MCO1SEL_FIELD(0U)
#define RCC_CFGR1_MCO1SEL_LSE               RCC_CFGR1_MCO1SEL_FIELD(1U)
#define RCC_CFGR1_MCO1SEL_HSE               RCC_CFGR1_MCO1SEL_FIELD(2U)
#define RCC_CFGR1_MCO1SEL_PLL1P             RCC_CFGR1_MCO1SEL_FIELD(3U)
#define RCC_CFGR1_MCO1SEL_HSI48             RCC_CFGR1_MCO1SEL_FIELD(4U)

#define RCC_CFGR1_MCO1PRE_FIELD(n)          ((n) << RCC_CFGR1_MCO1PRE_Pos)
#define RCC_CFGR1_MCO1PRE_NOCLOCK           RCC_CFGR1_MCO1PRE_FIELD(0U)

#define RCC_CFGR1_MCO2SEL_FIELD(n)          ((n) << RCC_CFGR1_MCO2SEL_Pos)
#define RCC_CFGR1_MCO2SEL_SYSCLK            RCC_CFGR1_MCO2SEL_FIELD(0U)
#define RCC_CFGR1_MCO2SEL_PLL2P             RCC_CFGR1_MCO2SEL_FIELD(1U)
#define RCC_CFGR1_MCO2SEL_HSE               RCC_CFGR1_MCO2SEL_FIELD(2U)
#define RCC_CFGR1_MCO2SEL_PLL1P             RCC_CFGR1_MCO2SEL_FIELD(3U)
#define RCC_CFGR1_MCO2SEL_CSI               RCC_CFGR1_MCO2SEL_FIELD(4U)
#define RCC_CFGR1_MCO2SEL_LSI               RCC_CFGR1_MCO2SEL_FIELD(5U)

#define RCC_CFGR1_MCO2PRE_FIELD(n)          ((n) << RCC_CFGR1_MCO2PRE_Pos)
#define RCC_CFGR1_MCO2PRE_NOCLOCK           RCC_CFGR1_MCO2PRE_FIELD(0U)
/** @} */

/**
 * @name    RCC_PLL1CFGR register helpers
 * @{
 */
#define RCC_PLL1CFGR_PLL1SRC_FIELD(n)       ((n) << RCC_PLL1CFGR_PLL1SRC_Pos)
#define RCC_PLL1CFGR_PLL1SRC_NOCLOCK        RCC_PLL1CFGR_PLL1SRC_FIELD(0U)
#define RCC_PLL1CFGR_PLL1SRC_HSI            RCC_PLL1CFGR_PLL1SRC_FIELD(1U)
#define RCC_PLL1CFGR_PLL1SRC_CSI            RCC_PLL1CFGR_PLL1SRC_FIELD(2U)
#define RCC_PLL1CFGR_PLL1SRC_HSE            RCC_PLL1CFGR_PLL1SRC_FIELD(3U)
/** @} */

/**
 * @name    RCC_PLL2CFGR register helpers
 * @{
 */
#define RCC_PLL2CFGR_PLL2SRC_FIELD(n)       ((n) << RCC_PLL2CFGR_PLL2SRC_Pos)
#define RCC_PLL2CFGR_PLL2SRC_NOCLOCK        RCC_PLL2CFGR_PLL2SRC_FIELD(0U)
#define RCC_PLL2CFGR_PLL2SRC_HSI            RCC_PLL2CFGR_PLL2SRC_FIELD(1U)
#define RCC_PLL2CFGR_PLL2SRC_CSI            RCC_PLL2CFGR_PLL2SRC_FIELD(2U)
#define RCC_PLL2CFGR_PLL2SRC_HSE            RCC_PLL2CFGR_PLL2SRC_FIELD(3U)
/** @} */

/**
 * @name    RCC_PLL3CFGR register helpers
 * @{
 */
#define RCC_PLL3CFGR_PLL3SRC_FIELD(n)       ((n) << RCC_PLL3CFGR_PLL3SRC_Pos)
#define RCC_PLL3CFGR_PLL3SRC_NOCLOCK        RCC_PLL3CFGR_PLL3SRC_FIELD(0U)
#define RCC_PLL3CFGR_PLL3SRC_HSI            RCC_PLL3CFGR_PLL3SRC_FIELD(1U)
#define RCC_PLL3CFGR_PLL3SRC_CSI            RCC_PLL3CFGR_PLL3SRC_FIELD(2U)
#define RCC_PLL3CFGR_PLL3SRC_HSE            RCC_PLL3CFGR_PLL3SRC_FIELD(3U)
/** @} */

/**
 * @name    RCC_CCIPR1 register helpers
 * @{
 */
#define RCC_CCIPR1_TIMICSEL_FIELD(n)        ((n) << RCC_CCIPR1_TIMICSEL_Pos)
#define RCC_CCIPR1_TIMICSEL_NOCLK           RCC_CCIPR1_TIMICSEL_FIELD(0U)
#define RCC_CCIPR1_TIMICSEL_INTCLK          RCC_CCIPR1_TIMICSEL_FIELD(1U)

#define RCC_CCIPR1_USART1SEL_FIELD(n)       ((n) << RCC_CCIPR1_USART1SEL_Pos)
#define RCC_CCIPR1_USART1SEL_PCLK2          RCC_CCIPR1_USART1SEL_FIELD(0U)
#define RCC_CCIPR1_USART1SEL_PLL2Q          RCC_CCIPR1_USART1SEL_FIELD(1U)
#define RCC_CCIPR1_USART1SEL_PLL3Q          RCC_CCIPR1_USART1SEL_FIELD(2U)
#define RCC_CCIPR1_USART1SEL_HSI            RCC_CCIPR1_USART1SEL_FIELD(3U)
#define RCC_CCIPR1_USART1SEL_CSI            RCC_CCIPR1_USART1SEL_FIELD(4U)
#define RCC_CCIPR1_USART1SEL_LSE            RCC_CCIPR1_USART1SEL_FIELD(5U)

#define RCC_CCIPR1_USART2SEL_FIELD(n)       ((n) << RCC_CCIPR1_USART2SEL_Pos)
#define RCC_CCIPR1_USART2SEL_PCLK1          RCC_CCIPR1_USART2SEL_FIELD(0U)
#define RCC_CCIPR1_USART2SEL_PLL2Q          RCC_CCIPR1_USART2SEL_FIELD(1U)
#define RCC_CCIPR1_USART2SEL_PLL3Q          RCC_CCIPR1_USART2SEL_FIELD(2U)
#define RCC_CCIPR1_USART2SEL_HSI            RCC_CCIPR1_USART2SEL_FIELD(3U)
#define RCC_CCIPR1_USART2SEL_CSI            RCC_CCIPR1_USART2SEL_FIELD(4U)
#define RCC_CCIPR1_USART2SEL_LSE            RCC_CCIPR1_USART2SEL_FIELD(5U)

#define RCC_CCIPR1_USART3SEL_FIELD(n)       ((n) << RCC_CCIPR1_USART3SEL_Pos)
#define RCC_CCIPR1_USART3SEL_PCLK1          RCC_CCIPR1_USART3SEL_FIELD(0U)
#define RCC_CCIPR1_USART3SEL_PLL2Q          RCC_CCIPR1_USART3SEL_FIELD(1U)
#define RCC_CCIPR1_USART3SEL_PLL3Q          RCC_CCIPR1_USART3SEL_FIELD(2U)
#define RCC_CCIPR1_USART3SEL_HSI            RCC_CCIPR1_USART3SEL_FIELD(3U)
#define RCC_CCIPR1_USART3SEL_CSI            RCC_CCIPR1_USART3SEL_FIELD(4U)
#define RCC_CCIPR1_USART3SEL_LSE            RCC_CCIPR1_USART3SEL_FIELD(5U)

#define RCC_CCIPR1_UART4SEL_FIELD(n)        ((n) << RCC_CCIPR1_UART4SEL_Pos)
#define RCC_CCIPR1_UART4SEL_PCLK1           RCC_CCIPR1_UART4SEL_FIELD(0U)
#define RCC_CCIPR1_UART4SEL_PLL2Q           RCC_CCIPR1_UART4SEL_FIELD(1U)
#define RCC_CCIPR1_UART4SEL_PLL3Q           RCC_CCIPR1_UART4SEL_FIELD(2U)
#define RCC_CCIPR1_UART4SEL_HSI             RCC_CCIPR1_UART4SEL_FIELD(3U)
#define RCC_CCIPR1_UART4SEL_CSI             RCC_CCIPR1_UART4SEL_FIELD(4U)
#define RCC_CCIPR1_UART4SEL_LSE             RCC_CCIPR1_UART4SEL_FIELD(5U)

#define RCC_CCIPR1_UART5SEL_FIELD(n)        ((n) << RCC_CCIPR1_UART5SEL_Pos)
#define RCC_CCIPR1_UART5SEL_PCLK1           RCC_CCIPR1_UART5SEL_FIELD(0U)
#define RCC_CCIPR1_UART5SEL_PLL2Q           RCC_CCIPR1_UART5SEL_FIELD(1U)
#define RCC_CCIPR1_UART5SEL_PLL3Q           RCC_CCIPR1_UART5SEL_FIELD(2U)
#define RCC_CCIPR1_UART5SEL_HSI             RCC_CCIPR1_UART5SEL_FIELD(3U)
#define RCC_CCIPR1_UART5SEL_CSI             RCC_CCIPR1_UART5SEL_FIELD(4U)
#define RCC_CCIPR1_UART5SEL_LSE             RCC_CCIPR1_UART5SEL_FIELD(5U)

#define RCC_CCIPR1_USART6SEL_FIELD(n)       ((n) << RCC_CCIPR1_USART6SEL_Pos)
#define RCC_CCIPR1_USART6SEL_PCLK1          RCC_CCIPR1_USART6SEL_FIELD(0U)
#define RCC_CCIPR1_USART6SEL_PLL2Q          RCC_CCIPR1_USART6SEL_FIELD(1U)
#define RCC_CCIPR1_USART6SEL_PLL3Q          RCC_CCIPR1_USART6SEL_FIELD(2U)
#define RCC_CCIPR1_USART6SEL_HSI            RCC_CCIPR1_USART6SEL_FIELD(3U)
#define RCC_CCIPR1_USART6SEL_CSI            RCC_CCIPR1_USART6SEL_FIELD(4U)
#define RCC_CCIPR1_USART6SEL_LSE            RCC_CCIPR1_USART6SEL_FIELD(5U)

#define RCC_CCIPR1_UART7SEL_FIELD(n)        ((n) << RCC_CCIPR1_UART7SEL_Pos)
#define RCC_CCIPR1_UART7SEL_PCLK1           RCC_CCIPR1_UART7SEL_FIELD(0U)
#define RCC_CCIPR1_UART7SEL_PLL2Q           RCC_CCIPR1_UART7SEL_FIELD(1U)
#define RCC_CCIPR1_UART7SEL_PLL3Q           RCC_CCIPR1_UART7SEL_FIELD(2U)
#define RCC_CCIPR1_UART7SEL_HSI             RCC_CCIPR1_UART7SEL_FIELD(3U)
#define RCC_CCIPR1_UART7SEL_CSI             RCC_CCIPR1_UART7SEL_FIELD(4U)
#define RCC_CCIPR1_UART7SEL_LSE             RCC_CCIPR1_UART7SEL_FIELD(5U)

#define RCC_CCIPR1_UART8SEL_FIELD(n)        ((n) << RCC_CCIPR1_UART8SEL_Pos)
#define RCC_CCIPR1_UART8SEL_PCLK1           RCC_CCIPR1_UART8SEL_FIELD(0U)
#define RCC_CCIPR1_UART8SEL_PLL2Q           RCC_CCIPR1_UART8SEL_FIELD(1U)
#define RCC_CCIPR1_UART8SEL_PLL3Q           RCC_CCIPR1_UART8SEL_FIELD(2U)
#define RCC_CCIPR1_UART8SEL_HSI             RCC_CCIPR1_UART8SEL_FIELD(3U)
#define RCC_CCIPR1_UART8SEL_CSI             RCC_CCIPR1_UART8SEL_FIELD(4U)
#define RCC_CCIPR1_UART8SEL_LSE             RCC_CCIPR1_UART8SEL_FIELD(5U)

#define RCC_CCIPR1_UART9SEL_FIELD(n)        ((n) << RCC_CCIPR1_UART9SEL_Pos)
#define RCC_CCIPR1_UART9SEL_PCLK1           RCC_CCIPR1_UART9SEL_FIELD(0U)
#define RCC_CCIPR1_UART9SEL_PLL2Q           RCC_CCIPR1_UART9SEL_FIELD(1U)
#define RCC_CCIPR1_UART9SEL_PLL3Q           RCC_CCIPR1_UART9SEL_FIELD(2U)
#define RCC_CCIPR1_UART9SEL_HSI             RCC_CCIPR1_UART9SEL_FIELD(3U)
#define RCC_CCIPR1_UART9SEL_CSI             RCC_CCIPR1_UART9SEL_FIELD(4U)
#define RCC_CCIPR1_UART9SEL_LSE             RCC_CCIPR1_UART9SEL_FIELD(5U)

#define RCC_CCIPR1_USART10SEL_FIELD(n)      ((n) << RCC_CCIPR1_USART10SEL_Pos)
#define RCC_CCIPR1_USART10SEL_PCLK1         RCC_CCIPR1_USART10SEL_FIELD(0U)
#define RCC_CCIPR1_USART10SEL_PLL2Q         RCC_CCIPR1_USART10SEL_FIELD(1U)
#define RCC_CCIPR1_USART10SEL_PLL3Q         RCC_CCIPR1_USART10SEL_FIELD(2U)
#define RCC_CCIPR1_USART10SEL_HSI           RCC_CCIPR1_USART10SEL_FIELD(3U)
#define RCC_CCIPR1_USART10SEL_CSI           RCC_CCIPR1_USART10SEL_FIELD(4U)
#define RCC_CCIPR1_USART10SEL_LSE           RCC_CCIPR1_USART10SEL_FIELD(5U)
/** @} */

/**
 * @name    RCC_CCIPR2 register helpers
 * @{
 */
#define RCC_CCIPR2_USART11SEL_FIELD(n)      ((n) << RCC_CCIPR2_USART11SEL_Pos)
#define RCC_CCIPR2_USART11SEL_PCLK1         RCC_CCIPR2_USART11SEL_FIELD(0U)
#define RCC_CCIPR2_USART11SEL_PLL2Q         RCC_CCIPR2_USART11SEL_FIELD(1U)
#define RCC_CCIPR2_USART11SEL_PLL3Q         RCC_CCIPR2_USART11SEL_FIELD(2U)
#define RCC_CCIPR2_USART11SEL_HSI           RCC_CCIPR2_USART11SEL_FIELD(3U)
#define RCC_CCIPR2_USART11SEL_CSI           RCC_CCIPR2_USART11SEL_FIELD(4U)
#define RCC_CCIPR2_USART11SEL_LSE           RCC_CCIPR2_USART11SEL_FIELD(5U)

#define RCC_CCIPR2_UART12SEL_FIELD(n)       ((n) << RCC_CCIPR2_UART12SEL_Pos)
#define RCC_CCIPR2_UART12SEL_PCLK1          RCC_CCIPR2_UART12SEL_FIELD(0U)
#define RCC_CCIPR2_UART12SEL_PLL2Q          RCC_CCIPR2_UART12SEL_FIELD(1U)
#define RCC_CCIPR2_UART12SEL_PLL3Q          RCC_CCIPR2_UART12SEL_FIELD(2U)
#define RCC_CCIPR2_UART12SEL_HSI            RCC_CCIPR2_UART12SEL_FIELD(3U)
#define RCC_CCIPR2_UART12SEL_CSI            RCC_CCIPR2_UART12SEL_FIELD(4U)
#define RCC_CCIPR2_UART12SEL_LSE            RCC_CCIPR2_UART12SEL_FIELD(5U)

#define RCC_CCIPR2_LPTIM1SEL_FIELD(n)       ((n) << RCC_CCIPR2_LPTIM1SEL_Pos)
#define RCC_CCIPR2_LPTIM1SEL_PCLK3          RCC_CCIPR2_LPTIM1SEL_FIELD(0U)
#define RCC_CCIPR2_LPTIM1SEL_PLL2P          RCC_CCIPR2_LPTIM1SEL_FIELD(1U)
#define RCC_CCIPR2_LPTIM1SEL_PLL3R          RCC_CCIPR2_LPTIM1SEL_FIELD(2U)
#define RCC_CCIPR2_LPTIM1SEL_LSE            RCC_CCIPR2_LPTIM1SEL_FIELD(3U)
#define RCC_CCIPR2_LPTIM1SEL_LSI            RCC_CCIPR2_LPTIM1SEL_FIELD(4U)
#define RCC_CCIPR2_LPTIM1SEL_PER            RCC_CCIPR2_LPTIM1SEL_FIELD(5U)

#define RCC_CCIPR2_LPTIM2SEL_FIELD(n)       ((n) << RCC_CCIPR2_LPTIM2SEL_Pos)
#define RCC_CCIPR2_LPTIM2SEL_PCLK1          RCC_CCIPR2_LPTIM2SEL_FIELD(0U)
#define RCC_CCIPR2_LPTIM2SEL_PLL2P          RCC_CCIPR2_LPTIM2SEL_FIELD(1U)
#define RCC_CCIPR2_LPTIM2SEL_PLL3R          RCC_CCIPR2_LPTIM2SEL_FIELD(2U)
#define RCC_CCIPR2_LPTIM2SEL_LSE            RCC_CCIPR2_LPTIM2SEL_FIELD(3U)
#define RCC_CCIPR2_LPTIM2SEL_LSI            RCC_CCIPR2_LPTIM2SEL_FIELD(4U)
#define RCC_CCIPR2_LPTIM2SEL_PER            RCC_CCIPR2_LPTIM2SEL_FIELD(5U)

#define RCC_CCIPR2_LPTIM3SEL_FIELD(n)       ((n) << RCC_CCIPR2_LPTIM3SEL_Pos)
#define RCC_CCIPR2_LPTIM3SEL_PCLK3          RCC_CCIPR2_LPTIM3SEL_FIELD(0U)
#define RCC_CCIPR2_LPTIM3SEL_PLL2P          RCC_CCIPR2_LPTIM3SEL_FIELD(1U)
#define RCC_CCIPR2_LPTIM3SEL_PLL3R          RCC_CCIPR2_LPTIM3SEL_FIELD(2U)
#define RCC_CCIPR2_LPTIM3SEL_LSE            RCC_CCIPR2_LPTIM3SEL_FIELD(3U)
#define RCC_CCIPR2_LPTIM3SEL_LSI            RCC_CCIPR2_LPTIM3SEL_FIELD(4U)
#define RCC_CCIPR2_LPTIM3SEL_PER            RCC_CCIPR2_LPTIM3SEL_FIELD(5U)

#define RCC_CCIPR2_LPTIM4SEL_FIELD(n)       ((n) << RCC_CCIPR2_LPTIM4SEL_Pos)
#define RCC_CCIPR2_LPTIM4SEL_PCLK3          RCC_CCIPR2_LPTIM4SEL_FIELD(0U)
#define RCC_CCIPR2_LPTIM4SEL_PLL2P          RCC_CCIPR2_LPTIM4SEL_FIELD(1U)
#define RCC_CCIPR2_LPTIM4SEL_PLL3R          RCC_CCIPR2_LPTIM4SEL_FIELD(2U)
#define RCC_CCIPR2_LPTIM4SEL_LSE            RCC_CCIPR2_LPTIM4SEL_FIELD(3U)
#define RCC_CCIPR2_LPTIM4SEL_LSI            RCC_CCIPR2_LPTIM4SEL_FIELD(4U)
#define RCC_CCIPR2_LPTIM4SEL_PER            RCC_CCIPR2_LPTIM4SEL_FIELD(5U)

#define RCC_CCIPR2_LPTIM5SEL_FIELD(n)       ((n) << RCC_CCIPR2_LPTIM5SEL_Pos)
#define RCC_CCIPR2_LPTIM5SEL_PCLK3          RCC_CCIPR2_LPTIM5SEL_FIELD(0U)
#define RCC_CCIPR2_LPTIM5SEL_PLL2P          RCC_CCIPR2_LPTIM5SEL_FIELD(1U)
#define RCC_CCIPR2_LPTIM5SEL_PLL3R          RCC_CCIPR2_LPTIM5SEL_FIELD(2U)
#define RCC_CCIPR2_LPTIM5SEL_LSE            RCC_CCIPR2_LPTIM5SEL_FIELD(3U)
#define RCC_CCIPR2_LPTIM5SEL_LSI            RCC_CCIPR2_LPTIM5SEL_FIELD(4U)
#define RCC_CCIPR2_LPTIM5SEL_PER            RCC_CCIPR2_LPTIM5SEL_FIELD(5U)

#define RCC_CCIPR2_LPTIM6SEL_FIELD(n)       ((n) << RCC_CCIPR2_LPTIM6SEL_Pos)
#define RCC_CCIPR2_LPTIM6SEL_PCLK3          RCC_CCIPR2_LPTIM6SEL_FIELD(0U)
#define RCC_CCIPR2_LPTIM6SEL_PLL2P          RCC_CCIPR2_LPTIM6SEL_FIELD(1U)
#define RCC_CCIPR2_LPTIM6SEL_PLL3R          RCC_CCIPR2_LPTIM6SEL_FIELD(2U)
#define RCC_CCIPR2_LPTIM6SEL_LSE            RCC_CCIPR2_LPTIM6SEL_FIELD(3U)
#define RCC_CCIPR2_LPTIM6SEL_LSI            RCC_CCIPR2_LPTIM6SEL_FIELD(4U)
#define RCC_CCIPR2_LPTIM6SEL_PER            RCC_CCIPR2_LPTIM6SEL_FIELD(5U)
/** @} */

/**
 * @name    RCC_CCIPR3 register helpers
 * @{
 */
#define RCC_CCIPR3_SPI1SEL_FIELD(n)         ((n) << RCC_CCIPR3_SPI1SEL_Pos)
#define RCC_CCIPR3_SPI1SEL_PLL1Q            RCC_CCIPR3_SPI1SEL_FIELD(0U)
#define RCC_CCIPR3_SPI1SEL_PLL2P            RCC_CCIPR3_SPI1SEL_FIELD(1U)
#define RCC_CCIPR3_SPI1SEL_PLL3P            RCC_CCIPR3_SPI1SEL_FIELD(2U)
#define RCC_CCIPR3_SPI1SEL_AUDIOCLK         RCC_CCIPR3_SPI1SEL_FIELD(3U)
#define RCC_CCIPR3_SPI1SEL_PER              RCC_CCIPR3_SPI1SEL_FIELD(4U)

#define RCC_CCIPR3_SPI2SEL_FIELD(n)         ((n) << RCC_CCIPR3_SPI2SEL_Pos)
#define RCC_CCIPR3_SPI2SEL_PLL1Q            RCC_CCIPR3_SPI2SEL_FIELD(0U)
#define RCC_CCIPR3_SPI2SEL_PLL2P            RCC_CCIPR3_SPI2SEL_FIELD(1U)
#define RCC_CCIPR3_SPI2SEL_PLL3P            RCC_CCIPR3_SPI2SEL_FIELD(2U)
#define RCC_CCIPR3_SPI2SEL_AUDIOCLK         RCC_CCIPR3_SPI2SEL_FIELD(3U)
#define RCC_CCIPR3_SPI2SEL_PER              RCC_CCIPR3_SPI2SEL_FIELD(4U)

#define RCC_CCIPR3_SPI3SEL_FIELD(n)         ((n) << RCC_CCIPR3_SPI3SEL_Pos)
#define RCC_CCIPR3_SPI3SEL_PLL1Q            RCC_CCIPR3_SPI3SEL_FIELD(0U)
#define RCC_CCIPR3_SPI3SEL_PLL2P            RCC_CCIPR3_SPI3SEL_FIELD(1U)
#define RCC_CCIPR3_SPI3SEL_PLL3P            RCC_CCIPR3_SPI3SEL_FIELD(2U)
#define RCC_CCIPR3_SPI3SEL_AUDIOCLK         RCC_CCIPR3_SPI3SEL_FIELD(3U)
#define RCC_CCIPR3_SPI3SEL_PER              RCC_CCIPR3_SPI3SEL_FIELD(4U)

#define RCC_CCIPR3_SPI4SEL_FIELD(n)         ((n) << RCC_CCIPR3_SPI4SEL_Pos)
#define RCC_CCIPR3_SPI4SEL_PCLK2            RCC_CCIPR3_SPI4SEL_FIELD(0U)
#define RCC_CCIPR3_SPI4SEL_PLL2P            RCC_CCIPR3_SPI4SEL_FIELD(1U)
#define RCC_CCIPR3_SPI4SEL_PLL3P            RCC_CCIPR3_SPI4SEL_FIELD(2U)
#define RCC_CCIPR3_SPI4SEL_HSI              RCC_CCIPR3_SPI4SEL_FIELD(3U)
#define RCC_CCIPR3_SPI4SEL_CSI              RCC_CCIPR3_SPI4SEL_FIELD(4U)
#define RCC_CCIPR3_SPI4SEL_HSE              RCC_CCIPR3_SPI4SEL_FIELD(5U)

#define RCC_CCIPR3_SPI5SEL_FIELD(n)         ((n) << RCC_CCIPR3_SPI5SEL_Pos)
#define RCC_CCIPR3_SPI5SEL_PCLK3            RCC_CCIPR3_SPI5SEL_FIELD(0U)
#define RCC_CCIPR3_SPI5SEL_PLL2P            RCC_CCIPR3_SPI5SEL_FIELD(1U)
#define RCC_CCIPR3_SPI5SEL_PLL3P            RCC_CCIPR3_SPI5SEL_FIELD(2U)
#define RCC_CCIPR3_SPI5SEL_HSI              RCC_CCIPR3_SPI5SEL_FIELD(3U)
#define RCC_CCIPR3_SPI5SEL_CSI              RCC_CCIPR3_SPI5SEL_FIELD(4U)
#define RCC_CCIPR3_SPI5SEL_HSE              RCC_CCIPR3_SPI5SEL_FIELD(5U)

#define RCC_CCIPR3_SPI6SEL_FIELD(n)         ((n) << RCC_CCIPR3_SPI6SEL_Pos)
#define RCC_CCIPR3_SPI6SEL_PCLK2            RCC_CCIPR3_SPI6SEL_FIELD(0U)
#define RCC_CCIPR3_SPI6SEL_PLL2P            RCC_CCIPR3_SPI6SEL_FIELD(1U)
#define RCC_CCIPR3_SPI6SEL_PLL3P            RCC_CCIPR3_SPI6SEL_FIELD(2U)
#define RCC_CCIPR3_SPI6SEL_HSI              RCC_CCIPR3_SPI6SEL_FIELD(3U)
#define RCC_CCIPR3_SPI6SEL_CSI              RCC_CCIPR3_SPI6SEL_FIELD(4U)
#define RCC_CCIPR3_SPI6SEL_HSE              RCC_CCIPR3_SPI6SEL_FIELD(5U)

#define RCC_CCIPR3_LPUART1SEL_FIELD(n)      ((n) << RCC_CCIPR3_LPUART1SEL_Pos)
#define RCC_CCIPR3_LPUART1SEL_PCLK3         RCC_CCIPR3_LPUART1SEL_FIELD(0U)
#define RCC_CCIPR3_LPUART1SEL_PLL2Q         RCC_CCIPR3_LPUART1SEL_FIELD(1U)
#define RCC_CCIPR3_LPUART1SEL_PLL3Q         RCC_CCIPR3_LPUART1SEL_FIELD(2U)
#define RCC_CCIPR3_LPUART1SEL_HSI           RCC_CCIPR3_LPUART1SEL_FIELD(3U)
#define RCC_CCIPR3_LPUART1SEL_CSI           RCC_CCIPR3_LPUART1SEL_FIELD(4U)
#define RCC_CCIPR3_LPUART1SEL_LSE           RCC_CCIPR3_LPUART1SEL_FIELD(5U)
/** @} */

/**
 * @name    RCC_CCIPR4 register helpers
 * @{
 */
#define RCC_CCIPR4_OSPISEL_FIELD(n)         ((n) << RCC_CCIPR4_OSPISEL_Pos)
#define RCC_CCIPR4_OSPISEL_HCLK4            RCC_CCIPR4_OSPISEL_FIELD(0U)
#define RCC_CCIPR4_OSPISEL_PLL1Q            RCC_CCIPR4_OSPISEL_FIELD(1U)
#define RCC_CCIPR4_OSPISEL_PLL2R            RCC_CCIPR4_OSPISEL_FIELD(2U)
#define RCC_CCIPR4_OSPISEL_PER              RCC_CCIPR4_OSPISEL_FIELD(3U)

#define RCC_CCIPR4_SYSTICKSEL_FIELD(n)      ((n) << RCC_CCIPR4_SYSTICKSEL_Pos)
#define RCC_CCIPR4_SYSTICKSEL_HCLKDIV8      RCC_CCIPR4_SYSTICKSEL_FIELD(0U)
#define RCC_CCIPR4_SYSTICKSEL_LSI           RCC_CCIPR4_SYSTICKSEL_FIELD(1U)
#define RCC_CCIPR4_SYSTICKSEL_LSE           RCC_CCIPR4_SYSTICKSEL_FIELD(2U)
#define RCC_CCIPR4_SYSTICKSEL_NOCLOCK       RCC_CCIPR4_SYSTICKSEL_FIELD(3U)

#define RCC_CCIPR4_USBSEL_FIELD(n)          ((n) << RCC_CCIPR4_USBSEL_Pos)
#define RCC_CCIPR4_USBSEL_NOCLOCK           RCC_CCIPR4_USBSEL_FIELD(0U)
#define RCC_CCIPR4_USBSEL_PLL1Q             RCC_CCIPR4_USBSEL_FIELD(1U)
#define RCC_CCIPR4_USBSEL_PLL3Q             RCC_CCIPR4_USBSEL_FIELD(2U)
#define RCC_CCIPR4_USBSEL_HSI48             RCC_CCIPR4_USBSEL_FIELD(3U)

#define RCC_CCIPR4_SDMMC1SEL_FIELD(n)       ((n) << RCC_CCIPR4_SDMMC1SEL_Pos)
#define RCC_CCIPR4_SDMMC1SEL_PLL1Q          RCC_CCIPR4_SDMMC1SEL_FIELD(0U)
#define RCC_CCIPR4_SDMMC1SEL_PLL2R          RCC_CCIPR4_SDMMC1SEL_FIELD(1U)

#define RCC_CCIPR4_SDMMC2SEL_FIELD(n)       ((n) << RCC_CCIPR4_SDMMC2SEL_Pos)
#define RCC_CCIPR4_SDMMC2SEL_PLL1Q          RCC_CCIPR4_SDMMC2SEL_FIELD(0U)
#define RCC_CCIPR4_SDMMC2SEL_PLL2R          RCC_CCIPR4_SDMMC2SEL_FIELD(1U)

#define RCC_CCIPR4_I2C1SEL_FIELD(n)         ((n) << RCC_CCIPR4_I2C1SEL_Pos)
#define RCC_CCIPR4_I2C1SEL_PCLK1            RCC_CCIPR4_I2C1SEL_FIELD(0U)
#define RCC_CCIPR4_I2C1SEL_PLL3R            RCC_CCIPR4_I2C1SEL_FIELD(1U)
#define RCC_CCIPR4_I2C1SEL_HSI              RCC_CCIPR4_I2C1SEL_FIELD(2U)
#define RCC_CCIPR4_I2C1SEL_CSI              RCC_CCIPR4_I2C1SEL_FIELD(3U)

#define RCC_CCIPR4_I2C2SEL_FIELD(n)         ((n) << RCC_CCIPR4_I2C2SEL_Pos)
#define RCC_CCIPR4_I2C2SEL_PCLK1            RCC_CCIPR4_I2C2SEL_FIELD(0U)
#define RCC_CCIPR4_I2C2SEL_PLL3R            RCC_CCIPR4_I2C2SEL_FIELD(1U)
#define RCC_CCIPR4_I2C2SEL_HSI              RCC_CCIPR4_I2C2SEL_FIELD(2U)
#define RCC_CCIPR4_I2C2SEL_CSI              RCC_CCIPR4_I2C2SEL_FIELD(3U)

#define RCC_CCIPR4_I2C3SEL_FIELD(n)         ((n) << RCC_CCIPR4_I2C3SEL_Pos)
#define RCC_CCIPR4_I2C3SEL_PCLK3            RCC_CCIPR4_I2C3SEL_FIELD(0U)
#define RCC_CCIPR4_I2C3SEL_PLL3R            RCC_CCIPR4_I2C3SEL_FIELD(1U)
#define RCC_CCIPR4_I2C3SEL_HSI              RCC_CCIPR4_I2C3SEL_FIELD(2U)
#define RCC_CCIPR4_I2C3SEL_CSI              RCC_CCIPR4_I2C3SEL_FIELD(3U)

#define RCC_CCIPR4_I2C4SEL_FIELD(n)         ((n) << RCC_CCIPR4_I2C4SEL_Pos)
#define RCC_CCIPR4_I2C4SEL_PCLK3            RCC_CCIPR4_I2C4SEL_FIELD(0U)
#define RCC_CCIPR4_I2C4SEL_PLL3R            RCC_CCIPR4_I2C4SEL_FIELD(1U)
#define RCC_CCIPR4_I2C4SEL_HSI              RCC_CCIPR4_I2C4SEL_FIELD(2U)
#define RCC_CCIPR4_I2C4SEL_CSI              RCC_CCIPR4_I2C4SEL_FIELD(3U)

#define RCC_CCIPR4_I3C1SEL_FIELD(n)         ((n) << RCC_CCIPR4_I3C1SEL_Pos)
#define RCC_CCIPR4_I3C1SEL_PCLK1            RCC_CCIPR4_I3C1SEL_FIELD(0U)
#define RCC_CCIPR4_I3C1SEL_PLL3R            RCC_CCIPR4_I3C1SEL_FIELD(1U)
#define RCC_CCIPR4_I3C1SEL_HSI              RCC_CCIPR4_I3C1SEL_FIELD(2U)
/** @} */

/**
 * @name    RCC_CCIPR5 register helpers
 * @{
 */
#define RCC_CCIPR5_ADCDACSEL_FIELD(n)       ((n) << RCC_CCIPR5_ADCDACSEL_Pos)
#define RCC_CCIPR5_ADCDACSEL_HCLK           RCC_CCIPR5_ADCDACSEL_FIELD(0U)
#define RCC_CCIPR5_ADCDACSEL_SYSCLK         RCC_CCIPR5_ADCDACSEL_FIELD(1U)
#define RCC_CCIPR5_ADCDACSEL_PLL2R          RCC_CCIPR5_ADCDACSEL_FIELD(2U)
#define RCC_CCIPR5_ADCDACSEL_HSE            RCC_CCIPR5_ADCDACSEL_FIELD(3U)
#define RCC_CCIPR5_ADCDACSEL_HSI            RCC_CCIPR5_ADCDACSEL_FIELD(4U)
#define RCC_CCIPR5_ADCDACSEL_CSI            RCC_CCIPR5_ADCDACSEL_FIELD(5U)

#define RCC_CCIPR5_DACSEL_FIELD(n)          ((n) << RCC_CCIPR5_DACSEL_Pos)
#define RCC_CCIPR5_DACSEL_IGNORE            0xFFFFFFFFU
#define RCC_CCIPR5_DACSEL_LSE               RCC_CCIPR5_DACSEL_FIELD(0U)
#define RCC_CCIPR5_DACSEL_LSI               RCC_CCIPR5_DACSEL_FIELD(1U)

#define RCC_CCIPR5_RNGSEL_FIELD(n)          ((n) << RCC_CCIPR5_RNGSEL_Pos)
#define RCC_CCIPR5_RNGSEL_IGNORE            0xFFFFFFFFU
#define RCC_CCIPR5_RNGSEL_HSI48             RCC_CCIPR5_RNGSEL_FIELD(0U)
#define RCC_CCIPR5_RNGSEL_PLL1Q             RCC_CCIPR5_RNGSEL_FIELD(1U)
#define RCC_CCIPR5_RNGSEL_LSE               RCC_CCIPR5_RNGSEL_FIELD(2U)
#define RCC_CCIPR5_RNGSEL_LSI               RCC_CCIPR5_RNGSEL_FIELD(3U)

#define RCC_CCIPR5_CECSEL_FIELD(n)          ((n) << RCC_CCIPR5_CECSEL_Pos)
#define RCC_CCIPR5_CECSEL_IGNORE            0xFFFFFFFFU
#define RCC_CCIPR5_CECSEL_LSE               RCC_CCIPR5_CECSEL_FIELD(0U)
#define RCC_CCIPR5_CECSEL_LSI               RCC_CCIPR5_CECSEL_FIELD(1U)
#define RCC_CCIPR5_CECSEL_CSIDIV128         RCC_CCIPR5_CECSEL_FIELD(2U)

#define RCC_CCIPR5_FDCANSEL_FIELD(n)        ((n) << RCC_CCIPR5_FDCANSEL_Pos)
#define RCC_CCIPR5_FDCANSEL_IGNORE          0xFFFFFFFFU
#define RCC_CCIPR5_FDCANSEL_HSE             RCC_CCIPR5_FDCANSEL_FIELD(0U)
#define RCC_CCIPR5_FDCANSEL_PLL1Q           RCC_CCIPR5_FDCANSEL_FIELD(1U)
#define RCC_CCIPR5_FDCANSEL_PLL2Q           RCC_CCIPR5_FDCANSEL_FIELD(2U)

#define RCC_CCIPR5_SAI1SEL_FIELD(n)         ((n) << RCC_CCIPR5_SAI1SEL_Pos)
#define RCC_CCIPR5_SAI1SEL_PLL1Q            RCC_CCIPR5_SAI1SEL_FIELD(0U)
#define RCC_CCIPR5_SAI1SEL_PLL2P            RCC_CCIPR5_SAI1SEL_FIELD(1U)
#define RCC_CCIPR5_SAI1SEL_PLL3P            RCC_CCIPR5_SAI1SEL_FIELD(2U)
#define RCC_CCIPR5_SAI1SEL_AUDIOCLK         RCC_CCIPR5_SAI1SEL_FIELD(3U)
#define RCC_CCIPR5_SAI1SEL_PER              RCC_CCIPR5_SAI1SEL_FIELD(4U)

#define RCC_CCIPR5_SAI2SEL_FIELD(n)         ((n) << RCC_CCIPR5_SAI2SEL_Pos)
#define RCC_CCIPR5_SAI2SEL_PLL1Q            RCC_CCIPR5_SAI2SEL_FIELD(0U)
#define RCC_CCIPR5_SAI2SEL_PLL2P            RCC_CCIPR5_SAI2SEL_FIELD(1U)
#define RCC_CCIPR5_SAI2SEL_PLL3P            RCC_CCIPR5_SAI2SEL_FIELD(2U)
#define RCC_CCIPR5_SAI2SEL_AUDIOCLK         RCC_CCIPR5_SAI2SEL_FIELD(3U)
#define RCC_CCIPR5_SAI2SEL_PER              RCC_CCIPR5_SAI2SEL_FIELD(4U)

#define RCC_CCIPR5_CKPERSEL_FIELD(n)        ((n) << RCC_CCIPR5_CKPERSEL_Pos)
#define RCC_CCIPR5_CKPERSEL_HSI             RCC_CCIPR5_CKPERSEL_FIELD(0U)
#define RCC_CCIPR5_CKPERSEL_CSI             RCC_CCIPR5_CKPERSEL_FIELD(1U)
#define RCC_CCIPR5_CKPERSEL_HSE             RCC_CCIPR5_CKPERSEL_FIELD(2U)
/** @} */

/**
 * @name    RCC_BDCR register helpers
 * @{
 */
#define RCC_BDCR_RTCSEL_FIELD(n)            ((n) << RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_NOCLOCK             RCC_BDCR_RTCSEL_FIELD(0U)
#define RCC_BDCR_RTCSEL_LSE                 RCC_BDCR_RTCSEL_FIELD(1U)
#define RCC_BDCR_RTCSEL_LSI                 RCC_BDCR_RTCSEL_FIELD(2U)
#define RCC_BDCR_RTCSEL_HSEDIV              RCC_BDCR_RTCSEL_FIELD(3U)

#define RCC_BDCR_LSCOSEL_FIELD(n)           ((n) << RCC_BDCR_LSCOSEL_Pos)
#define RCC_BDCR_LSCOSEL_NOCLOCK            RCC_BDCR_LSCOSEL_FIELD(0U)
#define RCC_BDCR_LSCOSEL_LSI                RCC_BDCR_LSCOSEL_FIELD(1U)
#define RCC_BDCR_LSCOSEL_LSE                RCC_BDCR_LSCOSEL_FIELD(3U)
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
 * @brief   PWR VOSCR register initialization value.
 */
#if !defined(STM32_PWR_VOSCR) || defined(__DOXYGEN__)
#define STM32_PWR_VOSCR                     PWR_VOSCR_VOS_RANGE0
#endif

/**
 * @brief   PWR BDCR register initialization value.
 */
#if !defined(STM32_PWR_BDCR) || defined(__DOXYGEN__)
#define STM32_PWR_BDCR                      (0U)
#endif

/**
 * @brief   PWR UCPDR register initialization value.
 */
#if !defined(STM32_PWR_UCPDR) || defined(__DOXYGEN__)
#define STM32_PWR_UCPDR                     (0U)
#endif

/**
 * @brief   PWR SCCR register initialization value.
 */
#if !defined(STM32_PWR_SCCR) || defined(__DOXYGEN__)
#define STM32_PWR_SCCR                      (0U)
#endif

/**
 * @brief   PWR VMCR register initialization value.
 */
#if !defined(STM32_PWR_VMCR) || defined(__DOXYGEN__)
#define STM32_PWR_VMCR                      (0U)
#endif

/**
 * @brief   PWR USBSCR register initialization value.
 */
#if !defined(STM32_PWR_USBSCR) || defined(__DOXYGEN__)
#define STM32_PWR_USBSCR                    (0U)
#endif

/**
 * @brief   PWR WUCR register initialization value.
 */
#if !defined(STM32_PWR_WUCR) || defined(__DOXYGEN__)
#define STM32_PWR_WUCR                      (0U)
#endif

/**
 * @brief   PWR IORETR register initialization value.
 */
#if !defined(STM32_PWR_IORETR) || defined(__DOXYGEN__)
#define STM32_PWR_IORETR                    (0U)
#endif

/**
 * @brief   PWR SECCFGR register initialization value.
 */
#if !defined(STM32_PWR_SECCFGR) || defined(__DOXYGEN__)
#define STM32_PWR_SECCFGR                   (0U)
#endif

/**
 * @brief   PWR PRIVCFGR register initialization value.
 */
#if !defined(STM32_PWR_PRIVCFGR) || defined(__DOXYGEN__)
#define STM32_PWR_PRIVCFGR                  (0U)
#endif

/**
 * @brief   Enables or disables the HSI clock source.
 */
#if !defined(STM32_HSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI_ENABLED                   TRUE
#endif

/**
 * @brief   HSIDIV divider value.
 * @note    The allowed values are 1, 2, 4, 8.
 */
#if !defined(STM32_HSIDIV_VALUE) || defined(__DOXYGEN__)
#define STM32_HSIDIV_VALUE                  2
#endif

/**
 * @brief   Enables or disables the HSI48 clock source.
 */
#if !defined(STM32_HSI48_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI48_ENABLED                 FALSE
#endif

/**
 * @brief   Enables or disables the CSI clock source.
 */
#if !defined(STM32_CSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_CSI_ENABLED                   FALSE
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
 * @brief   Main clock source selection.
 * @note    If the selected clock source is not the PLL then the PLL is not
 *          initialized and started.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                            RCC_CFGR1_SW_PLL1P
#endif

/**
 * @brief   Clock source for PLL1.
 */
#if !defined(STM32_PLL1SRC) || defined(__DOXYGEN__)
#define STM32_PLL1SRC                       RCC_PLL1CFGR_PLL1SRC_HSI
#endif

/**
 * @brief   PLL1M divider value.
 * @note    The allowed values are 1..63.
 */
#if !defined(STM32_PLL1M_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1M_VALUE                   4
#endif

/**
 * @brief   PLL1N multiplier value.
 * @note    The allowed values are 4..512.
 */
#if !defined(STM32_PLL1N_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1N_VALUE                   250
#endif

/**
 * @brief   PLL1P divider value.
 * @note    The allowed values are 2..128 (odd values forbidden).
 */
#if !defined(STM32_PLL1P_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1P_VALUE                   2
#endif

/**
 * @brief   PLL1Q divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL1Q_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1Q_VALUE                   4
#endif

/**
 * @brief   PLL1R divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL1R_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1R_VALUE                   2
#endif

/**
 * @brief   Clock source for PLL2.
 */
#if !defined(STM32_PLL2SRC) || defined(__DOXYGEN__)
#define STM32_PLL2SRC                       RCC_PLL2CFGR_PLL2SRC_HSI
#endif

/**
 * @brief   PLL2M divider value.
 * @note    The allowed values are 1..63.
 */
#if !defined(STM32_PLL2M_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2M_VALUE                   4
#endif

/**
 * @brief   PLL2N multiplier value.
 * @note    The allowed values are 4..512.
 */
#if !defined(STM32_PLL2N_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2N_VALUE                   200
#endif

/**
 * @brief   PLL2P divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL2P_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2P_VALUE                   2
#endif

/**
 * @brief   PLL2Q divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL2Q_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2Q_VALUE                   2
#endif

/**
 * @brief   PLL2R divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL2R_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2R_VALUE                   2
#endif

/**
 * @brief   Clock source for PLL3.
 */
#if !defined(STM32_PLL3SRC) || defined(__DOXYGEN__)
#define STM32_PLL3SRC                       RCC_PLL3CFGR_PLL3SRC_HSI
#endif

/**
 * @brief   PLL3M divider value.
 * @note    The allowed values are 1..63.
 */
#if !defined(STM32_PLL3M_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3M_VALUE                   4
#endif

/**
 * @brief   PLL3N multiplier value.
 * @note    The allowed values are 4..512.
 */
#if !defined(STM32_PLL3N_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3N_VALUE                   240
#endif

/**
 * @brief   PLL3P divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL3P_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3P_VALUE                   2
#endif

/**
 * @brief   PLL3Q divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL3Q_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3Q_VALUE                   10
#endif

/**
 * @brief   PLL3R divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL3R_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3R_VALUE                   2
#endif

/**
 * @brief   AHB prescaler value.
 */
#if !defined(STM32_HPRE) || defined(__DOXYGEN__)
#define STM32_HPRE                          RCC_CFGR2_HPRE_DIV1
#endif

/**
 * @brief   APB1 prescaler value.
 */
#if !defined(STM32_PPRE1) || defined(__DOXYGEN__)
#define STM32_PPRE1                         RCC_CFGR2_PPRE1_DIV1
#endif

/**
 * @brief   APB2 prescaler value.
 */
#if !defined(STM32_PPRE2) || defined(__DOXYGEN__)
#define STM32_PPRE2                         RCC_CFGR2_PPRE2_DIV1
#endif

/**
 * @brief   APB3 prescaler value.
 */
#if !defined(STM32_PPRE3) || defined(__DOXYGEN__)
#define STM32_PPRE3                         RCC_CFGR2_PPRE3_DIV1
#endif

/**
 * @brief   System clock source after STOP.
 */
#if !defined(STM32_STOPWUCK) || defined(__DOXYGEN__)
#define STM32_STOPWUCK                      RCC_CFGR1_STOPWUCK_HSI
#endif

/**
 * @brief   Kernel clock source after STOP.
 */
#if !defined(STM32_STOPKERWUCK) || defined(__DOXYGEN__)
#define STM32_STOPKERWUCK                   RCC_CFGR1_STOPKERWUCK_HSI
#endif

/**
 * @brief   RTC prescaler value.
 */
#if !defined(STM32_RTCPRE_VALUE) || defined(__DOXYGEN__)
#define STM32_RTCPRE_VALUE                  RCC_CFGR1_RTCPRE_NOCLOCK
#endif

/**
 * @brief   TIMPRE timers clocks prescaler selection.
 */
#if !defined(STM32_TIMPRE) || defined(__DOXYGEN__)
#define STM32_TIMPRE                        RCC_CFGR1_TIMPRE_LOW
#endif

/**
 * @brief   MCO1 clock source.
 */
#if !defined(STM32_MCO1SEL) || defined(__DOXYGEN__)
#define STM32_MCO1SEL                       RCC_CFGR1_MCO1SEL_HSI
#endif

/**
 * @brief   MCO1 divider setting.
 */
#if !defined(STM32_MCO1PRE_VALUE) || defined(__DOXYGEN__)
#define STM32_MCO1PRE_VALUE                 RCC_CFGR1_MCO1PRE_NOCLOCK
#endif

/**
 * @brief   MCO2 clock source.
 */
#if !defined(STM32_MCO2SEL) || defined(__DOXYGEN__)
#define STM32_MCO2SEL                       RCC_CFGR1_MCO2SEL_SYSCLK
#endif

/**
 * @brief   MCO1 divider setting.
 */
#if !defined(STM32_MCO2PRE_VALUE) || defined(__DOXYGEN__)
#define STM32_MCO2PRE_VALUE                 RCC_CFGR1_MCO2PRE_NOCLOCK
#endif

/**
 * @brief   LSCO clock source.
 */
#if !defined(STM32_LSCOSEL) || defined(__DOXYGEN__)
#define STM32_LSCOSEL                       RCC_BDCR_LSCOSEL_NOCLOCK
#endif

/**
 * @brief   TIMICSEL clock source.
 */
#if !defined(STM32_TIMICSEL) || defined(__DOXYGEN__)
#define STM32_TIMICSEL                      RCC_CCIPR1_TIMICSEL_NOCLK
#endif

/**
 * @brief   USART1 clock source.
 */
#if !defined(STM32_USART1SEL) || defined(__DOXYGEN__)
#define STM32_USART1SEL                     RCC_CCIPR1_USART1SEL_PCLK2
#endif

/**
 * @brief   USART2 clock source.
 */
#if !defined(STM32_USART2SEL) || defined(__DOXYGEN__)
#define STM32_USART2SEL                     RCC_CCIPR1_USART2SEL_PCLK1
#endif

/**
 * @brief   USART3 clock source.
 */
#if !defined(STM32_USART3SEL) || defined(__DOXYGEN__)
#define STM32_USART3SEL                     RCC_CCIPR1_USART3SEL_PCLK1
#endif

/**
 * @brief   UART4 clock source.
 */
#if !defined(STM32_UART4SEL) || defined(__DOXYGEN__)
#define STM32_UART4SEL                      RCC_CCIPR1_UART4SEL_PCLK1
#endif

/**
 * @brief   UART5 clock source.
 */
#if !defined(STM32_UART5SEL) || defined(__DOXYGEN__)
#define STM32_UART5SEL                      RCC_CCIPR1_UART5SEL_PCLK1
#endif

/**
 * @brief   USART6 clock source.
 */
#if !defined(STM32_USART6SEL) || defined(__DOXYGEN__)
#define STM32_USART6SEL                     RCC_CCIPR1_USART6SEL_PCLK1
#endif

/**
 * @brief   UART7 clock source.
 */
#if !defined(STM32_UART7SEL) || defined(__DOXYGEN__)
#define STM32_UART7SEL                      RCC_CCIPR1_UART7SEL_PCLK1
#endif

/**
 * @brief   UART8 clock source.
 */
#if !defined(STM32_UART8SEL) || defined(__DOXYGEN__)
#define STM32_UART8SEL                      RCC_CCIPR1_UART8SEL_PCLK1
#endif

/**
 * @brief   UART9 clock source.
 */
#if !defined(STM32_UART9SEL) || defined(__DOXYGEN__)
#define STM32_UART9SEL                      RCC_CCIPR1_UART9SEL_PCLK1
#endif

/**
 * @brief   USART10 clock source.
 */
#if !defined(STM32_USART10SEL) || defined(__DOXYGEN__)
#define STM32_USART10SEL                    RCC_CCIPR1_USART10SEL_PCLK1
#endif

/**
 * @brief   USART11 clock source.
 */
#if !defined(STM32_USART11SEL) || defined(__DOXYGEN__)
#define STM32_USART11SEL                    RCC_CCIPR2_USART11SEL_PCLK1
#endif

/**
 * @brief   UART12 clock source.
 */
#if !defined(STM32_UART12SEL) || defined(__DOXYGEN__)
#define STM32_UART12SEL                     RCC_CCIPR2_UART12SEL_PCLK1
#endif

/**
 * @brief   LPUART1 clock source.
 */
#if !defined(STM32_LPUART1SEL) || defined(__DOXYGEN__)
#define STM32_LPUART1SEL                    RCC_CCIPR3_LPUART1SEL_PCLK3
#endif

/**
 * @brief   LPTIM1 clock source.
 */
#if !defined(STM32_LPTIM1SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM1SEL                     RCC_CCIPR2_LPTIM1SEL_PCLK3
#endif

/**
 * @brief   LPTIM2 clock source.
 */
#if !defined(STM32_LPTIM2SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM2SEL                     RCC_CCIPR2_LPTIM2SEL_PCLK1
#endif

/**
 * @brief   LPTIM3 clock source.
 */
#if !defined(STM32_LPTIM3SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM3SEL                     RCC_CCIPR2_LPTIM3SEL_PCLK3
#endif

/**
 * @brief   LPTIM4 clock source.
 */
#if !defined(STM32_LPTIM4SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM4SEL                     RCC_CCIPR2_LPTIM4SEL_PCLK3
#endif

/**
 * @brief   LPTIM5 clock source.
 */
#if !defined(STM32_LPTIM5SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM5SEL                     RCC_CCIPR2_LPTIM5SEL_PCLK3
#endif

/**
 * @brief   LPTIM6 clock source.
 */
#if !defined(STM32_LPTIM6SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM6SEL                     RCC_CCIPR2_LPTIM6SEL_PCLK3
#endif

/**
 * @brief   SPI1 clock source.
 */
#if !defined(STM32_SPI1SEL) || defined(__DOXYGEN__)
#define STM32_SPI1SEL                       RCC_CCIPR3_SPI1SEL_PLL1Q
#endif

/**
 * @brief   SPI2 clock source.
 */
#if !defined(STM32_SPI2SEL) || defined(__DOXYGEN__)
#define STM32_SPI2SEL                       RCC_CCIPR3_SPI2SEL_PLL1Q
#endif

/**
 * @brief   SPI3 clock source.
 */
#if !defined(STM32_SPI3SEL) || defined(__DOXYGEN__)
#define STM32_SPI3SEL                       RCC_CCIPR3_SPI3SEL_PLL1Q
#endif

/**
 * @brief   SPI4 clock source.
 */
#if !defined(STM32_SPI4SEL) || defined(__DOXYGEN__)
#define STM32_SPI4SEL                       RCC_CCIPR3_SPI4SEL_PCLK2
#endif

/**
 * @brief   SPI5 clock source.
 */
#if !defined(STM32_SPI5SEL) || defined(__DOXYGEN__)
#define STM32_SPI5SEL                       RCC_CCIPR3_SPI5SEL_PCLK3
#endif

/**
 * @brief   SPI6 clock source.
 */
#if !defined(STM32_SPI6SEL) || defined(__DOXYGEN__)
#define STM32_SPI6SEL                       RCC_CCIPR3_SPI6SEL_PCLK2
#endif

/**
 * @brief   QSPI clock source.
 */
#if !defined(STM32_OSPISEL) || defined(__DOXYGEN__)
#define STM32_OSPISEL                       RCC_CCIPR4_OSPISEL_HCLK4
#endif

/**
 * @brief   SYSTICK clock source.
 */
#if !defined(STM32_SYSTICKSEL) || defined(__DOXYGEN__)
#define STM32_SYSTICKSEL                    RCC_CCIPR4_SYSTICKSEL_HCLKDIV8
#endif

/**
 * @brief   USB clock source.
 */
#if !defined(STM32_USBSEL) || defined(__DOXYGEN__)
#define STM32_USBSEL                        RCC_CCIPR4_USBSEL_NOCLOCK
#endif

/**
 * @brief   SDMMC1 clock source.
 */
#if !defined(STM32_SDMMC1SEL) || defined(__DOXYGEN__)
#define STM32_SDMMC1SEL                     RCC_CCIPR4_SDMMC1SEL_PLL1Q
#endif

/**
 * @brief   SDMMC2 clock source.
 */
#if !defined(STM32_SDMMC2SEL) || defined(__DOXYGEN__)
#define STM32_SDMMC2SEL                     RCC_CCIPR4_SDMMC2SEL_PLL1Q
#endif

/**
 * @brief   I2C1 clock source.
 */
#if !defined(STM32_I2C1SEL) || defined(__DOXYGEN__)
#define STM32_I2C1SEL                       RCC_CCIPR4_I2C1SEL_PCLK1
#endif

/**
 * @brief   I2C2 clock source.
 */
#if !defined(STM32_I2C2SEL) || defined(__DOXYGEN__)
#define STM32_I2C2SEL                       RCC_CCIPR4_I2C2SEL_PCLK1
#endif

/**
 * @brief   I2C3 clock source.
 */
#if !defined(STM32_I2C3SEL) || defined(__DOXYGEN__)
#define STM32_I2C3SEL                       RCC_CCIPR4_I2C3SEL_PCLK3
#endif

/**
 * @brief   I2C4 clock source.
 */
#if !defined(STM32_I2C4SEL) || defined(__DOXYGEN__)
#define STM32_I2C4SEL                       RCC_CCIPR4_I2C4SEL_PCLK3
#endif

/**
 * @brief   I3C1 clock source.
 */
#if !defined(STM32_I3C1SEL) || defined(__DOXYGEN__)
#define STM32_I3C1SEL                       RCC_CCIPR4_I3C1SEL_PCLK1
#endif

/**
 * @brief   ADCDACSEL clock source.
 */
#if !defined(STM32_ADCDACSEL) || defined(__DOXYGEN__)
#define STM32_ADCDACSEL                     RCC_CCIPR5_ADCDACSEL_HCLK
#endif

/**
 * @brief   DACSEL clock source.
 */
#if !defined(STM32_DACSEL) || defined(__DOXYGEN__)
#define STM32_DACSEL                        RCC_CCIPR5_DACSEL_LSE
#endif

/**
 * @brief   RNG clock source.
 */
#if !defined(STM32_RNGSEL) || defined(__DOXYGEN__)
#define STM32_RNGSEL                        RCC_CCIPR5_RNGSEL_HSI48
#endif

/**
 * @brief   CEC clock source.
 */
#if !defined(STM32_CECSEL) || defined(__DOXYGEN__)
#define STM32_CECSEL                        RCC_CCIPR5_CECSEL_LSE
#endif

/**
 * @brief   FDCAN clock source.
 */
#if !defined(STM32_FDCANSEL) || defined(__DOXYGEN__)
#define STM32_FDCANSEL                      RCC_CCIPR5_FDCANSEL_HSE
#endif

/**
 * @brief   SAI1 clock source.
 */
#if !defined(STM32_SAI1SEL) || defined(__DOXYGEN__)
#define STM32_SAI1SEL                       RCC_CCIPR5_SAI1SEL_PLL1Q
#endif

/**
 * @brief   SAI2 clock source.
 */
#if !defined(STM32_SAI2SEL) || defined(__DOXYGEN__)
#define STM32_SAI2SEL                       RCC_CCIPR5_SAI2SEL_PLL1Q
#endif

/**
 * @brief   CKPERSEL clock source.
 */
#if !defined(STM32_CKPERSEL) || defined(__DOXYGEN__)
#define STM32_CKPERSEL                      RCC_CCIPR5_CKPERSEL_HSI
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
#if !defined(STM32H5xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32H5xx_MCUCONF not defined"
#endif

#if defined(STM32H503xx) && !defined(STM32H503_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32H503_MCUCONF not defined"

#elif defined(STM32H562xx) && !defined(STM32H562_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32H562_MCUCONF not defined"

#elif defined(STM32H563xx) && !defined(STM32H563_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32H563_MCUCONF not defined"

#elif defined(STM32H573xx) && !defined(STM32H573_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32H573_MCUCONF not defined"

#endif

/* Device limits.*/
#include "stm32_limits.h"

/* ICache handler.*/
#include "stm32_icache.inc"

/* Clock handlers.*/
#include "stm32_lsi.inc"
#include "stm32_csi.inc"
#include "stm32_hsi48.inc"
#include "stm32_hsidiv.inc"
#include "stm32_lse.inc"
#include "stm32_hse.inc"

/*
 * PLL1 enable check.
 */
#if (STM32_SW           == RCC_CFGR1_SW_PLL1P)          ||                  \
    (STM32_MCO1SEL      == RCC_CFGR1_MCO1SEL_PLL1P)     ||                  \
    (STM32_MCO2SEL      == RCC_CFGR1_MCO2SEL_PLL1P)     ||                  \
    (STM32_SPI1SEL      == RCC_CCIPR3_SPI1SEL_PLL1Q)     ||                 \
    (STM32_SPI2SEL      == RCC_CCIPR3_SPI2SEL_PLL1Q)     ||                 \
    (STM32_SPI3SEL      == RCC_CCIPR3_SPI3SEL_PLL1Q)     ||                 \
    (STM32_OSPISEL      == RCC_CCIPR4_OSPISEL_PLL1Q)     ||                 \
    (STM32_USBSEL       == RCC_CCIPR4_USBSEL_PLL1Q)      ||                 \
    (STM32_SDMMC1SEL    == RCC_CCIPR4_SDMMC1SEL_PLL1Q)   ||                 \
    (STM32_SDMMC2SEL    == RCC_CCIPR4_SDMMC2SEL_PLL1Q)   ||                 \
    (STM32_RNGSEL       == RCC_CCIPR5_RNGSEL_PLL1Q)      ||                 \
    (STM32_FDCANSEL     == RCC_CCIPR5_FDCANSEL_PLL1Q)    ||                 \
    (STM32_SAI1SEL      == RCC_CCIPR5_SAI1SEL_PLL1Q)     ||                 \
    (STM32_SAI2SEL      == RCC_CCIPR5_SAI2SEL_PLL1Q)     ||                 \
    defined(__DOXYGEN__)
  /**
   * @brief   PLL1 activation flag.
   */
  #define STM32_ACTIVATE_PLL1       TRUE
#else

  #define STM32_ACTIVATE_PLL1       FALSE
#endif

/*
 * PLL2 enable check.
 */
#if (STM32_MCO2SEL      == RCC_CFGR1_MCO2SEL_PLL2P)      ||                 \
    (STM32_USART1SEL    == RCC_CCIPR1_USART1SEL_PLL2Q)   ||                 \
    (STM32_USART2SEL    == RCC_CCIPR1_USART2SEL_PLL2Q)   ||                 \
    (STM32_USART3SEL    == RCC_CCIPR1_USART3SEL_PLL2Q)   ||                 \
    (STM32_UART4SEL     == RCC_CCIPR1_UART4SEL_PLL2Q)    ||                 \
    (STM32_UART5SEL     == RCC_CCIPR1_UART5SEL_PLL2Q)    ||                 \
    (STM32_USART6SEL    == RCC_CCIPR1_USART6SEL_PLL2Q)   ||                 \
    (STM32_UART7SEL     == RCC_CCIPR1_UART7SEL_PLL2Q)    ||                 \
    (STM32_UART8SEL     == RCC_CCIPR1_UART8SEL_PLL2Q)    ||                 \
    (STM32_UART9SEL     == RCC_CCIPR1_UART9SEL_PLL2Q)    ||                 \
    (STM32_USART10SEL   == RCC_CCIPR1_USART10SEL_PLL2Q)  ||                 \
    (STM32_USART11SEL   == RCC_CCIPR2_USART11SEL_PLL2Q)  ||                 \
    (STM32_UART12SEL    == RCC_CCIPR2_UART12SEL_PLL2Q)   ||                 \
    (STM32_LPUART1SEL   == RCC_CCIPR3_LPUART1SEL_PLL2Q)  ||                 \
    (STM32_LPTIM1SEL    == RCC_CCIPR2_LPTIM1SEL_PLL2P)   ||                 \
    (STM32_LPTIM2SEL    == RCC_CCIPR2_LPTIM2SEL_PLL2P)   ||                 \
    (STM32_LPTIM3SEL    == RCC_CCIPR2_LPTIM3SEL_PLL2P)   ||                 \
    (STM32_LPTIM4SEL    == RCC_CCIPR2_LPTIM4SEL_PLL2P)   ||                 \
    (STM32_LPTIM5SEL    == RCC_CCIPR2_LPTIM5SEL_PLL2P)   ||                 \
    (STM32_LPTIM6SEL    == RCC_CCIPR2_LPTIM6SEL_PLL2P)   ||                 \
    (STM32_SPI1SEL      == RCC_CCIPR3_SPI1SEL_PLL2P)     ||                 \
    (STM32_SPI2SEL      == RCC_CCIPR3_SPI2SEL_PLL2P)     ||                 \
    (STM32_SPI3SEL      == RCC_CCIPR3_SPI3SEL_PLL2P)     ||                 \
    (STM32_SPI4SEL      == RCC_CCIPR3_SPI4SEL_PLL2P)     ||                 \
    (STM32_SPI5SEL      == RCC_CCIPR3_SPI5SEL_PLL2P)     ||                 \
    (STM32_SPI6SEL      == RCC_CCIPR3_SPI6SEL_PLL2P)     ||                 \
    (STM32_OSPISEL      == RCC_CCIPR4_OSPISEL_PLL2R )    ||                 \
    (STM32_SDMMC1SEL    == RCC_CCIPR4_SDMMC1SEL_PLL2R)   ||                 \
    (STM32_SDMMC2SEL    == RCC_CCIPR4_SDMMC2SEL_PLL2R)   ||                 \
    (STM32_ADCDACSEL    == RCC_CCIPR5_ADCDACSEL_PLL2R)   ||                 \
    (STM32_FDCANSEL     == RCC_CCIPR5_FDCANSEL_PLL2Q)    ||                 \
    (STM32_SAI1SEL      == RCC_CCIPR5_SAI1SEL_PLL2P)     ||                 \
    (STM32_SAI2SEL      == RCC_CCIPR5_SAI2SEL_PLL2P)     ||                 \
    defined(__DOXYGEN__)
  /**
   * @brief   PLL2 activation flag.
   */
  #define STM32_ACTIVATE_PLL2       TRUE
#else

  #define STM32_ACTIVATE_PLL2       FALSE
#endif

/*
 * PLL3 enable check.
 */
#if (STM32_USART1SEL    == RCC_CCIPR1_USART1SEL_PLL3Q)   ||                 \
    (STM32_USART2SEL    == RCC_CCIPR1_USART2SEL_PLL3Q)   ||                 \
    (STM32_USART3SEL    == RCC_CCIPR1_USART3SEL_PLL3Q)   ||                 \
    (STM32_UART4SEL     == RCC_CCIPR1_UART4SEL_PLL3Q)    ||                 \
    (STM32_UART5SEL     == RCC_CCIPR1_UART5SEL_PLL3Q)    ||                 \
    (STM32_USART6SEL    == RCC_CCIPR1_USART6SEL_PLL3Q)   ||                 \
    (STM32_UART7SEL     == RCC_CCIPR1_UART7SEL_PLL3Q)    ||                 \
    (STM32_UART8SEL     == RCC_CCIPR1_UART8SEL_PLL3Q)    ||                 \
    (STM32_UART9SEL     == RCC_CCIPR1_UART9SEL_PLL3Q)    ||                 \
    (STM32_USART10SEL   == RCC_CCIPR1_USART10SEL_PLL3Q)  ||                 \
    (STM32_USART11SEL   == RCC_CCIPR2_USART11SEL_PLL3Q)  ||                 \
    (STM32_UART12SEL    == RCC_CCIPR2_UART12SEL_PLL3Q)   ||                 \
    (STM32_LPUART1SEL   == RCC_CCIPR3_LPUART1SEL_PLL3Q)  ||                 \
    (STM32_LPTIM1SEL    == RCC_CCIPR2_LPTIM1SEL_PLL3R)   ||                 \
    (STM32_LPTIM2SEL    == RCC_CCIPR2_LPTIM2SEL_PLL3R)   ||                 \
    (STM32_LPTIM3SEL    == RCC_CCIPR2_LPTIM3SEL_PLL3R)   ||                 \
    (STM32_LPTIM4SEL    == RCC_CCIPR2_LPTIM4SEL_PLL3R)   ||                 \
    (STM32_LPTIM5SEL    == RCC_CCIPR2_LPTIM5SEL_PLL3R)   ||                 \
    (STM32_LPTIM6SEL    == RCC_CCIPR2_LPTIM6SEL_PLL3R)   ||                 \
    (STM32_SPI1SEL      == RCC_CCIPR3_SPI1SEL_PLL3P)     ||                 \
    (STM32_SPI2SEL      == RCC_CCIPR3_SPI2SEL_PLL3P)     ||                 \
    (STM32_SPI3SEL      == RCC_CCIPR3_SPI3SEL_PLL3P)     ||                 \
    (STM32_SPI4SEL      == RCC_CCIPR3_SPI4SEL_PLL3P)     ||                 \
    (STM32_SPI5SEL      == RCC_CCIPR3_SPI5SEL_PLL3P)     ||                 \
    (STM32_SPI6SEL      == RCC_CCIPR3_SPI6SEL_PLL3P)     ||                 \
    (STM32_USBSEL       == RCC_CCIPR4_USBSEL_PLL3Q)      ||                 \
    (STM32_I2C1SEL      == RCC_CCIPR4_I2C1SEL_PLL3R)     ||                 \
    (STM32_I2C2SEL      == RCC_CCIPR4_I2C2SEL_PLL3R)     ||                 \
    (STM32_I2C3SEL      == RCC_CCIPR4_I2C3SEL_PLL3R)     ||                 \
    (STM32_I2C4SEL      == RCC_CCIPR4_I2C4SEL_PLL3R)     ||                 \
    (STM32_I3C1SEL      == RCC_CCIPR4_I3C1SEL_PLL3R)     ||                 \
    (STM32_SAI1SEL      == RCC_CCIPR5_SAI1SEL_PLL3P)     ||                 \
    (STM32_SAI2SEL      == RCC_CCIPR5_SAI2SEL_PLL3P)     ||                 \
    defined(__DOXYGEN__)
  /**
   * @brief   PLL3 activation flag.
   */
  #define STM32_ACTIVATE_PLL3       TRUE
  #if STM32_RCC_HAS_PLL3 == FALSE
    #error "PLL3 not present on this device"
  #endif

#else

  #define STM32_ACTIVATE_PLL3       FALSE
#endif

/*
 * LSI related checks.
 */
#if STM32_LSI_ENABLED
#else /* !STM32_LSI_ENABLED */

  #if STM32_MCO2SEL == RCC_CFGR1_MCO2SEL_LSI
    #error "LSI not enabled, required by STM32_MCO2SEL"
  #endif

  #if (STM32_LPTIM1SEL == RCC_CCIPR2_LPTIM1SEL_LSI)
    #error "LSI not enabled, required by STM32_LPTIM1SEL"
  #endif
  #if (STM32_LPTIM2SEL == RCC_CCIPR2_LPTIM2SEL_LSI)
    #error "LSI not enabled, required by STM32_LPTIM2SEL"
  #endif
  #if (STM32_LPTIM3SEL == RCC_CCIPR2_LPTIM3SEL_LSI)
    #error "LSI not enabled, required by STM32_LPTIM3SEL"
  #endif
  #if (STM32_LPTIM4SEL == RCC_CCIPR2_LPTIM4SEL_LSI)
    #error "LSI not enabled, required by STM32_LPTIM4SEL"
  #endif
  #if (STM32_LPTIM5SEL == RCC_CCIPR2_LPTIM5SEL_LSI)
    #error "LSI not enabled, required by STM32_LPTIM5SEL"
  #endif
  #if (STM32_LPTIM6SEL == RCC_CCIPR2_LPTIM6SEL_LSI)
    #error "LSI not enabled, required by STM32_LPTIM6SEL"
  #endif

  #if STM32_SYSTICKSEL == RCC_CCIPR4_SYSTICKSEL_LSI
    #error "LSI not enabled, required by STM32_SYSTICKSEL"
  #endif

  #if STM32_RNGSEL == RCC_CCIPR5_RNGSEL_LSI
    #error "LSI not enabled, required by STM32_RNGSEL"
  #endif

  #if STM32_DACSEL == RCC_CCIPR5_DACSEL_LSI
    #error "LSI not enabled, required by STM32_DACSEL"
  #endif

  #if STM32_CECSEL == RCC_CCIPR5_CECSEL_LSI
    #error "LSI not enabled, required by STM32_CECSEL"
  #endif

  #if HAL_USE_RTC && (STM32_RTCSEL == RCC_BDCR_RTCSEL_LSI)
    #error "LSI not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_LSCOSEL == RCC_BDCR_LSCOSEL_LSI
    #error "LSI not enabled, required by STM32_LSCOSEL"
  #endif

#endif /* !STM32_LSI_ENABLED */

/*
 * CSI related checks.
 */
#if STM32_CSI_ENABLED
#else /* !STM32_CSI_ENABLED */

  #if STM32_ACTIVATE_PLL1 && (STM32_PLL1SRC == RCC_PLL1CFGR_PLL1SRC_CSI)
    #error "CSI not enabled, required by STM32_PLL1SRC"
  #endif
  #if STM32_ACTIVATE_PLL2 && (STM32_PLL2SRC == RCC_PLL2CFGR_PLL2SRC_CSI)
    #error "CSI not enabled, required by STM32_PLL2SRC"
  #endif
  #if STM32_ACTIVATE_PLL3 && (STM32_PLL3SRC == RCC_PLL3CFGR_PLL3SRC_CSI)
    #error "CSI not enabled, required by STM32_PLL3SRC"
  #endif

  #if STM32_SW == RCC_CFGR1_SW_CSI
    #error "CSI not enabled, required by STM32_SW"
  #endif

  #if STM32_MCO2SEL == RCC_CFGR1_MCO2SEL_CSI
    #error "CSI not enabled, required by STM32_MCO2SEL"
  #endif

  #if (STM32_USART1SEL == RCC_CCIPR1_USART1SEL_CSI)
    #error "CSI not enabled, required by STM32_USART1SEL"
  #endif
  #if (STM32_USART2SEL == RCC_CCIPR1_USART2SEL_CSI)
    #error "CSI not enabled, required by STM32_USART2SEL"
  #endif
  #if (STM32_USART3SEL == RCC_CCIPR1_USART3SEL_CSI)
    #error "CSI not enabled, required by STM32_USART3SEL"
  #endif
  #if (STM32_UART4SEL == RCC_CCIPR1_UART4SEL_CSI)
    #error "CSI not enabled, required by STM32_UART4SEL"
  #endif
  #if (STM32_UART5SEL == RCC_CCIPR1_UART5SEL_CSI)
    #error "CSI not enabled, required by STM32_UART5SEL"
  #endif
  #if (STM32_USART6SEL == RCC_CCIPR1_USART6SEL_CSI)
    #error "CSI not enabled, required by STM32_USART6SEL"
  #endif
  #if (STM32_UART7SEL == RCC_CCIPR1_UART7SEL_CSI)
    #error "CSI not enabled, required by STM32_UART7SEL"
  #endif
  #if (STM32_UART8SEL == RCC_CCIPR1_UART8SEL_CSI)
    #error "CSI not enabled, required by STM32_UART8SEL"
  #endif
  #if (STM32_UART9SEL == RCC_CCIPR1_UART9SEL_CSI)
    #error "CSI not enabled, required by STM32_UART9SEL"
  #endif
  #if (STM32_USART10SEL == RCC_CCIPR1_USART10SEL_CSI)
    #error "CSI not enabled, required by STM32_USART10SEL"
  #endif
  #if (STM32_USART11SEL == RCC_CCIPR2_USART11SEL_CSI)
    #error "CSI not enabled, required by STM32_USART11SEL"
  #endif
  #if (STM32_UART12SEL == RCC_CCIPR2_UART12SEL_CSI)
    #error "CSI not enabled, required by STM32_UART12SEL"
  #endif
  #if (STM32_LPUART1SEL == RCC_CCIPR3_LPUART1SEL_CSI)
    #error "CSI not enabled, required by STM32_LPUART1SEL"
  #endif

  #if (STM32_SPI4SEL == RCC_CCIPR3_SPI4SEL_CSI)
    #error "CSI not enabled, required by STM32_SPI4SEL"
  #endif
  #if (STM32_SPI5SEL == RCC_CCIPR3_SPI5SEL_CSI)
    #error "CSI not enabled, required by STM32_SPI5SEL"
  #endif
  #if (STM32_SPI6SEL == RCC_CCIPR3_SPI6SEL_CSI)
    #error "CSI not enabled, required by STM32_SPI6SEL"
  #endif

  #if (STM32_I2C1SEL == RCC_CCIPR4_I2C1SEL_CSI)
    #error "CSI not enabled, required by STM32_I2C1SEL"
  #endif
  #if (STM32_I2C2SEL == RCC_CCIPR4_I2C2SEL_CSI)
    #error "CSI not enabled, required by STM32_I2C2SEL"
  #endif
  #if (STM32_I2C3SEL == RCC_CCIPR4_I2C3SEL_CSI)
    #error "CSI not enabled, required by STM32_I2C3SEL"
  #endif
  #if (STM32_I2C4SEL == RCC_CCIPR4_I2C4SEL_CSI)
    #error "CSI not enabled, required by STM32_I2C4SEL"
  #endif

  #if (STM32_ADCDACSEL == RCC_CCIPR5_ADCDACSEL_CSI)
    #error "CSI not enabled, required by STM32_ADCDACSEL"
  #endif

  #if STM32_CECSEL == RCC_CCIPR5_CECSEL_CSIDIV128
    #error "CSI not enabled, required by STM32_CECSEL"
  #endif

  #if STM32_CKPERSEL == RCC_CCIPR5_CKPERSEL_CSI
    #if (STM32_LPTIM1SEL == RCC_CCIPR5_LPTIM1SEL_PER)
      #error "CSI not enabled, required by STM32_LPTIM1SEL"
    #endif
    #if (STM32_LPTIM2SEL == RCC_CCIPR5_LPTIM2SEL_PER)
      #error "CSI not enabled, required by STM32_LPTIM2SEL"
    #endif
    #if (STM32_LPTIM3SEL == RCC_CCIPR5_LPTIM3SEL_PER)
      #error "CSI not enabled, required by STM32_LPTIM3SEL"
    #endif
    #if (STM32_LPTIM4SEL == RCC_CCIPR5_LPTIM4SEL_PER)
      #error "CSI not enabled, required by STM32_LPTIM4SEL"
    #endif
    #if (STM32_LPTIM5SEL == RCC_CCIPR5_LPTIM5SEL_PER)
      #error "CSI not enabled, required by STM32_LPTIM5SEL"
    #endif
    #if (STM32_LPTIM6SEL == RCC_CCIPR5_LPTIM6SEL_PER)
      #error "CSI not enabled, required by STM32_LPTIM6SEL"
    #endif

    #if (STM32_SPI1SEL == RCC_CCIPR3_SPI1SEL_PER)
      #error "CSI not enabled, required by STM32_SPI1SEL"
    #endif
    #if (STM32_SPI2SEL == RCC_CCIPR3_SPI2SEL_PER)
      #error "CSI not enabled, required by STM32_SPI2SEL"
    #endif
    #if (STM32_SPI3SEL == RCC_CCIPR3_SPI3SEL_PER)
      #error "CSI not enabled, required by STM32_SPI3SEL"
    #endif

    #if (STM32_OSPISEL == RCC_CCIPR4_OSPISEL_PER)
      #error "CSI not enabled, required by STM32_OSPISEL"
    #endif

    #if (STM32_SAI1SEL == RCC_CCIPR5_SAI1SEL_PER)
      #error "CSI not enabled, required by STM32_SAI1SEL"
    #endif
    #if (STM32_SAI2SEL == RCC_CCIPR5_SAI2SEL_PER)
      #error "CSI not enabled, required by STM32_SAI2SEL"
    #endif
  #endif
#endif /* !STM32_CSI_ENABLED */

/*
 * HSI48 related checks.
 */
#if STM32_HSI48_ENABLED
#else /* !STM32_HSI48_ENABLED */

  #if STM32_MCO1SEL == RCC_CFGR1_MCO1SEL_HSI48
    #error "HSI48 not enabled, required by STM32_MCO1SEL"
  #endif

  #if STM32_USBSEL == RCC_CCIPR4_USBSEL_HSI48
    #error "HSI48 not enabled, required by STM32_USBSEL"
  #endif

  #if STM32_RNGSEL == RCC_CCIPR5_RNGSEL_HSI48
    #error "HSI48 not enabled, required by STM32_RNGSEL"
  #endif

#endif /* !STM32_HSI48_ENABLED */

/*
 * HSI related checks.
 */
#if STM32_HSI_ENABLED
#else /* !STM32_HSI_ENABLED */

  #if STM32_ACTIVATE_PLL1 && (STM32_PLL1SRC == RCC_PLL1CFGR_PLL1SRC_HSI)
    #error "HSI not enabled, required by STM32_PLL1SRC"
  #endif
  #if STM32_ACTIVATE_PLL2 && (STM32_PLL2SRC == RCC_PLL2CFGR_PLL2SRC_HSI)
    #error "HSI not enabled, required by STM32_PLL2SRC"
  #endif
  #if STM32_ACTIVATE_PLL3 && (STM32_PLL3SRC == RCC_PLL3CFGR_PLL3SRC_HSI)
    #error "HSI not enabled, required by STM32_PLL3SRC"
  #endif

  #if STM32_SW == RCC_CFGR1_SW_HSI
    #error "HSI not enabled, required by STM32_SW"
  #endif

  #if STM32_MCO1SEL == RCC_CFGR1_MCO1SEL_HSI
    #error "HSI not enabled, required by STM32_MCO1SEL"
  #endif

  #if (STM32_USART1SEL == RCC_CCIPR1_USART1SEL_HSI)
    #error "HSI not enabled, required by STM32_USART1SEL"
  #endif
  #if (STM32_USART2SEL == RCC_CCIPR1_USART2SEL_HSI)
    #error "HSI not enabled, required by STM32_USART2SEL"
  #endif
  #if (STM32_USART3SEL == RCC_CCIPR1_USART3SEL_HSI)
    #error "HSI not enabled, required by STM32_USART3SEL"
  #endif
  #if (STM32_UART4SEL == RCC_CCIPR1_UART4SEL_HSI)
    #error "HSI not enabled, required by STM32_UART4SEL"
  #endif
  #if (STM32_UART5SEL == RCC_CCIPR1_UART5SEL_HSI)
    #error "HSI not enabled, required by STM32_UART5SEL"
  #endif
  #if (STM32_USART6SEL == RCC_CCIPR1_USART6SEL_HSI)
    #error "HSI not enabled, required by STM32_USART6SEL"
  #endif
  #if (STM32_UART7SEL == RCC_CCIPR1_UART7SEL_HSI)
    #error "HSI not enabled, required by STM32_UART7SEL"
  #endif
  #if (STM32_UART8SEL == RCC_CCIPR1_UART8SEL_HSI)
    #error "HSI not enabled, required by STM32_UART8SEL"
  #endif
  #if (STM32_UART9SEL == RCC_CCIPR1_UART9SEL_HSI)
    #error "HSI not enabled, required by STM32_UART9SEL"
  #endif
  #if (STM32_USART10SEL == RCC_CCIPR1_USART10SEL_HSI)
    #error "HSI not enabled, required by STM32_USART10SEL"
  #endif
  #if (STM32_USART11SEL == RCC_CCIPR2_USART11SEL_HSI)
    #error "HSI not enabled, required by STM32_USART11SEL"
  #endif
  #if (STM32_UART12SEL == RCC_CCIPR2_UART12SEL_HSI)
    #error "HSI not enabled, required by STM32_UART12SEL"
  #endif
  #if (STM32_LPUART1SEL == RCC_CCIPR3_LPUART1SEL_HSI)
    #error "HSI not enabled, required by STM32_LPUART1SEL"
  #endif

  #if (STM32_SPI4SEL == RCC_CCIPR3_SPI4SEL_HSI)
    #error "HSI not enabled, required by STM32_SPI4SEL"
  #endif
  #if (STM32_SPI5SEL == RCC_CCIPR3_SPI5SEL_HSI)
    #error "HSI not enabled, required by STM32_SPI5SEL"
  #endif
  #if (STM32_SPI6SEL == RCC_CCIPR3_SPI6SEL_HSI)
    #error "HSI not enabled, required by STM32_SPI6SEL"
  #endif

  #if (STM32_I2C1SEL == RCC_CCIPR4_I2C1SEL_HSI)
    #error "HSI not enabled, required by STM32_I2C1SEL"
  #endif
  #if (STM32_I2C2SEL == RCC_CCIPR4_I2C2SEL_HSI)
    #error "HSI not enabled, required by STM32_I2C2SEL"
  #endif
  #if (STM32_I2C3SEL == RCC_CCIPR4_I2C3SEL_HSI)
    #error "HSI not enabled, required by STM32_I2C3SEL"
  #endif
  #if (STM32_I2C4SEL == RCC_CCIPR4_I2C4SEL_HSI)
    #error "HSI not enabled, required by STM32_I2C4SEL"
  #endif

  #if (STM32_I3C1SEL == RCC_CCIPR4_I3C1SEL_HSI)
    #error "HSI not enabled, required by STM32_I3C1SEL"
  #endif

  #if (STM32_ADCDACSEL == RCC_CCIPR5_ADCDACSEL_HSI)
    #error "HSI not enabled, required by STM32_ADCDACSEL"
  #endif

  #if STM32_CKPERSEL == RCC_CCIPR5_CKPERSEL_HSI
    #if (STM32_LPTIM1SEL == RCC_CCIPR2_LPTIM1SEL_PER)
      #error "HSI not enabled, required by STM32_LPTIM1SEL"
    #endif
    #if (STM32_LPTIM2SEL == RCC_CCIPR2_LPTIM2SEL_PER)
      #error "HSI not enabled, required by STM32_LPTIM2SEL"
    #endif
    #if (STM32_LPTIM3SEL == RCC_CCIPR2_LPTIM3SEL_PER)
      #error "HSI not enabled, required by STM32_LPTIM3SEL"
    #endif
    #if (STM32_LPTIM4SEL == RCC_CCIPR2_LPTIM4SEL_PER)
      #error "HSI not enabled, required by STM32_LPTIM4SEL"
    #endif
    #if (STM32_LPTIM5SEL == RCC_CCIPR2_LPTIM5SEL_PER)
      #error "HSI not enabled, required by STM32_LPTIM5SEL"
    #endif
    #if (STM32_LPTIM6SEL == RCC_CCIPR2_LPTIM6SEL_PER)
      #error "HSI not enabled, required by STM32_LPTIM6SEL"
    #endif

    #if (STM32_SPI1SEL == RCC_CCIPR3_SPI1SEL_PER)
      #error "HSI not enabled, required by STM32_SPI1SEL"
    #endif
    #if (STM32_SPI2SEL == RCC_CCIPR3_SPI2SEL_PER)
      #error "HSI not enabled, required by STM32_SPI2SEL"
    #endif
    #if (STM32_SPI3SEL == RCC_CCIPR3_SPI3SEL_PER)
      #error "HSI not enabled, required by STM32_SPI3SEL"
    #endif

    #if (STM32_OSPISEL == RCC_CCIPR4_OSPISEL_PER)
      #error "HSI not enabled, required by STM32_OSPISEL"
    #endif

    #if (STM32_SAI1SEL == RCC_CCIPR5_SAI1SEL_PER)
      #error "HSI not enabled, required by STM32_SAI1SEL"
    #endif
    #if (STM32_SAI2SEL == RCC_CCIPR5_SAI2SEL_PER)
      #error "HSI not enabled, required by STM32_SAI2SEL"
    #endif
  #endif

#endif /* !STM32_HSI_ENABLED */

/*
 * LSE related checks.
 */
#if STM32_LSE_ENABLED
#else /* !STM32_LSE_ENABLED */

  #if STM32_MCO1SEL == RCC_CFGR1_MCO1SEL_LSE
    #error "LSE not enabled, required by STM32_MCO1SEL"
  #endif

  #if (STM32_USART1SEL == RCC_CCIPR1_USART1SEL_LSE)
    #error "LSE not enabled, required by STM32_USART1SEL"
  #endif
  #if (STM32_USART2SEL == RCC_CCIPR1_USART2SEL_LSE)
    #error "LSE not enabled, required by STM32_USART2SEL"
  #endif
  #if (STM32_USART3SEL == RCC_CCIPR1_USART3SEL_LSE)
    #error "LSE not enabled, required by STM32_USART3SEL"
  #endif
  #if (STM32_UART4SEL == RCC_CCIPR1_UART4SEL_LSE)
    #error "LSE not enabled, required by STM32_UART4SEL"
  #endif
  #if (STM32_UART5SEL == RCC_CCIPR1_UART5SEL_LSE)
    #error "LSE not enabled, required by STM32_UART5SEL"
  #endif
  #if (STM32_USART6SEL == RCC_CCIPR1_USART6SEL_LSE)
    #error "LSE not enabled, required by STM32_USART6SEL"
  #endif
  #if (STM32_UART7SEL == RCC_CCIPR1_UART7SEL_LSE)
    #error "LSE not enabled, required by STM32_UART7SEL"
  #endif
  #if (STM32_UART8SEL == RCC_CCIPR1_UART8SEL_LSE)
    #error "LSE not enabled, required by STM32_UART8SEL"
  #endif
  #if (STM32_UART9SEL == RCC_CCIPR1_UART9SEL_LSE)
    #error "LSE not enabled, required by STM32_UART9SEL"
  #endif
  #if (STM32_USART10SEL == RCC_CCIPR1_USART10SEL_LSE)
    #error "LSE not enabled, required by STM32_USART10SEL"
  #endif
  #if (STM32_USART11SEL == RCC_CCIPR2_USART11SEL_LSE)
    #error "LSE not enabled, required by STM32_USART11SEL"
  #endif
  #if (STM32_UART12SEL == RCC_CCIPR2_UART12SEL_LSE)
    #error "LSE not enabled, required by STM32_UART12SEL"
  #endif
  #if (STM32_LPUART1SEL == RCC_CCIPR3_LPUART1SEL_LSE)
    #error "LSE not enabled, required by STM32_LPUART1SEL"
  #endif

  #if (STM32_LPTIM1SEL == RCC_CCIPR2_LPTIM1SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM1SEL"
  #endif
  #if (STM32_LPTIM2SEL == RCC_CCIPR2_LPTIM2SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM2SEL"
  #endif
  #if (STM32_LPTIM3SEL == RCC_CCIPR2_LPTIM3SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM3SEL"
  #endif
  #if (STM32_LPTIM4SEL == RCC_CCIPR2_LPTIM4SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM4SEL"
  #endif
  #if (STM32_LPTIM5SEL == RCC_CCIPR2_LPTIM5SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM5SEL"
  #endif
  #if (STM32_LPTIM6SEL == RCC_CCIPR2_LPTIM6SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM6SEL"
  #endif

  #if STM32_SYSTICKSEL == RCC_CCIPR4_SYSTICKSEL_LSE
    #error "LSE not enabled, required by STM32_SYSTICKSEL"
  #endif

  #if STM32_DACSEL == RCC_CCIPR5_DACSEL_LSE
    #error "LSE not enabled, required by STM32_DACSEL"
  #endif

  #if STM32_CECSEL == RCC_CCIPR5_CECSEL_LSE
    #error "LSE not enabled, required by STM32_CECSEL"
  #endif

  #if STM32_RTCSEL == RCC_BDCR_RTCSEL_LSE
    #error "LSE not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_LSCOSEL == RCC_BDCR_LSCOSEL_LSE
    #error "LSE not enabled, required by STM32_LSCOSEL"
  #endif

#endif /* !STM32_LSE_ENABLED */

/*
 * HSE related checks.
 */
#if STM32_HSE_ENABLED
#else /* !STM32_HSE_ENABLED */

  #if STM32_ACTIVATE_PLL1 && (STM32_PLL1SRC == RCC_PLL1CFGR_PLL1SRC_HSE)
    #error "HSE not enabled, required by STM32_PLL1SRC"
  #endif
  #if STM32_ACTIVATE_PLL2 && (STM32_PLL2SRC == RCC_PLL2CFGR_PLL2SRC_HSE)
    #error "HSE not enabled, required by STM32_PLL2SRC"
  #endif
  #if STM32_ACTIVATE_PLL3 && (STM32_PLL3SRC == RCC_PLL3CFGR_PLL3SRC_HSE)
    #error "HSE not enabled, required by STM32_PLL3SRC"
  #endif

  #if STM32_SW == RCC_CFGR1_SW_HSE
    #error "HSE not enabled, required by STM32_SW"
  #endif

  #if STM32_MCO1SEL == RCC_CFGR1_MCO1SEL_HSE
    #error "HSE not enabled, required by STM32_MCO1SEL"
  #endif

  #if STM32_MCO2SEL == RCC_CFGR1_MCO2SEL_HSE
    #error "HSE not enabled, required by STM32_MCO2SEL"
  #endif

  #if (STM32_SPI4SEL == RCC_CCIPR3_SPI4SEL_HSE)
    #error "HSE not enabled, required by STM32_SPI4SEL"
  #endif
  #if (STM32_SPI5SEL == RCC_CCIPR3_SPI5SEL_HSE)
    #error "HSE not enabled, required by STM32_SPI5SEL"
  #endif
  #if (STM32_SPI6SEL == RCC_CCIPR3_SPI6SEL_HSE)
    #error "HSE not enabled, required by STM32_SPI6SEL"
  #endif

  #if STM32_FDCANSEL == RCC_CCIPR5_FDCANSEL_HSE
    #error "HSE not enabled, required by STM32_FDCANSEL"
  #endif

  #if STM32_RTCSEL == RCC_BDCR_RTCSEL_HSEDIV
    #error "HSE not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_CKPERSEL == RCC_CCIPR5_CKPERSEL_HSE
    #if (STM32_LPTIM1SEL == RCC_CCIPR2_LPTIM1SEL_PER)
      #error "HSE not enabled, required by STM32_LPTIM1SEL"
    #endif
    #if (STM32_LPTIM2SEL == RCC_CCIPR2_LPTIM2SEL_PER)
      #error "HSE not enabled, required by STM32_LPTIM2SEL"
    #endif
    #if (STM32_LPTIM3SEL == RCC_CCIPR2_LPTIM3SEL_PER)
      #error "HSE not enabled, required by STM32_LPTIM3SEL"
    #endif
    #if (STM32_LPTIM4SEL == RCC_CCIPR2_LPTIM4SEL_PER)
      #error "HSE not enabled, required by STM32_LPTIM4SEL"
    #endif
    #if (STM32_LPTIM5SEL == RCC_CCIPR2_LPTIM5SEL_PER)
      #error "HSE not enabled, required by STM32_LPTIM5SEL"
    #endif
    #if (STM32_LPTIM6SEL == RCC_CCIPR2_LPTIM6SEL_PER)
      #error "HSE not enabled, required by STM32_LPTIM6SEL"
    #endif

    #if (STM32_SPI1SEL == RCC_CCIPR3_SPI1SEL_PER)
      #error "HSE not enabled, required by STM32_SPI1SEL"
    #endif
    #if (STM32_SPI2SEL == RCC_CCIPR3_SPI2SEL_PER)
      #error "HSE not enabled, required by STM32_SPI2SEL"
    #endif
    #if (STM32_SPI3SEL == RCC_CCIPR3_SPI3SEL_PER)
      #error "HSE not enabled, required by STM32_SPI3SEL"
    #endif

    #if (STM32_OSPISEL == RCC_CCIPR4_OSPISEL_PER)
      #error "HSE not enabled, required by STM32_OSPISEL"
    #endif

    #if (STM32_SAI1SEL == RCC_CCIPR5_SAI1SEL_PER)
      #error "HSE not enabled, required by STM32_SAI1SEL"
    #endif
    #if (STM32_SAI2SEL == RCC_CCIPR5_SAI2SEL_PER)
      #error "HSE not enabled, required by STM32_SAI2SEL"
    #endif
  #endif
#endif /* !STM32_HSE_ENABLED */

/**
 * @brief   PLL1 input clock frequency.
 */
#if (STM32_PLL1SRC == RCC_PLL1CFGR_PLL1SRC_HSE) || defined(__DOXYGEN__)
  #define STM32_PLL1IN              STM32_HSECLK

#elif STM32_PLL1SRC == RCC_PLL1CFGR_PLL1SRC_CSI
  #define STM32_PLL1IN              STM32_CSICLK

#elif STM32_PLL1SRC == RCC_PLL1CFGR_PLL1SRC_HSI
  #define STM32_PLL1IN              hal_lld_get_clock_point(CLK_HSI)

#elif STM32_PLL1SRC == RCC_PLL1CFGR_PLL1SRC_NOCLOCK
  #define STM32_PLL1IN              0

#else
  #error "invalid STM32_PLL1SRC value specified"
#endif

/**
 * @brief   PLL2 input clock frequency.
 */
#if (STM32_PLL2SRC == RCC_PLL2CFGR_PLL2SRC_HSE) || defined(__DOXYGEN__)
  #define STM32_PLL2IN              STM32_HSECLK

#elif STM32_PLL2SRC == RCC_PLL2CFGR_PLL2SRC_CSI
  #define STM32_PLL2IN              STM32_CSICLK

#elif STM32_PLL2SRC == RCC_PLL2CFGR_PLL2SRC_HSI
  #define STM32_PLL2IN              hal_lld_get_clock_point(CLK_HSI)

#elif STM32_PLL2SRC == RCC_PLL2CFGR_PLL2SRC_NOCLOCK
  #define STM32_PLL2IN              0

#else
  #error "invalid STM32_PLL2SRC value specified"
#endif

/**
 * @brief   PLL3 input clock frequency.
 */
#if STM32_RCC_HAS_PLL3
  #if (STM32_PLL3SRC == RCC_PLL3CFGR_PLL3SRC_HSE) || defined(__DOXYGEN__)
    #define STM32_PLL3IN              STM32_HSECLK

  #elif STM32_PLL3SRC == RCC_PLL3CFGR_PLL3SRC_CSI
    #define STM32_PLL3IN              STM32_CSICLK

  #elif STM32_PLL3SRC == RCC_PLL3CFGR_PLL3SRC_HSI
    #define STM32_PLL3IN              STM32_HSICLK

  #elif STM32_PLL3SRC == RCC_PLL3CFGR_PLL3SRC_NOCLOCK
    #define STM32_PLL3IN              0

  #else
    #error "invalid STM32_PLL3SRC value specified"
  #endif
#endif

/**
 * @brief   STM32_PLL1PEN field.
 */
#if (STM32_SW           == RCC_CFGR1_SW_PLL1P)          ||                  \
    (STM32_MCO1SEL      == RCC_CFGR1_MCO1SEL_PLL1P)     ||                  \
    (STM32_MCO2SEL      == RCC_CFGR1_MCO2SEL_PLL1P)     ||                  \
    defined(__DOXYGEN__)
  #define STM32_PLL1PEN             RCC_PLL1CFGR_PLL1PEN

#else
  #define STM32_PLL1PEN             0U
#endif

/**
 * @brief   STM32_PLL1QEN field.
 */
#if (STM32_SPI1SEL      == RCC_CCIPR3_SPI1SEL_PLL1Q)     ||                 \
    (STM32_SPI2SEL      == RCC_CCIPR3_SPI2SEL_PLL1Q)     ||                 \
    (STM32_SPI3SEL      == RCC_CCIPR3_SPI3SEL_PLL1Q)     ||                 \
    (STM32_OSPISEL      == RCC_CCIPR4_OSPISEL_PLL1Q)     ||                 \
    (STM32_USBSEL       == RCC_CCIPR4_USBSEL_PLL1Q)      ||                 \
    (STM32_SDMMC1SEL    == RCC_CCIPR4_SDMMC1SEL_PLL1Q)   ||                 \
    (STM32_SDMMC2SEL    == RCC_CCIPR4_SDMMC2SEL_PLL1Q)   ||                 \
    (STM32_RNGSEL       == RCC_CCIPR5_RNGSEL_PLL1Q)      ||                 \
    (STM32_FDCANSEL     == RCC_CCIPR5_FDCANSEL_PLL1Q)    ||                 \
    (STM32_SAI1SEL      == RCC_CCIPR5_SAI1SEL_PLL1Q)     ||                 \
    (STM32_SAI2SEL      == RCC_CCIPR5_SAI2SEL_PLL1Q)     ||                 \
    defined(__DOXYGEN__)
  #define STM32_PLL1QEN             RCC_PLL1CFGR_PLL1QEN

#else
  #define STM32_PLL1QEN             0U
#endif

/**
 * @brief   STM32_PLL1REN field.
 */
#if FALSE                                           ||                      \
    defined(__DOXYGEN__)
  #define STM32_PLL1REN             RCC_PLL1CFGR_PLL1REN

#else
  #define STM32_PLL1REN             0U
#endif

/**
 * @brief   STM32_PLL2PEN field.
 */
#if (STM32_SW           == RCC_CFGR1_SW_PLL1P)          ||                  \
    (STM32_MCO2SEL      == RCC_CFGR1_MCO2SEL_PLL1P)     ||                  \
    (STM32_LPTIM1SEL    == RCC_CCIPR2_LPTIM1SEL_PLL2P)   ||                 \
    (STM32_LPTIM2SEL    == RCC_CCIPR2_LPTIM2SEL_PLL2P)   ||                 \
    (STM32_LPTIM3SEL    == RCC_CCIPR2_LPTIM3SEL_PLL2P)   ||                 \
    (STM32_LPTIM4SEL    == RCC_CCIPR2_LPTIM4SEL_PLL2P)   ||                 \
    (STM32_LPTIM5SEL    == RCC_CCIPR2_LPTIM5SEL_PLL2P)   ||                 \
    (STM32_LPTIM6SEL    == RCC_CCIPR2_LPTIM6SEL_PLL2P)   ||                 \
    (STM32_SPI1SEL      == RCC_CCIPR3_SPI1SEL_PLL2P)     ||                 \
    (STM32_SPI2SEL      == RCC_CCIPR3_SPI2SEL_PLL2P)     ||                 \
    (STM32_SPI3SEL      == RCC_CCIPR3_SPI3SEL_PLL2P)     ||                 \
    (STM32_SPI4SEL      == RCC_CCIPR3_SPI4SEL_PLL2P)     ||                 \
    (STM32_SPI5SEL      == RCC_CCIPR3_SPI5SEL_PLL2P)     ||                 \
    (STM32_SPI6SEL      == RCC_CCIPR3_SPI6SEL_PLL2P)     ||                 \
    (STM32_SAI1SEL      == RCC_CCIPR5_SAI1SEL_PLL2P)     ||                 \
    (STM32_SAI2SEL      == RCC_CCIPR5_SAI2SEL_PLL2P)     ||                 \
    defined(__DOXYGEN__)
  #define STM32_PLL2PEN             RCC_PLL2CFGR_PLL2PEN

#else
  #define STM32_PLL2PEN             0U
#endif

/**
 * @brief   STM32_PLL2QEN field.
 */
#if (STM32_USART1SEL    == RCC_CCIPR1_USART1SEL_PLL2Q)   ||                 \
    (STM32_USART2SEL    == RCC_CCIPR1_USART2SEL_PLL2Q)   ||                 \
    (STM32_USART3SEL    == RCC_CCIPR1_USART3SEL_PLL2Q)   ||                 \
    (STM32_UART4SEL     == RCC_CCIPR1_UART4SEL_PLL2Q)    ||                 \
    (STM32_UART5SEL     == RCC_CCIPR1_UART5SEL_PLL2Q)    ||                 \
    (STM32_USART6SEL    == RCC_CCIPR1_USART6SEL_PLL2Q)   ||                 \
    (STM32_UART7SEL     == RCC_CCIPR1_UART7SEL_PLL2Q)    ||                 \
    (STM32_UART8SEL     == RCC_CCIPR1_UART8SEL_PLL2Q)    ||                 \
    (STM32_UART9SEL     == RCC_CCIPR1_UART9SEL_PLL2Q)    ||                 \
    (STM32_USART10SEL   == RCC_CCIPR1_USART10SEL_PLL2Q)  ||                 \
    (STM32_USART11SEL   == RCC_CCIPR2_USART11SEL_PLL2Q)  ||                 \
    (STM32_UART12SEL    == RCC_CCIPR2_UART12SEL_PLL2Q)   ||                 \
    (STM32_LPUART1SEL   == RCC_CCIPR3_LPUART1SEL_PLL2Q)  ||                 \
    (STM32_FDCANSEL     == RCC_CCIPR5_FDCANSEL_PLL2Q)    ||                 \
    defined(__DOXYGEN__)
  #define STM32_PLL2QEN             RCC_PLL2CFGR_PLL2QEN

#else
  #define STM32_PLL2QEN             0U
#endif

/**
 * @brief   STM32_PLL2REN field.
 */
#if (STM32_OSPISEL      == RCC_CCIPR4_OSPISEL_PLL2R)     ||                 \
    (STM32_SDMMC1SEL    == RCC_CCIPR4_SDMMC1SEL_PLL2R)   ||                 \
    (STM32_SDMMC2SEL    == RCC_CCIPR4_SDMMC2SEL_PLL2R)   ||                 \
    (STM32_ADCDACSEL    == RCC_CCIPR5_ADCDACSEL_PLL2R)   ||                 \
    defined(__DOXYGEN__)
  #define STM32_PLL2REN             RCC_PLL2CFGR_PLL2REN

#else
  #define STM32_PLL2REN             0U
#endif

/**
 * @brief   STM32_PLL3PEN field.
 */
#if (STM32_SPI1SEL      == RCC_CCIPR3_SPI1SEL_PLL3P)     ||                 \
    (STM32_SPI2SEL      == RCC_CCIPR3_SPI2SEL_PLL3P)     ||                 \
    (STM32_SPI3SEL      == RCC_CCIPR3_SPI3SEL_PLL3P)     ||                 \
    (STM32_SPI4SEL      == RCC_CCIPR3_SPI4SEL_PLL3P)     ||                 \
    (STM32_SPI5SEL      == RCC_CCIPR3_SPI5SEL_PLL3P)     ||                 \
    (STM32_SPI6SEL      == RCC_CCIPR3_SPI6SEL_PLL3P)     ||                 \
    (STM32_SAI1SEL      == RCC_CCIPR5_SAI1SEL_PLL3P)     ||                 \
    (STM32_SAI2SEL      == RCC_CCIPR5_SAI2SEL_PLL3P)     ||                 \
    defined(__DOXYGEN__)
  #define STM32_PLL3PEN             RCC_PLL3CFGR_PLL3PEN

#else
  #define STM32_PLL3PEN             0U
#endif

/**
 * @brief   STM32_PLL3QEN field.
 */
#if (STM32_USART1SEL    == RCC_CCIPR1_USART1SEL_PLL3Q)   ||                 \
    (STM32_USART2SEL    == RCC_CCIPR1_USART2SEL_PLL3Q)   ||                 \
    (STM32_USART3SEL    == RCC_CCIPR1_USART3SEL_PLL3Q)   ||                 \
    (STM32_UART4SEL     == RCC_CCIPR1_UART4SEL_PLL3Q)    ||                 \
    (STM32_UART5SEL     == RCC_CCIPR1_UART5SEL_PLL3Q)    ||                 \
    (STM32_USART6SEL    == RCC_CCIPR1_USART6SEL_PLL3Q)   ||                 \
    (STM32_UART7SEL     == RCC_CCIPR1_UART7SEL_PLL3Q)    ||                 \
    (STM32_UART8SEL     == RCC_CCIPR1_UART8SEL_PLL3Q)    ||                 \
    (STM32_UART9SEL     == RCC_CCIPR1_UART9SEL_PLL3Q)    ||                 \
    (STM32_USART10SEL   == RCC_CCIPR1_USART10SEL_PLL3Q)  ||                 \
    (STM32_USART11SEL   == RCC_CCIPR2_USART11SEL_PLL3Q)  ||                 \
    (STM32_UART12SEL    == RCC_CCIPR2_UART12SEL_PLL3Q)   ||                 \
    (STM32_LPUART1SEL   == RCC_CCIPR3_LPUART1SEL_PLL3Q)  ||                 \
    (STM32_USBSEL       == RCC_CCIPR4_USBSEL_PLL3Q)  ||                     \
    defined(__DOXYGEN__)
  #define STM32_PLL3QEN             RCC_PLL3CFGR_PLL3QEN

#else
  #define STM32_PLL3QEN             0U
#endif

/**
 * @brief   STM32_PLL3REN field.
 */
#if (STM32_LPTIM1SEL    == RCC_CCIPR2_LPTIM1SEL_PLL3R)   ||                 \
    (STM32_LPTIM2SEL    == RCC_CCIPR2_LPTIM2SEL_PLL3R)   ||                 \
    (STM32_LPTIM3SEL    == RCC_CCIPR2_LPTIM3SEL_PLL3R)   ||                 \
    (STM32_LPTIM4SEL    == RCC_CCIPR2_LPTIM4SEL_PLL3R)   ||                 \
    (STM32_LPTIM5SEL    == RCC_CCIPR2_LPTIM5SEL_PLL3R)   ||                 \
    (STM32_LPTIM6SEL    == RCC_CCIPR2_LPTIM6SEL_PLL3R)   ||                 \
    (STM32_I2C1SEL      == RCC_CCIPR4_I2C1SEL_PLL3R)     ||                 \
    (STM32_I2C2SEL      == RCC_CCIPR4_I2C2SEL_PLL3R)     ||                 \
    (STM32_I2C3SEL      == RCC_CCIPR4_I2C3SEL_PLL3R)     ||                 \
    (STM32_I2C4SEL      == RCC_CCIPR4_I2C4SEL_PLL3R)     ||                 \
    (STM32_I3C1SEL      == RCC_CCIPR4_I3C1SEL_PLL3R)     ||                 \
    defined(__DOXYGEN__)
  #define STM32_PLL3REN             RCC_PLL3CFGR_PLL3REN

#else
  #define STM32_PLL3REN             0U
#endif

/* Inclusion of PLL-related checks and calculations.*/
#include <stm32_pll1.inc>
#include <stm32_pll2.inc>
#include <stm32_pll3.inc>

/**
 * @brief   System clock source.
 */
#if (STM32_SW == RCC_CFGR1_SW_HSI) || defined(__DOXYGEN__)
  #define STM32_SYSCLK              STM32_HSICLK

#elif STM32_SW == RCC_CFGR1_SW_CSI
  #define STM32_SYSCLK              STM32_CSICLK

#elif STM32_SW == RCC_CFGR1_SW_HSE
  #define STM32_SYSCLK              STM32_HSECLK

#elif STM32_SW == RCC_CFGR1_SW_PLL1P
  #define STM32_SYSCLK              STM32_PLL1_P_CLKOUT

#else
  #error "invalid STM32_SW value specified"
#endif

/* Bus handlers.*/
#include "stm32_ahb.inc"
#include "stm32_apb1.inc"
#include "stm32_apb2.inc"
#include "stm32_apb3.inc"

/* STOPWUCK setting check.*/
#if (STM32_STOPWUCK == RCC_CFGR1_STOPWUCK_HSI) || defined(__DOXYGEN__)

#elif STM32_STOPWUCK == RCC_CFGR1_STOPWUCK_CSI

#else
  #error "invalid STM32_STOPWUCK value specified"
#endif

/* STOPKERWUCK setting check.*/
#if (STM32_STOPKERWUCK == RCC_CFGR1_STOPKERWUCK_HSI) || defined(__DOXYGEN__)

#elif STM32_STOPKERWUCK == RCC_CFGR1_STOPKERWUCK_CSI

#else
  #error "invalid STM32_STOPKERWUCK value specified"
#endif

/**
 * @brief   RTCPRE clock frequency.
 */
#if (STM32_RTCPRE_VALUE == RCC_CFGR1_RTCPRE_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_RTCPRECLK           0U

#elif (STM32_RTCPRE_VALUE >= 2) && (STM32_RTCPRE_VALUE <= 63)
  #define STM32_RTCPRECLK           (STM32_HSECLK / STM32_RTCPRE_VALUE)

#else
  #error "invalid STM32_RTCPRECLK_VALUE value specified"
#endif

/**
 * @brief   RTCPRE field.
 */
#define STM32_RTCPRE                RCC_CFGR1_RTCPRE_FIELD(STM32_RTCPRE_VALUE)

/**
 * @brief   MCO1 source clock.
 */
#if (STM32_MCO1SEL == RCC_CFGR1_MCO1SEL_HSI) || defined(__DOXYGEN__)
  #define STM32_MCO1DIVCLK          STM32_HSICLK

#elif STM32_MCO1SEL == RCC_CFGR1_MCO1SEL_LSE
  #define STM32_MCO1DIVCLK          STM32_LSECLK

#elif STM32_MCO1SEL == RCC_CFGR1_MCO1SEL_HSI
  #define STM32_MCO1DIVCLK          STM32_HSICLK

#elif STM32_MCO1SEL == RCC_CFGR1_MCO1SEL_HSE
  #define STM32_MCO1DIVCLK          STM32_HSECLK

#elif STM32_MCO1SEL == RCC_CFGR1_MCO1SEL_PLL1P
  #define STM32_MCO1DIVCLK          STM32_PLL1_P_CLKOUT

#elif STM32_MCO1SEL == RCC_CFGR1_MCO1SEL_HSI48
  #define STM32_MCO1DIVCLK          STM32_HSI48CLK

#else
  #error "invalid STM32_MCO1SEL value specified"
#endif

/**
 * @brief   MCO1 output pin clock frequency.
 */
#if (STM32_MCO1PRE_VALUE == RCC_CFGR1_MCO1PRE_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_MCO1CLK             0U

#elif (STM32_MCO1PRE_VALUE > RCC_CFGR1_MCO1PRE_NOCLOCK) &&                  \
      (STM32_MCO1PRE_VALUE <= 15)
  #define STM32_MCO1CLK             (STM32_MCO1DIVCLK / STM32_MCO1PRE_VALUE)

#else
#error "invalid STM32_MCO1PRE_VALUE value specified"
#endif

/**
 * @brief   MCO1PRE field.
 */
#define STM32_MCO1PRE               RCC_CFGR1_MCO1PRE_FIELD(STM32_MCO1PRE_VALUE)

/**
 * @brief   MCO2 source clock.
 */
#if (STM32_MCO2SEL == RCC_CFGR1_MCO2SEL_SYSCLK) || defined(__DOXYGEN__)
  #define STM32_MCO2DIVCLK          STM32_SYSCLK

#elif STM32_MCO2SEL == RCC_CFGR1_MCO2SEL_PLL2P
  #define STM32_MCO2DIVCLK          STM32_PLL2_P_CLKOUT

#elif STM32_MCO2SEL == RCC_CFGR1_MCO2SEL_HSE
  #define STM32_MCO2DIVCLK          STM32_HSECLK

#elif STM32_MCO2SEL == RCC_CFGR1_MCO2SEL_PLL1P
  #define STM32_MCO2DIVCLK          STM32_PLL1_P_CLKOUT

#elif STM32_MCO2SEL == RCC_CFGR1_MCO2SEL_CSI
  #define STM32_MCO2DIVCLK          hal_lld_get_clock_point(CLK_CSI)

#elif STM32_MCO2SEL == RCC_CFGR1_MCO2SEL_LSI
  #define STM32_MCO2DIVCLK          STM32_LSICLK

#else
  #error "invalid STM32_MCO2SEL value specified"
#endif

/**
 * @brief   MCO2 output pin clock frequency.
 */
#if (STM32_MCO2PRE_VALUE == RCC_CFGR1_MCO2PRE_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_MCO2CLK             0U

#elif (STM32_MCO2PRE_VALUE > RCC_CFGR1_MCO2PRE_NOCLOCK) &&                  \
      (STM32_MCO2PRE_VALUE <= 15)
  #define STM32_MCO2CLK             (STM32_MCO2DIVCLK / STM32_MCO2PRE_VALUE)

#else
#error "invalid STM32_MCO2PRE_VALUE value specified"
#endif

/**
 * @brief   MCO2PRE field.
 */
#define STM32_MCO2PRE               RCC_CFGR1_MCO2PRE_FIELD(STM32_MCO2PRE_VALUE)

/**
 * @brief   RTC clock frequency.
 */
#if (STM32_RTCSEL == RCC_BDCR_RTCSEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_RTCCLK              0U

#elif STM32_RTCSEL == RCC_BDCR_RTCSEL_LSE
  #define STM32_RTCCLK              STM32_LSECLK

#elif STM32_RTCSEL == RCC_BDCR_RTCSEL_LSI
  #define STM32_RTCCLK              STM32_LSICLK

#elif STM32_RTCSEL == RCC_BDCR_RTCSEL_HSEDIV
  #define STM32_RTCCLK              STM32_RTCPRECLK

#else
  #error "invalid STM32_RTCSEL value specified"
#endif

/**
 * @brief   LSCO clock frequency.
 */
#if (STM32_LSCOSEL == RCC_BDCR_LSCOSEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_LSCOCLK             0U

#elif STM32_LSCOSEL == RCC_BDCR_LSCOSEL_LSI
  #define STM32_LSCOCLK             STM32_LSICLK

#elif STM32_LSCOSEL == RCC_BDCR_LSCOSEL_LSE
  #define STM32_LSCOCLK             STM32_LSECLK

#else
  #error "invalid STM32_LSCOSEL value specified"
#endif

/**
 * @brief   USART1 clock frequency.
 */
#if (STM32_USART1SEL == RCC_CCIPR1_USART1SEL_PCLK2) || defined(__DOXYGEN__)
  #define STM32_USART1CLK           hal_lld_get_clock_point(CLK_PCLK2)

#elif STM32_USART1SEL == RCC_CCIPR1_USART1SEL_PLL2Q
  #define STM32_USART1CLK           hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_USART1SEL == RCC_CCIPR1_USART1SEL_PLL3Q
  #define STM32_USART1CLK           hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USART1SEL == RCC_CCIPR1_USART1SEL_HSI
  #define STM32_USART1CLK           hal_lld_get_clock_point(CLK_HSI)

#elif STM32_USART1SEL == RCC_CCIPR1_USART1SEL_CSI
  #define STM32_USART1CLK           hal_lld_get_clock_point(CLK_CSI)

#elif STM32_USART1SEL == RCC_CCIPR1_USART1SEL_LSE
  #define STM32_USART1CLK           STM32_LSECLK

#else
  #error "invalid source selected for USART1 clock"
#endif

/**
 * @brief   USART2 clock frequency.
 */
#if (STM32_USART2SEL == RCC_CCIPR1_USART2SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_USART2CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_USART2SEL == RCC_CCIPR1_USART2SEL_PLL2Q
  #define STM32_USART2CLK           hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_USART2SEL == RCC_CCIPR1_USART2SEL_PLL3Q
  #define STM32_USART2CLK           hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USART2SEL == RCC_CCIPR1_USART2SEL_HSI
  #define STM32_USART2CLK           hal_lld_get_clock_point(CLK_HSI)

#elif STM32_USART2SEL == RCC_CCIPR1_USART2SEL_CSI
  #define STM32_USART2CLK           hal_lld_get_clock_point(CLK_CSI)

#elif STM32_USART2SEL == RCC_CCIPR1_USART2SEL_LSE
  #define STM32_USART2CLK           STM32_LSECLK

#else
  #error "invalid source selected for USART2 clock"
#endif

/**
 * @brief   USART3 clock frequency.
 */
#if (STM32_USART3SEL == RCC_CCIPR1_USART3SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_USART3CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_USART3SEL == RCC_CCIPR1_USART3SEL_PLL2Q
  #define STM32_USART3CLK           hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_USART3SEL == RCC_CCIPR1_USART3SEL_PLL3Q
  #define STM32_USART3CLK           hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USART3SEL == RCC_CCIPR1_USART3SEL_HSI
  #define STM32_USART3CLK           hal_lld_get_clock_point(CLK_HSI)

#elif STM32_USART3SEL == RCC_CCIPR1_USART3SEL_CSI
  #define STM32_USART3CLK           hal_lld_get_clock_point(CLK_CSI)

#elif STM32_USART3SEL == RCC_CCIPR1_USART3SEL_LSE
  #define STM32_USART3CLK           STM32_LSECLK

#else
  #error "invalid source selected for USART3 clock"
#endif

/**
 * @brief   UART4 clock frequency.
 */
#if (STM32_UART4SEL == RCC_CCIPR1_UART4SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART4CLK            hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART4SEL == RCC_CCIPR1_UART4SEL_PLL2Q
  #define STM32_UART4CLK            hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_UART4SEL == RCC_CCIPR1_UART4SEL_PLL3Q
  #define STM32_UART4CLK            hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_UART4SEL == RCC_CCIPR1_UART4SEL_HSI
  #define STM32_UART4CLK            hal_lld_get_clock_point(CLK_HSI)

#elif STM32_UART4SEL == RCC_CCIPR1_UART4SEL_CSI
  #define STM32_UART4CLK            hal_lld_get_clock_point(CLK_CSI)

#elif STM32_UART4SEL == RCC_CCIPR1_UART4SEL_LSE
  #define STM32_UART4CLK            STM32_LSECLK

#else
  #error "invalid source selected for UART4 clock"
#endif

/**
 * @brief   UART5 clock frequency.
 */
#if (STM32_UART5SEL == RCC_CCIPR1_UART5SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART5CLK            hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART5SEL == RCC_CCIPR1_UART5SEL_PLL2Q
  #define STM32_UART5CLK            hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_UART5SEL == RCC_CCIPR1_UART5SEL_PLL3Q
  #define STM32_UART5CLK            hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_UART5SEL == RCC_CCIPR1_UART5SEL_HSI
  #define STM32_UART5CLK            hal_lld_get_clock_point(CLK_HSI)

#elif STM32_UART5SEL == RCC_CCIPR1_UART5SEL_CSI
  #define STM32_UART5CLK            hal_lld_get_clock_point(CLK_CSI)

#elif STM32_UART5SEL == RCC_CCIPR1_UART5SEL_LSE
  #define STM32_UART5CLK            STM32_LSECLK

#else
  #error "invalid source selected for UART5 clock"
#endif

/**
 * @brief   USART6 clock frequency.
 */
#if (STM32_USART6SEL == RCC_CCIPR1_USART6SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_USART6CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_USART6SEL == RCC_CCIPR1_USART6SEL_PLL2Q
  #define STM32_USART6CLK           hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_USART6SEL == RCC_CCIPR1_USART6SEL_PLL3Q
  #define STM32_USART6CLK           hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USART6SEL == RCC_CCIPR1_USART6SEL_HSI
  #define STM32_USART6CLK           hal_lld_get_clock_point(CLK_HSI)

#elif STM32_USART6SEL == RCC_CCIPR1_USART6SEL_CSI
  #define STM32_USART6CLK           hal_lld_get_clock_point(CLK_CSI)

#elif STM32_USART6SEL == RCC_CCIPR1_USART6SEL_LSE
  #define STM32_USART6CLK           STM32_LSECLK

#else
  #error "invalid source selected for USART6 clock"
#endif

/**
 * @brief   UART7 clock frequency.
 */
#if (STM32_UART7SEL == RCC_CCIPR1_UART7SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART7CLK            hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART7SEL == RCC_CCIPR1_UART7SEL_PLL2Q
  #define STM32_UART7CLK            hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_UART7SEL == RCC_CCIPR1_UART7SEL_PLL3Q
  #define STM32_UART7CLK            hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_UART7SEL == RCC_CCIPR1_UART7SEL_HSI
  #define STM32_UART7CLK            hal_lld_get_clock_point(CLK_HSI)

#elif STM32_UART7SEL == RCC_CCIPR1_UART7SEL_CSI
  #define STM32_UART7CLK            hal_lld_get_clock_point(CLK_CSI)

#elif STM32_UART7SEL == RCC_CCIPR1_UART7SEL_LSE
  #define STM32_UART7CLK            STM32_LSECLK

#else
  #error "invalid source selected for UART7 clock"
#endif

/**
 * @brief   UART8 clock frequency.
 */
#if (STM32_UART8SEL == RCC_CCIPR1_UART8SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART8CLK            hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART8SEL == RCC_CCIPR1_UART8SEL_PLL2Q
  #define STM32_UART8CLK            hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_UART8SEL == RCC_CCIPR1_UART8SEL_PLL3Q
  #define STM32_UART8CLK            hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_UART8SEL == RCC_CCIPR1_UART8SEL_HSI
  #define STM32_UART8CLK            hal_lld_get_clock_point(CLK_HSI)

#elif STM32_UART8SEL == RCC_CCIPR1_UART8SEL_CSI
  #define STM32_UART8CLK            hal_lld_get_clock_point(CLK_CSI)

#elif STM32_UART8SEL == RCC_CCIPR1_UART8SEL_LSE
  #define STM32_UART8CLK            STM32_LSECLK

#else
  #error "invalid source selected for UART8 clock"
#endif

/**
 * @brief   UART9 clock frequency.
 */
#if (STM32_UART9SEL == RCC_CCIPR1_UART9SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART9CLK            hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART9SEL == RCC_CCIPR1_UART9SEL_PLL2Q
  #define STM32_UART9CLK            hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_UART9SEL == RCC_CCIPR1_UART9SEL_PLL3Q
  #define STM32_UART9CLK            hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_UART9SEL == RCC_CCIPR1_UART9SEL_HSI
  #define STM32_UART9CLK            hal_lld_get_clock_point(CLK_HSI)

#elif STM32_UART9SEL == RCC_CCIPR1_UART9SEL_CSI
  #define STM32_UART9CLK            hal_lld_get_clock_point(CLK_CSI)

#elif STM32_UART9SEL == RCC_CCIPR1_UART9SEL_LSE
  #define STM32_UART9CLK            STM32_LSECLK

#else
  #error "invalid source selected for UART9 clock"
#endif

/**
 * @brief   USART10 clock frequency.
 */
#if (STM32_USART10SEL == RCC_CCIPR1_USART10SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_USART10CLK          hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_USART10SEL == RCC_CCIPR1_USART10SEL_PLL2Q
  #define STM32_USART10CLK          hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_USART10SEL == RCC_CCIPR1_USART10SEL_PLL3Q
  #define STM32_USART10CLK          hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USART10SEL == RCC_CCIPR1_USART10SEL_HSI
  #define STM32_USART10CLK          hal_lld_get_clock_point(CLK_HSI)

#elif STM32_USART10SEL == RCC_CCIPR1_USART10SEL_CSI
  #define STM32_USART10CLK          hal_lld_get_clock_point(CLK_CSI)

#elif STM32_USART10SEL == RCC_CCIPR1_USART10SEL_LSE
  #define STM32_USART10CLK          STM32_LSECLK

#else
  #error "invalid source selected for USART10 clock"
#endif

/**
 * @brief   USART11 clock frequency.
 */
#if (STM32_USART11SEL == RCC_CCIPR2_USART11SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_USART11CLK          hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_USART11SEL == RCC_CCIPR2_USART11SEL_PLL2Q
  #define STM32_USART11CLK          hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_USART11SEL == RCC_CCIPR2_USART11SEL_PLL3Q
  #define STM32_USART11CLK          hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USART11SEL == RCC_CCIPR2_USART11SEL_HSI
  #define STM32_USART11CLK          hal_lld_get_clock_point(CLK_HSI)

#elif STM32_USART11SEL == RCC_CCIPR2_USART11SEL_CSI
  #define STM32_USART11CLK          hal_lld_get_clock_point(CLK_CSI)

#elif STM32_USART11SEL == RCC_CCIPR2_USART11SEL_LSE
  #define STM32_USART11CLK          STM32_LSECLK

#else
  #error "invalid source selected for USART11 clock"
#endif

/**
 * @brief   UART12 clock frequency.
 */
#if (STM32_UART12SEL == RCC_CCIPR2_UART12SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART12CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART12SEL == RCC_CCIPR2_UART12SEL_PLL2Q
  #define STM32_UART12CLK           hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_UART12SEL == RCC_CCIPR2_UART12SEL_PLL3Q
  #define STM32_UART12CLK           hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_UART12SEL == RCC_CCIPR2_UART12SEL_HSI
  #define STM32_UART12CLK           hal_lld_get_clock_point(CLK_HSI)

#elif STM32_UART12SEL == RCC_CCIPR2_UART12SEL_CSI
  #define STM32_UART12CLK           hal_lld_get_clock_point(CLK_CSI)

#elif STM32_UART12SEL == RCC_CCIPR2_UART12SEL_LSE
  #define STM32_UART12CLK           STM32_LSECLK

#else
  #error "invalid source selected for UART12 clock"
#endif

/**
 * @brief   LPUART1 clock frequency.
 */
#if (STM32_LPUART1SEL == RCC_CCIPR3_LPUART1SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_LPUART1SEL == RCC_CCIPR3_LPUART1SEL_PLL2Q
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_LPUART1SEL == RCC_CCIPR3_LPUART1SEL_PLL3Q
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_LPUART1SEL == RCC_CCIPR3_LPUART1SEL_HSI
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_HSI)

#elif STM32_LPUART1SEL == RCC_CCIPR3_LPUART1SEL_CSI
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_CSI)

#elif STM32_LPUART1SEL == RCC_CCIPR3_LPUART1SEL_LSE
  #define STM32_LPUART1CLK          STM32_LSECLK

#else
  #error "invalid source selected for LPUART1 clock"
#endif

#if (STM32_TIMICSEL == RCC_CCIPR1_TIMICSEL_NOCLK) || defined(__DOXYGEN__)
  /**
   * @brief   TIM12 internal capture clock frequency.
   */
  #define STM32_TIM12CCLK           0
  /**
   * @brief   TIM15 internal capture clock frequency.
   */
  #define STM32_TIM15CCLK           0
  /**
   * @brief   LPTIM2 internal capture clock frequency.
   */
  #define STM32_LPTIM2CCLK          0

#elif STM32_TIMICSEL == RCC_CCIPR1_TIMICSEL_INTCLK
  #define STM32_TIM12CCLK           (hal_lld_get_clock_point(CLK_HSI) / 1024)
  #define STM32_TIM15CCLK           (hal_lld_get_clock_point(CLK_HSI) / 8)
  #define STM32_LPTIM2CCLK          (hal_lld_get_clock_point(CLK_CSI) / 128)

#else
  #error "invalid source selected for TIMICSEL clock"
#endif

/**
 * @brief   LPTIM1 clock frequency.
 */
#if (STM32_LPTIM1SEL == RCC_CCIPR2_LPTIM1SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_LPTIM1CLK           hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_LPTIM1SEL == RCC_CCIPR2_LPTIM1SEL_PLL2P
  #define STM32_LPTIM1CLK           hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_LPTIM1SEL == RCC_CCIPR2_LPTIM1SEL_PLL3R
  #define STM32_LPTIM1CLK           hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_LPTIM1SEL == RCC_CCIPR2_LPTIM1SEL_LSE
  #define STM32_LPTIM1CLK           STM32_LSECLK

#elif STM32_LPTIM1SEL == RCC_CCIPR2_LPTIM1SEL_LSI
  #define STM32_LPTIM1CLK           STM32_LSICLK

#elif STM32_LPTIM1SEL == RCC_CCIPR2_LPTIM1SEL_PER
  #define STM32_LPTIM1CLK           STM32_PERCLK

#else
  #error "invalid source selected for LPTIM1 clock"
#endif

/**
 * @brief   LPTIM2 clock frequency.
 */
#if (STM32_LPTIM2SEL == RCC_CCIPR2_LPTIM2SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_LPTIM2CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_LPTIM2SEL == RCC_CCIPR2_LPTIM2SEL_PLL2P
  #define STM32_LPTIM2CLK           hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_LPTIM2SEL == RCC_CCIPR2_LPTIM2SEL_PLL3R
  #define STM32_LPTIM2CLK           hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_LPTIM2SEL == RCC_CCIPR2_LPTIM2SEL_LSE
  #define STM32_LPTIM2CLK           STM32_LSECLK

#elif STM32_LPTIM2SEL == RCC_CCIPR2_LPTIM2SEL_LSI
  #define STM32_LPTIM2CLK           STM32_LSICLK

#elif STM32_LPTIM2SEL == RCC_CCIPR2_LPTIM2SEL_PER
  #define STM32_LPTIM2CLK           STM32_PERCLK

#else
  #error "invalid source selected for LPTIM2 clock"
#endif

/**
 * @brief   LPTIM3 clock frequency.
 */
#if (STM32_LPTIM3SEL == RCC_CCIPR2_LPTIM3SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_LPTIM3CLK           hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_LPTIM3SEL == RCC_CCIPR2_LPTIM3SEL_PLL2P
  #define STM32_LPTIM3CLK           hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_LPTIM3SEL == RCC_CCIPR2_LPTIM3SEL_PLL3R
  #define STM32_LPTIM3CLK           hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_LPTIM3SEL == RCC_CCIPR2_LPTIM3SEL_LSE
  #define STM32_LPTIM3CLK           STM32_LSECLK

#elif STM32_LPTIM3SEL == RCC_CCIPR2_LPTIM3SEL_LSI
  #define STM32_LPTIM3CLK           STM32_LSICLK

#elif STM32_LPTIM3SEL == RCC_CCIPR2_LPTIM3SEL_PER
  #define STM32_LPTIM3CLK           STM32_PERCLK

#else
  #error "invalid source selected for LPTIM3 clock"
#endif

/**
 * @brief   LPTIM4 clock frequency.
 */
#if (STM32_LPTIM4SEL == RCC_CCIPR2_LPTIM4SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_LPTIM4CLK           hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_LPTIM4SEL == RCC_CCIPR2_LPTIM4SEL_PLL2P
  #define STM32_LPTIM4CLK           hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_LPTIM4SEL == RCC_CCIPR2_LPTIM4SEL_PLL3R
  #define STM32_LPTIM4CLK           hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_LPTIM4SEL == RCC_CCIPR2_LPTIM4SEL_LSE
  #define STM32_LPTIM4CLK           STM32_LSECLK

#elif STM32_LPTIM4SEL == RCC_CCIPR2_LPTIM4SEL_LSI
  #define STM32_LPTIM4CLK           STM32_LSICLK

#elif STM32_LPTIM4SEL == RCC_CCIPR2_LPTIM4SEL_PER
  #define STM32_LPTIM4CLK           STM32_PERCLK

#else
  #error "invalid source selected for LPTIM4 clock"
#endif

/**
 * @brief   LPTIM5 clock frequency.
 */
#if (STM32_LPTIM5SEL == RCC_CCIPR2_LPTIM5SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_LPTIM5CLK           hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_LPTIM5SEL == RCC_CCIPR2_LPTIM5SEL_PLL2P
  #define STM32_LPTIM5CLK           hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_LPTIM5SEL == RCC_CCIPR2_LPTIM5SEL_PLL3R
  #define STM32_LPTIM5CLK           hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_LPTIM5SEL == RCC_CCIPR2_LPTIM5SEL_LSE
  #define STM32_LPTIM5CLK           STM32_LSECLK

#elif STM32_LPTIM5SEL == RCC_CCIPR2_LPTIM5SEL_LSI
  #define STM32_LPTIM5CLK           STM32_LSICLK

#elif STM32_LPTIM5SEL == RCC_CCIPR2_LPTIM5SEL_PER
  #define STM32_LPTIM5CLK           STM32_PERCLK

#else
  #error "invalid source selected for LPTIM5 clock"
#endif

/**
 * @brief   LPTIM6 clock frequency.
 */
#if (STM32_LPTIM6SEL == RCC_CCIPR2_LPTIM6SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_LPTIM6CLK           hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_LPTIM6SEL == RCC_CCIPR2_LPTIM6SEL_PLL2P
  #define STM32_LPTIM6CLK           hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_LPTIM6SEL == RCC_CCIPR2_LPTIM6SEL_PLL3R
  #define STM32_LPTIM6CLK           hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_LPTIM6SEL == RCC_CCIPR2_LPTIM6SEL_LSE
  #define STM32_LPTIM6CLK           STM32_LSECLK

#elif STM32_LPTIM6SEL == RCC_CCIPR2_LPTIM6SEL_LSI
  #define STM32_LPTIM6CLK           STM32_LSICLK

#elif STM32_LPTIM6SEL == RCC_CCIPR2_LPTIM6SEL_PER
  #define STM32_LPTIM6CLK           STM32_PERCLK

#else
  #error "invalid source selected for LPTIM6 clock"
#endif

/**
 * @brief   SPI1 clock frequency.
 */
#if (STM32_SPI1SEL == RCC_CCIPR3_SPI1SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SPI1CLK             hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SPI1SEL == RCC_CCIPR3_SPI1SEL_PLL2P
  #define STM32_SPI1CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SPI1SEL == RCC_CCIPR3_SPI1SEL_PLL3P
  #define STM32_SPI1CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SPI1SEL == RCC_CCIPR3_SPI1SEL_AUDIOCLK
  #define STM32_SPI1CLK             0 /* TODO board.h */

#elif STM32_SPI1SEL == RCC_CCIPR3_SPI1SEL_PER
  #define STM32_SPI1CLK             STM32_PERCLK

#else
  #error "invalid source selected for SPI1 clock"
#endif

/**
 * @brief   SPI2 clock frequency.
 */
#if (STM32_SPI2SEL == RCC_CCIPR3_SPI2SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SPI2CLK             hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SPI2SEL == RCC_CCIPR3_SPI2SEL_PLL2P
  #define STM32_SPI2CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SPI2SEL == RCC_CCIPR3_SPI2SEL_PLL3P
  #define STM32_SPI2CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SPI2SEL == RCC_CCIPR3_SPI2SEL_AUDIOCLK
  #define STM32_SPI2CLK             0 /* TODO board.h */

#elif STM32_SPI2SEL == RCC_CCIPR3_SPI2SEL_PER
  #define STM32_SPI2CLK             STM32_PERCLK

#else
  #error "invalid source selected for SPI2 clock"
#endif

/**
 * @brief   SPI3 clock frequency.
 */
#if (STM32_SPI3SEL == RCC_CCIPR3_SPI3SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SPI3CLK             hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SPI3SEL == RCC_CCIPR3_SPI3SEL_PLL2P
  #define STM32_SPI3CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SPI3SEL == RCC_CCIPR3_SPI3SEL_PLL3P
  #define STM32_SPI3CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SPI3SEL == RCC_CCIPR3_SPI3SEL_AUDIOCLK
  #define STM32_SPI3CLK             0 /* TODO board.h */

#elif STM32_SPI3SEL == RCC_CCIPR3_SPI3SEL_PER
  #define STM32_SPI3CLK             STM32_PERCLK

#else
  #error "invalid source selected for SPI3 clock"
#endif

/**
 * @brief   SPI4 clock frequency.
 */
#if (STM32_SPI4SEL == RCC_CCIPR3_SPI4SEL_PCLK2) || defined(__DOXYGEN__)
  #define STM32_SPI4CLK             hal_lld_get_clock_point(CLK_PCLK2)

#elif STM32_SPI4SEL == RCC_CCIPR3_SPI4SEL_PLL2P
  #define STM32_SPI4CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SPI4SEL == RCC_CCIPR3_SPI4SEL_PLL3P
  #define STM32_SPI4CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SPI4SEL == RCC_CCIPR3_SPI4SEL_HSI
  #define STM32_SPI4CLK             hal_lld_get_clock_point(CLK_HSI)

#elif STM32_SPI4SEL == RCC_CCIPR3_SPI4SEL_CSI
  #define STM32_SPI4CLK             hal_lld_get_clock_point(CLK_CSI)

#elif STM32_SPI4SEL == RCC_CCIPR3_SPI4SEL_HSE
  #define STM32_SPI4CLK             hal_lld_get_clock_point(CLK_HSE)

#else
  #error "invalid source selected for SPI4 clock"
#endif

/**
 * @brief   SPI5 clock frequency.
 */
#if (STM32_SPI5SEL == RCC_CCIPR3_SPI5SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_SPI5CLK             hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_SPI5SEL == RCC_CCIPR3_SPI5SEL_PLL2P
  #define STM32_SPI5CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SPI5SEL == RCC_CCIPR3_SPI5SEL_PLL3P
  #define STM32_SPI5CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SPI5SEL == RCC_CCIPR3_SPI5SEL_HSI
  #define STM32_SPI5CLK             hal_lld_get_clock_point(CLK_HSI)

#elif STM32_SPI5SEL == RCC_CCIPR3_SPI5SEL_CSI
  #define STM32_SPI5CLK             hal_lld_get_clock_point(CLK_CSI)

#elif STM32_SPI5SEL == RCC_CCIPR3_SPI5SEL_HSE
  #define STM32_SPI5CLK             hal_lld_get_clock_point(CLK_HSE)

#else
  #error "invalid source selected for SPI5 clock"
#endif

/**
 * @brief   SPI6 clock frequency.
 */
#if (STM32_SPI6SEL == RCC_CCIPR3_SPI6SEL_PCLK2) || defined(__DOXYGEN__)
  #define STM32_SPI6CLK             hal_lld_get_clock_point(CLK_PCLK2)

#elif STM32_SPI6SEL == RCC_CCIPR3_SPI6SEL_PLL2P
  #define STM32_SPI6CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SPI6SEL == RCC_CCIPR3_SPI6SEL_PLL3P
  #define STM32_SPI6CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SPI6SEL == RCC_CCIPR3_SPI6SEL_HSI
  #define STM32_SPI6CLK             hal_lld_get_clock_point(CLK_HSI)

#elif STM32_SPI6SEL == RCC_CCIPR3_SPI6SEL_CSI
  #define STM32_SPI6CLK             hal_lld_get_clock_point(CLK_CSI)

#elif STM32_SPI6SEL == RCC_CCIPR3_SPI6SEL_HSE
  #define STM32_SPI6CLK             hal_lld_get_clock_point(CLK_HSE)

#else
  #error "invalid source selected for SPI6 clock"
#endif

/**
 * @brief   OSPI clock frequency.
 */
#if (STM32_OSPISEL == RCC_CCIPR4_OSPISEL_HCLK4) || defined(__DOXYGEN__)
  #define STM32_OSPICLK             hal_lld_get_clock_point(CLK_HCLK)

#elif STM32_OSPISEL == RCC_CCIPR4_OSPISEL_PLL1Q
  #define STM32_OSPICLK             hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_OSPISEL == RCC_CCIPR4_OSPISEL_PLL2R
  #define STM32_OSPICLK             hal_lld_get_clock_point(CLK_PLL2RCLK)

#elif STM32_OSPISEL == RCC_CCIPR4_OSPISEL_PER
  #define STM32_OSPICLK             STM32_PERCLK

#else
  #error "invalid source selected for OSPI clock"
#endif

/**
 * @brief   SYSTICK clock frequency.
 */
#if (STM32_SYSTICKSEL == RCC_CCIPR4_SYSTICKSEL_HCLKDIV8) || defined(__DOXYGEN__)
  #define STM32_SYSTICKCLK          (hal_lld_get_clock_point(CLK_HCLK) / 8)

#elif STM32_SYSTICKSEL == RCC_CCIPR4_SYSTICKSEL_LSI
  #define STM32_SYSTICKCLK          STM32_LSICLK

#elif STM32_SYSTICKSEL == RCC_CCIPR4_SYSTICKSEL_LSE
  #define STM32_SYSTICKCLK          STM32_LSECLK

#elif STM32_SYSTICKSEL == RCC_CCIPR4_SYSTICKSEL_NOCLOCK
  #define STM32_SYSTICKCLK          0

#else
  #error "invalid source selected for STM32_SYSTICKSEL clock"
#endif

/**
 * @brief   USB clock frequency.
 */
#if (STM32_USBSEL == RCC_CCIPR4_USBSEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_USBCLK              0

#elif STM32_USBSEL == RCC_CCIPR4_USBSEL_PLL1Q
  #define STM32_USBCLK              hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_USBSEL == RCC_CCIPR4_USBSEL_PLL3Q
  #define STM32_USBCLK              hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USBSEL == RCC_CCIPR4_USBSEL_HSI48
  #define STM32_USBCLK              STM32_HSI48CLK

#else
  #error "invalid source selected for USB clock"
#endif

/**
 * @brief   SDMMC1 clock frequency.
 */
#if (STM32_SDMMC1SEL == RCC_CCIPR4_SDMMC1SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SDMMC1CLK           hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SDMMC1SEL == RCC_CCIPR4_SDMMC1SEL_PLL2R
  #define STM32_SDMMC1CLK           hal_lld_get_clock_point(CLK_PLL2RCLK)

#else
  #error "invalid source selected for SDMMC1 clock"
#endif

/**
 * @brief   SDMMC2 clock frequency.
 */
#if (STM32_SDMMC2SEL == RCC_CCIPR4_SDMMC2SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SDMMC2CLK           hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SDMMC2SEL == RCC_CCIPR4_SDMMC2SEL_PLL2R
  #define STM32_SDMMC2CLK           hal_lld_get_clock_point(CLK_PLL2RCLK)

#else
  #error "invalid source selected for SDMMC2 clock"
#endif

/**
 * @brief   I2C1 clock frequency.
 */
#if (STM32_I2C1SEL == RCC_CCIPR4_I2C1SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_I2C1CLK             hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_I2C1SEL == RCC_CCIPR4_I2C1SEL_PLL3R
  #define STM32_I2C1CLK             hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_I2C1SEL == RCC_CCIPR4_I2C1SEL_HSI
  #define STM32_I2C1CLK             hal_lld_get_clock_point(CLK_HSI)

#elif STM32_I2C1SEL == RCC_CCIPR4_I2C1SEL_CSI
  #define STM32_I2C1CLK             hal_lld_get_clock_point(CLK_CSI)

#else
  #error "invalid source selected for I2C1 clock"
#endif

/**
 * @brief   I2C2 clock frequency.
 */
#if (STM32_I2C2SEL == RCC_CCIPR4_I2C2SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_I2C2CLK             hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_I2C2SEL == RCC_CCIPR4_I2C2SEL_PLL3R
  #define STM32_I2C2CLK             hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_I2C2SEL == RCC_CCIPR4_I2C2SEL_HSI
  #define STM32_I2C2CLK             hal_lld_get_clock_point(CLK_HSI)

#elif STM32_I2C2SEL == RCC_CCIPR4_I2C2SEL_CSI
  #define STM32_I2C2CLK             hal_lld_get_clock_point(CLK_CSI)

#else
  #error "invalid source selected for I2C2 clock"
#endif

/**
 * @brief   I2C3 clock frequency.
 */
#if (STM32_I2C3SEL == RCC_CCIPR4_I2C3SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_I2C3CLK             hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_I2C3SEL == RCC_CCIPR4_I2C3SEL_PLL3R
  #define STM32_I2C3CLK             hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_I2C3SEL == RCC_CCIPR4_I2C3SEL_HSI
  #define STM32_I2C3CLK             hal_lld_get_clock_point(CLK_HSI)

#elif STM32_I2C3SEL == RCC_CCIPR4_I2C3SEL_CSI
  #define STM32_I2C3CLK             hal_lld_get_clock_point(CLK_CSI)

#else
  #error "invalid source selected for I2C3 clock"
#endif

/**
 * @brief   I2C4 clock frequency.
 */
#if (STM32_I2C4SEL == RCC_CCIPR4_I2C4SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_I2C4CLK             hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_I2C4SEL == RCC_CCIPR4_I2C4SEL_PLL3R
  #define STM32_I2C4CLK             hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_I2C4SEL == RCC_CCIPR4_I2C4SEL_HSI
  #define STM32_I2C4CLK             hal_lld_get_clock_point(CLK_HSI)

#elif STM32_I2C4SEL == RCC_CCIPR4_I2C4SEL_CSI
  #define STM32_I2C4CLK             hal_lld_get_clock_point(CLK_CSI)

#else
  #error "invalid source selected for I2C4 clock"
#endif

/**
 * @brief   I3C1 clock frequency.
 */
#if (STM32_I3C1SEL == RCC_CCIPR4_I3C1SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_I3C1CLK             hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_I3C1SEL == RCC_CCIPR4_I3C1SEL_PLL3R
  #define STM32_I3C1CLK             hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_I3C1SEL == RCC_CCIPR4_I3C1SEL_HSI
  #define STM32_I3C1CLK             hal_lld_get_clock_point(CLK_HSI)

#else
  #error "invalid source selected for I3C1 clock"
#endif

/**
 * @brief   ADCDACSEL clock frequency.
 */
#if (STM32_ADCDACSEL == RCC_CCIPR5_ADCDACSEL_HCLK) || defined(__DOXYGEN__)
  #define STM32_ADCDACCLK           hal_lld_get_clock_point(CLK_HCLK)

#elif STM32_ADCDACSEL == RCC_CCIPR5_ADCDACSEL_SYSCLK
  #define STM32_ADCDACCLK           hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_ADCDACSEL == RCC_CCIPR5_ADCDACSEL_PLL2R
  #define STM32_ADCDACCLK           hal_lld_get_clock_point(CLK_PLL2RCLK)

#elif STM32_ADCDACSEL == RCC_CCIPR5_ADCDACSEL_HSE
  #define STM32_ADCDACCLK           hal_lld_get_clock_point(CLK_HSE)

#elif STM32_ADCDACSEL == RCC_CCIPR5_ADCDACSEL_HSI
  #define STM32_ADCDACCLK           hal_lld_get_clock_point(CLK_HSI)

#elif STM32_ADCDACSEL == RCC_CCIPR5_ADCDACSEL_CSI
  #define STM32_ADCDACCLK           hal_lld_get_clock_point(CLK_CSI)

#else
  #error "invalid source selected for ADCDACSEL clock"
#endif

/**
 * @brief   DACSEL clock frequency.
 */
#if (STM32_DACSEL == RCC_CCIPR5_DACSEL_LSI) || defined(__DOXYGEN__)
  #define STM32_DACSELCLK           STM32_LSICLK

#elif STM32_DACSEL == RCC_CCIPR5_DACSEL_LSE
  #define STM32_DACSELCLK           STM32_LSECLK

#elif STM32_DACSEL == RCC_CCIPR5_DACSEL_IGNORE
  #define STM32_DACSELCLK           0

#else
  #error "invalid source selected for DACSEL clock"
#endif

/**
 * @brief   RNG clock frequency.
 */
#if (STM32_RNGSEL == RCC_CCIPR5_RNGSEL_HSI48) || defined(__DOXYGEN__)
  #define STM32_RNGCLK              STM32_HSI48CLK

#elif STM32_RNGSEL == RCC_CCIPR5_RNGSEL_PLL1Q
  #define STM32_RNGCLK              hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_RNGSEL == RCC_CCIPR5_RNGSEL_LSE
  #define STM32_RNGCLK              STM32_LSECLK

#elif STM32_RNGSEL == RCC_CCIPR5_RNGSEL_LSI
  #define STM32_RNGCLK              STM32_LSICLK

#elif STM32_RNGSEL == RCC_CCIPR5_RNGSEL_IGNORE
  #define STM32_RNGCLK              0

#else
  #error "invalid source selected for RNG clock"
#endif

/**
 * @brief   CEC clock frequency.
 */
#if (STM32_CECSEL == RCC_CCIPR5_CECSEL_LSE) || defined(__DOXYGEN__)
  #define STM32_CECCLK              STM32_LSECLK

#elif STM32_CECSEL == RCC_CCIPR5_CECSEL_LSI
  #define STM32_CECCLK              STM32_LSICLK

#elif STM32_CECSEL == RCC_CCIPR5_CECSEL_CSIDIV128
  #define STM32_CECCLK              (hal_lld_get_clock_point(CLK_CSI) / 128)

#elif STM32_CECSEL == RCC_CCIPR5_CECSEL_IGNORE
  #define STM32_CECCLK              0

#else
  #error "invalid source selected for CEC clock"
#endif

/**
 * @brief   FDCAN clock frequency.
 */
#if (STM32_FDCANSEL == RCC_CCIPR5_FDCANSEL_HSE) || defined(__DOXYGEN__)
  #define STM32_FDCANCLK            hal_lld_get_clock_point(CLK_HSE)

#elif STM32_FDCANSEL == RCC_CCIPR5_FDCANSEL_PLL1Q
  #define STM32_FDCANCLK            hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_FDCANSEL == RCC_CCIPR5_FDCANSEL_PLL2Q
  #define STM32_FDCANCLK            hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_FDCANSEL == RCC_CCIPR5_FDCANSEL_IGNORE
  #define STM32_FDCANCLK            0

#else
  #error "invalid source selected for FDCAN clock"
#endif

/**
 * @brief   SAI1 clock frequency.
 */
#if (STM32_SAI1SEL == RCC_CCIPR5_SAI1SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SAI1CLK             hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SAI1SEL == RCC_CCIPR5_SAI1SEL_PLL2P
  #define STM32_SAI1CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SAI1SEL == RCC_CCIPR5_SAI1SEL_PLL3P
  #define STM32_SAI1CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SAI1SEL == RCC_CCIPR5_SAI1SEL_AUDIOCLK
  #define STM32_SAI1CLK             0 /* TODO board.h */

#elif STM32_SAI1SEL == RCC_CCIPR5_SAI1SEL_PER
  #define STM32_SAI1CLK             STM32_PERCLK

#else
  #error "invalid source selected for SAI1 clock"
#endif

/**
 * @brief   SAI2 clock frequency.
 */
#if (STM32_SAI2SEL == RCC_CCIPR5_SAI2SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SAI2CLK             hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SAI2SEL == RCC_CCIPR5_SAI2SEL_PLL2P
  #define STM32_SAI2CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SAI2SEL == RCC_CCIPR5_SAI2SEL_PLL3P
  #define STM32_SAI2CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SAI2SEL == RCC_CCIPR5_SAI2SEL_AUDIOCLK
  #define STM32_SAI2CLK             0 /* TODO board.h */

#elif STM32_SAI2SEL == RCC_CCIPR5_SAI2SEL_PER
  #define STM32_SAI2CLK             STM32_PERCLK

#else
  #error "invalid source selected for SAI2 clock"
#endif

/**
 * @brief   CKPER clock frequency.
 */
#if (STM32_CKPERSEL == RCC_CCIPR5_CKPERSEL_HSI) || defined(__DOXYGEN__)
#define STM32_PERCLK                hal_lld_get_clock_point(CLK_HSI)

#elif STM32_CKPERSEL == RCC_CCIPR5_CKPERSEL_CSI
#define STM32_PERCLK                hal_lld_get_clock_point(CLK_CSI)

#elif STM32_CKPERSEL == RCC_CCIPR5_CKPERSEL_HSE
#define STM32_PERCLK                hal_lld_get_clock_point(CLK_HSE)

#else
  #error "invalid source selected for CKPER clock"
#endif

#if (STM32_TIMPRE == RCC_CFGR1_TIMPRE_LOW) || defined(__DOXYGEN__)
  /**
   * @brief   TIMP1CLK clock frequency.
   */
  #if (STM32_PPRE1 == RCC_CFGR2_PPRE1_DIV1) ||                              \
      (STM32_PPRE1 == RCC_CFGR2_PPRE1_DIV2) || defined(__DOXYGEN__)
    #define STM32_TIMP1CLK            STM32_HCLK
  #else
    #define STM32_TIMP1CLK            (STM32_PCLK1 * 2)
  #endif

  /**
   * @brief   TIMP2CLK clock frequency.
   */
  #if (STM32_PPRE2 == RCC_CFGR2_PPRE2_DIV1) ||                              \
      (STM32_PPRE2 == RCC_CFGR2_PPRE2_DIV2) || defined(__DOXYGEN__)
    #define STM32_TIMP2CLK            STM32_HCLK
  #else
    #define STM32_TIMP2CLK            (STM32_PCLK2 * 2)
  #endif

#else
  #if (STM32_PPRE1 == RCC_CFGR2_PPRE1_DIV1) ||                              \
      (STM32_PPRE1 == RCC_CFGR2_PPRE1_DIV2) ||                              \
      (STM32_PPRE1 == RCC_CFGR2_PPRE1_DIV4) || defined(__DOXYGEN__)
    #define STM32_TIMP1CLK            (STM32_PCLK1 * 2)
  #else
    #define STM32_TIMP1CLK            (STM32_PCLK1 * 4)
  #endif

  #if (STM32_PPRE2 == RCC_CFGR2_PPRE2_DIV1) ||                              \
      (STM32_PPRE2 == RCC_CFGR2_PPRE2_DIV2) ||                              \
      (STM32_PPRE2 == RCC_CFGR2_PPRE2_DIV4) || defined(__DOXYGEN__)
    #define STM32_TIMP2CLK            (STM32_PCLK2 * 2)
  #else
    #define STM32_TIMP2CLK            (STM32_PCLK2 * 4)
  #endif
#endif

/**
 * @brief   ETH clock frequency.
 */
#define STM32_ETHCLK               hal_lld_get_clock_point(CLK_PLL1QCLK)

/**
 * @brief   Clock of timers connected to APB1.
 */
#define STM32_TIMCLK1               hal_lld_get_clock_point(CLK_PCLK1TIM)

/**
 * @brief   Clock of timers connected to APB2.
 */
#define STM32_TIMCLK2               hal_lld_get_clock_point(CLK_PCLK2TIM)

/**
 * @brief   Flash settings.
 */
#if (STM32_HCLK <= STM32_0WS_THRESHOLD) || defined(__DOXYGEN__)
  #define STM32_FLASHBITS           (FLASH_ACR_LATENCY_0WS | 0U                     | 0U)

#elif STM32_HCLK <= STM32_1WS_THRESHOLD
  #define STM32_FLASHBITS           (FLASH_ACR_LATENCY_1WS | 0U)                    | FLASH_ACR_PRFTEN

#elif STM32_HCLK <= STM32_2WS_THRESHOLD
  #define STM32_FLASHBITS           (FLASH_ACR_LATENCY_2WS | FLASH_ACR_WRHIGHFREQ_0 | FLASH_ACR_PRFTEN)

#elif STM32_HCLK <= STM32_3WS_THRESHOLD
  #define STM32_FLASHBITS           (FLASH_ACR_LATENCY_3WS | FLASH_ACR_WRHIGHFREQ_0 | FLASH_ACR_PRFTEN)

#elif STM32_HCLK <= STM32_4WS_THRESHOLD
  #define STM32_FLASHBITS           (FLASH_ACR_LATENCY_4WS | FLASH_ACR_WRHIGHFREQ_1 | FLASH_ACR_PRFTEN)

#else
  #define STM32_FLASHBITS           (FLASH_ACR_LATENCY_5WS | FLASH_ACR_WRHIGHFREQ_1 | FLASH_ACR_PRFTEN)
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of PLL configuration registers.
 */
typedef struct {
  uint32_t          cfgr;
  uint32_t          divr;
  uint32_t          frac;
} stm32_pll_regs_t;

/**
 * @brief   Type of a clock configuration structure.
 */
typedef struct {
  uint32_t          pwr_voscr;
  uint32_t          rcc_cr;
  uint32_t          rcc_cfgr1;
  uint32_t          rcc_cfgr2;
  uint32_t          flash_acr;
#if STM32_RCC_HAS_PLL3 || defined(__DOXYGEN__)
  stm32_pll_regs_t  plls[3];
#else
  stm32_pll_regs_t  plls[2];
#endif
} halclkcfg_t;

/**
 * @brief   Type of a timeout counter.
 */
typedef uint32_t halcnt_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Real time counter frequency exported to the safety module.
 * @note    The counter is the internal DWS cycles counter so in runs at
 *          the same frequency of CPU.
 */
#define HAL_LLD_GET_CNT_FREQUENCY()         SystemCoreClock

/**
 * @brief   Real time counter value exported to the safety module.
 */
#define HAL_LLD_GET_CNT_VALUE()             (DWT->CYCCNT)

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
#if STM32_RCC_HAS_PLL3 || defined(__DOXYGEN__)
#define hal_lld_get_clock_point(clkpt)                                      \
  ((clkpt) == CLK_SYSCLK    ? STM32_SYSCLK          :                       \
   (clkpt) == CLK_PLL1PCLK  ? STM32_PLL1_P_CLKOUT   :                       \
   (clkpt) == CLK_PLL1QCLK  ? STM32_PLL1_Q_CLKOUT   :                       \
   (clkpt) == CLK_PLL1RCLK  ? STM32_PLL1_R_CLKOUT   :                       \
   (clkpt) == CLK_PLL2PCLK  ? STM32_PLL2_P_CLKOUT   :                       \
   (clkpt) == CLK_PLL2QCLK  ? STM32_PLL2_Q_CLKOUT   :                       \
   (clkpt) == CLK_PLL2RCLK  ? STM32_PLL2_R_CLKOUT   :                       \
   (clkpt) == CLK_PLL3PCLK  ? STM32_PLL3_P_CLKOUT   :                       \
   (clkpt) == CLK_PLL3QCLK  ? STM32_PLL3_Q_CLKOUT   :                       \
   (clkpt) == CLK_PLL3RCLK  ? STM32_PLL3_R_CLKOUT   :                       \
   (clkpt) == CLK_HCLK      ? STM32_HCLK            :                       \
   (clkpt) == CLK_PCLK1     ? STM32_PCLK1           :                       \
   (clkpt) == CLK_PCLK1TIM  ? STM32_TIMP1CLK        :                       \
   (clkpt) == CLK_PCLK2     ? STM32_PCLK2           :                       \
   (clkpt) == CLK_PCLK2TIM  ? STM32_TIMP2CLK        :                       \
   (clkpt) == CLK_PCLK3     ? STM32_PCLK3           :                       \
   (clkpt) == CLK_MCO1      ? STM32_MCO1CLK         :                       \
   (clkpt) == CLK_MCO2      ? STM32_MCO2CLK         :                       \
   (clkpt) == CLK_HSI48     ? STM32_HSI48CLK        :                       \
   0U)
#else
#define hal_lld_get_clock_point(clkpt)                                      \
  ((clkpt) == CLK_SYSCLK    ? STM32_SYSCLK          :                       \
   (clkpt) == CLK_PLL1PCLK  ? STM32_PLL1_P_CLKOUT   :                       \
   (clkpt) == CLK_PLL1QCLK  ? STM32_PLL1_Q_CLKOUT   :                       \
   (clkpt) == CLK_PLL1RCLK  ? STM32_PLL1_R_CLKOUT   :                       \
   (clkpt) == CLK_PLL2PCLK  ? STM32_PLL2_P_CLKOUT   :                       \
   (clkpt) == CLK_PLL2QCLK  ? STM32_PLL2_Q_CLKOUT   :                       \
   (clkpt) == CLK_PLL2RCLK  ? STM32_PLL2_R_CLKOUT   :                       \
   (clkpt) == CLK_HCLK      ? STM32_HCLK            :                       \
   (clkpt) == CLK_PCLK1     ? STM32_PCLK1           :                       \
   (clkpt) == CLK_PCLK1TIM  ? STM32_TIMP1CLK        :                       \
   (clkpt) == CLK_PCLK2     ? STM32_PCLK2           :                       \
   (clkpt) == CLK_PCLK2TIM  ? STM32_TIMP2CLK        :                       \
   (clkpt) == CLK_PCLK3     ? STM32_PCLK3           :                       \
   (clkpt) == CLK_MCO1      ? STM32_MCO1CLK         :                       \
   (clkpt) == CLK_MCO2      ? STM32_MCO2CLK         :                       \
   (clkpt) == CLK_HSI48     ? STM32_HSI48CLK        :                       \
   0U)
#endif
#endif /* !defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
//#include "mpu_v8m.h"
#include "stm32_isr.h"
#include "stm32_dma3.h"
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
