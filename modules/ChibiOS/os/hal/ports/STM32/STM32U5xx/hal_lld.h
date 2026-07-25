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
 * @file    STM32U5xx/hal_lld.h
 * @brief   STM32U5xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32U535xx, STM32U545xx.
 *          - STM32H575xx, STM32H585xx.
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
#if defined(STM32U535xx) || defined(__DOXYGEN__)
  #define PLATFORM_NAME         "STM32U5 Ultra low power"

#elif defined(STM32U535xx)
  #define PLATFORM_NAME         "STM32U5 Ultra low power with security"

#elif defined(STM32H575xx)
  #define PLATFORM_NAME         "STM32U5 Ultra low power"

#elif defined(STM32H585xx)
  #define PLATFORM_NAME         "STM32U5 Ultra low power with security"

#else
  #error "STM32U5 device not specified"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(STM32U5XX) || defined(__DOXYGEN__)
  #define STM32U5XX
#endif
/** @} */

/**
 * @name   Clock points names
 * @{
 */
#define CLK_HSI                 0U
#define CLK_MSIS                1U
#define CLK_MSIK                2U
#define CLK_HSI48               3U
#define CLK_HSE                 4U
#define CLK_SYSCLK              5U
#define CLK_PLL1PCLK            6U
#define CLK_PLL1QCLK            7U
#define CLK_PLL1RCLK            8U
#define CLK_PLL2PCLK            9U
#define CLK_PLL2QCLK            10U
#define CLK_PLL2RCLK            11U
#define CLK_PLL3PCLK            12U
#define CLK_PLL3QCLK            13U
#define CLK_PLL3RCLK            14U
#define CLK_HCLK                15U
#define CLK_PCLK1               16U
#define CLK_PCLK1TIM            17U
#define CLK_PCLK2               18U
#define CLK_PCLK2TIM            19U
#define CLK_PCLK3               20U
#define CLK_MCO1                21U
#define CLK_MCO2                22U
#define CLK_ARRAY_SIZE          23U
/** @} */

/**
 * @name    VOS field definitions
 * @{
 */
#define STM32_VOS_POS           PWR_VOSR_VOS_Pos
#define STM32_VOS_MASK          PWR_VOSR_VOS_Msk
#define STM32_VOS_RANGE4        (0U << STM32_VOS_POS)   /**< 0.99 Volt.     */
#define STM32_VOS_RANGE3        (1U << STM32_VOS_POS)   /**< 1.05 Volts.    */
#define STM32_VOS_RANGE2        (2U << STM32_VOS_POS)   /**< 1.15 Volts.    */
#define STM32_VOS_RANGE1        (3U << STM32_VOS_POS)   /**< 1.27 Volts.    */
/** @} */
/*-------------------------------------------------------------------------------*/
/**
 * @name    RCC_CR register bits definitions
 * @{
 */
#define STM32_PLL3ON_POS        RCC_CR_PLL3ON_Pos
#define STM32_PLL3ON_MASK       RCC_CR_PLL3ON_Msk
#define STM32_PLL3ON            (1U << STM32_PLL3ON_POS)

#define STM32_PLL2ON_POS        RCC_CR_PLL2ON_Pos
#define STM32_PLL2ON_MASK       RCC_CR_PLL2ON_Msk
#define STM32_PLL2ON            (1U << STM32_PLL2ON_POS)

#define STM32_PLL1ON_POS        RCC_CR_PLL1ON_Pos
#define STM32_PLL1ON_MASK       RCC_CR_PLL1ON_Msk
#define STM32_PLL1ON            (1U << STM32_PLL1ON_POS)

#define STM32_HSERDY_POS        RCC_CR_HSERDY_Pos
#define STM32_HSERDY_MASK       RCC_CR_HSERDY_Msk
#define STM32_HSERDY            (1U << STM32_HSERDY_POS)

#define STM32_HSEON_POS         RCC_CR_HSEON_Pos
#define STM32_HSEON_MASK        RCC_CR_HSEON_Msk
#define STM32_HSEON             (1U << STM32_HSEON_POS)

#define STM32_HSI48RDY_POS      RCC_CR_HSI48RDY_Pos
#define STM32_HSI48RDY_MASK     RCC_CR_HSI48RDY_Msk
#define STM32_HSI48RDY          (1U << STM32_HSI48RDY_POS)

#define STM32_HSI48ON_POS       RCC_CR_HSI48ON_Pos
#define STM32_HSI48ON_MASK      RCC_CR_HSI48ON_Msk
#define STM32_HSI48ON           (1U << STM32_HSI48ON_POS)

#define STM32_CSIRDY_POS        RCC_CR_CSIRDY_Pos
#define STM32_CSIRDY_MASK       RCC_CR_CSIRDY_Msk
#define STM32_CSIRDY            (1U << STM32_CSIRDY_POS)

#define STM32_CSION_POS         RCC_CR_CSION_Pos
#define STM32_CSION_MASK        RCC_CR_CSION_Msk
#define STM32_CSION             (1U << STM32_CSION_POS)

#define STM32_HSIRDY_POS        RCC_CR_HSIRDY_Pos
#define STM32_HSIRDY_MASK       RCC_CR_HSIRDY_Msk
#define STM32_HSIRDY            (1U << STM32_HSIRDY_POS)

#define STM32_HSION_POS         RCC_CR_HSION_Pos
#define STM32_HSION_MASK        RCC_CR_HSION_Msk
#define STM32_HSION             (1U << STM32_HSION_POS)

#define STM32_HSIDIVF_POS       RCC_CR_HSIDIVF_Pos
#define STM32_HSIDIVF_MASK      RCC_CR_HSIDIVF_Msk
#define STM32_HSIDIVF           (1U << STM32_HSIDIVF_POS)

#define STM32_HSIDIV_POS        RCC_CR_HSIDIV_Pos
#define STM32_HSIDIV_MASK       RCC_CR_HSIDIV_Msk
#define STM32_HSIDIV_FIELD(n)   ((n) << STM32_HSIDIV_POS)
#define STM32_HSIDIV_DIV1       STM32_HSIDIV_FIELD(0U)
#define STM32_HSIDIV_DIV2       STM32_HSIDIV_FIELD(1U)
#define STM32_HSIDIV_DIV4       STM32_HSIDIV_FIELD(2U)
#define STM32_HSIDIV_DIV8       STM32_HSIDIV_FIELD(3U)
/** @} */

/**
 * @name    RCC_CFGR1 register bits definitions
 * @{
 */
#define STM32_SW_POS            RCC_CFGR1_SW_Pos
#define STM32_SW_MASK           RCC_CFGR1_SW_Msk
#define STM32_SW_FIELD(n)       ((n) << STM32_SW_POS)
#define STM32_SW_HSI            STM32_SW_FIELD(0U)
#define STM32_SW_CSI            STM32_SW_FIELD(1U)
#define STM32_SW_HSE            STM32_SW_FIELD(2U)
#define STM32_SW_PLL1P          STM32_SW_FIELD(3U)

#define STM32_SWS_POS           RCC_CFGR1_SWS_Pos
#define STM32_SWS_MASK          RCC_CFGR1_SWS_Msk
#define STM32_SWS_FIELD(n)      ((n) << STM32_SWS_POS)
#define STM32_SWS_HSI           STM32_SWS_FIELD(0U)
#define STM32_SWS_CSI           STM32_SWS_FIELD(1U)
#define STM32_SWS_HSE           STM32_SWS_FIELD(2U)
#define STM32_SWS_PLL1P         STM32_SWS_FIELD(3U)

#define STM32_STOPWUCK_MASK     (1U << 6)
#define STM32_STOPWUCK_FIELD(n) ((n) << 6)
#define STM32_STOPWUCK_HSI      STM32_STOPWUCK_FIELD(0U)
#define STM32_STOPWUCK_CSI      STM32_STOPWUCK_FIELD(1U)

#define STM32_STOPKERWUCK_MASK  (1U << 7)
#define STM32_STOPKERWUCK_FIELD(n) ((n) << 7)
#define STM32_STOPKERWUCK_HSI   STM32_STOPKERWUCK_FIELD(0U)
#define STM32_STOPKERWUCK_CSI   STM32_STOPKERWUCK_FIELD(1U)

#define STM32_RTCPRE_MASK       (63U << 8)
#define STM32_RTCPRE_FIELD(n)   ((n) << 8)
#define STM32_RTCPRE_NOCLOCK    STM32_RTCPRE_FIELD(0U)

#define STM32_TIMPRE_MASK       (1U << 15)
#define STM32_TIMPRE_FIELD(n)   ((n) << 15)
#define STM32_TIMPRE_LOW        STM32_TIMPRE_FIELD(0U)
#define STM32_TIMPRE_HIGH       STM32_TIMPRE_FIELD(1U)

#define STM32_MCO1SEL_POS       RCC_CFGR1_MCO1SEL_Pos
#define STM32_MCO1SEL_MASK      RCC_CFGR1_MCO1SEL_Msk
#define STM32_MCO1SEL_FIELD(n)  ((n) << STM32_MCO1SEL_POS)
#define STM32_MCO1SEL_HSI       STM32_MCO1PRE_FIELD(0U)
#define STM32_MCO1SEL_LSE       STM32_MCO1PRE_FIELD(1U)
#define STM32_MCO1SEL_HSE       STM32_MCO1PRE_FIELD(2U)
#define STM32_MCO1SEL_PLL1P     STM32_MCO1PRE_FIELD(3U)
#define STM32_MCO1SEL_HSI48     STM32_MCO1PRE_FIELD(4U)

#define STM32_MCO1PRE_POS       RCC_CFGR1_MCO1PRE_Pos
#define STM32_MCO1PRE_MASK      RCC_CFGR1_MCO1PRE_Msk
#define STM32_MCO1PRE_FIELD(n)  ((n) << STM32_MCO1PRE_POS)
#define STM32_MCO1PRE_NOCLOCK   STM32_MCO1PRE_FIELD(0U)

#define STM32_MCO2PRE_POS       RCC_CFGR1_MCO2PRE_Pos
#define STM32_MCO2SEL_MASK      RCC_CFGR1_MCO2SEL_Msk
#define STM32_MCO2SEL_FIELD(n)  ((n) << STM32_MCO2PRE_POS)
#define STM32_MCO2SEL_SYSCLK    STM32_MCO2PRE_FIELD(0U)
#define STM32_MCO2SEL_PLL2P     STM32_MCO2PRE_FIELD(1U)
#define STM32_MCO2SEL_HSE       STM32_MCO2PRE_FIELD(2U)
#define STM32_MCO2SEL_PLL1P     STM32_MCO2PRE_FIELD(3U)
#define STM32_MCO2SEL_CSI       STM32_MCO2PRE_FIELD(4U)
#define STM32_MCO2SEL_LSI       STM32_MCO2PRE_FIELD(5U)

#define STM32_MCO2PRE_MASK      (7U << 18)
#define STM32_MCO2PRE_FIELD(n)  ((n) << 18)
#define STM32_MCO2PRE_NOCLOCK   STM32_MCO2PRE_FIELD(0U)
/** @} */

/**
 * @name    RCC_CFGR2 register bits definitions
 * @{
 */
#define STM32_APB3DIS_POS       RCC_CFGR2_APB3DIS_Pos
#define STM32_APB3DIS_MASK      RCC_CFGR2_APB3DIS_Msk
#define STM32_APB3DIS           (1U << STM32_APB3DIS_POS)

#define STM32_APB2DIS_POS       RCC_CFGR2_APB2DIS_Pos
#define STM32_APB2DIS_MASK      RCC_CFGR2_APB2DIS_Msk
#define STM32_APB2DIS           (1U << STM32_APB2DIS_POS)

#define STM32_APB1DIS_POS       RCC_CFGR2_APB1DIS_Pos
#define STM32_APB1DIS_MASK      RCC_CFGR2_APB1DIS_Msk
#define STM32_APB1DIS           (1U << STM32_APB1DIS_POS)

#define STM32_AHB2DIS_POS       RCC_CFGR2_AHB2DIS_Pos
#define STM32_AHB2DIS_MASK      RCC_CFGR2_AHB2DIS_Msk
#define STM32_AHB2DIS           (1U << STM32_AHB2DIS_POS)

#define STM32_AHB1DIS_POS       RCC_CFGR2_AHB1DIS_Pos
#define STM32_AHB1DIS_MASK      RCC_CFGR2_AHB1DIS_Msk
#define STM32_AHB1DIS           (1U << STM32_AHB1DIS_POS)
/** @} */

/**
 * @name    RCC_PLLxCFGR register bits definitions
 * @{
 */
#define STM32_PLLREN_POS        18
#define STM32_PLLREN            (1U << STM32_PLLREN_POS)

#define STM32_PLLQEN_POS        17
#define STM32_PLLQEN            (1U << STM32_PLLQEN_POS)

#define STM32_PLLPEN_POS        16
#define STM32_PLLPEN            (1U << STM32_PLLPEN_POS)

#define STM32_PLLM_POS          8
#define STM32_PLLM_MASK         (0x3FU << STM32_PLLM_POS)
#define STM32_PLLM_FIELD(n)     ((n) << STM32_PLLM_POS)

#define STM32_PLLVCOSEL_POS     5
#define STM32_PLLVCOSEL_MASK    (1U << STM32_PLLVCOSEL_POS)
#define STM32_PLLVCOSEL         (1U << STM32_PLLVCOSEL_POS)

#define STM32_PLLFRACEN_POS     4
#define STM32_PLLFRACEN_MASK    (1U << STM32_PLLFRACEN_POS)
#define STM32_PLLFRACEN         (1U << STM32_PLLFRACEN_POS)

#define STM32_PLLRGE_POS        2
#define STM32_PLLRGE_MASK       (3U << STM32_PLLRGE_POS)
#define STM32_PLLRGE_FIELD(n)   ((n) << STM32_PLLRGE_POS)

#define STM32_PLLSRC_POS        0
#define STM32_PLLSRC_MASK       (3U << STM32_PLLSRC_POS)
#define STM32_PLLSRC_FIELD(n)   ((n) << STM32_PLLSRC_POS)
#define STM32_PLLSRC_NOCLOCK    STM32_PLLSRC_FIELD(0U)
#define STM32_PLLSRC_HSI        STM32_PLLSRC_FIELD(1U)
#define STM32_PLLSRC_CSI        STM32_PLLSRC_FIELD(2U)
#define STM32_PLLSRC_HSE        STM32_PLLSRC_FIELD(3U)

#define STM32_PLL1SRC_NOCLOCK   STM32_PLLSRC_NOCLOCK
#define STM32_PLL1SRC_HSI       STM32_PLLSRC_HSI
#define STM32_PLL1SRC_CSI       STM32_PLLSRC_CSI
#define STM32_PLL1SRC_HSE       STM32_PLLSRC_HSE

#define STM32_PLL2SRC_NOCLOCK   STM32_PLLSRC_NOCLOCK
#define STM32_PLL2SRC_HSI       STM32_PLLSRC_HSI
#define STM32_PLL2SRC_CSI       STM32_PLLSRC_CSI
#define STM32_PLL2SRC_HSE       STM32_PLLSRC_HSE

#define STM32_PLL3SRC_NOCLOCK   STM32_PLLSRC_NOCLOCK
#define STM32_PLL3SRC_HSI       STM32_PLLSRC_HSI
#define STM32_PLL3SRC_CSI       STM32_PLLSRC_CSI
#define STM32_PLL3SRC_HSE       STM32_PLLSRC_HSE
/** @} */

/**
 * @name    RCC_PLLxDIVR register bits definitions
 * @{
 */
#define STM32_PLLR_POS          24
#define STM32_PLLR_MASK         (0x7FU << STM32_PLLR_POS)
#define STM32_PLLR_FIELD(n)     ((n) << STM32_PLLR_POS)

#define STM32_PLLQ_POS          16
#define STM32_PLLQ_MASK         (0x7FU << STM32_PLLQ_POS)
#define STM32_PLLQ_FIELD(n)     ((n) << STM32_PLLQ_POS)

#define STM32_PLLP_POS          9
#define STM32_PLLP_MASK         (0x7FU << STM32_PLLP_POS)
#define STM32_PLLP_FIELD(n)     ((n) << STM32_PLLP_POS)

#define STM32_PLLN_POS          0
#define STM32_PLLN_MASK         (0x1FFU << STM32_PLLN_POS)
#define STM32_PLLN_FIELD(n)     ((n) << STM32_PLLN_POS)
/** @} */

/**
 * @name    RCC_CCIPR1 register bits definitions
 * @{
 */
#define STM32_USART1SEL_MASK    (7U << 0)
#define STM32_USART1SEL_FIELD(n) ((n) << 0)
#define STM32_USART1SEL_PCLK2   STM32_USART1SEL_FIELD(0U)
#define STM32_USART1SEL_PLL2Q   STM32_USART1SEL_FIELD(1U)
#define STM32_USART1SEL_PLL3Q   STM32_USART1SEL_FIELD(2U)
#define STM32_USART1SEL_HSI     STM32_USART1SEL_FIELD(3U)
#define STM32_USART1SEL_CSI     STM32_USART1SEL_FIELD(4U)
#define STM32_USART1SEL_LSE     STM32_USART1SEL_FIELD(5U)

#define STM32_USART2SEL_MASK    (7U << 3)
#define STM32_USART2SEL_FIELD(n) ((n) << 3)
#define STM32_USART2SEL_PCLK1   STM32_USART2SEL_FIELD(0U)
#define STM32_USART2SEL_PLL2Q   STM32_USART2SEL_FIELD(1U)
#define STM32_USART2SEL_PLL3Q   STM32_USART2SEL_FIELD(2U)
#define STM32_USART2SEL_HSI     STM32_USART2SEL_FIELD(3U)
#define STM32_USART2SEL_CSI     STM32_USART2SEL_FIELD(4U)
#define STM32_USART2SEL_LSE     STM32_USART2SEL_FIELD(5U)

#define STM32_USART3SEL_MASK    (7U << 6)
#define STM32_USART3SEL_FIELD(n) ((n) << 6)
#define STM32_USART3SEL_PCLK1   STM32_USART3SEL_FIELD(0U)
#define STM32_USART3SEL_PLL2Q   STM32_USART3SEL_FIELD(1U)
#define STM32_USART3SEL_PLL3Q   STM32_USART3SEL_FIELD(2U)
#define STM32_USART3SEL_HSI     STM32_USART3SEL_FIELD(3U)
#define STM32_USART3SEL_CSI     STM32_USART3SEL_FIELD(4U)
#define STM32_USART3SEL_LSE     STM32_USART3SEL_FIELD(5U)

#define STM32_UART4SEL_MASK     (7U << 9)
#define STM32_UART4SEL_FIELD(n) ((n) << 9)
#define STM32_UART4SEL_PCLK1    STM32_UART4SEL_FIELD(0U)
#define STM32_UART4SEL_PLL2Q    STM32_UART4SEL_FIELD(1U)
#define STM32_UART4SEL_PLL3Q    STM32_UART4SEL_FIELD(2U)
#define STM32_UART4SEL_HSI      STM32_UART4SEL_FIELD(3U)
#define STM32_UART4SEL_CSI      STM32_UART4SEL_FIELD(4U)
#define STM32_UART4SEL_LSE      STM32_UART4SEL_FIELD(5U)

#define STM32_UART5SEL_MASK     (7U << 12)
#define STM32_UART5SEL_FIELD(n) ((n) << 12)
#define STM32_UART5SEL_PCLK1    STM32_UART5SEL_FIELD(0U)
#define STM32_UART5SEL_PLL2Q    STM32_UART5SEL_FIELD(1U)
#define STM32_UART5SEL_PLL3Q    STM32_UART5SEL_FIELD(2U)
#define STM32_UART5SEL_HSI      STM32_UART5SEL_FIELD(3U)
#define STM32_UART5SEL_CSI      STM32_UART5SEL_FIELD(4U)
#define STM32_UART5SEL_LSE      STM32_UART5SEL_FIELD(5U)

#define STM32_USART6SEL_MASK    (7U << 15)
#define STM32_USART6SEL_FIELD(n) ((n) << 15)
#define STM32_USART6SEL_PCLK1   STM32_USART6SEL_FIELD(0U)
#define STM32_USART6SEL_PLL2Q   STM32_USART6SEL_FIELD(1U)
#define STM32_USART6SEL_PLL3Q   STM32_USART6SEL_FIELD(2U)
#define STM32_USART6SEL_HSI     STM32_USART6SEL_FIELD(3U)
#define STM32_USART6SEL_CSI     STM32_USART6SEL_FIELD(4U)
#define STM32_USART6SEL_LSE     STM32_USART6SEL_FIELD(5U)

#define STM32_UART7SEL_MASK     (7U << 18)
#define STM32_UART7SEL_FIELD(n) ((n) << 18)
#define STM32_UART7SEL_PCLK1    STM32_UART7SEL_FIELD(0U)
#define STM32_UART7SEL_PLL2Q    STM32_UART7SEL_FIELD(1U)
#define STM32_UART7SEL_PLL3Q    STM32_UART7SEL_FIELD(2U)
#define STM32_UART7SEL_HSI      STM32_UART7SEL_FIELD(3U)
#define STM32_UART7SEL_CSI      STM32_UART7SEL_FIELD(4U)
#define STM32_UART7SEL_LSE      STM32_UART7SEL_FIELD(5U)

#define STM32_UART8SEL_MASK     (7U << 21)
#define STM32_UART8SEL_FIELD(n) ((n) << 21)
#define STM32_UART8SEL_PCLK1    STM32_UART8SEL_FIELD(0U)
#define STM32_UART8SEL_PLL2Q    STM32_UART8SEL_FIELD(1U)
#define STM32_UART8SEL_PLL3Q    STM32_UART8SEL_FIELD(2U)
#define STM32_UART8SEL_HSI      STM32_UART8SEL_FIELD(3U)
#define STM32_UART8SEL_CSI      STM32_UART8SEL_FIELD(4U)
#define STM32_UART8SEL_LSE      STM32_UART8SEL_FIELD(5U)

#define STM32_UART9SEL_MASK     (7U << 24)
#define STM32_UART9SEL_FIELD(n) ((n) << 24)
#define STM32_UART9SEL_PCLK1    STM32_UART9SEL_FIELD(0U)
#define STM32_UART9SEL_PLL2Q    STM32_UART9SEL_FIELD(1U)
#define STM32_UART9SEL_PLL3Q    STM32_UART9SEL_FIELD(2U)
#define STM32_UART9SEL_HSI      STM32_UART9SEL_FIELD(3U)
#define STM32_UART9SEL_CSI      STM32_UART9SEL_FIELD(4U)
#define STM32_UART9SEL_LSE      STM32_UART9SEL_FIELD(5U)

#define STM32_USART10SEL_MASK   (7U << 27)
#define STM32_USART10SEL_FIELD(n) ((n) << 27)
#define STM32_USART10SEL_PCLK1  STM32_USART10SEL_FIELD(0U)
#define STM32_USART10SEL_PLL2Q  STM32_USART10SEL_FIELD(1U)
#define STM32_USART10SEL_PLL3Q  STM32_USART10SEL_FIELD(2U)
#define STM32_USART10SEL_HSI    STM32_USART10SEL_FIELD(3U)
#define STM32_USART10SEL_CSI    STM32_USART10SEL_FIELD(4U)
#define STM32_USART10SEL_LSE    STM32_USART10SEL_FIELD(5U)

#define STM32_TIMICSEL_NOCLK    0U
#define STM32_TIMICSEL_INTCLK   (1U << 31)
/** @} */

/**
 * @name    RCC_CCIPR2 register bits definitions
 * @{
 */
#define STM32_USART11SEL_MASK   (7U << 0)
#define STM32_USART11SEL_FIELD(n) ((n) << 0)
#define STM32_USART11SEL_PCLK1  STM32_USART11SEL_FIELD(0U)
#define STM32_USART11SEL_PLL2Q  STM32_USART11SEL_FIELD(1U)
#define STM32_USART11SEL_PLL3Q  STM32_USART11SEL_FIELD(2U)
#define STM32_USART11SEL_HSI    STM32_USART11SEL_FIELD(3U)
#define STM32_USART11SEL_CSI    STM32_USART11SEL_FIELD(4U)
#define STM32_USART11SEL_LSE    STM32_USART11SEL_FIELD(5U)

#define STM32_UART12SEL_MASK    (7U << 4)
#define STM32_UART12SEL_FIELD(n) ((n) << 4)
#define STM32_UART12SEL_PCLK1   STM32_UART12SEL_FIELD(0U)
#define STM32_UART12SEL_PLL2Q   STM32_UART12SEL_FIELD(1U)
#define STM32_UART12SEL_PLL3Q   STM32_UART12SEL_FIELD(2U)
#define STM32_UART12SEL_HSI     STM32_UART12SEL_FIELD(3U)
#define STM32_UART12SEL_CSI     STM32_UART12SEL_FIELD(4U)
#define STM32_UART12SEL_LSE     STM32_UART12SEL_FIELD(5U)

#define STM32_LPTIM1SEL_MASK    (7U << 8)
#define STM32_LPTIM1SEL_FIELD(n) ((n) << 8)
#define STM32_LPTIM1SEL_PCLK3   STM32_LPTIM1SEL_FIELD(0U)
#define STM32_LPTIM1SEL_PLL2P   STM32_LPTIM1SEL_FIELD(1U)
#define STM32_LPTIM1SEL_PLL3R   STM32_LPTIM1SEL_FIELD(2U)
#define STM32_LPTIM1SEL_LSE     STM32_LPTIM1SEL_FIELD(3U)
#define STM32_LPTIM1SEL_LSI     STM32_LPTIM1SEL_FIELD(4U)
#define STM32_LPTIM1SEL_PER     STM32_LPTIM1SEL_FIELD(5U)

#define STM32_LPTIM2SEL_MASK    (7U << 12)
#define STM32_LPTIM2SEL_FIELD(n) ((n) << 12)
#define STM32_LPTIM2SEL_PCLK1   STM32_LPTIM2SEL_FIELD(0U)
#define STM32_LPTIM2SEL_PLL2P   STM32_LPTIM2SEL_FIELD(1U)
#define STM32_LPTIM2SEL_PLL3R   STM32_LPTIM2SEL_FIELD(2U)
#define STM32_LPTIM2SEL_LSE     STM32_LPTIM2SEL_FIELD(3U)
#define STM32_LPTIM2SEL_LSI     STM32_LPTIM2SEL_FIELD(4U)
#define STM32_LPTIM2SEL_PER     STM32_LPTIM2SEL_FIELD(5U)

#define STM32_LPTIM3SEL_MASK    (7U << 16)
#define STM32_LPTIM3SEL_FIELD(n) ((n) << 16)
#define STM32_LPTIM3SEL_PCLK3   STM32_LPTIM3SEL_FIELD(0U)
#define STM32_LPTIM3SEL_PLL2P   STM32_LPTIM3SEL_FIELD(1U)
#define STM32_LPTIM3SEL_PLL3R   STM32_LPTIM3SEL_FIELD(2U)
#define STM32_LPTIM3SEL_LSE     STM32_LPTIM3SEL_FIELD(3U)
#define STM32_LPTIM3SEL_LSI     STM32_LPTIM3SEL_FIELD(4U)
#define STM32_LPTIM3SEL_PER     STM32_LPTIM3SEL_FIELD(5U)

#define STM32_LPTIM4SEL_MASK    (7U << 20)
#define STM32_LPTIM4SEL_FIELD(n) ((n) << 20)
#define STM32_LPTIM4SEL_PCLK3   STM32_LPTIM4SEL_FIELD(0U)
#define STM32_LPTIM4SEL_PLL2P   STM32_LPTIM4SEL_FIELD(1U)
#define STM32_LPTIM4SEL_PLL3R   STM32_LPTIM4SEL_FIELD(2U)
#define STM32_LPTIM4SEL_LSE     STM32_LPTIM4SEL_FIELD(3U)
#define STM32_LPTIM4SEL_LSI     STM32_LPTIM4SEL_FIELD(4U)
#define STM32_LPTIM4SEL_PER     STM32_LPTIM4SEL_FIELD(5U)

#define STM32_LPTIM5SEL_MASK    (7U << 24)
#define STM32_LPTIM5SEL_FIELD(n) ((n) << 24)
#define STM32_LPTIM5SEL_PCLK3   STM32_LPTIM5SEL_FIELD(0U)
#define STM32_LPTIM5SEL_PLL2P   STM32_LPTIM5SEL_FIELD(1U)
#define STM32_LPTIM5SEL_PLL3R   STM32_LPTIM5SEL_FIELD(2U)
#define STM32_LPTIM5SEL_LSE     STM32_LPTIM5SEL_FIELD(3U)
#define STM32_LPTIM5SEL_LSI     STM32_LPTIM5SEL_FIELD(4U)
#define STM32_LPTIM5SEL_PER     STM32_LPTIM5SEL_FIELD(5U)

#define STM32_LPTIM6SEL_MASK    (7U << 28)
#define STM32_LPTIM6SEL_FIELD(n) ((n) << 28)
#define STM32_LPTIM6SEL_PCLK3   STM32_LPTIM6SEL_FIELD(0U)
#define STM32_LPTIM6SEL_PLL2P   STM32_LPTIM6SEL_FIELD(1U)
#define STM32_LPTIM6SEL_PLL3R   STM32_LPTIM6SEL_FIELD(2U)
#define STM32_LPTIM6SEL_LSE     STM32_LPTIM6SEL_FIELD(3U)
#define STM32_LPTIM6SEL_LSI     STM32_LPTIM6SEL_FIELD(4U)
#define STM32_LPTIM6SEL_PER     STM32_LPTIM6SEL_FIELD(5U)
/** @} */

/**
 * @name    RCC_CCIPR3 register bits definitions
 * @{
 */
#define STM32_SPI1SEL_MASK      (7U << 0)
#define STM32_SPI1SEL_FIELD(n)  ((n) << 0)
#define STM32_SPI1SEL_PLL1Q     STM32_SPI1SEL_FIELD(0U)
#define STM32_SPI1SEL_PLL2P     STM32_SPI1SEL_FIELD(1U)
#define STM32_SPI1SEL_PLL3P     STM32_SPI1SEL_FIELD(2U)
#define STM32_SPI1SEL_AUDIOCLK  STM32_SPI1SEL_FIELD(3U)
#define STM32_SPI1SEL_PER       STM32_SPI1SEL_FIELD(4U)

#define STM32_SPI2SEL_MASK      (7U << 3)
#define STM32_SPI2SEL_FIELD(n)  ((n) << 3)
#define STM32_SPI2SEL_PLL1Q     STM32_SPI2SEL_FIELD(0U)
#define STM32_SPI2SEL_PLL2P     STM32_SPI2SEL_FIELD(1U)
#define STM32_SPI2SEL_PLL3P     STM32_SPI2SEL_FIELD(2U)
#define STM32_SPI2SEL_AUDIOCLK  STM32_SPI2SEL_FIELD(3U)
#define STM32_SPI2SEL_PER       STM32_SPI2SEL_FIELD(4U)

#define STM32_SPI3SEL_MASK      (7U << 6)
#define STM32_SPI3SEL_FIELD(n)  ((n) << 6)
#define STM32_SPI3SEL_PLL1Q     STM32_SPI3SEL_FIELD(0U)
#define STM32_SPI3SEL_PLL2P     STM32_SPI3SEL_FIELD(1U)
#define STM32_SPI3SEL_PLL3P     STM32_SPI3SEL_FIELD(2U)
#define STM32_SPI3SEL_AUDIOCLK  STM32_SPI3SEL_FIELD(3U)
#define STM32_SPI3SEL_PER       STM32_SPI3SEL_FIELD(4U)

#define STM32_SPI4SEL_MASK      (7U << 9)
#define STM32_SPI4SEL_FIELD(n)  ((n) << 9)
#define STM32_SPI4SEL_PCLK2     STM32_SPI4SEL_FIELD(0U)
#define STM32_SPI4SEL_PLL2P     STM32_SPI4SEL_FIELD(1U)
#define STM32_SPI4SEL_PLL3P     STM32_SPI4SEL_FIELD(2U)
#define STM32_SPI4SEL_HSI       STM32_SPI4SEL_FIELD(3U)
#define STM32_SPI4SEL_CSI       STM32_SPI4SEL_FIELD(4U)
#define STM32_SPI4SEL_HSE       STM32_SPI4SEL_FIELD(5U)

#define STM32_SPI5SEL_MASK      (7U << 12)
#define STM32_SPI5SEL_FIELD(n)  ((n) << 12)
#define STM32_SPI5SEL_PCLK3     STM32_SPI5SEL_FIELD(0U)
#define STM32_SPI5SEL_PLL2P     STM32_SPI5SEL_FIELD(1U)
#define STM32_SPI5SEL_PLL3P     STM32_SPI5SEL_FIELD(2U)
#define STM32_SPI5SEL_HSI       STM32_SPI5SEL_FIELD(3U)
#define STM32_SPI5SEL_CSI       STM32_SPI5SEL_FIELD(4U)
#define STM32_SPI5SEL_HSE       STM32_SPI5SEL_FIELD(5U)

#define STM32_SPI6SEL_MASK      (7U << 15)
#define STM32_SPI6SEL_FIELD(n)  ((n) << 15)
#define STM32_SPI6SEL_PCLK2     STM32_SPI6SEL_FIELD(0U)
#define STM32_SPI6SEL_PLL2P     STM32_SPI6SEL_FIELD(1U)
#define STM32_SPI6SEL_PLL3P     STM32_SPI6SEL_FIELD(2U)
#define STM32_SPI6SEL_HSI       STM32_SPI6SEL_FIELD(3U)
#define STM32_SPI6SEL_CSI       STM32_SPI6SEL_FIELD(4U)
#define STM32_SPI6SEL_HSE       STM32_SPI6SEL_FIELD(5U)

#define STM32_LPUART1SEL_MASK   (7U << 24)
#define STM32_LPUART1SEL_FIELD(n) ((n) << 24)
#define STM32_LPUART1SEL_PCLK3  STM32_LPUART1SEL_FIELD(0U)
#define STM32_LPUART1SEL_PLL2Q  STM32_LPUART1SEL_FIELD(1U)
#define STM32_LPUART1SEL_PLL3Q  STM32_LPUART1SEL_FIELD(2U)
#define STM32_LPUART1SEL_HSI    STM32_LPUART1SEL_FIELD(3U)
#define STM32_LPUART1SEL_CSI    STM32_LPUART1SEL_FIELD(4U)
#define STM32_LPUART1SEL_LSE    STM32_LPUART1SEL_FIELD(5U)
/** @} */

/**
 * @name    RCC_CCIPR4 register bits definitions
 * @{
 */
#define STM32_OSPISEL_MASK      (3U << 0)
#define STM32_OSPISEL_FIELD(n)  ((n) << 0)
#define STM32_OSPISEL_HCLK4     STM32_OSPISEL_FIELD(0U)
#define STM32_OSPISEL_PLL1Q     STM32_OSPISEL_FIELD(1U)
#define STM32_OSPISEL_PLL2R     STM32_OSPISEL_FIELD(2U)
#define STM32_OSPISEL_PER       STM32_OSPISEL_FIELD(3U)

#define STM32_SYSTICKSEL_MASK   (3U << 2)
#define STM32_SYSTICKSEL_FIELD(n) ((n) << 2)
#define STM32_SYSTICKSEL_HCLKDIV8 STM32_SYSTICKSEL_FIELD(0U)
#define STM32_SYSTICKSEL_LSI    STM32_SYSTICKSEL_FIELD(1U)
#define STM32_SYSTICKSEL_LSE    STM32_SYSTICKSEL_FIELD(2U)
#define STM32_SYSTICKSEL_NOCLOCK STM32_SYSTICKSEL_FIELD(3U)

#define STM32_USBSEL_MASK      (3U << 4)
#define STM32_USBSEL_FIELD(n)  ((n) << 4)
#define STM32_USBSEL_NOCLOCK   STM32_USBSEL_FIELD(0U)
#define STM32_USBSEL_PLL1Q     STM32_USBSEL_FIELD(1U)
#define STM32_USBSEL_PLL3Q     STM32_USBSEL_FIELD(2U)
#define STM32_USBSEL_HSI48     STM32_USBSEL_FIELD(3U)

#define STM32_SDMMC1SEL_MASK   (1U << 6)
#define STM32_SDMMC1SEL_FIELD(n) ((n) << 6)
#define STM32_SDMMC1SEL_PLL1Q  STM32_SDMMC1SEL_FIELD(0U)
#define STM32_SDMMC1SEL_PLL2R  STM32_SDMMC1SEL_FIELD(1U)

#define STM32_SDMMC2SEL_MASK   (1U << 7)
#define STM32_SDMMC2SEL_FIELD(n) ((n) << 7)
#define STM32_SDMMC2SEL_PLL1Q  STM32_SDMMC2SEL_FIELD(0U)
#define STM32_SDMMC2SEL_PLL2R  STM32_SDMMC2SEL_FIELD(1U)

#define STM32_I2C1SEL_MASK      (3U << 16)
#define STM32_I2C1SEL_FIELD(n)  ((n) << 16)
#define STM32_I2C1SEL_PCLK1     STM32_I2C1SEL_FIELD(0U)
#define STM32_I2C1SEL_PLL3R     STM32_I2C1SEL_FIELD(1U)
#define STM32_I2C1SEL_HSI       STM32_I2C1SEL_FIELD(2U)
#define STM32_I2C1SEL_CSI       STM32_I2C1SEL_FIELD(3U)

#define STM32_I2C2SEL_MASK      (3U << 18)
#define STM32_I2C2SEL_FIELD(n)  ((n) << 18)
#define STM32_I2C2SEL_PCLK1     STM32_I2C2SEL_FIELD(0U)
#define STM32_I2C2SEL_PLL3R     STM32_I2C2SEL_FIELD(1U)
#define STM32_I2C2SEL_HSI       STM32_I2C2SEL_FIELD(2U)
#define STM32_I2C2SEL_CSI       STM32_I2C2SEL_FIELD(3U)

#define STM32_I2C3SEL_MASK      (3U << 20)
#define STM32_I2C3SEL_FIELD(n)  ((n) << 20)
#define STM32_I2C3SEL_PCLK3     STM32_I2C3SEL_FIELD(0U)
#define STM32_I2C3SEL_PLL3R     STM32_I2C3SEL_FIELD(1U)
#define STM32_I2C3SEL_HSI       STM32_I2C3SEL_FIELD(2U)
#define STM32_I2C3SEL_CSI       STM32_I2C3SEL_FIELD(3U)

#define STM32_I2C4SEL_MASK      (3U << 22)
#define STM32_I2C4SEL_FIELD(n)  ((n) << 22)
#define STM32_I2C4SEL_PCLK3     STM32_I2C4SEL_FIELD(0U)
#define STM32_I2C4SEL_PLL3R     STM32_I2C4SEL_FIELD(1U)
#define STM32_I2C4SEL_HSI       STM32_I2C4SEL_FIELD(2U)
#define STM32_I2C4SEL_CSI       STM32_I2C4SEL_FIELD(3U)

#define STM32_I3C1SEL_MASK      (3U << 24)
#define STM32_I3C1SEL_FIELD(n)  ((n) << 24)
#define STM32_I3C1SEL_PCLK1     STM32_I3C1SEL_FIELD(0U)
#define STM32_I3C1SEL_PLL3R     STM32_I3C1SEL_FIELD(1U)
#define STM32_I3C1SEL_HSI       STM32_I3C1SEL_FIELD(2U)
/** @} */

/**
 * @name    RCC_CCIPR5 register bits definitions
 * @{
 */
#define STM32_ADCDACSEL_MASK    (3U << 0)
#define STM32_ADCDACSEL_FIELD(n) ((n) << 0)
#define STM32_ADCDACSEL_HCLK    STM32_ADCDACSEL_FIELD(0U)
#define STM32_ADCDACSEL_SYSCLK  STM32_ADCDACSEL_FIELD(1U)
#define STM32_ADCDACSEL_PLL2R   STM32_ADCDACSEL_FIELD(2U)
#define STM32_ADCDACSEL_HSE     STM32_ADCDACSEL_FIELD(3U)
#define STM32_ADCDACSEL_HSI     STM32_ADCDACSEL_FIELD(4U)
#define STM32_ADCDACSEL_CSI     STM32_ADCDACSEL_FIELD(5U)

#define STM32_DACSEL_MASK       (1U << 3)
#define STM32_DACSEL_FIELD(n)   ((n) << 3)
#define STM32_DACSEL_IGNORE     0xFFFFFFFFU
#define STM32_DACSEL_LSE        STM32_DACSEL_FIELD(0U)
#define STM32_DACSEL_LSI        STM32_DACSEL_FIELD(1U)

#define STM32_RNGSEL_MASK       (3U << 4)
#define STM32_RNGSEL_FIELD(n)   ((n) << 4)
#define STM32_RNGSEL_IGNORE     0xFFFFFFFFU
#define STM32_RNGSEL_HSI48      STM32_RNGSEL_FIELD(0U)
#define STM32_RNGSEL_PLL1Q      STM32_RNGSEL_FIELD(1U)
#define STM32_RNGSEL_LSE        STM32_RNGSEL_FIELD(2U)
#define STM32_RNGSEL_LSI        STM32_RNGSEL_FIELD(3U)

#define STM32_CECSEL_MASK       (3U << 6)
#define STM32_CECSEL_FIELD(n)   ((n) << 6)
#define STM32_CECSEL_IGNORE     0xFFFFFFFFU
#define STM32_CECSEL_LSE        STM32_CECSEL_FIELD(0U)
#define STM32_CECSEL_LSI        STM32_CECSEL_FIELD(1U)
#define STM32_CECSEL_CSIDIV128  STM32_CECSEL_FIELD(2U)

#define STM32_FDCANSEL_MASK     (3U << 8)
#define STM32_FDCANSEL_FIELD(n) ((n) << 8)
#define STM32_FDCANSEL_IGNORE   0xFFFFFFFFU
#define STM32_FDCANSEL_HSE      STM32_FDCANSEL_FIELD(0U)
#define STM32_FDCANSEL_PLL1Q    STM32_FDCANSEL_FIELD(1U)
#define STM32_FDCANSEL_PLL2Q    STM32_FDCANSEL_FIELD(2U)

#define STM32_SAI1SEL_MASK      (7U << 16)
#define STM32_SAI1SEL_FIELD(n)  ((n) << 16)
#define STM32_SAI1SEL_PLL1Q     STM32_SAI1SEL_FIELD(0U)
#define STM32_SAI1SEL_PLL2P     STM32_SAI1SEL_FIELD(1U)
#define STM32_SAI1SEL_PLL3P     STM32_SAI1SEL_FIELD(2U)
#define STM32_SAI1SEL_AUDIOCLK  STM32_SAI1SEL_FIELD(3U)
#define STM32_SAI1SEL_PER       STM32_SAI1SEL_FIELD(4U)

#define STM32_SAI2SEL_MASK      (7U << 19)
#define STM32_SAI2SEL_FIELD(n)  ((n) << 19)
#define STM32_SAI2SEL_PLL1Q     STM32_SAI2SEL_FIELD(0U)
#define STM32_SAI2SEL_PLL2P     STM32_SAI2SEL_FIELD(1U)
#define STM32_SAI2SEL_PLL3P     STM32_SAI2SEL_FIELD(2U)
#define STM32_SAI2SEL_AUDIOCLK  STM32_SAI2SEL_FIELD(3U)
#define STM32_SAI2SEL_PER       STM32_SAI2SEL_FIELD(4U)

#define STM32_CKPERSEL_MASK     (3U << 30)
#define STM32_CKPERSEL_FIELD(n) ((n) << 30)
#define STM32_CKPERSEL_HSI      STM32_CKPERSEL_FIELD(0U)
#define STM32_CKPERSEL_CSI      STM32_CKPERSEL_FIELD(1U)
#define STM32_CKPERSEL_HSE      STM32_CKPERSEL_FIELD(2U)
/** @} */

/**
 * @name    RCC_BDCR register bits definitions
 * @{
 */
#define STM32_RTCSEL_MASK       (3U << 8)
#define STM32_RTCSEL_FIELD(n)   ((n) << 8)
#define STM32_RTCSEL_NOCLOCK    STM32_RTCSEL_FIELD(0U)
#define STM32_RTCSEL_LSE        STM32_RTCSEL_FIELD(1U)
#define STM32_RTCSEL_LSI        STM32_RTCSEL_FIELD(2U)
#define STM32_RTCSEL_HSEDIV     STM32_RTCSEL_FIELD(3U)

#define STM32_LSCOSEL_MASK      (2U << 24)
#define STM32_LSCOSEL_FIELD(n)  ((n) << 24)
#define STM32_LSCOSEL_NOCLOCK   STM32_LSCOSEL_FIELD(0U)
#define STM32_LSCOSEL_LSI       STM32_LSCOSEL_FIELD(1U)
#define STM32_LSCOSEL_LSE       STM32_LSCOSEL_FIELD(3U)
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
#define STM32_PWR_VOSCR                     STM32_VOS_RANGE0
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
#define STM32_SW                            STM32_SW_PLL1PCLK
#endif

/**
 * @brief   Clock source for PLL1.
 */
#if !defined(STM32_PLL1SRC) || defined(__DOXYGEN__)
#define STM32_PLL1SRC                       STM32_PLL1SRC_HSI
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
#define STM32_PLL1N_VALUE                   84
#endif

/**
 * @brief   PLL1P divider value.
 * @note    The allowed values are 4..128 (odd values forbidden).
 */
#if !defined(STM32_PLL1P_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1P_VALUE                   7
#endif

/**
 * @brief   PLL1Q divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL1Q_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1Q_VALUE                   8
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
#define STM32_PLL2SRC                       STM32_PLL2SRC_HSI
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
#define STM32_PLL2N_VALUE                   84
#endif

/**
 * @brief   PLL2P divider value.
 * @note    The allowed values are 4..128.
 */
#if !defined(STM32_PLL2P_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2P_VALUE                   7
#endif

/**
 * @brief   PLL2Q divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL2Q_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2Q_VALUE                   8
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
#define STM32_PLL3SRC                       STM32_PLL3SRC_HSI
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
#define STM32_PLL3N_VALUE                   84
#endif

/**
 * @brief   PLL3P divider value.
 * @note    The allowed values are 4..128.
 */
#if !defined(STM32_PLL3P_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3P_VALUE                   7
#endif

/**
 * @brief   PLL3Q divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL3Q_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3Q_VALUE                   8
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
#define STM32_HPRE                          STM32_HPRE_DIV1
#endif

/**
 * @brief   APB1 prescaler value.
 */
#if !defined(STM32_PPRE1) || defined(__DOXYGEN__)
#define STM32_PPRE1                         STM32_PPRE1_DIV1
#endif

/**
 * @brief   APB2 prescaler value.
 */
#if !defined(STM32_PPRE2) || defined(__DOXYGEN__)
#define STM32_PPRE2                         STM32_PPRE2_DIV1
#endif

/**
 * @brief   APB3 prescaler value.
 */
#if !defined(STM32_PPRE3) || defined(__DOXYGEN__)
#define STM32_PPRE3                         STM32_PPRE3_DIV1
#endif

/**
 * @brief   System clock source after STOP.
 */
#if !defined(STM32_STOPWUCK) || defined(__DOXYGEN__)
#define STM32_STOPWUCK                      STM32_STOPWUCK_HSI
#endif

/**
 * @brief   Kernel clock source after STOP.
 */
#if !defined(STM32_STOPKERWUCK) || defined(__DOXYGEN__)
#define STM32_STOPKERWUCK                   STM32_STOPKERWUCK_HSI
#endif

/**
 * @brief   RTC prescaler value.
 */
#if !defined(STM32_RTCPRE_VALUE) || defined(__DOXYGEN__)
#define STM32_RTCPRE_VALUE                  STM32_RTCPRE_NOCLOCK
#endif

/**
 * @brief   TIMPRE timers clocks prescaler selection.
 */
#if !defined(STM32_TIMPRE) || defined(__DOXYGEN__)
#define STM32_TIMPRE                        STM32_TIMPRE_LOW
#endif

/**
 * @brief   MCO1 clock source.
 */
#if !defined(STM32_MCO1SEL) || defined(__DOXYGEN__)
#define STM32_MCO1SEL                       STM32_MCO1SEL_HSI
#endif

/**
 * @brief   MCO1 divider setting.
 */
#if !defined(STM32_MCO1PRE_VALUE) || defined(__DOXYGEN__)
#define STM32_MCO1PRE_VALUE                 STM32_MCO1PRE_NOCLOCK
#endif

/**
 * @brief   MCO2 clock source.
 */
#if !defined(STM32_MCO2SEL) || defined(__DOXYGEN__)
#define STM32_MCO2SEL                       STM32_MCO2SEL_SYSCLK
#endif

/**
 * @brief   MCO1 divider setting.
 */
#if !defined(STM32_MCO2PRE_VALUE) || defined(__DOXYGEN__)
#define STM32_MCO2PRE_VALUE                 STM32_MCO2PRE_NOCLOCK
#endif

/**
 * @brief   LSCO clock source.
 */
#if !defined(STM32_LSCOSEL) || defined(__DOXYGEN__)
#define STM32_LSCOSEL                       STM32_LSCOSEL_NOCLOCK
#endif

/**
 * @brief   TIMICSEL clock source.
 */
#if !defined(STM32_TIMICSEL) || defined(__DOXYGEN__)
#define STM32_TIMICSEL                      STM32_TIMICSEL_NOCLK
#endif

/**
 * @brief   USART1 clock source.
 */
#if !defined(STM32_USART1SEL) || defined(__DOXYGEN__)
#define STM32_USART1SEL                     STM32_USART1SEL_PCLK2
#endif

/**
 * @brief   USART2 clock source.
 */
#if !defined(STM32_USART2SEL) || defined(__DOXYGEN__)
#define STM32_USART2SEL                     STM32_USART2SEL_PCLK1
#endif

/**
 * @brief   USART3 clock source.
 */
#if !defined(STM32_USART3SEL) || defined(__DOXYGEN__)
#define STM32_USART3SEL                     STM32_USART3SEL_PCLK1
#endif

/**
 * @brief   UART4 clock source.
 */
#if !defined(STM32_UART4SEL) || defined(__DOXYGEN__)
#define STM32_UART4SEL                      STM32_UART4SEL_PCLK1
#endif

/**
 * @brief   UART5 clock source.
 */
#if !defined(STM32_UART5SEL) || defined(__DOXYGEN__)
#define STM32_UART5SEL                      STM32_UART5SEL_PCLK1
#endif

/**
 * @brief   USART6 clock source.
 */
#if !defined(STM32_USART6SEL) || defined(__DOXYGEN__)
#define STM32_USART6SEL                     STM32_USART6SEL_PCLK1
#endif

/**
 * @brief   UART7 clock source.
 */
#if !defined(STM32_UART7SEL) || defined(__DOXYGEN__)
#define STM32_UART7SEL                      STM32_UART7SEL_PCLK1
#endif

/**
 * @brief   UART8 clock source.
 */
#if !defined(STM32_UART8SEL) || defined(__DOXYGEN__)
#define STM32_UART8SEL                      STM32_UART8SEL_PCLK1
#endif

/**
 * @brief   UART9 clock source.
 */
#if !defined(STM32_UART9SEL) || defined(__DOXYGEN__)
#define STM32_UART9SEL                      STM32_UART9SEL_PCLK1
#endif

/**
 * @brief   USART10 clock source.
 */
#if !defined(STM32_USART10SEL) || defined(__DOXYGEN__)
#define STM32_USART10SEL                    STM32_USART10SEL_PCLK1
#endif

/**
 * @brief   USART11 clock source.
 */
#if !defined(STM32_USART11SEL) || defined(__DOXYGEN__)
#define STM32_USART11SEL                    STM32_USART11SEL_PCLK1
#endif

/**
 * @brief   UART12 clock source.
 */
#if !defined(STM32_UART12SEL) || defined(__DOXYGEN__)
#define STM32_UART12SEL                     STM32_UART12SEL_PCLK1
#endif

/**
 * @brief   LPUART1 clock source.
 */
#if !defined(STM32_LPUART1SEL) || defined(__DOXYGEN__)
#define STM32_LPUART1SEL                    STM32_LPUART1SEL_PCLK3
#endif

/**
 * @brief   LPTIM1 clock source.
 */
#if !defined(STM32_LPTIM1SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM1SEL                     STM32_LPTIM1SEL_PCLK3
#endif

/**
 * @brief   LPTIM2 clock source.
 */
#if !defined(STM32_LPTIM2SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM2SEL                     STM32_LPTIM2SEL_PCLK1
#endif

/**
 * @brief   LPTIM3 clock source.
 */
#if !defined(STM32_LPTIM3SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM3SEL                     STM32_LPTIM3SEL_PCLK3
#endif

/**
 * @brief   LPTIM4 clock source.
 */
#if !defined(STM32_LPTIM4SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM4SEL                     STM32_LPTIM4SEL_PCLK3
#endif

/**
 * @brief   LPTIM5 clock source.
 */
#if !defined(STM32_LPTIM5SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM5SEL                     STM32_LPTIM5SEL_PCLK3
#endif

/**
 * @brief   LPTIM6 clock source.
 */
#if !defined(STM32_LPTIM6SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM6SEL                     STM32_LPTIM6SEL_PCLK3
#endif

/**
 * @brief   SPI1 clock source.
 */
#if !defined(STM32_SPI1SEL) || defined(__DOXYGEN__)
#define STM32_SPI1SEL                       STM32_SPI1SEL_PLL1Q
#endif

/**
 * @brief   SPI2 clock source.
 */
#if !defined(STM32_SPI2SEL) || defined(__DOXYGEN__)
#define STM32_SPI2SEL                       STM32_SPI2SEL_PLL1Q
#endif

/**
 * @brief   SPI3 clock source.
 */
#if !defined(STM32_SPI3SEL) || defined(__DOXYGEN__)
#define STM32_SPI3SEL                       STM32_SPI3SEL_PLL1Q
#endif

/**
 * @brief   SPI4 clock source.
 */
#if !defined(STM32_SPI4SEL) || defined(__DOXYGEN__)
#define STM32_SPI4SEL                       STM32_SPI4SEL_PCLK2
#endif

/**
 * @brief   SPI5 clock source.
 */
#if !defined(STM32_SPI5SEL) || defined(__DOXYGEN__)
#define STM32_SPI5SEL                       STM32_SPI5SEL_PCLK3
#endif

/**
 * @brief   SPI6 clock source.
 */
#if !defined(STM32_SPI6SEL) || defined(__DOXYGEN__)
#define STM32_SPI6SEL                       STM32_SPI6SEL_PCLK2
#endif

/**
 * @brief   QSPI clock source.
 */
#if !defined(STM32_OSPISEL) || defined(__DOXYGEN__)
#define STM32_OSPISEL                       STM32_OSPISEL_HCLK4
#endif

/**
 * @brief   SYSTICK clock source.
 */
#if !defined(STM32_SYSTICKSEL) || defined(__DOXYGEN__)
#define STM32_SYSTICKSEL                    STM32_SYSTICKSEL_HCLKDIV8
#endif

/**
 * @brief   USB clock source.
 */
#if !defined(STM32_USBSEL) || defined(__DOXYGEN__)
#define STM32_USBSEL                        STM32_USBSEL_NOCLOCK
#endif

/**
 * @brief   SDMMC1 clock source.
 */
#if !defined(STM32_SDMMC1SEL) || defined(__DOXYGEN__)
#define STM32_SDMMC1SEL                     STM32_SDMMC1SEL_PLL1Q
#endif

/**
 * @brief   SDMMC2 clock source.
 */
#if !defined(STM32_SDMMC2SEL) || defined(__DOXYGEN__)
#define STM32_SDMMC2SEL                     STM32_SDMMC2SEL_PLL1Q
#endif

/**
 * @brief   I2C1 clock source.
 */
#if !defined(STM32_I2C1SEL) || defined(__DOXYGEN__)
#define STM32_I2C1SEL                       STM32_I2C1SEL_PCLK1
#endif

/**
 * @brief   I2C2 clock source.
 */
#if !defined(STM32_I2C2SEL) || defined(__DOXYGEN__)
#define STM32_I2C2SEL                       STM32_I2C2SEL_PCLK1
#endif

/**
 * @brief   I2C3 clock source.
 */
#if !defined(STM32_I2C3SEL) || defined(__DOXYGEN__)
#define STM32_I2C3SEL                       STM32_I2C3SEL_PCLK3
#endif

/**
 * @brief   I2C4 clock source.
 */
#if !defined(STM32_I2C4SEL) || defined(__DOXYGEN__)
#define STM32_I2C4SEL                       STM32_I2C4SEL_PCLK3
#endif

/**
 * @brief   I3C1 clock source.
 */
#if !defined(STM32_I3C1SEL) || defined(__DOXYGEN__)
#define STM32_I3C1SEL                       STM32_I3C1SEL_PCLK1
#endif

/**
 * @brief   ADCDACSEL clock source.
 */
#if !defined(STM32_ADCDACSEL) || defined(__DOXYGEN__)
#define STM32_ADCDACSEL                     STM32_ADCDACSEL_HCLK
#endif

/**
 * @brief   DACSEL clock source.
 */
#if !defined(STM32_DACSEL) || defined(__DOXYGEN__)
#define STM32_DACSEL                        STM32_DACSEL_LSE
#endif

/**
 * @brief   RNG clock source.
 */
#if !defined(STM32_RNGSEL) || defined(__DOXYGEN__)
#define STM32_RNGSEL                        STM32_RNGSEL_HSI48
#endif

/**
 * @brief   CEC clock source.
 */
#if !defined(STM32_CECSEL) || defined(__DOXYGEN__)
#define STM32_CECSEL                        STM32_CECSEL_LSE
#endif

/**
 * @brief   FDCAN clock source.
 */
#if !defined(STM32_FDCANSEL) || defined(__DOXYGEN__)
#define STM32_FDCANSEL                      STM32_FDCANSEL_HSE
#endif

/**
 * @brief   SAI1 clock source.
 */
#if !defined(STM32_SAI1SEL) || defined(__DOXYGEN__)
#define STM32_SAI1SEL                       STM32_SAI1SEL_PLL1Q
#endif

/**
 * @brief   SAI2 clock source.
 */
#if !defined(STM32_SAI2SEL) || defined(__DOXYGEN__)
#define STM32_SAI2SEL                       STM32_SAI2SEL_PLL1Q
#endif

/**
 * @brief   CKPERSEL clock source.
 */
#if !defined(STM32_CKPERSEL) || defined(__DOXYGEN__)
#define STM32_CKPERSEL                      STM32_CKPERSEL_HSI
#endif

/**
 * @brief   RTC clock source.
 */
#if !defined(STM32_RTCSEL) || defined(__DOXYGEN__)
#define STM32_RTCSEL                        STM32_RTCSEL_NOCLOCK
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
#if (STM32_SW           == STM32_SW_PLL1P)          ||                      \
    (STM32_MCO1SEL      == STM32_MCO1SEL_PLL1P)     ||                      \
    (STM32_MCO2SEL      == STM32_MCO2SEL_PLL1P)     ||                      \
    (STM32_SPI1SEL      == STM32_SPI1SEL_PLL1Q)     ||                      \
    (STM32_SPI2SEL      == STM32_SPI2SEL_PLL1Q)     ||                      \
    (STM32_SPI3SEL      == STM32_SPI3SEL_PLL1Q)     ||                      \
    (STM32_OSPISEL      == STM32_OSPISEL_PLL1Q)     ||                      \
    (STM32_USBSEL       == STM32_USBSEL_PLL1Q)      ||                      \
    (STM32_SDMMC1SEL    == STM32_SDMMC1SEL_PLL1Q)   ||                      \
    (STM32_SDMMC2SEL    == STM32_SDMMC2SEL_PLL1Q)   ||                      \
    (STM32_RNGSEL       == STM32_RNGSEL_PLL1Q)      ||                      \
    (STM32_FDCANSEL     == STM32_FDCANSEL_PLL1Q)    ||                      \
    (STM32_SAI1SEL      == STM32_SAI1SEL_PLL1Q)     ||                      \
    (STM32_SAI2SEL      == STM32_SAI2SEL_PLL1Q)     ||                      \
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
#if (STM32_MCO2SEL      == STM32_MCO2SEL_PLL2P)     ||                      \
    (STM32_USART1SEL    == STM32_USART1SEL_PLL2Q)   ||                      \
    (STM32_USART2SEL    == STM32_USART2SEL_PLL2Q)   ||                      \
    (STM32_USART3SEL    == STM32_USART3SEL_PLL2Q)   ||                      \
    (STM32_UART4SEL     == STM32_UART4SEL_PLL2Q)    ||                      \
    (STM32_UART5SEL     == STM32_UART5SEL_PLL2Q)    ||                      \
    (STM32_USART6SEL    == STM32_USART6SEL_PLL2Q)   ||                      \
    (STM32_UART7SEL     == STM32_UART7SEL_PLL2Q)    ||                      \
    (STM32_UART8SEL     == STM32_UART8SEL_PLL2Q)    ||                      \
    (STM32_UART9SEL     == STM32_UART9SEL_PLL2Q)    ||                      \
    (STM32_USART10SEL   == STM32_USART10SEL_PLL2Q)  ||                      \
    (STM32_USART11SEL   == STM32_USART11SEL_PLL2Q)  ||                      \
    (STM32_UART12SEL    == STM32_UART12SEL_PLL2Q)   ||                      \
    (STM32_LPUART1SEL   == STM32_LPUART1SEL_PLL2Q)  ||                      \
    (STM32_LPTIM1SEL    == STM32_LPTIM1SEL_PLL2P)   ||                      \
    (STM32_LPTIM2SEL    == STM32_LPTIM2SEL_PLL2P)   ||                      \
    (STM32_LPTIM3SEL    == STM32_LPTIM3SEL_PLL2P)   ||                      \
    (STM32_LPTIM4SEL    == STM32_LPTIM4SEL_PLL2P)   ||                      \
    (STM32_LPTIM5SEL    == STM32_LPTIM5SEL_PLL2P)   ||                      \
    (STM32_LPTIM6SEL    == STM32_LPTIM6SEL_PLL2P)   ||                      \
    (STM32_SPI1SEL      == STM32_SPI1SEL_PLL2P)     ||                      \
    (STM32_SPI2SEL      == STM32_SPI2SEL_PLL2P)     ||                      \
    (STM32_SPI3SEL      == STM32_SPI3SEL_PLL2P)     ||                      \
    (STM32_SPI4SEL      == STM32_SPI4SEL_PLL2P)     ||                      \
    (STM32_SPI5SEL      == STM32_SPI5SEL_PLL2P)     ||                      \
    (STM32_SPI6SEL      == STM32_SPI6SEL_PLL2P)     ||                      \
    (STM32_OSPISEL      == STM32_OSPISEL_PLL2R )    ||                      \
    (STM32_SDMMC1SEL    == STM32_SDMMC1SEL_PLL2R)   ||                      \
    (STM32_SDMMC2SEL    == STM32_SDMMC2SEL_PLL2R)   ||                      \
    (STM32_ADCDACSEL    == STM32_ADCDACSEL_PLL2R)   ||                      \
    (STM32_FDCANSEL     == STM32_FDCANSEL_PLL2Q)    ||                      \
    (STM32_SAI1SEL      == STM32_SAI1SEL_PLL2P)     ||                      \
    (STM32_SAI2SEL      == STM32_SAI2SEL_PLL2P)     ||                      \
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
#if (STM32_USART1SEL    == STM32_USART1SEL_PLL3Q)   ||                      \
    (STM32_USART2SEL    == STM32_USART2SEL_PLL3Q)   ||                      \
    (STM32_USART3SEL    == STM32_USART3SEL_PLL3Q)   ||                      \
    (STM32_UART4SEL     == STM32_UART4SEL_PLL3Q)    ||                      \
    (STM32_UART5SEL     == STM32_UART5SEL_PLL3Q)    ||                      \
    (STM32_USART6SEL    == STM32_USART6SEL_PLL3Q)   ||                      \
    (STM32_UART7SEL     == STM32_UART7SEL_PLL3Q)    ||                      \
    (STM32_UART8SEL     == STM32_UART8SEL_PLL3Q)    ||                      \
    (STM32_UART9SEL     == STM32_UART9SEL_PLL3Q)    ||                      \
    (STM32_USART10SEL   == STM32_USART10SEL_PLL3Q)  ||                      \
    (STM32_USART11SEL   == STM32_USART11SEL_PLL3Q)  ||                      \
    (STM32_UART12SEL    == STM32_UART12SEL_PLL3Q)   ||                      \
    (STM32_LPUART1SEL   == STM32_LPUART1SEL_PLL3Q)  ||                      \
    (STM32_LPTIM1SEL    == STM32_LPTIM1SEL_PLL3R)   ||                      \
    (STM32_LPTIM2SEL    == STM32_LPTIM2SEL_PLL3R)   ||                      \
    (STM32_LPTIM3SEL    == STM32_LPTIM3SEL_PLL3R)   ||                      \
    (STM32_LPTIM4SEL    == STM32_LPTIM4SEL_PLL3R)   ||                      \
    (STM32_LPTIM5SEL    == STM32_LPTIM5SEL_PLL3R)   ||                      \
    (STM32_LPTIM6SEL    == STM32_LPTIM6SEL_PLL3R)   ||                      \
    (STM32_SPI1SEL      == STM32_SPI1SEL_PLL3P)     ||                      \
    (STM32_SPI2SEL      == STM32_SPI2SEL_PLL3P)     ||                      \
    (STM32_SPI3SEL      == STM32_SPI3SEL_PLL3P)     ||                      \
    (STM32_SPI4SEL      == STM32_SPI4SEL_PLL3P)     ||                      \
    (STM32_SPI5SEL      == STM32_SPI5SEL_PLL3P)     ||                      \
    (STM32_SPI6SEL      == STM32_SPI6SEL_PLL3P)     ||                      \
    (STM32_USBSEL       == STM32_USBSEL_PLL3Q)      ||                      \
    (STM32_I2C1SEL      == STM32_I2C1SEL_PLL3R)     ||                      \
    (STM32_I2C2SEL      == STM32_I2C2SEL_PLL3R)     ||                      \
    (STM32_I2C3SEL      == STM32_I2C3SEL_PLL3R)     ||                      \
    (STM32_I2C4SEL      == STM32_I2C4SEL_PLL3R)     ||                      \
    (STM32_I3C1SEL      == STM32_I3C1SEL_PLL3R)     ||                      \
    (STM32_SAI1SEL      == STM32_SAI1SEL_PLL3P)     ||                      \
    (STM32_SAI2SEL      == STM32_SAI2SEL_PLL3P)     ||                      \
    defined(__DOXYGEN__)
  /**
   * @brief   PLL3 activation flag.
   */
  #define STM32_ACTIVATE_PLL3       TRUE
#else

  #define STM32_ACTIVATE_PLL3       FALSE
#endif

/*
 * LSI related checks.
 */
#if STM32_LSI_ENABLED
#else /* !STM32_LSI_ENABLED */

  #if STM32_MCO2SEL == STM32_MCO2SEL_LSI
    #error "LSI not enabled, required by STM32_MCO2SEL"
  #endif

  #if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSI)
    #error "HSI not enabled, required by STM32_LPTIM1SEL"
  #endif
  #if (STM32_LPTIM2SEL == STM32_LPTIM2SEL_LSI)
    #error "HSI not enabled, required by STM32_LPTIM2SEL"
  #endif
  #if (STM32_LPTIM3SEL == STM32_LPTIM3SEL_LSI)
    #error "HSI not enabled, required by STM32_LPTIM3SEL"
  #endif
  #if (STM32_LPTIM4SEL == STM32_LPTIM4SEL_LSI)
    #error "HSI not enabled, required by STM32_LPTIM4SEL"
  #endif
  #if (STM32_LPTIM5SEL == STM32_LPTIM5SEL_LSI)
    #error "HSI not enabled, required by STM32_LPTIM5SEL"
  #endif
  #if (STM32_LPTIM6SEL == STM32_LPTIM6SEL_LSI)
    #error "HSI not enabled, required by STM32_LPTIM6SEL"
  #endif

  #if STM32_SYSTICKSEL == STM32_SYSTICKSEL_LSI
    #error "LSI not enabled, required by STM32_SYSTICKSEL"
  #endif

  #if STM32_RNGSEL == STM32_RNGSEL_LSI
    #error "LSI not enabled, required by STM32_RNGSEL"
  #endif

  #if STM32_DACSEL == STM32_DACSEL_LSI
    #error "LSI not enabled, required by STM32_DACSEL"
  #endif

  #if STM32_CECSEL == STM32_CECSEL_LSI
    #error "LSI not enabled, required by STM32_CECSEL"
  #endif

  #if HAL_USE_RTC && (STM32_RTCSEL == STM32_RTCSEL_LSI)
    #error "LSI not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_LSCOSEL == STM32_LSCOSEL_LSI
    #error "LSI not enabled, required by STM32_LSCOSEL"
  #endif

#endif /* !STM32_LSI_ENABLED */

/*
 * CSI related checks.
 */
#if STM32_CSI_ENABLED
#else /* !STM32_CSI_ENABLED */

  #if STM32_ACTIVATE_PLL1 && (STM32_PLL1SRC == STM32_PLL1SRC_CSI)
    #error "CSI not enabled, required by STM32_PLL1SRC"
  #endif
  #if STM32_ACTIVATE_PLL2 && (STM32_PLL1SRC == STM32_PLL2SRC_CSI)
    #error "CSI not enabled, required by STM32_PLL2SRC"
  #endif
  #if STM32_ACTIVATE_PLL3 && (STM32_PLL1SRC == STM32_PLL3SRC_CSI)
    #error "CSI not enabled, required by STM32_PLL3SRC"
  #endif

  #if STM32_SW == STM32_SW_CSI
    #error "CSI not enabled, required by STM32_SW"
  #endif

  #if STM32_MCO2SEL == STM32_MCO2SEL_CSI
    #error "CSI not enabled, required by STM32_MCO2SEL"
  #endif

  #if (STM32_USART1SEL == STM32_USART1SEL_CSI)
    #error "CSI not enabled, required by STM32_USART1SEL"
  #endif
  #if (STM32_USART2SEL == STM32_USART2SEL_CSI)
    #error "CSI not enabled, required by STM32_USART2SEL"
  #endif
  #if (STM32_USART3SEL == STM32_USART3SEL_CSI)
    #error "CSI not enabled, required by STM32_USART3SEL"
  #endif
  #if (STM32_UART4SEL == STM32_UART4SEL_CSI)
    #error "CSI not enabled, required by STM32_UART4SEL"
  #endif
  #if (STM32_UART5SEL == STM32_UART5SEL_CSI)
    #error "CSI not enabled, required by STM32_UART5SEL"
  #endif
  #if (STM32_USART6SEL == STM32_USART6SEL_CSI)
    #error "CSI not enabled, required by STM32_USART6SEL"
  #endif
  #if (STM32_UART7SEL == STM32_UART7SEL_CSI)
    #error "CSI not enabled, required by STM32_UART7SEL"
  #endif
  #if (STM32_UART8SEL == STM32_UART8SEL_CSI)
    #error "CSI not enabled, required by STM32_UART8SEL"
  #endif
  #if (STM32_UART9SEL == STM32_UART9SEL_CSI)
    #error "CSI not enabled, required by STM32_UART9SEL"
  #endif
  #if (STM32_USART10SEL == STM32_USART10SEL_CSI)
    #error "CSI not enabled, required by STM32_USART10SEL"
  #endif
  #if (STM32_USART11SEL == STM32_USART11SEL_CSI)
    #error "CSI not enabled, required by STM32_USART11SEL"
  #endif
  #if (STM32_UART12SEL == STM32_UART12SEL_CSI)
    #error "CSI not enabled, required by STM32_UART12SEL"
  #endif
  #if (STM32_LPUART1SEL == STM32_LPUART1SEL_CSI)
    #error "CSI not enabled, required by STM32_LPUART1SEL"
  #endif

  #if (STM32_SPI4SEL == STM32_SPI4SEL_CSI)
    #error "CSI not enabled, required by STM32_SPI4SEL"
  #endif
  #if (STM32_SPI5SEL == STM32_SPI5SEL_CSI)
    #error "CSI not enabled, required by STM32_SPI5SEL"
  #endif
  #if (STM32_SPI6SEL == STM32_SPI6SEL_CSI)
    #error "CSI not enabled, required by STM32_SPI6SEL"
  #endif

  #if (STM32_I2C1SEL == STM32_I2C1SEL_CSI)
    #error "CSI not enabled, required by STM32_I2C1SEL"
  #endif
  #if (STM32_I2C2SEL == STM32_I2C2SEL_CSI)
    #error "CSI not enabled, required by STM32_I2C2SEL"
  #endif
  #if (STM32_I2C3SEL == STM32_I2C3SEL_CSI)
    #error "CSI not enabled, required by STM32_I2C3SEL"
  #endif
  #if (STM32_I2C4SEL == STM32_I2C4SEL_CSI)
    #error "CSI not enabled, required by STM32_I2C4SEL"
  #endif

  #if (STM32_ADCDACSEL == STM32_ADCDACSEL_CSI)
    #error "CSI not enabled, required by STM32_ADCDACSEL"
  #endif

  #if STM32_CECSEL == STM32_CECSEL_CSIDIV128
    #error "CSI not enabled, required by STM32_CECSEL"
  #endif

  #if STM32_CKPERSEL == STM32_CKPERSEL_CSI
    #if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_PER)
      #error "CSI not enabled, required by STM32_LPTIM1SEL"
    #endif
    #if (STM32_LPTIM2SEL == STM32_LPTIM2SEL_PER)
      #error "CSI not enabled, required by STM32_LPTIM2SEL"
    #endif
    #if (STM32_LPTIM3SEL == STM32_LPTIM3SEL_PER)
      #error "CSI not enabled, required by STM32_LPTIM3SEL"
    #endif
    #if (STM32_LPTIM4SEL == STM32_LPTIM4SEL_PER)
      #error "CSI not enabled, required by STM32_LPTIM4SEL"
    #endif
    #if (STM32_LPTIM5SEL == STM32_LPTIM5SEL_PER)
      #error "CSI not enabled, required by STM32_LPTIM5SEL"
    #endif
    #if (STM32_LPTIM6SEL == STM32_LPTIM6SEL_PER)
      #error "CSI not enabled, required by STM32_LPTIM6SEL"
    #endif

    #if (STM32_SPI1SEL == STM32_SPI1SEL_PER)
      #error "CSI not enabled, required by STM32_SPI1SEL"
    #endif
    #if (STM32_SPI2SEL == STM32_SPI2SEL_PER)
      #error "CSI not enabled, required by STM32_SPI2SEL"
    #endif
    #if (STM32_SPI3SEL == STM32_SPI3SEL_PER)
      #error "CSI not enabled, required by STM32_SPI3SEL"
    #endif

    #if (STM32_OSPISEL == STM32_OSPISEL_PER)
      #error "CSI not enabled, required by STM32_OSPISEL"
    #endif

    #if (STM32_SAI1SEL == STM32_SAI1SEL_PER)
      #error "CSI not enabled, required by STM32_SAI1SEL"
    #endif
    #if (STM32_SAI2SEL == STM32_SAI2SEL_PER)
      #error "CSI not enabled, required by STM32_SAI2SEL"
    #endif
  #endif
#endif /* !STM32_CSI_ENABLED */

/*
 * HSI48 related checks.
 */
#if STM32_HSI48_ENABLED
#else /* !STM32_HSI48_ENABLED */

  #if STM32_MCO1SEL == STM32_MCO1SEL_HSI48
    #error "HSI48 not enabled, required by STM32_MCO1SEL"
  #endif

  #if STM32_USBSEL == STM32_USBSEL_HSI48
    #error "HSI48 not enabled, required by STM32_USBSEL"
  #endif

  #if STM32_RNGSEL == STM32_RNGSEL_HSI48
    #error "HSI48 not enabled, required by STM32_RNGSEL"
  #endif

#endif /* !STM32_HSI48_ENABLED */

/*
 * HSI related checks.
 */
#if STM32_HSI_ENABLED
#else /* !STM32_HSI_ENABLED */

  #if STM32_ACTIVATE_PLL1 && (STM32_PLL1SRC == STM32_PLL1SRC_HSI))
    #error "HSI not enabled, required by STM32_PLL1SRC"
  #endif
  #if STM32_ACTIVATE_PLL2 && (STM32_PLL1SRC == STM32_PLL2SRC_HSI))
    #error "HSI not enabled, required by STM32_PLL2SRC"
  #endif
  #if STM32_ACTIVATE_PLL3 && (STM32_PLL1SRC == STM32_PLL3SRC_HSI))
    #error "HSI not enabled, required by STM32_PLL3SRC"
  #endif

  #if STM32_SW == STM32_SW_HSI
    #error "HSI not enabled, required by STM32_SW"
  #endif

  #if STM32_MCO1SEL == STM32_MCO1SEL_HSI
    #error "HSI not enabled, required by STM32_MCO1SEL"
  #endif

  #if (STM32_USART1SEL == STM32_USART1SEL_HSI)
    #error "HSI not enabled, required by STM32_USART1SEL"
  #endif
  #if (STM32_USART2SEL == STM32_USART2SEL_HSI)
    #error "HSI not enabled, required by STM32_USART2SEL"
  #endif
  #if (STM32_USART3SEL == STM32_USART3SEL_HSI)
    #error "HSI not enabled, required by STM32_USART3SEL"
  #endif
  #if (STM32_UART4SEL == STM32_UART4SEL_HSI)
    #error "HSI not enabled, required by STM32_UART4SEL"
  #endif
  #if (STM32_UART5SEL == STM32_UART5SEL_HSI)
    #error "HSI not enabled, required by STM32_UART5SEL"
  #endif
  #if (STM32_USART6SEL == STM32_USART6SEL_HSI)
    #error "HSI not enabled, required by STM32_USART6SEL"
  #endif
  #if (STM32_UART7SEL == STM32_UART7SEL_HSI)
    #error "HSI not enabled, required by STM32_UART7SEL"
  #endif
  #if (STM32_UART8SEL == STM32_UART8SEL_HSI)
    #error "HSI not enabled, required by STM32_UART8SEL"
  #endif
  #if (STM32_UART9SEL == STM32_UART9SEL_HSI)
    #error "HSI not enabled, required by STM32_UART9SEL"
  #endif
  #if (STM32_USART10SEL == STM32_USART10SEL_HSI)
    #error "HSI not enabled, required by STM32_USART10SEL"
  #endif
  #if (STM32_USART11SEL == STM32_USART11SEL_HSI)
    #error "HSI not enabled, required by STM32_USART11SEL"
  #endif
  #if (STM32_UART12SEL == STM32_UART12SEL_HSI)
    #error "HSI not enabled, required by STM32_UART12SEL"
  #endif
  #if (STM32_LPUART1SEL == STM32_LPUART1SEL_HSI)
    #error "HSI not enabled, required by STM32_LPUART1SEL"
  #endif

  #if (STM32_SPI4SEL == STM32_SPI4SEL_HSI)
    #error "HSI not enabled, required by STM32_SPI4SEL"
  #endif
  #if (STM32_SPI5SEL == STM32_SPI5SEL_HSI)
    #error "HSI not enabled, required by STM32_SPI5SEL"
  #endif
  #if (STM32_SPI6SEL == STM32_SPI6SEL_HSI)
    #error "HSI not enabled, required by STM32_SPI6SEL"
  #endif

  #if (STM32_I2C1SEL == STM32_I2C1SEL_HSI)
    #error "HSI not enabled, required by STM32_I2C1SEL"
  #endif
  #if (STM32_I2C2SEL == STM32_I2C2SEL_HSI)
    #error "HSI not enabled, required by STM32_I2C2SEL"
  #endif
  #if (STM32_I2C3SEL == STM32_I2C3SEL_HSI)
    #error "HSI not enabled, required by STM32_I2C3SEL"
  #endif
  #if (STM32_I2C4SEL == STM32_I2C4SEL_HSI)
    #error "HSI not enabled, required by STM32_I2C4SEL"
  #endif

  #if (STM32_I3C1SEL == STM32_I3C1SEL_HSI)
    #error "HSI not enabled, required by STM32_I3C1SEL"
  #endif

  #if (STM32_ADCDACSEL == STM32_ADCDACSEL_HSI)
    #error "HSI not enabled, required by STM32_ADCDACSEL"
  #endif

  #if STM32_PERSEL == STM32_PERSEL_HSI
    #if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_PER)
      #error "HSI not enabled, required by STM32_LPTIM1SEL"
    #endif
    #if (STM32_LPTIM2SEL == STM32_LPTIM2SEL_PER)
      #error "HSI not enabled, required by STM32_LPTIM2SEL"
    #endif
    #if (STM32_LPTIM3SEL == STM32_LPTIM3SEL_PER)
      #error "HSI not enabled, required by STM32_LPTIM3SEL"
    #endif
    #if (STM32_LPTIM4SEL == STM32_LPTIM4SEL_PER)
      #error "HSI not enabled, required by STM32_LPTIM4SEL"
    #endif
    #if (STM32_LPTIM5SEL == STM32_LPTIM5SEL_PER)
      #error "HSI not enabled, required by STM32_LPTIM5SEL"
    #endif
    #if (STM32_LPTIM6SEL == STM32_LPTIM6SEL_PER)
      #error "HSI not enabled, required by STM32_LPTIM6SEL"
    #endif

    #if (STM32_SPI1SEL == STM32_SPI1SEL_PER)
      #error "HSI not enabled, required by STM32_SPI1SEL"
    #endif
    #if (STM32_SPI2SEL == STM32_SPI2SEL_PER)
      #error "HSI not enabled, required by STM32_SPI2SEL"
    #endif
    #if (STM32_SPI3SEL == STM32_SPI3SEL_PER)
      #error "HSI not enabled, required by STM32_SPI3SEL"
    #endif

    #if (STM32_OSPISEL == STM32_OSPISEL_PER)
      #error "HSI not enabled, required by STM32_OSPISEL"
    #endif

    #if (STM32_SAI1SEL == STM32_SAI1SEL_PER)
      #error "HSI not enabled, required by STM32_SAI1SEL"
    #endif
    #if (STM32_SAI2SEL == STM32_SAI2SEL_PER)
      #error "HSI not enabled, required by STM32_SAI2SEL"
    #endif
  #endif

#endif /* !STM32_HSI_ENABLED */

/*
 * LSE related checks.
 */
#if STM32_LSE_ENABLED
#else /* !STM32_LSE_ENABLED */

  #if STM32_MCO1SEL == STM32_MCO1SEL_LSE
    #error "LSE not enabled, required by STM32_MCO1SEL"
  #endif

  #if (STM32_USART1SEL == STM32_USART1SEL_LSE)
    #error "LSE not enabled, required by STM32_USART1SEL"
  #endif
  #if (STM32_USART2SEL == STM32_USART2SEL_LSE)
    #error "LSE not enabled, required by STM32_USART2SEL"
  #endif
  #if (STM32_USART3SEL == STM32_USART3SEL_LSE)
    #error "LSE not enabled, required by STM32_USART3SEL"
  #endif
  #if (STM32_UART4SEL == STM32_UART4SEL_LSE)
    #error "LSE not enabled, required by STM32_UART4SEL"
  #endif
  #if (STM32_UART5SEL == STM32_UART5SEL_LSE)
    #error "LSE not enabled, required by STM32_UART5SEL"
  #endif
  #if (STM32_USART6SEL == STM32_USART6SEL_LSE)
    #error "LSE not enabled, required by STM32_USART6SEL"
  #endif
  #if (STM32_UART7SEL == STM32_UART7SEL_LSE)
    #error "LSE not enabled, required by STM32_UART7SEL"
  #endif
  #if (STM32_UART8SEL == STM32_UART8SEL_LSE)
    #error "LSE not enabled, required by STM32_UART8SEL"
  #endif
  #if (STM32_UART9SEL == STM32_UART9SEL_LSE)
    #error "LSE not enabled, required by STM32_UART9SEL"
  #endif
  #if (STM32_USART10SEL == STM32_USART10SEL_LSE)
    #error "LSE not enabled, required by STM32_USART10SEL"
  #endif
  #if (STM32_USART11SEL == STM32_USART11SEL_LSE)
    #error "LSE not enabled, required by STM32_USART11SEL"
  #endif
  #if (STM32_UART12SEL == STM32_UART12SEL_LSE)
    #error "LSE not enabled, required by STM32_UART12SEL"
  #endif
  #if (STM32_LPUART1SEL == STM32_LPUART1SEL_LSE)
    #error "LSE not enabled, required by STM32_LPUART1SEL"
  #endif

  #if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM1SEL"
  #endif
  #if (STM32_LPTIM2SEL == STM32_LPTIM2SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM2SEL"
  #endif
  #if (STM32_LPTIM3SEL == STM32_LPTIM3SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM3SEL"
  #endif
  #if (STM32_LPTIM4SEL == STM32_LPTIM4SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM4SEL"
  #endif
  #if (STM32_LPTIM5SEL == STM32_LPTIM5SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM5SEL"
  #endif
  #if (STM32_LPTIM6SEL == STM32_LPTIM6SEL_LSE)
    #error "LSE not enabled, required by STM32_LPTIM6SEL"
  #endif

  #if STM32_SYSTICKSEL == STM32_SYSTICKSEL_LSE
    #error "LSE not enabled, required by STM32_SYSTICKSEL"
  #endif

  #if STM32_DACSEL == STM32_DACSEL_LSE
    #error "LSE not enabled, required by STM32_DACSEL"
  #endif

  #if STM32_CECSEL == STM32_CECSEL_LSE
    #error "LSE not enabled, required by STM32_CECSEL"
  #endif

  #if STM32_RTCSEL == STM32_RTCSEL_LSE
    #error "LSE not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_LSCOSEL == STM32_LSCOSEL_LSE
    #error "LSE not enabled, required by STM32_LSCOSEL"
  #endif

#endif /* !STM32_LSE_ENABLED */

/*
 * HSE related checks.
 */
#if STM32_HSE_ENABLED
#else /* !STM32_HSE_ENABLED */

  #if STM32_ACTIVATE_PLL1 && (STM32_PLL1SRC == STM32_PLL1SRC_HSE)
    #error "HSE not enabled, required by STM32_PLL1SRC"
  #endif
  #if STM32_ACTIVATE_PLL2 && (STM32_PLL1SRC == STM32_PLL2SRC_HSE)
    #error "HSE not enabled, required by STM32_PLL2SRC"
  #endif
  #if STM32_ACTIVATE_PLL3 && (STM32_PLL1SRC == STM32_PLL3SRC_HSE)
    #error "HSE not enabled, required by STM32_PLL3SRC"
  #endif

  #if STM32_SW == STM32_SW_HSE
    #error "HSE not enabled, required by STM32_SW"
  #endif

  #if STM32_MCO1SEL == STM32_MCO1SEL_HSE
    #error "HSE not enabled, required by STM32_MCO1SEL"
  #endif

  #if STM32_MCO2SEL == STM32_MCO2SEL_HSE
    #error "HSE not enabled, required by STM32_MCO2SEL"
  #endif

  #if (STM32_SPI4SEL == STM32_SPI4SEL_HSE)
    #error "HSE not enabled, required by STM32_SPI4SEL"
  #endif
  #if (STM32_SPI5SEL == STM32_SPI5SEL_HSE)
    #error "HSE not enabled, required by STM32_SPI5SEL"
  #endif
  #if (STM32_SPI6SEL == STM32_SPI6SEL_HSE)
    #error "HSE not enabled, required by STM32_SPI6SEL"
  #endif

  #if STM32_FDCANSEL == STM32_FDCANSEL_HSE
    #error "HSE not enabled, required by STM32_FDCANSEL"
  #endif

  #if STM32_RTCSEL == STM32_RTCSEL_HSEDIV
    #error "HSE not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_CKPERSEL == STM32_CKPERSEL_HSE
    #if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_HSE)
      #error "HSE not enabled, required by STM32_LPTIM1SEL"
    #endif
    #if (STM32_LPTIM2SEL == STM32_LPTIM2SEL_HSE)
      #error "HSE not enabled, required by STM32_LPTIM2SEL"
    #endif
    #if (STM32_LPTIM3SEL == STM32_LPTIM3SEL_HSE)
      #error "HSE not enabled, required by STM32_LPTIM3SEL"
    #endif
    #if (STM32_LPTIM4SEL == STM32_LPTIM4SEL_HSE)
      #error "HSE not enabled, required by STM32_LPTIM4SEL"
    #endif
    #if (STM32_LPTIM5SEL == STM32_LPTIM5SEL_HSE)
      #error "HSE not enabled, required by STM32_LPTIM5SEL"
    #endif
    #if (STM32_LPTIM6SEL == STM32_LPTIM6SEL_HSE)
      #error "HSE not enabled, required by STM32_LPTIM6SEL"
    #endif

    #if (STM32_SPI1SEL == STM32_SPI1SEL_HSE)
      #error "HSE not enabled, required by STM32_SPI1SEL"
    #endif
    #if (STM32_SPI2SEL == STM32_SPI2SEL_HSE)
      #error "HSE not enabled, required by STM32_SPI2SEL"
    #endif
    #if (STM32_SPI3SEL == STM32_SPI3SEL_HSE)
      #error "HSE not enabled, required by STM32_SPI3SEL"
    #endif

    #if (STM32_OSPISEL == STM32_OSPISEL_HSE)
      #error "HSE not enabled, required by STM32_OSPISEL"
    #endif

    #if (STM32_SAI1SEL == STM32_SAI1SEL_HSE)
      #error "HSE not enabled, required by STM32_SAI1SEL"
    #endif
    #if (STM32_SAI2SEL == STM32_SAI2SEL_HSE)
      #error "HSE not enabled, required by STM32_SAI2SEL"
    #endif
  #endif
#endif /* !STM32_HSE_ENABLED */

/**
 * @brief   PLL1 input clock frequency.
 */
#if (STM32_PLL1SRC == STM32_PLL1SRC_HSE) || defined(__DOXYGEN__)
  #define STM32_PLL1IN              STM32_HSECLK

#elif STM32_PLL1SRC == STM32_PLL1SRC_CSI
  #define STM32_PLL1IN              STM32_CSICLK

#elif STM32_PLL1SRC == STM32_PLL1SRC_HSI
  #define STM32_PLL1IN              STM32_HSICLK

#elif STM32_PLL1SRC == STM32_PLL1SRC_NOCLOCK
  #define STM32_PLL1IN              0

#else
  #error "invalid STM32_PLL1SRC value specified"
#endif

/**
 * @brief   PLL2 input clock frequency.
 */
#if (STM32_PLL2SRC == STM32_PLL2SRC_HSE) || defined(__DOXYGEN__)
  #define STM32_PLL2IN              STM32_HSECLK

#elif STM32_PLL2SRC == STM32_PLL2SRC_CSI
  #define STM32_PLL2IN              STM32_CSICLK

#elif STM32_PLL2SRC == STM32_PLL2SRC_HSI
  #define STM32_PLL2IN              STM32_HSICLK

#elif STM32_PLL2SRC == STM32_PLL2SRC_NOCLOCK
  #define STM32_PLL2IN              0

#else
  #error "invalid STM32_PLL2SRC value specified"
#endif

/**
 * @brief   PLL3 input clock frequency.
 */
#if (STM32_PLL3SRC == STM32_PLL3SRC_HSE) || defined(__DOXYGEN__)
  #define STM32_PLL3IN              STM32_HSECLK

#elif STM32_PLL3SRC == STM32_PLL3SRC_CSI
  #define STM32_PLL3IN              STM32_CSICLK

#elif STM32_PLL3SRC == STM32_PLL3SRC_HSI
  #define STM32_PLL3IN              STM32_HSICLK

#elif STM32_PLL3SRC == STM32_PLL3SRC_NOCLOCK
  #define STM32_PLL3IN              0

#else
  #error "invalid STM32_PLL3SRC value specified"
#endif

/**
 * @brief   STM32_PLL1PEN field.
 */
#if (STM32_SW           == STM32_SW_PLL1P)          ||                      \
    (STM32_MCO1SEL      == STM32_MCO1SEL_PLL1P)     ||                      \
    (STM32_MCO2SEL      == STM32_MCO2SEL_PLL1P)     ||                      \
    defined(__DOXYGEN__)
  #define STM32_PLL1PEN             (1U << 16)

#else
  #define STM32_PLL1PEN             (0U << 16)
#endif

/**
 * @brief   STM32_PLL1QEN field.
 */
#if (STM32_SPI1SEL      == STM32_SPI1SEL_PLL1Q)     ||                      \
    (STM32_SPI2SEL      == STM32_SPI2SEL_PLL1Q)     ||                      \
    (STM32_SPI3SEL      == STM32_SPI3SEL_PLL1Q)     ||                      \
    (STM32_OSPISEL      == STM32_OSPISEL_PLL1Q)     ||                      \
    (STM32_USBSEL       == STM32_USBSEL_PLL1Q)      ||                      \
    (STM32_SDMMC1SEL    == STM32_SDMMC1SEL_PLL1Q)   ||                      \
    (STM32_SDMMC2SEL    == STM32_SDMMC2SEL_PLL1Q)   ||                      \
    (STM32_RNGSEL       == STM32_RNGSEL_PLL1Q)      ||                      \
    (STM32_FDCANSEL     == STM32_FDCANSEL_PLL1Q)    ||                      \
    (STM32_SAI1SEL      == STM32_SAI1SEL_PLL1Q)     ||                      \
    (STM32_SAI2SEL      == STM32_SAI2SEL_PLL1Q)     ||                      \
    defined(__DOXYGEN__)
  #define STM32_PLL1QEN             (1U << 17)

#else
  #define STM32_PLL1QEN             (0U << 17)
#endif

/**
 * @brief   STM32_PLL1REN field.
 */
#if FALSE                                           ||                      \
    defined(__DOXYGEN__)
  #define STM32_PLL1REN             (1U << 18)

#else
  #define STM32_PLL1REN             (0U << 18)
#endif

/**
 * @brief   STM32_PLL2PEN field.
 */
#if (STM32_SW           == STM32_SW_PLL1P)          ||                      \
    (STM32_MCO2SEL      == STM32_MCO2SEL_PLL1P)     ||                      \
    (STM32_LPTIM1SEL    == STM32_LPTIM1SEL_PLL2P)   ||                      \
    (STM32_LPTIM2SEL    == STM32_LPTIM2SEL_PLL2P)   ||                      \
    (STM32_LPTIM3SEL    == STM32_LPTIM3SEL_PLL2P)   ||                      \
    (STM32_LPTIM4SEL    == STM32_LPTIM4SEL_PLL2P)   ||                      \
    (STM32_LPTIM5SEL    == STM32_LPTIM5SEL_PLL2P)   ||                      \
    (STM32_LPTIM6SEL    == STM32_LPTIM6SEL_PLL2P)   ||                      \
    (STM32_SPI1SEL      == STM32_SPI1SEL_PLL2P)     ||                      \
    (STM32_SPI2SEL      == STM32_SPI2SEL_PLL2P)     ||                      \
    (STM32_SPI3SEL      == STM32_SPI3SEL_PLL2P)     ||                      \
    (STM32_SPI4SEL      == STM32_SPI4SEL_PLL2P)     ||                      \
    (STM32_SPI5SEL      == STM32_SPI5SEL_PLL2P)     ||                      \
    (STM32_SPI6SEL      == STM32_SPI6SEL_PLL2P)     ||                      \
    (STM32_SAI1SEL      == STM32_SAI1SEL_PLL2P)     ||                      \
    (STM32_SAI2SEL      == STM32_SAI2SEL_PLL2P)     ||                      \
    defined(__DOXYGEN__)
  #define STM32_PLL2PEN             (1U << 16)

#else
  #define STM32_PLL2PEN             (0U << 16)
#endif

/**
 * @brief   STM32_PLL2QEN field.
 */
#if (STM32_USART1SEL    == STM32_USART1SEL_PLL2Q)   ||                      \
    (STM32_USART2SEL    == STM32_USART2SEL_PLL2Q)   ||                      \
    (STM32_USART3SEL    == STM32_USART3SEL_PLL2Q)   ||                      \
    (STM32_UART4SEL     == STM32_UART4SEL_PLL2Q)    ||                      \
    (STM32_UART5SEL     == STM32_UART5SEL_PLL2Q)    ||                      \
    (STM32_USART6SEL    == STM32_USART6SEL_PLL2Q)   ||                      \
    (STM32_UART7SEL     == STM32_UART7SEL_PLL2Q)    ||                      \
    (STM32_UART8SEL     == STM32_UART8SEL_PLL2Q)    ||                      \
    (STM32_UART9SEL     == STM32_UART9SEL_PLL2Q)    ||                      \
    (STM32_USART10SEL   == STM32_USART10SEL_PLL2Q)  ||                      \
    (STM32_USART11SEL   == STM32_USART11SEL_PLL2Q)  ||                      \
    (STM32_UART12SEL    == STM32_UART12SEL_PLL2Q)   ||                      \
    (STM32_LPUART1SEL   == STM32_LPUART1SEL_PLL2Q)  ||                      \
    (STM32_FDCANSEL     == STM32_FDCANSEL_PLL2Q)    ||                      \
    defined(__DOXYGEN__)
  #define STM32_PLL2QEN             (1U << 17)

#else
  #define STM32_PLL2QEN             (0U << 17)
#endif

/**
 * @brief   STM32_PLL2REN field.
 */
#if (STM32_OSPISEL      == STM32_OSPISEL_PLL2R)     ||                      \
    (STM32_SDMMC1SEL    == STM32_SDMMC1SEL_PLL2R)   ||                      \
    (STM32_SDMMC2SEL    == STM32_SDMMC2SEL_PLL2R)   ||                      \
    (STM32_ADCDACSEL    == STM32_ADCDACSEL_PLL2R)   ||                      \
    defined(__DOXYGEN__)
  #define STM32_PLL2REN             (1U << 18)

#else
  #define STM32_PLL2REN             (0U << 18)
#endif

/**
 * @brief   STM32_PLL3PEN field.
 */
#if (STM32_SPI1SEL      == STM32_SPI1SEL_PLL3P)     ||                      \
    (STM32_SPI2SEL      == STM32_SPI2SEL_PLL3P)     ||                      \
    (STM32_SPI3SEL      == STM32_SPI3SEL_PLL3P)     ||                      \
    (STM32_SPI4SEL      == STM32_SPI4SEL_PLL3P)     ||                      \
    (STM32_SPI5SEL      == STM32_SPI5SEL_PLL3P)     ||                      \
    (STM32_SPI6SEL      == STM32_SPI6SEL_PLL3P)     ||                      \
    (STM32_SAI1SEL      == STM32_SAI1SEL_PLL3P)     ||                      \
    (STM32_SAI2SEL      == STM32_SAI2SEL_PLL3P)     ||                      \
    defined(__DOXYGEN__)
  #define STM32_PLL3PEN             (1U << 16)

#else
  #define STM32_PLL3PEN             (0U << 16)
#endif

/**
 * @brief   STM32_PLL3QEN field.
 */
#if (STM32_USART1SEL    == STM32_USART1SEL_PLL3Q)   ||                      \
    (STM32_USART2SEL    == STM32_USART2SEL_PLL3Q)   ||                      \
    (STM32_USART3SEL    == STM32_USART3SEL_PLL3Q)   ||                      \
    (STM32_UART4SEL     == STM32_UART4SEL_PLL3Q)    ||                      \
    (STM32_UART5SEL     == STM32_UART5SEL_PLL3Q)    ||                      \
    (STM32_USART6SEL    == STM32_USART6SEL_PLL3Q)   ||                      \
    (STM32_UART7SEL     == STM32_UART7SEL_PLL3Q)    ||                      \
    (STM32_UART8SEL     == STM32_UART8SEL_PLL3Q)    ||                      \
    (STM32_UART9SEL     == STM32_UART9SEL_PLL3Q)    ||                      \
    (STM32_USART10SEL   == STM32_USART10SEL_PLL3Q)  ||                      \
    (STM32_USART11SEL   == STM32_USART11SEL_PLL3Q)  ||                      \
    (STM32_UART12SEL    == STM32_UART12SEL_PLL3Q)   ||                      \
    (STM32_LPUART1SEL   == STM32_LPUART1SEL_PLL3Q)  ||                      \
    (STM32_USBSEL       == STM32_USBSEL_PLL3Q)  ||                          \
    defined(__DOXYGEN__)
  #define STM32_PLL3QEN             (1U << 17)

#else
  #define STM32_PLL3QEN             (0U << 17)
#endif

/**
 * @brief   STM32_PLL3REN field.
 */
#if (STM32_LPTIM1SEL    == STM32_LPTIM1SEL_PLL3R)   ||                      \
    (STM32_LPTIM2SEL    == STM32_LPTIM2SEL_PLL3R)   ||                      \
    (STM32_LPTIM3SEL    == STM32_LPTIM3SEL_PLL3R)   ||                      \
    (STM32_LPTIM4SEL    == STM32_LPTIM4SEL_PLL3R)   ||                      \
    (STM32_LPTIM5SEL    == STM32_LPTIM5SEL_PLL3R)   ||                      \
    (STM32_LPTIM6SEL    == STM32_LPTIM6SEL_PLL3R)   ||                      \
    (STM32_I2C1SEL      == STM32_I2C1SEL_PLL3R)     ||                      \
    (STM32_I2C2SEL      == STM32_I2C2SEL_PLL3R)     ||                      \
    (STM32_I2C3SEL      == STM32_I2C3SEL_PLL3R)     ||                      \
    (STM32_I2C4SEL      == STM32_I2C4SEL_PLL3R)     ||                      \
    (STM32_I3C1SEL      == STM32_I3C1SEL_PLL3R)     ||                      \
    defined(__DOXYGEN__)
  #define STM32_PLL3REN             (1U << 18)

#else
  #define STM32_PLL3REN             (0U << 18)
#endif

/* Inclusion of PLL-related checks and calculations.*/
#include <stm32_pll1.inc>
#include <stm32_pll2.inc>
#include <stm32_pll3.inc>

/**
 * @brief   System clock source.
 */
#if (STM32_SW == STM32_SW_HSI) || defined(__DOXYGEN__)
  #define STM32_SYSCLK              STM32_HSICLK

#elif STM32_SW == STM32_SW_CSI
  #define STM32_SYSCLK              STM32_CSICLK

#elif STM32_SW == STM32_SW_HSE
  #define STM32_SYSCLK              STM32_HSECLK

#elif STM32_SW == STM32_SW_PLL1P
  #define STM32_SYSCLK              STM32_PLL1_P_CLKOUT

#else
  #error "invalid STM32_SW value specified"
#endif

/* Bus handlers.*/
#include "stm32_ahb.inc"
#include "stm32_apb1.inc"
#include "stm32_apb2.inc"
#include "stm32_apb3.inc"

/**
 * @brief   STOPWUCK source clock.
 */
#if (STM32_STOPWUCK == STM32_STOPWUCK_HSI) || defined(__DOXYGEN__)

#elif STM32_STOPWUCK == STM32_STOPWUCK_CSI

#else
  #error "invalid STM32_STOPWUCK value specified"
#endif

/**
 * @brief   STOPKERWUCK source clock.
 */
#if (STM32_STOPKERWUCK == STM32_STOPKERWUCK_HSI) || defined(__DOXYGEN__)

#elif STM32_STOPKERWUCK == STM32_STOPKERWUCK_CSI

#else
  #error "invalid STM32_STOPKERWUCK value specified"
#endif

/**
 * @brief   RTCPRE clock frequency.
 */
#if (STM32_RTCPRE_VALUE == STM32_RTCPRE_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_RTCPRECLK           0U

#elif (STM32_RTCPRE_VALUE >= 2) && (STM32_RTCPRE_VALUE <= 63)
  #define STM32_RTCPRECLK           (STM32_HSECLK / STM32_RTCPRE_VALUE)

#else
  #error "invalid STM32_RTCPRECLK_VALUE value specified"
#endif

/**
 * @brief   RTCPRE field.
 */
#define STM32_RTCPRE                STM32_RTCPRE_FIELD(STM32_RTCPRE_VALUE)

/**
 * @brief   MCO1 source clock.
 */
#if (STM32_MCO1SEL == STM32_MCO1SEL_HSI) || defined(__DOXYGEN__)
  #define STM32_MCO1DIVCLK          STM32_HSICLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_LSE
  #define STM32_MCO1DIVCLK          STM32_LSECLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_HSI
  #define STM32_MCO1DIVCLK          STM32_HSICLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_HSE
  #define STM32_MCO1DIVCLK          STM32_HSECLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_PLL1P
  #define STM32_MCO1DIVCLK          STM32_PLL1_P_CLKOUT

#elif STM32_MCO1SEL == STM32_MCO1SEL_HSI48
  #define STM32_MCO1DIVCLK          STM32_HSI48CLK

#else
  #error "invalid STM32_MCO1SEL value specified"
#endif

/**
 * @brief   MCO1 output pin clock frequency.
 */
#if (STM32_MCO1PRE_VALUE == STM32_MCO1PRE_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_MCO1CLK             0U

#elif (STM32_MCO1PRE_VALUE > STM32_MCO1PRE_NOCLOCK) &&                      \
      (STM32_MCO1PRE_VALUE <= 15)
  #define STM32_MCO1CLK             (STM32_MCO1DIVCLK / STM32_MCO1PRE_VALUE)

#else
#error "invalid STM32_MCO1PRE_VALUE value specified"
#endif

/**
 * @brief   MCO1PRE field.
 */
#define STM32_MCO1PRE               STM32_MCO1PRE_FIELD(STM32_MCO1PRE_VALUE)

/**
 * @brief   MCO2 source clock.
 */
#if (STM32_MCO2SEL == STM32_MCO2SEL_SYSCLK) || defined(__DOXYGEN__)
  #define STM32_MCO2DIVCLK          STM32_SYSCLK

#elif STM32_MCO2SEL == STM32_MCO2SEL_PLL2P
  #define STM32_MCO2DIVCLK          STM32_PLL2_P_CLKOUT

#elif STM32_MCO2SEL == STM32_MCO2SEL_HSE
  #define STM32_MCO2DIVCLK          STM32_HSECLK

#elif STM32_MCO2SEL == STM32_MCO2SEL_PLL1P
  #define STM32_MCO2DIVCLK          STM32_PLL1_P_CLKOUT

#elif STM32_MCO2SEL == STM32_MCO2SEL_CSI
  #define STM32_MCO2DIVCLK          STM32_CSICLK

#elif STM32_MCO2SEL == STM32_MCO2SEL_LSI
  #define STM32_MCO2DIVCLK          STM32_LSICLK

#else
  #error "invalid STM32_MCO2SEL value specified"
#endif

/**
 * @brief   MCO2 output pin clock frequency.
 */
#if (STM32_MCO2PRE_VALUE == STM32_MCO2PRE_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_MCO2CLK             0U

#elif (STM32_MCO2PRE_VALUE > STM32_MCO2PRE_NOCLOCK) &&                      \
      (STM32_MCO2PRE_VALUE <= 15)
  #define STM32_MCO2CLK             (STM32_MCO2DIVCLK / STM32_MCO2PRE_VALUE)

#else
#error "invalid STM32_MCO2PRE_VALUE value specified"
#endif

/**
 * @brief   MCO2PRE field.
 */
#define STM32_MCO2PRE               STM32_MCO2PRE_FIELD(STM32_MCO2PRE_VALUE)

/**
 * @brief   RTC clock frequency.
 */
#if (STM32_RTCSEL == STM32_RTCSEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_RTCCLK              0U

#elif STM32_RTCSEL == STM32_RTCSEL_LSE
  #define STM32_RTCCLK              STM32_LSECLK

#elif STM32_RTCSEL == STM32_RTCSEL_LSI
  #define STM32_RTCCLK              STM32_LSICLK

#elif STM32_RTCSEL == STM32_RTCSEL_HSEDIV
  #define STM32_RTCCLK              STM32_RTCPRECLK

#else
  #error "invalid STM32_RTCSEL value specified"
#endif

/**
 * @brief   LSCO clock frequency.
 */
#if (STM32_LSCOSEL == STM32_LSCOSEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_LSCOCLK             0U

#elif STM32_LSCOSEL == STM32_LSCOSEL_LSI
  #define STM32_LSCOCLK             STM32_LSICLK

#elif STM32_LSCOSEL == STM32_LSCOSEL_LSE
  #define STM32_LSCOCLK             STM32_LSECLK

#else
  #error "invalid STM32_LSCOSEL value specified"
#endif

/**
 * @brief   USART1 clock frequency.
 */
#if (STM32_USART1SEL == STM32_USART1SEL_PCLK2) || defined(__DOXYGEN__)
  #define STM32_USART1CLK           hal_lld_get_clock_point(CLK_PCLK2)

#elif STM32_USART1SEL == STM32_USART1SEL_PLL2Q
  #define STM32_USART1CLK           hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_USART1SEL == STM32_USART1SEL_PLL3Q
  #define STM32_USART1CLK           hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USART1SEL == STM32_USART1SEL_HSI
  #define STM32_USART1CLK           STM32_HSICLK

#elif STM32_USART1SEL == STM32_USART1SEL_CSI
  #define STM32_USART1CLK           STM32_CSICLK

#elif STM32_USART1SEL == STM32_USART1SEL_LSE
  #define STM32_USART1CLK           STM32_LSECLK

#else
  #error "invalid source selected for USART1 clock"
#endif

/**
 * @brief   USART2 clock frequency.
 */
#if (STM32_USART2SEL == STM32_USART2SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_USART2CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_USART2SEL == STM32_USART2SEL_PLL2Q
  #define STM32_USART2CLK           hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_USART2SEL == STM32_USART2SEL_PLL3Q
  #define STM32_USART2CLK           hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USART2SEL == STM32_USART2SEL_HSI
  #define STM32_USART2CLK           STM32_HSICLK

#elif STM32_USART2SEL == STM32_USART2SEL_CSI
  #define STM32_USART2CLK           STM32_CSICLK

#elif STM32_USART2SEL == STM32_USART2SEL_LSE
  #define STM32_USART2CLK           STM32_LSECLK

#else
  #error "invalid source selected for USART2 clock"
#endif

/**
 * @brief   USART3 clock frequency.
 */
#if (STM32_USART3SEL == STM32_USART3SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_USART3CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_USART3SEL == STM32_USART3SEL_PLL2Q
  #define STM32_USART3CLK           hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_USART3SEL == STM32_USART3SEL_PLL3Q
  #define STM32_USART3CLK           hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USART3SEL == STM32_USART3SEL_HSI
  #define STM32_USART3CLK           STM32_HSICLK

#elif STM32_USART3SEL == STM32_USART3SEL_CSI
  #define STM32_USART3CLK           STM32_CSICLK

#elif STM32_USART3SEL == STM32_USART3SEL_LSE
  #define STM32_USART3CLK           STM32_LSECLK

#else
  #error "invalid source selected for USART3 clock"
#endif

/**
 * @brief   UART4 clock frequency.
 */
#if (STM32_UART4SEL == STM32_UART4SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART4CLK            hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART4SEL == STM32_UART4SEL_PLL2Q
  #define STM32_UART4CLK            hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_UART4SEL == STM32_UART4SEL_PLL3Q
  #define STM32_UART4CLK            hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_UART4SEL == STM32_UART4SEL_HSI
  #define STM32_UART4CLK            STM32_HSICLK

#elif STM32_UART4SEL == STM32_UART4SEL_CSI
  #define STM32_UART4CLK            STM32_CSICLK

#elif STM32_UART4SEL == STM32_UART4SEL_LSE
  #define STM32_UART4CLK            STM32_LSECLK

#else
  #error "invalid source selected for UART4 clock"
#endif

/**
 * @brief   UART5 clock frequency.
 */
#if (STM32_UART5SEL == STM32_UART5SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART5CLK            hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART5SEL == STM32_UART5SEL_PLL2Q
  #define STM32_UART5CLK            hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_UART5SEL == STM32_UART5SEL_PLL3Q
  #define STM32_UART5CLK            hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_UART5SEL == STM32_UART5SEL_HSI
  #define STM32_UART5CLK            STM32_HSICLK

#elif STM32_UART5SEL == STM32_UART5SEL_CSI
  #define STM32_UART5CLK            STM32_CSICLK

#elif STM32_UART5SEL == STM32_UART5SEL_LSE
  #define STM32_UART5CLK            STM32_LSECLK

#else
  #error "invalid source selected for UART5 clock"
#endif

/**
 * @brief   USART6 clock frequency.
 */
#if (STM32_USART6SEL == STM32_USART6SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_USART6CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_USART6SEL == STM32_USART6SEL_PLL2Q
  #define STM32_USART6CLK           hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_USART6SEL == STM32_USART6SEL_PLL3Q
  #define STM32_USART6CLK           hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USART6SEL == STM32_USART6SEL_HSI
  #define STM32_USART6CLK           STM32_HSICLK

#elif STM32_USART6SEL == STM32_USART6SEL_CSI
  #define STM32_USART6CLK           STM32_CSICLK

#elif STM32_USART6SEL == STM32_USART6SEL_LSE
  #define STM32_USART6CLK           STM32_LSECLK

#else
  #error "invalid source selected for USART6 clock"
#endif

/**
 * @brief   UART7 clock frequency.
 */
#if (STM32_UART7SEL == STM32_UART7SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART7CLK            hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART7SEL == STM32_UART7SEL_PLL2Q
  #define STM32_UART7CLK            hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_UART7SEL == STM32_UART7SEL_PLL3Q
  #define STM32_UART7CLK            hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_UART7SEL == STM32_UART7SEL_HSI
  #define STM32_UART7CLK            STM32_HSICLK

#elif STM32_UART7SEL == STM32_UART7SEL_CSI
  #define STM32_UART7CLK            STM32_CSICLK

#elif STM32_UART7SEL == STM32_UART7SEL_LSE
  #define STM32_UART7CLK            STM32_LSECLK

#else
  #error "invalid source selected for UART7 clock"
#endif

/**
 * @brief   UART8 clock frequency.
 */
#if (STM32_UART8SEL == STM32_UART8SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART8CLK            hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART8SEL == STM32_UART8SEL_PLL2Q
  #define STM32_UART8CLK            hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_UART8SEL == STM32_UART8SEL_PLL3Q
  #define STM32_UART8CLK            hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_UART8SEL == STM32_UART8SEL_HSI
  #define STM32_UART8CLK            STM32_HSICLK

#elif STM32_UART8SEL == STM32_UART8SEL_CSI
  #define STM32_UART8CLK            STM32_CSICLK

#elif STM32_UART8SEL == STM32_UART8SEL_LSE
  #define STM32_UART8CLK            STM32_LSECLK

#else
  #error "invalid source selected for UART8 clock"
#endif

/**
 * @brief   UART9 clock frequency.
 */
#if (STM32_UART9SEL == STM32_UART9SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART9CLK            hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART9SEL == STM32_UART9SEL_PLL2Q
  #define STM32_UART9CLK            hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_UART9SEL == STM32_UART9SEL_PLL3Q
  #define STM32_UART9CLK            hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_UART9SEL == STM32_UART9SEL_HSI
  #define STM32_UART9CLK            STM32_HSICLK

#elif STM32_UART9SEL == STM32_UART9SEL_CSI
  #define STM32_UART9CLK            STM32_CSICLK

#elif STM32_UART9SEL == STM32_UART9SEL_LSE
  #define STM32_UART9CLK            STM32_LSECLK

#else
  #error "invalid source selected for UART9 clock"
#endif

/**
 * @brief   USART10 clock frequency.
 */
#if (STM32_USART10SEL == STM32_USART10SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_USART10CLK          hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_USART10SEL == STM32_USART10SEL_PLL2Q
  #define STM32_USART10CLK          hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_USART10SEL == STM32_USART10SEL_PLL3Q
  #define STM32_USART10CLK          hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USART10SEL == STM32_USART10SEL_HSI
  #define STM32_USART10CLK          STM32_HSICLK

#elif STM32_USART10SEL == STM32_USART10SEL_CSI
  #define STM32_USART10CLK          STM32_CSICLK

#elif STM32_USART10SEL == STM32_USART10SEL_LSE
  #define STM32_USART10CLK          STM32_LSECLK

#else
  #error "invalid source selected for USART10 clock"
#endif

/**
 * @brief   USART11 clock frequency.
 */
#if (STM32_USART11SEL == STM32_USART11SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_USART11CLK          hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_USART11SEL == STM32_USART11SEL_PLL2Q
  #define STM32_USART11CLK          hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_USART11SEL == STM32_USART11SEL_PLL3Q
  #define STM32_USART11CLK          hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USART11SEL == STM32_USART11SEL_HSI
  #define STM32_USART11CLK          STM32_HSICLK

#elif STM32_USART11SEL == STM32_USART11SEL_CSI
  #define STM32_USART11CLK          STM32_CSICLK

#elif STM32_USART11SEL == STM32_USART11SEL_LSE
  #define STM32_USART11CLK          STM32_LSECLK

#else
  #error "invalid source selected for USART11 clock"
#endif

/**
 * @brief   UART12 clock frequency.
 */
#if (STM32_UART12SEL == STM32_UART12SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART12CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART12SEL == STM32_UART12SEL_PLL2Q
  #define STM32_UART12CLK           hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_UART12SEL == STM32_UART12SEL_PLL3Q
  #define STM32_UART12CLK           hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_UART12SEL == STM32_UART12SEL_HSI
  #define STM32_UART12CLK           STM32_HSICLK

#elif STM32_UART12SEL == STM32_UART12SEL_CSI
  #define STM32_UART12CLK           STM32_CSICLK

#elif STM32_UART12SEL == STM32_UART12SEL_LSE
  #define STM32_UART12CLK           STM32_LSECLK

#else
  #error "invalid source selected for UART12 clock"
#endif

/**
 * @brief   LPUART1 clock frequency.
 */
#if (STM32_LPUART1SEL == STM32_LPUART1SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_LPUART1SEL == STM32_LPUART1SEL_PLL2Q
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_LPUART1SEL == STM32_LPUART1SEL_PLL3Q
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_LPUART1SEL == STM32_LPUART1SEL_HSI
  #define STM32_LPUART1CLK          STM32_HSICLK

#elif STM32_LPUART1SEL == STM32_LPUART1SEL_CSI
  #define STM32_LPUART1CLK          STM32_CSICLK

#elif STM32_LPUART1SEL == STM32_LPUART1SEL_LSE
  #define STM32_LPUART1CLK          STM32_LSECLK

#else
  #error "invalid source selected for LPUART1 clock"
#endif

#if (STM32_TIMICSEL == STM32_TIMICSEL_NOCLK) || defined(__DOXYGEN__)
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

#elif STM32_TIMICSEL == STM32_TIMICSEL_INTCLK
  #define STM32_TIM12CCLK           (STM32_HSICLK / 1024)
  #define STM32_TIM15CCLK           (STM32_HSICLK / 8)
  #define STM32_LPTIM2CCLK          (STM32_CSICLK / 128)

#else
  #error "invalid source selected for TIMICSEL clock"
#endif

/**
 * @brief   LPTIM1 clock frequency.
 */
#if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_LPTIM1CLK           hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_PLL2P
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_PLL3R
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSE
  #define STM32_LPTIM1CLK           STM32_LSECLK

#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSI
  #define STM32_LPTIM1CLK           STM32_LSICLK

#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_PER
  #define STM32_LPTIM1CLK           STM32_PERCLK

#else
  #error "invalid source selected for LPTIM1 clock"
#endif

/**
 * @brief   LPTIM2 clock frequency.
 */
#if (STM32_LPTIM2SEL == STM32_LPTIM2SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_LPTIM2CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_PLL2P
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_PLL3R
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_LSE
  #define STM32_LPTIM2CLK           STM32_LSECLK

#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_LSI
  #define STM32_LPTIM2CLK           STM32_LSICLK

#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_PER
  #define STM32_LPTIM2CLK           STM32_PERCLK

#else
  #error "invalid source selected for LPTIM2 clock"
#endif

/**
 * @brief   LPTIM3 clock frequency.
 */
#if (STM32_LPTIM3SEL == STM32_LPTIM3SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_LPTIM3CLK           hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_LPTIM3SEL == STM32_LPTIM3SEL_PLL2P
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_LPTIM3SEL == STM32_LPTIM3SEL_PLL3R
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_LPTIM3SEL == STM32_LPTIM3SEL_LSE
  #define STM32_LPTIM3CLK           STM32_LSECLK

#elif STM32_LPTIM3SEL == STM32_LPTIM3SEL_LSI
  #define STM32_LPTIM3CLK           STM32_LSICLK

#elif STM32_LPTIM3SEL == STM32_LPTIM3SEL_PER
  #define STM32_LPTIM3CLK           STM32_PERCLK

#else
  #error "invalid source selected for LPTIM3 clock"
#endif

/**
 * @brief   LPTIM4 clock frequency.
 */
#if (STM32_LPTIM4SEL == STM32_LPTIM4SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_LPTIM4CLK           hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_LPTIM4SEL == STM32_LPTIM4SEL_PLL2P
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_LPTIM4SEL == STM32_LPTIM4SEL_PLL3R
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_LPTIM4SEL == STM32_LPTIM4SEL_LSE
  #define STM32_LPTIM4CLK           STM32_LSECLK

#elif STM32_LPTIM4SEL == STM32_LPTIM4SEL_LSI
  #define STM32_LPTIM4CLK           STM32_LSICLK

#elif STM32_LPTIM4SEL == STM32_LPTIM4SEL_PER
  #define STM32_LPTIM4CLK           STM32_PERCLK

#else
  #error "invalid source selected for LPTIM4 clock"
#endif

/**
 * @brief   LPTIM5 clock frequency.
 */
#if (STM32_LPTIM5SEL == STM32_LPTIM5SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_LPTIM5CLK           hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_LPTIM5SEL == STM32_LPTIM5SEL_PLL2P
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_LPTIM5SEL == STM32_LPTIM5SEL_PLL3R
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_LPTIM5SEL == STM32_LPTIM5SEL_LSE
  #define STM32_LPTIM5CLK           STM32_LSECLK

#elif STM32_LPTIM5SEL == STM32_LPTIM5SEL_LSI
  #define STM32_LPTIM5CLK           STM32_LSICLK

#elif STM32_LPTIM5SEL == STM32_LPTIM5SEL_PER
  #define STM32_LPTIM5CLK           STM32_PERCLK

#else
  #error "invalid source selected for LPTIM5 clock"
#endif

/**
 * @brief   LPTIM6 clock frequency.
 */
#if (STM32_LPTIM6SEL == STM32_LPTIM6SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_LPTIM6CLK           hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_LPTIM6SEL == STM32_LPTIM6SEL_PLL2P
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_LPTIM6SEL == STM32_LPTIM6SEL_PLL3R
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_LPTIM6SEL == STM32_LPTIM6SEL_LSE
  #define STM32_LPTIM6CLK           STM32_LSECLK

#elif STM32_LPTIM6SEL == STM32_LPTIM6SEL_LSI
  #define STM32_LPTIM6CLK           STM32_LSICLK

#elif STM32_LPTIM6SEL == STM32_LPTIM6SEL_PER
  #define STM32_LPTIM6CLK           STM32_PERCLK

#else
  #error "invalid source selected for LPTIM6 clock"
#endif

/**
 * @brief   SPI1 clock frequency.
 */
#if (STM32_SPI1SEL == STM32_SPI1SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SPI1CLK             hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SPI1SEL == STM32_SPI1SEL_PLL2P
  #define STM32_SPI1CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SPI1SEL == STM32_SPI1SEL_PLL3P
  #define STM32_SPI1CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SPI1SEL == STM32_SPI1SEL_AUDIOCLK
  #define STM32_SPI1CLK             0 /* TODO board.h */

#elif STM32_SPI1SEL == STM32_SPI2SEL_PER
  #define STM32_SPI1CLK             STM32_PERCLK

#else
  #error "invalid source selected for SPI1 clock"
#endif

/**
 * @brief   SPI2 clock frequency.
 */
#if (STM32_SPI2SEL == STM32_SPI2SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SPI2CLK             hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SPI2SEL == STM32_SPI2SEL_PLL2P
  #define STM32_SPI2CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SPI2SEL == STM32_SPI2SEL_PLL3P
  #define STM32_SPI2CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SPI2SEL == STM32_SPI2SEL_AUDIOCLK
  #define STM32_SPI2CLK             0 /* TODO board.h */

#elif STM32_SPI2SEL == STM32_SPI2SEL_PER
  #define STM32_SPI2CLK             STM32_PERCLK

#else
  #error "invalid source selected for SPI2 clock"
#endif

/**
 * @brief   SPI3 clock frequency.
 */
#if (STM32_SPI3SEL == STM32_SPI3SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SPI3CLK             hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SPI3SEL == STM32_SPI3SEL_PLL2P
  #define STM32_SPI3CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SPI3SEL == STM32_SPI3SEL_PLL3P
  #define STM32_SPI3CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SPI3SEL == STM32_SPI3SEL_AUDIOCLK
  #define STM32_SPI3CLK             0 /* TODO board.h */

#elif STM32_SPI3SEL == STM32_SPI2SEL_PER
  #define STM32_SPI3CLK             STM32_PERCLK

#else
  #error "invalid source selected for SPI3 clock"
#endif

/**
 * @brief   SPI4 clock frequency.
 */
#if (STM32_SPI4SEL == STM32_SPI4SEL_PCLK2) || defined(__DOXYGEN__)
  #define STM32_SPI4CLK             hal_lld_get_clock_point(CLK_PCLK2)

#elif STM32_SPI4SEL == STM32_SPI4SEL_PLL2P
  #define STM32_SPI4CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SPI4SEL == STM32_SPI4SEL_PLL3P
  #define STM32_SPI4CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SPI4SEL == STM32_SPI4SEL_HSI
  #define STM32_SPI4CLK             STM32_HSICLK

#elif STM32_SPI4SEL == STM32_SPI4SEL_CSI
  #define STM32_SPI4CLK             STM32_CSICLK

#elif STM32_SPI4SEL == STM32_SPI4SEL_HSE
  #define STM32_SPI4CLK             STM32_HSECLK

#else
  #error "invalid source selected for SPI4 clock"
#endif

/**
 * @brief   SPI5 clock frequency.
 */
#if (STM32_SPI5SEL == STM32_SPI5SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_SPI5CLK             hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_SPI5SEL == STM32_SPI5SEL_PLL2P
  #define STM32_SPI5CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SPI5SEL == STM32_SPI5SEL_PLL3P
  #define STM32_SPI5CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SPI5SEL == STM32_SPI5SEL_HSI
  #define STM32_SPI5CLK             STM32_HSICLK

#elif STM32_SPI5SEL == STM32_SPI5SEL_CSI
  #define STM32_SPI5CLK             STM32_CSICLK

#elif STM32_SPI5SEL == STM32_SPI5SEL_HSE
  #define STM32_SPI5CLK             STM32_HSECLK

#else
  #error "invalid source selected for SPI5 clock"
#endif

/**
 * @brief   SPI6 clock frequency.
 */
#if (STM32_SPI6SEL == STM32_SPI6SEL_PCLK2) || defined(__DOXYGEN__)
  #define STM32_SPI6CLK             hal_lld_get_clock_point(CLK_PCLK2)

#elif STM32_SPI6SEL == STM32_SPI6SEL_PLL2P
  #define STM32_SPI6CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SPI6SEL == STM32_SPI6SEL_PLL3P
  #define STM32_SPI6CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SPI6SEL == STM32_SPI6SEL_HSI
  #define STM32_SPI6CLK             STM32_HSICLK

#elif STM32_SPI6SEL == STM32_SPI6SEL_CSI
  #define STM32_SPI6CLK             STM32_CSICLK

#elif STM32_SPI6SEL == STM32_SPI6SEL_HSE
  #define STM32_SPI6CLK             STM32_HSECLK

#else
  #error "invalid source selected for SPI6 clock"
#endif

/**
 * @brief   OSPI clock frequency.
 */
#if (STM32_OSPISEL == STM32_OSPISEL_HCLK4) || defined(__DOXYGEN__)
  #define STM32_OSPICLK             hal_lld_get_clock_point(CLK_HCLK)

#elif STM32_OSPISEL == STM32_OSPISEL_PLL1Q
  #define STM32_OSPICLK             hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_OSPISEL == STM32_OSPISEL_PLL2R
  #define STM32_OSPICLK             hal_lld_get_clock_point(CLK_PLL2RCLK)

#elif STM32_OSPISEL == STM32_OSPISEL_PER
  #define STM32_OSPICLK             STM32_PERCLK

#else
  #error "invalid source selected for OSPI clock"
#endif

/**
 * @brief   SYSTICK clock frequency.
 */
#if (STM32_SYSTICKSEL == STM32_SYSTICKSEL_HCLKDIV8) || defined(__DOXYGEN__)
  #define STM32_SYSTICKCLK          (hal_lld_get_clock_point(CLK_HCLK) / 8)

#elif STM32_SYSTICKSEL == STM32_SYSTICKSEL_LSI
  #define STM32_SYSTICKCLK          STM32_LSICLK

#elif STM32_SYSTICKSEL == STM32_SYSTICKSEL_LSE
  #define STM32_SYSTICKCLK          STM32_LSECLK

#elif STM32_SYSTICKSEL == STM32_SYSTICKSEL_NOCLOCK
  #define STM32_SYSTICKCLK          0

#else
  #error "invalid source selected for STM32_SYSTICKSEL clock"
#endif

/**
 * @brief   USB clock frequency.
 */
#if (STM32_USBSEL == STM32_USBSEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_USBCLK              0

#elif STM32_USBSEL == STM32_USBSEL_PLL1Q
  #define STM32_USBCLK              hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_USBSEL == STM32_USBSEL_PLL3Q
  #define STM32_USBCLK              hal_lld_get_clock_point(CLK_PLL3QCLK)

#elif STM32_USBSEL == STM32_USBSEL_HSI48
  #define STM32_USBCLK              STM32_HSI48CLK

#else
  #error "invalid source selected for USB clock"
#endif

/**
 * @brief   SDMMC1 clock frequency.
 */
#if (STM32_SDMMC1SEL == STM32_SDMMC1SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SDMMC1CLK           hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SDMMC1SEL == STM32_SDMMC1SEL_PLL2R
  #define STM32_SDMMC1CLK           hal_lld_get_clock_point(CLK_PLL2RCLK)

#else
  #error "invalid source selected for SDMMC1 clock"
#endif

/**
 * @brief   SDMMC2 clock frequency.
 */
#if (STM32_SDMMC2SEL == STM32_SDMMC2SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SDMMC2CLK           hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SDMMC2SEL == STM32_SDMMC2SEL_PLL2R
  #define STM32_SDMMC2CLK           hal_lld_get_clock_point(CLK_PLL2RCLK)

#else
  #error "invalid source selected for SDMMC2 clock"
#endif

/**
 * @brief   I2C1 clock frequency.
 */
#if (STM32_I2C1SEL == STM32_I2C1SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_I2C1CLK             hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_I2C1SEL == STM32_I2C1SEL_PLL3R
  #define STM32_I2C1CLK             hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_I2C1SEL == STM32_I2C1SEL_HSI
  #define STM32_I2C1CLK             STM32_HSICLK

#elif STM32_I2C1SEL == STM32_I2C1SEL_CSI
  #define STM32_I2C1CLK             STM32_CSICLK

#else
  #error "invalid source selected for I2C1 clock"
#endif

/**
 * @brief   I2C2 clock frequency.
 */
#if (STM32_I2C2SEL == STM32_I2C2SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_I2C2CLK             hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_I2C2SEL == STM32_I2C2SEL_PLL3R
  #define STM32_I2C2CLK             hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_I2C2SEL == STM32_I2C2SEL_HSI
  #define STM32_I2C2CLK             STM32_HSICLK

#elif STM32_I2C2SEL == STM32_I2C2SEL_CSI
  #define STM32_I2C2CLK             STM32_CSICLK

#else
  #error "invalid source selected for I2C2 clock"
#endif

/**
 * @brief   I2C3 clock frequency.
 */
#if (STM32_I2C3SEL == STM32_I2C3SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_I2C3CLK             hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_I2C3SEL == STM32_I2C3SEL_PLL3R
  #define STM32_I2C3CLK             hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_I2C3SEL == STM32_I2C3SEL_HSI
  #define STM32_I2C3CLK             STM32_HSICLK

#elif STM32_I2C3SEL == STM32_I2C3SEL_CSI
  #define STM32_I2C3CLK             STM32_CSICLK

#else
  #error "invalid source selected for I2C3 clock"
#endif

/**
 * @brief   I2C4 clock frequency.
 */
#if (STM32_I2C4SEL == STM32_I2C4SEL_PCLK3) || defined(__DOXYGEN__)
  #define STM32_I2C4CLK             hal_lld_get_clock_point(CLK_PCLK3)

#elif STM32_I2C4SEL == STM32_I2C4SEL_PLL3R
  #define STM32_I2C4CLK             hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_I2C4SEL == STM32_I2C4SEL_HSI
  #define STM32_I2C4CLK             STM32_HSICLK

#elif STM32_I2C4SEL == STM32_I2C4SEL_CSI
  #define STM32_I2C4CLK             STM32_CSICLK

#else
  #error "invalid source selected for I2C4 clock"
#endif

/**
 * @brief   I3C1 clock frequency.
 */
#if (STM32_I3C1SEL == STM32_I3C1SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_I3C1CLK             hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_I3C1SEL == STM32_I3C1SEL_PLL3R
  #define STM32_I3C1CLK             hal_lld_get_clock_point(CLK_PLL3RCLK)

#elif STM32_I3C1SEL == STM32_I3C1SEL_HSI
  #define STM32_I3C1CLK             STM32_HSICLK

#else
  #error "invalid source selected for I3C1 clock"
#endif

/**
 * @brief   ADCDACSEL clock frequency.
 */
#if (STM32_ADCDACSEL == STM32_ADCDACSEL_HCLK) || defined(__DOXYGEN__)
  #define STM32_ADCDACCLK           hal_lld_get_clock_point(CLK_HCLK)

#elif STM32_ADCDACSEL == STM32_ADCDACSEL_SYSCLK
  #define STM32_ADCDACCLK           hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_ADCDACSEL == STM32_ADCDACSEL_PLL2R
  #define STM32_ADCDACCLK           hal_lld_get_clock_point(CLK_PLL2RCLK)

#elif STM32_ADCDACSEL == STM32_ADCDACSEL_HSE
  #define STM32_ADCDACCLK           STM32_HSECLK

#elif STM32_ADCDACSEL == STM32_ADCDACSEL_HSI
  #define STM32_ADCDACCLK           STM32_HSICLK

#elif STM32_ADCDACSEL == STM32_ADCDACSEL_CSI
  #define STM32_ADCDACCLK           STM32_CSICLK

#else
  #error "invalid source selected for ADCDACSEL clock"
#endif

/**
 * @brief   DACSEL clock frequency.
 */
#if (STM32_DACSEL == STM32_DACSEL_LSI) || defined(__DOXYGEN__)
  #define STM32_DACSELCLK           STM32_LSICLK

#elif STM32_DACSEL == STM32_DACSEL_LSE
  #define STM32_DACSELCLK           STM32_LSECLK

#elif STM32_DACSEL == STM32_DACSEL_IGNORE
  #define STM32_DACSELCLK           0

#else
  #error "invalid source selected for DACSEL clock"
#endif

/**
 * @brief   RNG clock frequency.
 */
#if (STM32_RNGSEL == STM32_RNGSEL_HSI48) || defined(__DOXYGEN__)
  #define STM32_RNGCLK              STM32_HSI48CLK

#elif STM32_RNGSEL == STM32_RNGSEL_PLL1Q
  #define STM32_RNGCLK              hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_RNGSEL == STM32_RNGSEL_LSE
  #define STM32_RNGCLK              STM32_LSECLK

#elif STM32_RNGSEL == STM32_RNGSEL_LSI
  #define STM32_RNGCLK              STM32_LSICLK

#elif STM32_RNGSEL == STM32_RNGSEL_IGNORE
  #define STM32_RNGCLK              0

#else
  #error "invalid source selected for RNG clock"
#endif

/**
 * @brief   CEC clock frequency.
 */
#if (STM32_CECSEL == STM32_CECSEL_LSE) || defined(__DOXYGEN__)
  #define STM32_CECCLK              STM32_LSECLK

#elif STM32_CECSEL == STM32_CECSEL_LSI
  #define STM32_CECCLK              STM32_LSICLK

#elif STM32_CECSEL == STM32_CECSEL_CSIDIV128
  #define STM32_CECCLK              (STM32_CSICLK / 128)

#elif STM32_CECSEL == STM32_CECSEL_IGNORE
  #define STM32_CECCLK              0

#else
  #error "invalid source selected for CEC clock"
#endif

/**
 * @brief   FDCAN clock frequency.
 */
#if (STM32_FDCANSEL == STM32_FDCANSEL_HSE) || defined(__DOXYGEN__)
  #define STM32_FDCANCLK            STM32_HSECLK

#elif STM32_FDCANSEL == STM32_FDCANSEL_PLL1Q
  #define STM32_FDCANCLK            hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_FDCANSEL == STM32_FDCANSEL_PLL2Q
  #define STM32_FDCANCLK            hal_lld_get_clock_point(CLK_PLL2QCLK)

#elif STM32_FDCANSEL == STM32_FDCANSEL_IGNORE
  #define STM32_FDCANCLK            0

#else
  #error "invalid source selected for FDCAN clock"
#endif

/**
 * @brief   SAI1 clock frequency.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SAI1CLK             hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SAI1SEL == STM32_SAI2SEL_PLL2P
  #define STM32_SAI1CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SAI1SEL == STM32_SAI1SEL_PLL3P
  #define STM32_SAI1CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SAI1SEL == STM32_SAI1SEL_AUDIOCLK
  #define STM32_SAI1CLK             0 /* TODO board.h */

#elif STM32_SAI1SEL == STM32_SAI1SEL_PER
  #define STM32_SAI1CLK             STM32_PERCLK

#else
  #error "invalid source selected for SAI1 clock"
#endif

/**
 * @brief   SAI2 clock frequency.
 */
#if (STM32_SAI2SEL == STM32_SAI2SEL_PLL1Q) || defined(__DOXYGEN__)
  #define STM32_SAI2CLK             hal_lld_get_clock_point(CLK_PLL1QCLK)

#elif STM32_SAI2SEL == STM32_SAI2SEL_PLL2P
  #define STM32_SAI2CLK             hal_lld_get_clock_point(CLK_PLL2PCLK)

#elif STM32_SAI2SEL == STM32_SAI2SEL_PLL3P
  #define STM32_SAI2CLK             hal_lld_get_clock_point(CLK_PLL3PCLK)

#elif STM32_SAI2SEL == STM32_SAI2SEL_AUDIOCLK
  #define STM32_SAI2CLK             0 /* TODO board.h */

#elif STM32_SAI2SEL == STM32_SAI2SEL_PER
  #define STM32_SAI2CLK             STM32_PERCLK

#else
  #error "invalid source selected for SAI2 clock"
#endif

#if (STM32_TIMPRE == STM32_TIMPRE_LOW) || defined(__DOXYGEN__)
  /**
   * @brief   TIMP1CLK clock frequency.
   */
  #if (STM32_PPRE1 == STM32_PPRE1_DIV1) ||                                  \
      (STM32_PPRE1 == STM32_PPRE1_DIV2) || defined(__DOXYGEN__)
    #define STM32_TIMP1CLK            STM32_HCLK
  #else
    #define STM32_TIMP1CLK            (STM32_PCLK1 * 2)
  #endif

  /**
   * @brief   TIMP2CLK clock frequency.
   */
  #if (STM32_PPRE2 == STM32_PPRE2_DIV1) ||                                  \
      (STM32_PPRE2 == STM32_PPRE2_DIV2) || defined(__DOXYGEN__)
    #define STM32_TIMP2CLK            STM32_HCLK
  #else
    #define STM32_TIMP2CLK            (STM32_PCLK2 * 2)
  #endif

#else
  #if (STM32_PPRE1 == STM32_PPRE1_DIV1) ||                                  \
      (STM32_PPRE1 == STM32_PPRE1_DIV2) ||                                  \
      (STM32_PPRE1 == STM32_PPRE1_DIV4) || defined(__DOXYGEN__)
    #define STM32_TIMP1CLK            (STM32_PCLK1 * 2)
  #else
    #define STM32_TIMP1CLK            (STM32_PCLK1 * 4)
  #endif

  #if (STM32_PPRE2 == STM32_PPRE2_DIV1) ||                                  \
      (STM32_PPRE2 == STM32_PPRE2_DIV2) ||                                  \
      (STM32_PPRE2 == STM32_PPRE2_DIV4) || defined(__DOXYGEN__)
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
  #define STM32_FLASHBITS           (FLASH_ACR_LATENCY_0WS | 0U)

#elif STM32_HCLK <= STM32_1WS_THRESHOLD
  #define STM32_FLASHBITS           (FLASH_ACR_LATENCY_1WS | 0U)

#elif STM32_HCLK <= STM32_2WS_THRESHOLD
  #define STM32_FLASHBITS           (FLASH_ACR_LATENCY_2WS | FLASH_ACR_WRHIGHFREQ_0)

#elif STM32_HCLK <= STM32_3WS_THRESHOLD
  #define STM32_FLASHBITS           (FLASH_ACR_LATENCY_3WS | FLASH_ACR_WRHIGHFREQ_0)

#elif STM32_HCLK <= STM32_4WS_THRESHOLD
  #define STM32_FLASHBITS           (FLASH_ACR_LATENCY_4WS | FLASH_ACR_WRHIGHFREQ_1)

#else
  #define STM32_FLASHBITS           (FLASH_ACR_LATENCY_5WS | FLASH_ACR_WRHIGHFREQ_1)
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a clock point identifier.
 */
typedef unsigned halclkpt_t;

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Type of a clock point frequency in Hz.
 */
typedef uint32_t halfreq_t;

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
  uint32_t          pwr_vmcr;
  uint32_t          rcc_cr;
  uint32_t          rcc_cfgr1;
  uint32_t          rcc_cfgr2;
  uint32_t          flash_acr;
  stm32_pll_regs_t  plls[3];
} halclkcfg_t;
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

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
#endif /* !defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
//#include "mpu_v8m.h"
#include "stm32_isr.h"
#include "stm32_gpdma.h"
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
