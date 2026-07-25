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
 * @file    STM32H7xx/hal_lld_type2.h
 * @brief   STM32H7xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          - STM32_VDD (as hundredths of Volt).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32H723xx, STM32H733xx, STM32H730xx very high-performance MCUs.
 *          - STM32H725xx, STM32H735xx very high-performance MCUs with SMPS.
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_TYPE2_H
#define HAL_LLD_TYPE2_H

#include "stm32_registry.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification macros
 * @{
 */
#if defined(STM32H723xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32H723 Single Core Very High Performance with DSP and FPU"

#elif defined(STM32H733xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32H733 Single Core Very High Performance with DSP and FPU"

#elif defined(STM32H725xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32H725 Single Core Very High Performance with DSP and FPU"

#elif defined(STM32H735xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32H735 Single Core Very High Performance with DSP and FPU"

#elif defined(STM32H730xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32H730 Single Core Very High Performance with DSP and FPU"

#else
#error "STM32H7xx device not specified"
#endif
/** @} */

/**
 * @name    Absolute Maximum Ratings
 * @{
 */
/**
 * @brief   Absolute maximum system clock.
 */
#define STM32_SYSCLK_MAX                550000000

/**
 * @brief   Absolute maximum HCLK clock.
 */
#define STM32_HCLK_MAX                  (STM32_SYSCLK_MAX / 2)

/**
 * @brief   Maximum HSE clock frequency.
 */
#define STM32_HSECLK_MAX                50000000

/**
 * @brief   Maximum HSE clock frequency using an external source.
 */
#define STM32_HSECLK_BYP_MAX            50000000

/**
 * @brief   Minimum HSE clock frequency.
 */
#define STM32_HSECLK_MIN                4000000

/**
 * @brief   Minimum HSE clock frequency.
 */
#define STM32_HSECLK_BYP_MIN            4000000

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSE_CK_MAX                32768

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSE_CK_BYP_MAX            1000000

/**
 * @brief   Minimum LSE clock frequency.
 */
#define STM32_LSE_CK_MIN                32768

/**
 * @brief   Minimum PLLs input clock frequency.
 */
#define STM32_PLLIN_MIN                 1000000

/**
 * @brief   PLLs input threshold frequency 1.
 */
#define STM32_PLLIN_THRESHOLD1          2000000

/**
 * @brief   PLLs input threshold frequency 2.
 */
#define STM32_PLLIN_THRESHOLD2          4000000

/**
 * @brief   PLLs input threshold frequency 3.
 */
#define STM32_PLLIN_THRESHOLD3          8000000

/**
 * @brief   Maximum PLLs input clock frequency.
 */
#define STM32_PLLIN_MAX                 16000000

/**
 * @brief   Absolute minimum PLLs VCO clock frequency.
 */
#define STM32_PLLVCO_MIN                192000000

/**
 * @brief   Absolute maximum PLLs VCOH clock frequency.
 */
#define STM32_PLLVCO_MAX                836000000

/**
 * @brief   Wide range VCO minimum PLL clock frequency.
 */
#define STM32_PLLVCO_WIDE_MIN           192000000

/**
 * @brief   Wide range VCO maximum clock frequency.
 */
#define STM32_PLLVCO_WIDE_MAX           836000000

/**
 * @brief   Medium range VCO minimum PLL clock frequency.
 */
#define STM32_PLLVCO_MEDIUM_MIN         150000000

/**
 * @brief   Medium range VCO maximum PLL clock frequency.
 */
#define STM32_PLLVCO_MEDIUM_MAX         420000000

/**
 * @brief   Maximum APB1 clock frequency.
 */
#define STM32_PCLK1_MAX                 (STM32_HCLK_MAX / 2)

/**
 * @brief   Maximum APB2 clock frequency.
 */
#define STM32_PCLK2_MAX                 (STM32_HCLK_MAX / 2)

/**
 * @brief   Maximum APB3 clock frequency.
 */
#define STM32_PCLK3_MAX                 (STM32_HCLK_MAX / 2)

/**
 * @brief   Maximum APB4 clock frequency.
 */
#define STM32_PCLK4_MAX                 (STM32_HCLK_MAX / 2)

/**
 * @brief   Maximum SPI1, SPI2 and SPI3 clock frequency.
 */
#define STM32_SPI123_MAX                125000000

/**
 * @brief   Maximum SPI4, SPI5 and SPI6 clock frequency.
 */
#define STM32_SPI456_MAX                68500000

/**
 * @brief   Maximum ADC clock frequency.
 */
#define STM32_ADCCLK_MAX                50000000
/** @} */

/**
 * @name    Internal clock sources frequencies
 * @{
 */
#define STM32_HSI_OSC                   64000000
#define STM32_HSI48_OSC                 48000000
#define STM32_CSI_OSC                   4000000
#define STM32_LSI_OSC                   32000
/** @} */

/**
 * @name    Register helpers not found in ST headers
 * @{
 */
#define RCC_CR_HSIDIV_VALUE(n)          ((n) << 3U)

#define RCC_CFGR_SW_VALUE(n)            ((n) << 0U)
#define RCC_CFGR_RTCPRE_VALUE(n)        ((n) << 8U)
#define RCC_CFGR_MCO1PRE_VALUE(n)       ((n) << 18U)
#define RCC_CFGR_MCO1_VALUE(n)          ((n) << 22U)
#define RCC_CFGR_MCO2PRE_VALUE(n)       ((n) << 25U)
#define RCC_CFGR_MCO2_VALUE(n)          ((n) << 29U)

#define RCC_D1CFGR_D1HPRE_VALUE(n)      ((n) << RCC_D1CFGR_HPRE_Pos)
#define RCC_D1CFGR_D1CPRE_VALUE(n)      ((n) << RCC_D1CFGR_D1CPRE_Pos)
#define RCC_D1CFGR_D1PPRE3_VALUE(n)     ((n) << RCC_D1CFGR_D1PPRE_Pos)

#define RCC_D2CFGR_D2PPRE1_VALUE(n)     ((n) << RCC_D2CFGR_D2PPRE1_Pos)
#define RCC_D2CFGR_D2PPRE2_VALUE(n)     ((n) << RCC_D2CFGR_D2PPRE2_Pos)

#define RCC_D3CFGR_D3PPRE4_VALUE(n)     ((n) << RCC_D3CFGR_D3PPRE_Pos)

#define RCC_PLLCKSELR_PLLSRC_VALUE(n)   ((n) << RCC_PLLCKSELR_PLLSRC_Pos)

#define RCC_PLLCKSELR_DIVM1_VALUE(n)    ((n) << RCC_PLLCKSELR_DIVM1_Pos)
#define RCC_PLLCKSELR_DIVM2_VALUE(n)    ((n) << RCC_PLLCKSELR_DIVM2_Pos)
#define RCC_PLLCKSELR_DIVM3_VALUE(n)    ((n) << RCC_PLLCKSELR_DIVM3_Pos)

#define RCC_PLL1DIVR_DIVN1_VALUE(n)     ((n) << RCC_PLL1DIVR_N1)
#define RCC_PLL1DIVR_DIVP1_VALUE(n)     ((n) << RCC_PLL1DIVR_P1)
#define RCC_PLL1DIVR_DIVQ1_VALUE(n)     ((n) << RCC_PLL1DIVR_Q1)
#define RCC_PLL1DIVR_DIVR1_VALUE(n)     ((n) << RCC_PLL1DIVR_R1)

#define RCC_PLL1FRACR_FRACN1_VALUE(n)   ((n) << RCC_PLL1FRACR_FRACN1_Pos)

#define RCC_PLL2DIVR_DIVN2_VALUE(n)     ((n) << RCC_PLL2DIVR_N2)
#define RCC_PLL2DIVR_DIVP2_VALUE(n)     ((n) << RCC_PLL2DIVR_P2)
#define RCC_PLL2DIVR_DIVQ2_VALUE(n)     ((n) << RCC_PLL2DIVR_Q2)
#define RCC_PLL2DIVR_DIVR2_VALUE(n)     ((n) << RCC_PLL2DIVR_R2)

#define RCC_PLL2FRACR_FRACN2_VALUE(n)   ((n) << RCC_PLL2FRACR_FRACN2_Pos)

#define RCC_PLL3DIVR_DIVN3_VALUE(n)     ((n) << RCC_PLL3DIVR_N3)
#define RCC_PLL3DIVR_DIVP3_VALUE(n)     ((n) << RCC_PLL3DIVR_P3)
#define RCC_PLL3DIVR_DIVQ3_VALUE(n)     ((n) << RCC_PLL3DIVR_Q3)
#define RCC_PLL3DIVR_DIVR3_VALUE(n)     ((n) << RCC_PLL3DIVR_R3)

#define RCC_PLL3FRACR_FRACN3_VALUE(n)   ((n) << RCC_PLL3FRACR_FRACN3_Pos)

#define RCC_D1CCIPR_CKPERSEL_VALUE(n)   ((n) << RCC_D1CCIPR_CKPERSEL_Pos)
#define RCC_D1CCIPR_SDMMCSEL_VALUE(n)   ((n) << RCC_D1CCIPR_SDMMCSEL_Pos)
#define RCC_D1CCIPR_OCTOSPISEL_VALUE(n) ((n) << RCC_D1CCIPR_OCTOSPISEL_Pos)
#define RCC_D1CCIPR_FMCSEL_VALUE(n)     ((n) << RCC_D1CCIPR_FMCSEL_Pos)

#define RCC_D2CCIP1R_SWPSEL_VALUE(n)    ((n) << RCC_D2CCIP1R_SWPSEL_Pos)
#define RCC_D2CCIP1R_FDCANSEL_VALUE(n)  ((n) << RCC_D2CCIP1R_FDCANSEL_Pos)
#define RCC_D2CCIP1R_DFSDM1SEL_VALUE(n) ((n) << RCC_D2CCIP1R_DFSDM1SEL_Pos)
#define RCC_D2CCIP1R_SPDIFSEL_VALUE(n)  ((n) << RCC_D2CCIP1R_SPDIFSEL_Pos)
#define RCC_D2CCIP1R_SPI45SEL_VALUE(n)  ((n) << RCC_D2CCIP1R_SPI45SEL_Pos)
#define RCC_D2CCIP1R_SPI123SEL_VALUE(n) ((n) << RCC_D2CCIP1R_SPI123SEL_Pos)
#define RCC_D2CCIP1R_SAI1SEL_VALUE(n)   ((n) << RCC_D2CCIP1R_SAI1SEL_Pos)

#define RCC_D2CCIP2R_LPTIM1SEL_VALUE(n) ((n) << RCC_D2CCIP2R_LPTIM1SEL_Pos)
#define RCC_D2CCIP2R_CECSEL_VALUE(n)    ((n) << RCC_D2CCIP2R_CECSEL_Pos)
#define RCC_D2CCIP2R_USBSEL_VALUE(n)    ((n) << RCC_D2CCIP2R_USBSEL_Pos)
#define RCC_D2CCIP2R_I2C1235SEL_VALUE(n) ((n) << RCC_D2CCIP2R_I2C1235SEL_Pos)
#define RCC_D2CCIP2R_RNGSEL_VALUE(n)    ((n) << RCC_D2CCIP2R_RNGSEL_Pos)
#define RCC_D2CCIP2R_USART16910SEL_VALUE(n) ((n) << RCC_D2CCIP2R_USART16910SEL_Pos)
#define RCC_D2CCIP2R_USART234578SEL_VALUE(n) ((n) << RCC_D2CCIP2R_USART28SEL_Pos)

#define RCC_D3CCIPR_SPI6SEL_VALUE(n)    ((n) << RCC_D3CCIPR_SPI6SEL_Pos)
#define RCC_D3CCIPR_SAI4BSEL_VALUE(n)   ((n) << RCC_D3CCIPR_SAI4BSEL_Pos)
#define RCC_D3CCIPR_SAI4ASEL_VALUE(n)   ((n) << RCC_D3CCIPR_SAI4ASEL_Pos)
#define RCC_D3CCIPR_ADCSEL_VALUE(n)     ((n) << RCC_D3CCIPR_ADCSEL_Pos)
#define RCC_D3CCIPR_LPTIM345SEL_VALUE(n) ((n) << RCC_D3CCIPR_LPTIM345SEL_Pos)
#define RCC_D3CCIPR_LPTIM2SEL_VALUE(n)  ((n) << RCC_D3CCIPR_LPTIM2SEL_Pos)
#define RCC_D3CCIPR_I2C4SEL_VALUE(n)    ((n) << RCC_D3CCIPR_I2C4SEL_Pos)
#define RCC_D3CCIPR_LPUART1SEL_VALUE(n) ((n) << RCC_D3CCIPR_LPUART1SEL_Pos)

#define RCC_BDCR_RTCSEL_VALUE(n)        ((n) << RCC_BDCR_RTCSEL_Pos)
/** @} */

/**
 * @name    Configuration switches to be used in @p mcuconf.h
 * @{
 */
#define STM32_VOS_SCALE3                (PWR_D3CR_VOS_0)
#define STM32_VOS_SCALE2                (PWR_D3CR_VOS_1)
#define STM32_VOS_SCALE1                (PWR_D3CR_VOS_1 | PWR_D3CR_VOS_0)
#define STM32_VOS_SCALE0                0U

#define STM32_SW_HSI_CK                 RCC_CFGR_SW_VALUE(0U)
#define STM32_SW_CSI_CK                 RCC_CFGR_SW_VALUE(1U)
#define STM32_SW_HSE_CK                 RCC_CFGR_SW_VALUE(2U)
#define STM32_SW_PLL1_P_CK              RCC_CFGR_SW_VALUE(3U)

#define STM32_D1CPRE_DIV1               RCC_D1CFGR_D1CPRE_VALUE(0U)
#define STM32_D1CPRE_DIV2               RCC_D1CFGR_D1CPRE_VALUE(8U)
#define STM32_D1CPRE_DIV4               RCC_D1CFGR_D1CPRE_VALUE(9U)
#define STM32_D1CPRE_DIV8               RCC_D1CFGR_D1CPRE_VALUE(10U)
#define STM32_D1CPRE_DIV16              RCC_D1CFGR_D1CPRE_VALUE(11U)
#define STM32_D1CPRE_DIV64              RCC_D1CFGR_D1CPRE_VALUE(12U)
#define STM32_D1CPRE_DIV128             RCC_D1CFGR_D1CPRE_VALUE(13U)
#define STM32_D1CPRE_DIV256             RCC_D1CFGR_D1CPRE_VALUE(14U)
#define STM32_D1CPRE_DIV512             RCC_D1CFGR_D1CPRE_VALUE(15U)

#define STM32_D1HPRE_DIV1               RCC_D1CFGR_D1HPRE_VALUE(0U)
#define STM32_D1HPRE_DIV2               RCC_D1CFGR_D1HPRE_VALUE(8U)
#define STM32_D1HPRE_DIV4               RCC_D1CFGR_D1HPRE_VALUE(9U)
#define STM32_D1HPRE_DIV8               RCC_D1CFGR_D1HPRE_VALUE(10U)
#define STM32_D1HPRE_DIV16              RCC_D1CFGR_D1HPRE_VALUE(11U)
#define STM32_D1HPRE_DIV64              RCC_D1CFGR_D1HPRE_VALUE(12U)
#define STM32_D1HPRE_DIV128             RCC_D1CFGR_D1HPRE_VALUE(13U)
#define STM32_D1HPRE_DIV256             RCC_D1CFGR_D1HPRE_VALUE(14U)
#define STM32_D1HPRE_DIV512             RCC_D1CFGR_D1HPRE_VALUE(15U)

#define STM32_D1PPRE3_DIV1              RCC_D1CFGR_D1PPRE3_VALUE(0U)
#define STM32_D1PPRE3_DIV2              RCC_D1CFGR_D1PPRE3_VALUE(4U)
#define STM32_D1PPRE3_DIV4              RCC_D1CFGR_D1PPRE3_VALUE(5U)
#define STM32_D1PPRE3_DIV8              RCC_D1CFGR_D1PPRE3_VALUE(6U)
#define STM32_D1PPRE3_DIV16             RCC_D1CFGR_D1PPRE3_VALUE(7U)

#define STM32_D2PPRE1_DIV1              RCC_D2CFGR_D2PPRE1_VALUE(0U)
#define STM32_D2PPRE1_DIV2              RCC_D2CFGR_D2PPRE1_VALUE(4U)
#define STM32_D2PPRE1_DIV4              RCC_D2CFGR_D2PPRE1_VALUE(5U)
#define STM32_D2PPRE1_DIV8              RCC_D2CFGR_D2PPRE1_VALUE(6U)
#define STM32_D2PPRE1_DIV16             RCC_D2CFGR_D2PPRE1_VALUE(7U)

#define STM32_D2PPRE2_DIV1              RCC_D2CFGR_D2PPRE2_VALUE(0U)
#define STM32_D2PPRE2_DIV2              RCC_D2CFGR_D2PPRE2_VALUE(4U)
#define STM32_D2PPRE2_DIV4              RCC_D2CFGR_D2PPRE2_VALUE(5U)
#define STM32_D2PPRE2_DIV8              RCC_D2CFGR_D2PPRE2_VALUE(6U)
#define STM32_D2PPRE2_DIV16             RCC_D2CFGR_D2PPRE2_VALUE(7U)

#define STM32_D3PPRE4_DIV1              RCC_D3CFGR_D3PPRE4_VALUE(0U)
#define STM32_D3PPRE4_DIV2              RCC_D3CFGR_D3PPRE4_VALUE(4U)
#define STM32_D3PPRE4_DIV4              RCC_D3CFGR_D3PPRE4_VALUE(5U)
#define STM32_D3PPRE4_DIV8              RCC_D3CFGR_D3PPRE4_VALUE(6U)
#define STM32_D3PPRE4_DIV16             RCC_D3CFGR_D3PPRE4_VALUE(7U)

#define STM32_HSIDIV_DIV1               RCC_CR_HSIDIV_VALUE(0U)
#define STM32_HSIDIV_DIV2               RCC_CR_HSIDIV_VALUE(1U)
#define STM32_HSIDIV_DIV4               RCC_CR_HSIDIV_VALUE(2U)
#define STM32_HSIDIV_DIV8               RCC_CR_HSIDIV_VALUE(3U)

#define STM32_MCO1SEL_HSI_CK            RCC_CFGR_MCO1_VALUE(0U)
#define STM32_MCO1SEL_LSE_CK            RCC_CFGR_MCO1_VALUE(1U)
#define STM32_MCO1SEL_HSE_CK            RCC_CFGR_MCO1_VALUE(2U)
#define STM32_MCO1SEL_PLL1_Q_CK         RCC_CFGR_MCO1_VALUE(3U)
#define STM32_MCO1SEL_HSI48_CK          RCC_CFGR_MCO1_VALUE(4U)

#define STM32_MCO2SEL_SYS_CK            RCC_CFGR_MCO2_VALUE(0U)
#define STM32_MCO2SEL_PLL2_P_CK         RCC_CFGR_MCO2_VALUE(1U)
#define STM32_MCO2SEL_HSE_CK            RCC_CFGR_MCO2_VALUE(2U)
#define STM32_MCO2SEL_PLL1_P_CK         RCC_CFGR_MCO2_VALUE(3U)
#define STM32_MCO2SEL_CSI_CK            RCC_CFGR_MCO2_VALUE(4U)
#define STM32_MCO2SEL_LSI_CK            RCC_CFGR_MCO2_VALUE(5U)

#define STM32_RTCSEL_MASK               RCC_BDCR_RTCSEL_Msk
#define STM32_RTCSEL_NOCLK              RCC_BDCR_RTCSEL_VALUE(0U)
#define STM32_RTCSEL_LSE_CK             RCC_BDCR_RTCSEL_VALUE(1U)
#define STM32_RTCSEL_LSI_CK             RCC_BDCR_RTCSEL_VALUE(2U)
#define STM32_RTCSEL_HSE_1M_CK          RCC_BDCR_RTCSEL_VALUE(3U)

#define STM32_HRTIMSEL_C_CLK            RCC_CFGR_HRTIMSEL

#define STM32_STOPKERWUCK_ENABLED       RCC_CFGR_STOPKERWUCK

#define STM32_STOPWUCK_ENABLED          RCC_CFGR_STOPKERWUCK

#define STM32_PLLSRC_HSI_CK             RCC_PLLCKSELR_PLLSRC_VALUE(0U)
#define STM32_PLLSRC_CSI_CK             RCC_PLLCKSELR_PLLSRC_VALUE(1U)
#define STM32_PLLSRC_HSE_CK             RCC_PLLCKSELR_PLLSRC_VALUE(2U)
#define STM32_PLLSRC_DISABLE            RCC_PLLCKSELR_PLLSRC_VALUE(23U)

#define STM32_CKPERSEL_HSI_CK           RCC_D1CCIPR_CKPERSEL_VALUE(0U)
#define STM32_CKPERSEL_CSI_CK           RCC_D1CCIPR_CKPERSEL_VALUE(1U)
#define STM32_CKPERSEL_HSE_CK           RCC_D1CCIPR_CKPERSEL_VALUE(2U)

#define STM32_SDMMCSEL_PLL1_Q_CK        RCC_D1CCIPR_SDMMCSEL_VALUE(0U)
#define STM32_SDMMCSEL_PLL2_R_CK        RCC_D1CCIPR_SDMMCSEL_VALUE(1U)

#define STM32_OCTOSPISEL_HCLK           RCC_D1CCIPR_OCTOSPISEL_VALUE(0U)
#define STM32_OCTOSPISEL_PLL1_Q_CK      RCC_D1CCIPR_OCTOSPISEL_VALUE(1U)
#define STM32_OCTOSPISEL_PLL2_R_CK      RCC_D1CCIPR_OCTOSPISEL_VALUE(2U)
#define STM32_OCTOSPISEL_PER_CK         RCC_D1CCIPR_OCTOSPISEL_VALUE(3U)

#define STM32_FMCSEL_HCLK               RCC_D1CCIPR_FMCSEL_VALUE(0U)
#define STM32_FMCSEL_PLL1_Q_CK          RCC_D1CCIPR_FMCSEL_VALUE(1U)
#define STM32_FMCSEL_PLL2_R_CK          RCC_D1CCIPR_FMCSEL_VALUE(2U)
#define STM32_FMCSEL_PER_CK             RCC_D1CCIPR_FMCSEL_VALUE(3U)

#define STM32_SWPSEL_PCLK1              RCC_D2CCIP1R_SWPSEL_VALUE(0U)
#define STM32_SWPSEL_HSI_KER_CK         RCC_D2CCIP1R_SWPSEL_VALUE(1U)

#define STM32_FDCANSEL_HSE_CK           RCC_D2CCIP1R_FDCANSEL_VALUE(0U)
#define STM32_FDCANSEL_PLL1_Q_CK        RCC_D2CCIP1R_FDCANSEL_VALUE(1U)
#define STM32_FDCANSEL_PLL2_Q_CK        RCC_D2CCIP1R_FDCANSEL_VALUE(2U)

#define STM32_DFSDM1SEL_PCLK2           RCC_D2CCIP1R_DFSDM1SEL_VALUE(0U)
#define STM32_DFSDM1SEL_SYS_CK          RCC_D2CCIP1R_DFSDM1SEL_VALUE(1U)

#define STM32_SPDIFSEL_PLL1_Q_CK        RCC_D2CCIP1R_SPDIFSEL_VALUE(0U)
#define STM32_SPDIFSEL_PLL2_R_CK        RCC_D2CCIP1R_SPDIFSEL_VALUE(1U)
#define STM32_SPDIFSEL_PLL3_R_CK        RCC_D2CCIP1R_SPDIFSEL_VALUE(2U)
#define STM32_SPDIFSEL_HSI_KET_CLK      RCC_D2CCIP1R_SPDIFSEL_VALUE(3U)

#define STM32_SPI45SEL_PCLK2            RCC_D2CCIP1R_SPI45SEL_VALUE(0U)
#define STM32_SPI45SEL_PLL2_Q_CK        RCC_D2CCIP1R_SPI45SEL_VALUE(1U)
#define STM32_SPI45SEL_PLL3_Q_CK        RCC_D2CCIP1R_SPI45SEL_VALUE(2U)
#define STM32_SPI45SEL_HSI_KER_CK       RCC_D2CCIP1R_SPI45SEL_VALUE(3U)
#define STM32_SPI45SEL_CSI_KER_CK       RCC_D2CCIP1R_SPI45SEL_VALUE(4U)
#define STM32_SPI45SEL_HSE_CK           RCC_D2CCIP1R_SPI45SEL_VALUE(5U)

#define STM32_SPI123SEL_PLL1_Q_CK       RCC_D2CCIP1R_SPI123SEL_VALUE(0U)
#define STM32_SPI123SEL_PLL2_P_CK       RCC_D2CCIP1R_SPI123SEL_VALUE(1U)
#define STM32_SPI123SEL_PLL3_P_CK       RCC_D2CCIP1R_SPI123SEL_VALUE(2U)
#define STM32_SPI123SEL_I2S_CKIN        RCC_D2CCIP1R_SPI123SEL_VALUE(3U)
#define STM32_SPI123SEL_PER_CK          RCC_D2CCIP1R_SPI123SEL_VALUE(4U)

#define STM32_SAI1SEL_PLL1_Q_CK         RCC_D2CCIP1R_SAI1SEL_VALUE(0U)
#define STM32_SAI1SEL_PLL2_P_CK         RCC_D2CCIP1R_SAI1SEL_VALUE(1U)
#define STM32_SAI1SEL_PLL3_P_CK         RCC_D2CCIP1R_SAI1SEL_VALUE(2U)
#define STM32_SAI1SEL_I2S_CKIN          RCC_D2CCIP1R_SAI1SEL_VALUE(3U)
#define STM32_SAI1SEL_PER_CK            RCC_D2CCIP1R_SAI1SEL_VALUE(4U)

#define STM32_LPTIM1SEL_PCLK1           RCC_D2CCIP2R_LPTIM1SEL_VALUE(0U)
#define STM32_LPTIM1SEL_PLL2_P_CK       RCC_D2CCIP2R_LPTIM1SEL_VALUE(1U)
#define STM32_LPTIM1SEL_PLL3_R_CK       RCC_D2CCIP2R_LPTIM1SEL_VALUE(2U)
#define STM32_LPTIM1SEL_LSE_CK          RCC_D2CCIP2R_LPTIM1SEL_VALUE(3U)
#define STM32_LPTIM1SEL_LSI_CK          RCC_D2CCIP2R_LPTIM1SEL_VALUE(4U)
#define STM32_LPTIM1SEL_PER_CK          RCC_D2CCIP2R_LPTIM1SEL_VALUE(5U)

#define STM32_CECSEL_LSE_CK             RCC_D2CCIP2R_CECSEL_VALUE(0U)
#define STM32_CECSEL_LSI_CK             RCC_D2CCIP2R_CECSEL_VALUE(1U)
#define STM32_CECSEL_CSI_KER_CK         RCC_D2CCIP2R_CECSEL_VALUE(2U)
#define STM32_CECSEL_DISABLE            RCC_D2CCIP2R_CECSEL_VALUE(3U)

#define STM32_USBSEL_DISABLE            RCC_D2CCIP2R_USBSEL_VALUE(0U)
#define STM32_USBSEL_PLL1_Q_CK          RCC_D2CCIP2R_USBSEL_VALUE(1U)
#define STM32_USBSEL_PLL3_Q_CK          RCC_D2CCIP2R_USBSEL_VALUE(2U)
#define STM32_USBSEL_HSI48_CK           RCC_D2CCIP2R_USBSEL_VALUE(3U)

#define STM32_I2C1235SEL_PCLK1          RCC_D2CCIP2R_I2C1235SEL_VALUE(0U)
#define STM32_I2C1235SEL_PLL3_R_CK      RCC_D2CCIP2R_I2C1235SEL_VALUE(1U)
#define STM32_I2C1235SEL_HSI_KER_CK     RCC_D2CCIP2R_I2C1235SEL_VALUE(2U)
#define STM32_I2C1235SEL_CSI_KER_CK     RCC_D2CCIP2R_I2C1235SEL_VALUE(3U)

#define STM32_RNGSEL_HSI48_CK           RCC_D2CCIP2R_RNGSEL_VALUE(0U)
#define STM32_RNGSEL_PLL1_Q_CK          RCC_D2CCIP2R_RNGSEL_VALUE(1U)
#define STM32_RNGSEL_LSE_CK             RCC_D2CCIP2R_RNGSEL_VALUE(2U)
#define STM32_RNGSEL_LSI_CK             RCC_D2CCIP2R_RNGSEL_VALUE(3U)

#define STM32_USART16910SEL_PCLK2       RCC_D2CCIP2R_USART16910SEL_VALUE(0U)
#define STM32_USART16910SEL_PLL2_Q_CK   RCC_D2CCIP2R_USART16910SEL_VALUE(1U)
#define STM32_USART16910SEL_PLL3_Q_CK   RCC_D2CCIP2R_USART16910SEL_VALUE(2U)
#define STM32_USART16910SEL_HSI_KER_CK  RCC_D2CCIP2R_USART16910SEL_VALUE(3U)
#define STM32_USART16910SEL_CSI_KER_CK  RCC_D2CCIP2R_USART16910SEL_VALUE(4U)
#define STM32_USART16910SEL_LSE_CK      RCC_D2CCIP2R_USART16910SEL_VALUE(5U)

#define STM32_USART234578SEL_PCLK1      RCC_D2CCIP2R_USART234578SEL_VALUE(0U)
#define STM32_USART234578SEL_PLL2_Q_CK  RCC_D2CCIP2R_USART234578SEL_VALUE(1U)
#define STM32_USART234578SEL_PLL3_Q_CK  RCC_D2CCIP2R_USART234578SEL_VALUE(2U)
#define STM32_USART234578SEL_HSI_KER_CK RCC_D2CCIP2R_USART234578SEL_VALUE(3U)
#define STM32_USART234578SEL_CSI_KER_CK RCC_D2CCIP2R_USART234578SEL_VALUE(4U)
#define STM32_USART234578SEL_LSE_CK     RCC_D2CCIP2R_USART234578SEL_VALUE(5U)

#define STM32_SPI6SEL_PCLK4             RCC_D3CCIPR_SPI6SEL_VALUE(0U)
#define STM32_SPI6SEL_PLL2_Q_CK         RCC_D3CCIPR_SPI6SEL_VALUE(1U)
#define STM32_SPI6SEL_PLL3_Q_CK         RCC_D3CCIPR_SPI6SEL_VALUE(2U)
#define STM32_SPI6SEL_HSI_KER_CK        RCC_D3CCIPR_SPI6SEL_VALUE(3U)
#define STM32_SPI6SEL_CSI_KER_CK        RCC_D3CCIPR_SPI6SEL_VALUE(4U)
#define STM32_SPI6SEL_HSE_CK            RCC_D3CCIPR_SPI6SEL_VALUE(5U)

#define STM32_SAI4BSEL_PLL1_Q_CK        RCC_D3CCIPR_SAI4BSEL_VALUE(0U)
#define STM32_SAI4BSEL_PLL2_P_CK        RCC_D3CCIPR_SAI4BSEL_VALUE(1U)
#define STM32_SAI4BSEL_PLL3_P_CK        RCC_D3CCIPR_SAI4BSEL_VALUE(2U)
#define STM32_SAI4BSEL_I2S_CKIN         RCC_D3CCIPR_SAI4BSEL_VALUE(3U)
#define STM32_SAI4BSEL_PER_CK           RCC_D3CCIPR_SAI4BSEL_VALUE(4U)

#define STM32_SAI4ASEL_PLL1_Q_CK        RCC_D3CCIPR_SAI4ASEL_VALUE(0U)
#define STM32_SAI4ASEL_PLL2_P_CK        RCC_D3CCIPR_SAI4ASEL_VALUE(1U)
#define STM32_SAI4ASEL_PLL3_P_CK        RCC_D3CCIPR_SAI4ASEL_VALUE(2U)
#define STM32_SAI4ASEL_I2S_CKIN         RCC_D3CCIPR_SAI4ASEL_VALUE(3U)
#define STM32_SAI4ASEL_PER_CK           RCC_D3CCIPR_SAI4ASEL_VALUE(4U)

#define STM32_ADCSEL_PLL2_P_CK          RCC_D3CCIPR_ADCSEL_VALUE(0U)
#define STM32_ADCSEL_PLL3_R_CK          RCC_D3CCIPR_ADCSEL_VALUE(1U)
#define STM32_ADCSEL_PER_CK             RCC_D3CCIPR_ADCSEL_VALUE(2U)
#define STM32_ADCSEL_DISABLE            RCC_D3CCIPR_ADCSEL_VALUE(3U)

#define STM32_LPTIM345SEL_PCLK4         RCC_D3CCIPR_LPTIM345SEL_VALUE(0U)
#define STM32_LPTIM345SEL_PLL2_P_CK     RCC_D3CCIPR_LPTIM345SEL_VALUE(1U)
#define STM32_LPTIM345SEL_PLL3_P_CK     RCC_D3CCIPR_LPTIM345SEL_VALUE(2U)
#define STM32_LPTIM345SEL_LSE_CK        RCC_D3CCIPR_LPTIM345SEL_VALUE(3U)
#define STM32_LPTIM345SEL_LSI_CK        RCC_D3CCIPR_LPTIM345SEL_VALUE(4U)
#define STM32_LPTIM345SEL_PER_CK        RCC_D3CCIPR_LPTIM345SEL_VALUE(5U)

#define STM32_LPTIM2SEL_PCLK4           RCC_D3CCIPR_LPTIM2SEL_VALUE(0U)
#define STM32_LPTIM2SEL_PLL2_P_CK       RCC_D3CCIPR_LPTIM2SEL_VALUE(1U)
#define STM32_LPTIM2SEL_PLL3_P_CK       RCC_D3CCIPR_LPTIM2SEL_VALUE(2U)
#define STM32_LPTIM2SEL_LSE_CK          RCC_D3CCIPR_LPTIM2SEL_VALUE(3U)
#define STM32_LPTIM2SEL_LSI_CK          RCC_D3CCIPR_LPTIM2SEL_VALUE(4U)
#define STM32_LPTIM2SEL_PER_CK          RCC_D3CCIPR_LPTIM2SEL_VALUE(5U)

#define STM32_I2C4SEL_PCLK4             RCC_D3CCIPR_I2C4SEL_VALUE(0U)
#define STM32_I2C4SEL_PLL3_R_CK         RCC_D3CCIPR_I2C4SEL_VALUE(1U)
#define STM32_I2C4SEL_HSI_KER_CK        RCC_D3CCIPR_I2C4SEL_VALUE(2U)
#define STM32_I2C4SEL_CSI_KER_CK        RCC_D3CCIPR_I2C4SEL_VALUE(3U)

#define STM32_LPUART1SEL_PCLK4          RCC_D3CCIPR_LPUART1SEL_VALUE(0U)
#define STM32_LPUART1SEL_PLL2_Q_CK      RCC_D3CCIPR_LPUART1SEL_VALUE(1U)
#define STM32_LPUART1SEL_PLL3_Q_CK      RCC_D3CCIPR_LPUART1SEL_VALUE(2U)
#define STM32_LPUART1SEL_HSI_KER_CK     RCC_D3CCIPR_LPUART1SEL_VALUE(3U)
#define STM32_LPUART1SEL_CSI_KER_CK     RCC_D3CCIPR_LPUART1SEL_VALUE(4U)
#define STM32_LPUART1SEL_LSE_CK         RCC_D3CCIPR_LPUART1SEL_VALUE(5U)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options (type 2)
 * @{
 */
/**
 * @brief   PWR CR1 initializer.
 */
#if !defined(STM32_PWR_CR1) || defined(__DOXYGEN__)
#define STM32_PWR_CR1                       (PWR_CR1_SVOS_1 |               \
                                             PWR_CR1_SVOS_0)
#endif

/**
 * @brief   PWR CR2 initializer.
 */
#if !defined(STM32_PWR_CR2) || defined(__DOXYGEN__)
#define STM32_PWR_CR2                       (PWR_CR2_BREN)
#endif

/**
 * @brief   PWR CR3 initializer.
 */
#if !defined(STM32_PWR_CR3) || defined(__DOXYGEN__)
#define STM32_PWR_CR3                       (PWR_CR3_LDOEN |                \
                                             PWR_CR3_USBREGEN |             \
                                             PWR_CR3_USB33DEN)
#endif

/**
 * @brief   PWR CPUCR initializer.
 */
#if !defined(STM32_PWR_CPUCR) || defined(__DOXYGEN__)
#define STM32_PWR_CPUCR                     0
#endif

/**
 * @brief   VOS setting.
 */
#if !defined(STM32_VOS) || defined(__DOXYGEN__)
#define STM32_VOS                           STM32_VOS_SCALE1
#endif

/**
 * @brief   Enables or disables the HSI clock source.
 */
#if !defined(STM32_HSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI_ENABLED                   TRUE
#endif

/**
 * @brief   Enables or disables the LSI clock source.
 */
#if !defined(STM32_LSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSI_ENABLED                   FALSE
#endif

/**
 * @brief   Enables or disables the LSI clock source.
 */
#if !defined(STM32_CSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_CSI_ENABLED                   FALSE
#endif

/**
 * @brief   Enables or disables the HSI48 clock source.
 */
#if !defined(STM32_HSI48_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI48_ENABLED                 TRUE
#endif

/**
 * @brief   Enables or disables the HSE clock source.
 */
#if !defined(STM32_HSE_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSE_ENABLED                   TRUE
#endif

/**
 * @brief   Enables or disables the LSE clock source.
 */
#if !defined(STM32_LSE_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSE_ENABLED                   TRUE
#endif

/**
 * @brief   HSI divider.
 */
#if !defined(STM32_HSIDIV) || defined(__DOXYGEN__)
#define STM32_HSIDIV                        STM32_HSIDIV_DIV1
#endif

/**
 * @brief   Clock source for all PLLs.
 */
#if !defined(STM32_PLLSRC) || defined(__DOXYGEN__)
#define STM32_PLLSRC                        STM32_PLLSRC_HSE_CK
#endif

/**
 * @brief   Masking of PLLCFGR register.
 * @note    By default all options in PLLCFGR are enabled, this option
 *          allows to mask specific bits for power saving reasons.
 *          Use with caution.
 */
#if !defined(STM32_PLLCFGR_MASK) || defined(__DOXYGEN__)
#define STM32_PLLCFGR_MASK                  ~0
#endif

/**
 * @brief   Enables or disables the PLL1.
 */
#if !defined(STM32_PLL1_ENABLED) || defined(__DOXYGEN__)
#define STM32_PLL1_ENABLED                  TRUE
#endif

/**
 * @brief   Enables or disables the PLL1 P output.
 */
#if !defined(STM32_PLL1_P_ENABLED) || defined(__DOXYGEN__)
#define STM32_PLL1_P_ENABLED                TRUE
#endif

/**
 * @brief   Enables or disables the PLL1 Q output.
 */
#if !defined(STM32_PLL1_Q_ENABLED) || defined(__DOXYGEN__)
#define STM32_PLL1_Q_ENABLED                TRUE
#endif

/**
 * @brief   Enables or disables the PLL1 R output.
 */
#if !defined(STM32_PLL1_R_ENABLED) || defined(__DOXYGEN__)
#define STM32_PLL1_R_ENABLED                TRUE
#endif

/**
 * @brief   PLL1 DIVM divider.
 * @note    The allowed values are 1..63.
 */
#if !defined(STM32_PLL1_DIVM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1_DIVM_VALUE               4
#endif

/**
 * @brief   PLL1 DIVN multiplier.
 * @note    The allowed values are 4..512.
 */
#if !defined(STM32_PLL1_DIVN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1_DIVN_VALUE               260
#endif

/**
 * @brief   PLL1 FRACN multiplier, zero if no fractional part.
 * @note    The allowed values are 0..8191.
 */
#if !defined(STM32_PLL1_FRACN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1_FRACN_VALUE              0
#endif

/**
 * @brief   PLL1 DIVP divider.
 * @note    The allowed values are 1..128, odd values not allowed.
 */
#if !defined(STM32_PLL1_DIVP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1_DIVP_VALUE               1
#endif

/**
 * @brief   PLL1 DIVQ divider.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL1_DIVQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1_DIVQ_VALUE               4
#endif

/**
 * @brief   PLL1 DIVR divider.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL1_DIVR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL1_DIVR_VALUE               4
#endif

/**
 * @brief   Enables or disables the PLL2.
 */
#if !defined(STM32_PLL2_ENABLED) || defined(__DOXYGEN__)
#define STM32_PLL2_ENABLED                  TRUE
#endif

/**
 * @brief   Enables or disables the PLL2 P output.
 */
#if !defined(STM32_PLL2_P_ENABLED) || defined(__DOXYGEN__)
#define STM32_PLL1_2_ENABLED                TRUE
#endif

/**
 * @brief   Enables or disables the PLL2 Q output.
 */
#if !defined(STM32_PLL2_Q_ENABLED) || defined(__DOXYGEN__)
#define STM32_PLL2_Q_ENABLED                TRUE
#endif

/**
 * @brief   Enables or disables the PLL2 R output.
 */
#if !defined(STM32_PLL2_R_ENABLED) || defined(__DOXYGEN__)
#define STM32_PLL2_R_ENABLED                TRUE
#endif

/**
 * @brief   PLL2 DIVM divider.
 * @note    The allowed values are 1..63.
 */
#if !defined(STM32_PLL2_DIVM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2_DIVM_VALUE               4
#endif

/**
 * @brief   PLL2 DIVN multiplier.
 * @note    The allowed values are 4..512.
 */
#if !defined(STM32_PLL2_DIVN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2_DIVN_VALUE               400
#endif

/**
 * @brief   PLL2 FRACN multiplier, zero if no fractional part.
 * @note    The allowed values are 0..8191.
 */
#if !defined(STM32_PLL2_FRACN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2_FRACN_VALUE              0
#endif

/**
 * @brief   PLL2 DIVP divider.
 * @note    The allowed values are 2..128, odd values not allowed.
 */
#if !defined(STM32_PLL2_DIVP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2_DIVP_VALUE               40
#endif

/**
 * @brief   PLL2 DIVQ divider.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL2_DIVQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2_DIVQ_VALUE               8
#endif

/**
 * @brief   PLL2 DIVR divider.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL2_DIVR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2_DIVR_VALUE               8
#endif

/**
 * @brief   Enables or disables the PLL3.
 */
#if !defined(STM32_PLL3_ENABLED) || defined(__DOXYGEN__)
#define STM32_PLL3_ENABLED                  TRUE
#endif

/**
 * @brief   Enables or disables the PLL3 P output.
 */
#if !defined(STM32_PLL3_P_ENABLED) || defined(__DOXYGEN__)
#define STM32_PLL3_P_ENABLED                TRUE
#endif

/**
 * @brief   Enables or disables the PLL3 Q output.
 */
#if !defined(STM32_PLL3_Q_ENABLED) || defined(__DOXYGEN__)
#define STM32_PLL3_Q_ENABLED                TRUE
#endif

/**
 * @brief   Enables or disables the PLL3 R output.
 */
#if !defined(STM32_PLL3_R_ENABLED) || defined(__DOXYGEN__)
#define STM32_PLL3_R_ENABLED                TRUE
#endif

/**
 * @brief   PLL3 DIVM divider.
 * @note    The allowed values are 1..63.
 */
#if !defined(STM32_PLL3_DIVM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3_DIVM_VALUE               4
#endif

/**
 * @brief   PLL3 DIVN multiplier.
 * @note    The allowed values are 4..512.
 */
#if !defined(STM32_PLL3_DIVN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3_DIVN_VALUE               400
#endif

/**
 * @brief   PLL3 FRACN multiplier, zero if no fractional part.
 * @note    The allowed values are 0..8191.
 */
#if !defined(STM32_PLL3_FRACN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3_FRACN_VALUE              0
#endif

/**
 * @brief   PLL3 DIVP divider.
 * @note    The allowed values are 2..128, odd values not allowed.
 */
#if !defined(STM32_PLL3_DIVP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3_DIVP_VALUE               8
#endif

/**
 * @brief   PLL3 DIVQ divider.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL3_DIVQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3_DIVQ_VALUE               8
#endif

/**
 * @brief   PLL3 DIVR divider.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL3_DIVR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3_DIVR_VALUE               8
#endif

/**
 * @brief   Peripherals clock selector.
 */
#if !defined(STM32_CKPERSEL) || defined(__DOXYGEN__)
#define STM32_CKPERSEL                      STM32_CKPERSEL_HSE_CK
#endif

/**
 * @brief   MCO1 clock selector.
 */
#if !defined(STM32_MCO1SEL) || defined(__DOXYGEN__)
#define STM32_MCO1SEL                       STM32_MCO1SEL_HSI_CK
#endif

/**
 * @brief   MCO1 clock prescaler.
 */
#if !defined(STM32_MCO1PRE_VALUE) || defined(__DOXYGEN__)
#define STM32_MCO1PRE_VALUE                 4
#endif

/**
 * @brief   MCO2 clock selector.
 */
#if !defined(STM32_MCO2SEL) || defined(__DOXYGEN__)
#define STM32_MCO2SEL                       STM32_MCO2SEL_SYS_CK
#endif

/**
 * @brief   MCO2 clock prescaler.
 */
#if !defined(STM32_MCO2PRE_VALUE) || defined(__DOXYGEN__)
#define STM32_MCO2PRE_VALUE                 4
#endif

/**
 * @brief   TIM clock prescaler selection.
 */
#if !defined(STM32_TIMPRE_ENABLE) || defined(__DOXYGEN__)
#define STM32_TIMPRE_ENABLE                 FALSE
#endif

/**
 * @brief   HRTIM clock prescaler selection.
 */
#if !defined(STM32_HRTIMSEL) || defined(__DOXYGEN__)
#define STM32_HRTIMSEL                      0
#endif

/**
 * @brief   Kernel clock selection after a wake up from system Stop.
 */
#if !defined(STM32_STOPKERWUCK) || defined(__DOXYGEN__)
#define STM32_STOPKERWUCK                   0
#endif

/**
 * @brief   System clock selection after a wake up from system Stop.
 */
#if !defined(STM32_STOPWUCK) || defined(__DOXYGEN__)
#define STM32_STOPWUCK                      0
#endif

/**
 * @brief   RTC HSE prescaler value.
 * @note    The allowed values are 2..63.
 */
#if !defined(STM32_RTCPRE_VALUE) || defined(__DOXYGEN__)
#define STM32_RTCPRE_VALUE                  8
#endif

/**
 * @brief   Main clock source selection.
 * @note    This setting can be modified at runtime.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                            STM32_SW_PLL1_P_CK1_P_CK
#endif

/**
 * @brief   RTC clock selector.
 * @note    This setting can be modified at runtime.
 */
#if !defined(STM32_RTCSEL) || defined(__DOXYGEN__)
#define STM32_RTCSEL                        STM32_RTCSEL_LSE_CK
#endif

/**
 * @brief   Clock domain 1 core bus prescaler.
 * @note    This setting can be modified at runtime.
 */
#if !defined(STM32_D1CPRE) || defined(__DOXYGEN__)
#define STM32_D1CPRE                        STM32_D1CPRE_DIV1
#endif

/**
 * @brief   Clock domain 1 HPRE prescaler.
 * @note    This setting can be modified at runtime.
 */
#if !defined(STM32_D1HPRE) || defined(__DOXYGEN__)
#define STM32_D1HPRE                        STM32_D1HPRE_DIV4
#endif

/**
 * @brief   Clock domain 1 peripherals bus prescaler.
 * @note    This setting can be modified at runtime.
 */
#if !defined(STM32_D1PPRE3) || defined(__DOXYGEN__)
#define STM32_D1PPRE3                       STM32_D1PPRE3_DIV1
#endif

/**
 * @brief   Clock domain 2 peripherals bus 1 prescaler.
 * @note    This setting can be modified at runtime.
 */
#if !defined(STM32_D2PPRE1) || defined(__DOXYGEN__)
#define STM32_D2PPRE1                       STM32_D2PPRE1_DIV1
#endif

/**
 * @brief   Clock domain 2 peripherals bus 2 prescaler.
 * @note    This setting can be modified at runtime.
 */
#if !defined(STM32_D2PPRE2) || defined(__DOXYGEN__)
#define STM32_D2PPRE2                       STM32_D2PPRE2_DIV1
#endif

/**
 * @brief   Clock domain 3 peripherals bus prescaler.
 * @note    This setting can be modified at runtime.
 */
#if !defined(STM32_D3PPRE4) || defined(__DOXYGEN__)
#define STM32_D3PPRE4                       STM32_D3PPRE4_DIV1
#endif

/**
 * @brief   SDMMC clock source.
 */
#if !defined(STM32_SDMMCSEL) || defined(__DOXYGEN__)
#define STM32_SDMMCSEL                      STM32_SDMMCSEL_PLL1_Q_CK
#endif

/**
 * @brief   OCTOSPI clock source.
 */
#if !defined(STM32_OCTOSPISEL) || defined(__DOXYGEN__)
#define STM32_OCTOSPISEL                    STM32_OCTOSPISEL_HCLK
#endif

/**
 * @brief   FMC clock source.
 */
#if !defined(STM32_FMCSEL) || defined(__DOXYGEN__)
#define STM32_FMCSEL                        STM32_FMCSEL_HCLK
#endif

/**
 * @brief   SWP clock source.
 */
#if !defined(STM32_SWPSEL) || defined(__DOXYGEN__)
#define STM32_SWPSEL                        STM32_SWPSEL_PCLK1
#endif

/**
 * @brief   FDCAN clock source.
 */
#if !defined(STM32_FDCANSEL) || defined(__DOXYGEN__)
#define STM32_FDCANSEL                      STM32_FDCANSEL_HSE_CK
#endif

/**
 * @brief   DFSDM1 clock source.
 */
#if !defined(STM32_DFSDM1SEL) || defined(__DOXYGEN__)
#define STM32_DFSDM1SEL                     STM32_DFSDM1SEL_PCLK2
#endif

/**
 * @brief   SPDIF clock source.
 */
#if !defined(STM32_SPDIFSEL) || defined(__DOXYGEN__)
#define STM32_SPDIFSEL                      STM32_SPDIFSEL_PLL1_Q_CK
#endif

/**
 * @brief   SPI45 clock source.
 */
#if !defined(STM32_SPI45SEL) || defined(__DOXYGEN__)
#define STM32_SPI45SEL                      STM32_SPI45SEL_PCLK2
#endif

/**
 * @brief   SPI123 clock source.
 */
#if !defined(STM32_SPI123SEL) || defined(__DOXYGEN__)
#define STM32_SPI123SEL                     STM32_SPI123SEL_PLL1_Q_CK
#endif

/**
 * @brief   SAI23 clock source.
 */
#if !defined(STM32_SAI23SEL) || defined(__DOXYGEN__)
#define STM32_SAI23SEL                      0U /* Not present.*/
#endif

/**
 * @brief   SAI1 clock source.
 */
#if !defined(STM32_SAI1SEL) || defined(__DOXYGEN__)
#define STM32_SAI1SEL                       STM32_SAI1SEL_PLL1_Q_CK
#endif

/**
 * @brief   LPTIM1 clock source.
 */
#if !defined(STM32_LPTIM1SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM1SEL                     STM32_LPTIM1SEL_PCLK1
#endif

/**
 * @brief   CEC clock source.
 */
#if !defined(STM32_CECSEL) || defined(__DOXYGEN__)
#define STM32_CECSEL                        STM32_CECSEL_LSE_CK
#endif

/**
 * @brief   USB clock source.
 */
#if !defined(STM32_USBSEL) || defined(__DOXYGEN__)
#define STM32_USBSEL                        STM32_USBSEL_PLL3_Q_CK
#endif

/**
 * @brief   I2C1235 clock source.
 */
#if !defined(STM32_I2C1235SEL) || defined(__DOXYGEN__)
#define STM32_I2C1235SEL                    STM32_I2C1235SEL_PCLK1
#endif

/**
 * @brief   RNG clock source.
 */
#if !defined(STM32_RNGSEL) || defined(__DOXYGEN__)
#define STM32_RNGSEL                        STM32_RNGSEL_HSI48_CK
#endif

/**
 * @brief   USART16 clock source.
 */
#if !defined(STM32_USART16910SEL) || defined(__DOXYGEN__)
#define STM32_USART16910SEL                 STM32_USART16910SEL_PCLK2
#endif

/**
 * @brief   USART234578 clock source.
 */
#if !defined(STM32_USART234578SEL) || defined(__DOXYGEN__)
#define STM32_USART234578SEL                STM32_USART234578SEL_PCLK1
#endif

/**
 * @brief   SPI6SEL clock source.
 */
#if !defined(STM32_SPI6SEL) || defined(__DOXYGEN__)
#define STM32_SPI6SEL                       STM32_SPI6SEL_PCLK4
#endif

/**
 * @brief   SAI4BSEL clock source.
 */
#if !defined(STM32_SAI4BSEL) || defined(__DOXYGEN__)
#define STM32_SAI4BSEL                      STM32_SAI4BSEL_PLL1_Q_CK
#endif

/**
 * @brief   SAI4ASEL clock source.
 */
#if !defined(STM32_SAI4ASEL) || defined(__DOXYGEN__)
#define STM32_SAI4ASEL                      STM32_SAI4ASEL_PLL1_Q_CK
#endif

/**
 * @brief   ADCSEL clock source.
 */
#if !defined(STM32_ADCSEL) || defined(__DOXYGEN__)
#define STM32_ADCSEL                        STM32_ADCSEL_PLL2_P_CK
#endif

/**
 * @brief   LPTIM345SEL clock source.
 */
#if !defined(STM32_LPTIM345SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM345SEL                   STM32_LPTIM345SEL_PCLK4
#endif

/**
 * @brief   LPTIM2SEL clock source.
 */
#if !defined(STM32_LPTIM2SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM2SEL                     STM32_LPTIM2SEL_PCLK4
#endif

/**
 * @brief   I2C4SEL clock source.
 */
#if !defined(STM32_I2C4SEL) || defined(__DOXYGEN__)
#define STM32_I2C4SEL                       STM32_I2C4SEL_PCLK4
#endif

/**
 * @brief   LPUART1SEL clock source.
 */
#if !defined(STM32_LPUART1SEL) || defined(__DOXYGEN__)
#define STM32_LPUART1SEL                    STM32_LPUART1SEL_PCLK4
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(STM32H7xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32H7xx_MCUCONF not defined"
#endif

#if defined(STM32H730xx) && !defined(STM32H730_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32H730_MCUCONF not defined"
#endif

#if defined(STM32H723xx) && !defined(STM32H723_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32H723_MCUCONF not defined"
#endif

#if defined(STM32H733xx) && !defined(STM32H733_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32H733_MCUCONF not defined"
#endif

#if defined(STM32H725xx) && !defined(STM32H725_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32H725_MCUCONF not defined"
#endif

#if defined(STM32H735xx) && !defined(STM32H735_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32H735_MCUCONF not defined"
#endif

/*
 * Board file checks.
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

/**
 * @name    Constants depending on VOS setting
 * @{
 */
#if STM32_VOS == STM32_VOS_SCALE0
#define STM32_SDMMC_MAXCLK          250000000U
#define STM32_0WS_THRESHOLD         70000000U
#define STM32_1WS_THRESHOLD         140000000U
#define STM32_2WS_THRESHOLD         210000000U
#define STM32_3WS_THRESHOLD         275000000U
#define STM32_4WS_THRESHOLD         0U
#define STM32_PLLOUT_MAX            550000000U
#define STM32_PLLOUT_MIN            1500000U

#elif STM32_VOS == STM32_VOS_SCALE1
#define STM32_SDMMC_MAXCLK          200000000U
#define STM32_0WS_THRESHOLD         67000000U
#define STM32_1WS_THRESHOLD         133000000U
#define STM32_2WS_THRESHOLD         200000000U
#define STM32_3WS_THRESHOLD         0U
#define STM32_4WS_THRESHOLD         0U
#define STM32_PLLOUT_MAX            400000000U
#define STM32_PLLOUT_MIN            1500000U

#elif STM32_VOS == STM32_VOS_SCALE2
#define STM32_SDMMC_MAXCLK          150000000U
#define STM32_0WS_THRESHOLD         50000000U
#define STM32_1WS_THRESHOLD         100000000U
#define STM32_2WS_THRESHOLD         150000000U
#define STM32_3WS_THRESHOLD         0U
#define STM32_4WS_THRESHOLD         0U
#define STM32_PLLOUT_MAX            300000000U
#define STM32_PLLOUT_MIN            1500000U

#elif STM32_VOS == STM32_VOS_SCALE3
#define STM32_SDMMC_MAXCLK          100000000U
#define STM32_0WS_THRESHOLD         35000000U
#define STM32_1WS_THRESHOLD         70000000U
#define STM32_2WS_THRESHOLD         85000000U
#define STM32_3WS_THRESHOLD         0U
#define STM32_4WS_THRESHOLD         0U
#define STM32_PLLOUT_MAX            170000000U
#define STM32_PLLOUT_MIN            1500000U

#else
#error "invalid STM32_VOS setting specified"
#endif
/** @} */

/*
 * HSI related checks.
 */
#if STM32_HSI_ENABLED
#define STM32_HSICLK            STM32_HSI_OSC

#else /* !STM32_HSI_ENABLED */
#define STM32_HSICLK            0U

#if STM32_SW == STM32_SW_HSI_CK
#error "HSI not enabled, required by STM32_SW"
#endif

#if (STM32_PLLSRC == STM32_PLLSRC_HSI_CK) &&                                \
    (STM32_PLL1_ENABLED || STM32_PLL2_ENABLED || STM32_PLL3_ENABLED)
#error "HSI not enabled, required by STM32_PLLSRC and STM32_PLLx_ENABLED"
#endif

#if STM32_CKPERSEL == STM32_CKPERSEL_HSI_CK
#error "HSI not enabled, required by STM32_CKPERSEL"
#endif

#if STM32_MCO1SEL == STM32_MCO1SEL_HSI_CK
#error "HSI not enabled, required by STM32_MCO1SEL"
#endif

#endif /* !STM32_HSI_ENABLED */

/*
 * HSI48 related checks.
 */
#if STM32_HSI48_ENABLED
#define STM32_HSI48_CK          STM32_HSI48_OSC

#else /* !STM32_HSI48_ENABLED */
#define STM32_HSI48_CK          0U

#if STM32_MCO1SEL == STM32_MCO1SEL_HSI48_CK
#error "HSI48 not enabled, required by STM32_MCO1SEL"
#endif

#endif /* !STM32_HSI48_ENABLED */

/*
 * CSI related checks.
 */
#if STM32_CSI_ENABLED
#define STM32_CSI_CK            STM32_CSI_OSC

#else /* !STM32_CSI_ENABLED */
#define STM32_CSI_CK            0U

#if STM32_SW == STM32_SW_CSI_CK
#error "CSI not enabled, required by STM32_SW"
#endif

#if (STM32_PLLSRC == STM32_PLLSRC_CSI_CK) &&                                \
    (STM32_PLL1_ENABLED || STM32_PLL2_ENABLED || STM32_PLL3_ENABLED)
#error "CSI not enabled, required by STM32_PLLSRC and STM32_PLLx_ENABLED"
#endif

#if STM32_CKPERSEL == STM32_CKPERSEL_CSI_CK
#error "CSI not enabled, required by STM32_CKPERSEL"
#endif

#if STM32_MCO2SEL == STM32_MCO2SEL_CSI_CK
#error "CSI not enabled, required by STM32_MCO2SEL"
#endif

#endif /* !STM32_CSI_ENABLED */

/*
 * HSE related checks.
 */
#if STM32_HSE_ENABLED

#if !defined(STM32_HSECLK)
#error "HSE frequency not defined"
#endif

#define STM32_HSE_CK            STM32_HSECLK

#if STM32_HSECLK == 0
#error "HSE oscllator not available"
#else /* STM32_HSECLK != 0 */
#if defined(STM32_HSE_BYPASS)
#if (STM32_HSECLK < STM32_HSECLK_BYP_MIN) || (STM32_HSECLK > STM32_HSECLK_BYP_MAX)
#error "STM32_HSECLK outside acceptable range (STM32_HSECLK_BYP_MIN..STM32_HSECLK_BYP_MAX)"
#endif
#else /* !defined(STM32_HSE_BYPASS) */
#if (STM32_HSECLK < STM32_HSECLK_MIN) || (STM32_HSECLK > STM32_HSECLK_MAX)
#error "STM32_HSECLK outside acceptable range (STM32_HSECLK_MIN..STM32_HSECLK_MAX)"
#endif
#endif /* !defined(STM32_HSE_BYPASS) */
#endif /* STM32_HSECLK != 0 */
#else /* !STM32_HSE_ENABLED */

#if STM32_SW == STM32_SW_HSE_CK
#error "HSE not enabled, required by STM32_SW"
#endif

#if (STM32_PLLSRC == STM32_PLLSRC_HSE_CK) &&                                \
    (STM32_PLL1_ENABLED || STM32_PLL2_ENABLED || STM32_PLL3_ENABLED)
#error "HSE not enabled, required by STM32_PLLSRC and STM32_PLLx_ENABLED"
#endif

#if STM32_MCO1SEL == STM32_MCO1SEL_HSE_CK
#error "HSE not enabled, required by STM32_MCO1SEL"
#endif

#if STM32_MCO2SEL == STM32_MCO2SEL_HSE_CK
#error "HSE not enabled, required by STM32_MCO2SEL"
#endif

#if STM32_RTCSEL == STM32_RTCSEL_HSE_1M_CK
#error "HSE not enabled, required by STM32_RTCSEL"
#endif

#endif /* !STM32_HSE_ENABLED */

/*
 * LSI related checks.
 */
#if STM32_LSI_ENABLED
#define STM32_LSI_CK            STM32_LSI_OSC

#else /* !STM32_LSI_ENABLED */
#define STM32_LSI_CK            0U

#if HAL_USE_RTC && (STM32_RTCSEL == STM32_RTCSEL_LSI_CK)
#error "LSI not enabled, required by STM32_RTCSEL"
#endif

#if STM32_MCO2SEL == STM32_MCO2SEL_LSI_CK
#error "HSE not enabled, required by STM32_MCO2SEL"
#endif

#endif /* !STM32_LSI_ENABLED */

/*
 * LSE related checks.
 */
#if STM32_LSE_ENABLED

#if !defined(STM32_LSECLK)
#error "LSE frequency not defined"
#endif

#define STM32_LSE_CK            STM32_LSECLK

#if (STM32_LSE_CK == 0)
#error "LSE oscillator not available"
#endif

#if defined(STM32_LSE_BYPASS)
#if (STM32_LSE_CK < STM32_LSE_CK_MIN) || (STM32_LSE_CK > STM32_LSE_CK_BYP_MAX)
#error "STM32_LSE_CK outside acceptable range (STM32_LSE_CK_MIN..STM32_LSE_CK_BYP_MAX)"
#endif
#else
#if (STM32_LSE_CK < STM32_LSE_CK_MIN) || (STM32_LSE_CK > STM32_LSE_CK_MAX)
#error "STM32_LSE_CK outside acceptable range (STM32_LSE_CK_MIN..STM32_LSE_CK_MAX)"
#endif
#endif

#if !defined(STM32_LSEDRV)
#error "STM32_LSEDRV not defined"
#endif

#if (STM32_LSEDRV >> 3) > 3
#error "STM32_LSEDRV outside acceptable range ((0<<3)..(3<<3))"
#endif

#else /* !STM32_LSE_ENABLED */

#if STM32_RTCSEL == STM32_RTCSEL_LSE_CK
#error "LSE not enabled, required by STM32_RTCSEL"
#endif

#if STM32_MCO1SEL == STM32_MCO1SEL_LSE_CK
#error "LSE not enabled, required by STM32_MCO1SEL"
#endif

#endif /* !STM32_LSE_ENABLED */

/**
 * @brief   HSI divided clock.
 */
#if (STM32_HSIDIV == STM32_HSIDIV_DIV1) || defined(__DOXYGEN__)
#define STM32_HSI_CK                (STM32_HSICLK / 1U)
#elif STM32_HSIDIV == STM32_HSIDIV_DIV2
#define STM32_HSI_CK                (STM32_HSICLK / 2U)
#elif STM32_HSIDIV == STM32_HSIDIV_DIV4
#define STM32_HSI_CK                (STM32_HSICLK / 4U)
#elif STM32_HSIDIV == STM32_HSIDIV_DIV8
#define STM32_HSI_CK                (STM32_HSICLK / 8U)
#else
#error "invalid STM32_HSIDIV value specified"
#endif

/**
 * @brief   HSE divided clock for RTC.
 */
#if ((STM32_RTCPRE_VALUE >= 2) && (STM32_RTCPRE_VALUE <= 63)) ||            \
    defined(__DOXYGEN__)
#define STM32_HSE_1M_CK             (STM32_HSE_CK / STM32_RTCPRE_VALUE)
#else
#error "invalid STM32_RTCPRE_VALUE value specified"
#endif

/**
 * @brief   PLLs input clock frequency.
 */
#if (STM32_PLLSRC == STM32_PLLSRC_HSE_CK) || defined(__DOXYGEN__)
#define STM32_PLLCLKIN              STM32_HSE_CK

#elif STM32_PLLSRC == STM32_PLLSRC_HSI_CK
#define STM32_PLLCLKIN              STM32_HSI_CK

#elif STM32_PLLSRC == STM32_PLLSRC_CSI_CK
#define STM32_PLLCLKIN              STM32_CSI_CK

#else
#error "invalid STM32_PLLSRC value specified"
#endif

/**
 * @brief   PLL1 DIVM field.
 */
#if ((STM32_PLL1_DIVM_VALUE >= 1) && (STM32_PLL1_DIVM_VALUE <= 63)) ||      \
    defined(__DOXYGEN__)
#define STM32_PLL1_DIVM             (STM32_PLL1_DIVM_VALUE << 4)
#define STM32_PLL1_REF_CK           (STM32_PLLCLKIN / STM32_PLL1_DIVM_VALUE)
#else
#error "invalid STM32_PLL1_DIVM_VALUE value specified"
#endif

/*
 * PLL1 input frequency range check.
 */
#if (STM32_PLL1_REF_CK < STM32_PLLIN_MIN) || (STM32_PLL1_REF_CK > STM32_PLLIN_MAX)
#error "STM32_PLL1_REF_CK outside acceptable range (STM32_PLLIN_MIN..STM32_PLLIN_MAX)"
#endif

/**
 * @brief   PLL1 input range selector.
 */
#if (STM32_PLL1_REF_CK < STM32_PLLIN_THRESHOLD1) || defined(__DOXYGEN__)
#define STM32_PLLCFGR_PLL1RGE       RCC_PLLCFGR_PLL1RGE_0
#elif STM32_PLL1_REF_CK < STM32_PLLIN_THRESHOLD2
#define STM32_PLLCFGR_PLL1RGE       RCC_PLLCFGR_PLL1RGE_1
#elif STM32_PLL1_REF_CK < STM32_PLLIN_THRESHOLD3
#define STM32_PLLCFGR_PLL1RGE       RCC_PLLCFGR_PLL1RGE_2
#else
#define STM32_PLLCFGR_PLL1RGE       RCC_PLLCFGR_PLL1RGE_3
#endif

/**
 * @brief   PLL2 DIVM field.
 */
#if ((STM32_PLL2_DIVM_VALUE >= 1) && (STM32_PLL2_DIVM_VALUE <= 63)) ||      \
    defined(__DOXYGEN__)
#define STM32_PLL2_DIVM             (STM32_PLL2_DIVM_VALUE << 12)
#define STM32_PLL2_REF_CK           (STM32_PLLCLKIN / STM32_PLL2_DIVM_VALUE)
#else
#error "invalid STM32_PLL2_DIVM_VALUE value specified"
#endif

/*
 * PLL2 input frequency range check.
 */
#if (STM32_PLL2_REF_CK < STM32_PLLIN_MIN) || (STM32_PLL2_REF_CK > STM32_PLLIN_MAX)
#error "STM32_PLL2_REF_CK outside acceptable range (STM32_PLLIN_MIN..STM32_PLLIN_MAX)"
#endif

/**
 * @brief   PLL2 input range selector.
 */
#if (STM32_PLL2_REF_CK < STM32_PLLIN_THRESHOLD1) || defined(__DOXYGEN__)
#define STM32_PLLCFGR_PLL2RGE       RCC_PLLCFGR_PLL2RGE_0
#elif STM32_PLL2_REF_CK < STM32_PLLIN_THRESHOLD2
#define STM32_PLLCFGR_PLL2RGE       RCC_PLLCFGR_PLL2RGE_1
#elif STM32_PLL2_REF_CK < STM32_PLLIN_THRESHOLD3
#define STM32_PLLCFGR_PLL2RGE       RCC_PLLCFGR_PLL2RGE_2
#else
#define STM32_PLLCFGR_PLL2RGE       RCC_PLLCFGR_PLL2RGE_3
#endif

/**
 * @brief   PLL3 DIVM field.
 */
#if ((STM32_PLL3_DIVM_VALUE >= 1) && (STM32_PLL3_DIVM_VALUE <= 63)) ||      \
    defined(__DOXYGEN__)
#define STM32_PLL3_DIVM             (STM32_PLL3_DIVM_VALUE << 20)
#define STM32_PLL3_REF_CK           (STM32_PLLCLKIN / STM32_PLL3_DIVM_VALUE)
#else
#error "invalid STM32_PLL3_DIVM_VALUE value specified"
#endif

/*
 * PLL3 input frequency range check.
 */
#if (STM32_PLL3_REF_CK < STM32_PLLIN_MIN) || (STM32_PLL3_REF_CK > STM32_PLLIN_MAX)
#error "STM32_PLL3_REF_CK outside acceptable range (STM32_PLLIN_MIN..STM32_PLLIN_MAX)"
#endif

/**
 * @brief   PLL3 input range selector.
 */
#if (STM32_PLL3_REF_CK < STM32_PLLIN_THRESHOLD1) || defined(__DOXYGEN__)
#define STM32_PLLCFGR_PLL3RGE       RCC_PLLCFGR_PLL3RGE_0
#elif STM32_PLL3_REF_CK < STM32_PLLIN_THRESHOLD2
#define STM32_PLLCFGR_PLL3RGE       RCC_PLLCFGR_PLL3RGE_1
#elif STM32_PLL3_REF_CK < STM32_PLLIN_THRESHOLD3
#define STM32_PLLCFGR_PLL3RGE       RCC_PLLCFGR_PLL3RGE_2
#else
#define STM32_PLLCFGR_PLL3RGE       RCC_PLLCFGR_PLL3RGE_3
#endif

/**
 * @brief   PLL1 DIVN field.
 */
#if ((STM32_PLL1_DIVN_VALUE >= 4) && (STM32_PLL1_DIVN_VALUE <= 512)) ||    \
    defined(__DOXYGEN__)
#define STM32_PLL1_DIVN             ((STM32_PLL1_DIVN_VALUE - 1U) << 0U)
#else
#error "invalid STM32_PLL1_DIVN_VALUE value specified"
#endif

/**
 * @brief   PLL2 DIVN field.
 */
#if ((STM32_PLL2_DIVN_VALUE >= 4) && (STM32_PLL2_DIVN_VALUE <= 512)) ||    \
    defined(__DOXYGEN__)
#define STM32_PLL2_DIVN             ((STM32_PLL2_DIVN_VALUE - 1U) << 0U)
#else
#error "invalid STM32_PLL2_DIVN_VALUE value specified"
#endif

/**
 * @brief   PLL3 DIVN field.
 */
#if ((STM32_PLL3_DIVN_VALUE >= 4) && (STM32_PLL3_DIVN_VALUE <= 512)) ||    \
    defined(__DOXYGEN__)
#define STM32_PLL3_DIVN             ((STM32_PLL3_DIVN_VALUE - 1U) << 0U)
#else
#error "invalid STM32_PLL3_DIVN_VALUE value specified"
#endif

/**
 * @brief   PLL1 FRACN field.
 */
#if ((STM32_PLL1_FRACN_VALUE >= 0) && (STM32_PLL1_FRACN_VALUE <= 8191)) ||  \
    defined(__DOXYGEN__)
#define STM32_PLL1_FRACN             (STM32_PLL1_FRACN_VALUE << 3U)
#else
#error "invalid STM32_PLL1_FRACN_VALUE value specified"
#endif

/**
 * @brief   PLL2 FRACN field.
 */
#if ((STM32_PLL2_FRACN_VALUE >= 0) && (STM32_PLL2_FRACN_VALUE <= 8191)) ||  \
    defined(__DOXYGEN__)
#define STM32_PLL2_FRACN             (STM32_PLL2_FRACN_VALUE << 3U)
#else
#error "invalid STM32_PLL2_FRACN_VALUE value specified"
#endif

/**
 * @brief   PLL3 FRACN field.
 */
#if ((STM32_PLL3_FRACN_VALUE >= 0) && (STM32_PLL3_FRACN_VALUE <= 8191)) ||  \
    defined(__DOXYGEN__)
#define STM32_PLL3_FRACN             (STM32_PLL3_FRACN_VALUE << 3U)
#else
#error "invalid STM32_PLL3_FRACN_VALUE value specified"
#endif

/**
 * @brief   PLL1 DIVP field.
 */
#if ((STM32_PLL1_DIVP_VALUE >= 1) && (STM32_PLL1_DIVP_VALUE <= 128)) &&     \
    ((STM32_PLL1_DIVP_VALUE == 1) || ((STM32_PLL1_DIVP_VALUE & 1) == 0))
#define STM32_PLL1_DIVP             ((STM32_PLL1_DIVP_VALUE - 1U) << 9U)
#else
#error "invalid STM32_PLL1_DIVP_VALUE value specified"
#endif

/**
 * @brief   PLL2 DIVP field.
 */
#if ((STM32_PLL2_DIVP_VALUE >= 1) && (STM32_PLL2_DIVP_VALUE <= 128)) ||     \
    defined(__DOXYGEN__)
#define STM32_PLL2_DIVP             ((STM32_PLL2_DIVP_VALUE - 1U) << 9U)
#else
#error "invalid STM32_PLL2_DIVP_VALUE value specified"
#endif

/**
 * @brief   PLL3 DIVP field.
 */
#if ((STM32_PLL3_DIVP_VALUE >= 1) && (STM32_PLL3_DIVP_VALUE <= 128)) ||     \
    defined(__DOXYGEN__)
#define STM32_PLL3_DIVP             ((STM32_PLL3_DIVP_VALUE - 1U) << 9U)
#else
#error "invalid STM32_PLL3_DIVP_VALUE value specified"
#endif

/**
 * @brief   PLL1 DIVQ field.
 */
#if ((STM32_PLL1_DIVQ_VALUE >= 1) && (STM32_PLL1_DIVQ_VALUE <= 128)) ||     \
    defined(__DOXYGEN__)
#define STM32_PLL1_DIVQ             ((STM32_PLL1_DIVQ_VALUE - 1U) << 16U)
#else
#error "invalid STM32_PLL1_DIVQ_VALUE value specified"
#endif

/**
 * @brief   PLL2 DIVQ field.
 */
#if ((STM32_PLL2_DIVQ_VALUE >= 1) && (STM32_PLL2_DIVQ_VALUE <= 128)) ||     \
    defined(__DOXYGEN__)
#define STM32_PLL2_DIVQ             ((STM32_PLL2_DIVQ_VALUE - 1U) << 16U)
#else
#error "invalid STM32_PLL2_DIVQ_VALUE value specified"
#endif

/**
 * @brief   PLL3 DIVQ field.
 */
#if ((STM32_PLL3_DIVQ_VALUE >= 1) && (STM32_PLL3_DIVQ_VALUE <= 128)) ||     \
    defined(__DOXYGEN__)
#define STM32_PLL3_DIVQ             ((STM32_PLL3_DIVQ_VALUE - 1U) << 16U)
#else
#error "invalid STM32_PLL3_DIVQ_VALUE value specified"
#endif

/**
 * @brief   PLL1 DIVR field.
 */
#if ((STM32_PLL1_DIVR_VALUE >= 1) && (STM32_PLL1_DIVR_VALUE <= 128)) ||     \
    defined(__DOXYGEN__)
#define STM32_PLL1_DIVR             ((STM32_PLL1_DIVR_VALUE - 1U) << 24U)
#else
#error "invalid STM32_PLL1_DIVR_VALUE value specified"
#endif

/**
 * @brief   PLL2 DIVR field.
 */
#if ((STM32_PLL2_DIVR_VALUE >= 1) && (STM32_PLL2_DIVR_VALUE <= 128)) ||     \
    defined(__DOXYGEN__)
#define STM32_PLL2_DIVR             ((STM32_PLL2_DIVR_VALUE - 1U) << 24U)
#else
#error "invalid STM32_PLL2_DIVR_VALUE value specified"
#endif

/**
 * @brief   PLL3 DIVR field.
 */
#if ((STM32_PLL3_DIVR_VALUE >= 1) && (STM32_PLL3_DIVR_VALUE <= 128)) ||     \
    defined(__DOXYGEN__)
#define STM32_PLL3_DIVR             ((STM32_PLL3_DIVR_VALUE - 1U) << 24U)
#else
#error "invalid STM32_PLL3_DIVR_VALUE value specified"
#endif

/**
 * @brief   PLL1 VCO frequency.
 */
#define STM32_PLL1_VCO_CK           (STM32_PLL1_REF_CK * STM32_PLL1_DIVN_VALUE)

/*
 * PLL1 frequency check.
 */
#if STM32_PLL1_REF_CK < STM32_PLLIN_THRESHOLD1
/* Check for medium range.*/
#if (STM32_PLL1_VCO_CK < STM32_PLLVCO_MEDIUM_MIN) || (STM32_PLL1_VCO_CK > STM32_PLLVCO_MEDIUM_MAX)
#error "STM32_PLL1_VCO_CK outside acceptable range (STM32_PLLVCO_MEDIUM_MIN..STM32_PLLVCO_MEDIUM_MAX)"
#endif

#elif STM32_PLL1_REF_CK == STM32_PLLIN_THRESHOLD1
/* Check for medium or wide range.*/
#if (STM32_PLL1_VCO_CK < STM32_PLLVCO_MIN) || (STM32_PLL1_VCO_CK > STM32_PLLVCO_MAX)
#error "STM32_PLL1_VCO_CK outside acceptable range (STM32_PLLVCO_MIN..STM32_PLLVCO_MAX)"
#endif

#elif STM32_PLL1_REF_CK > STM32_PLLIN_THRESHOLD1
/* Check for wide range.*/
#if (STM32_PLL1_VCO_CK < STM32_PLLVCO_WIDE_MIN) || (STM32_PLL1_VCO_CK > STM32_PLLVCO_WIDE_MAX)
#error "STM32_PLL1_VCO_CK outside acceptable range (STM32_PLLVCO_WIDE_MIN..STM32_PLLVCO_WIDE_MAX)"
#endif
#endif

/*
 * PLL1 VCO mode.
 */
#if ((STM32_PLL1_VCO_CK >= STM32_PLLVCO_MEDIUM_MIN) &&                      \
     (STM32_PLL1_VCO_CK <= STM32_PLLVCO_MEDIUM_MAX)) || defined(__DOXYGEN__)
#define STM32_PLLCFGR_PLL1VCOSEL    RCC_PLLCFGR_PLL1VCOSEL
#else
#define STM32_PLLCFGR_PLL1VCOSEL    0U
#endif

/**
 * @brief   PLL2 VCO frequency.
 */
#define STM32_PLL2_VCO_CK           (STM32_PLL2_REF_CK * STM32_PLL2_DIVN_VALUE)

/*
 * PLL2 frequency check.
 */
#if STM32_PLL2_REF_CK < STM32_PLLIN_THRESHOLD1
/* Check for medium range.*/
#if (STM32_PLL2_VCO_CK < STM32_PLLVCO_MEDIUM_MIN) || (STM32_PLL2_VCO_CK > STM32_PLLVCO_MEDIUM_MAX)
#error "STM32_PLL2_VCO_CK outside acceptable range (STM32_PLLVCO_MEDIUM_MIN..STM32_PLLVCO_MEDIUM_MAX)"
#endif

#elif STM32_PLL2_REF_CK == STM32_PLLIN_THRESHOLD1
/* Check for medium or wide range.*/
#if (STM32_PLL2_VCO_CK < STM32_PLLVCO_MIN) || (STM32_PLL2_VCO_CK > STM32_PLLVCO_MAX)
#error "STM32_PLL2_VCO_CK outside acceptable range (STM32_PLLVCO_MIN..STM32_PLLVCO_MAX)"
#endif

#elif STM32_PLL2_REF_CK > STM32_PLLIN_THRESHOLD1
/* Check for wide range.*/
#if (STM32_PLL2_VCO_CK < STM32_PLLVCO_WIDE_MIN) || (STM32_PLL2_VCO_CK > STM32_PLLVCO_WIDE_MAX)
#error "STM32_PLL2_VCO_CK outside acceptable range (STM32_PLLVCO_WIDE_MIN..STM32_PLLVCO_WIDE_MAX)"
#endif
#endif

/*
 * PLL2 VCO mode.
 */
#if ((STM32_PLL2_VCO_CK >= STM32_PLLVCO_MEDIUM_MIN) &&                      \
     (STM32_PLL2_VCO_CK <= STM32_PLLVCO_MEDIUM_MAX)) || defined(__DOXYGEN__)
#define STM32_PLLCFGR_PLL2VCOSEL    RCC_PLLCFGR_PLL2VCOSEL
#else
#define STM32_PLLCFGR_PLL2VCOSEL    0U
#endif

/**
 * @brief   PLL3 VCO frequency.
 */
#define STM32_PLL3_VCO_CK           (STM32_PLL3_REF_CK * STM32_PLL3_DIVN_VALUE)

/*
 * PLL3 frequency check.
 */
#if STM32_PLL3_REF_CK < STM32_PLLIN_THRESHOLD1
/* Check for medium range.*/
#if (STM32_PLL3_VCO_CK < STM32_PLLVCO_MEDIUM_MIN) || (STM32_PLL3_VCO_CK > STM32_PLLVCO_MEDIUM_MAX)
#error "STM32_PLL3_VCO_CK outside acceptable range (STM32_PLLVCO_MEDIUM_MIN..STM32_PLLVCO_MEDIUM_MAX)"
#endif

#elif STM32_PLL3_REF_CK == STM32_PLLIN_THRESHOLD1
/* Check for medium or wide range.*/
#if (STM32_PLL3_VCO_CK < STM32_PLLVCO_MIN) || (STM32_PLL3_VCO_CK > STM32_PLLVCO_MAX)
#error "STM32_PLL3_VCO_CK outside acceptable range (STM32_PLLVCO_MIN..STM32_PLLVCO_MAX)"
#endif

#elif STM32_PLL3_REF_CK > STM32_PLLIN_THRESHOLD1
/* Check for wide range.*/
#if (STM32_PLL3_VCO_CK < STM32_PLLVCO_WIDE_MIN) || (STM32_PLL3_VCO_CK > STM32_PLLVCO_WIDE_MAX)
#error "STM32_PLL3_VCO_CK outside acceptable range (STM32_PLLVCO_WIDE_MIN..STM32_PLLVCO_WIDE_MAX)"
#endif
#endif

/*
 * PLL3 VCO mode.
 */
#if ((STM32_PLL3_VCO_CK >= STM32_PLLVCO_MEDIUM_MIN) &&                      \
     (STM32_PLL3_VCO_CK <= STM32_PLLVCO_MEDIUM_MAX)) || defined(__DOXYGEN__)
#define STM32_PLLCFGR_PLL3VCOSEL    RCC_PLLCFGR_PLL3VCOSEL
#else
#define STM32_PLLCFGR_PLL3VCOSEL    0U
#endif

#if ((STM32_PLL1_ENABLED == TRUE) && (STM32_PLL1_P_ENABLED == TRUE)) ||     \
    defined(__DOXYGEN__)
/**
 * @brief   PLL1 P output clock frequency.
 */
#define STM32_PLL1_P_CK             (STM32_PLL1_VCO_CK / STM32_PLL1_DIVP_VALUE)

/*
 * PLL1 P output frequency range check.
 */
#if (STM32_PLL1_P_CK < STM32_PLLOUT_MIN) || (STM32_PLL1_P_CK > STM32_PLLOUT_MAX)
#error "STM32_PLL1_P_CLKOUT outside acceptable range (STM32_PLLOUT_MIN..STM32_PLLOUT_MAX)"
#endif
#else
#define STM32_PLL1_P_CK             0U
#endif

#if ((STM32_PLL2_ENABLED == TRUE) && (STM32_PLL2_P_ENABLED == TRUE)) ||     \
    defined(__DOXYGEN__)
/**
 * @brief   PLL2 P output clock frequency.
 */
#define STM32_PLL2_P_CK             (STM32_PLL2_VCO_CK / STM32_PLL2_DIVP_VALUE)

/*
 * PLL2 P output frequency range check.
 */
#if (STM32_PLL2_P_CK < STM32_PLLOUT_MIN) || (STM32_PLL2_P_CK > STM32_PLLOUT_MAX)
#error "STM32_PLL2_P_CLKOUT outside acceptable range (STM32_PLLOUT_MIN..STM32_PLLOUT_MAX)"
#endif
#else
#define STM32_PLL2_P_CK             0U
#endif

#if ((STM32_PLL3_ENABLED == TRUE) && (STM32_PLL3_P_ENABLED == TRUE)) ||     \
    defined(__DOXYGEN__)
/**
 * @brief   PLL3 P output clock frequency.
 */
#define STM32_PLL3_P_CK             (STM32_PLL3_VCO_CK / STM32_PLL3_DIVP_VALUE)

/*
 * PLL3 P output frequency range check.
 */
#if (STM32_PLL3_P_CK < STM32_PLLOUT_MIN) || (STM32_PLL3_P_CK > STM32_PLLOUT_MAX)
#error "STM32_PLL3_P_CLKOUT outside acceptable range (STM32_PLLOUT_MIN..STM32_PLLOUT_MAX)"
#endif
#else
#define STM32_PLL3_P_CK             0U
#endif

#if ((STM32_PLL1_ENABLED == TRUE) && (STM32_PLL1_Q_ENABLED == TRUE)) ||     \
    defined(__DOXYGEN__)
/**
 * @brief   PLL1 Q output clock frequency.
 */
#define STM32_PLL1_Q_CK             (STM32_PLL1_VCO_CK / STM32_PLL1_DIVQ_VALUE)

/*
 * PLL1 Q output frequency range check.
 */
#if (STM32_PLL1_Q_CK < STM32_PLLOUT_MIN) || (STM32_PLL1_Q_CK > STM32_PLLOUT_MAX)
#error "STM32_PLL1_Q_CLKOUT outside acceptable range (STM32_PLLOUT_MIN..STM32_PLLOUT_MAX)"
#endif
#else
#define STM32_PLL1_Q_CK             0U
#endif

#if ((STM32_PLL2_ENABLED == TRUE) && (STM32_PLL2_Q_ENABLED == TRUE)) ||     \
    defined(__DOXYGEN__)
/**
 * @brief   PLL2 Q output clock frequency.
 */
#define STM32_PLL2_Q_CK             (STM32_PLL2_VCO_CK / STM32_PLL2_DIVQ_VALUE)

/*
 * PLL2 Q output frequency range check.
 */
#if (STM32_PLL2_Q_CK < STM32_PLLOUT_MIN) || (STM32_PLL2_Q_CK > STM32_PLLOUT_MAX)
#error "STM32_PLL2_Q_CLKOUT outside acceptable range (STM32_PLLOUT_MIN..STM32_PLLOUT_MAX)"
#endif
#else
#define STM32_PLL2_Q_CK             0U
#endif

#if ((STM32_PLL3_ENABLED == TRUE) && (STM32_PLL3_Q_ENABLED == TRUE)) ||     \
    defined(__DOXYGEN__)
/**
 * @brief   PLL3 Q output clock frequency.
 */
#define STM32_PLL3_Q_CK             (STM32_PLL3_VCO_CK / STM32_PLL3_DIVQ_VALUE)

/*
 * PLL3 Q output frequency range check.
 */
#if (STM32_PLL3_Q_CK < STM32_PLLOUT_MIN) || (STM32_PLL3_Q_CK > STM32_PLLOUT_MAX)
#error "STM32_PLL3_Q_CLKOUT outside acceptable range (STM32_PLLOUT_MIN..STM32_PLLOUT_MAX)"
#endif
#else
#define STM32_PLL3_Q_CK             0U
#endif

#if ((STM32_PLL1_ENABLED == TRUE) && (STM32_PLL1_R_ENABLED == TRUE)) ||     \
    defined(__DOXYGEN__)
/**
 * @brief   PLL1 R output clock frequency.
 */
#define STM32_PLL1_R_CK             (STM32_PLL1_VCO_CK / STM32_PLL1_DIVR_VALUE)

/*
 * PLL1 R output frequency range check.
 */
#if (STM32_PLL1_R_CK < STM32_PLLOUT_MIN) || (STM32_PLL1_R_CK > STM32_PLLOUT_MAX)
#error "STM32_PLL1_R_CLKOUT outside acceptable range (STM32_PLLOUT_MIN..STM32_PLLOUT_MAX)"
#endif
#else
#define STM32_PLL1_R_CK             0U
#endif

#if ((STM32_PLL2_ENABLED == TRUE) && (STM32_PLL2_R_ENABLED == TRUE)) ||     \
    defined(__DOXYGEN__)
/**
 * @brief   PLL2 R output clock frequency.
 */
#define STM32_PLL2_R_CK             (STM32_PLL2_VCO_CK / STM32_PLL2_DIVR_VALUE)

/*
 * PLL2 R output frequency range check.
 */
#if (STM32_PLL2_R_CK < STM32_PLLOUT_MIN) || (STM32_PLL2_R_CK > STM32_PLLOUT_MAX)
#error "STM32_PLL2_R_CLKOUT outside acceptable range (STM32_PLLOUT_MIN..STM32_PLLOUT_MAX)"
#endif
#else
#define STM32_PLL2_R_CK             0U
#endif

#if ((STM32_PLL3_ENABLED == TRUE) && (STM32_PLL3_R_ENABLED == TRUE)) ||     \
    defined(__DOXYGEN__)
/**
 * @brief   PLL3 R output clock frequency.
 */
#define STM32_PLL3_R_CK             (STM32_PLL3_VCO_CK / STM32_PLL3_DIVR_VALUE)

/*
 * PLL3 R output frequency range check.
 */
#if (STM32_PLL3_R_CK < STM32_PLLOUT_MIN) || (STM32_PLL3_R_CK > STM32_PLLOUT_MAX)
#error "STM32_PLL3_R_CLKOUT outside acceptable range (STM32_PLLOUT_MIN..STM32_PLLOUT_MAX)"
#endif
#else
#define STM32_PLL3_R_CK             0U
#endif

/**
 * @brief   System clock source.
 */
#if (STM32_SW == STM32_SW_HSI_CK) || defined(__DOXYGEN__)
#define STM32_SYS_CK                STM32_HSI_CK

#elif (STM32_SW == STM32_SW_CSI_CK)
#define STM32_SYS_CK                STM32_CSI_CK

#elif (STM32_SW == STM32_SW_HSE_CK)
#define STM32_SYS_CK                STM32_HSE_CK

#elif (STM32_SW == STM32_SW_PLL1_P_CK)
#define STM32_SYS_CK                STM32_PLL1_P_CK

#else
#error "invalid STM32_SW value specified"
#endif

/*
 * Check on the system clock.
 */
#if STM32_SYS_CK > STM32_SYSCLK_MAX
#error "STM32_SYS_CK above maximum rated frequency (STM32_SYSCLK_MAX)"
#endif

/**
 * @brief   Peripherals clock source.
 */
#if (STM32_CKPERSEL == STM32_CKPERSEL_HSI_CK) || defined(__DOXYGEN__)
#define STM32_PER_CK                STM32_HSI_CK

#elif (STM32_CKPERSEL == STM32_CKPERSEL_CSI_CK)
#define STM32_PER_CK                STM32_CSI_CK

#elif (STM32_CKPERSEL == STM32_CKPERSEL_HSE_CK)
#define STM32_PER_CK                STM32_HSE_CK

#else
#error "invalid STM32_CKPERSEL value specified"
#endif

/*
 * Check on the peripherals clock.
 */
#if STM32_PER_CK > STM32_HCLK_MAX
#error "STM32_PER_CK above maximum rated frequency (STM32_HCLK_MAX)"
#endif

/**
 * @brief   MCO1 divider clock.
 */
#if (STM32_MCO1SEL == STM32_MCO1SEL_HSI_CK) || defined(__DOXYGEN__)
#define STM32_MCO1DIVCLK            STM32_HSI_CK

#elif STM32_MCO1SEL == STM32_MCO1SEL_LSE_CK
#define STM32_MCO1DIVCLK            STM32_LSE_CK

#elif STM32_MCO1SEL == STM32_MCO1SEL_HSE_CK
#define STM32_MCO1DIVCLK            STM32_HSE_CK

#elif STM32_MCO1SEL == STM32_MCO1SEL_PLL1_Q_CK
#define STM32_MCO1DIVCLK            STM32_PLL1_P_CK

#elif STM32_MCO1SEL == STM32_MCO1SEL_HSI48_CK
#define STM32_MCO1DIVCLK            STM32_HSI48_CK

#else
#error "invalid STM32_MCO1SEL value specified"
#endif

/**
 * @brief   MCO1 output pin clock.
 */
#if (STM32_MCO1PRE_VALUE < 1) || (STM32_MCO1PRE_VALUE > 15)
#error "STM32_MCO1PRE_VALUE outside acceptable range (1..15)"
#endif

/**
 * @brief   MCO2 divider clock.
 */
#if (STM32_MCO2SEL == STM32_MCO2SEL_SYS_CK) || defined(__DOXYGEN__)
#define STM32_MCO2DIVCLK            STM32_SYS_CK

#elif STM32_MCO2SEL == STM32_MCO2SEL_PLL1_P_CK
#define STM32_MCO2DIVCLK            STM32_PLL2_P_CK

#elif STM32_MCO2SEL == STM32_MCO2SEL_HSE_CK
#define STM32_MCO2DIVCLK            STM32_HSE_CK

#elif STM32_MCO2SEL == STM32_MCO2SEL_PLL2_P_CK
#define STM32_MCO2DIVCLK            STM32_PLL2_P_CK

#elif STM32_MCO2SEL == STM32_MCO2SEL_CSI_CK
#define STM32_MCO2DIVCLK            STM32_CSI_CK

#elif STM32_MCO2SEL == STM32_MCO2SEL_LSI_CK
#define STM32_MCO2DIVCLK            STM32_LSI_CK

#else
#error "invalid STM32_MCO2SEL value specified"
#endif

/**
 * @brief   MCO2 output pin clock.
 */
#if (STM32_MCO2PRE_VALUE < 1) || (STM32_MCO2PRE_VALUE > 15)
#error "STM32_MCO2PRE_VALUE outside acceptable range (1..15)"
#endif

/**
 * @brief   RTC clock.
 */
#if (STM32_RTCSEL == STM32_RTCSEL_NOCLK) || defined(__DOXYGEN__)
#define STM32_RTC_CK                0

#elif STM32_RTCSEL == STM32_RTCSEL_LSE_CK
#define STM32_RTC_CK                STM32_LSE_CK

#elif STM32_RTCSEL == STM32_RTCSEL_LSI_CK
#define STM32_RTC_CK                STM32_LSI_CK

#elif STM32_RTCSEL == STM32_RTCSEL_HSE_1M_CK
#define STM32_RTC_CK                STM32_HSE_1M_CK

#else
#error "invalid STM32_RTCSEL value specified"
#endif

/*
 * Check on the RTC clock.
 */
#if STM32_RTC_CK > 1000000
#error "STM32_RTC_CK above maximum rated frequency (1000000)"
#endif

/**
 * @brief   D1CPRE clock.
 */
#if (STM32_D1CPRE == STM32_D1CPRE_DIV1) || defined(__DOXYGEN__)
#define STM32_SYS_D1CPRE_CK         (STM32_SYS_CK / 1U)
#elif STM32_D1CPRE == STM32_D1CPRE_DIV2
#define STM32_SYS_D1CPRE_CK         (STM32_SYS_CK / 2U)
#elif STM32_D1CPRE == STM32_D1CPRE_DIV4
#define STM32_SYS_D1CPRE_CK         (STM32_SYS_CK / 4U)
#elif STM32_D1CPRE == STM32_D1CPRE_DIV8
#define STM32_SYS_D1CPRE_CK         (STM32_SYS_CK / 8U)
#elif STM32_D1CPRE == STM32_D1CPRE_DIV16
#define STM32_SYS_D1CPRE_CK         (STM32_SYS_CK / 16U)
#elif STM32_D1CPRE == STM32_D1CPRE_DIV64
#define STM32_SYS_D1CPRE_CK         (STM32_SYS_CK / 64U)
#elif STM32_D1CPRE == STM32_D1CPRE_DIV128
#define STM32_SYS_D1CPRE_CK         (STM32_SYS_CK / 128U)
#elif STM32_D1CPRE == STM32_D1CPRE_DIV256
#define STM32_SYS_D1CPRE_CK         (STM32_SYS_CK / 256U)
#elif STM32_D1CPRE == STM32_D1CPRE_DIV512
#define STM32_SYS_D1CPRE_CK         (STM32_SYS_CK / 512U)
#else
#error "invalid STM32_D1CPRE value specified"
#endif

/**
 * @brief   HCLK clock.
 */
#if (STM32_D1HPRE == STM32_D1HPRE_DIV1) || defined(__DOXYGEN__)
#define STM32_HCLK                  (STM32_SYS_D1CPRE_CK / 1U)
#elif STM32_D1HPRE == STM32_D1HPRE_DIV2
#define STM32_HCLK                  (STM32_SYS_D1CPRE_CK / 2U)
#elif STM32_D1HPRE == STM32_D1HPRE_DIV4
#define STM32_HCLK                  (STM32_SYS_D1CPRE_CK / 4U)
#elif STM32_D1HPRE == STM32_D1HPRE_DIV8
#define STM32_HCLK                  (STM32_SYS_D1CPRE_CK / 8U)
#elif STM32_D1HPRE == STM32_D1HPRE_DIV16
#define STM32_HCLK                  (STM32_SYS_D1CPRE_CK / 16U)
#elif STM32_D1HPRE == STM32_D1HPRE_DIV64
#define STM32_HCLK                  (STM32_SYS_D1CPRE_CK / 64U)
#elif STM32_D1HPRE == STM32_D1HPRE_DIV128
#define STM32_HCLK                  (STM32_SYS_D1CPRE_CK / 128U)
#elif STM32_D1HPRE == STM32_D1HPRE_DIV256
#define STM32_HCLK                  (STM32_SYS_D1CPRE_CK / 256U)
#elif STM32_D1HPRE == STM32_D1HPRE_DIV512
#define STM32_HCLK                  (STM32_SYS_D1CPRE_CK / 512U)
#else
#error "invalid STM32_D1HPRE value specified"
#endif

/*
 * AHB frequency check.
 */
#if STM32_HCLK > STM32_HCLK_MAX
#error "STM32_HCLK exceeding maximum frequency (STM32_HCLK_MAX)"
#endif

/**
 * @brief   Core 1 clock.
 */
#define STM32_CORE1_CK              STM32_SYS_D1CPRE_CK

/**
 * @brief   Core 2 clock.
 */
#define STM32_CORE2_CK              0U

/**
 * @brief   Current core clock.
 */
#define STM32_CORE_CK               STM32_CORE1_CK

/**
 * @brief   D1 PCLK3 clock.
 */
#if (STM32_D1PPRE3 == STM32_D1PPRE3_DIV1) || defined(__DOXYGEN__)
#define STM32_PCLK3                 (STM32_HCLK / 1U)
#elif STM32_D1PPRE3 == STM32_D1PPRE3_DIV2
#define STM32_PCLK3                 (STM32_HCLK / 2U)
#elif STM32_D1PPRE3 == STM32_D1PPRE3_DIV4
#define STM32_PCLK3                 (STM32_HCLK / 4U)
#elif STM32_D1PPRE3 == STM32_D1PPRE3_DIV8
#define STM32_PCLK3                 (STM32_HCLK / 8U)
#elif STM32_D1PPRE3 == STM32_D1PPRE3_DIV16
#define STM32_PCLK3                 (STM32_HCLK / 16U)
#else
#error "invalid STM32_D1PPRE3 value specified"
#endif

/*
 * D1 PCLK3 frequency check.
 */
#if STM32_PCLK3 > STM32_PCLK3_MAX
#error "STM32_PCLK3 exceeding maximum frequency (STM32_PCLK3_MAX)"
#endif

/**
 * @brief   D2 PCLK1 clock.
 */
#if (STM32_D2PPRE1 == STM32_D2PPRE1_DIV1) || defined(__DOXYGEN__)
#define STM32_PCLK1                 (STM32_HCLK / 1U)
#elif STM32_D2PPRE1 == STM32_D2PPRE1_DIV2
#define STM32_PCLK1                 (STM32_HCLK / 2U)
#elif STM32_D2PPRE1 == STM32_D2PPRE1_DIV4
#define STM32_PCLK1                 (STM32_HCLK / 4U)
#elif STM32_D2PPRE1 == STM32_D2PPRE1_DIV8
#define STM32_PCLK1                 (STM32_HCLK / 8U)
#elif STM32_D2PPRE1 == STM32_D2PPRE1_DIV16
#define STM32_PCLK1                 (STM32_HCLK / 16U)
#else
#error "invalid STM32_D2PPRE1 value specified"
#endif

/*
 * D2 PCLK1 frequency check.
 */
#if STM32_PCLK1 > STM32_PCLK1_MAX
#error "STM32_PCLK1 exceeding maximum frequency (STM32_PCLK1_MAX)"
#endif

/**
 * @brief   D2 PCLK2 clock.
 */
#if (STM32_D2PPRE2 == STM32_D2PPRE2_DIV1) || defined(__DOXYGEN__)
#define STM32_PCLK2                 (STM32_HCLK / 1U)
#elif STM32_D2PPRE2 == STM32_D2PPRE2_DIV2
#define STM32_PCLK2                 (STM32_HCLK / 2U)
#elif STM32_D2PPRE2 == STM32_D2PPRE2_DIV4
#define STM32_PCLK2                 (STM32_HCLK / 4U)
#elif STM32_D2PPRE2 == STM32_D2PPRE2_DIV8
#define STM32_PCLK2                 (STM32_HCLK / 8U)
#elif STM32_D2PPRE2 == STM32_D2PPRE2_DIV16
#define STM32_PCLK2                 (STM32_HCLK / 16U)
#else
#error "invalid STM32_D2PPRE2 value specified"
#endif

/*
 * D2 PCLK2 frequency check.
 */
#if STM32_PCLK2 > STM32_PCLK2_MAX
#error "STM32_PCLK2 exceeding maximum frequency (STM32_PCLK2_MAX)"
#endif

/**
 * @brief   D3 PCLK4 clock.
 */
#if (STM32_D3PPRE4 == STM32_D3PPRE4_DIV1) || defined(__DOXYGEN__)
#define STM32_PCLK4                 (STM32_HCLK / 1U)
#elif STM32_D3PPRE4 == STM32_D3PPRE4_DIV2
#define STM32_PCLK4                 (STM32_HCLK / 2U)
#elif STM32_D3PPRE4 == STM32_D3PPRE4_DIV4
#define STM32_PCLK4                 (STM32_HCLK / 4U)
#elif STM32_D3PPRE4 == STM32_D3PPRE4_DIV8
#define STM32_PCLK4                 (STM32_HCLK / 8U)
#elif STM32_D3PPRE4 == STM32_D3PPRE4_DIV16
#define STM32_PCLK4                 (STM32_HCLK / 16U)
#else
#error "invalid STM32_D3PPRE4 value specified"
#endif

/*
 * D3 PCLK4 frequency check.
 */
#if STM32_PCLK4 > STM32_PCLK4_MAX
#error "STM32_PCLK4 exceeding maximum frequency (STM32_PCLK4_MAX)"
#endif

/**
 * @brief   Flash settings.
 */
#if (STM32_HCLK <= STM32_0WS_THRESHOLD) || defined(__DOXYGEN__)
#define STM32_FLASHBITS             0x00000000

#elif STM32_HCLK <= STM32_1WS_THRESHOLD
#define STM32_FLASHBITS             0x00000001

#elif STM32_HCLK <= STM32_2WS_THRESHOLD
#define STM32_FLASHBITS             0x00000002

#elif STM32_HCLK <= STM32_3WS_THRESHOLD
#define STM32_FLASHBITS             0x00000003

#elif STM32_HCLK <= STM32_4WS_THRESHOLD
#define STM32_FLASHBITS             0x00000004

#else
#define STM32_FLASHBITS             0x00000007
#endif

#if (STM32_D2PPRE1 == STM32_D2PPRE1_DIV1) || defined(__DOXYGEN__)
/**
 * @brief   Clock of timers connected to APB1
 */
#define STM32_TIMCLK1               (STM32_PCLK1 * 1)
#else
#if (STM32_TIMPRE_ENABLE == FALSE) || (STM32_D2PPRE1 == STM32_D2PPRE1_DIV2)
#define STM32_TIMCLK1               (STM32_PCLK1 * 2)
#else
#define STM32_TIMCLK1               (STM32_PCLK1 * 4)
#endif
#endif

#if (STM32_D2PPRE2 == STM32_D2PPRE2_DIV1) || defined(__DOXYGEN__)
/**
 * @brief   Clock of timers connected to APB2.
 */
#define STM32_TIMCLK2               (STM32_PCLK2 * 1)
#else
#if (STM32_TIMPRE_ENABLE == FALSE) || (STM32_D2PPRE2 == STM32_D2PPRE2_DIV2)
#define STM32_TIMCLK2               (STM32_PCLK2 * 2)
#else
#define STM32_TIMCLK2               (STM32_PCLK2 * 4)
#endif
#endif

#if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_PCLK1) || defined(__DOXYGEN__)
/**
 * @brief   LPTIM1 clock.
 */
#define STM32_LPTIM1CLK             STM32_PCLK1

#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_PLL2_P_CK
#define STM32_LPTIM1CLK             STM32_PLL2_P_CK
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_PLL3_R_CK
#define STM32_LPTIM1CLK             STM32_PLL3_R_CK
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSE_CK
#define STM32_LPTIM1CLK             STM32_LSE_CK
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSI_CK
#define STM32_LPTIM1CLK             STM32_LSI_CK
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_PER_CK
#define STM32_LPTIM1CLK             STM32_PER_CK
#else
#error "invalid source selected for STM32_LPTIM1SEL clock"
#endif

#if (STM32_LPTIM2SEL == STM32_LPTIM2SEL_PCLK4) || defined(__DOXYGEN__)
/**
 * @brief   LPTIM2 clock.
 */
#define STM32_LPTIM2CLK             STM32_PCLK4

#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_PLL2_P_CK
#define STM32_LPTIM2CLK             STM32_PLL2_P_CK
#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_PLL3_P_CK
#define STM32_LPTIM2CLK             STM32_PLL3_P_CK
#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_LSE_CK
#define STM32_LPTIM2CLK             STM32_LSE_CK
#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_LSI_CK
#define STM32_LPTIM2CLK             STM32_LSI_CK
#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_PER_CK
#define STM32_LPTIM2CLK             STM32_PER_CK
#else
#error "invalid source selected for STM32_LPTIM2SEL clock"
#endif

#if (STM32_LPTIM345SEL == STM32_LPTIM345SEL_PCLK4) || defined(__DOXYGEN__)
/**
 * @brief   LPTIM3 clock.
 */
#define STM32_LPTIM3CLK             STM32_PCLK4

/**
 * @brief   LPTIM4 clock.
 */
#define STM32_LPTIM4CLK             STM32_PCLK4

/**
 * @brief   LPTIM5 clock.
 */
#define STM32_LPTIM5CLK             STM32_PCLK4

#elif STM32_LPTIM345SEL == STM32_LPTIM345SEL_PLL2_P_CK
#define STM32_LPTIM3CLK             STM32_PLL2_P_CK
#define STM32_LPTIM4CLK             STM32_PLL2_P_CK
#define STM32_LPTIM5CLK             STM32_PLL2_P_CK
#elif STM32_LPTIM345SEL == STM32_LPTIM345SEL_PLL3_P_CK
#define STM32_LPTIM3CLK             STM32_PLL3_P_CK
#define STM32_LPTIM4CLK             STM32_PLL3_P_CK
#define STM32_LPTIM5CLK             STM32_PLL3_P_CK
#elif STM32_LPTIM345SEL == STM32_LPTIM345SEL_LSE_CK
#define STM32_LPTIM3CLK             STM32_LSE_CK
#define STM32_LPTIM4CLK             STM32_LSE_CK
#define STM32_LPTIM5CLK             STM32_LSE_CK
#elif STM32_LPTIM345SEL == STM32_LPTIM345SEL_LSI_CK
#define STM32_LPTIM3CLK             STM32_LSI_CK
#define STM32_LPTIM4CLK             STM32_LSI_CK
#define STM32_LPTIM5CLK             STM32_LSI_CK
#elif STM32_LPTIM345SEL == STM32_LPTIM345SEL_PER_CK
#define STM32_LPTIM3CLK             STM32_PER_CK
#define STM32_LPTIM4CLK             STM32_PER_CK
#define STM32_LPTIM5CLK             STM32_PER_CK
#else
#error "invalid source selected for STM32_LPTIM345SEL clock"
#endif

#if (STM32_USART16910SEL == STM32_USART16910SEL_PCLK2) || defined(__DOXYGEN__)
/**
 * @brief   USART1 clock.
 */
#define STM32_USART1CLK             STM32_PCLK2

/**
 * @brief   USART6 clock.
 */
#define STM32_USART6CLK             STM32_PCLK2

/**
 * @brief   UART9 clock.
 */
#define STM32_UART9CLK              STM32_PCLK2

/**
 * @brief   USART10 clock.
 */
#define STM32_USART10CLK            STM32_PCLK2

#elif STM32_USART16910SEL == STM32_USART16910SEL_PLL2_Q_CK
#define STM32_USART1CLK             STM32_PLL2_Q_CK
#define STM32_USART6CLK             STM32_PLL2_Q_CK
#define STM32_UART9CLK              STM32_PLL2_Q_CK
#define STM32_USART10CLK            STM32_PLL2_Q_CK
#elif STM32_USART16910SEL == STM32_USART16910SEL_PLL3_Q_CK
#define STM32_USART1CLK             STM32_PLL3_Q_CK
#define STM32_USART6CLK             STM32_PLL3_Q_CK
#define STM32_UART9CLK              STM32_PLL3_Q_CK
#define STM32_USART10CLK            STM32_PLL3_Q_CK
#elif STM32_USART16910SEL == STM32_USART16910SEL_HSI_KER_CK
#define STM32_USART1CLK             STM32_HSI_CK
#define STM32_USART6CLK             STM32_HSI_CK
#define STM32_UART9CLK              STM32_HSI_CK
#define STM32_USART10CLK            STM32_HSI_CK
#elif STM32_USART16910SEL == STM32_USART16910SEL_CSI_KER_CK
#define STM32_USART1CLK             STM32_CSI_CK
#define STM32_USART6CLK             STM32_CSI_CK
#define STM32_UART9CLK              STM32_CSI_CK
#define STM32_USART10CLK            STM32_CSI_CK
#elif STM32_USART16910SEL == STM32_USART16910SEL_LSE_CK
#define STM32_USART1CLK             STM32_LSE_CK
#define STM32_USART6CLK             STM32_LSE_CK
#define STM32_UART9CLK              STM32_LSE_CK
#define STM32_USART10CLK            STM32_LSE_CK
#else
#error "invalid source selected for STM32_USART16910SEL clock"
#endif

#if (STM32_USART234578SEL == STM32_USART234578SEL_PCLK1) || defined(__DOXYGEN__)
/**
 * @brief   USART2 clock.
 */
#define STM32_USART2CLK             STM32_PCLK1

/**
 * @brief   USART3 clock.
 */
#define STM32_USART3CLK             STM32_PCLK1

/**
 * @brief   USART4 clock.
 */
#define STM32_UART4CLK              STM32_PCLK1

/**
 * @brief   USART5 clock.
 */
#define STM32_UART5CLK              STM32_PCLK1

/**
 * @brief   USART7 clock.
 */
#define STM32_UART7CLK              STM32_PCLK1

/**
 * @brief   USART8 clock.
 */
#define STM32_UART8CLK              STM32_PCLK1

#elif STM32_USART234578SEL == STM32_USART234578SEL_PLL2_Q_CK
#define STM32_USART2CLK             STM32_PLL2_Q_CK
#define STM32_USART3CLK             STM32_PLL2_Q_CK
#define STM32_UART4CLK              STM32_PLL2_Q_CK
#define STM32_UART5CLK              STM32_PLL2_Q_CK
#define STM32_UART7CLK              STM32_PLL2_Q_CK
#define STM32_UART8CLK              STM32_PLL2_Q_CK
#elif STM32_USART234578SEL == STM32_USART234578SEL_PLL3_Q_CK
#define STM32_USART2CLK             STM32_PLL3_Q_CK
#define STM32_USART3CLK             STM32_PLL3_Q_CK
#define STM32_UART4CLK              STM32_PLL3_Q_CK
#define STM32_UART5CLK              STM32_PLL3_Q_CK
#define STM32_UART7CLK              STM32_PLL3_Q_CK
#define STM32_UART8CLK              STM32_PLL3_Q_CK
#elif STM32_USART234578SEL == STM32_USART234578SEL_HSI_KER_CK
#define STM32_USART2CLK             STM32_HSI_CK
#define STM32_USART3CLK             STM32_HSI_CK
#define STM32_UART4CLK              STM32_HSI_CK
#define STM32_UART5CLK              STM32_HSI_CK
#define STM32_UART7CLK              STM32_HSI_CK
#define STM32_UART8CLK              STM32_HSI_CK
#elif STM32_USART234578SEL == STM32_USART234578SEL_CSI_KER_CK
#define STM32_USART2CLK             STM32_CSI_CK
#define STM32_USART3CLK             STM32_CSI_CK
#define STM32_UART4CLK              STM32_CSI_CK
#define STM32_UART5CLK              STM32_CSI_CK
#define STM32_UART7CLK              STM32_CSI_CK
#define STM32_UART8CLK              STM32_CSI_CK
#elif STM32_USART234578SEL == STM32_USART234578SEL_LSE_CK
#define STM32_USART2CLK             STM32_LSE_CK
#define STM32_USART3CLK             STM32_LSE_CK
#define STM32_UART4CLK              STM32_LSE_CK
#define STM32_UART6CLK              STM32_LSE_CK
#define STM32_UART7CLK              STM32_LSE_CK
#define STM32_UART8CLK              STM32_LSE_CK
#else
#error "invalid source selected for STM32_USART234578SEL clock"
#endif

#if (STM32_LPUART1SEL == STM32_LPUART1SEL_PCLK4) || defined(__DOXYGEN__)
/**
 * @brief   LPUART1 clock.
 */
#define STM32_LPUART1CLK            STM32_PCLK4

#elif STM32_LPUART1SEL == STM32_LPUART1SEL_PLL2_Q_CK
#define STM32_LPUART1CLK            STM32_PLL2_Q_CK
#elif STM32_LPUART1SEL == STM32_LPUART1SEL_PLL3_Q_CK
#define STM32_LPUART1CLK            STM32_PLL3_Q_CK
#elif STM32_LPUART1SEL == STM32_LPUART1SEL_HSI_KER_CK
#define STM32_LPUART1CLK            STM32_HSI_CK
#elif STM32_LPUART1SEL == STM32_LPUART1SEL_CSI_KER_CK
#define STM32_LPUART1CLK            STM32_CSI_CK
#elif STM32_LPUART1SEL == STM32_LPUART1SEL_LSE_CK
#define STM32_LPUART1CLK            STM32_LSE_CK
#else
#error "invalid source selected for STM32_LPUART1SEL clock"
#endif

#if (STM32_SPI123SEL == STM32_SPI123SEL_PLL1_Q_CK) || defined(__DOXYGEN__)
/**
 * @brief   SPI1 clock.
 */
#define STM32_SPI1CLK               STM32_PLL1_Q_CK

/**
 * @brief   SPI2 clock.
 */
#define STM32_SPI2CLK               STM32_PLL1_Q_CK

/**
 * @brief   SPI3 clock.
 */
#define STM32_SPI3CLK               STM32_PLL1_Q_CK
#elif STM32_SPI123SEL == STM32_SPI123SEL_PLL2_P_CK
#define STM32_SPI1CLK               STM32_PLL2_P_CK
#define STM32_SPI2CLK               STM32_PLL2_P_CK
#define STM32_SPI3CLK               STM32_PLL2_P_CK
#elif STM32_SPI123SEL == STM32_SPI123SEL_PLL3_P_CK
#define STM32_SPI1CLK               STM32_PLL3_P_CK
#define STM32_SPI2CLK               STM32_PLL3_P_CK
#define STM32_SPI3CLK               STM32_PLL3_P_CK
#elif STM32_SPI123SEL == STM32_SPI123SEL_I2S_CKIN
#define STM32_SPI1CLK               0 /* Unknown, would require a board value */
#define STM32_SPI2CLK               0 /* Unknown, would require a board value */
#define STM32_SPI3CLK               0 /* Unknown, would require a board value */
#elif STM32_SPI123SEL == STM32_SPI123SEL_PER_CK
#define STM32_SPI1CLK               STM32_PER_CK
#define STM32_SPI2CLK               STM32_PER_CK
#define STM32_SPI3CLK               STM32_PER_CK
#else
#error "invalid source selected for STM32_SPI123SEL clock"
#endif

#if (STM32_SPI45SEL == STM32_SPI45SEL_PCLK2) || defined(__DOXYGEN__)
/**
 * @brief   SPI4 clock.
 */
#define STM32_SPI4CLK               STM32_PCLK2

/**
 * @brief   SPI5 clock.
 */
#define STM32_SPI5CLK               STM32_PCLK2

#elif STM32_SPI45SEL == STM32_SPI45SEL_PLL2_Q_CK
#define STM32_SPI4CLK               STM32_PLL2_Q_CK
#define STM32_SPI5CLK               STM32_PLL2_Q_CK
#elif STM32_SPI45SEL == STM32_SPI45SEL_PLL3_Q_CK
#define STM32_SPI4CLK               STM32_PLL3_Q_CK
#define STM32_SPI5CLK               STM32_PLL3_Q_CK
#elif STM32_SPI45SEL == STM32_SPI45SEL_HSI_KER_CK
#define STM32_SPI4CLK               STM32_HSI_CK
#define STM32_SPI5CLK               STM32_HSI_CK
#elif STM32_SPI45SEL == STM32_SPI45SEL_CSI_KER_CK
#define STM32_SPI4CLK               STM32_CSI_CK
#define STM32_SPI5CLK               STM32_CSI_CK
#elif STM32_SPI45SEL == STM32_SPI45SEL_HSE_CK
#define STM32_SPI4CLK               STM32_HSE_CK
#define STM32_SPI5CLK               STM32_HSE_CK
#else
#error "invalid source selected for STM32_SPI45SEL clock"
#endif

#if (STM32_SPI6SEL == STM32_SPI6SEL_PCLK4) || defined(__DOXYGEN__)
/**
 * @brief   SPI6 clock.
 */
#define STM32_SPI6CLK               STM32_PCLK4

#elif STM32_SPI6SEL == STM32_SPI6SEL_PLL2_Q_CK
#define STM32_SPI6CLK               STM32_PLL2_Q_CK
#elif STM32_SPI6SEL == STM32_SPI6SEL_PLL3_Q_CK
#define STM32_SPI6CLK               STM32_PLL3_Q_CK
#elif STM32_SPI6SEL == STM32_SPI6SEL_HSI_KER_CK
#define STM32_SPI6CLK               STM32_HSI_CK
#elif STM32_SPI6SEL == STM32_SPI6SEL_CSI_KER_CK
#define STM32_SPI6CLK               STM32_CSI_CK
#elif STM32_SPI6SEL == STM32_SPI6SEL_HSE_CK
#define STM32_SPI6CLK               STM32_HSE_CK
#else
#error "invalid source selected for STM32_SPI6SEL clock"
#endif

#if (STM32_I2C1235SEL == STM32_I2C1235SEL_PCLK1) || defined(__DOXYGEN__)
/**
 * @brief   I2C1 clock.
 */
#define STM32_I2C1CLK               STM32_PCLK1

/**
 * @brief   I2C2 clock.
 */
#define STM32_I2C2CLK               STM32_PCLK1

/**
 * @brief   I2C3 clock.
 */
#define STM32_I2C3CLK               STM32_PCLK1

/**
 * @brief   I2C5 clock.
 */
#define STM32_I2C5CLK               STM32_PCLK1

#elif STM32_I2C123SEL == STM32_I2C123SEL_PLL3_R_CK
#define STM32_I2C1CLK               STM32_PLL3_R_CK
#define STM32_I2C2CLK               STM32_PLL3_R_CK
#define STM32_I2C2CLK               STM32_PLL3_R_CK
#define STM32_I2C5CLK               STM32_PLL3_R_CK

#elif STM32_I2C123SEL == STM32_I2C123SEL_HSI_KER_CK
#define STM32_I2C1CLK               STM32_HSI_CK
#define STM32_I2C2CLK               STM32_HSI_CK
#define STM32_I2C2CLK               STM32_HSI_CK
#define STM32_I2C5CLK               STM32_HSI_CK

#elif STM32_I2C123SEL == STM32_I2C123SEL_CSI_KER_CK
#define STM32_I2C1CLK               STM32_CSI_CK
#define STM32_I2C2CLK               STM32_CSI_CK
#define STM32_I2C2CLK               STM32_CSI_CK
#define STM32_I2C5CLK               STM32_CSI_CK

#else
#error "invalid source selected for STM32_I2C123SEL clock"
#endif

#if (STM32_I2C4SEL == STM32_I2C4SEL_PCLK4) || defined(__DOXYGEN__)
/**
 * @brief   I2C1 clock.
 */
#define STM32_I2C4CLK               STM32_PCLK4

#elif STM32_I2C4SEL == STM32_I2C4SEL_PLL3_R_CK
#define STM32_I2C4CLK               STM32_PLL3_R_CK
#elif STM32_I2C4SEL == STM32_I2C4SEL_HSI_KER_CK
#define STM32_I2C4CLK               STM32_HSI_CK
#elif STM32_I2C4SEL == STM32_I2C4SEL_CSI_KER_CK
#define STM32_I2C4CLK               STM32_CSI_CK
#else
#error "invalid source selected for STM32_I2C4SEL clock"
#endif

#if (STM32_SAI1SEL == STM32_SAI1SEL_PLL1_Q_CK) || defined(__DOXYGEN__)
/**
 * @brief   SAI1 clock.
 */
#define STM32_SAI1CLK               STM32_PLL1_Q_CK

#elif STM32_SAI1SEL == STM32_SAI1SEL_PLL2_P_CK
#define STM32_SAI1CLK               STM32_PLL2_P_CK
#elif STM32_SAI1SEL == STM32_SAI1SEL_PLL3_P_CK
#define STM32_SAI1CLK               STM32_PLL3_P_CK
#elif STM32_SAI1SEL == STM32_SAI1SEL_I2S_CKIN
#define STM32_SAI1CLK               0 /* Unknown, would require a board value */
#elif STM32_SAI1SEL == STM32_SAI1SEL_PER_CK
#define STM32_SAI1CLK               STM32_PER_CK
#else
#error "invalid source selected for STM32_SAI1SEL clock"
#endif

#if (STM32_SAI4ASEL == STM32_SAI4ASEL_PLL1_Q_CK) || defined(__DOXYGEN__)
/**
 * @brief   SAI4A clock.
 */
#define STM32_SAI4ACLK              STM32_PLL1_Q_CK

#elif STM32_SAI4ASEL == STM32_SAI4ASEL_PLL2_P_CK
#define STM32_SAI4ACLK              STM32_PLL2_P_CK
#elif STM32_SAI4ASEL == STM32_SAI4ASEL_PLL3_P_CK
#define STM32_SAI4ACLK              STM32_PLL3_P_CK
#elif STM32_SAI4ASEL == STM32_SAI4ASEL_I2S_CKIN
#define STM32_SAI4ACLK              0 /* Unknown, would require a board value */
#elif STM32_SAI4ASEL == STM32_SAI4ASEL_PER_CK
#define STM32_SAI4ACLK              STM32_PER_CK
#else
#error "invalid source selected for STM32_SAI4ASEL clock"
#endif

#if (STM32_SAI4BSEL == STM32_SAI4BSEL_PLL1_Q_CK) || defined(__DOXYGEN__)
/**
 * @brief   SAI4B clock.
 */
#define STM32_SAI4BCLK              STM32_PLL1_Q_CK

#elif STM32_SAI4BSEL == STM32_SAI4BSEL_PLL2_P_CK
#define STM32_SAI4BCLK              STM32_PLL2_P_CK
#elif STM32_SAI4BSEL == STM32_SAI4BSEL_PLL3_P_CK
#define STM32_SAI4BCLK              STM32_PLL3_P_CK
#elif STM32_SAI4BSEL == STM32_SAI4BSEL_I2S_CKIN
#define STM32_SAI4BCLK              0 /* Unknown, would require a board value */
#elif STM32_SAI4BSEL == STM32_SAI4BSEL_PER_CK
#define STM32_SAI4BCLK              STM32_PER_CK
#else
#error "invalid source selected for STM32_SAI4BSEL clock"
#endif

#if (STM32_USBSEL == STM32_USBSEL_DISABLE) || defined(__DOXYGEN__)
/**
 * @brief   USB clock.
 */
#define STM32_USBCLK               0

#elif STM32_USBSEL == STM32_USBSEL_PLL1_Q_CK
#define STM32_USBCLK               STM32_PLL1_Q_CK
#elif STM32_USBSEL == STM32_USBSEL_PLL3_Q_CK
#define STM32_USBCLK               STM32_PLL3_Q_CK
#elif STM32_USBSEL == STM32_USBSEL_HSI48_CK
#define STM32_USBCLK               STM32_HSI48_CK
#else
#error "invalid source selected for STM32_USBSEL clock"
#endif

#if (STM32_SDMMCSEL == STM32_SDMMCSEL_PLL1_Q_CK) || defined(__DOXYGEN__)
/**
 * @brief   SDMMC1 frequency.
 */
#define STM32_SDMMC1CLK             STM32_PLL1_Q_CK

/**
 * @brief   SDMMC2 frequency.
 */
#define STM32_SDMMC2CLK             STM32_PLL1_Q_CK

#elif STM32_SDMMCSEL == STM32_SDMMCSEL_PLL2_R_CK
#define STM32_SDMMC1CLK             STM32_PLL2_R_CK
#define STM32_SDMMC2CLK             STM32_PLL2_R_CK
#else
#error "invalid source selected for STM32_SDMMCxSEL clock"
#endif

#if (STM32_OCTOSPISEL == STM32_OCTOSPISEL_HCLK) || defined(__DOXYGEN__)
/**
 * @brief   OCTOSPI frequency.
 */
#define STM32_OCTOSPICLK            STM32_HCLK

#elif STM32_OCTOSPISEL == STM32_OCTOSPISEL_PLL1_Q_CK
#define STM32_OCTOSPICLK            STM32_PLL1_Q_CK
#elif STM32_OCTOSPISEL == STM32_OCTOSPISEL_PLL2_R_CK
#define STM32_OCTOSPICLK            STM32_PLL2_R_CK
#elif STM32_OCTOSPISEL == STM32_OCTOSPISEL_PER_CK
#define STM32_OCTOSPICLK            STM32_PER_CK
#else
#error "invalid source selected for STM32_OCTOSPISEL clock"
#endif

#if (STM32_FMCSEL == STM32_FMCSEL_HCLK) || defined(__DOXYGEN__)
/**
 * @brief   FMC frequency.
 */
#define STM32_FMCCLK                STM32_HCLK

#elif STM32_FMCSEL == STM32_FMCSEL_PLL1_Q_CK
#define STM32_FMCCLK                STM32_PLL1_Q_CK
#elif STM32_FMCSEL == STM32_FMCSEL_PLL2_R_CK
#define STM32_FMCCLK                STM32_PLL2_R_CK
#elif STM32_FMCSEL == STM32_FMCSEL_PER_CK
#define STM32_FMCCLK                STM32_PER_CK
#else
#error "invalid source selected for STM32_FMCSEL clock"
#endif

#if (STM32_SWPSEL == STM32_SWPSEL_PCLK1) || defined(__DOXYGEN__)
/**
 * @brief   SDMMC frequency.
 */
#define STM32_SWPCLK                STM32_PCLK1

#elif STM32_SWPSEL == STM32_SWPSEL_HSI_KER_CK
#define STM32_SWPCLK                STM32_HSI_CK
#else
#error "invalid source selected for STM32_SWPSEL clock"
#endif

#if (STM32_FDCANSEL == STM32_FDCANSEL_HSE_CK) || defined(__DOXYGEN__)
/**
 * @brief   FDCAN frequency.
 */
#define STM32_FDCANCLK              STM32_HSE_CK

#elif STM32_FDCANSEL == STM32_FDCANSEL_PLL1_Q_CK
#define STM32_FDCANCLK              STM32_PLL1_Q_CK
#elif STM32_FDCANSEL == STM32_FDCANSEL_PLL2_Q_CK
#define STM32_FDCANCLK              STM32_PLL2_Q_CK
#else
#error "invalid source selected for STM32_FDCANSEL clock"
#endif

#if (STM32_DFSDM1SEL == STM32_DFSDM1SEL_PCLK2) || defined(__DOXYGEN__)
/**
 * @brief   SDMMC frequency.
 */
#define STM32_DFSDM1CLK             STM32_PCLK2

#elif STM32_DFSDM1SEL == STM32_DFSDM1SEL_SYS_CK
#define STM32_DFSDM1CLK             STM32_SYS_CK
#else
#error "invalid source selected for STM32_DFSDM1SEL clock"
#endif

#if (STM32_SPDIFSEL == STM32_SPDIFSEL_PLL1_Q_CK) || defined(__DOXYGEN__)
/**
 * @brief   SPDIF frequency.
 */
#define STM32_SPDIFCLK              STM32_PLL1_Q_CK

#elif STM32_SPDIFSEL == STM32_SPDIFSEL_PLL2_R_CK
#define STM32_SPDIFCLK              STM32_PLL2_R_CK
#elif STM32_SPDIFSEL == STM32_SPDIFSEL_PLL3_R_CK
#define STM32_SPDIFCLK              STM32_PLL3_R_CK
#elif STM32_SPDIFSEL == STM32_SPDIFSEL_HSI_KET_CLK
#define STM32_SPDIFCLK              STM32_HSI_CK
#else
#error "invalid source selected for STM32_SPDIFSEL clock"
#endif

#if (STM32_CECSEL == STM32_CECSEL_LSE_CK) || defined(__DOXYGEN__)
/**
 * @brief   CEC frequency.
 */
#define STM32_CECCLK                STM32_LSE_CK

#elif STM32_CECSEL == STM32_CECSEL_LSI_CK
#define STM32_CECCLK                STM32_LSI_CK
#elif STM32_CECSEL == STM32_CECSEL_CSI_KER_CK
#define STM32_CECCLK                STM32_CSI_CK
#elif STM32_CECSEL == STM32_CECSEL_DISABLE
#define STM32_CECCLK                0
#else
#error "invalid source selected for STM32_CECSEL clock"
#endif

#if (STM32_RNGSEL == STM32_RNGSEL_HSI48_CK) || defined(__DOXYGEN__)
/**
 * @brief   RNG frequency.
 */
#define STM32_RNGCLK                STM32_HSI48_CK

#elif STM32_RNGSEL == STM32_RNGSEL_PLL1_Q_CK
#define STM32_RNGCLK                STM32_PLL1_Q_CK
#elif STM32_RNGSEL == STM32_RNGSEL_LSE_CK
#define STM32_RNGCLK                STM32_LSE_CK
#elif STM32_RNGSEL == STM32_RNGSEL_LSI_CK
#define STM32_RNGCLK                STM32_LSI_CK
#else
#error "invalid source selected for STM32_RNGSEL clock"
#endif

#if (STM32_ADCSEL == STM32_ADCSEL_PLL2_P_CK) || defined(__DOXYGEN__)
/**
 * @brief   ADC frequency.
 */
#define STM32_ADCCLK                STM32_PLL2_P_CK

#elif STM32_ADCSEL == STM32_ADCSEL_PLL3_R_CK
#define STM32_ADCCLK                STM32_PLL3_R_CK
#elif STM32_ADCSEL == STM32_ADCSEL_PER_CK
#define STM32_ADCCLK                STM32_PER_CK
#elif STM32_ADCSEL == STM32_ADCSEL_DISABLE
#define STM32_ADCCLK                0
#else
#error "invalid source selected for STM32_ADCSEL clock"
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

#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_TYPE2_H */

/** @} */
