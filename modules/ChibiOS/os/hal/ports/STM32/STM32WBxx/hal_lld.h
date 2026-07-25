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
/*
    Concepts and parts of this file have been contributed by Ilya Kharin.
*/

/**
 * @file    STM32WBxx/hal_lld.h
 * @brief   STM32WBxx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSE32CLK.
 *          .
 *          One of the following macros must also be defined:
 *          - STM32WB55xx.
 *          - STM32WB50xx.
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
#if defined(STM32WB55xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32WBxx Ultra Low Power"

#elif defined(STM32WB50xx)
#define PLATFORM_NAME           "STM32WBxx Ultra Low Power Value Line"

#else
#error "STM32WBxx device not specified"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(STM32WBXX) || defined(__DOXYGEN__)
#define STM32WBXX
#endif
/** @} */

/**
 * @name    PWR_CR1 register bits definitions
 * @{
 */
#define STM32_VOS_MASK          (3 << 9)    /**< Core voltage mask.         */
#define STM32_VOS_RANGE1        (1 << 9)    /**< Core voltage 1.2 Volts.    */
#define STM32_VOS_RANGE2        (2 << 9)    /**< Core voltage 1.0 Volts.    */
/** @} */

/**
 * @name    PWR_CR2 register bits definitions
 * @{
 */
#define STM32_PLS_MASK          (7 << 1)    /**< PLS bits mask.             */
#define STM32_PLS_LEV0          (0 << 1)    /**< PVD level 0.               */
#define STM32_PLS_LEV1          (1 << 1)    /**< PVD level 1.               */
#define STM32_PLS_LEV2          (2 << 1)    /**< PVD level 2.               */
#define STM32_PLS_LEV3          (3 << 1)    /**< PVD level 3.               */
#define STM32_PLS_LEV4          (4 << 1)    /**< PVD level 4.               */
#define STM32_PLS_LEV5          (5 << 1)    /**< PVD level 5.               */
#define STM32_PLS_LEV6          (6 << 1)    /**< PVD level 6.               */
#define STM32_PLS_EXT           (7 << 1)    /**< PVD level 7.               */
/** @} */

/**
 * @brief   HSE SYSCLK and PLL M divider prescaler.
 */
#define STM32_HSE32PRE_MASK     (1 << 20)   /**< HSEPRE mask.               */
#define STM32_HSE32PRE_DIV1     (0 << 20)   /**< HSE divided by 1.          */
#define STM32_HSE32PRE_DIV2     (1 << 20)   /**< HSE divided by 2.          */
/** @} */

/**
 * @name    RCC_CFGR register bits definitions
 * @{
 */
#define STM32_SW_MASK           (3 << 0)    /**< SW field mask.             */
#define STM32_SW_MSI            (0 << 0)    /**< SYSCLK source is MSI.      */
#define STM32_SW_HSI16          (1 << 0)    /**< SYSCLK source is HSI.      */
#define STM32_SW_HSE            (2 << 0)    /**< SYSCLK source is HSE.      */
#define STM32_SW_PLL            (3 << 0)    /**< SYSCLK source is PLL.      */

/* STM32WB CMSIS headers don't have these macros */
#if !defined(RCC_CFGR_SW_MSI)
#define RCC_CFGR_SW_MSI         (0x0U << RCC_CFGR_SW_Pos)
#endif
#if !defined(RCC_CFGR_SW_HSI)
#define RCC_CFGR_SW_HSI         (0x1U << RCC_CFGR_SW_Pos)
#endif
#if !defined(RCC_CFGR_SW_HSE)
#define RCC_CFGR_SW_HSE         (0x2U << RCC_CFGR_SW_Pos)
#endif
#if !defined(RCC_CFGR_SW_PLL)
#define RCC_CFGR_SW_PLL         (0x3U << RCC_CFGR_SW_Pos)
#endif

#if !defined(RCC_CFGR_SWS_MSI)
#define RCC_CFGR_SWS_MSI        (0x0U << RCC_CFGR_SWS_Pos)
#endif
#if !defined(RCC_CFGR_SWS_HSI)
#define RCC_CFGR_SWS_HSI        (0x1U << RCC_CFGR_SWS_Pos)
#endif
#if !defined(RCC_CFGR_SWS_HSE)
#define RCC_CFGR_SWS_HSE        (0x2U << RCC_CFGR_SWS_Pos)
#endif
#if !defined(RCC_CFGR_SWS_PLL)
#define RCC_CFGR_SWS_PLL        (0x3U << RCC_CFGR_SWS_Pos)
#endif

#define STM32_STOPWUCK_MASK     (1 << 15)   /**< STOPWUCK field mask.       */
#define STM32_STOPWUCK_MSI      (0 << 15)   /**< Wakeup clock is MSI.       */
#define STM32_STOPWUCK_HSI16    (1 << 15)   /**< Wakeup clock is HSI16.     */

#define STM32_MCOSEL_MASK       (15 << 24)  /**< MCOSEL field mask.         */
#define STM32_MCOSEL_NOCLOCK    (0 << 24)   /**< No clock on MCO pin.       */
#define STM32_MCOSEL_SYSCLK     (1 << 24)   /**< SYSCLK on MCO pin.         */
#define STM32_MCOSEL_MSI        (2 << 24)   /**< MSI clock on MCO pin.      */
#define STM32_MCOSEL_HSI16      (3 << 24)   /**< HSI16 clock on MCO pin.    */
#define STM32_MCOSEL_HSE        (4 << 24)   /**< HSE clock on MCO pin.
                                                 (after stabilization)      */
#define STM32_MCOSEL_PLLRCLK    (5 << 24)   /**< PLLR clock on MCO pin.     */
#define STM32_MCOSEL_LSI1       (6 << 24)   /**< LSI clock on MCO pin.      */
#define STM32_MCOSEL_LSI2       (7 << 24)   /**< LSI clock on MCO pin.      */
#define STM32_MCOSEL_LSE        (8 << 24)   /**< LSE clock on MCO pin.      */
#define STM32_MCOSEL_HSI48      (9 << 24)   /**< HSI48 clock on MCO pin.    */
#define STM32_MCOSEL_HSE2       (4 << 24)   /**< HSE clock on MCO pin.
                                                 (before stabilization)     */

#define STM32_MCOPRE_MASK       (7 << 28)   /**< MCOPRE field mask.         */
#define STM32_MCOPRE_DIV1       (0 << 28)   /**< MCO divided by 1.          */
#define STM32_MCOPRE_DIV2       (1 << 28)   /**< MCO divided by 2.          */
#define STM32_MCOPRE_DIV4       (2 << 28)   /**< MCO divided by 4.          */
#define STM32_MCOPRE_DIV8       (3 << 28)   /**< MCO divided by 8.          */
#define STM32_MCOPRE_DIV16      (4 << 28)   /**< MCO divided by 16.         */
/** @} */

/**
 * @name    RCC_EXTCFGR register bits definitions
 * @{
 */
/**
 * @brief   HCLK5 clock source (RFC and APB3).
 */
#define STM32_RFCSSSEL_MASK     (1 << 20)  /**< RFCSS field mask.           */
#define STM32_RFCSSSEL_HSI16    (0 << 20)  /**< RFCSS source is HSI16.      */
#define STM32_RFCSSSEL_HSE      (1 << 20)  /**< RFCSS source is HSE/2.      */

/**
 * @brief   HCLK4 shared prescaler (AHB3, Flash memory and SRAM2).
 */
#define STM32_SHDHPRE_MASK      (15 << 0)  /**< SHDHPRE field mask.         */
#define STM32_SHDHPRE_DIV1      (0 << 0)   /**< SYSCLK divided by 1.        */
#define STM32_SHDHPRE_DIV2      (8 << 0)   /**< SYSCLK divided by 2.        */
#define STM32_SHDHPRE_DIV3      (1 << 0)   /**< SYSCLK divided by 3.        */
#define STM32_SHDHPRE_DIV4      (9 << 0)   /**< SYSCLK divided by 4.        */
#define STM32_SHDHPRE_DIV5      (2 << 0)   /**< SYSCLK divided by 5.        */
#define STM32_SHDHPRE_DIV6      (5 << 0)   /**< SYSCLK divided by 6.        */
#define STM32_SHDHPRE_DIV8      (10 << 0)  /**< SYSCLK divided by 8.        */
#define STM32_SHDHPRE_DIV10     (6 << 0)   /**< SYSCLK divided by 10.       */
#define STM32_SHDHPRE_DIV16     (11 << 0)  /**< SYSCLK divided by 16.       */
#define STM32_SHDHPRE_DIV32     (7 << 0)   /**< SYSCLK divided by 32.       */
#define STM32_SHDHPRE_DIV64     (12 << 0)  /**< SYSCLK divided by 64.       */
#define STM32_SHDHPRE_DIV128    (13 << 0)  /**< SYSCLK divided by 128.      */
#define STM32_SHDHPRE_DIV256    (14 << 0)  /**< SYSCLK divided by 256.      */
#define STM32_SHDHPRE_DIV512    (15 << 0)  /**< SYSCLK divided by 512.      */

/**
 * @brief   HCLK2 prescaler (CPU2).
 */
#define STM32_C2HPRE_MASK       (15 << 4)  /**< C2HPRE field mask.          */
#define STM32_C2HPRE_DIV1       (0 << 4)   /**< SYSCLK divided by 1.        */
#define STM32_C2HPRE_DIV2       (8 << 4)   /**< SYSCLK divided by 2.        */
#define STM32_C2HPRE_DIV3       (1 << 4)   /**< SYSCLK divided by 3.        */
#define STM32_C2HPRE_DIV4       (9 << 4)   /**< SYSCLK divided by 4.        */
#define STM32_C2HPRE_DIV5       (2 << 4)   /**< SYSCLK divided by 5.        */
#define STM32_C2HPRE_DIV6       (5 << 4)   /**< SYSCLK divided by 6.        */
#define STM32_C2HPRE_DIV8       (10 << 4)  /**< SYSCLK divided by 8.        */
#define STM32_C2HPRE_DIV10      (6 << 4)   /**< SYSCLK divided by 10.       */
#define STM32_C2HPRE_DIV16      (11 << 4)  /**< SYSCLK divided by 16.       */
#define STM32_C2HPRE_DIV32      (7 << 4)   /**< SYSCLK divided by 32.       */
#define STM32_C2HPRE_DIV64      (12 << 4)  /**< SYSCLK divided by 64.       */
#define STM32_C2HPRE_DIV128     (13 << 4)  /**< SYSCLK divided by 128.      */
#define STM32_C2HPRE_DIV256     (14 << 4)  /**< SYSCLK divided by 256.      */
#define STM32_C2HPRE_DIV512     (15 << 4)  /**< SYSCLK divided by 512.      */

/**
 * @brief   HCLK5 and APB3 clock source.
 */
#define STM32_RFCSS_MASK        (1 << 20)  /**< RFCSS field mask.           */
#define STM32_RFCSS_HSI16       (0 << 20)  /**< HSI16 on HCLK5 and APB3.    */
#define STM32_RFCSS_HSEDIV2     (1 << 20)  /**< HSE/2 on HCLK5 and APB3.    */
/** @} */

/**
 * @name    RCC_PLLCFGR register bits definitions
 * @{
 */
#define STM32_PLLSRC_MASK       (3 << 0)    /**< PLL clock source mask.     */
#define STM32_PLLSRC_NOCLOCK    (0 << 0)    /**< PLL clock source disabled. */
#define STM32_PLLSRC_MSI        (1 << 0)    /**< PLL clock source is MSI.   */
#define STM32_PLLSRC_HSI16      (2 << 0)    /**< PLL clock source is HSI16. */
#define STM32_PLLSRC_HSE        (3 << 0)    /**< PLL clock source is HSE.   */
/** @} */

/**
 * @name    RCC_CCIPR register bits definitions
 * @{
 */
#define STM32_USART1SEL_MASK    (3 << 0)    /**< USART1SEL mask.            */
#define STM32_USART1SEL_PCLK2   (0 << 0)    /**< USART1 source is PCLK2.    */
#define STM32_USART1SEL_SYSCLK  (1 << 0)    /**< USART1 source is SYSCLK.   */
#define STM32_USART1SEL_HSI16   (2 << 0)    /**< USART1 source is HSI16.    */
#define STM32_USART1SEL_LSE     (3 << 0)    /**< USART1 source is LSE.      */

#define STM32_LPUART1SEL_MASK   (3 << 10)   /**< LPUART1 mask.              */
#define STM32_LPUART1SEL_PCLK1  (0 << 10)   /**< LPUART1 source is PCLK1.   */
#define STM32_LPUART1SEL_SYSCLK (1 << 10)   /**< LPUART1 source is SYSCLK.  */
#define STM32_LPUART1SEL_HSI16  (2 << 10)   /**< LPUART1 source is HSI16.   */
#define STM32_LPUART1SEL_LSE    (3 << 10)   /**< LPUART1 source is LSE.     */

#define STM32_I2C1SEL_MASK      (3 << 12)   /**< I2C1SEL mask.              */
#define STM32_I2C1SEL_PCLK1     (0 << 12)   /**< I2C1 source is PCLK1.      */
#define STM32_I2C1SEL_SYSCLK    (1 << 12)   /**< I2C1 source is SYSCLK.     */
#define STM32_I2C1SEL_HSI16     (2 << 12)   /**< I2C1 source is HSI16.      */

#define STM32_I2C3SEL_MASK      (3 << 16)   /**< I2C3SEL mask.              */
#define STM32_I2C3SEL_PCLK1     (0 << 16)   /**< I2C3 source is PCLK1.      */
#define STM32_I2C3SEL_SYSCLK    (1 << 16)   /**< I2C3 source is SYSCLK.     */
#define STM32_I2C3SEL_HSI16     (2 << 16)   /**< I2C3 source is HSI16.      */

#define STM32_LPTIM1SEL_MASK    (3 << 18)   /**< LPTIM1SEL mask.            */
#define STM32_LPTIM1SEL_PCLK1   (0 << 18)   /**< LPTIM1 source is PCLK1.    */
#define STM32_LPTIM1SEL_LSI     (1 << 18)   /**< LPTIM1 source is LSI.      */
#define STM32_LPTIM1SEL_HSI16   (2 << 18)   /**< LPTIM1 source is HSI16.    */
#define STM32_LPTIM1SEL_LSE     (3 << 18)   /**< LPTIM1 source is LSE.      */

#define STM32_LPTIM2SEL_MASK    (3 << 20)   /**< LPTIM2SEL mask.            */
#define STM32_LPTIM2SEL_PCLK1   (0 << 20)   /**< LPTIM2 source is PCLK1.    */
#define STM32_LPTIM2SEL_LSI     (1 << 20)   /**< LPTIM2 source is LSI.      */
#define STM32_LPTIM2SEL_HSI16   (2 << 20)   /**< LPTIM2 source is HSI16.    */
#define STM32_LPTIM2SEL_LSE     (3 << 20)   /**< LPTIM2 source is LSE.      */

#define STM32_SAI1SEL_MASK        (3 << 22)   /**< SAI1SEL mask.            */
#define STM32_SAI1SEL_PLLSAI1PCLK (0 << 22)   /**< SAI1 source is PLLSAI1PCLK.*/
#define STM32_SAI1SEL_PLLPCLK     (2 << 22)   /**< SAI1 source is PLLPCLK.  */
#define STM32_SAI1SEL_EXTCLK      (3 << 22)   /**< SAI1 source is external. */
#define STM32_SAI1SEL_OFF         0xFFFFFFFFU /**< SAI1 clock is not required.*/

#define STM32_CLK48SEL_MASK        (3 << 26) /**< CLK48SEL mask.            */
#define STM32_CLK48SEL_HSI48       (0 << 26) /**< CLK48 source is HSI48.    */
#define STM32_CLK48SEL_PLLSAI1QCLK (1 << 26) /**< CLK48 source is SAI1-Q.   */
#define STM32_CLK48SEL_PLLQCLK     (2 << 26) /**< CLK48 source is PLLQCLK.  */
#define STM32_CLK48SEL_MSI         (3 << 26) /**< CLK48 source is MSI.      */

#define STM32_ADCSEL_MASK        (3 << 28)  /**< ADCSEL mask.               */
#define STM32_ADCSEL_NOCLK       (0 << 28)  /**< ADC clock disabled.        */
#define STM32_ADCSEL_PLLSAI1RCLK (1 << 28)  /**< ADC source is PLLSAI1RCLK. */
#define STM32_ADCSEL_PLLPCLK     (2 << 28)  /**< ADC source is PLLPCLK.     */
#define STM32_ADCSEL_SYSCLK      (3 << 28)  /**< ADC source is SYSCLK.      */

#define STM32_RNGSEL_MASK       (3 << 30)   /**< RNGSEL mask.               */
#define STM32_RNGSEL_48CLK      (0 << 30)   /**< RNG source is CLK48SEL.    */
#define STM32_RNGSEL_LSI        (1 << 30)   /**< RNG source is LSI.         */
#define STM32_RNGSEL_LSE        (3 << 30)   /**< RNG source is LSE.         */
/** @} */

/**
 * @name    RCC_BDCR register bits definitions
 * @{
 */
#define STM32_RTCSEL_MASK       (3 << 8)    /**< RTC source mask.           */
#define STM32_RTCSEL_NOCLOCK    (0 << 8)    /**< No RTC source.             */
#define STM32_RTCSEL_LSE        (1 << 8)    /**< RTC source is LSE.         */
#define STM32_RTCSEL_LSI        (2 << 8)    /**< RTC source is LSI.         */
#define STM32_RTCSEL_HSEDIV     (3 << 8)    /**< RTC source is HSE divided. */

#define STM32_LSCOSEL_MASK      (3 << 24)   /**< LSCO pin clock source.     */
#define STM32_LSCOSEL_NOCLOCK   (0 << 24)   /**< No clock on LSCO pin.      */
#define STM32_LSCOSEL_LSI       (1 << 24)   /**< LSI on LSCO pin.           */
#define STM32_LSCOSEL_LSE       (3 << 24)   /**< LSE on LSCO pin.           */
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
 * @brief   Core voltage selection.
 * @note    This setting affects all the performance and clock related
 *          settings, the maximum performance is only obtainable selecting
 *          the maximum voltage.
 */
#if !defined(STM32_VOS) || defined(__DOXYGEN__)
#define STM32_VOS                           STM32_VOS_RANGE1
#endif

/**
 * @brief   PWR CR2 register initialization value.
 */
#if !defined(STM32_PWR_CR2) || defined(__DOXYGEN__)
#define STM32_PWR_CR2                       (PWR_CR2_PLS_0)
#endif

/**
 * @brief   PWR CR3 register initialization value.
 */
#if !defined(STM32_PWR_CR3) || defined(__DOXYGEN__)
#define STM32_PWR_CR3                       (PWR_CR3_EIWUL)
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
 * @brief   PWR PUCRG register initialization value.
 */
#if !defined(STM32_PWR_PUCRG) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRG                     (0U)
#endif

/**
 * @brief   PWR PDCRG register initialization value.
 */
#if !defined(STM32_PWR_PDCRG) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRG                     (0U)
#endif

/**
 * @brief   PWR PUCRH register initialization value.
 */
#if !defined(STM32_PWR_PUCRH) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRH                     (0U)
#endif

/**
 * @brief   PWR PDCRH register initialization value.
 */
#if !defined(STM32_PWR_PDCRH) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRH                     (0U)
#endif

/**
 * @brief   Enables or disables the programmable voltage detector.
 */
#if !defined(STM32_PVD_ENABLE) || defined(__DOXYGEN__)
#define STM32_PVD_ENABLE                    FALSE
#endif

/**
 * @brief   Sets voltage level for programmable voltage detector.
 */
#if !defined(STM32_PLS) || defined(__DOXYGEN__)
#define STM32_PLS                           STM32_PLS_LEV0
#endif

/**
 * @brief   Enables or disables the HSI16 clock source.
 */
#if !defined(STM32_HSI16_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI16_ENABLED                 TRUE
#endif

/**
 * @brief   Enables or disables the HSI48 clock source.
 */
#if !defined(STM32_HSI48_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI48_ENABLED                 FALSE
#endif

/**
 * @brief   Enables or disables the LSI clock source.
 */
#if !defined(STM32_LSI1_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSI1_ENABLED                  TRUE
#endif

/**
 * @brief   Enables or disables the LSI2 clock source.
 */
#if !defined(STM32_LSI2_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSI2_ENABLED                  FALSE
#endif

/**
 * @brief   Enables or disables the HSE clock source.
 */
#if !defined(STM32_HSE32_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSE32_ENABLED                 FALSE
#endif

/**
 * @brief   Enables or disables the LSE clock source.
 */
#if !defined(STM32_LSE_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSE_ENABLED                   FALSE
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
#define STM32_MSIRANGE                      STM32_MSIRANGE_4M
#endif

/**
 * @brief   HSE32 prescaler value.
 */
#if !defined(STM32_HSE32PRE) || defined(__DOXYGEN__)
#define STM32_HSE32PRE                      STM32_HSE32PRE_DIV1
#endif

/**
 * @brief   Main clock source selection.
 * @note    If the selected clock source is not the PLL then the PLL is not
 *          initialized and started.
 * @note    The default value is calculated for a 32MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                            STM32_SW_PLL
#endif

/**
 * @brief   Clock source for the PLL.
 * @note    This setting has only effect if the PLL is selected as the
 *          system clock source.
 * @note    The default value is calculated for a 32MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_PLLSRC) || defined(__DOXYGEN__)
#define STM32_PLLSRC                        STM32_PLLSRC_MSI
#endif

/**
 * @brief   PLLM divider value.
 * @note    The allowed values are 1..8.
 * @note    The default value is calculated for a 32MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_PLLM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLM_VALUE                    1
#endif

/**
 * @brief   PLLN multiplier value.
 * @note    The allowed values are 8..86.
 * @note    The default value is calculated for a 32MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_PLLN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLN_VALUE                    32
#endif

/**
 * @brief   PLLP divider value.
 * @note    The allowed values are 2..31.
 */
#if !defined(STM32_PLLP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLP_VALUE                    2
#endif

/**
 * @brief   PLLQ divider value.
 * @note    The allowed values are 2, 4, 6, 8.
 */
#if !defined(STM32_PLLQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLQ_VALUE                    2
#endif

/**
 * @brief   PLLR divider value.
 * @note    The allowed values are 2, 4, 6, 8.
 * @note    The default value is calculated for a 32MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_PLLR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLR_VALUE                    2
#endif

/**
 * @brief   HCLK1 (CPU1, AHB1, AHB2, AHB3 and SRAM1) prescaler value.
 * @note    The default value is calculated for a 32MHz system clock from
 *          the internal 4MHz MSI clock.
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
 * @brief   HCLK2 (CPU2) prescaler value.
 * @note    The default value is calculated for a 32MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_C2HPRE) || defined(__DOXYGEN__)
#define STM32_C2HPRE                        STM32_C2HPRE_DIV2
#endif

/**
 * @brief   HCLK4 (AHB4, Flash memory and SRAM2) prescaler value.
 */
#if !defined(STM32_SHDHPRE) || defined(__DOXYGEN__)
#define STM32_SHDHPRE                       STM32_SHDHPRE_DIV1
#endif

/**
 * @brief   HCLK5 (APB3, AHB5 and Radio system) clock source.
 */
#if !defined(STM32_RFCSSSEL) || defined(__DOXYGEN__)
#define STM32_RFCSSSEL                      STM32_RFCSSSEL_HSI16
#endif

/**
 * @brief   STOPWUCK clock setting.
 */
#if !defined(STM32_STOPWUCK) || defined(__DOXYGEN__)
#define STM32_STOPWUCK                      STM32_STOPWUCK_MSI
#endif

/**
 * @brief   MCO clock source.
 */
#if !defined(STM32_MCOSEL) || defined(__DOXYGEN__)
#define STM32_MCOSEL                        STM32_MCOSEL_NOCLOCK
#endif

/**
 * @brief   MCO divider setting.
 */
#if !defined(STM32_MCOPRE) || defined(__DOXYGEN__)
#define STM32_MCOPRE                        STM32_MCOPRE_DIV1
#endif

/**
 * @brief   LSCO clock source.
 */
#if !defined(STM32_LSCOSEL) || defined(__DOXYGEN__)
#define STM32_LSCOSEL                       STM32_LSCOSEL_NOCLOCK
#endif

/**
 * @brief   PLLSAI1N multiplier value.
 * @note    The allowed values are 4..86.
 */
#if !defined(STM32_PLLSAI1N_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAI1N_VALUE                24
#endif

/**
 * @brief   PLLSAI1P divider value.
 * @note    The allowed values are 2..32.
 */
#if !defined(STM32_PLLSAI1P_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAI1P_VALUE                2
#endif

/**
 * @brief   PLLSAI1Q divider value.
 * @note    The allowed values are 2..8.
 */
#if !defined(STM32_PLLSAI1Q_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAI1Q_VALUE                2
#endif

/**
 * @brief   PLLSAI1R divider value.
 * @note    The allowed values are 2..8.
 */
#if !defined(STM32_PLLSAI1R_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAI1R_VALUE                2
#endif

/**
 * @brief   USART1 clock source.
 */
#if !defined(STM32_USART1SEL) || defined(__DOXYGEN__)
#define STM32_USART1SEL                     STM32_USART1SEL_PCLK2
#endif

/**
 * @brief   LPUART1 clock source.
 */
#if !defined(STM32_LPUART1SEL) || defined(__DOXYGEN__)
#define STM32_LPUART1SEL                    STM32_LPUART1SEL_PCLK1
#endif

/**
 * @brief   I2C1 clock source.
 */
#if !defined(STM32_I2C1SEL) || defined(__DOXYGEN__)
#define STM32_I2C1SEL                       STM32_I2C1SEL_PCLK1
#endif

/**
 * @brief   I2C3 clock source.
 */
#if !defined(STM32_I2C3SEL) || defined(__DOXYGEN__)
#define STM32_I2C3SEL                       STM32_I2C3SEL_PCLK1
#endif

/**
 * @brief   LPTIM1 clock source.
 */
#if !defined(STM32_LPTIM1SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM1SEL                     STM32_LPTIM1SEL_PCLK1
#endif

/**
 * @brief   LPTIM2 clock source.
 */
#if !defined(STM32_LPTIM2SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM2SEL                     STM32_LPTIM2SEL_PCLK1
#endif

/**
 * @brief   SAI1SEL value (SAI1 clock source).
 */
#if !defined(STM32_SAI1SEL) || defined(__DOXYGEN__)
#define STM32_SAI1SEL                       STM32_SAI1SEL_OFF
#endif

/**
 * @brief   CLK48SEL value (48MHz clock source).
 */
#if !defined(STM32_CLK48SEL) || defined(__DOXYGEN__)
#define STM32_CLK48SEL                      STM32_CLK48SEL_PLLQCLK
// #define STM32_CLK48SEL                      STM32_CLK48SEL_PLLSAI1QCLK
#endif

/**
 * @brief   ADCSEL value (ADCs clock source).
 */
#if !defined(STM32_ADCSEL) || defined(__DOXYGEN__)
#define STM32_ADCSEL                        STM32_ADCSEL_SYSCLK
#endif

/**
 * @brief   RNGSEL value (RNGs clock source).
 */
#if !defined(STM32_RNGSEL) || defined(__DOXYGEN__)
#define STM32_RNGSEL                        STM32_RNGSEL_48CLK
#endif

/**
 * @brief   RTC/LCD clock source.
 */
#if !defined(STM32_RTCSEL) || defined(__DOXYGEN__)
#define STM32_RTCSEL                        STM32_RTCSEL_LSI
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(STM32WBxx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32WBxx_MCUCONF not defined"
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

/* Voltage related limits.*/
#if (STM32_VOS == STM32_VOS_RANGE1) || defined(__DOXYGEN__)
/**
 * @name    System Limits
 * @{
 */
/**
 * @brief   Maximum SYSCLK clock frequency at current voltage setting.
 */
#define STM32_SYSCLK_MAX            64000000

/**
 * @brief   Maximum C2HPRE clock frequency at current voltage setting.
 */
#define STM32_C2HPRE_MAX            32000000

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_MAX            32768

/**
 * @brief   Maximum LSE clock frequency using an external source.
 */
#define STM32_LSECLK_BYP_MAX        1000000

/**
 * @brief   Minimum LSE clock frequency.
 */
#define STM32_LSECLK_MIN            32768

/**
 * @brief   Minimum LSE clock frequency using an external source.
 */
#define STM32_LSECLK_BYP_MIN        32768

/**
 * @brief   Maximum PLLs input clock frequency.
 */
#define STM32_PLLIN_MAX             16000000

/**
 * @brief   Minimum PLLs input clock frequency.
 */
#define STM32_PLLIN_MIN             2660000

/**
 * @brief   Maximum VCO clock frequency at current voltage setting.
 */
#define STM32_PLLVCO_MAX            344000000

/**
 * @brief   Minimum VCO clock frequency at current voltage setting.
 */
#define STM32_PLLVCO_MIN            96000000

/**
 * @brief   Maximum PLL-P output clock frequency.
 */
#define STM32_PLLP_MAX              64000000

/**
 * @brief   Minimum PLL-P output clock frequency.
 */
#define STM32_PLLP_MIN              2000000

/**
 * @brief   Maximum PLL-Q output clock frequency.
 */
#define STM32_PLLQ_MAX              64000000

/**
 * @brief   Minimum PLL-Q output clock frequency.
 */
#define STM32_PLLQ_MIN              8000000

/**
 * @brief   Maximum PLL-R output clock frequency.
 */
#define STM32_PLLR_MAX              64000000

/**
 * @brief   Minimum PLL-R output clock frequency.
 */
#define STM32_PLLR_MIN              8000000

/**
 * @brief   Maximum VCO clock frequency at current voltage setting.
 */
#define STM32_PLLSAI1VCO_MAX        344000000

/**
 * @brief   Minimum VCO clock frequency at current voltage setting.
 */
#define STM32_PLLSAI1VCO_MIN        64000000

/**
 * @brief   Maximum APB1 clock frequency.
 */
#define STM32_PCLK1_MAX             64000000

/**
 * @brief   Maximum APB2 clock frequency.
 */
#define STM32_PCLK2_MAX             64000000

/**
 * @brief   Maximum ADC clock frequency.
 */
#define STM32_ADCCLK_MAX            64000000
/** @} */

/**
 * @name    Flash Wait states
 * @{
 */
#define STM32_0WS_THRESHOLD         18000000
#define STM32_1WS_THRESHOLD         36000000
#define STM32_2WS_THRESHOLD         54000000
/** @} */

#elif STM32_VOS == STM32_VOS_RANGE2
#define STM32_SYSCLK_MAX            16000000
#define STM32_C2HPRE_MAX            16000000
#define STM32_LSECLK_MAX            32768
#define STM32_LSECLK_BYP_MAX        1000000
#define STM32_LSECLK_MIN            32768
#define STM32_LSECLK_BYP_MIN        32768
#define STM32_PLLIN_MAX             16000000
#define STM32_PLLIN_MIN             2660000
#define STM32_PLLVCO_MAX            128000000
#define STM32_PLLVCO_MIN            64000000
#define STM32_PLLSAI1VCO_MAX        128000000
#define STM32_PLLSAI1VCO_MIN        64000000
#define STM32_PLLP_MAX              16000000
#define STM32_PLLP_MIN              2000000
#define STM32_PLLQ_MAX              16000000
#define STM32_PLLQ_MIN              8000000
#define STM32_PLLR_MAX              16000000
#define STM32_PLLR_MIN              8000000
#define STM32_PCLK1_MAX             26000000
#define STM32_PCLK2_MAX             26000000
#define STM32_ADCCLK_MAX            26000000

#define STM32_0WS_THRESHOLD         6000000
#define STM32_1WS_THRESHOLD         12000000
#define STM32_2WS_THRESHOLD         16000000

#else
#error "invalid STM32_VOS value specified"
#endif

/**
 * @name    PLL dividers limits
 * @{
 */
#define STM32_PLLM_VALUE_MAX            8
#define STM32_PLLM_VALUE_MIN            1
#define STM32_PLLN_VALUE_MAX            127
#define STM32_PLLN_VALUE_MIN            6
#define STM32_PLLR_VALUE_MAX            8
#define STM32_PLLR_VALUE_MIN            2
#define STM32_PLLQ_VALUE_MAX            8
#define STM32_PLLQ_VALUE_MIN            2
#define STM32_PLLP_VALUE_MAX            32
#define STM32_PLLP_VALUE_MIN            2
/** @} */

/**
 * @name    PLLSAI1 dividers limits
 * @{
 */
#define STM32_PLLSAI1N_VALUE_MAX        86
#define STM32_PLLSAI1N_VALUE_MIN        4
#define STM32_PLLSAI1R_VALUE_MAX        8
#define STM32_PLLSAI1R_VALUE_MIN        2
#define STM32_PLLSAI1Q_VALUE_MAX        8
#define STM32_PLLSAI1Q_VALUE_MIN        2
#define STM32_PLLSAI1P_VALUE_MAX        32
#define STM32_PLLSAI1P_VALUE_MIN        2
/** @} */

/* Clock handlers.*/
#include "stm32_bd.inc"
#include "stm32_lse.inc"
#include "stm32_lsi_v2.inc"
#include "stm32_msi_v2.inc"
#include "stm32_hsi16.inc"
#include "stm32_hsi48.inc"
#include "stm32_hse32.inc"

/*
 * HSI16 related checks.
 */
#if STM32_HSI16_ENABLED
#else /* !STM32_HSI16_ENABLED */

#if STM32_SW == STM32_SW_HSI16
#error "HSI16 not enabled, required by STM32_SW"
#endif

#if (STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSI16)
#error "HSI16 not enabled, required by STM32_SW and STM32_PLLSRC"
#endif

#if (STM32_MCOSEL == STM32_MCOSEL_HSI16) ||                                 \
    ((STM32_MCOSEL == STM32_MCOSEL_PLLRCLK) &&                              \
     (STM32_PLLSRC == STM32_PLLSRC_HSI16))
#error "HSI16 not enabled, required by STM32_MCOSEL"
#endif

#if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI1PCLK) &&                         \
    (STM32_PLLSRC == STM32_PLLSRC_HSI16)
#error "HSI16 not enabled, required by STM32_SAI1SEL"
#endif

#if (STM32_USART1SEL == STM32_USART1SEL_HSI16)
#error "HSI16 not enabled, required by STM32_USART1SEL"
#endif
#if (STM32_LPUART1SEL == STM32_LPUART1SEL_HSI16)
#error "HSI16 not enabled, required by STM32_LPUART1SEL"
#endif

#if (STM32_I2C1SEL == STM32_I2C1SEL_HSI16)
#error "HSI16 not enabled, required by I2C1SEL"
#endif
#if (STM32_I2C3SEL == STM32_I2C3SEL_HSI16)
#error "HSI16 not enabled, required by I2C3SEL"
#endif

#if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_HSI16)
#error "HSI16 not enabled, required by LPTIM1SEL"
#endif
#if (STM32_LPTIM2SEL == STM32_LPTIM2SEL_HSI16)
#error "HSI16 not enabled, required by LPTIM2SEL"
#endif

#if (STM32_RFCSSSEL == STM32_RFCSSSEL_HSI16)
#error "HSI16 not enabled, required by RFCSS"
#endif

#endif /* !STM32_HSI16_ENABLED */

#if STM32_HSI48_ENABLED
#else /* !STM32_HSI48_ENABLED */

#if STM32_MCOSEL == STM32_MCOSEL_HSI48
#error "HSI48 not enabled, required by STM32_MCOSEL"
#endif

#if STM32_CLK48SEL == STM32_CLK48SEL_HSI48
#error "HSI48 not enabled, required by STM32_CLK48SEL"
#endif
#endif /* !STM32_HSI48_ENABLED */

/*
 * HSE related checks.
 */
#if STM32_HSE32_ENABLED

  #if STM32_HSE32CLK == 0
    #error "HSE frequency not defined"
  #else /* STM32_HSE32CLK != 0 */
    #if STM32_HSE32CLK != 32000000
      #error "STM32_HSE32CLK is not 32 Mhz"
    #endif
  #endif /* STM32_HSE32CLK != 0 */

  #else /* !STM32_HSE32_ENABLED */

    #if STM32_SW == STM32_SW_HSE
      #error "HSE not enabled, required by STM32_SW"
    #endif

    #if (STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSE)
      #error "HSE not enabled, required by STM32_SW and STM32_PLLSRC"
    #endif

    #if (STM32_MCOSEL == STM32_MCOSEL_HSE) ||                               \
        ((STM32_MCOSEL == STM32_MCOSEL_PLLRCLK) &&                          \
         (STM32_PLLSRC == STM32_PLLSRC_HSE))
      #error "HSE not enabled, required by STM32_MCOSEL"
    #endif

    #if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI1PCLK) &&                     \
        (STM32_PLLSRC == STM32_PLLSRC_HSE)
      #error "HSE not enabled, required by STM32_SAI1SEL"
    #endif

    #if STM32_RTCSEL == STM32_RTCSEL_HSEDIV
      #error "HSE not enabled, required by STM32_RTCSEL"
    #endif

    #if (STM32_RFCSSSEL == STM32_RFCSSSEL_HSE)
      #error "HSE not enabled, required by RFCSS"
    #endif

#endif /* !STM32_HSE32_ENABLED */

/*
 * LSI related checks.
 */
#if !(STM32_LSI1_ENABLED || STM32_LSI2_ENABLED)

  #if STM32_RTCSEL == STM32_RTCSEL_LSI
    #error "LSI1 or LSI2 not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_LSCOSEL == STM32_LSCOSEL_LSI
    #error "LSI1 or LSI2 not enabled, required by STM32_LSCOSEL"
  #endif

  #if STM32_RNGSEL == STM32_RNGSEL_LSI
    #error "LSI1 or LSI2 not enabled, required by STM32_RNGSEL"
  #endif

#endif /* !(STM32_LSI1_ENABLED || STM32_LSI2_ENABLED) */

#if !STM32_LSI1_ENABLED
  #if STM32_MCOSEL == STM32_MCOSEL_LSI1
    #error "LSI1 not enabled, required by STM32_MCOSEL"
  #endif
#endif

#if !STM32_LSI2_ENABLED
  #if STM32_MCOSEL == STM32_MCOSEL_LSI2
    #error "LSI2 not enabled, required by STM32_MCOSEL"
  #endif
#endif

/*
 * LSE related checks.
 */
#if STM32_LSE_ENABLED

  #if (STM32_LSECLK == 0)
    #error "LSE frequency not defined"
  #endif

  #if (STM32_LSECLK < STM32_LSECLK_MIN) || (STM32_LSECLK > STM32_LSECLK_MAX)
    #error "STM32_LSECLK outside acceptable range (STM32_LSECLK_MIN...STM32_LSECLK_MAX)"
  #endif

#else /* !STM32_LSE_ENABLED */

  #if STM32_RTCSEL == STM32_RTCSEL_LSE
    #error "LSE not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_MCOSEL == STM32_MCOSEL_LSE
    #error "LSE not enabled, required by STM32_MCOSEL"
  #endif

  #if STM32_LSCOSEL == STM32_LSCOSEL_LSE
    #error "LSE not enabled, required by STM32_LSCOSEL"
  #endif

  #if STM32_RNGSEL == STM32_RNGSEL_LSE
    #error "LSE not enabled, required by STM32_RNGSEL"
  #endif

  #if STM32_MSIPLL_ENABLED == TRUE
    #error "LSE not enabled, required by STM32_MSIPLL_ENABLED"
  #endif

#endif /* !STM32_LSE_ENABLED */

/*
 * MSI related checks.
 */
#if (STM32_MSIRANGE == STM32_MSIRANGE_48M) && !STM32_MSIPLL_ENABLED
#warning "STM32_MSIRANGE_48M should be used with STM32_MSIPLL_ENABLED"
#endif

/**
 * @brief   PLLs input clock frequency.
 */
#if (STM32_PLLSRC == STM32_PLLSRC_HSE) || defined(__DOXYGEN__)
#define STM32_PLLCLKIN              (STM32_HSECLK / STM32_PLLM_VALUE)

#elif STM32_PLLSRC == STM32_PLLSRC_MSI
#define STM32_PLLCLKIN              (STM32_MSICLK / STM32_PLLM_VALUE)

#elif STM32_PLLSRC == STM32_PLLSRC_HSI16
#define STM32_PLLCLKIN              (STM32_HSI16CLK / STM32_PLLM_VALUE)

#elif STM32_PLLSRC == STM32_PLLSRC_NOCLOCK
#define STM32_PLLCLKIN              0

#else
#error "invalid STM32_PLLSRC value specified"
#endif

/*
 * PLLs input frequency range check.
 */
#if (STM32_PLLCLKIN != 0) &&                                                \
    ((STM32_PLLCLKIN < STM32_PLLIN_MIN) || (STM32_PLLCLKIN > STM32_PLLIN_MAX))
#error "STM32_PLLCLKIN outside acceptable range (STM32_PLLIN_MIN...STM32_PLLIN_MAX)"
#endif

/*
 * PLLSAI1 enable check.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI1PCLK) ||                         \
    (STM32_CLK48SEL == STM32_CLK48SEL_PLLSAI1QCLK) ||                       \
    (STM32_ADCSEL == STM32_ADCSEL_PLLSAI1RCLK) ||                           \
    defined(__DOXYGEN__)
/**
 * @brief   PLLSAI1 activation flag.
 */
#define STM32_ACTIVATE_PLLSAI1      TRUE
#else
#define STM32_ACTIVATE_PLLSAI1      FALSE
#endif

/*
 * PLL enable check.
 */
#if (STM32_HSI48_ENABLED && (STM32_CLK48SEL == STM32_CLK48SEL_PLLQCLK)) ||  \
    (STM32_SW == STM32_SW_PLL) ||                                           \
    (STM32_MCOSEL == STM32_MCOSEL_PLLRCLK) ||                               \
    (STM32_SAI1SEL == STM32_SAI1SEL_PLLPCLK) ||                             \
    (STM32_ADCSEL == STM32_ADCSEL_PLLPCLK) ||                               \
    (STM32_ACTIVATE_PLLSAI1) ||                                             \
    defined(__DOXYGEN__)

#if STM32_PLLCLKIN == 0
#error "PLL activation required but no PLL clock selected"
#endif

/**
 * @brief   PLL activation flag.
 */
#define STM32_ACTIVATE_PLL          TRUE
#else
#define STM32_ACTIVATE_PLL          FALSE
#endif

/**
 * @brief   STM32_PLLPEN field.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_PLLPCLK) ||                             \
    (STM32_ADCSEL == STM32_ADCSEL_PLLPCLK) ||                               \
     defined(__DOXYGEN__)
#define STM32_PLLPEN                (1 << 16)
#else
#define STM32_PLLPEN                (0 << 16)
#endif

/**
 * @brief   STM32_PLLQEN field.
 */
#if (STM32_CLK48SEL == STM32_CLK48SEL_PLLQCLK) || defined(__DOXYGEN__)
#define STM32_PLLQEN                (1 << 24)
#else
#define STM32_PLLQEN                (0 << 24)
#endif

/**
 * @brief   STM32_PLLREN field.
 */
#if (STM32_SW == STM32_SW_PLL) ||                                           \
    (STM32_MCOSEL == STM32_MCOSEL_PLLRCLK) ||                                   \
    defined(__DOXYGEN__)
#define STM32_PLLREN                (1 << 28)
#else
#define STM32_PLLREN                (0 << 28)
#endif

/* Inclusion of PLL-related checks and calculations.*/
#include "stm32_pll_v2.inc"

/**
 * @brief   System clock source.
 */
#if (STM32_SW == STM32_SW_MSI)
#define STM32_SYSCLK                STM32_MSICLK

#elif (STM32_SW == STM32_SW_HSI16)
#define STM32_SYSCLK                STM32_HSI16CLK

#elif (STM32_SW == STM32_SW_HSE)
#define STM32_SYSCLK                STM32_HSECLK

#elif (STM32_SW == STM32_SW_PLL)
#define STM32_SYSCLK                STM32_PLL_R_CLKOUT

#else
#error "invalid STM32_SW value specified"
#endif

/* Bus handlers.*/
#include "stm32_ahb.inc"
#include "stm32_apb1.inc"
#include "stm32_apb2.inc"

/**
 * @brief   HCLK2 (CPU2) frequency.
 */
#if (STM32_C2HPRE == STM32_C2HPRE_DIV1) || defined(__DOXYGEN__)
#define STM32_HCLK2                 (STM32_SYSCLK / 1)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV2
#define STM32_HCLK2                 (STM32_SYSCLK / 2)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV3
#define STM32_HCLK2                 (STM32_SYSCLK / 3)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV4
#define STM32_HCLK2                 (STM32_SYSCLK / 4)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV5
#define STM32_HCLK2                 (STM32_SYSCLK / 5)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV6
#define STM32_HCLK2                 (STM32_SYSCLK / 6)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV8
#define STM32_HCLK2                 (STM32_SYSCLK / 8)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV10
#define STM32_HCLK2                 (STM32_SYSCLK / 10)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV16
#define STM32_HCLK2                 (STM32_SYSCLK / 16)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV32
#define STM32_HCLK2                 (STM32_SYSCLK / 32)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV64
#define STM32_HCLK2                 (STM32_SYSCLK / 64)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV128
#define STM32_HCLK2                 (STM32_SYSCLK / 128)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV256
#define STM32_HCLK2                 (STM32_SYSCLK / 256)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV512
#define STM32_HCLK2                 (STM32_SYSCLK / 512)

#else
#error "invalid STM32_C2HPRE value specified"
#endif

/*
 * HCLK2 (CPU2) frequency check.
 */
#if STM32_HCLK2 > STM32_C2HPRE_MAX
#error "STM32_HCLK2 exceeding maximum frequency (STM32_C2HPRE_MAX)"
#endif

/**
 * @brief   AHB4 frequency.
 */
#if (STM32_SHDHPRE == STM32_SHDHPRE_DIV1) || defined(__DOXYGEN__)
#define STM32_HCLK4                 (STM32_SYSCLK / 1)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV2
#define STM32_HCLK4                 (STM32_SYSCLK / 2)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV3
#define STM32_HCLK4                 (STM32_SYSCLK / 3)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV4
#define STM32_HCLK4                 (STM32_SYSCLK / 4)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV5
#define STM32_HCLK4                 (STM32_SYSCLK / 5)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV6
#define STM32_HCLK4                 (STM32_SYSCLK / 6)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV8
#define STM32_HCLK4                 (STM32_SYSCLK / 8)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV10
#define STM32_HCLK4                 (STM32_SYSCLK / 10)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV16
#define STM32_HCLK4                 (STM32_SYSCLK / 16)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV32
#define STM32_HCLK4                 (STM32_SYSCLK / 32)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV64
#define STM32_HCLK4                 (STM32_SYSCLK / 64)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV128
#define STM32_HCLK4                 (STM32_SYSCLK / 128)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV256
#define STM32_HCLK4                 (STM32_SYSCLK / 256)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV512
#define STM32_HCLK4                 (STM32_SYSCLK / 512)

#else
#error "invalid STM32_SHDHPRE value specified"
#endif

/**
 * @brief   STM32_PLLSAI1PEN field.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI1PCLK) ||                         \
    defined(__DOXYGEN__)
#define STM32_PLLSAI1PEN            (1 << 16)
#else
#define STM32_PLLSAI1PEN            (0 << 16)
#endif

/**
 * @brief   STM32_PLLSAI1QEN field.
 */
#if (STM32_CLK48SEL == STM32_CLK48SEL_PLLSAI1QCLK) || defined(__DOXYGEN__)
#define STM32_PLLSAI1QEN            (1 << 24)
#else
#define STM32_PLLSAI1QEN            (0 << 24)
#endif

/**
 * @brief   STM32_PLLSAI1REN field.
 */
#if (STM32_ADCSEL == STM32_ADCSEL_PLLSAI1RCLK) || defined(__DOXYGEN__)
#define STM32_PLLSAI1REN            (1 << 28)
#else
#define STM32_PLLSAI1REN            (0 << 28)
#endif

/* Inclusion of PLLSAI-related checks and calculations, all PLLs share the
   same clock source so enforcing this condition.*/
#define STM32_PLLSAI1CLKIN STM32_PLLCLKIN
#include "stm32_pllsai1_v2.inc"

/**
 * @brief   MCO divider clock frequency.
 */
#if (STM32_MCOSEL == STM32_MCOSEL_NOCLOCK) || defined(__DOXYGEN__)
#define STM32_MCODIVCLK             0

#elif STM32_MCOSEL == STM32_MCOSEL_SYSCLK
#define STM32_MCODIVCLK             STM32_SYSCLK

#elif STM32_MCOSEL == STM32_MCOSEL_MSI
#define STM32_MCODIVCLK             STM32_MSICLK

#elif STM32_MCOSEL == STM32_MCOSEL_HSI16
#define STM32_MCODIVCLK             STM32_HSI16CLK

#elif STM32_MCOSEL == STM32_MCOSEL_HSE
#define STM32_MCODIVCLK             STM32_HSE32CLK

#elif STM32_MCOSEL == STM32_MCOSEL_PLLRCLK
#define STM32_MCODIVCLK             STM32_PLL_P_CLKOUT

#elif (STM32_MCOSEL == STM32_MCOSEL_LSI1 || STM32_MCOSEL == STM32_MCOSEL_LSI2)
#define STM32_MCODIVCLK             STM32_LSICLK

#elif STM32_MCOSEL == STM32_MCOSEL_LSE
#define STM32_MCODIVCLK             STM32_LSECLK

#elif STM32_MCOSEL == STM32_MCOSEL_HSI48
#define STM32_MCODIVCLK             STM32_HSI48CLK

#else
#error "invalid STM32_MCOSEL value specified"
#endif

/**
 * @brief   MCO output pin clock frequency.
 */
#if (STM32_MCOPRE == STM32_MCOPRE_DIV1) || defined(__DOXYGEN__)
#define STM32_MCOCLK                STM32_MCODIVCLK

#elif STM32_MCOPRE == STM32_MCOPRE_DIV2
#define STM32_MCOCLK                (STM32_MCODIVCLK / 2)

#elif STM32_MCOPRE == STM32_MCOPRE_DIV4
#define STM32_MCOCLK                (STM32_MCODIVCLK / 4)

#elif STM32_MCOPRE == STM32_MCOPRE_DIV8
#define STM32_MCOCLK                (STM32_MCODIVCLK / 8)

#elif STM32_MCOPRE == STM32_MCOPRE_DIV16
#define STM32_MCOCLK                (STM32_MCODIVCLK / 16)

#else
#error "invalid STM32_MCOPRE value specified"
#endif

/**
 * @brief   RTC clock frequency.
 */
#if (STM32_RTCSEL == STM32_RTCSEL_NOCLOCK) || defined(__DOXYGEN__)
#define STM32_RTCCLK                0

#elif STM32_RTCSEL == STM32_RTCSEL_LSE
#define STM32_RTCCLK                STM32_LSECLK

#elif STM32_RTCSEL == STM32_RTCSEL_LSI
#define STM32_RTCCLK                STM32_LSICLK

#elif STM32_RTCSEL == STM32_RTCSEL_HSEDIV
#define STM32_RTCCLK                (STM32_HSE32CLK / 32)

#else
#error "invalid STM32_RTCSEL value specified"
#endif

/**
 * @brief   USART1 clock frequency.
 */
#if (STM32_USART1SEL == STM32_USART1SEL_PCLK2) || defined(__DOXYGEN__)
#define STM32_USART1CLK             STM32_PCLK2
#elif STM32_USART1SEL == STM32_USART1SEL_SYSCLK
#define STM32_USART1CLK             STM32_SYSCLK
#elif STM32_USART1SEL == STM32_USART1SEL_HSI16
#define STM32_USART1CLK             STM32_HSI16CLK
#elif STM32_USART1SEL == STM32_USART1SEL_LSE
#define STM32_USART1CLK             STM32_LSECLK
#else
#error "invalid source selected for USART1 clock"
#endif

/**
 * @brief   LPUART1 clock frequency.
 */
#if (STM32_LPUART1SEL == STM32_LPUART1SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_LPUART1CLK            STM32_PCLK1
#elif STM32_LPUART1SEL == STM32_LPUART1SEL_SYSCLK
#define STM32_LPUART1CLK            STM32_SYSCLK
#elif STM32_LPUART1SEL == STM32_LPUART1SEL_HSI16
#define STM32_LPUART1CLK            STM32_HSI16CLK
#elif STM32_LPUART1SEL == STM32_LPUART1SEL_LSE
#define STM32_LPUART1CLK            STM32_LSECLK
#else
#error "invalid source selected for LPUART1 clock"
#endif

/**
 * @brief   I2C1 clock frequency.
 */
#if (STM32_I2C1SEL == STM32_I2C1SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_I2C1CLK               STM32_PCLK1
#elif STM32_I2C1SEL == STM32_I2C1SEL_SYSCLK
#define STM32_I2C1CLK               STM32_SYSCLK
#elif STM32_I2C1SEL == STM32_I2C1SEL_HSI16
#define STM32_I2C1CLK               STM32_HSI16CLK
#else
#error "invalid source selected for I2C1 clock"
#endif

/**
 * @brief   I2C3 clock frequency.
 */
#if (STM32_I2C3SEL == STM32_I2C3SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_I2C3CLK               STM32_PCLK1
#elif STM32_I2C3SEL == STM32_I2C3SEL_SYSCLK
#define STM32_I2C3CLK               STM32_SYSCLK
#elif STM32_I2C3SEL == STM32_I2C3SEL_HSI16
#define STM32_I2C3CLK               STM32_HSI16CLK
#else
#error "invalid source selected for I2C3 clock"
#endif

/**
 * @brief   LPTIM1 clock frequency.
 */
#if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_LPTIM1CLK             STM32_PCLK1
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSI
#define STM32_LPTIM1CLK             STM32_LSICLK
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_HSI16
#define STM32_LPTIM1CLK             STM32_HSI16CLK
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSE
#define STM32_LPTIM1CLK             STM32_LSECLK
#else
#error "invalid source selected for LPTIM1 clock"
#endif

/**
 * @brief   LPTIM2 clock frequency.
 */
#if (STM32_LPTIM2SEL == STM32_LPTIM2SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_LPTIM2CLK             STM32_PCLK1
#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_LSI
#define STM32_LPTIM2CLK             STM32_LSICLK
#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_HSI16
#define STM32_LPTIM2CLK             STM32_HSI16CLK
#elif STM32_LPTIM2SEL == STM32_LPTIM2SEL_LSE
#define STM32_LPTIM2CLK             STM32_LSECLK
#else
#error "invalid source selected for LPTIM2 clock"
#endif

/**
 * @brief   48MHz clock frequency.
 */
#if (STM32_CLK48SEL == STM32_CLK48SEL_HSI48) || defined(__DOXYGEN__)
#define STM32_48CLK                 STM32_HSI48CLK
#elif STM32_CLK48SEL == STM32_CLK48SEL_PLLSAI1QCLK
#define STM32_48CLK                 (STM32_PLLSAI1VCO / STM32_PLLSAI1Q_VALUE)
#elif STM32_CLK48SEL == STM32_CLK48SEL_PLLQCLK
#define STM32_48CLK                 (STM32_PLLVCO / STM32_PLLQ_VALUE)
#elif STM32_CLK48SEL == STM32_CLK48SEL_MSI
#define STM32_48CLK                 STM32_MSICLK
#else
#error "invalid source selected for 48CLK clock"
#endif

/**
 * @brief   SAI1 clock frequency.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI1PCLK) || defined(__DOXYGEN__)
#define STM32_SAI1CLK               STM32_PLLSAI1_P_CLKOUT
#elif STM32_SAI1SEL == STM32_SAI1SEL_PLLPCLK
#define STM32_SAI1CLK               STM32_PLL_P_CLKOUT
#elif STM32_SAI1SEL == STM32_SAI1SEL_EXTCLK
#define STM32_SAI1CLK               0 /* Unknown, would require a board value */
#elif STM32_SAI1SEL == STM32_SAI1SEL_OFF
#define STM32_SAI1CLK               0
#else
#error "invalid source selected for SAI1 clock"
#endif

/**
 * @brief   USB clock point.
 */
#define STM32_USBCLK                STM32_48CLK

/**
 * @brief   RNG clock frequency.
 */
#if (STM32_RNGSEL == STM32_RNGSEL_48CLK) || defined(__DOXYGEN__)
#define STM32_RNGCLK                STM32_48CLK
#elif STM32_RNGSEL == STM32_RNGSEL_LSI
#define STM32_RNGCLK                STM32_LSICLK
#elif STM32_RNGSEL == STM32_RNGSEL_LSE
#define STM32_RNGCLK                STM32_LSECLK
#else
#error "invalid source selected for RNG clock"
#endif

/**
 * @brief   ADC clock frequency.
 */
#if (STM32_ADCSEL == STM32_ADCSEL_NOCLK) || defined(__DOXYGEN__)
#define STM32_ADCCLK                0
#elif STM32_ADCSEL == STM32_ADCSEL_PLLSAI1RCLK
#define STM32_ADCCLK                STM32_PLLSAI1_R_CLKOUT
#elif STM32_ADCSEL == STM32_ADCSEL_PLLPCLK
#define STM32_ADCCLK                STM32_PLL_P_CLKOUT
#elif STM32_ADCSEL == STM32_ADCSEL_SYSCLK
#define STM32_ADCCLK                STM32_SYSCLK
#else
#error "invalid source selected for ADC clock"
#endif

/**
 * @brief   SDMMC frequency.
 */
#define STM32_SDMMC1CLK             STM32_48CLK

/**
 * @brief   Clock of timers connected to APB1
 */
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK1               (STM32_PCLK1 * 1)
#else
#define STM32_TIMCLK1               (STM32_PCLK1 * 2)
#endif

/**
 * @brief   Clock of timers connected to APB2.
 */
#if (STM32_PPRE2 == STM32_PPRE2_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK2               (STM32_PCLK2 * 1)
#else
#define STM32_TIMCLK2               (STM32_PCLK2 * 2)
#endif

/**
 * @brief   Flash settings.
 */
#if (STM32_HCLK <= STM32_0WS_THRESHOLD) || defined(__DOXYGEN__)
#define STM32_FLASHBITS             FLASH_ACR_LATENCY_0WS

#elif STM32_HCLK <= STM32_1WS_THRESHOLD
#define STM32_FLASHBITS             FLASH_ACR_LATENCY_1WS

#elif STM32_HCLK <= STM32_2WS_THRESHOLD
#define STM32_FLASHBITS             FLASH_ACR_LATENCY_2WS

#else
#define STM32_FLASHBITS             FLASH_ACR_LATENCY_3WS
#endif

/**
 * @brief   Flash settings for MSI.
 */
#if (STM32_MSICLK <= STM32_0WS_THRESHOLD) || defined(__DOXYGEN__)
#define STM32_MSI_FLASHBITS         FLASH_ACR_LATENCY_0WS

#elif STM32_MSICLK <= STM32_1WS_THRESHOLD
#define STM32_MSI_FLASHBITS         FLASH_ACR_LATENCY_1WS

#elif STM32_MSICLK <= STM32_2WS_THRESHOLD
#define STM32_MSI_FLASHBITS         FLASH_ACR_LATENCY_2WS

#else
#define STM32_MSI_FLASHBITS         FLASH_ACR_LATENCY_3WS
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

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
#define hal_lld_get_clock_point(clkpt) 0U

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "mpu_v7m.h"
#include "stm32_isr.h"
#include "stm32_dma.h"
#include "stm32_exti.h"
#include "stm32_rcc.h"
#include "stm32_tim.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void stm32_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
