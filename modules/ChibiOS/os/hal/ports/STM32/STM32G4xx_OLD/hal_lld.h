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
 * @file    STM32G4xx/hal_lld.h
 * @brief   STM32G4xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32G431xx, STM32G441xx, STM32G471xx, STM32G491xx.
 *          - STM32G473xx, STM32G483xx.
 *          - STM32G474xx, STM32G484xx.
 *          - STM32GBK1CB.
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
#if defined(STM32G431xx) || defined(STM32G441xx) || defined(STM32G471xx) || defined(STM32G491xx) || \
    defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32G4 Access Line"

#elif defined(STM32G473xx)
#define PLATFORM_NAME           "STM32G4 Performance Line"

#elif defined(STM32G483xx)
#define PLATFORM_NAME           "STM32G4 Performance Line with Crypto"

#elif defined(STM32G474xx)
#define PLATFORM_NAME           "STM32G4 Hi-resolution Line"

#elif defined(STM32G484xx)
#define PLATFORM_NAME           "STM32G4 Hi-resolution Line with Crypto"

#elif defined(STM32GBK1CB)
#define PLATFORM_NAME           "STM32G4 Mystery Line"

#else
#error "STM32G4 device not specified"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(STM32G4XX) || defined(__DOXYGEN__)
#define STM32G4XX
#endif
/** @} */

/**
 * @name   Clock points indexes and names
 * @{
 */
#define CLK_HSI16               0U
#define CLK_HSI48               1U
#define CLK_HSE                 2U
#define CLK_SYSCLK              3U
#define CLK_PLLPCLK             4U
#define CLK_PLLQCLK             5U
#define CLK_PLLRCLK             6U
#define CLK_HCLK                7U
#define CLK_PCLK1               8U
#define CLK_PCLK1TIM            9U
#define CLK_PCLK2               10U
#define CLK_PCLK2TIM            11U
#define CLK_MCO                 12U
#define CLK_ARRAY_SIZE          13U

#define CLK_POINT_NAMES                                                     \
  {                                                                         \
    "HSI16", "HSI48", "HSE", "SYSCLK", "PLLP", "PLLQ", "PLLR", "HCLK",      \
    "PCLK1", "PCLK1TIM", "PCLK2", "PCLK2TIM", "MCO"                         \
  }
/** @} */

/**
 * @name    VOS field definitions
 * @{
 */
#define STM32_VOS_MASK          (3U << 9U)  /**< Core voltage mask.         */
#define STM32_VOS_RANGE1        (1U << 9U)  /**< Core voltage 1.2 Volts.    */
#define STM32_VOS_RANGE2        (2U << 9U)  /**< Core voltage 1.0 Volts.    */
/** @} */

/**
 * @name    RCC_CFGR register bits definitions
 * @{
 */
#define STM32_SW_MASK           (3U << 0U)  /**< SW field mask.             */
#define STM32_SW_HSI16          (1U << 0U)  /**< SYSCLK source is HSI16.    */
#define STM32_SW_HSE            (2U << 0U)  /**< SYSCLK source is HSE.      */
#define STM32_SW_PLLRCLK        (3U << 0U)  /**< SYSCLK source is PLL.      */

#define STM32_MCOSEL_MASK       (15U << 24U)/**< MCOSEL field mask.         */
#define STM32_MCOSEL_NOCLOCK    (0U << 24U) /**< No clock on MCO pin.       */
#define STM32_MCOSEL_SYSCLK     (1U << 24U) /**< SYSCLK on MCO pin.         */
#define STM32_MCOSEL_HSI16      (3U << 24U) /**< HSI16 clock on MCO pin.    */
#define STM32_MCOSEL_HSE        (4U << 24U) /**< HSE clock on MCO pin.      */
#define STM32_MCOSEL_PLLRCLK    (5U << 24U) /**< PLLR clock on MCO pin.     */
#define STM32_MCOSEL_LSI        (6U << 24U) /**< LSI clock on MCO pin.      */
#define STM32_MCOSEL_LSE        (7U << 24U) /**< LSE clock on MCO pin.      */
#define STM32_MCOSEL_HSI48      (8U << 24U) /**< HSI48 clock on MCO pin.    */

#define STM32_MCOPRE_MASK       (7U << 28U) /**< MCOPRE field mask.         */
#define STM32_MCOPRE_FIELD(n)   ((n) << 28U)/**< MCOPRE field value         */
#define STM32_MCOPRE_DIV1       STM32_MCOPRE_FIELD(0U)
#define STM32_MCOPRE_DIV2       STM32_MCOPRE_FIELD(1U)
#define STM32_MCOPRE_DIV4       STM32_MCOPRE_FIELD(2U)
#define STM32_MCOPRE_DIV8       STM32_MCOPRE_FIELD(3U)
#define STM32_MCOPRE_DIV16      STM32_MCOPRE_FIELD(4U)
/** @} */

/**
 * @name    RCC_PLLCFGR register bits definitions
 * @{
 */
#define STM32_PLLSRC_MASK       (3 << 0)    /**< PLL clock source mask.     */
#define STM32_PLLSRC_NOCLOCK    (0 << 0)    /**< PLL clock source disabled. */
#define STM32_PLLSRC_HSI16      (2 << 0)    /**< PLL clock source is HSI16. */
#define STM32_PLLSRC_HSE        (3 << 0)    /**< PLL clock source is HSE.   */
/** @} */

/**
 * @name    RCC_CCIPR register bits definitions
 * @{
 */
#define STM32_USART1SEL_MASK    (3U << 0U)  /**< USART1SEL mask.            */
#define STM32_USART1SEL_PCLK2   (0U << 0U)  /**< USART1 source is PCLK2.    */
#define STM32_USART1SEL_SYSCLK  (1U << 0U)  /**< USART1 source is SYSCLK.   */
#define STM32_USART1SEL_HSI16   (2U << 0U)  /**< USART1 source is HSI16.    */
#define STM32_USART1SEL_LSE     (3U << 0U)  /**< USART1 source is LSE.      */

#define STM32_USART2SEL_MASK    (3U << 2U)  /**< USART2 mask.               */
#define STM32_USART2SEL_PCLK1   (0U << 2U)  /**< USART2 source is PCLK1.    */
#define STM32_USART2SEL_SYSCLK  (1U << 2U)  /**< USART2 source is SYSCLK.   */
#define STM32_USART2SEL_HSI16   (2U << 2U)  /**< USART2 source is HSI16.    */
#define STM32_USART2SEL_LSE     (3U << 2U)  /**< USART2 source is LSE.      */

#define STM32_USART3SEL_MASK    (3U << 4U)  /**< USART3 mask.               */
#define STM32_USART3SEL_PCLK1   (0U << 4U)  /**< USART3 source is PCLK1.    */
#define STM32_USART3SEL_SYSCLK  (1U << 4U)  /**< USART3 source is SYSCLK.   */
#define STM32_USART3SEL_HSI16   (2U << 4U)  /**< USART3 source is HSI16.    */
#define STM32_USART3SEL_LSE     (3U << 4U)  /**< USART3 source is LSE.      */

#define STM32_UART4SEL_MASK     (3U << 6U)  /**< UART4 mask.                */
#define STM32_UART4SEL_PCLK1    (0U << 6U)  /**< UART4 source is PCLK1.     */
#define STM32_UART4SEL_SYSCLK   (1U << 6U)  /**< UART4 source is SYSCLK.    */
#define STM32_UART4SEL_HSI16    (2U << 6U)  /**< UART4 source is HSI16.     */
#define STM32_UART4SEL_LSE      (3U << 6U)  /**< UART4 source is LSE.       */

#define STM32_UART5SEL_MASK     (3U << 8U)  /**< UART5 mask.                */
#define STM32_UART5SEL_PCLK1    (0U << 8U)  /**< UART5 source is PCLK1.     */
#define STM32_UART5SEL_SYSCLK   (1U << 8U)  /**< UART5 source is SYSCLK.    */
#define STM32_UART5SEL_HSI16    (2U << 8U)  /**< UART5 source is HSI16.     */
#define STM32_UART5SEL_LSE      (3U << 8U)  /**< UART5 source is LSE.       */

#define STM32_LPUART1SEL_MASK   (3U << 10U) /**< LPUART1 mask.              */
#define STM32_LPUART1SEL_PCLK1  (0U << 10U) /**< LPUART1 source is PCLK1.   */
#define STM32_LPUART1SEL_SYSCLK (1U << 10U) /**< LPUART1 source is SYSCLK.  */
#define STM32_LPUART1SEL_HSI16  (2U << 10U) /**< LPUART1 source is HSI16.   */
#define STM32_LPUART1SEL_LSE    (3U << 10U) /**< LPUART1 source is LSE.     */

#define STM32_I2C1SEL_MASK      (3U << 12U) /**< I2C1SEL mask.              */
#define STM32_I2C1SEL_PCLK1     (0U << 12U) /**< I2C1 source is PCLK1.      */
#define STM32_I2C1SEL_SYSCLK    (1U << 12U) /**< I2C1 source is SYSCLK.     */
#define STM32_I2C1SEL_HSI16     (2U << 12U) /**< I2C1 source is HSI16.      */

#define STM32_I2C2SEL_MASK      (3U << 14U) /**< I2C2SEL mask.              */
#define STM32_I2C2SEL_PCLK1     (0U << 14U) /**< I2C2 source is PCLK1.      */
#define STM32_I2C2SEL_SYSCLK    (1U << 14U) /**< I2C2 source is SYSCLK.     */
#define STM32_I2C2SEL_HSI16     (2U << 14U) /**< I2C2 source is HSI16.      */

#define STM32_I2C3SEL_MASK      (3U << 16U) /**< I2C3SEL mask.              */
#define STM32_I2C3SEL_PCLK1     (0U << 16U) /**< I2C3 source is PCLK1.      */
#define STM32_I2C3SEL_SYSCLK    (1U << 16U) /**< I2C3 source is SYSCLK.     */
#define STM32_I2C3SEL_HSI16     (2U << 16U) /**< I2C3 source is HSI16.      */

#define STM32_LPTIM1SEL_MASK    (3U << 18U) /**< LPTIM1SEL mask.            */
#define STM32_LPTIM1SEL_PCLK1   (0U << 18U) /**< LPTIM1 source is PCLK1.    */
#define STM32_LPTIM1SEL_LSI     (1U << 18U) /**< LPTIM1 source is LSI.      */
#define STM32_LPTIM1SEL_HSI16   (2U << 18U) /**< LPTIM1 source is HSI16.    */
#define STM32_LPTIM1SEL_LSE     (3U << 18U) /**< LPTIM1 source is LSE.      */

#define STM32_SAI1SEL_MASK      (3U << 20U) /**< SAI1SEL mask.              */
#define STM32_SAI1SEL_SYSCLK    (0U << 20U) /**< SAI1 source is SYSCLK.     */
#define STM32_SAI1SEL_PLLQCLK   (1U << 20U) /**< SAI1 source is PLLQCLK.    */
#define STM32_SAI1SEL_CKIN      (2U << 20U) /**< SAI1 source is CKIN.       */
#define STM32_SAI1SEL_HSI16     (3U << 20U) /**< SAI1 source is HSI16.      */

#define STM32_I2S23SEL_MASK     (3U << 22U) /**< I2S23SEL mask.             */
#define STM32_I2S23SEL_SYSCLK   (0U << 22U) /**< I2S23 source is SYSCLK.    */
#define STM32_I2S23SEL_PLLQCLK  (1U << 22U) /**< I2S23 source is PLLQCLK.   */
#define STM32_I2S23SEL_CKIN     (2U << 22U) /**< I2S23 source is CKIN.      */
#define STM32_I2S23SEL_HSI16    (3U << 22U) /**< I2S23 source is HSI16.     */

#define STM32_FDCANSEL_MASK     (3U << 24U) /**< FDCANSEL mask.             */
#define STM32_FDCANSEL_HSE      (0U << 24U) /**< FDCAN source is HSE.       */
#define STM32_FDCANSEL_PLLQCLK  (1U << 24U) /**< FDCAN source is PLLQCLK.   */
#define STM32_FDCANSEL_PCLK1    (2U << 24U) /**< FDCAN source is PCLK1.    */

#define STM32_CLK48SEL_MASK     (3U << 26U) /**< CLK48SEL mask.             */
#define STM32_CLK48SEL_HSI48    (0U << 26U) /**< CLK48 source is HSI48.     */
#define STM32_CLK48SEL_PLLQCLK  (2U << 26U) /**< CLK48 source is PLLQCLK.   */

#define STM32_ADC12SEL_MASK     (3U << 28U) /**< ADC12SEL mask.             */
#define STM32_ADC12SEL_NOCLK    (0U << 28U) /**< ADC12 source is none.      */
#define STM32_ADC12SEL_PLLPCLK  (1U << 28U) /**< ADC12 source is PLLPCLK.   */
#define STM32_ADC12SEL_SYSCLK   (2U << 28U) /**< ADC12 source is SYSCLK.    */

#define STM32_ADC345SEL_MASK    (3U << 30U) /**< ADC345SEL mask.            */
#define STM32_ADC345SEL_NOCLK   (0U << 30U) /**< ADC345 source is none.     */
#define STM32_ADC345SEL_PLLPCLK (1U << 30U) /**< ADC345 source is PLLPCLK.  */
#define STM32_ADC345SEL_SYSCLK  (2U << 30U) /**< ADC345 source is SYSCLK.   */
/** @} */

/**
 * @name    RCC_CCIPR2 register bits definitions
 * @{
 */
#define STM32_I2C4SEL_MASK      (3U << 0U)  /**< I2C4SEL mask.              */
#define STM32_I2C4SEL_PCLK1     (0U << 0U)  /**< I2C4 source is PCLK1.      */
#define STM32_I2C4SEL_SYSCLK    (1U << 0U)  /**< I2C4 source is SYSCLK.     */
#define STM32_I2C4SEL_HSI16     (2U << 0U)  /**< I2C4 source is HSI16.      */

#define STM32_QSPISEL_MASK      (3U << 20U) /**< QSPISEL mask.              */
#define STM32_QSPISEL_SYSCLK    (0U << 20U) /**< QSPI source is SYSCLK.     */
#define STM32_QSPISEL_HSI16     (1U << 20U) /**< QSPI source is HSI16.      */
#define STM32_QSPISEL_PLLQCLK   (2U << 20U) /**< QSPI source is PLLQCLK.    */
/** @} */

/**
 * @name    RCC_BDCR register bits definitions
 * @{
 */
#define STM32_RTCSEL_MASK       (3U << 8U)  /**< RTC source mask.           */
#define STM32_RTCSEL_NOCLOCK    (0U << 8U)  /**< No RTC source.             */
#define STM32_RTCSEL_LSE        (1U << 8U)  /**< RTC source is LSE.         */
#define STM32_RTCSEL_LSI        (2U << 8U)  /**< RTC source is LSI.         */
#define STM32_RTCSEL_HSEDIV     (3U << 8U)  /**< RTC source is HSE divided. */

#define STM32_LSCOSEL_MASK      (3U << 24U) /**< LSCO pin clock source.     */
#define STM32_LSCOSEL_NOCLOCK   (0U << 24U) /**< No clock on LSCO pin.      */
#define STM32_LSCOSEL_LSI       (1U << 24U) /**< LSI on LSCO pin.           */
#define STM32_LSCOSEL_LSE       (3U << 24U) /**< LSE on LSCO pin.           */
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
 * @brief   Core voltage selection.
 * @note    This setting affects all the performance and clock related
 *          settings, the maximum performance is only obtainable selecting
 *          the maximum voltage.
 */
#if !defined(STM32_VOS) || defined(__DOXYGEN__)
#define STM32_VOS                           STM32_VOS_RANGE1
#endif

/**
 * @brief   Core voltage boost.
 * @note    The boost can only be used when STM32_VOS==STM32_VOS_RANGE1.
 */
#if !defined(STM32_PWR_BOOST) || defined(__DOXYGEN__)
#define STM32_PWR_BOOST                     TRUE
#endif

/**
 * @brief   PWR CR2 register initialization value.
 */
#if !defined(STM32_PWR_CR2) || defined(__DOXYGEN__)
#define STM32_PWR_CR2                       (PWR_CR2_PLS_LEV0)
#endif

/**
 * @brief   PWR CR3 register initialization value.
 */
#if !defined(STM32_PWR_CR3) || defined(__DOXYGEN__)
#define STM32_PWR_CR3                       (PWR_CR3_EIWF)
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
 * @brief   Main clock source selection.
 * @note    If the selected clock source is not the PLL then the PLL is not
 *          initialized and started.
 * @note    The default value is calculated for a 170MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                            STM32_SW_PLLRCLK
#endif

/**
 * @brief   Clock source for the PLL.
 * @note    This setting has only effect if the PLL is selected as the
 *          system clock source.
 * @note    The default value is calculated for a 170MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_PLLSRC) || defined(__DOXYGEN__)
#define STM32_PLLSRC                        STM32_PLLSRC_HSI16
#endif

/**
 * @brief   PLLM divider value.
 * @note    The allowed values are 1..16.
 * @note    The default value is calculated for a 170MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_PLLM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLM_VALUE                    4
#endif

/**
 * @brief   PLLN multiplier value.
 * @note    The allowed values are 8..127.
 * @note    The default value is calculated for a 170MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_PLLN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLN_VALUE                    84
#endif

/**
 * @brief   PLLPDIV divider value or zero if disabled.
 * @note    The allowed values are 0, 2..31.
 */
#if !defined(STM32_PLLPDIV_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLPDIV_VALUE                 0
#endif

/**
 * @brief   PLLP divider value.
 * @note    The allowed values are 7, 17.
 */
#if !defined(STM32_PLLP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLP_VALUE                    7
#endif

/**
 * @brief   PLLQ divider value.
 * @note    The allowed values are 2, 4, 6, 8.
 */
#if !defined(STM32_PLLQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLQ_VALUE                    8
#endif

/**
 * @brief   PLLR divider value.
 * @note    The allowed values are 2, 4, 6, 8.
 * @note    The default value is calculated for a 170MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_PLLR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLR_VALUE                    2
#endif

/**
 * @brief   AHB prescaler value.
 * @note    The default value is calculated for a 170MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_HPRE) || defined(__DOXYGEN__)
#define STM32_HPRE                          STM32_HPRE_DIV1
#endif

/**
 * @brief   APB1 prescaler value.
 */
#if !defined(STM32_PPRE1) || defined(__DOXYGEN__)
#define STM32_PPRE1                         STM32_PPRE1_DIV2
#endif

/**
 * @brief   APB2 prescaler value.
 */
#if !defined(STM32_PPRE2) || defined(__DOXYGEN__)
#define STM32_PPRE2                         STM32_PPRE2_DIV1
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
 * @brief   USART1 clock source.
 */
#if !defined(STM32_USART1SEL) || defined(__DOXYGEN__)
#define STM32_USART1SEL                     STM32_USART1SEL_SYSCLK
#endif

/**
 * @brief   USART2 clock source.
 */
#if !defined(STM32_USART2SEL) || defined(__DOXYGEN__)
#define STM32_USART2SEL                     STM32_USART2SEL_SYSCLK
#endif

/**
 * @brief   USART3 clock source.
 */
#if !defined(STM32_USART3SEL) || defined(__DOXYGEN__)
#define STM32_USART3SEL                     STM32_USART3SEL_SYSCLK
#endif

/**
 * @brief   UART4 clock source.
 */
#if !defined(STM32_UART4SEL) || defined(__DOXYGEN__)
#define STM32_UART4SEL                      STM32_UART4SEL_SYSCLK
#endif

/**
 * @brief   UART5 clock source.
 */
#if !defined(STM32_UART5SEL) || defined(__DOXYGEN__)
#define STM32_UART5SEL                      STM32_UART5SEL_SYSCLK
#endif

/**
 * @brief   LPUART1 clock source.
 */
#if !defined(STM32_LPUART1SEL) || defined(__DOXYGEN__)
#define STM32_LPUART1SEL                    STM32_LPUART1SEL_SYSCLK
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
#define STM32_I2C3SEL                       STM32_I2C3SEL_PCLK1
#endif

/**
 * @brief   I2C4 clock source.
 */
#if !defined(STM32_I2C4SEL) || defined(__DOXYGEN__)
#define STM32_I2C4SEL                       STM32_I2C4SEL_PCLK1
#endif

/**
 * @brief   LPTIM1 clock source.
 */
#if !defined(STM32_LPTIM1SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM1SEL                     STM32_LPTIM1SEL_PCLK1
#endif

/**
 * @brief   SAI1 clock source.
 */
#if !defined(STM32_SAI1SEL) || defined(__DOXYGEN__)
#define STM32_SAI1SEL                       STM32_SAI1SEL_SYSCLK
#endif

/**
 * @brief   I2S23 clock source.
 */
#if !defined(STM32_I2S23SEL) || defined(__DOXYGEN__)
#define STM32_I2S23SEL                      STM32_I2S23SEL_SYSCLK
#endif

/**
 * @brief   FDCAN clock source.
 */
#if !defined(STM32_FDCANSEL) || defined(__DOXYGEN__)
#define STM32_FDCANSEL                      STM32_FDCANSEL_HSE
#endif

/**
 * @brief   CLK48 clock source.
 */
#if !defined(STM32_CLK48SEL) || defined(__DOXYGEN__)
#define STM32_CLK48SEL                      STM32_CLK48SEL_HSI48
#endif

/**
 * @brief   ADC12 clock source.
 */
#if !defined(STM32_ADC12SEL) || defined(__DOXYGEN__)
#define STM32_ADC12SEL                      STM32_ADC12SEL_PLLPCLK
#endif

/**
 * @brief   ADC34 clock source.
 */
#if !defined(STM32_ADC345SEL) || defined(__DOXYGEN__)
#define STM32_ADC345SEL                     STM32_ADC345SEL_PLLPCLK
#endif

/**
 * @brief   QSPI clock source.
 */
#if !defined(STM32_QSPISEL) || defined(__DOXYGEN__)
#define STM32_QSPISEL                       STM32_QSPISEL_SYSCLK
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

/* Boost mode checks.*/
#if STM32_PWR_BOOST && (STM32_VOS != STM32_VOS_RANGE1)
#error  "STM32_PWR_BOOST requires STM32_VOS_RANGE1"
#endif

/*
 * Configuration-related checks.
 */
#if !defined(STM32G4xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32G4xx_MCUCONF not defined"
#endif

#if defined(STM32G431xx) && !defined(STM32G431_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32G431_MCUCONF not defined"

#elif defined(STM32G441xx) && !defined(STM32G441_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32G441_MCUCONF not defined"

#elif defined(STM32G471xx) && !defined(STM32G471_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32G471_MCUCONF not defined"

#elif defined(STM32G473xx) && !defined(STM32G473_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32G473_MCUCONF not defined"

#elif defined(STM32G483xx) && !defined(STM32G483_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32G483_MCUCONF not defined"

#elif defined(STM32G474xx) && !defined(STM32G474_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32G474_MCUCONF not defined"

#elif defined(STM32G484xx) && !defined(STM32G484_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32G484_MCUCONF not defined"

#elif defined(STM32GBK1CB) && !defined(STM32GBK1CB_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32GBK1CB_MCUCONF not defined"

#elif defined(STM32G491xx) && !defined(STM32G491_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32G491_MCUCONF not defined"

#endif

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

/* Clock handlers.*/
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

  #if STM32_SW == STM32_SW_HSI16
    #error "HSI16 not enabled, required by STM32_SW"
  #endif

  #if (STM32_SW == STM32_SW_PLLRCLK) && (STM32_PLLSRC == STM32_PLLSRC_HSI16)
    #error "HSI16 not enabled, required by STM32_SW and STM32_PLLSRC"
  #endif

  #if (STM32_MCOSEL == STM32_MCOSEL_HSI16) ||                               \
      ((STM32_MCOSEL == STM32_MCOSEL_PLLRCLK) &&                            \
       (STM32_PLLSRC == STM32_PLLSRC_HSI16))
    #error "HSI16 not enabled, required by STM32_MCOSEL"
  #endif

  #if (STM32_USART1SEL == STM32_USART1SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_USART1SEL"
  #endif
  #if (STM32_USART2SEL == STM32_USART2SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_USART2SEL"
  #endif
  #if (STM32_USART3SEL == STM32_USART3SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_USART3SEL"
  #endif
  #if (STM32_UART4SEL == STM32_UART4SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_UART4SEL"
  #endif
  #if (STM32_UART5SEL == STM32_UART5SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_UART5SEL"
  #endif
  #if (STM32_LPUART1SEL == STM32_LPUART1SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_LPUART1SEL"
  #endif

  #if (STM32_I2C1SEL == STM32_I2C1SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_I2C1SEL"
  #endif
  #if (STM32_I2C2SEL == STM32_I2C2SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_I2C2SEL"
  #endif
  #if (STM32_I2C3SEL == STM32_I2C3SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_I2C3SEL"
  #endif
  #if (STM32_I2C4SEL == STM32_I2C4SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_I2C4SEL"
  #endif

  #if (STM32_SAI1SEL == STM32_SAI1SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_SAI1SEL"
  #endif
  #if (STM32_I2S23SEL == STM32_I2S23SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_I2S23SEL"
  #endif

  #if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_HSI16)
    #error "HSI16 not enabled, required by STM32_LPTIM1SEL"
  #endif

  #if (STM32_QSPISEL == STM32_QSPISEL_HSI16)
    #error "HSI16 not enabled, required by STM32_QSPISEL"
  #endif

#endif /* !STM32_HSI16_ENABLED */

/*
 * HSI48 related checks.
 */
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
#if STM32_HSE_ENABLED

#else /* !STM32_HSE_ENABLED */

  #if STM32_SW == STM32_SW_HSE
    #error "HSE not enabled, required by STM32_SW"
  #endif

  #if (STM32_SW == STM32_SW_PLLRCLK) && (STM32_PLLSRC == STM32_PLLSRC_HSE)
    #error "HSE not enabled, required by STM32_SW and STM32_PLLSRC"
  #endif

  #if (STM32_MCOSEL == STM32_MCOSEL_HSE) ||                                 \
      ((STM32_MCOSEL == STM32_MCOSEL_PLLRCLK) &&                            \
       (STM32_PLLSRC == STM32_PLLSRC_HSE))
    #error "HSE not enabled, required by STM32_MCOSEL"
  #endif

  #if STM32_RTCSEL == STM32_RTCSEL_HSEDIV
    #error "HSE not enabled, required by STM32_RTCSEL"
  #endif

#endif /* !STM32_HSE_ENABLED */

/*
 * LSI related checks.
 */
#if STM32_LSI_ENABLED
#else /* !STM32_LSI_ENABLED */

  #if HAL_USE_RTC && (STM32_RTCSEL == STM32_RTCSEL_LSI)
    #error "LSI not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_MCOSEL == STM32_MCOSEL_LSI
    #error "LSI not enabled, required by STM32_MCOSEL"
  #endif

  #if STM32_LSCOSEL == STM32_LSCOSEL_LSI
    #error "LSI not enabled, required by STM32_LSCOSEL"
  #endif

#endif /* !STM32_LSI_ENABLED */

/*
 * LSE related checks.
 */
#if STM32_LSE_ENABLED
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

#endif /* !STM32_LSE_ENABLED */

/**
 * @brief   PLL input clock frequency.
 */
#if (STM32_PLLSRC == STM32_PLLSRC_HSE) || defined(__DOXYGEN__)
  #define STM32_PLLCLKIN            (STM32_HSECLK / STM32_PLLM_VALUE)

#elif STM32_PLLSRC == STM32_PLLSRC_HSI16
  #define STM32_PLLCLKIN            (STM32_HSI16CLK / STM32_PLLM_VALUE)

#elif STM32_PLLSRC == STM32_PLLSRC_NOCLOCK
  #define STM32_PLLCLKIN            0

#else
  #error "invalid STM32_PLLSRC value specified"
#endif

/*
 * PLL enable check.
 */
#if (STM32_SW == STM32_SW_PLLRCLK) ||                                       \
    (STM32_MCOSEL == STM32_MCOSEL_PLLRCLK) ||                               \
    (STM32_ADC12SEL == STM32_ADC12SEL_PLLPCLK) ||                           \
    (STM32_ADC345SEL == STM32_ADC345SEL_PLLPCLK) ||                         \
    (STM32_SAI1SEL == STM32_SAI1SEL_PLLQCLK) ||                             \
    (STM32_I2S23SEL == STM32_I2S23SEL_PLLQCLK) ||                           \
    (STM32_FDCANSEL == STM32_FDCANSEL_PLLQCLK) ||                           \
    (STM32_CLK48SEL == STM32_CLK48SEL_PLLQCLK) ||                           \
    (STM32_QSPISEL == STM32_QSPISEL_PLLQCLK) ||                             \
    defined(__DOXYGEN__)
  /**
   * @brief   PLL activation flag.
   */
  #define STM32_ACTIVATE_PLL        TRUE
#else

  #define STM32_ACTIVATE_PLL        FALSE
#endif

/**
 * @brief   STM32_PLLPEN field.
 */
#if (STM32_ADC12SEL == STM32_ADC12SEL_PLLPCLK) ||                           \
    (STM32_ADC345SEL == STM32_ADC345SEL_PLLPCLK) ||                         \
    defined(__DOXYGEN__)
  #define STM32_PLLPEN              (1 << 16)

#else
  #define STM32_PLLPEN              (0 << 16)
#endif

/**
 * @brief   STM32_PLLQEN field.
 */
#if (STM32_QSPISEL == STM32_QSPISEL_PLLQCLK) ||                             \
    (STM32_FDCANSEL == STM32_FDCANSEL_PLLQCLK) ||                           \
    (STM32_CLK48SEL == STM32_CLK48SEL_PLLQCLK) ||                           \
    (STM32_SAI1SEL == STM32_SAI1SEL_PLLQCLK) ||                             \
    (STM32_I2S23SEL == STM32_I2S23SEL_PLLQCLK) ||                           \
    defined(__DOXYGEN__)
  #define STM32_PLLQEN              (1 << 20)

#else
  #define STM32_PLLQEN              (0 << 20)
#endif

/**
 * @brief   STM32_PLLREN field.
 */
#if (STM32_SW == STM32_SW_PLLRCLK) ||                                       \
    (STM32_MCOSEL == STM32_MCOSEL_PLLRCLK) ||                               \
    defined(__DOXYGEN__)
  #define STM32_PLLREN              (1 << 24)

#else
  #define STM32_PLLREN              (0 << 24)
#endif

/* Inclusion of PLL-related checks and calculations.*/
#include <stm32_pll.inc>

/**
 * @brief   System clock source.
 */
#if (STM32_SW == STM32_SW_HSI16)
  #define STM32_SYSCLK              STM32_HSI16CLK

#elif (STM32_SW == STM32_SW_HSE)
  #define STM32_SYSCLK              STM32_HSECLK

#elif (STM32_SW == STM32_SW_PLLRCLK)
  #define STM32_SYSCLK              STM32_PLL_R_CLKOUT

#else
  #error "invalid STM32_SW value specified"
#endif

/* Bus handlers.*/
#include "stm32_ahb.inc"
#include "stm32_apb1.inc"
#include "stm32_apb2.inc"

/**
 * @brief   MCO divider clock frequency.
 */
#if (STM32_MCOSEL == STM32_MCOSEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_MCODIVCLK           0

#elif STM32_MCOSEL == STM32_MCOSEL_SYSCLK
  #define STM32_MCODIVCLK           STM32_SYSCLK

#elif STM32_MCOSEL == STM32_MCOSEL_HSI16
  #define STM32_MCODIVCLK           STM32_HSI16CLK

#elif STM32_MCOSEL == STM32_MCOSEL_HSE
  #define STM32_MCODIVCLK           STM32_HSECLK

#elif STM32_MCOSEL == STM32_MCOSEL_PLLRCLK
  #define STM32_MCODIVCLK           STM32_PLL_R_CLKOUT

#elif STM32_MCOSEL == STM32_MCOSEL_LSI
  #define STM32_MCODIVCLK           STM32_LSICLK

#elif STM32_MCOSEL == STM32_MCOSEL_LSE
  #define STM32_MCODIVCLK           STM32_LSECLK

#elif STM32_MCOSEL == STM32_MCOSEL_HSI48
  #define STM32_MCODIVCLK           STM32_HSI48CLK

#else
  #error "invalid STM32_MCOSEL value specified"
#endif

/**
 * @brief   MCO output pin clock frequency.
 */
#if (STM32_MCOPRE == STM32_MCOPRE_DIV1) || defined(__DOXYGEN__)
  #define STM32_MCOCLK              STM32_MCODIVCLK

#elif STM32_MCOPRE == STM32_MCOPRE_DIV2
  #define STM32_MCOCLK              (STM32_MCODIVCLK / 2)

#elif STM32_MCOPRE == STM32_MCOPRE_DIV4
  #define STM32_MCOCLK              (STM32_MCODIVCLK / 4)

#elif STM32_MCOPRE == STM32_MCOPRE_DIV8
  #define STM32_MCOCLK              (STM32_MCODIVCLK / 8)

#elif STM32_MCOPRE == STM32_MCOPRE_DIV16
  #define STM32_MCOCLK              (STM32_MCODIVCLK / 16)

#else
#error "invalid STM32_MCOPRE value specified"
#endif

/**
 * @brief   RTC clock frequency.
 */
#if (STM32_RTCSEL == STM32_RTCSEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_RTCCLK              0

#elif STM32_RTCSEL == STM32_RTCSEL_LSE
  #define STM32_RTCCLK              STM32_LSECLK

#elif STM32_RTCSEL == STM32_RTCSEL_LSI
  #define STM32_RTCCLK              STM32_LSICLK

#elif STM32_RTCSEL == STM32_RTCSEL_HSEDIV
  #define STM32_RTCCLK              (STM32_HSECLK / 32)

#else
  #error "invalid STM32_RTCSEL value specified"
#endif

/**
 * @brief   USART1 clock frequency.
 */
#if (STM32_USART1SEL == STM32_USART1SEL_PCLK2) || defined(__DOXYGEN__)
  #define STM32_USART1CLK           hal_lld_get_clock_point(CLK_PCLK2)

#elif STM32_USART1SEL == STM32_USART1SEL_SYSCLK
  #define STM32_USART1CLK           hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_USART1SEL == STM32_USART1SEL_HSI16
  #define STM32_USART1CLK           STM32_HSI16CLK

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

 #elif STM32_USART2SEL == STM32_USART2SEL_SYSCLK
   #define STM32_USART2CLK           hal_lld_get_clock_point(CLK_SYSCLK)

 #elif STM32_USART2SEL == STM32_USART2SEL_HSI16
   #define STM32_USART2CLK           STM32_HSI16CLK

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

 #elif STM32_USART3SEL == STM32_USART3SEL_SYSCLK
   #define STM32_USART3CLK           hal_lld_get_clock_point(CLK_SYSCLK)

 #elif STM32_USART3SEL == STM32_USART3SEL_HSI16
   #define STM32_USART3CLK           STM32_HSI16CLK

 #elif STM32_USART3SEL == STM32_USART3SEL_LSE
   #define STM32_USART3CLK           STM32_LSECLK

 #else
   #error "invalid source selected for USART3 clock"
 #endif

/**
 * @brief   UART4 clock frequency.
 */
#if (STM32_UART4SEL == STM32_UART4SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART4CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART4SEL == STM32_UART4SEL_SYSCLK
  #define STM32_UART4CLK           hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_UART4SEL == STM32_UART4SEL_HSI16
  #define STM32_UART4CLK           STM32_HSI16CLK

#elif STM32_UART4SEL == STM32_UART4SEL_LSE
  #define STM32_UART4CLK           STM32_LSECLK

#else
  #error "invalid source selected for UART4 clock"
#endif

/**
 * @brief   UART5 clock frequency.
 */
#if (STM32_UART5SEL == STM32_UART5SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_UART5CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_UART5SEL == STM32_UART5SEL_SYSCLK
  #define STM32_UART5CLK           hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_UART5SEL == STM32_UART5SEL_HSI16
  #define STM32_UART5CLK           STM32_HSI16CLK

#elif STM32_UART5SEL == STM32_UART5SEL_LSE
  #define STM32_UART5CLK           STM32_LSECLK

#else
  #error "invalid source selected for UART5 clock"
#endif

/**
 * @brief   LPUART1 clock frequency.
 */
#if (STM32_LPUART1SEL == STM32_LPUART1SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_LPUART1SEL == STM32_LPUART1SEL_SYSCLK
  #define STM32_LPUART1CLK          hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_LPUART1SEL == STM32_LPUART1SEL_HSI16
  #define STM32_LPUART1CLK          STM32_HSI16CLK

#elif STM32_LPUART1SEL == STM32_LPUART1SEL_LSE
  #define STM32_LPUART1CLK          STM32_LSECLK

#else
#error "invalid source selected for LPUART1 clock"
#endif

/**
 * @brief   I2C1 clock frequency.
 */
#if (STM32_I2C1SEL == STM32_I2C1SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_I2C1CLK             hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_I2C1SEL == STM32_I2C1SEL_SYSCLK
  #define STM32_I2C1CLK             hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_I2C1SEL == STM32_I2C1SEL_HSI16
  #define STM32_I2C1CLK             STM32_HSI16CLK

#else
  #error "invalid source selected for I2C1 clock"
#endif

/**
 * @brief   I2C2 clock frequency.
 */
#if (STM32_I2C2SEL == STM32_I2C2SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_I2C2CLK             hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_I2C2SEL == STM32_I2C2SEL_SYSCLK
  #define STM32_I2C2CLK             hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_I2C2SEL == STM32_I2C2SEL_HSI16
  #define STM32_I2C2CLK             STM32_HSI16CLK

#else
  #error "invalid source selected for I2C2 clock"
#endif

/**
 * @brief   I2C3 clock frequency.
 */
#if (STM32_I2C3SEL == STM32_I2C3SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_I2C3CLK             hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_I2C3SEL == STM32_I2C3SEL_SYSCLK
  #define STM32_I2C3CLK             hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_I2C3SEL == STM32_I2C3SEL_HSI16
  #define STM32_I2C3CLK             STM32_HSI16CLK

#else
  #error "invalid source selected for I2C3 clock"
#endif

/**
 * @brief   I2C4 clock frequency.
 */
#if (STM32_I2C4SEL == STM32_I2C4SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_I2C4CLK             hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_I2C4SEL == STM32_I2C4SEL_SYSCLK
  #define STM32_I2C4CLK             hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_I2C4SEL == STM32_I2C4SEL_HSI16
  #define STM32_I2C4CLK             STM32_HSI16CLK

#else
  #error "invalid source selected for I2C4 clock"
#endif

/**
 * @brief   LPTIM1 clock frequency.
 */
#if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_PCLK1) || defined(__DOXYGEN__)
  #define STM32_LPTIM1CLK           hal_lld_get_clock_point(CLK_PCLK1)

#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSI
  #define STM32_LPTIM1CLK           STM32_LSICLK

#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_HSI16
  #define STM32_LPTIM1CLK           STM32_HSI16CLK

#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSE
  #define STM32_LPTIM1CLK           STM32_LSECLK

#else
  #error "invalid source selected for LPTIM1 clock"
#endif

/**
 * @brief   SAI1 clock frequency.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_SYSCLK) || defined(__DOXYGEN__)
  #define STM32_SAI1CLK             hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_SAI1SEL == STM32_SAI1SEL_PLLQCLK
  #define STM32_SAI1CLK             hal_lld_get_clock_point(CLK_PLLQCLK)

#elif STM32_SAI1SEL == STM32_SAI1SEL_HSI16
  #define STM32_SAI1CLK             STM32_HSI16CLK

#elif STM32_SAI1SEL == STM32_SAI1SEL_CKIN
  #define STM32_SAI1CLK             0 /* Unknown, would require a board value */

#else
  #error "invalid source selected for SAI1 clock"
#endif

/**
 * @brief   I2S23 clock frequency.
 */
#if (STM32_I2S23SEL == STM32_I2S23SEL_SYSCLK) || defined(__DOXYGEN__)
  #define STM32_I2S23CLK            hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_I2S23SEL == STM32_I2S23SEL_PLLQCLK
  #define STM32_I2S23CLK            hal_lld_get_clock_point(CLK_PLLQCLK)

#elif STM32_I2S23SEL == STM32_I2S23SEL_HSI16
  #define STM32_I2S23CLK            STM32_HSI16CLK

#elif STM32_I2S23SEL == STM32_I2S23SEL_CKIN
  #define STM32_I2S23CLK            0 /* Unknown, would require a board value */

#else
  #error "invalid source selected for I2S23 clock"
#endif

/**
 * @brief   FDCAN clock frequency.
 */
#if (STM32_FDCANSEL == STM32_FDCANSEL_HSE) || defined(__DOXYGEN__)
  #define STM32_FDCANCLK            STM32_HSECLK

#elif STM32_FDCANSEL == STM32_FDCANSEL_PLLQCLK
  #define STM32_FDCANCLK            hal_lld_get_clock_point(CLK_PLLQCLK)

#elif STM32_FDCANSEL == STM32_FDCANSEL_PCLK1
  #define STM32_FDCANCLK            hal_lld_get_clock_point(CLK_PCLK1)

#else
  #error "invalid source selected for FDCAN clock"
#endif

/**
 * @brief   48MHz clock frequency.
 */
#if (STM32_CLK48SEL == STM32_CLK48SEL_HSI48) || defined(__DOXYGEN__)
  #define STM32_48CLK               STM32_HSI48CLK

#elif STM32_CLK48SEL == STM32_CLK48SEL_PLLQCLK
  #define STM32_48CLK               hal_lld_get_clock_point(CLK_PLLQCLK)

#else
  #error "invalid source selected for 48MHz clock"
#endif

/**
 * @brief   ADC12 clock frequency.
 */
#if (STM32_ADC12SEL == STM32_ADC12SEL_NOCLK) || defined(__DOXYGEN__)
  #define STM32_ADC12CLK            0

#elif STM32_ADC12SEL == STM32_ADC12SEL_PLLPCLK
  #define STM32_ADC12CLK            hal_lld_get_clock_point(CLK_PLLPCLK)

#elif STM32_ADC12SEL == STM32_ADC12SEL_SYSCLK
  #define STM32_ADC12CLK            hal_lld_get_clock_point(CLK_SYSCLK)

#else
  #error "invalid source selected for ADC clock"
#endif

/**
 * @brief   ADC345 clock frequency.
 */
#if (STM32_ADC345SEL == STM32_ADC345SEL_NOCLK) || defined(__DOXYGEN__)
  #define STM32_ADC345CLK           0

#elif STM32_ADC345SEL == STM32_ADC345SEL_PLLPCLK
  #define STM32_ADC345CLK           hal_lld_get_clock_point(CLK_PLLPCLK)

#elif STM32_ADC345SEL == STM32_ADC345SEL_SYSCLK
  #define STM32_ADC345CLK           hal_lld_get_clock_point(CLK_SYSCLK)

#else
  #error "invalid source selected for ADC clock"
#endif

/**
 * @brief   TIMP1CLK clock frequency.
 */
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) || defined(__DOXYGEN__)
  #define STM32_TIMP1CLK            (STM32_PCLK1 * 1)
#else
  #define STM32_TIMP1CLK            (STM32_PCLK1 * 2)
#endif

/**
 * @brief   TIMP2CLK clock frequency.
 */
#if (STM32_PPRE2 == STM32_PPRE2_DIV1) || defined(__DOXYGEN__)
  #define STM32_TIMP2CLK            (STM32_PCLK2 * 1)
#else
  #define STM32_TIMP2CLK            (STM32_PCLK2 * 2)
#endif

/**
 * @brief   Clock of timers connected to APB1.
 */
#define STM32_TIMCLK1               hal_lld_get_clock_point(CLK_PCLK1TIM)

/**
 * @brief   Clock of timers connected to APB2.
 */
#define STM32_TIMCLK2               hal_lld_get_clock_point(CLK_PCLK2TIM)

/**
 * @brief   RNG clock point.
 */
#define STM32_RNGCLK                STM32_48CLK

/**
 * @brief   USB clock point.
 */
#define STM32_USBCLK                STM32_48CLK

/**
 * @brief   Voltage boost settings.
 */
#if STM32_PWR_BOOST || defined(__DOXYGEN__)
#define STM32_CR5BITS               PWR_CR5_R1MODE
#else
#define STM32_CR5BITS               0U
#endif

/**
 * @brief   Flash settings.
 */
#if (STM32_HCLK <= STM32_0WS_THRESHOLD) || defined(__DOXYGEN__)
  #define STM32_FLASHBITS           FLASH_ACR_LATENCY_0WS

#elif STM32_HCLK <= STM32_1WS_THRESHOLD
  #define STM32_FLASHBITS           FLASH_ACR_LATENCY_1WS

#elif STM32_HCLK <= STM32_2WS_THRESHOLD
  #define STM32_FLASHBITS           FLASH_ACR_LATENCY_2WS

#elif STM32_HCLK <= STM32_3WS_THRESHOLD
  #define STM32_FLASHBITS           FLASH_ACR_LATENCY_3WS

#elif STM32_HCLK <= STM32_4WS_THRESHOLD
  #define STM32_FLASHBITS           FLASH_ACR_LATENCY_4WS

#else
  #define STM32_FLASHBITS           FLASH_ACR_LATENCY_5WS
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Type of a clock configuration structure.
 */
typedef struct {
  uint32_t          pwr_cr1;
  uint32_t          pwr_cr2;
  uint32_t          pwr_cr5;
  uint32_t          rcc_cr;
  uint32_t          rcc_cfgr;
  uint32_t          rcc_pllcfgr;
  uint32_t          flash_acr;
  uint32_t          rcc_crrcr;
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
  ((clkpt) == CLK_HSI16     ? STM32_HSI16CLK        :                       \
   (clkpt) == CLK_HSI48     ? STM32_HSI48CLK        :                       \
   (clkpt) == CLK_HSE       ? STM32_HSECLK          :                       \
   (clkpt) == CLK_SYSCLK    ? STM32_SYSCLK          :                       \
   (clkpt) == CLK_PLLPCLK   ? STM32_PLL_P_CLKOUT    :                       \
   (clkpt) == CLK_PLLQCLK   ? STM32_PLL_Q_CLKOUT    :                       \
   (clkpt) == CLK_PLLRCLK   ? STM32_PLL_R_CLKOUT    :                       \
   (clkpt) == CLK_HCLK      ? STM32_HCLK            :                       \
   (clkpt) == CLK_PCLK1     ? STM32_PCLK1           :                       \
   (clkpt) == CLK_PCLK1TIM  ? STM32_TIMP1CLK        :                       \
   (clkpt) == CLK_PCLK2     ? STM32_PCLK2           :                       \
   (clkpt) == CLK_PCLK2TIM  ? STM32_TIMP2CLK        :                       \
   (clkpt) == CLK_MCO       ? STM32_MCOCLK          :                        \
   0U)
#endif /* !defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

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
