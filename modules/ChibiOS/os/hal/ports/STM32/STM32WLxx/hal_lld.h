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
 * @file    STM32WLxx/hal_lld.h
 * @brief   STM32WLxx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32WL55xx, STM32WL54xx.
 *          - STM32WLE5xx, STM32WLE4xx.
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
#if defined(STM32WLE5xx) || defined(STM32WLE4xx) || \
    defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32WLExx Ultra-low power multi-modulation wireless microcontrollers"
#elif defined(STM32WL55xx) || defined(STM32WL54xx) || \
      defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32WL5xx Ultra-low power multi-modulation wireless microcontrollers, dual core"
#else
#error "STM32WLxx device not specified"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(STM32WLXX) || defined(__DOXYGEN__)
#define STM32WLXX
#endif
/** @} */

/**
 * @name   Clock points names
 * @{
 */
#define CLK_SYSCLK              0U
#define CLK_PLLPCLK             1U
#define CLK_PLLQCLK             2U
#define CLK_PLLRCLK             3U
#define CLK_HCLK                4U
#define CLK_PCLK1               5U
#define CLK_PCLK1TIM            6U
#define CLK_HCLK2               7U
#define CLK_PCLK2               8U
#define CLK_PCLK2TIM            9U
#define CLK_HCLK3               10U
#define CLK_MCO                 11U
#define CLK_ARRAY_SIZE          12U
/** @} */

/**
 * @name    HSE32 clock sources
 * @{
 */
#define STM32_HSE32_XTAL        0U          /**< External crystal.          */
#define STM32_HSE32_TCXO        1U          /**< TCXO.                      */
#define STM32_HSE32_EXTS        2U          /**< External source.           */
/** @} */

/**
 * @name    PWR_CR1 register bits definitions
 * @{
 */
#define STM32_VOS_MASK          (3U << 9)   /**< Core voltage mask.         */
#define STM32_VOS_RANGE1        (1U << 9)   /**< Core voltage 1.2 Volts.    */
#define STM32_VOS_RANGE2        (2U << 9)   /**< Core voltage 1.0 Volts.    */
/** @} */

/* Some ST headers do not have this definition.*/
#if !defined(PWR_CR2_PLS_LVL0)
#define PWR_CR2_PLS_LVL0        (0x0UL << PWR_CR2_PLS_Pos)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(PWR_CR2_PLS_LVL1)
#define PWR_CR2_PLS_LVL1        (0x1UL << PWR_CR2_PLS_Pos)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(PWR_CR2_PLS_LVL2)
#define PWR_CR2_PLS_LVL2        (0x2UL << PWR_CR2_PLS_Pos)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(PWR_CR2_PLS_LVL3)
#define PWR_CR2_PLS_LVL3        (0x3UL << PWR_CR2_PLS_Pos)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(PWR_CR2_PLS_LVL4)
#define PWR_CR2_PLS_LVL4        (0x4UL << PWR_CR2_PLS_Pos)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(PWR_CR2_PLS_LVL5)
#define PWR_CR2_PLS_LVL5        (0x5UL << PWR_CR2_PLS_Pos)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(PWR_CR2_PLS_LVL6)
#define PWR_CR2_PLS_LVL6        (0x6UL << PWR_CR2_PLS_Pos)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(PWR_CR2_PLS_LVL7)
#define PWR_CR2_PLS_LVL7        (0x7UL << PWR_CR2_PLS_Pos)
#endif

/**
 * @name    PWR_CR4 register bits definitions
 * @{
 */
#define STM32_VBRS_MASK         (1U << 9)   /**< VBRS field mask.           */
#define STM32_VBRS_RES_5K       (0U << 9)   /**< VBRS 5k resistor.          */
#define STM32_VBRS_RES_1K5      (1U << 9)   /**< VBRS 1.5k resistor.        */
#define STM32_VBE_MASK          (1U << 8)   /**< VBE field mask.            */
#define STM32_VBE_RESET         (0U << 8)   /**< VBE reset.                 */
#define STM32_VBE_SET           (1U << 8)   /**< VBE set.                   */
/** @} */

/**
 * @name    RCC_CR register bits definitions
 * @{
 */
#define STM32_HSE32PRE_MASK     (1U << 20)  /**< HSEPRE field mask.         */
#define STM32_HSE32PRE_DIV1     (0U << 20)  /**< HSE divided by 1.          */
#define STM32_HSE32PRE_DIV2     (1U << 20)  /**< HSE divided by 2.          */
/** @} */

/**
 * @name    RCC_CFGR register bits definitions
 * @{
 */
#define STM32_SW_MASK           (3U << 0)   /**< SW field mask.             */
#define STM32_SW_MSI            (0U << 0)   /**< SYSCLK source is MSI.      */
#define STM32_SW_HSI16          (1U << 0)   /**< SYSCLK source is HSI.      */
#define STM32_SW_HSE            (2U << 0)   /**< SYSCLK source is HSE.      */
#define STM32_SW_PLL            (3U << 0)   /**< SYSCLK source is PLL.      */

#define STM32_STOPWUCK_MASK     (1U << 15)  /**< STOPWUCK field mask.       */
#define STM32_STOPWUCK_MSI      (0U << 15)  /**< Wakeup clock is MSI.       */
#define STM32_STOPWUCK_HSI16    (1U << 15)  /**< Wakeup clock is HSI16.     */

#define STM32_MCOSEL_MASK       (15U << 24) /**< MCOSEL field mask.         */
#define STM32_MCOSEL_NOCLOCK    (0U << 24)  /**< No clock on MCO pin.       */
#define STM32_MCOSEL_SYSCLK     (1U << 24)  /**< SYSCLK on MCO pin.         */
#define STM32_MCOSEL_MSI        (2U << 24)  /**< MSI clock on MCO pin.      */
#define STM32_MCOSEL_HSI16      (3U << 24)  /**< HSI16 clock on MCO pin.    */
#define STM32_MCOSEL_HSE32      (4U << 24)  /**< HSE32 clock on MCO pin.    */
#define STM32_MCOSEL_PLLRCLK    (5U << 24)  /**< PLLR clock on MCO pin.     */
#define STM32_MCOSEL_LSI        (6U << 24)  /**< LSI clock on MCO pin.      */
#define STM32_MCOSEL_LSE        (8U << 24)  /**< LSE clock on MCO pin.      */
#define STM32_MCOSEL_PLLPCLK    (13U << 24) /**< PLLP clock on MCO pin.     */
#define STM32_MCOSEL_PLLQCLK    (14U << 24) /**< PLLQ clock on MCO pin.     */

#define STM32_MCOPRE_MASK       (7U << 28)  /**< MCOPRE field mask.         */
#define STM32_MCOPRE_DIV1       (0U << 28)  /**< MCO divided by 1.          */
#define STM32_MCOPRE_DIV2       (1U << 28)  /**< MCO divided by 2.          */
#define STM32_MCOPRE_DIV4       (2U << 28)  /**< MCO divided by 4.          */
#define STM32_MCOPRE_DIV8       (3U << 28)  /**< MCO divided by 8.          */
#define STM32_MCOPRE_DIV16      (4U << 28)  /**< MCO divided by 16.         */
/** @} */

/* Some ST headers do not have this definition.*/
#if !defined(RCC_CFGR_SW_MSI)
#define RCC_CFGR_SW_MSI         (0x00000000UL)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(RCC_CFGR_SW_HSI)
#define RCC_CFGR_SW_HSI         (0x00000001UL)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(RCC_CFGR_SW_HSE)
#define RCC_CFGR_SW_HSE         (0x00000002UL)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(RCC_CFGR_SW_PLL)
#define RCC_CFGR_SW_PLL         (0x00000003UL)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(RCC_CFGR_SWS_MSI)
#define RCC_CFGR_SWS_MSI        (0x00000000UL)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(RCC_CFGR_SWS_HSI)
#define RCC_CFGR_SWS_HSI        (0x00000004UL)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(RCC_CFGR_SWS_HSE)
#define RCC_CFGR_SWS_HSE        (0x00000008UL)
#endif

/* Some ST headers do not have this definition.*/
#if !defined(RCC_CFGR_SWS_PLL)
#define RCC_CFGR_SWS_PLL        (0x0000000CUL)
#endif

/**
 * @name    RCC_PLLCFGR register bits definitions
 * @{
 */
#define STM32_PLLSRC_MASK       (3U << 0)   /**< PLL clock source mask.     */
#define STM32_PLLSRC_NOCLOCK    (0U << 0)   /**< PLL clock source disabled. */
#define STM32_PLLSRC_MSI        (1U << 0)   /**< PLL clock source is MSI.   */
#define STM32_PLLSRC_HSI16      (2U << 0)   /**< PLL clock source is HSI16. */
#define STM32_PLLSRC_HSE        (3U << 0)   /**< PLL clock source is HSE.   */
/** @} */

/**
 * @name    RCC_CCIPR register bits definitions
 * @{
 */
#define STM32_USART1SEL_MASK    (3U << 0)   /**< USART1SEL mask.            */
#define STM32_USART1SEL_PCLK2   (0U << 0)   /**< USART1 source is PCLK2.    */
#define STM32_USART1SEL_SYSCLK  (1U << 0)   /**< USART1 source is SYSCLK.   */
#define STM32_USART1SEL_HSI16   (2U << 0)   /**< USART1 source is HSI16.    */
#define STM32_USART1SEL_LSE     (3U << 0)   /**< USART1 source is LSE.      */

#define STM32_USART2SEL_MASK    (3U << 2)   /**< USART2 mask.               */
#define STM32_USART2SEL_PCLK1   (0U << 2)   /**< USART2 source is PCLK1.    */
#define STM32_USART2SEL_SYSCLK  (1U << 2)   /**< USART2 source is SYSCLK.   */
#define STM32_USART2SEL_HSI16   (2U << 2)   /**< USART2 source is HSI16.    */
#define STM32_USART2SEL_LSE     (3U << 2)   /**< USART2 source is LSE.      */

#define STM32_SPI2S2SEL_MASK    (3U << 8)   /**< SPI2S2SEL mask.            */
#define STM32_SPI2S2SEL_PLLQCLK (1U << 8)   /**< SPI2S2 source is PLLQ.     */
#define STM32_SPI2S2SEL_HSI16   (2U << 8)   /**< SPI2S2 source is HSI16.    */
#define STM32_SPI2S2SEL_CKIN    (3U << 8)   /**< SPI2S2 source is External Input.*/

#define STM32_LPUART1SEL_MASK   (3U << 10)  /**< LPUART1 mask.              */
#define STM32_LPUART1SEL_PCLK1  (0U << 10)  /**< LPUART1 source is PCLK1.   */
#define STM32_LPUART1SEL_SYSCLK (1U << 10)  /**< LPUART1 source is SYSCLK.  */
#define STM32_LPUART1SEL_HSI16  (2U << 10)  /**< LPUART1 source is HSI16.   */
#define STM32_LPUART1SEL_LSE    (3U << 10)  /**< LPUART1 source is LSE.     */

#define STM32_I2C1SEL_MASK      (3U << 12)  /**< I2C1SEL mask.              */
#define STM32_I2C1SEL_PCLK1     (0U << 12)  /**< I2C1 source is PCLK1.      */
#define STM32_I2C1SEL_SYSCLK    (1U << 12)  /**< I2C1 source is SYSCLK.     */
#define STM32_I2C1SEL_HSI16     (2U << 12)  /**< I2C1 source is HSI16.      */

#define STM32_I2C2SEL_MASK      (3U << 14)  /**< I2C2SEL mask.              */
#define STM32_I2C2SEL_PCLK1     (0U << 14)  /**< I2C2 source is PCLK1.      */
#define STM32_I2C2SEL_SYSCLK    (1U << 14)  /**< I2C2 source is SYSCLK.     */
#define STM32_I2C2SEL_HSI16     (2U << 14)  /**< I2C2 source is HSI16.      */

#define STM32_I2C3SEL_MASK      (3U << 16)  /**< I2C3SEL mask.              */
#define STM32_I2C3SEL_PCLK1     (0U << 16)  /**< I2C3 source is PCLK1.      */
#define STM32_I2C3SEL_SYSCLK    (1U << 16)  /**< I2C3 source is SYSCLK.     */
#define STM32_I2C3SEL_HSI16     (2U << 16)  /**< I2C3 source is HSI16.      */

#define STM32_LPTIM1SEL_MASK    (3U << 18)  /**< LPTIM1SEL mask.            */
#define STM32_LPTIM1SEL_PCLK1   (0U << 18)  /**< LPTIM1 source is PCLK1.    */
#define STM32_LPTIM1SEL_LSI     (1U << 18)  /**< LPTIM1 source is LSI.      */
#define STM32_LPTIM1SEL_HSI16   (2U << 18)  /**< LPTIM1 source is HSI16.    */
#define STM32_LPTIM1SEL_LSE     (3U << 18)  /**< LPTIM1 source is LSE.      */

#define STM32_LPTIM2SEL_MASK    (3U << 20)  /**< LPTIM2SEL mask.            */
#define STM32_LPTIM2SEL_PCLK1   (0U << 20)  /**< LPTIM2 source is PCLK1.    */
#define STM32_LPTIM2SEL_LSI     (1U << 20)  /**< LPTIM2 source is LSI.      */
#define STM32_LPTIM2SEL_HSI16   (2U << 20)  /**< LPTIM2 source is HSI16.    */
#define STM32_LPTIM2SEL_LSE     (3U << 20)  /**< LPTIM2 source is LSE.      */

#define STM32_LPTIM3SEL_MASK    (3U << 22)  /**< LPTIM2SEL mask.            */
#define STM32_LPTIM3SEL_PCLK1   (0U << 22)  /**< LPTIM2 source is PCLK1.    */
#define STM32_LPTIM3SEL_LSI     (1U << 22)  /**< LPTIM2 source is LSI.      */
#define STM32_LPTIM3SEL_HSI16   (2U << 22)  /**< LPTIM2 source is HSI16.    */
#define STM32_LPTIM3SEL_LSE     (3U << 22)  /**< LPTIM2 source is LSE.      */

#define STM32_ADCSEL_MASK       (3U << 28)  /**< ADCSEL mask.               */
#define STM32_ADCSEL_NOCLK      (0U << 28)  /**< ADC clock disabled.        */
#define STM32_ADCSEL_HSI16      (1U << 28)  /**< ADC source is HSI16.       */
#define STM32_ADCSEL_PLLPCLK    (2U << 28)  /**< ADC source is PLL.         */
#define STM32_ADCSEL_SYSCLK     (3U << 28)  /**< ADC source is SYSCLK.      */

#define STM32_RNGSEL_MASK       (3U << 30)  /**< RNGSEL mask.               */
#define STM32_RNGSEL_PLLQCLK    (0U << 30)  /**< RNG source is PLL.         */
#define STM32_RNGSEL_LSI        (1U << 30)  /**< RNG source is LSI.         */
#define STM32_RNGSEL_LSE        (2U << 30)  /**< RNG source is LSE.         */
#define STM32_RNGSEL_MSI        (3U << 30)  /**< RNG source is MSI.         */
/** @} */

/**
 * @name    RCC_BDCR register bits definitions
 * @{
 */
#define STM32_RTCSEL_MASK       (3U << 8)   /**< RTC source mask.           */
#define STM32_RTCSEL_NOCLOCK    (0U << 8)   /**< No RTC source.             */
#define STM32_RTCSEL_LSE        (1U << 8)   /**< RTC source is LSE.         */
#define STM32_RTCSEL_LSI        (2U << 8)   /**< RTC source is LSI.         */
#define STM32_RTCSEL_HSE32DIV   (3U << 8)   /**< RTC source is HSE divided. */

#define STM32_LSCOSEL_MASK      (3U << 24)  /**< LSCO pin clock source.     */
#define STM32_LSCOSEL_NOCLOCK   (0U << 24)  /**< No clock on LSCO pin.      */
#define STM32_LSCOSEL_LSI       (1U << 24)  /**< LSI on LSCO pin.           */
#define STM32_LSCOSEL_LSE       (3U << 24)  /**< LSE on LSCO pin.           */
/** @} */

/**
 * @brief   Mapping SUBGHZSPI on SPI3.
 */
#define SPI3                    SUBGHZSPI

/**
 * @brief   Mapping ADC on ADC1.
 */
#define ADC1                    ADC

/**
 * @brief   Mapping ADC_COMMON on ADC1_COMMON.
 */
#define ADC1_COMMON             ADC_COMMON

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
 * @brief   PWR CR2 register initialization value.
 */
#if !defined(STM32_PWR_CR2) || defined(__DOXYGEN__)
#define STM32_PWR_CR2                       (PWR_CR2_PLS_LVL0)
#endif

/**
 * @brief   PWR CR3 register initialization value.
 */
#if !defined(STM32_PWR_CR3) || defined(__DOXYGEN__)
#define STM32_PWR_CR3                       (PWR_CR3_EWRFBUSY)
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
#if !defined(STM32_PWR_PUCRH) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRH                     (0U)
#endif

/**
 * @brief   PWR PDCRD register initialization value.
 */
#if !defined(STM32_PWR_PDCRH) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRH                     (0U)
#endif

/**
 * @brief   Enables or disables the HSI16 clock source.
 */
#if !defined(STM32_HSI16_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI16_ENABLED                 FALSE
#endif

/**
 * @brief   Enables or disables the LSI clock source.
 */
#if !defined(STM32_LSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSI_ENABLED                   TRUE
#endif

/**
 * @brief   LSI prescaler value.
 */
#if !defined(STM32_LSIPRE) || defined(__DOXYGEN__)
#define STM32_LSIPRE                        STM32_LSIPRE_NODIV
#endif

/**
 * @brief   Enables or disables the HSE32 clock source.
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
 * @brief   MSI frequency setting after standby.
 */
#if !defined(STM32_MSISRANGE) || defined(__DOXYGEN__)
#define STM32_MSISRANGE                     STM32_MSISRANGE_4M
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
 * @note    The default value is calculated for a 48MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                            STM32_SW_PLL
#endif

/**
 * @brief   Clock source for the PLL.
 * @note    This setting has only effect if the PLL is selected as the
 *          system clock source.
 * @note    The default value is calculated for a 48MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_PLLSRC) || defined(__DOXYGEN__)
#define STM32_PLLSRC                        STM32_PLLSRC_MSI
#endif

/**
 * @brief   PLLM divider value.
 * @note    The allowed values are 1..8.
 * @note    The default value is calculated for a 48MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_PLLM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLM_VALUE                    1U
#endif

/**
 * @brief   PLLN multiplier value.
 * @note    The allowed values are 6..127.
 * @note    The default value is calculated for a 48MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_PLLN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLN_VALUE                    24U
#endif

/**
 * @brief   PLLP divider value.
 * @note    The allowed values are 2..32.
 * @note    The default value is calculated for a 48MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_PLLP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLP_VALUE                    2U
#endif

/**
 * @brief   PLLQ divider value.
 * @note    The allowed values are 2..8.
 * @note    The default value is calculated for a 48MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_PLLQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLQ_VALUE                    2U
#endif

/**
 * @brief   PLLR divider value.
 * @note    The allowed values are 2..8.
 * @note    The default value is calculated for a 48MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_PLLR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLR_VALUE                    2U
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
#define STM32_LSCOSEL                       STM32_LSCOSEL_LSI
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
 * @brief   LPUART1 clock source.
 */
#if !defined(STM32_LPUART1SEL) || defined(__DOXYGEN__)
#define STM32_LPUART1SEL                    STM32_LPUART1SEL_SYSCLK
#endif

/**
 * @brief   I2C1 clock source.
 */
#if !defined(STM32_I2C1SEL) || defined(__DOXYGEN__)
#define STM32_I2C1SEL                       STM32_I2C1SEL_SYSCLK
#endif

/**
 * @brief   I2C2 clock source.
 */
#if !defined(STM32_I2C2SEL) || defined(__DOXYGEN__)
#define STM32_I2C2SEL                       STM32_I2C2SEL_SYSCLK
#endif

/**
 * @brief   I2C3 clock source.
 */
#if !defined(STM32_I2C3SEL) || defined(__DOXYGEN__)
#define STM32_I2C3SEL                       STM32_I2C3SEL_SYSCLK
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
 * @brief   LPTIM3 clock source.
 */
#if !defined(STM32_LPTIM3SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM3SEL                     STM32_LPTIM3SEL_PCLK1
#endif

/**
 * @brief   ADCSEL value (ADCs clock source).
 */
#if !defined(STM32_ADCSEL) || defined(__DOXYGEN__)
#define STM32_ADCSEL                        STM32_ADCSEL_SYSCLK
#endif

/**
 * @brief   SPI2S2SEL value (SPI2S2s clock source).
 */
#if !defined(STM32_SPI2S2SEL) || defined(__DOXYGEN__)
#define STM32_SPI2S2SEL                     STM32_SPI2S2SEL_CKIN
#endif

/**
 * @brief   RTC clock source.
 */
#if !defined(STM32_RTCSEL) || defined(__DOXYGEN__)
#define STM32_RTCSEL                        STM32_RTCSEL_LSI
#endif

/**
 * @brief   RNG clock source.
 */
#if !defined(STM32_RNGSEL) || defined(__DOXYGEN__)
#define STM32_RNGSEL                        STM32_RNGSEL_PLLQCLK
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
#if !defined(STM32WLxx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32WLxx_MCUCONF not defined"
#endif

/*
 * Target core checks.
 */
#if !defined(STM32_TARGET_CORE)
#error "STM32_TARGET_CORE not defined in mcuconf.h"
#endif

#if (STM32_TARGET_CORE == 2) && (STM32_HAS_M0 == FALSE)
#error "Cortex-M0 core not present in the selected device."
#endif

#if (defined(CORE_CM0PLUS) && (STM32_TARGET_CORE == 1)) || \
    (!defined(CORE_CM0PLUS) && (STM32_TARGET_CORE == 2))
#error "wrong target core specified in mcuconf.h"
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

/* PLL multipliers/dividers limits*/
/**
 * @name    PLL limits
 * @{
 */

/**
 * @brief   Maximum PLLM value.
 */
#define STM32_PLLM_VALUE_MAX        8

/**
 * @brief   Minimum PLLM value.
 */
#define STM32_PLLM_VALUE_MIN        1

/**
 * @brief   Maximum PLLN value.
 */
#define STM32_PLLN_VALUE_MAX        127

/**
 * @brief   Minimum PLLN value.
 */
#define STM32_PLLN_VALUE_MIN        6

/**
 * @brief   Maximum PLLR value.
 */
#define STM32_PLLR_VALUE_MAX        8

/**
 * @brief   Minimum PLLR value.
 */
#define STM32_PLLR_VALUE_MIN        2

/**
 * @brief   Maximum PLLQ value.
 */
#define STM32_PLLQ_VALUE_MAX        8

/**
 * @brief   Minimum PLLQ value.
 */
#define STM32_PLLQ_VALUE_MIN        2

/**
 * @brief   Maximum PLLP value.
 */
#define STM32_PLLP_VALUE_MAX        32

/**
 * @brief   Minimum PLLP value.
 */
#define STM32_PLLP_VALUE_MIN        2

/** @} */

/**
 * @brief   Low Power Run mode sysclk limit.
 */
#define STM32_LPRUN_SYSCLK_MAX      2000000U

/**
 * @brief   Range 1 Voltage related limits.
 * @{
 */
#define STM32_VOS1_SYSCLK_MAX       48000000U
#define STM32_VOS1_LSECLK_MAX       32768U
#define STM32_VOS1_LSECLK_BYP_MAX   1000000U
#define STM32_VOS1_LSECLK_MIN       32768U
#define STM32_VOS1_LSECLK_BYP_MIN   32768U
#define STM32_VOS1_PLLIN_MAX        16000000U
#define STM32_VOS1_PLLIN_MIN        2660000U
#define STM32_VOS1_PLLVCO_MAX       344000000U
#define STM32_VOS1_PLLVCO_MIN       95950000U /*Tolerance for MSIPLL real frequency.*/
#define STM32_VOS1_PLLP_MAX         48000000U
#define STM32_VOS1_PLLP_MIN         3000000U
#define STM32_VOS1_PLLQ_MAX         48000000U
#define STM32_VOS1_PLLQ_MIN         12000000U
#define STM32_VOS1_PLLR_MAX         48000000U
#define STM32_VOS1_PLLR_MIN         12000000U
#define STM32_VOS1_PCLK1_MAX        80000000U
#define STM32_VOS1_PCLK2_MAX        80000000U
#define STM32_VOS1_ADCCLK_MAX       48000000U
#define STM32_VOS1_0WS_THRESHOLD    18000000U
#define STM32_VOS1_1WS_THRESHOLD    36000000U
#define STM32_VOS1_2WS_THRESHOLD    48000000U
/** @} */

/**
 * @brief   Range 2 Voltage related limits.
 * @{
 */
#define STM32_VOS2_SYSCLK_MAX       16000000U
#define STM32_VOS2_LSECLK_MAX       32768U
#define STM32_VOS2_LSECLK_BYP_MAX   1000000U
#define STM32_VOS2_LSECLK_MIN       32768U
#define STM32_VOS2_LSECLK_BYP_MIN   32768U
#define STM32_VOS2_PLLIN_MAX        16000000U
#define STM32_VOS2_PLLIN_MIN        2660000U
#define STM32_VOS2_PLLVCO_MAX       128000000U
#define STM32_VOS2_PLLVCO_MIN       95950000U /*Tolerance for MSIPLL real frequency.*/
#define STM32_VOS2_PLLP_MAX         16000000U
#define STM32_VOS2_PLLP_MIN         3000000U
#define STM32_VOS2_PLLQ_MAX         12000000U
#define STM32_VOS2_PLLQ_MIN         16000000U
#define STM32_VOS2_PLLR_MAX         16000000U
#define STM32_VOS2_PLLR_MIN         12000000U
#define STM32_VOS2_PCLK1_MAX        16000000U
#define STM32_VOS2_PCLK2_MAX        16000000U
#define STM32_VOS2_ADCCLK_MAX       16000000U
#define STM32_VOS2_0WS_THRESHOLD    6000000U
#define STM32_VOS2_1WS_THRESHOLD    12000000U
#define STM32_VOS2_2WS_THRESHOLD    16000000U
/** @} */

/* Voltage related limits.*/
#if (STM32_VOS == STM32_VOS_RANGE1) || defined(__DOXYGEN__)
/**
 * @name    System Limits
 * @{
 */
/**
 * @brief   Maximum SYSCLK clock frequency at current voltage setting.
 */
#define STM32_SYSCLK_MAX            STM32_VOS1_SYSCLK_MAX

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_MAX            STM32_VOS1_LSECLK_MAX

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_BYP_MAX        STM32_VOS1_LSECLK_BYP_MAX

/**
 * @brief   Minimum LSE clock frequency.
 */
#define STM32_LSECLK_MIN            STM32_VOS1_LSECLK_MIN

/**
 * @brief   Minimum LSE clock frequency.
 */
#define STM32_LSECLK_BYP_MIN        STM32_VOS1_LSECLK_BYP_MIN

/**
 * @brief   Maximum PLLs input clock frequency.
 */
#define STM32_PLLIN_MAX             STM32_VOS1_PLLIN_MAX

/**
 * @brief   Minimum PLLs input clock frequency.
 */
#define STM32_PLLIN_MIN             STM32_VOS1_PLLIN_MIN

/**
 * @brief   Maximum VCO clock frequency at current voltage setting.
 */
#define STM32_PLLVCO_MAX            STM32_VOS1_PLLVCO_MAX

/**
 * @brief   Minimum VCO clock frequency at current voltage setting.
 */
#define STM32_PLLVCO_MIN            STM32_VOS1_PLLVCO_MIN

/**
 * @brief   Maximum PLL-P output clock frequency.
 */
#define STM32_PLLP_MAX              STM32_VOS1_PLLP_MAX

/**
 * @brief   Minimum PLL-P output clock frequency.
 */
#define STM32_PLLP_MIN              STM32_VOS1_PLLP_MIN

/**
 * @brief   Maximum PLL-Q output clock frequency.
 */
#define STM32_PLLQ_MAX              STM32_VOS1_PLLQ_MAX

/**
 * @brief   Minimum PLL-Q output clock frequency.
 */
#define STM32_PLLQ_MIN              STM32_VOS1_PLLQ_MIN

/**
 * @brief   Maximum PLL-R output clock frequency.
 */
#define STM32_PLLR_MAX              STM32_VOS1_PLLR_MAX

/**
 * @brief   Minimum PLL-R output clock frequency.
 */
#define STM32_PLLR_MIN              STM32_VOS1_PLLR_MIN

/**
 * @brief   Maximum APB1 clock frequency.
 */
#define STM32_PCLK1_MAX             STM32_VOS1_PCLK1_MAX

/**
 * @brief   Maximum APB2 clock frequency.
 */
#define STM32_PCLK2_MAX             STM32_VOS1_PCLK2_MAX

/**
 * @brief   Maximum ADC clock frequency.
 */
#define STM32_ADCCLK_MAX            STM32_VOS1_ADCCLK_MAX

/**
 * @name    Flash Wait states
 * @{
 */
#define STM32_0WS_THRESHOLD         STM32_VOS1_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD         STM32_VOS1_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD         STM32_VOS1_2WS_THRESHOLD
/** @} */

#elif STM32_VOS == STM32_VOS_RANGE2
#define STM32_SYSCLK_MAX            STM32_VOS2_SYSCLK_MAX
#define STM32_LSECLK_MAX            STM32_VOS2_LSECLK_MAX
#define STM32_LSECLK_BYP_MAX        STM32_VOS2_LSECLK_BYP_MAX
#define STM32_LSECLK_MIN            STM32_VOS2_LSECLK_MIN
#define STM32_LSECLK_BYP_MIN        STM32_VOS2_LSECLK_BYP_MIN
#define STM32_PLLIN_MAX             STM32_VOS2_PLLIN_MAX
#define STM32_PLLIN_MIN             STM32_VOS2_PLLIN_MIN
#define STM32_PLLVCO_MAX            STM32_VOS2_PLLVCO_MAX
#define STM32_PLLVCO_MIN            STM32_VOS2_PLLVCO_MIN
#define STM32_PLLP_MAX              STM32_VOS2_PLLP_MAX
#define STM32_PLLP_MIN              STM32_VOS2_PLLP_MIN
#define STM32_PLLQ_MAX              STM32_VOS2_PLLQ_MAX
#define STM32_PLLQ_MIN              STM32_VOS2_PLLQ_MIN
#define STM32_PLLR_MAX              STM32_VOS2_PLLR_MAX
#define STM32_PLLR_MIN              STM32_VOS2_PLLR_MIN
#define STM32_PCLK1_MAX             STM32_VOS2_PCLK1_MAX
#define STM32_PCLK2_MAX             STM32_VOS2_PCLK2_MAX
#define STM32_ADCCLK_MAX            STM32_VOS2_ADCCLK_MAX

#define STM32_0WS_THRESHOLD         STM32_VOS2_0WS_THRESHOLD
#define STM32_1WS_THRESHOLD         STM32_VOS2_1WS_THRESHOLD
#define STM32_2WS_THRESHOLD         STM32_VOS2_2WS_THRESHOLD

#else
#error "invalid STM32_VOS value specified"
#endif

/* Clock handlers.*/
#include "stm32_bd.inc"
#include "stm32_lse.inc"
#include "stm32_lsi.inc"
#include "stm32_msi.inc"
#include "stm32_hsi16.inc"
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
    ((STM32_MCOSEL == STM32_MCOSEL_PLLRCLK ||                               \
      STM32_MCOSEL == STM32_MCOSEL_PLLPCLK ||                               \
      STM32_MCOSEL == STM32_MCOSEL_PLLQCLK) &&                              \
     (STM32_PLLSRC == STM32_PLLSRC_HSI16))
#error "HSI16 not enabled, required by STM32_MCOSEL"
#endif

#if (STM32_USART1SEL == STM32_USART1SEL_HSI16)
#error "HSI16 not enabled, required by STM32_USART1SEL"
#endif
#if (STM32_USART2SEL == STM32_USART2SEL_HSI16)
#error "HSI16 not enabled, required by STM32_USART2SEL"
#endif
#if (STM32_LPUART1SEL == STM32_LPUART1SEL_HSI16)
#error "HSI16 not enabled, required by STM32_LPUART1SEL"
#endif

#if (STM32_I2C1SEL == STM32_I2C1SEL_HSI16)
#error "HSI16 not enabled, required by I2C1SEL"
#endif
#if (STM32_I2C2SEL == STM32_I2C2SEL_HSI16)
#error "HSI16 not enabled, required by I2C2SEL"
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
#if (STM32_LPTIM3SEL == STM32_LPTIM3SEL_HSI16)
#error "HSI16 not enabled, required by LPTIM3SEL"
#endif

#if (STM32_SPI2S2SEL == STM32_SPI2S2SEL_HSI16)
#error "HSI16 not enabled, required by SPI2S2SEL"
#endif

#if (STM32_STOPWUCK == STM32_STOPWUCK_HSI16)
#error "HSI16 not enabled, required by STM32_STOPWUCK"
#endif

#endif /* !STM32_HSI16_ENABLED */

/*
 * HSE32 related checks.
 */
#if !defined(STM32_HSE32SRC)
#error "STM32_HSE32SRC should be defined in mcuconf.h"
#endif

#if STM32_HSE32SRC != STM32_HSE32_XTAL &&                                   \
    STM32_HSE32SRC != STM32_HSE32_TCXO &&                                   \
    STM32_HSE32SRC != STM32_HSE32_EXTS
#error "STM32_HSE32SRC should be defined in mcuconf.h with correct value"
#endif

#if STM32_HSE32_ENABLED

  #if STM32_HSE32SRC == STM32_HSE32_TCXO
    #error "HSE32 TCXO should be controled using SUBGHZ Radio Driver"
  #endif

#else /* !STM32_HSE32_ENABLED */

  #if STM32_SW == STM32_SW_HSE
    #error "HSE32 not enabled, required by STM32_SW"
  #endif

  #if (STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSE)
    #error "HSE32 not enabled, required by STM32_SW and STM32_PLLSRC"
  #endif

  #if (STM32_MCOSEL == STM32_MCOSEL_HSE32) ||                               \
      ((STM32_MCOSEL == STM32_MCOSEL_PLLRCLK ||                             \
        STM32_MCOSEL == STM32_MCOSEL_PLLPCLK ||                             \
        STM32_MCOSEL == STM32_MCOSEL_PLLQCLK) &&                            \
       (STM32_PLLSRC == STM32_PLLSRC_HSE))
    #error "HSE32 not enabled, required by STM32_MCOSEL"
  #endif

  #if STM32_RTCSEL == STM32_RTCSEL_HSE32DIV
    #error "HSE32 not enabled, required by STM32_RTCSEL"
  #endif

#endif /* !STM32_HSE32_ENABLED */

/*
 * LSI related checks.
 */
#if STM32_LSI_ENABLED
#else /* !STM32_LSI_ENABLED */

  #if STM32_RTCSEL == STM32_RTCSEL_LSI
    #error "LSI not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_MCOSEL == STM32_MCOSEL_LSI
    #error "LSI not enabled, required by STM32_MCOSEL"
  #endif

  #if STM32_LSCOSEL == STM32_LSCOSEL_LSI
    #error "LSI not enabled, required by STM32_LSCOSEL"
  #endif

  #if STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSI
    #error "LSI not enabled, required by STM32_LPTIM1SEL"
  #endif
  #if STM32_LPTIM2SEL == STM32_LPTIM2SEL_LSI
    #error "LSI not enabled, required by STM32_LPTIM1SEL"
  #endif
  #if STM32_LPTIM3SEL == STM32_LPTIM3SEL_LSI
    #error "LSI not enabled, required by STM32_LPTIM1SEL"
  #endif

  #if STM32_RNGSEL == STM32_RNGSEL_LSI
    #error "LSI not enabled, required by STM32_RNGSEL"
  #endif

#endif /* !STM32_LSI_ENABLED */

/*
 * LSE related checks.
 */
#if STM32_LSE_ENABLED

  #if (STM32_LSECLK == 0)
    #error "LSE frequency not defined"
  #endif

  #if !defined(STM32_LSE_BYPASS) && ((STM32_LSECLK < STM32_LSECLK_MIN) || (STM32_LSECLK > STM32_LSECLK_MAX))
    #error "STM32_LSECLK outside acceptable range (STM32_LSECLK_MIN...STM32_LSECLK_MAX)"
  #endif

  #if defined(STM32_LSE_BYPASS) && ((STM32_LSECLK < STM32_LSECLK_BYP_MIN) || (STM32_LSECLK > STM32_LSECLK_BYP_MAX))
    #error "STM32_LSECLK outside acceptable range (STM32_LSECLK_BYP_MIN...STM32_LSECLK_BYP_MAX)"
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

  #if STM32_MSIPLL_ENABLED
    #error "LSE not enabled, required by STM32_MSIPLL_ENABLED"
  #endif

  #if (STM32_USART1SEL == STM32_USART1SEL_LSE)
    #error "LSE not enabled, required by STM32_USART1SEL"
  #endif
  #if (STM32_USART2SEL == STM32_USART2SEL_LSE)
    #error "LSE not enabled, required by STM32_USART2SEL"
  #endif
  #if (STM32_LPUART1SEL == STM32_LPUART1SEL_LSE)
    #error "LSE not enabled, required by STM32_LPUART1SEL"
  #endif

  #if STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSE
    #error "LSE not enabled, required by STM32_LPTIM1SEL"
  #endif
  #if STM32_LPTIM2SEL == STM32_LPTIM2SEL_LSE
    #error "LSE not enabled, required by STM32_LPTIM1SEL"
  #endif
  #if STM32_LPTIM3SEL == STM32_LPTIM3SEL_LSE
    #error "LSE not enabled, required by STM32_LPTIM1SEL"
  #endif

  #if STM32_RNGSEL == STM32_RNGSEL_LSE
    #error "LSE not enabled, required by STM32_RNGSEL"
  #endif

#endif /* !STM32_LSE_ENABLED */

/**
 * @brief   STM32_PLLM field.
 */
#if ((STM32_PLLM_VALUE >= 1) && (STM32_PLLM_VALUE <= 8)) ||                 \
    defined(__DOXYGEN__)
#define STM32_PLLM                  ((STM32_PLLM_VALUE - 1) << RCC_PLLCFGR_PLLM_Pos)
#else
#error "invalid STM32_PLLM_VALUE value specified"
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
 * PLL enable check.
 */
#if (STM32_SW == STM32_SW_PLL) ||                                           \
    (STM32_ADCSEL == STM32_ADCSEL_PLLPCLK) ||                               \
    (STM32_MCOSEL == STM32_MCOSEL_PLLRCLK) ||                               \
    (STM32_MCOSEL == STM32_MCOSEL_PLLPCLK) ||                               \
    (STM32_MCOSEL == STM32_MCOSEL_PLLQCLK) ||                               \
    (STM32_RNGSEL == STM32_RNGSEL_PLLQCLK) ||                               \
    (STM32_SPI2S2SEL == STM32_SPI2S2SEL_PLLQCLK) ||                         \
    defined(__DOXYGEN__)

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
#if (STM32_ADCSEL == STM32_ADCSEL_PLLPCLK) ||                               \
    (STM32_MCOSEL == STM32_MCOSEL_PLLPCLK) ||                               \
    defined(__DOXYGEN__)
#define STM32_PLLPEN                (1U << 16)
#else
#define STM32_PLLPEN                (0U << 16)
#endif

/**
 * @brief   STM32_PLLQEN field.
 */
#if (STM32_MCOSEL == STM32_MCOSEL_PLLQCLK) ||                               \
    (STM32_RNGSEL == STM32_RNGSEL_PLLQCLK) ||                               \
    (STM32_SPI2S2SEL == STM32_SPI2S2SEL_PLLQCLK) ||                         \
    defined(__DOXYGEN__)
#define STM32_PLLQEN                (1U << 24)
#else
#define STM32_PLLQEN                (0U << 24)
#endif

/**
 * @brief   STM32_PLLREN field.
 */
#if (STM32_SW == STM32_SW_PLL) ||                                           \
    (STM32_MCOSEL == STM32_MCOSEL_PLLRCLK) ||                               \
    defined(__DOXYGEN__)
#define STM32_PLLREN                (1U << 28)
#else
#define STM32_PLLREN                (0U << 28)
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

/*
 * CM0+ Core clock related checks.
 */
#if (STM32_HAS_M0 == TRUE)
#if !defined(STM32_C2HPRE)
#error "STM32_C2HPRE not defined in mcuconf.h"
#endif
#else /* (STM32_HAS_M0 == FALSE) */
#if defined(STM32_C2HPRE)
#error "STM32_C2HPRE should not be defined in mcuconf.h"
#else
#define STM32_HCLK2                 0U
#define STM32_C2HPRE                0UL
#endif
#endif /* (STM32_HAS_M0 == TRUE) */

/* Bus handlers.*/
#include "stm32_ahb.inc"
#include "stm32_ahb3.inc"
#include "stm32_apb1.inc"
#include "stm32_apb2.inc"

#if (STM32_HAS_M0 == TRUE)
#include "stm32_c2ahb.inc"
#endif

/**
 * @brief   Core 1 clock.
 */

#define STM32_CORE1_CK              STM32_HCLK

/**
 * @brief   Core 2 clock.
 */
#define STM32_CORE2_CK              STM32_HCLK2

#if STM32_TARGET_CORE == 1
#define STM32_CORE_CK               STM32_CORE1_CK
#elif STM32_TARGET_CORE == 2
#define STM32_CORE_CK               STM32_CORE2_CK
#else
#error "invalid STM32_TARGET_CORE value specified"
#endif

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

#elif STM32_MCOSEL == STM32_MCOSEL_HSE32
#define STM32_MCODIVCLK             STM32_HSE32CLK

#elif STM32_MCOSEL == STM32_MCOSEL_PLLRCLK
#define STM32_MCODIVCLK             STM32_PLL_R_CLKOUT

#elif STM32_MCOSEL == STM32_MCOSEL_PLLPCLK
#define STM32_MCODIVCLK             STM32_PLL_P_CLKOUT

#elif STM32_MCOSEL == STM32_MCOSEL_PLLQCLK
#define STM32_MCODIVCLK             STM32_PLL_Q_CLKOUT

#elif STM32_MCOSEL == STM32_MCOSEL_LSI
#define STM32_MCODIVCLK             STM32_LSICLK

#elif STM32_MCOSEL == STM32_MCOSEL_LSE
#define STM32_MCODIVCLK             STM32_LSECLK

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

#elif STM32_RTCSEL == STM32_RTCSEL_HSE32DIV
#define STM32_RTCCLK                (STM32_HSE32CLK / 32)

#else
#error "invalid STM32_RTCSEL value specified"
#endif

/**
 * @brief   USART1 clock frequency.
 */
#if (STM32_USART1SEL == STM32_USART1SEL_PCLK2) || defined(__DOXYGEN__)
#define STM32_USART1CLK             hal_lld_get_clock_point(CLK_PCLK2)
#elif STM32_USART1SEL == STM32_USART1SEL_SYSCLK
#define STM32_USART1CLK             hal_lld_get_clock_point(CLK_SYSCLK)
#elif STM32_USART1SEL == STM32_USART1SEL_HSI16
#define STM32_USART1CLK             STM32_HSI16CLK
#elif STM32_USART1SEL == STM32_USART1SEL_LSE
#define STM32_USART1CLK             STM32_LSECLK
#else
#error "invalid source selected for USART1 clock"
#endif

/**
 * @brief   USART2 clock frequency.
 */
#if (STM32_USART2SEL == STM32_USART2SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_USART2CLK             hal_lld_get_clock_point(CLK_PCLK1)
#elif STM32_USART2SEL == STM32_USART2SEL_SYSCLK
#define STM32_USART2CLK             hal_lld_get_clock_point(CLK_SYSCLK)
#elif STM32_USART2SEL == STM32_USART2SEL_HSI16
#define STM32_USART2CLK             STM32_HSI16CLK
#elif STM32_USART2SEL == STM32_USART2SEL_LSE
#define STM32_USART2CLK             STM32_LSECLK
#else
#error "invalid source selected for USART2 clock"
#endif

/**
 * @brief   LPUART1 clock frequency.
 */
#if (STM32_LPUART1SEL == STM32_LPUART1SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_LPUART1CLK            hal_lld_get_clock_point(CLK_PCLK1)
#elif STM32_LPUART1SEL == STM32_LPUART1SEL_SYSCLK
#define STM32_LPUART1CLK            hal_lld_get_clock_point(CLK_SYSCLK)
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
#define STM32_I2C1CLK               hal_lld_get_clock_point(CLK_PCLK1)
#elif STM32_I2C1SEL == STM32_I2C1SEL_SYSCLK
#define STM32_I2C1CLK               hal_lld_get_clock_point(CLK_SYSCLK)
#elif STM32_I2C1SEL == STM32_I2C1SEL_HSI16
#define STM32_I2C1CLK               STM32_HSI16CLK
#else
#error "invalid source selected for I2C1 clock"
#endif

/**
 * @brief   I2C2 clock frequency.
 */
#if (STM32_I2C2SEL == STM32_I2C2SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_I2C2CLK               hal_lld_get_clock_point(CLK_PCLK1)
#elif STM32_I2C2SEL == STM32_I2C2SEL_SYSCLK
#define STM32_I2C2CLK               hal_lld_get_clock_point(CLK_SYSCLK)
#elif STM32_I2C2SEL == STM32_I2C2SEL_HSI16
#define STM32_I2C2CLK               STM32_HSI16CLK
#else
#error "invalid source selected for I2C2 clock"
#endif

/**
 * @brief   I2C3 clock frequency.
 */
#if (STM32_I2C3SEL == STM32_I2C3SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_I2C3CLK               hal_lld_get_clock_point(CLK_PCLK1)
#elif STM32_I2C3SEL == STM32_I2C3SEL_SYSCLK
#define STM32_I2C3CLK               hal_lld_get_clock_point(CLK_SYSCLK)
#elif STM32_I2C3SEL == STM32_I2C3SEL_HSI16
#define STM32_I2C3CLK               STM32_HSI16CLK
#else
#error "invalid source selected for I2C3 clock"
#endif

/**
 * @brief   LPTIM1 clock frequency.
 */
#if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_LPTIM1CLK             hal_lld_get_clock_point(CLK_PCLK1)
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
#define STM32_LPTIM2CLK             hal_lld_get_clock_point(CLK_PCLK1)
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
 * @brief   LPTIM3 clock frequency.
 */
#if (STM32_LPTIM3SEL == STM32_LPTIM3SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_LPTIM3CLK             hal_lld_get_clock_point(CLK_PCLK1)
#elif STM32_LPTIM1SEL == STM32_LPTIM3SEL_LSI
#define STM32_LPTIM3CLK             STM32_LSICLK
#elif STM32_LPTIM1SEL == STM32_LPTIM3SEL_HSI16
#define STM32_LPTIM3CLK             STM32_HSI16CLK
#elif STM32_LPTIM1SEL == STM32_LPTIM3SEL_LSE
#define STM32_LPTIM3CLK             STM32_LSECLK
#else
#error "invalid source selected for LPTIM3 clock"
#endif

/**
 * @brief   RNG clock point.
 */
#if (STM32_RNGSEL == STM32_RNGSEL_PLLQCLK) || defined(__DOXYGEN__)
#define STM32_RNGCLK                hal_lld_get_clock_point(CLK_PLLQCLK)
#elif STM32_RNGSEL == STM32_RNGSEL_LSI
#define STM32_RNGCLK                STM32_LSICLK
#elif STM32_RNGSEL == STM32_RNGSEL_LSE
#define STM32_RNGCLK                STM32_LSECLK
#elif STM32_RNGSEL == STM32_RNGSEL_MSI
#define STM32_RNGCLK                STM32_HSI16CLK
#else
#error "invalid source selected for RNG clock"
#endif

/**
 * @brief   SPI2S2 clock frequency.
 */
#if (STM32_SPI2S2SEL == STM32_SPI2S2SEL_PLLQCLK) || defined(__DOXYGEN__)
#define STM32_SPI2S2CLK           hal_lld_get_clock_point(CLK_PLLQCLK)
#elif STM32_SPI2S2SEL == STM32_SPI2S2SEL_HSI16
#define STM32_SPI2S2CLK           STM32_HSI16CLK
#elif STM32_SPI2S2SEL == STM32_SPI2S2SEL_CKIN
#define STM32_SPI2S2CLK            0 /* Unknown, would require a board value */
#else
#error "invalid source selected for SPI2S2 clock"
#endif

/**
 * @brief   ADC clock frequency.
 */
#if (STM32_ADCSEL == STM32_ADCSEL_NOCLK) || defined(__DOXYGEN__)
#define STM32_ADCCLK                0
#elif STM32_ADCSEL == STM32_ADCSEL_HSI16
#define STM32_ADCCLK                STM32_HSI16CLK
#elif STM32_ADCSEL == STM32_ADCSEL_PLLPCLK
#define STM32_ADCCLK                hal_lld_get_clock_point(CLK__PLLPCLK)
#elif STM32_ADCSEL == STM32_ADCSEL_SYSCLK
#define STM32_ADCCLK                hal_lld_get_clock_point(CLK_SYSCLK)
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
 * @brief   Clock of timers connected to APB1
 */
#define STM32_TIMCLK1               hal_lld_get_clock_point(CLK_PCLK1TIM)

/**
 * @brief   Clock of timers connected to APB2.
 */
#define STM32_TIMCLK2               hal_lld_get_clock_point(CLK_PCLK2TIM)

/**
 * @brief   Flash settings.
 */
#if (STM32_HCLK3 <= STM32_0WS_THRESHOLD) || defined(__DOXYGEN__)
#define STM32_FLASHBITS             (0U)

#elif STM32_HCLK3 <= STM32_1WS_THRESHOLD
#define STM32_FLASHBITS             FLASH_ACR_LATENCY_0

#elif STM32_HCLK3 <= STM32_2WS_THRESHOLD
#define STM32_FLASHBITS             FLASH_ACR_LATENCY_1

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
  uint32_t          rcc_cr;
  uint32_t          rcc_cfgr;
  uint32_t          rcc_extcfgr;
  uint32_t          rcc_pllcfgr;
  uint32_t          flash_acr;
} halclkcfg_t;

/**
 * @brief   Type of a clock switch-only structure.
 */
typedef struct {
  uint32_t          pwr_cr1;
  uint32_t          rcc_cfgr;
  uint32_t          rcc_extcfgr;
  uint32_t          flash_acr;
} halclkswc_t;
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
  ((clkpt) == CLK_SYSCLK   ? STM32_SYSCLK        :                          \
   (clkpt) == CLK_PLLPCLK  ? STM32_PLL_P_CLKOUT  :                          \
   (clkpt) == CLK_PLLQCLK  ? STM32_PLL_Q_CLKOUT  :                          \
   (clkpt) == CLK_PLLRCLK  ? STM32_PLL_R_CLKOUT  :                          \
   (clkpt) == CLK_HCLK     ? STM32_HCLK          :                          \
   (clkpt) == CLK_PCLK1    ? STM32_PCLK1         :                          \
   (clkpt) == CLK_PCLK1TIM ? STM32_TIMP1CLK      :                          \
   (clkpt) == CLK_HCLK2    ? STM32_HCLK2         :                          \
   (clkpt) == CLK_PCLK2    ? STM32_PCLK2         :                          \
   (clkpt) == CLK_PCLK2TIM ? STM32_TIMP2CLK      :                          \
   (clkpt) == CLK_HCLK3    ? STM32_HCLK3         :                          \
   (clkpt) == CLK_MCO      ? STM32_MCOCLK        :                          \
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
