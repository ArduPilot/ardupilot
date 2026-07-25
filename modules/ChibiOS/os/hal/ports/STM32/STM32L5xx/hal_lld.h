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
 * @file    STM32L5xx/hal_lld.h
 * @brief   STM32L5xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32L4R5xx, STM32L4R7xx, STM32L4R9xx.
 *          - STM32L4S5xx, STM32L4S7xx, STM32L4S9xx.
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
 * @name    Platform identification
 * @{
 */
#if defined(STM32L552xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32L5 Ultra Low Power"

#elif defined(STM32L562xx)
#define PLATFORM_NAME           "STM32L5 Ultra Low Power with Crypto"

#else
#error "STM32L5 device not specified"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(STM32L5XX) || defined(__DOXYGEN__)
#define STM32L5XX
#endif
/** @} */

/**
 * @name    PWR_CR1 register bits definitions
 * @{
 */
#define STM32_VOS_MASK          (3 << 9)    /**< Core voltage mask.         */
#define STM32_VOS_RANGE0        (0 << 9)    /**< Core voltage 1.28 Volts.   */
#define STM32_VOS_RANGE1        (1 << 9)    /**< Core voltage 1.2 Volts.    */
#define STM32_VOS_RANGE2        (2 << 9)    /**< Core voltage 1.0 Volts.    */
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

#define STM32_STOPWUCK_MASK     (1 << 15)   /**< STOPWUCK field mask.       */
#define STM32_STOPWUCK_MSI      (0 << 15)   /**< Wakeup clock is MSI.       */
#define STM32_STOPWUCK_HSI16    (1 << 15)   /**< Wakeup clock is HSI16.     */

#define STM32_MCOSEL_MASK       (15 << 24)  /**< MCOSEL field mask.         */
#define STM32_MCOSEL_NOCLOCK    (0 << 24)   /**< No clock on MCO pin.       */
#define STM32_MCOSEL_SYSCLK     (1 << 24)   /**< SYSCLK on MCO pin.         */
#define STM32_MCOSEL_MSI        (2 << 24)   /**< MSI clock on MCO pin.      */
#define STM32_MCOSEL_HSI16      (3 << 24)   /**< HSI16 clock on MCO pin.    */
#define STM32_MCOSEL_HSE        (4 << 24)   /**< HSE clock on MCO pin.      */
#define STM32_MCOSEL_PLL        (5 << 24)   /**< PLL clock on MCO pin.      */
#define STM32_MCOSEL_LSI        (6 << 24)   /**< LSI clock on MCO pin.      */
#define STM32_MCOSEL_LSE        (7 << 24)   /**< LSE clock on MCO pin.      */
#define STM32_MCOSEL_HSI48      (8 << 24)   /**< HSI48 clock on MCO pin.    */

#define STM32_MCOPRE_MASK       (7 << 28)   /**< MCOPRE field mask.         */
#define STM32_MCOPRE_DIV1       (0 << 28)   /**< MCO divided by 1.          */
#define STM32_MCOPRE_DIV2       (1 << 28)   /**< MCO divided by 2.          */
#define STM32_MCOPRE_DIV4       (2 << 28)   /**< MCO divided by 4.          */
#define STM32_MCOPRE_DIV8       (3 << 28)   /**< MCO divided by 8.          */
#define STM32_MCOPRE_DIV16      (4 << 28)   /**< MCO divided by 16.         */
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

/**
 * @name    RCC_PLLSAI1CFGR register bits definitions
 * @{
 */
#define STM32_PLLSAI1SRC_MASK   (3 << 0)    /**< PLL clock source mask.     */
#define STM32_PLLSAI1SRC_NOCLOCK (0 << 0)   /**< PLL clock source disabled. */
#define STM32_PLLSAI1SRC_MSI    (1 << 0)    /**< PLL clock source is MSI.   */
#define STM32_PLLSAI1SRC_HSI16  (2 << 0)    /**< PLL clock source is HSI16. */
#define STM32_PLLSAI1SRC_HSE    (3 << 0)    /**< PLL clock source is HSE.   */
/** @} */

/**
 * @name    RCC_PLLSAI2CFGR register bits definitions
 * @{
 */
#define STM32_PLLSAI2SRC_MASK   (3 << 0)    /**< PLL clock source mask.     */
#define STM32_PLLSAI2SRC_NOCLOCK (0 << 0)   /**< PLL clock source disabled. */
#define STM32_PLLSAI2SRC_MSI    (1 << 0)    /**< PLL clock source is MSI.   */
#define STM32_PLLSAI2SRC_HSI16  (2 << 0)    /**< PLL clock source is HSI16. */
#define STM32_PLLSAI2SRC_HSE    (3 << 0)    /**< PLL clock source is HSE.   */
/** @} */

/**
 * @name    RCC_CCIPR1 register bits definitions
 * @{
 */
#define STM32_USART1SEL_MASK    (3 << 0)    /**< USART1SEL mask.            */
#define STM32_USART1SEL_PCLK2   (0 << 0)    /**< USART1 source is PCLK2.    */
#define STM32_USART1SEL_SYSCLK  (1 << 0)    /**< USART1 source is SYSCLK.   */
#define STM32_USART1SEL_HSI16   (2 << 0)    /**< USART1 source is HSI16.    */
#define STM32_USART1SEL_LSE     (3 << 0)    /**< USART1 source is LSE.      */

#define STM32_USART2SEL_MASK    (3 << 2)    /**< USART2 mask.               */
#define STM32_USART2SEL_PCLK1   (0 << 2)    /**< USART2 source is PCLK1.    */
#define STM32_USART2SEL_SYSCLK  (1 << 2)    /**< USART2 source is SYSCLK.   */
#define STM32_USART2SEL_HSI16   (2 << 2)    /**< USART2 source is HSI16.    */
#define STM32_USART2SEL_LSE     (3 << 2)    /**< USART2 source is LSE.      */

#define STM32_USART3SEL_MASK    (3 << 4)    /**< USART3 mask.               */
#define STM32_USART3SEL_PCLK1   (0 << 4)    /**< USART3 source is PCLK1.    */
#define STM32_USART3SEL_SYSCLK  (1 << 4)    /**< USART3 source is SYSCLK.   */
#define STM32_USART3SEL_HSI16   (2 << 4)    /**< USART3 source is HSI16.    */
#define STM32_USART3SEL_LSE     (3 << 4)    /**< USART3 source is LSE.      */

#define STM32_UART4SEL_MASK     (3 << 6)    /**< UART4 mask.                */
#define STM32_UART4SEL_PCLK1    (0 << 6)    /**< UART4 source is PCLK1.     */
#define STM32_UART4SEL_SYSCLK   (1 << 6)    /**< UART4 source is SYSCLK.    */
#define STM32_UART4SEL_HSI16    (2 << 6)    /**< UART4 source is HSI16.     */
#define STM32_UART4SEL_LSE      (3 << 6)    /**< UART4 source is LSE.       */

#define STM32_UART5SEL_MASK     (3 << 8)    /**< UART5 mask.                */
#define STM32_UART5SEL_PCLK1    (0 << 8)    /**< UART5 source is PCLK1.     */
#define STM32_UART5SEL_SYSCLK   (1 << 8)    /**< UART5 source is SYSCLK.    */
#define STM32_UART5SEL_HSI16    (2 << 8)    /**< UART5 source is HSI16.     */
#define STM32_UART5SEL_LSE      (3 << 8)    /**< UART5 source is LSE.       */

#define STM32_LPUART1SEL_MASK   (3 << 10)   /**< LPUART1 mask.              */
#define STM32_LPUART1SEL_PCLK1  (0 << 10)   /**< LPUART1 source is PCLK1.   */
#define STM32_LPUART1SEL_SYSCLK (1 << 10)   /**< LPUART1 source is SYSCLK.  */
#define STM32_LPUART1SEL_HSI16  (2 << 10)   /**< LPUART1 source is HSI16.   */
#define STM32_LPUART1SEL_LSE    (3 << 10)   /**< LPUART1 source is LSE.     */

#define STM32_I2C1SEL_MASK      (3 << 12)   /**< I2C1SEL mask.              */
#define STM32_I2C1SEL_PCLK1     (0 << 12)   /**< I2C1 source is PCLK1.      */
#define STM32_I2C1SEL_SYSCLK    (1 << 12)   /**< I2C1 source is SYSCLK.     */
#define STM32_I2C1SEL_HSI16     (2 << 12)   /**< I2C1 source is HSI16.      */

#define STM32_I2C2SEL_MASK      (3 << 14)   /**< I2C2SEL mask.              */
#define STM32_I2C2SEL_PCLK1     (0 << 14)   /**< I2C2 source is PCLK1.      */
#define STM32_I2C2SEL_SYSCLK    (1 << 14)   /**< I2C2 source is SYSCLK.     */
#define STM32_I2C2SEL_HSI16     (2 << 14)   /**< I2C2 source is HSI16.      */

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

#define STM32_LPTIM3SEL_MASK    (3 << 22)   /**< LPTIM3SEL mask.            */
#define STM32_LPTIM3SEL_PCLK1   (0 << 22)   /**< LPTIM3 source is PCLK1.    */
#define STM32_LPTIM3SEL_LSI     (1 << 22)   /**< LPTIM3 source is LSI.      */
#define STM32_LPTIM3SEL_HSI16   (2 << 22)   /**< LPTIM3 source is HSI16.    */
#define STM32_LPTIM3SEL_LSE     (3 << 22)   /**< LPTIM3 source is LSE.      */

#define STM32_FDCANSEL_MASK     (3 << 24)   /**< FDCANSEL mask.             */
#define STM32_FDCANSEL_HSE      (0 << 24)   /**< FDCAN source is HSE.       */
#define STM32_FDCANSEL_PLL      (1 << 24)   /**< FDCAN source is PLL-Q.     */
#define STM32_FDCANSEL_PLLSAI1  (2 << 24)   /**< FDCAN source is PLLSAI1-P. */

#define STM32_CLK48SEL_MASK     (3 << 26)   /**< CLK48SEL mask.             */
#define STM32_CLK48SEL_HSI48    (0 << 26)   /**< CLK48 source is HSI48.     */
#define STM32_CLK48SEL_PLLSAI1  (1 << 26)   /**< CLK48 source is PLLSAI1-Q. */
#define STM32_CLK48SEL_PLL      (2 << 26)   /**< CLK48 source is PLL-Q.     */
#define STM32_CLK48SEL_MSI      (3 << 26)   /**< CLK48 source is MSI.       */

#define STM32_ADCSEL_MASK       (3 << 28)   /**< ADCSEL mask.               */
#define STM32_ADCSEL_NOCLK      (0 << 28)   /**< ADC clock disabled.        */
#define STM32_ADCSEL_PLLSAI1    (1 << 28)   /**< ADC source is PLLSAI1-R.   */
#define STM32_ADCSEL_SYSCLK     (3 << 28)   /**< ADC source is SYSCLK.      */
/** @} */

/**
 * @name    RCC_CCIPR2 register bits definitions
 * @{
 */
#define STM32_I2C4SEL_MASK      (3 << 0)    /**< I2C1SEL mask.              */
#define STM32_I2C4SEL_PCLK1     (0 << 0)    /**< I2C1 source is PCLK1.      */
#define STM32_I2C4SEL_SYSCLK    (1 << 0)    /**< I2C1 source is SYSCLK.     */
#define STM32_I2C4SEL_HSI16     (2 << 0)    /**< I2C1 source is HSI16.      */

#define STM32_DFSDMSEL_MASK     (1 << 2)    /**< DFSDMSEL mask.             */
#define STM32_DFSDMSEL_PCLK2    (0 << 2)    /**< DFSDMSEL source is PCLK2.  */
#define STM32_DFSDMSEL_SYSCLK   (1 << 2)    /**< DFSDMSEL source is SYSCLK. */

#define STM32_ADFSDMSEL_MASK    (3 << 3)    /**< ADFSDMSEL mask.            */
#define STM32_ADFSDMSEL_SAI1CLK (0 << 3)    /**< ADFSDMSEL source is
                                                 SAI1CLK.                   */
#define STM32_ADFSDMSEL_HSI16   (1 << 3)    /**< ADFSDMSEL source is HSI16. */
#define STM32_ADFSDMSEL_MSI     (2 << 3)    /**< ADFSDMSEL source is MSI.   */

#define STM32_SAI1SEL_MASK      (7 << 5)    /**< SAI1SEL mask.              */
#define STM32_SAI1SEL_PLLSAI1   (0 << 5)    /**< SAI1 source is PLLSAI1CLK. */
#define STM32_SAI1SEL_PLLSAI2   (1 << 5)    /**< SAI1 source is PLLSAI2CLK. */
#define STM32_SAI1SEL_PLL       (2 << 5)    /**< SAI1 source is PLLSAI3CLK  */
#define STM32_SAI1SEL_EXTCLK    (3 << 5)    /**< SAI1 source is external.   */
#define STM32_SAI1SEL_HSI16     (4 << 5)    /**< SAI1 source is HSI16.      */
#define STM32_SAI1SEL_OFF       0xFFFFFFFFU /**< SAI1 clock is not required.*/

#define STM32_SAI2SEL_MASK      (7 << 8)    /**< SAI2SEL mask.              */
#define STM32_SAI2SEL_PLLSAI1   (0 << 8)    /**< SAI2 source is PLLSAI1CLK. */
#define STM32_SAI2SEL_PLLSAI2   (1 << 8)    /**< SAI2 source is PLLSAI2CLK. */
#define STM32_SAI2SEL_PLL       (2 << 8)    /**< SAI2 source is PLLSAI3CLK  */
#define STM32_SAI2SEL_EXTCLK    (3 << 8)    /**< SAI2 source is external.   */
#define STM32_SAI2SEL_HSI16     (4 << 8)    /**< SAI2 source is HSI16.      */
#define STM32_SAI2SEL_OFF       0xFFFFFFFFU /**< SAI2 clock is not required.*/

#define STM32_SDMMCSEL_MASK     (1 << 14)   /**< SDMMCSEL mask.             */
#define STM32_SDMMCSEL_48CLK    (0 << 14)   /**< SDMMCSEL source is 48CLK.  */
#define STM32_SDMMCSEL_PLL      (1 << 14)   /**< SDMMCSEL source is
                                                 PLLSAI3CLK.                */

#define STM32_OSPISEL_MASK      (3 << 20)    /**< OSPISEL mask.             */
#define STM32_OSPISEL_SYSCLK    (0 << 20)    /**< OSPI source is SYSCLK.    */
#define STM32_OSPISEL_MSI       (1 << 20)    /**< OSPI source is MSI.       */
#define STM32_OSPISEL_48CLK     (2 << 20)    /**< OSPI source is 48CLK.     */
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
 * @brief   RCC-related security settings.
 */
#if !defined(STM32_RCC_SECCFGR) || defined(__DOXYGEN__)
#define STM32_RCC_SECCFGR                   0
#endif

/**
 * @brief   Core voltage selection.
 * @note    This setting affects all the performance and clock related
 *          settings, the maximum performance is only obtainable selecting
 *          the maximum voltage.
 */
#if !defined(STM32_VOS) || defined(__DOXYGEN__)
#define STM32_VOS                           STM32_VOS_RANGE0
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
#define STM32_PWR_CR3                       (0U)
#endif

/**
 * @brief   PWR CR4 register initialization value.
 */
#if !defined(STM32_PWR_CR4) || defined(__DOXYGEN__)
#define STM32_PWR_CR4                       (0U)
#endif

/**
 * @brief   Main clock source selection.
 * @note    If the selected clock source is not the PLL then the PLL is not
 *          initialized and started.
 * @note    The default value is calculated for a 120MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                            STM32_SW_PLL
#endif

/**
 * @brief   Clock source for the PLL.
 * @note    This setting has only effect if the PLL is selected as the
 *          system clock source.
 * @note    The default value is calculated for a 120MHz system clock from
 *          the internal 4MHz MSI clock.
 */
#if !defined(STM32_PLLSRC) || defined(__DOXYGEN__)
#define STM32_PLLSRC                        STM32_PLLSRC_MSI
#endif

/**
 * @brief   Clock source for the PLLSAL1.
 */
#if !defined(STM32_PLLSAI1SRC) || defined(__DOXYGEN__)
#define STM32_PLLSAI1SRC                    STM32_PLLSAI1SRC_MSI
#endif

/**
 * @brief   Clock source for the PLLSAL2.
 */
#if !defined(STM32_PLLSAI2SRC) || defined(__DOXYGEN__)
#define STM32_PLLSAI2SRC                    STM32_PLLSAI2SRC_MSI
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
 * @brief   I2C4 clock source.
 */
#if !defined(STM32_I2C4SEL) || defined(__DOXYGEN__)
#define STM32_I2C4SEL                       STM32_I2C4SEL_SYSCLK
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
 * @brief   FDCAN value (48MHz clock source).
 */
#if !defined(STM32_FDCANSEL) || defined(__DOXYGEN__)
#define STM32_FDCANSEL                      STM32_FDCANSEL_PLL
#endif

/**
 * @brief   CLK48SEL value (48MHz clock source).
 */
#if !defined(STM32_CLK48SEL) || defined(__DOXYGEN__)
#define STM32_CLK48SEL                      STM32_CLK48SEL_PLL
#endif

/**
 * @brief   ADCSEL value (ADCs clock source).
 */
#if !defined(STM32_ADCSEL) || defined(__DOXYGEN__)
#define STM32_ADCSEL                        STM32_ADCSEL_SYSCLK
#endif

/**
 * @brief   DFSDMSEL value (DFSDM clock source).
 */
#if !defined(STM32_DFSDMSEL) || defined(__DOXYGEN__)
#define STM32_DFSDMSEL                      STM32_DFSDMSEL_PCLK2
#endif

/**
 * @brief   ADFSDMSEL value (DFSDM clock source).
 */
#if !defined(STM32_ADFSDMSEL) || defined(__DOXYGEN__)
#define STM32_ADFSDMSEL                     STM32_ADFSDMSEL_SAI1CLK
#endif

/**
 * @brief   SAI1SEL value (SAI1 clock source).
 */
#if !defined(STM32_SAI1SEL) || defined(__DOXYGEN__)
#define STM32_SAI1SEL                       STM32_SAI1SEL_OFF
#endif

/**
 * @brief   SAI2SEL value (SAI2 clock source).
 */
#if !defined(STM32_SAI2SEL) || defined(__DOXYGEN__)
#define STM32_SAI2SEL                       STM32_SAI2SEL_OFF
#endif

/**
 * @brief   SDMMC value (SDMMC clock source).
 */
#if !defined(STM32_SDMMCSEL) || defined(__DOXYGEN__)
#define STM32_SDMMCSEL                      STM32_SDMMCSEL_48CLK
#endif

/**
 * @brief   OSPISEL value (OSPISEL clock source).
 */
#if !defined(STM32_OSPISEL) || defined(__DOXYGEN__)
#define STM32_OSPISEL                       STM32_OSPISEL_SYSCLK
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
#if !defined(STM32L5xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L5xx_MCUCONF not defined"
#endif

#if defined(STM32L552xx) && !defined(STM32L552_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L552_MCUCONF not defined"

#endif

#if defined(STM32L562xx) && !defined(STM32L562_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L562_MCUCONF not defined"

#endif

/* Voltage related limits.*/
#if (STM32_VOS == STM32_VOS_RANGE0) || defined(__DOXYGEN__)
/**
 * @name    System Limits
 * @{
 */
/**
 * @brief   Maximum SYSCLK clock frequency.
 */
#define STM32_SYSCLK_MAX            110000000

/**
 * @brief   Maximum HSE clock frequency at current voltage setting.
 */
#define STM32_HSECLK_MAX            48000000

/**
 * @brief   Maximum HSE clock frequency using an external source.
 */
#define STM32_HSECLK_BYP_MAX        48000000

/**
 * @brief   Minimum HSE clock frequency.
 */
#define STM32_HSECLK_MIN            4000000

/**
 * @brief   Minimum HSE clock frequency using an external source.
 */
#define STM32_HSECLK_BYP_MIN        8000000

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_MAX            32768

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_BYP_MAX        1000000

/**
 * @brief   Minimum LSE clock frequency.
 */
#define STM32_LSECLK_MIN            32768

/**
 * @brief   Minimum LSE clock frequency.
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
#define STM32_PLLVCO_MIN            63950000 /*Tolerance for MSIPLL real frequency.*/

/**
 * @brief   Maximum PLL-P output clock frequency.
 */
#define STM32_PLLP_MAX              110000000

/**
 * @brief   Minimum PLL-P output clock frequency.
 */
#define STM32_PLLP_MIN              2064500

/**
 * @brief   Maximum PLL-Q output clock frequency.
 */
#define STM32_PLLQ_MAX              110000000

/**
 * @brief   Minimum PLL-Q output clock frequency.
 */
#define STM32_PLLQ_MIN              8000000

/**
 * @brief   Maximum PLL-R output clock frequency.
 */
#define STM32_PLLR_MAX              110000000

/**
 * @brief   Minimum PLL-R output clock frequency.
 */
#define STM32_PLLR_MIN              8000000

/**
 * @brief   Maximum APB1 clock frequency.
 */
#define STM32_PCLK1_MAX             110000000

/**
 * @brief   Maximum APB2 clock frequency.
 */
#define STM32_PCLK2_MAX             110000000

/**
 * @brief   Maximum ADC clock frequency.
 */
#define STM32_ADCCLK_MAX            80000000
/** @} */

/**
 * @name    Flash Wait states
 * @{
 */
#define STM32_0WS_THRESHOLD         20000000
#define STM32_1WS_THRESHOLD         40000000
#define STM32_2WS_THRESHOLD         60000000
#define STM32_3WS_THRESHOLD         80000000
#define STM32_4WS_THRESHOLD         100000000
#define STM32_5WS_THRESHOLD         110000000
/** @} */

#elif STM32_VOS == STM32_VOS_RANGE1
#define STM32_SYSCLK_MAX            80000000
#define STM32_HSECLK_MAX            48000000
#define STM32_HSECLK_BYP_MAX        48000000
#define STM32_HSECLK_MIN            4000000
#define STM32_HSECLK_BYP_MIN        8000000
#define STM32_LSECLK_MAX            32768
#define STM32_LSECLK_BYP_MAX        1000000
#define STM32_LSECLK_MIN            32768
#define STM32_LSECLK_BYP_MIN        32768
#define STM32_PLLIN_MAX             16000000
#define STM32_PLLIN_MIN             2660000
#define STM32_PLLVCO_MAX            344000000
#define STM32_PLLVCO_MIN            63950000 /*Tolerance for MSIPLL real frequency.*/
#define STM32_PLLP_MAX              110000000
#define STM32_PLLP_MIN              2064500
#define STM32_PLLQ_MAX              110000000
#define STM32_PLLQ_MIN              8000000
#define STM32_PLLR_MAX              110000000
#define STM32_PLLR_MIN              8000000
#define STM32_PCLK1_MAX             80000000
#define STM32_PCLK2_MAX             80000000
#define STM32_ADCCLK_MAX            80000000

#define STM32_0WS_THRESHOLD         20000000
#define STM32_1WS_THRESHOLD         40000000
#define STM32_2WS_THRESHOLD         60000000
#define STM32_3WS_THRESHOLD         80000000
#define STM32_4WS_THRESHOLD         0
#define STM32_5WS_THRESHOLD         0

#elif STM32_VOS == STM32_VOS_RANGE2
#define STM32_SYSCLK_MAX            26000000
#define STM32_HSECLK_MAX            26000000
#define STM32_HSECLK_BYP_MAX        26000000
#define STM32_HSECLK_MIN            4000000
#define STM32_HSECLK_BYP_MIN        8000000
#define STM32_LSECLK_MAX            32768
#define STM32_LSECLK_BYP_MAX        1000000
#define STM32_LSECLK_MIN            32768
#define STM32_LSECLK_BYP_MIN        32768
#define STM32_PLLIN_MAX             16000000
#define STM32_PLLIN_MIN             2660000
#define STM32_PLLVCO_MAX            128000000
#define STM32_PLLVCO_MIN            63950000 /*Tolerance for MSIPLL real frequency.*/
#define STM32_PLLP_MAX              26000000
#define STM32_PLLP_MIN              2064500
#define STM32_PLLQ_MAX              26000000
#define STM32_PLLQ_MIN              8000000
#define STM32_PLLR_MAX              26000000
#define STM32_PLLR_MIN              8000000
#define STM32_PCLK1_MAX             26000000
#define STM32_PCLK2_MAX             26000000
#define STM32_ADCCLK_MAX            26000000

#define STM32_0WS_THRESHOLD         8000000
#define STM32_1WS_THRESHOLD         16000000
#define STM32_2WS_THRESHOLD         26000000
#define STM32_3WS_THRESHOLD         0
#define STM32_4WS_THRESHOLD         0
#define STM32_5WS_THRESHOLD         0

#else
  #error "invalid STM32_VOS value specified"
#endif

/* ICache handler.*/
#include "stm32_icache.inc"

/* Clock handlers.*/
#include "stm32_lse.inc"
#include "stm32_lsi.inc"
#include "stm32_msi.inc"
#include "stm32_hsi16.inc"
#include "stm32_hsi48.inc"
#include "stm32_hse.inc"

/*
 * Platform HSI16-related checks.
 */
#if STM32_HSI16_ENABLED
#else /* !STM32_HSI16_ENABLED */

  #if STM32_SW == STM32_SW_HSI16
    #error "HSI16 not enabled, required by STM32_SW"
  #endif

  #if (STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSI16)
    #error "HSI16 not enabled, required by STM32_SW and STM32_PLLSRC"
  #endif

  /* MCO-related checks.*/
  #if (STM32_MCOSEL == STM32_MCOSEL_HSI16) ||                               \
      ((STM32_MCOSEL == STM32_MCOSEL_PLL) &&                                \
       (STM32_PLLSRC == STM32_PLLSRC_HSI16))
    #error "HSI16 not enabled, required by STM32_MCOSEL"
  #endif

  /* SAI1-related checks.*/
  #if STM32_SAI1SEL == STM32_SAI1SEL_HSI16
    #error "HSI16 not enabled, required by STM32_SAI1SEL"
  #endif

  #if (STM32_SAI1SEL == STM32_SAI1SEL_PLL) &&                               \
      (STM32_PLLSRC == STM32_PLLSRC_HSI16)
    #error "HSI16 not enabled, required by STM32_SAI1SEL"
  #endif

  #if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI1) &&                           \
      (STM32_PLLSAI1SRC == STM32_PLLSAI1SRC_HSI16)
    #error "HSI16 not enabled, required by STM32_PLLSAI1SRC"
   #endif

  #if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI2) &&                           \
      (STM32_PLLSAI2SRC == STM32_PLLSAI2SRC_HSI16)
    #error "HSI16 not enabled, required by STM32_PLLSAI2SRC"
  #endif

  /* SAI2-related checks.*/
  #if STM32_SAI2SEL == STM32_SAI2SEL_HSI16
    #error "HSI16 not enabled, required by STM32_SAI2SEL"
  #endif

  #if (STM32_SAI2SEL == STM32_SAI2SEL_PLL) &&                               \
      (STM32_PLLSRC == STM32_PLLSRC_HSI16)
    #error "HSI16 not enabled, required by STM32_SAI2SEL"
  #endif

  #if (STM32_SAI2SEL == STM32_SAI2SEL_PLLSAI1) &&                           \
      (STM32_PLLSAI1SRC == STM32_PLLSAI1SRC_HSI16)
    #error "HSI16 not enabled, required by STM32_PLLSAI1SRC"
  #endif

  #if (STM32_SAI2SEL == STM32_SAI2SEL_PLLSAI2) &&                           \
      (STM32_PLLSAI2SRC == STM32_PLLSAI2SRC_HSI16)
    #error "HSI16 not enabled, required by STM32_PLLSAI2SRC"
  #endif

  /* USART/UART-related checks.*/
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

  /* I2C-related checks.*/
  #if (STM32_I2C1SEL == STM32_I2C1SEL_HSI16)
    #error "HSI16 not enabled, required by I2C1SEL"
  #endif
  #if (STM32_I2C2SEL == STM32_I2C2SEL_HSI16)
    #error "HSI16 not enabled, required by I2C2SEL"
  #endif
  #if (STM32_I2C3SEL == STM32_I2C3SEL_HSI16)
    #error "HSI16 not enabled, required by I2C3SEL"
  #endif
  #if (STM32_I2C4SEL == STM32_I2C4SEL_HSI16)
    #error "HSI16 not enabled, required by I2C4SEL"
  #endif

  /* LPTIM-related checks.*/
  #if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_HSI16)
    #error "HSI16 not enabled, required by LPTIM1SEL"
  #endif
  #if (STM32_LPTIM2SEL == STM32_LPTIM2SEL_HSI16)
    #error "HSI16 not enabled, required by LPTIM2SEL"
  #endif
  #if (STM32_LPTIM3SEL == STM32_LPTIM3SEL_HSI16)
    #error "HSI16 not enabled, required by LPTIM3SEL"
  #endif

  #if (STM32_STOPWUCK == STM32_STOPWUCK_HSI16)
    #error "HSI16 not enabled, required by STM32_STOPWUCK"
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
 * Platform HSE-related checks.
 */
#if STM32_HSE_ENABLED
#else /* !STM32_HSE_ENABLED */

  #if STM32_SW == STM32_SW_HSE
    #error "HSE not enabled, required by STM32_SW"
  #endif

  #if (STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSE)
    #error "HSE not enabled, required by STM32_SW and STM32_PLLSRC"
  #endif

  /* MCO-related checks.*/
  #if (STM32_MCOSEL == STM32_MCOSEL_HSE) ||                                 \
      ((STM32_MCOSEL == STM32_MCOSEL_PLL) &&                                \
       (STM32_PLLSRC == STM32_PLLSRC_HSE))
    #error "HSE not enabled, required by STM32_MCOSEL"
  #endif

  /* SAI1-related checks.*/
  #if (STM32_SAI1SEL == STM32_SAI1SEL_PLL) &&                               \
      (STM32_PLLSRC == STM32_PLLSRC_HSE)
    #error "HSE not enabled, required by STM32_SAI1SEL"
  #endif

  #if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI1) &&                           \
      (STM32_PLLSAI1SRC == STM32_PLLSAI1SRC_HSE)
    #error "HSE not enabled, required by STM32_PLLSAI1SRC"
   #endif

  #if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI2) &&                           \
      (STM32_PLLSAI2SRC == STM32_PLLSAI2SRC_HSE)
    #error "HSE not enabled, required by STM32_PLLSAI2SRC"
  #endif

  /* SAI2-related checks.*/
  #if (STM32_SAI2SEL == STM32_SAI2SEL_PLL) &&                               \
      (STM32_PLLSRC == STM32_PLLSRC_HSE)
    #error "HSE not enabled, required by STM32_SAI2SEL"
  #endif

  #if (STM32_SAI2SEL == STM32_SAI2SEL_PLLSAI1) &&                           \
      (STM32_PLLSAI1SRC == STM32_PLLSAI1SRC_HSE)
    #error "HSE not enabled, required by STM32_PLLSAI1SRC"
  #endif

  #if (STM32_SAI2SEL == STM32_SAI2SEL_PLLSAI2) &&                           \
      (STM32_PLLSAI2SRC == STM32_PLLSAI2SRC_HSE)
    #error "HSE not enabled, required by STM32_PLLSAI2SRC"
  #endif

  /* RTC-related checks.*/
  #if STM32_RTCSEL == STM32_RTCSEL_HSEDIV
    #error "HSE not enabled, required by STM32_RTCSEL"
  #endif

#endif /* !STM32_HSE_ENABLED */

/*
 * Platform LSI-related checks.
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
 * Platform LSE-related checks.
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

  #if STM32_MSIPLL_ENABLED == TRUE
    #error "LSE not enabled, required by STM32_MSIPLL_ENABLED"
  #endif

#endif /* !STM32_LSE_ENABLED */

/**
 * @brief   PLL input clock frequency.
 */
#if (STM32_PLLSRC == STM32_PLLSRC_HSE) || defined(__DOXYGEN__)
  #define STM32_PLLCLKIN            (STM32_HSECLK / STM32_PLLM_VALUE)

#elif STM32_PLLSRC == STM32_PLLSRC_MSI
  #define STM32_PLLCLKIN            (STM32_MSICLK / STM32_PLLM_VALUE)

#elif STM32_PLLSRC == STM32_PLLSRC_HSI16
  #define STM32_PLLCLKIN            (STM32_HSI16CLK / STM32_PLLM_VALUE)

#elif STM32_PLLSRC == STM32_PLLSRC_NOCLOCK
  #define STM32_PLLCLKIN            0

#else
  #error "invalid STM32_PLLSRC value specified"
#endif

/**
 * @brief   PLLSAI1 input clock frequency.
 */
#if (STM32_PLLSAI1SRC == STM32_PLLSAI1SRC_HSE) || defined(__DOXYGEN__)
  #define STM32_PLLSAI1CLKIN        (STM32_HSECLK / STM32_PLLSAI1M_VALUE)

#elif STM32_PLLSAI1SRC == STM32_PLLSAI1SRC_MSI
  #define STM32_PLLSAI1CLKIN        (STM32_MSICLK / STM32_PLLSAI1M_VALUE)

#elif STM32_PLLSAI1SRC == STM32_PLLSAI1SRC_HSI16
  #define STM32_PLLSAI1CLKIN        (STM32_HSI16CLK / STM32_PLLSAI1M_VALUE)

#elif STM32_PLLSSAI1RC == STM32_PLLSAI1SRC_NOCLOCK
  #define STM32_PLLSAI1CLKIN        0

#else
  #error "invalid STM32_PLLSAI1SRC value specified"
#endif

/**
 * @brief   PLLSAI2 input clock frequency.
 */
#if (STM32_PLLSAI2SRC == STM32_PLLSAI2SRC_HSE) || defined(__DOXYGEN__)
  #define STM32_PLLSAI2CLKIN        (STM32_HSECLK / STM32_PLLSAI2M_VALUE)

#elif STM32_PLLSAI2SRC == STM32_PLLSAI2SRC_MSI
  #define STM32_PLLSAI2CLKIN        (STM32_MSICLK / STM32_PLLSAI2M_VALUE)

#elif STM32_PLLSAI2SRC == STM32_PLLSAI2SRC_HSI16
  #define STM32_PLLSAI2CLKIN        (STM32_HSI16CLK / STM32_PLLSAI2M_VALUE)

#elif STM32_PLLSAI2SRC == STM32_PLLSAI2SRC_NOCLOCK
  #define STM32_PLLSAI2CLKIN        0

#else
  #error "invalid STM32_PLLSAI2SRC value specified"
#endif

/*
 * PLL enable check.
 */
#if (STM32_HSI48_ENABLED && (STM32_CLK48SEL == STM32_CLK48SEL_PLL)) ||      \
    (STM32_SW == STM32_SW_PLL) ||                                           \
    (STM32_MCOSEL == STM32_MCOSEL_PLL) ||                                   \
    (STM32_SAI1SEL == STM32_SAI1SEL_PLL) ||                                 \
    (STM32_SAI2SEL == STM32_SAI2SEL_PLL) ||                                 \
    defined(__DOXYGEN__)

  #if STM32_PLLCLKIN == 0
    #error "PLL activation required but no PLL clock selected"
  #endif

/**
 * @brief   PLL activation flag.
 */
  #define STM32_ACTIVATE_PLL        TRUE
#else
  #define STM32_ACTIVATE_PLL        FALSE
#endif

/*
 * PLLSAI1 enable check.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI1) ||                             \
    (STM32_SAI2SEL == STM32_SAI2SEL_PLLSAI1) ||                             \
    (STM32_CLK48SEL == STM32_CLK48SEL_PLLSAI1) ||                           \
    (STM32_ADCSEL == STM32_ADCSEL_PLLSAI1) ||                               \
    defined(__DOXYGEN__)

  #if STM32_PLLSAI1CLKIN == 0
    #error "PLLSAI1 activation required but no PLL clock selected"
  #endif

/**
 * @brief   PLLSAI1 activation flag.
 */
  #define STM32_ACTIVATE_PLLSAI1    TRUE
#else
  #define STM32_ACTIVATE_PLLSAI1    FALSE
#endif

/*
 * PLLSAI2 enable check.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI2) ||                             \
    (STM32_SAI2SEL == STM32_SAI2SEL_PLLSAI2) ||                             \
    (STM32_ADCSEL == STM32_ADCSEL_PLLSAI1) ||                               \
    defined(__DOXYGEN__)

  #if STM32_PLLSAI2CLKIN == 0
    #error "PLLSAI2 activation required but no PLL clock selected"
  #endif

/**
 * @brief   PLLSAI2 activation flag.
 */
  #define STM32_ACTIVATE_PLLSAI2    TRUE
#else
  #define STM32_ACTIVATE_PLLSAI2    FALSE
#endif

/**
 * @brief   STM32_PLLPEN field.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_PLL) ||                                 \
    (STM32_SAI2SEL == STM32_SAI2SEL_PLL) ||                                 \
    defined(__DOXYGEN__)
  #define STM32_PLLPEN              (1 << 16)
#else
  #define STM32_PLLPEN              (0 << 16)
#endif

/**
 * @brief   STM32_PLLQEN field.
 */
#if (STM32_CLK48SEL == STM32_CLK48SEL_PLL) || defined(__DOXYGEN__)
  #define STM32_PLLQEN              (1 << 20)
#else
  #define STM32_PLLQEN              (0 << 20)
#endif

/**
 * @brief   STM32_PLLREN field.
 */
#if (STM32_SW == STM32_SW_PLL) ||                                           \
    (STM32_MCOSEL == STM32_MCOSEL_PLL) ||                                   \
    defined(__DOXYGEN__)
  #define STM32_PLLREN              (1 << 24)
#else
  #define STM32_PLLREN              (0 << 24)
#endif

/**
 * @brief   STM32_PLLSAI1PEN field.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI1) ||                             \
    (STM32_SAI2SEL == STM32_SAI2SEL_PLLSAI1) ||                             \
    defined(__DOXYGEN__)
  #define STM32_PLLSAI1PEN          (1 << 16)
#else
  #define STM32_PLLSAI1PEN          (0 << 16)
#endif

/**
 * @brief   STM32_PLLSAI1QEN field.
 */
#if (STM32_CLK48SEL == STM32_CLK48SEL_PLLSAI1) || defined(__DOXYGEN__)
  #define STM32_PLLSAI1QEN          (1 << 20)
#else
  #define STM32_PLLSAI1QEN          (0 << 20)
#endif

/**
 * @brief   STM32_PLLSAI1REN field.
 */
#if (STM32_ADCSEL == STM32_ADCSEL_PLLSAI1) || defined(__DOXYGEN__)
  #define STM32_PLLSAI1REN          (1 << 24)
#else
  #define STM32_PLLSAI1REN          (0 << 24)
#endif

/**
 * @brief   STM32_PLLSAI2PEN field.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI2) ||                             \
    (STM32_SAI2SEL == STM32_SAI2SEL_PLLSAI2) ||                             \
    defined(__DOXYGEN__)
  #define STM32_PLLSAI2PEN          (1 << 16)
#else
  #define STM32_PLLSAI2PEN          (0 << 16)
#endif

/* Inclusion of PLL-related checks and calculations.*/
#include <stm32_pll.inc>
#include "stm32_pllsai1.inc"
#include "stm32_pllsai2.inc"

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
#define STM32_MCODIVCLK             STM32_HSECLK

#elif STM32_MCOSEL == STM32_MCOSEL_PLL
#define STM32_MCODIVCLK             STM32_PLL_P_CLKOUT

#elif STM32_MCOSEL == STM32_MCOSEL_LSI
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
#define STM32_RTCCLK                (STM32_HSECLK / 32)

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
 * @brief   USART2 clock frequency.
 */
#if (STM32_USART2SEL == STM32_USART2SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_USART2CLK             STM32_PCLK1

#elif STM32_USART2SEL == STM32_USART2SEL_SYSCLK
#define STM32_USART2CLK             STM32_SYSCLK

#elif STM32_USART2SEL == STM32_USART2SEL_HSI16
#define STM32_USART2CLK             STM32_HSI16CLK

#elif STM32_USART2SEL == STM32_USART2SEL_LSE
#define STM32_USART2CLK             STM32_LSECLK

#else
#error "invalid source selected for USART2 clock"
#endif

/**
 * @brief   USART3 clock frequency.
 */
#if (STM32_USART3SEL == STM32_USART3SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_USART3CLK             STM32_PCLK1

#elif STM32_USART3SEL == STM32_USART3SEL_SYSCLK
#define STM32_USART3CLK             STM32_SYSCLK

#elif STM32_USART3SEL == STM32_USART3SEL_HSI16
#define STM32_USART3CLK             STM32_HSI16CLK

#elif STM32_USART3SEL == STM32_USART3SEL_LSE
#define STM32_USART3CLK             STM32_LSECLK

#else
#error "invalid source selected for USART3 clock"
#endif

/**
 * @brief   UART4 clock frequency.
 */
#if (STM32_UART4SEL == STM32_UART4SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_UART4CLK              STM32_PCLK1

#elif STM32_UART4SEL == STM32_UART4SEL_SYSCLK
#define STM32_UART4CLK              STM32_SYSCLK

#elif STM32_UART4SEL == STM32_UART4SEL_HSI16
#define STM32_UART4CLK              STM32_HSI16CLK

#elif STM32_UART4SEL == STM32_UART4SEL_LSE
#define STM32_UART4CLK              STM32_LSECLK

#else
#error "invalid source selected for UART4 clock"
#endif

/**
 * @brief   UART5 clock frequency.
 */
#if (STM32_UART5SEL == STM32_UART5SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_UART5CLK              STM32_PCLK1

#elif STM32_UART5SEL == STM32_UART5SEL_SYSCLK
#define STM32_UART5CLK              STM32_SYSCLK

#elif STM32_UART5SEL == STM32_UART5SEL_HSI16
#define STM32_UART5CLK              STM32_HSI16CLK

#elif STM32_UART5SEL == STM32_UART5SEL_LSE
#define STM32_UART5CLK              STM32_LSECLK

#else
#error "invalid source selected for UART5 clock"
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
 * @brief   I2C2 clock frequency.
 */
#if (STM32_I2C2SEL == STM32_I2C2SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_I2C2CLK               STM32_PCLK1

#elif STM32_I2C2SEL == STM32_I2C2SEL_SYSCLK
#define STM32_I2C2CLK               STM32_SYSCLK

#elif STM32_I2C2SEL == STM32_I2C2SEL_HSI16
#define STM32_I2C2CLK               STM32_HSI16CLK

#else
#error "invalid source selected for I2C2 clock"
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
 * @brief   I2C4 clock frequency.
 */
#if (STM32_I2C4SEL == STM32_I2C4SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_I2C4CLK               STM32_PCLK1

#elif STM32_I2C4SEL == STM32_I2C4SEL_SYSCLK
#define STM32_I2C4CLK               STM32_SYSCLK

#elif STM32_I2C4SEL == STM32_I2C4SEL_HSI16
#define STM32_I2C4CLK               STM32_HSI16CLK

#else
#error "invalid source selected for I2C4 clock"
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
 * @brief   LPTIM3 clock frequency.
 */
#if (STM32_LPTIM3SEL == STM32_LPTIM3SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_LPTIM3CLK             STM32_PCLK1

#elif STM32_LPTIM3SEL == STM32_LPTIM3SEL_LSI
#define STM32_LPTIM3CLK             STM32_LSICLK

#elif STM32_LPTIM3SEL == STM32_LPTIM3SEL_HSI16
#define STM32_LPTIM3CLK             STM32_HSI16CLK

#elif STM32_LPTIM3SEL == STM32_LPTIM3SEL_LSE
#define STM32_LPTIM3CLK             STM32_LSECLK

#else
#error "invalid source selected for LPTIM3 clock"
#endif

/**
 * @brief   48MHz clock frequency.
 */
#if (STM32_CLK48SEL == STM32_CLK48SEL_HSI48) || defined(__DOXYGEN__)
#define STM32_48CLK                 STM32_HSI48CLK

#elif STM32_CLK48SEL == STM32_CLK48SEL_PLLSAI1
#define STM32_48CLK                 (STM32_PLLSAI1VCO / STM32_PLLSAI1Q_VALUE)

#elif STM32_CLK48SEL == STM32_CLK48SEL_PLL
#define STM32_48CLK                 (STM32_PLLVCO / STM32_PLLQ_VALUE)

#elif STM32_CLK48SEL == STM32_CLK48SEL_MSI
#define STM32_48CLK                 STM32_MSICLK

#else
#error "invalid source selected for 48CLK clock"
#endif

/**
 * @brief   SAI1 clock frequency.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI1) || defined(__DOXYGEN__)
#define STM32_SAI1CLK               STM32_PLLSAI1_P_CLKOUT

#elif STM32_SAI1SEL == STM32_SAI1SEL_PLLSAI2
#define STM32_SAI1CLK               STM32_PLLSAI2_P_CLKOUT

#elif STM32_SAI1SEL == STM32_SAI1SEL_PLL
#define STM32_SAI1CLK               STM32_PLL_P_CLKOUT

#elif STM32_SAI1SEL == STM32_SAI1SEL_EXTCLK
#define STM32_SAI1CLK               0 /* Unknown, would require a board value */

#elif STM32_SAI1SEL == STM32_SAI1SEL_HSI16
#define STM32_SAI1CLK               STM32_HSI16CLK

#elif STM32_SAI1SEL == STM32_SAI1SEL_OFF
#define STM32_SAI1CLK               0

#else
#error "invalid source selected for SAI1 clock"
#endif

/**
 * @brief   SAI2 clock frequency.
 */
#if (STM32_SAI2SEL == STM32_SAI2SEL_PLLSAI1) || defined(__DOXYGEN__)
#define STM32_SAI2CLK               STM32_PLLSAI1_P_CLKOUT

#elif STM32_SAI2SEL == STM32_SAI2SEL_PLLSAI2
#define STM32_SAI2CLK               STM32_PLLSAI2_P_CLKOUT

#elif STM32_SAI2SEL == STM32_SAI2SEL_PLL
#define STM32_SAI2CLK               STM32_PLL_P_CLKOUT

#elif STM32_SAI2SEL == STM32_SAI2SEL_EXTCLK
#define STM32_SAI2CLK               0 /* Unknown, would require a board value */

#elif STM32_SAI2SEL == STM32_SAI2SEL_HSI16
#define STM32_SAI2CLK               STM32_HSI16CLK

#elif STM32_SAI2SEL == STM32_SAI2SEL_OFF
#define STM32_SAI2CLK               0

#else
#error "invalid source selected for SAI2 clock"
#endif

/**
 * @brief   SDMMC clock frequency.
 */
#if (STM32_SDMMCSEL == STM32_SDMMCSEL_48CLK) || defined(__DOXYGEN__)
#define STM32_SDMMCCLK              STM32_48CLK

#elif STM32_SDMMCSEL == STM32_SDMMCSEL_PLLSAI3CLK
#define STM32_SDMMCCLK              STM32_PLL_P_CLKOUT

#else
#error "invalid source selected for SDMMC clock"
#endif

/**
 * @brief   USB clock point.
 */
#define STM32_USBCLK                STM32_48CLK

/**
 * @brief   RNG clock point.
 */
#define STM32_RNGCLK                STM32_48CLK

/**
 * @brief   ADC clock frequency.
 */
#if (STM32_ADCSEL == STM32_ADCSEL_NOCLK) || defined(__DOXYGEN__)
#define STM32_ADCCLK                0

#elif STM32_ADCSEL == STM32_ADCSEL_PLLSAI1
#define STM32_ADCCLK                STM32_PLLSAI1_R_CLKOUT

#elif STM32_ADCSEL == STM32_ADCSEL_SYSCLK
#define STM32_ADCCLK                STM32_SYSCLK

#else
#error "invalid source selected for ADC clock"
#endif

/**
 * @brief   DFSDM clock frequency.
 */
#if (STM32_DFSDMSEL == STM32_DFSDMSEL_PCLK2) || defined(__DOXYGEN__)
#define STM32_DFSDMCLK              STM32_PCLK2

#elif STM32_DFSDMSEL == STM32_DFSDMSEL_SYSCLK
#define STM32_DFSDMCLK              STM32_SYSCLK

#else
#error "invalid source selected for DFSDM clock"
#endif

/**
 * @brief   SDMMC frequency.
 */
#define STM32_SDMMC1CLK             STM32_48CLK

/**
 * @brief   OSPI clock frequency.
 */
#if (STM32_OSPISEL == STM32_OSPISEL_SYSCLK) || defined(__DOXYGEN__)
#define STM32_OSPICLK               STM32_SYSCLK

#elif STM32_OSPISEL == STM32_OSPISEL_MSI
#define STM32_OSPICLK               STM32_MSICLK

#elif STM32_OSPISEL == STM32_OSPISEL_48CLK
#define STM32_OSPICLK               STM32_PLLSAI1_Q_CLKOUT

#else
#error "invalid source selected for OSPI clock"
#endif

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

#elif STM32_HCLK <= STM32_3WS_THRESHOLD
#define STM32_FLASHBITS             FLASH_ACR_LATENCY_3WS

#else
#define STM32_FLASHBITS             FLASH_ACR_LATENCY_4WS
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

#elif STM32_MSICLK <= STM32_3WS_THRESHOLD
#define STM32_MSI_FLASHBITS         FLASH_ACR_LATENCY_3WS

#else
#define STM32_MSI_FLASHBITS         FLASH_ACR_LATENCY_4WS
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
#include "sau.h"
#include "cache.h"
//#include "mpu_v7m.h"
#include "stm32_isr.h"
//#include "stm32_dma.h"
//#include "stm32_exti.h"
#include "stm32_rcc.h"
#include "stm32_tim.h"

/* Secure mode handler.*/
#include "stm32_secure.inc"

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
