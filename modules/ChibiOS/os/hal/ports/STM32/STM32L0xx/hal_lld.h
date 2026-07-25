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
 * @file    STM32L0xx/hal_lld.h
 * @brief   STM32L0xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32L011xx, STM32L031xx,
 *            STM32L051xx, STM32L052xx, STM32L053xx,
 *            STM32L061xx, STM32L062xx, STM32L063xx,
 *            STM32L071xx, STM32L072xx, STM32L073xx for ultra-low-power MCUs.
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

/*
 * Registry definitions.
 */
#include "stm32_registry.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification macros
 * @{
 */
#if defined(STM32L011xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32L011xx ultra-low-power MCU"

#elif defined(STM32L031xx)
#define PLATFORM_NAME           "STM32L031xx ultra-low-power MCU"

#elif defined(STM32L051xx)
#define PLATFORM_NAME           "STM32L051xx ultra-low-power MCU"

#elif defined(STM32L052xx)
#define PLATFORM_NAME           "STM32L052xx ultra-low-power MCU"

#elif defined(STM32L053xx)
#define PLATFORM_NAME           "STM32L053xx ultra-low-power MCU"

#elif defined(STM32L061xx)
#define PLATFORM_NAME           "STM32L061xx ultra-low-power MCU"

#elif defined(STM32L062xx)
#define PLATFORM_NAME           "STM32L062xx ultra-low-power MCU"

#elif defined(STM32L063xx)
#define PLATFORM_NAME           "STM32L063xx ultra-low-power MCU"

#elif defined(STM32L071xx)
#define PLATFORM_NAME           "STM32L071xx ultra-low-power MCU"

#elif defined(STM32L072xx)
#define PLATFORM_NAME           "STM32L073xx ultra-low-power MCU"

#elif defined(STM32L073xx)
#define PLATFORM_NAME           "STM32L073xx ultra-low-power MCU"

#else
#error "STM32L0xx device not specified"
#endif
/** @} */

/**
 * @name    Sub-family identifier
 */
#if !defined(STM32L0XX) || defined(__DOXYGEN__)
#define STM32L0XX
#endif
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define STM32_HSI16CLK          16000000    /**< 16MHz internal clock.      */
#define STM32_HSI48CLK          48000000    /**< 48MHz internal clock.      */
#define STM32_LSICLK            37000       /**< Low speed internal clock.  */
/** @} */

/**
 * @name    PWR_CR register bits definitions
 * @{
 */
#define STM32_PLS_MASK          (7 << 5)    /**< PLS field mask.            */
#define STM32_PLS_LEV0          (0 << 5)    /**< PVD level 0.               */
#define STM32_PLS_LEV1          (1 << 5)    /**< PVD level 1.               */
#define STM32_PLS_LEV2          (2 << 5)    /**< PVD level 2.               */
#define STM32_PLS_LEV3          (3 << 5)    /**< PVD level 3.               */
#define STM32_PLS_LEV4          (4 << 5)    /**< PVD level 4.               */
#define STM32_PLS_LEV5          (5 << 5)    /**< PVD level 5.               */
#define STM32_PLS_LEV6          (6 << 5)    /**< PVD level 6.               */
#define STM32_PLS_EXT           (7 << 5)    /**< PVD level 7.               */

#define STM32_VOS_MASK          (3 << 11)   /**< VOS field mask.            */
#define STM32_VOS_1P8           (1 << 11)   /**< VOS level 1.8 volts.       */
#define STM32_VOS_1P5           (2 << 11)   /**< VOS level 1.5 volts.       */
#define STM32_VOS_1P2           (3 << 11)   /**< VOS level 1.2 volts.       */
/** @} */

/**
 * @name    RCC_CR register bits definitions
 * @{
 */
#define STM32_RTCPRE_MASK       (3 << 20)   /**< RTCPRE mask.               */
#define STM32_RTCPRE_DIV2       (0 << 20)   /**< HSE divided by 2.          */
#define STM32_RTCPRE_DIV4       (1 << 20)   /**< HSE divided by 4.          */
#define STM32_RTCPRE_DIV8       (2 << 20)   /**< HSE divided by 2.          */
#define STM32_RTCPRE_DIV16      (3 << 20)   /**< HSE divided by 16.         */
/** @} */

/**
 * @name    RCC_CFGR register bits definitions
 * @{
 */
#define STM32_SW_MASK           (3 << 0)    /**< SW field mask.             */
#define STM32_SW_MSI            (0 << 0)    /**< SYSCLK source is MSI.      */
#define STM32_SW_HSI16          (1 << 0)    /**< SYSCLK source is HSI16     */
#define STM32_SW_HSE            (2 << 0)    /**< SYSCLK source is HSE.      */
#define STM32_SW_PLL            (3 << 0)    /**< SYSCLK source is PLL.      */

#define STM32_HPRE_MASK         (15 << 4)   /**< HPRE field mask.           */
#define STM32_HPRE_DIV1         (0 << 4)    /**< SYSCLK divided by 1.       */
#define STM32_HPRE_DIV2         (8 << 4)    /**< SYSCLK divided by 2.       */
#define STM32_HPRE_DIV4         (9 << 4)    /**< SYSCLK divided by 4.       */
#define STM32_HPRE_DIV8         (10 << 4)   /**< SYSCLK divided by 8.       */
#define STM32_HPRE_DIV16        (11 << 4)   /**< SYSCLK divided by 16.      */
#define STM32_HPRE_DIV64        (12 << 4)   /**< SYSCLK divided by 64.      */
#define STM32_HPRE_DIV128       (13 << 4)   /**< SYSCLK divided by 128.     */
#define STM32_HPRE_DIV256       (14 << 4)   /**< SYSCLK divided by 256.     */
#define STM32_HPRE_DIV512       (15 << 4)   /**< SYSCLK divided by 512.     */

#define STM32_PPRE1_MASK        (7 << 8)    /**< PPRE2 field mask.          */
#define STM32_PPRE1_DIV1        (0 << 8)    /**< HCLK divided by 1.         */
#define STM32_PPRE1_DIV2        (4 << 8)    /**< HCLK divided by 2.         */
#define STM32_PPRE1_DIV4        (5 << 8)    /**< HCLK divided by 4.         */
#define STM32_PPRE1_DIV8        (6 << 8)    /**< HCLK divided by 8.         */
#define STM32_PPRE1_DIV16       (7 << 8)    /**< HCLK divided by 16.        */

#define STM32_PPRE2_MASK        (7 << 11)   /**< PPRE2 field mask.          */
#define STM32_PPRE2_DIV1        (0 << 11)   /**< HCLK divided by 1.         */
#define STM32_PPRE2_DIV2        (4 << 11)   /**< HCLK divided by 2.         */
#define STM32_PPRE2_DIV4        (5 << 11)   /**< HCLK divided by 4.         */
#define STM32_PPRE2_DIV8        (6 << 11)   /**< HCLK divided by 8.         */
#define STM32_PPRE2_DIV16       (7 << 11)   /**< HCLK divided by 16.        */

#define STM32_STOPWUCK_MASK     (1 << 15)   /**< PLLDIV field mask.         */
#define STM32_STOPWUCK_MSI      (0 << 15)   /**< MSI is wakeup clock.       */
#define STM32_STOPWUCK_HSI16    (1 << 15)   /**< HSI16 is wakeup clock.     */

#define STM32_PLLSRC_MASK       (1 << 16)   /**< PLLSRC field mask.         */
#define STM32_PLLSRC_HSI16      (0 << 16)   /**< PLL clock source is HSI16. */
#define STM32_PLLSRC_HSE        (1 << 16)   /**< PLL clock source is HSE.   */

#define STM32_PLLMUL_MASK       (15 << 18)  /**< PLLMUL field mask.         */
#define STM32_PLLMUL_MUL3       (0 << 18)   /**< PLL multiplier is 3.       */
#define STM32_PLLMUL_MUL4       (1 << 18)   /**< PLL multiplier is 4.       */
#define STM32_PLLMUL_MUL6       (2 << 18)   /**< PLL multiplier is 6.       */
#define STM32_PLLMUL_MUL8       (3 << 18)   /**< PLL multiplier is 8.       */
#define STM32_PLLMUL_MUL12      (4 << 18)   /**< PLL multiplier is 12.      */
#define STM32_PLLMUL_MUL16      (5 << 18)   /**< PLL multiplier is 16.      */
#define STM32_PLLMUL_MUL24      (6 << 18)   /**< PLL multiplier is 24.      */
#define STM32_PLLMUL_MUL32      (7 << 18)   /**< PLL multiplier is 32.      */
#define STM32_PLLMUL_MUL48      (8 << 18)   /**< PLL multiplier is 48.      */

#define STM32_PLLDIV_MASK       (3 << 22)   /**< PLLDIV field mask.         */
#define STM32_PLLDIV_DIV2       (1 << 22)   /**< PLL divided by 2.          */
#define STM32_PLLDIV_DIV3       (2 << 22)   /**< PLL divided by 3.          */
#define STM32_PLLDIV_DIV4       (3 << 22)   /**< PLL divided by 4.          */

#define STM32_MCOSEL_MASK       (15 << 24)  /**< MCOSEL field mask.         */
#define STM32_MCOSEL_NOCLOCK    (0 << 24)   /**< No clock on MCO pin.       */
#define STM32_MCOSEL_SYSCLK     (1 << 24)   /**< SYSCLK on MCO pin.         */
#define STM32_MCOSEL_HSI16      (2 << 24)   /**< HSI16 clock on MCO pin.    */
#define STM32_MCOSEL_MSI        (3 << 24)   /**< MSI clock on MCO pin.      */
#define STM32_MCOSEL_HSE        (4 << 24)   /**< HSE clock on MCO pin.      */
#define STM32_MCOSEL_PLL        (5 << 24)   /**< PLL clock on MCO pin.      */
#define STM32_MCOSEL_LSI        (6 << 24)   /**< LSI clock on MCO pin.      */
#define STM32_MCOSEL_LSE        (7 << 24)   /**< LSE clock on MCO pin.      */
#define STM32_MCOSEL_HSI48      (8 << 24)   /**< HSI48 clock on MCO pin.    */

#define STM32_MCOPRE_MASK       (7 << 28)   /**< MCOPRE field mask.         */
#define STM32_MCOPRE_DIV1       (0 << 28)   /**< MCO is divided by 1.       */
#define STM32_MCOPRE_DIV2       (1 << 28)   /**< MCO is divided by 1.       */
#define STM32_MCOPRE_DIV4       (2 << 28)   /**< MCO is divided by 1.       */
#define STM32_MCOPRE_DIV8       (3 << 28)   /**< MCO is divided by 1.       */
#define STM32_MCOPRE_DIV16      (4 << 28)   /**< MCO is divided by 1.       */
/** @} */

/**
 * @name    RCC_ICSCR register bits definitions
 * @{
 */
#define STM32_MSIRANGE_MASK     (7 << 13)   /**< MSIRANGE field mask.       */
#define STM32_MSIRANGE_64K      (0 << 13)   /**< 64kHz nominal.             */
#define STM32_MSIRANGE_128K     (1 << 13)   /**< 128kHz nominal.            */
#define STM32_MSIRANGE_256K     (2 << 13)   /**< 256kHz nominal.            */
#define STM32_MSIRANGE_512K     (3 << 13)   /**< 512kHz nominal.            */
#define STM32_MSIRANGE_1M       (4 << 13)   /**< 1MHz nominal.              */
#define STM32_MSIRANGE_2M       (5 << 13)   /**< 2MHz nominal.              */
#define STM32_MSIRANGE_4M       (6 << 13)   /**< 4MHz nominal               */
/** @} */

/**
 * @name    RCC_CSR register bits definitions
 * @{
 */
#define STM32_RTCSEL_MASK       (3 << 16)   /**< RTC source mask.           */
#define STM32_RTCSEL_NOCLOCK    (0 << 16)   /**< No RTC source.             */
#define STM32_RTCSEL_LSE        (1 << 16)   /**< RTC source is LSE.         */
#define STM32_RTCSEL_LSI        (2 << 16)   /**< RTC source is LSI.         */
#define STM32_RTCSEL_HSEDIV     (3 << 16)   /**< RTC source is HSE divided. */
/** @} */

/**
 * @name    RCC_CCIPR register bits definitions
 * @{
 */
#define STM32_USART1SEL_MASK    (3 << 0)    /**< USART1 clock source mask.  */
#define STM32_USART1SEL_APB     (0 << 0)    /**< USART1 clock is APB.       */
#define STM32_USART1SEL_SYSCLK  (1 << 0)    /**< USART1 clock is SYSCLK.    */
#define STM32_USART1SEL_HSI16   (2 << 0)    /**< USART1 clock is HSI16.     */
#define STM32_USART1SEL_LSE     (3 << 0)    /**< USART1 clock is LSE.       */

#define STM32_USART2SEL_MASK    (3 << 2)    /**< USART2 clock source mask.  */
#define STM32_USART2SEL_APB     (0 << 2)    /**< USART2 clock is APB.       */
#define STM32_USART2SEL_SYSCLK  (1 << 2)    /**< USART2 clock is SYSCLK.    */
#define STM32_USART2SEL_HSI16   (2 << 2)    /**< USART2 clock is HSI16.     */
#define STM32_USART2SEL_LSE     (3 << 2)    /**< USART2 clock is LSE.       */

#define STM32_LPUART1SEL_MASK   (3 << 10)   /**< LPUART1 clock source mask. */
#define STM32_LPUART1SEL_APB    (0 << 10)   /**< LPUART1 clock is APB.      */
#define STM32_LPUART1SEL_SYSCLK (1 << 10)   /**< LPUART1 clock is SYSCLK.   */
#define STM32_LPUART1SEL_HSI16  (2 << 10)   /**< LPUART1 clock is HSI16.    */
#define STM32_LPUART1SEL_LSE    (3 << 10)   /**< LPUART1 clock is LSE.      */

#define STM32_I2C1SEL_MASK      (3 << 12)   /**< I2C1 clock source mask.    */
#define STM32_I2C1SEL_APB       (0 << 12)   /**< I2C1 clock is APB.         */
#define STM32_I2C1SEL_SYSCLK    (1 << 12)   /**< I2C1 clock is SYSCLK.      */
#define STM32_I2C1SEL_HSI16     (2 << 12)   /**< I2C1 clock is HSI16.       */

#define STM32_I2C3SEL_MASK      (3 << 16)   /**< I2C3 clock source mask.    */
#define STM32_I2C3SEL_APB       (0 << 16)   /**< I2C3 clock is APB.         */
#define STM32_I2C3SEL_SYSCLK    (1 << 16)   /**< I2C3 clock is SYSCLK.      */
#define STM32_I2C3SEL_HSI16     (2 << 16)   /**< I2C3 clock is HSI16.       */

#define STM32_LPTIM1SEL_MASK    (3 << 18)   /**< LPTIM1 clock source mask.  */
#define STM32_LPTIM1SEL_APB     (0 << 18)   /**< LPTIM1 clock is APB.       */
#define STM32_LPTIM1SEL_LSI     (1 << 18)   /**< LPTIM1 clock is LSI.       */
#define STM32_LPTIM1SEL_HSI16   (2 << 18)   /**< LPTIM1 clock is HSI16.     */
#define STM32_LPTIM1SEL_LSE     (3 << 18)   /**< LPTIM1 clock is LSE.       */

#define STM32_HSI48SEL_MASK     (1 << 26)   /**< HSI48SEL clock source mask.*/
#define STM32_HSI48SEL_USBPLL   (0 << 26)   /**< USB48 clock is PLL/2.      */
#define STM32_HSI48SEL_HSI48    (1 << 26)   /**< USB48 clock is HSI48.      */
/** @} */

/**
 * @name    SYSCFG_CFGR3_ register bits definitions
 * @{
 */
#define STM32_VREFINT_EN        (1 << 0)    /**< VREFINT enable switch.     */
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
#define STM32_VOS                           STM32_VOS_1P8
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
 * @brief   Enables or disables the HSI16 clock divider.
 */
#if !defined(STM32_HSI16_DIVIDER_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI16_DIVIDER_ENABLED         FALSE
#endif

/**
 * @brief   Enables or disables the LSI clock source.
 */
#if !defined(STM32_LSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSI_ENABLED                   TRUE
#endif

/**
 * @brief   Enables or disables the HSE clock source.
 */
#if !defined(STM32_HSE_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSE_ENABLED                   FALSE
#endif

/**
 * @brief   Enables or disables the LSE clock source.
 */
#if !defined(STM32_LSE_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSE_ENABLED                   FALSE
#endif

/**
 * @brief   ADC clock setting.
 */
#if !defined(STM32_ADC_CLOCK_ENABLED) || defined(__DOXYGEN__)
#define STM32_ADC_CLOCK_ENABLED             TRUE
#endif

/**
 * @brief   USB clock setting.
 */
#if !defined(STM32_USB_CLOCK_ENABLED) || defined(__DOXYGEN__)
#define STM32_USB_CLOCK_ENABLED             TRUE
#endif

/**
 * @brief   MSI frequency setting.
 */
#if !defined(STM32_MSIRANGE) || defined(__DOXYGEN__)
#define STM32_MSIRANGE                      STM32_MSIRANGE_2M
#endif

/**
 * @brief   Main clock source selection.
 * @note    If the selected clock source is not the PLL then the PLL is not
 *          initialized and started.
 * @note    The default value is calculated for a 32MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                            STM32_SW_PLL
#endif

/**
 * @brief   Clock source for the PLL.
 * @note    This setting has only effect if the PLL is selected as the
 *          system clock source.
 * @note    The default value is calculated for a 32MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_PLLSRC) || defined(__DOXYGEN__)
#define STM32_PLLSRC                        STM32_PLLSRC_HSI16
#endif

/**
 * @brief   PLL multiplier value.
 * @note    The allowed values are 3, 4, 6, 8, 12, 16, 32, 48.
 * @note    The default value is calculated for a 32MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_PLLMUL_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLMUL_VALUE                  4
#endif

/**
 * @brief   PLL divider value.
 * @note    The allowed values are 2, 3, 4.
 * @note    The default value is calculated for a 32MHz system clock from
 *          the internal 16MHz HSI clock.
 */
#if !defined(STM32_PLLDIV_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLDIV_VALUE                  2
#endif

/**
 * @brief   AHB prescaler value.
 * @note    The default value is calculated for a 32MHz system clock from
 *          the internal 16MHz HSI clock.
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
 * @brief   RTC/LCD clock source.
 */
#if !defined(STM32_RTCSEL) || defined(__DOXYGEN__)
#define STM32_RTCSEL                        STM32_RTCSEL_LSI
#endif

/**
 * @brief   HSE divider toward RTC setting.
 */
#if !defined(STM32_RTCPRE) || defined(__DOXYGEN__)
#define STM32_RTCPRE                        STM32_RTCPRE_DIV2
#endif

/**
 * @brief   USART1 clock source.
 */
#if !defined(STM32_USART1SEL) || defined(__DOXYGEN__)
#define STM32_USART1SEL                     STM32_USART1SEL_APB
#endif

/**
 * @brief   USART2 clock source.
 */
#if !defined(STM32_USART2SEL) || defined(__DOXYGEN__)
#define STM32_USART2SEL                     STM32_USART2SEL_APB
#endif

/**
 * @brief   LPUART1 clock source.
 */
#if !defined(STM32_LPUART1SEL) || defined(__DOXYGEN__)
#define STM32_LPUART1SEL                    STM32_LPUART1SEL_APB
#endif

/**
 * @brief   I2C clock source.
 */
#if !defined(STM32_I2C1SEL) || defined(__DOXYGEN__)
#define STM32_I2C1SEL                       STM32_I2C1SEL_APB
#endif

/**
 * @brief   LPTIM1 clock source.
 */
#if !defined(STM32_LPTIM1SEL) || defined(__DOXYGEN__)
#define STM32_LPTIM1SEL                     STM32_LPTIM1SEL_APB
#endif

/**
 * @bief    USB/RNG clock source.
 */
#if !defined(STM32_HSI48SEL) || defined(__DOXYGEN__)
#define STM32_HSI48SEL                      STM32_HSI48SEL_USBPLL
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(STM32L0xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L0xx_MCUCONF not defined"
#endif

#if defined(STM32L052xx) && !defined(STM32L052_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L052_MCUCONF not defined"

#elif defined(STM32L053xx) && !defined(STM32L053_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L053_MCUCONF not defined"

#elif defined(STM32L072xx) && !defined(STM32L072_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L072_MCUCONF not defined"

#elif defined(STM32L073xx) && !defined(STM32L073_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L073_MCUCONF not defined"

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

/* Voltage related limits.*/
#if (STM32_VOS == STM32_VOS_1P8) || defined(__DOXYGEN__)
/**
 * @name    Absolute Maximum Ratings
 * @{
 */
/**
 * @brief   Maximum SYSCLK clock frequency at current voltage setting.
 */
#define STM32_SYSCLK_MAX            32000000

/**
 * @brief   Maximum HSE clock frequency at current voltage setting.
 */
#define STM32_HSECLK_MAX            32000000

/**
 * @brief   Minimum HSE clock frequency.
 */
#define STM32_HSECLK_MIN            1000000

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_MAX            1000000

/**
 * @brief   Minimum LSE clock frequency.
 */
#define STM32_LSECLK_MIN            1000

/**
 * @brief   Maximum PLL input frequency.
 */
#define STM32_PLLIN_MAX             24000000

/**
 * @brief   Maximum PLL input frequency.
 */
#define STM32_PLLIN_MIN             2000000

/**
 * @brief   Maximum VCO clock frequency at current voltage setting.
 */
#define STM32_PLLVCO_MAX            96000000

/**
 * @brief   Minimum VCO clock frequency at current voltage setting.
 */
#define STM32_PLLVCO_MIN            6000000

/**
 * @brief   Maximum PLL output frequency.
 */
#define STM32_PLLOUT_MAX            32000000

/**
 * @brief   Maximum PLL output frequency.
 */
#define STM32_PLLOUT_MIN            2000000

/**
 * @brief   Maximum APB1 clock frequency.
 */
#define STM32_PCLK1_MAX             32000000

/**
 * @brief   Maximum APB2 clock frequency.
 */
#define STM32_PCLK2_MAX             32000000

/**
 * @brief   Maximum frequency not requiring a wait state for flash accesses.
 */
#define STM32_0WS_THRESHOLD         16000000

/**
 * @brief   HSI availability at current voltage settings.
 */
#define STM32_HSI_AVAILABLE         TRUE
/** @} */

#elif STM32_VOS == STM32_VOS_1P5
#define STM32_SYSCLK_MAX            16000000
#define STM32_HSECLK_MAX            16000000
#define STM32_HSECLK_MIN            1000000
#define STM32_LSECLK_MAX            1000000
#define STM32_LSECLK_MIN            1000
#define STM32_PLLIN_MAX             16000000
#define STM32_PLLIN_MIN             2000000
#define STM32_PLLVCO_MAX            48000000
#define STM32_PLLVCO_MIN            6000000
#define STM32_PLLOUT_MAX            16000000
#define STM32_PLLOUT_MIN            2000000
#define STM32_PCLK1_MAX             16000000
#define STM32_PCLK2_MAX             16000000
#define STM32_0WS_THRESHOLD         8000000
#define STM32_HSI_AVAILABLE         TRUE
#elif STM32_VOS == STM32_VOS_1P2
#define STM32_SYSCLK_MAX            4000000
#define STM32_HSECLK_MAX            8000000
#define STM32_HSECLK_MIN            1000000
#define STM32_LSECLK_MAX            1000000
#define STM32_LSECLK_MIN            1000
#define STM32_PLLIN_MAX             8000000
#define STM32_PLLIN_MIN             2000000
#define STM32_PLLVCO_MAX            24000000
#define STM32_PLLVCO_MIN            6000000
#define STM32_PLLOUT_MAX            4000000
#define STM32_PLLOUT_MIN            2000000
#define STM32_PCLK1_MAX             4000000
#define STM32_PCLK2_MAX             4000000
#define STM32_0WS_THRESHOLD         4000000
#define STM32_HSI_AVAILABLE         FALSE
#else
#error "invalid STM32_VOS value specified"
#endif

/* HSI related checks.*/
#if STM32_HSI16_ENABLED
#if !STM32_HSI_AVAILABLE
  #error "impossible to activate HSI under the current voltage settings"
#endif
#else /* !STM32_HSI16_ENABLED */

#if STM32_ADC_CLOCK_ENABLED
#error "HSI16 not enabled, required by STM32_ADC_CLOCK_ENABLED"
#endif

#if (STM32_SW == STM32_SW_HSI16)
#error "HSI16 not enabled, required by STM32_SW"
#endif

#if ((STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSI16))
#error "HSI16 not enabled, required by STM32_PLLSRC"
#endif

#if (STM32_MCOSEL == STM32_MCOSEL_HSI16)
#error "HSI16 not enabled, required by STM32_MCOSEL"
#endif

#if ((STM32_MCOSEL == STM32_MCOSEL_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSI16))
#error "HSI16 not enabled, required by STM32_PLLSRC"
#endif

#endif /* !STM32_HSI16_ENABLED */

/*
 * @brief   Divided HSI16 clock.
 */
#if STM32_HSI16_DIVIDER_ENABLED || defined(__DOXYGEN__)
#define STM32_HSI16DIVCLK           (STM32_HSI16CLK / 4)
#else
#define STM32_HSI16DIVCLK           STM32_HSI16CLK
#endif

/* HSE related checks.*/
#if STM32_HSE_ENABLED
#if STM32_HSECLK == 0
#error "impossible to activate HSE, frequency is zero"
#endif
#if (STM32_HSECLK < STM32_HSECLK_MIN) || (STM32_HSECLK > STM32_HSECLK_MAX)
#error "STM32_HSECLK outside acceptable range (STM32_HSECLK_MIN...STM32_HSECLK_MAX)"
#endif
#else /* !STM32_HSE_ENABLED */

#if (STM32_SW == STM32_SW_HSE)
#error "HSE not enabled, required by STM32_SW"
#endif

#if ((STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSE))
#error "HSE not enabled, required by STM32_PLLSRC"
#endif

#if (STM32_MCOSEL == STM32_MCOSEL_HSE)
#error "HSE not enabled, required by STM32_MCOSEL"
#endif

#if ((STM32_MCOSEL == STM32_MCOSEL_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSE))
#error "HSE not enabled, required by STM32_PLLSRC"
#endif

#if (STM32_RTCSEL == STM32_RTCSEL_HSEDIV)
#error "HSE not enabled, required by STM32_RTCSEL"
#endif

#endif /* !STM32_HSE_ENABLED */

/* LSI related checks.*/
#if STM32_LSI_ENABLED
#else /* !STM32_LSI_ENABLED */

#if STM32_MCOSEL == STM32_MCOSEL_LSI
#error "LSI not enabled, required by STM32_MCOSEL"
#endif

#if HAL_USE_RTC && (STM32_RTCSEL == STM32_RTCSEL_LSI)
#error "LSI not enabled, required by STM32_RTCSEL"
#endif

#endif /* !STM32_LSI_ENABLED */

/* LSE related checks.*/
#if STM32_LSE_ENABLED
#if (STM32_LSECLK == 0)
#error "impossible to activate LSE, frequency is zero"
#endif
#if (STM32_LSECLK < STM32_LSECLK_MIN) || (STM32_LSECLK > STM32_LSECLK_MAX)
#error "STM32_LSECLK outside acceptable range (STM32_LSECLK_MIN...STM32_LSECLK_MAX)"
#endif
#else /* !STM32_LSE_ENABLED */

#if STM32_MCOSEL == STM32_MCOSEL_LSE
#error "LSE not enabled, required by STM32_MCOSEL"
#endif

#if STM32_RTCSEL == STM32_RTCSEL_LSE
#error "LSE not enabled, required by STM32_RTCSEL"
#endif

#endif /* !STM32_LSE_ENABLED */

/* PLL related checks.*/
#if (STM32_SW == STM32_SW_PLL) || (STM32_MCOSEL == STM32_MCOSEL_PLL) ||       \
    (STM32_USB_CLOCK_ENABLED && (STM32_HSI48SEL == STM32_HSI48SEL_USBPLL)) || \
    defined(__DOXYGEN__)
/**
 * @brief   PLL activation flag.
 */
#define STM32_ACTIVATE_PLL          TRUE
#else
#define STM32_ACTIVATE_PLL          FALSE
#endif

/* HSI48 related checks.*/
#if (STM32_USB_CLOCK_ENABLED && (STM32_HSI48SEL == STM32_HSI48SEL_HSI48)) ||  \
    defined(__DOXYGEN__)
/**
 * @brief   HSI48 activation flag.
 */
#define STM32_ACTIVATE_HSI48        TRUE
#else
#define STM32_ACTIVATE_HSI48        FALSE
#endif

/**
 * @brief   PLLMUL field.
 */
#if (STM32_PLLMUL_VALUE == 3) || defined(__DOXYGEN__)
#define STM32_PLLMUL                STM32_PLLMUL_MUL3
#elif STM32_PLLMUL_VALUE == 4
#define STM32_PLLMUL                STM32_PLLMUL_MUL4
#elif STM32_PLLMUL_VALUE == 6
#define STM32_PLLMUL                STM32_PLLMUL_MUL6
#elif STM32_PLLMUL_VALUE == 8
#define STM32_PLLMUL                STM32_PLLMUL_MUL8
#elif STM32_PLLMUL_VALUE == 12
#define STM32_PLLMUL                STM32_PLLMUL_MUL12
#elif STM32_PLLMUL_VALUE == 16
#define STM32_PLLMUL                STM32_PLLMUL_MUL16
#elif STM32_PLLMUL_VALUE == 24
#define STM32_PLLMUL                STM32_PLLMUL_MUL24
#elif STM32_PLLMUL_VALUE == 32
#define STM32_PLLMUL                STM32_PLLMUL_MUL32
#elif STM32_PLLMUL_VALUE == 48
#define STM32_PLLMUL                STM32_PLLMUL_MUL48
#else
#error "invalid STM32_PLLMUL_VALUE value specified"
#endif

/**
 * @brief   PLLDIV field.
 */
#if (STM32_PLLDIV_VALUE == 2) || defined(__DOXYGEN__)
#define STM32_PLLDIV                STM32_PLLDIV_DIV2
#elif STM32_PLLDIV_VALUE == 3
#define STM32_PLLDIV                STM32_PLLDIV_DIV3
#elif STM32_PLLDIV_VALUE == 4
#define STM32_PLLDIV                STM32_PLLDIV_DIV4
#else
#error "invalid STM32_PLLDIV_VALUE value specified"
#endif

/**
 * @brief   PLL input clock frequency.
 */
#if (STM32_PLLSRC == STM32_PLLSRC_HSE) || defined(__DOXYGEN__)
#define STM32_PLLCLKIN              STM32_HSECLK
#elif STM32_PLLSRC == STM32_PLLSRC_HSI16
#define STM32_PLLCLKIN              STM32_HSI16DIVCLK
#else
#error "invalid STM32_PLLSRC value specified"
#endif

/* PLL input frequency range check.*/
#if (STM32_PLLCLKIN < STM32_PLLIN_MIN) || (STM32_PLLCLKIN > STM32_PLLIN_MAX)
#error "STM32_PLLCLKIN outside acceptable range (STM32_PLLIN_MIN...STM32_PLLIN_MAX)"
#endif

/**
 * @brief   PLL VCO frequency.
 */
#define STM32_PLLVCO                (STM32_PLLCLKIN * STM32_PLLMUL_VALUE)

/* PLL output frequency range check.*/
#if (STM32_PLLVCO < STM32_PLLVCO_MIN) || (STM32_PLLVCO > STM32_PLLVCO_MAX)
#error "STM32_PLLVCO outside acceptable range (STM32_PLLVCO_MIN...STM32_PLLVCO_MAX)"
#endif

/**
 * @brief   PLL output clock frequency.
 */
#define STM32_PLLCLKOUT             (STM32_PLLVCO / STM32_PLLDIV_VALUE)

/* PLL output frequency range check.*/
#if (STM32_PLLCLKOUT < STM32_PLLOUT_MIN) || (STM32_PLLCLKOUT > STM32_PLLOUT_MAX)
#error "STM32_PLLCLKOUT outside acceptable range (STM32_PLLOUT_MIN...STM32_PLLOUT_MAX)"
#endif

/**
 * @brief   MSI frequency.
 * @note    Values are taken from the STM8Lxx datasheet.
 */
#if STM32_MSIRANGE == STM32_MSIRANGE_64K
#define STM32_MSICLK                65500
#elif STM32_MSIRANGE == STM32_MSIRANGE_128K
#define STM32_MSICLK                131000
#elif STM32_MSIRANGE == STM32_MSIRANGE_256K
#define STM32_MSICLK                262000
#elif STM32_MSIRANGE == STM32_MSIRANGE_512K
#define STM32_MSICLK                524000
#elif STM32_MSIRANGE == STM32_MSIRANGE_1M
#define STM32_MSICLK                1050000
#elif STM32_MSIRANGE == STM32_MSIRANGE_2M
#define STM32_MSICLK                2100000
#elif STM32_MSIRANGE == STM32_MSIRANGE_4M
#define STM32_MSICLK                4200000
#else
#error "invalid STM32_MSIRANGE value specified"
#endif

/**
 * @brief   System clock source.
 */
#if (STM32_SW == STM32_SW_MSI)
#define STM32_SYSCLK                STM32_MSICLK
#elif (STM32_SW == STM32_SW_HSI16)
#define STM32_SYSCLK                STM32_HSI16DIVCLK
#elif (STM32_SW == STM32_SW_HSE)
#define STM32_SYSCLK                STM32_HSECLK
#elif (STM32_SW == STM32_SW_PLL)
#define STM32_SYSCLK                STM32_PLLCLKOUT
#else
#error "invalid STM32_SW value specified"
#endif

/* Check on the system clock.*/
#if STM32_SYSCLK > STM32_SYSCLK_MAX
#error "STM32_SYSCLK above maximum rated frequency (STM32_SYSCLK_MAX)"
#endif

/**
 * @brief   AHB frequency.
 */
#if (STM32_HPRE == STM32_HPRE_DIV1) || defined(__DOXYGEN__)
#define STM32_HCLK                  (STM32_SYSCLK / 1)
#elif STM32_HPRE == STM32_HPRE_DIV2
#define STM32_HCLK                  (STM32_SYSCLK / 2)
#elif STM32_HPRE == STM32_HPRE_DIV4
#define STM32_HCLK                  (STM32_SYSCLK / 4)
#elif STM32_HPRE == STM32_HPRE_DIV8
#define STM32_HCLK                  (STM32_SYSCLK / 8)
#elif STM32_HPRE == STM32_HPRE_DIV16
#define STM32_HCLK                  (STM32_SYSCLK / 16)
#elif STM32_HPRE == STM32_HPRE_DIV64
#define STM32_HCLK                  (STM32_SYSCLK / 64)
#elif STM32_HPRE == STM32_HPRE_DIV128
#define STM32_HCLK                  (STM32_SYSCLK / 128)
#elif STM32_HPRE == STM32_HPRE_DIV256
#define STM32_HCLK                  (STM32_SYSCLK / 256)
#elif STM32_HPRE == STM32_HPRE_DIV512
#define STM32_HCLK                  (STM32_SYSCLK / 512)
#else
#error "invalid STM32_HPRE value specified"
#endif

/* AHB frequency check.*/
#if STM32_HCLK > STM32_SYSCLK_MAX
#error "STM32_HCLK exceeding maximum frequency (STM32_SYSCLK_MAX)"
#endif

/**
 * @brief   APB1 frequency.
 */
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) || defined(__DOXYGEN__)
#define STM32_PCLK1                 (STM32_HCLK / 1)
#elif STM32_PPRE1 == STM32_PPRE1_DIV2
#define STM32_PCLK1                 (STM32_HCLK / 2)
#elif STM32_PPRE1 == STM32_PPRE1_DIV4
#define STM32_PCLK1                 (STM32_HCLK / 4)
#elif STM32_PPRE1 == STM32_PPRE1_DIV8
#define STM32_PCLK1                 (STM32_HCLK / 8)
#elif STM32_PPRE1 == STM32_PPRE1_DIV16
#define STM32_PCLK1                 (STM32_HCLK / 16)
#else
#error "invalid STM32_PPRE1 value specified"
#endif

/* APB1 frequency check.*/
#if STM32_PCLK1 > STM32_PCLK1_MAX
#error "STM32_PCLK1 exceeding maximum frequency (STM32_PCLK1_MAX)"
#endif

/**
 * @brief   APB2 frequency.
 */
#if (STM32_PPRE2 == STM32_PPRE2_DIV1) || defined(__DOXYGEN__)
#define STM32_PCLK2                 (STM32_HCLK / 1)
#elif STM32_PPRE2 == STM32_PPRE2_DIV2
#define STM32_PCLK2                 (STM32_HCLK / 2)
#elif STM32_PPRE2 == STM32_PPRE2_DIV4
#define STM32_PCLK2                 (STM32_HCLK / 4)
#elif STM32_PPRE2 == STM32_PPRE2_DIV8
#define STM32_PCLK2                 (STM32_HCLK / 8)
#elif STM32_PPRE2 == STM32_PPRE2_DIV16
#define STM32_PCLK2                 (STM32_HCLK / 16)
#else
#error "invalid STM32_PPRE2 value specified"
#endif

/* APB2 frequency check.*/
#if STM32_PCLK2 > STM32_PCLK2_MAX
#error "STM32_PCLK2 exceeding maximum frequency (STM32_PCLK2_MAX)"
#endif

/**
 * @brief   MCO selector clock.
 */
#if (STM32_MCOSEL == STM32_MCOSEL_NOCLOCK) || defined(__DOXYGEN__)
#define STM32_MCODIVCLK             0
#elif STM32_MCOSEL == STM32_MCOSEL_SYSCLK
#define STM32_MCODIVCLK             STM32_SYSCLK
#elif STM32_MCOSEL == STM32_MCOSEL_HSI16
#define STM32_MCODIVCLK             STM32_HSI16DIVCLK
#elif STM32_MCOSEL == STM32_MCOSEL_MSI
#define STM32_MCODIVCLK             STM32_MSICLK
#elif STM32_MCOSEL == STM32_MCOSEL_HSE
#define STM32_MCODIVCLK             STM32_HSECLK
#elif STM32_MCOSEL == STM32_MCOSEL_PLL
#define STM32_MCODIVCLK             STM32_PLLCLKOUT
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
 * @brief   MCO output pin clock.
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
 * @brief   HSE divider toward RTC clock.
 */
#if (STM32_RTCPRE == STM32_RTCPRE_DIV2) || defined(__DOXYGEN__)
#define STM32_HSEDIVCLK             (STM32_HSECLK / 2)
#elif (STM32_RTCPRE == STM32_RTCPRE_DIV4) || defined(__DOXYGEN__)
#define STM32_HSEDIVCLK             (STM32_HSECLK / 4)
#elif (STM32_RTCPRE == STM32_RTCPRE_DIV8) || defined(__DOXYGEN__)
#define STM32_HSEDIVCLK             (STM32_HSECLK / 8)
#elif (STM32_RTCPRE == STM32_RTCPRE_DIV16) || defined(__DOXYGEN__)
#define STM32_HSEDIVCLK             (STM32_HSECLK / 16)
#else
#error "invalid STM32_RTCPRE value specified"
#endif

/**
 * @brief   RTC/LCD clock.
 */
#if (STM32_RTCSEL == STM32_RTCSEL_NOCLOCK) || defined(__DOXYGEN__)
#define STM32_RTCCLK                  0
#elif STM32_RTCSEL == STM32_RTCSEL_LSE
#define STM32_RTCCLK                  STM32_LSECLK
#elif STM32_RTCSEL == STM32_RTCSEL_LSI
#define STM32_RTCCLK                  STM32_LSICLK
#elif STM32_RTCSEL == STM32_RTCSEL_HSEDIV
#define STM32_RTCCLK                  STM32_HSEDIVCLK
#else
#error "invalid STM32_RTCSEL value specified"
#endif

/**
 * @brief   USART1 frequency.
 */
#if (STM32_USART1SEL == STM32_USART1SEL_APB) || defined(__DOXYGEN__)
#define STM32_USART1CLK             STM32_PCLK2
#elif STM32_USART1SEL == STM32_USART1SEL_SYSCLK
#define STM32_USART1CLK             STM32_SYSCLK
#elif STM32_USART1SEL == STM32_USART1SEL_HSI16
#define STM32_USART1CLK             STM32_HSI16DIVCLK
#elif STM32_USART1SEL == STM32_USART1SEL_LSE
#define STM32_USART1CLK             STM32_LSECLK
#else
#error "invalid source selected for USART1 clock"
#endif

/**
 * @brief   USART2 frequency.
 */
#if (STM32_USART2SEL == STM32_USART2SEL_APB) || defined(__DOXYGEN__)
#define STM32_USART2CLK             STM32_PCLK1
#elif STM32_USART2SEL == STM32_USART2SEL_SYSCLK
#define STM32_USART2CLK             STM32_SYSCLK
#elif STM32_USART2SEL == STM32_USART2SEL_HSI16
#define STM32_USART2CLK             STM32_HSI16DIVCLK
#elif STM32_USART2SEL == STM32_USART2SEL_LSE
#define STM32_USART2CLK             STM32_LSECLK
#else
#error "invalid source selected for USART2 clock"
#endif

/**
 * @brief   USART4 frequency.
 */
#define STM32_UART4CLK              STM32_PCLK1

/**
 * @brief   USART5 frequency.
 */
#define STM32_UART5CLK              STM32_PCLK1

/**
 * @brief   LPUART1 frequency.
 */
#if (STM32_LPUART1SEL == STM32_LPUART1SEL_APB) || defined(__DOXYGEN__)
#define STM32_LPUART1CLK            STM32_PCLK1
#elif STM32_LPUART1SEL == STM32_LPUART1SEL_SYSCLK
#define STM32_LPUART1CLK            STM32_SYSCLK
#elif STM32_LPUART1SEL == STM32_LPUART1SEL_HSI16
#define STM32_LPUART1CLK            STM32_HSI16DIVCLK
#elif STM32_LPUART1SEL == STM32_LPUART1SEL_LSE
#define STM32_LPUART1CLK            STM32_LSECLK
#else
#error "invalid source selected for LPUART1 clock"
#endif

/**
 * @brief   I2C1 frequency.
 */
#if (STM32_I2C1SEL == STM32_I2C1SEL_APB) || defined(__DOXYGEN__)
#define STM32_I2C1CLK               STM32_PCLK1
#elif STM32_I2C1SEL == STM32_I2C1SEL_SYSCLK
#define STM32_I2C1CLK               STM32_SYSCLK
#elif STM32_I2C1SEL == STM32_I2C1SEL_HSI16
#define STM32_I2C1CLK               STM32_HSI16DIVCLK
#else
#error "invalid source selected for I2C1 clock"
#endif

/**
 * @brief   LPTIM1 frequency.
 */
#if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_APB) || defined(__DOXYGEN__)
#define STM32_LPTIM1CLK             STM32_PCLK1
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSI
#define STM32_LPTIM1CLK             STM32_LSICLK
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_HSI16
#define STM32_LPTIM1CLK             STM32_HSI16DIVCLK
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSE
#define STM32_LPTIM1CLK             STM32_LSECLK
#else
#error "invalid source selected for LPTIM1 clock"
#endif

/**
 * @brief   USB clock point.
 */
#if (STM32_HSI48SEL == STM32_HSI48SEL_HSI48) || defined(__DOXYGEN__)
#define STM32_USBCLK                STM32_HSI48CLK
#elif STM32_HSI48SEL == STM32_HSI48SEL_USBPLL
#define STM32_USBCLK                (STM32_PLLVCO / 2)
#else
#error "invalid STM32_HSI48SEL value specified"
#endif

/**
 * @brief   RNG clock point.
 */
#define STM32_RNGCLK                STM32_USBCLK

/**
 * @brief   Timers LPTIM1, TIM2, TIM6 clock.
 */
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK1               (STM32_PCLK1 * 1)
#else
#define STM32_TIMCLK1               (STM32_PCLK1 * 2)
#endif

/**
 * @brief   Timers TIM21, TIM22 clock.
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
#define STM32_FLASHBITS             0
#else
#define STM32_FLASHBITS             (FLASH_ACR_PRE_READ |                   \
                                     FLASH_ACR_PRFTEN   |                   \
                                     FLASH_ACR_LATENCY)
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
