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
 * @file    STM32F3xx/hal_lld.h
 * @brief   STM32F3xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32F301x8 for Analog & DSP devices.
 *          - STM32F302x8 for Analog & DSP devices.
 *          - STM32F302xC for Analog & DSP devices.
 *          - STM32F302xE for Analog & DSP devices.
 *          - STM32F303x8 for Analog & DSP devices.
 *          - STM32F303xC for Analog & DSP devices.
 *          - STM32F303xE for Analog & DSP devices.
 *          - STM32F318xx for Analog & DSP devices.
 *          - STM32F328xx for Analog & DSP devices.
 *          - STM32F334x8 for Analog & DSP devices.
 *          - STM32F358xx for Analog & DSP devices.
 *          - STM32F398xx for Analog & DSP devices.
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
 * @name    Platform identification macros
 * @{
 */
#if defined(STM32F301x8) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32F301x8 Analog & DSP"

#elif defined(STM32F302x8)
#define PLATFORM_NAME           "STM32F302x8 Analog & DSP"

#elif defined(STM32F302xC)
#define PLATFORM_NAME           "STM32F302xC Analog & DSP"

#elif defined(STM32F302xE)
#define PLATFORM_NAME           "STM32F302xE Analog & DSP"

#elif defined(STM32F303x8)
#define PLATFORM_NAME           "STM32F303x8 Analog & DSP"

#elif defined(STM32F303xC)
#define PLATFORM_NAME           "STM32F303xC Analog & DSP"

#elif defined(STM32F303xE)
#define PLATFORM_NAME           "STM32F303xE Analog & DSP"

#elif defined(STM32F318xx)
#define PLATFORM_NAME           "STM32F318xx Analog & DSP"

#elif defined(STM32F328xx)
#define PLATFORM_NAME           "STM32F328xx Analog & DSP"

#elif defined(STM32F334x8)
#define PLATFORM_NAME           "STM32F334x8 Analog & DSP"

#elif defined(STM32F358xx)
#define PLATFORM_NAME           "STM32F358xx Analog & DSP"

#elif defined(STM32F398xx)
#define PLATFORM_NAME           "STM32F398xx Analog & DSP"

#else
#error "STM32F3xx device not specified"
#endif
/** @} */

/**
 * @name    Absolute Maximum Ratings
 * @{
 */
/**
 * @brief   Maximum system clock frequency.
 */
#define STM32_SYSCLK_MAX        72000000

/**
 * @brief   Maximum HSE clock frequency.
 */
#define STM32_HSECLK_MAX        32000000

/**
 * @brief   Minimum HSE clock frequency.
 */
#define STM32_HSECLK_MIN        1000000

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_MAX        1000000

/**
 * @brief   Minimum LSE clock frequency.
 */
#define STM32_LSECLK_MIN        32768

/**
 * @brief   Maximum PLLs input clock frequency.
 */
#define STM32_PLLIN_MAX         24000000

/**
 * @brief   Minimum PLLs input clock frequency.
 */
#define STM32_PLLIN_MIN         1000000

/**
 * @brief   Maximum PLL output clock frequency.
 */
#define STM32_PLLOUT_MAX        72000000

/**
 * @brief   Minimum PLL output clock frequency.
 */
#define STM32_PLLOUT_MIN        16000000

/**
 * @brief   Maximum APB1 clock frequency.
 */
#define STM32_PCLK1_MAX         36000000

/**
 * @brief   Maximum APB2 clock frequency.
 */
#define STM32_PCLK2_MAX         72000000

/**
 * @brief   Maximum ADC clock frequency.
 */
#define STM32_ADCCLK_MAX        72000000
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define STM32_HSICLK            8000000     /**< High speed internal clock. */
#define STM32_LSICLK            40000       /**< Low speed internal clock.  */
/** @} */

/**
 * @name    PWR_CR register bits definitions
 * @{
 */
#define STM32_PLS_MASK          (7 << 5)    /**< PLS bits mask.             */
#define STM32_PLS_LEV0          (0 << 5)    /**< PVD level 0.               */
#define STM32_PLS_LEV1          (1 << 5)    /**< PVD level 1.               */
#define STM32_PLS_LEV2          (2 << 5)    /**< PVD level 2.               */
#define STM32_PLS_LEV3          (3 << 5)    /**< PVD level 3.               */
#define STM32_PLS_LEV4          (4 << 5)    /**< PVD level 4.               */
#define STM32_PLS_LEV5          (5 << 5)    /**< PVD level 5.               */
#define STM32_PLS_LEV6          (6 << 5)    /**< PVD level 6.               */
#define STM32_PLS_LEV7          (7 << 5)    /**< PVD level 7.               */
/** @} */

/**
 * @name    RCC_CFGR register bits definitions
 * @{
 */
#define STM32_SW_HSI            (0 << 0)    /**< SYSCLK source is HSI.      */
#define STM32_SW_HSE            (1 << 0)    /**< SYSCLK source is HSE.      */
#define STM32_SW_PLL            (2 << 0)    /**< SYSCLK source is PLL.      */

#define STM32_HPRE_DIV1         (0 << 4)    /**< SYSCLK divided by 1.       */
#define STM32_HPRE_DIV2         (8 << 4)    /**< SYSCLK divided by 2.       */
#define STM32_HPRE_DIV4         (9 << 4)    /**< SYSCLK divided by 4.       */
#define STM32_HPRE_DIV8         (10 << 4)   /**< SYSCLK divided by 8.       */
#define STM32_HPRE_DIV16        (11 << 4)   /**< SYSCLK divided by 16.      */
#define STM32_HPRE_DIV64        (12 << 4)   /**< SYSCLK divided by 64.      */
#define STM32_HPRE_DIV128       (13 << 4)   /**< SYSCLK divided by 128.     */
#define STM32_HPRE_DIV256       (14 << 4)   /**< SYSCLK divided by 256.     */
#define STM32_HPRE_DIV512       (15 << 4)   /**< SYSCLK divided by 512.     */

#define STM32_PPRE1_DIV1        (0 << 8)    /**< HCLK divided by 1.         */
#define STM32_PPRE1_DIV2        (4 << 8)    /**< HCLK divided by 2.         */
#define STM32_PPRE1_DIV4        (5 << 8)    /**< HCLK divided by 4.         */
#define STM32_PPRE1_DIV8        (6 << 8)    /**< HCLK divided by 8.         */
#define STM32_PPRE1_DIV16       (7 << 8)    /**< HCLK divided by 16.        */

#define STM32_PPRE2_DIV1        (0 << 11)   /**< HCLK divided by 1.         */
#define STM32_PPRE2_DIV2        (4 << 11)   /**< HCLK divided by 2.         */
#define STM32_PPRE2_DIV4        (5 << 11)   /**< HCLK divided by 4.         */
#define STM32_PPRE2_DIV8        (6 << 11)   /**< HCLK divided by 8.         */
#define STM32_PPRE2_DIV16       (7 << 11)   /**< HCLK divided by 16.        */

#define STM32_PLLSRC_HSI        (0 << 16)   /**< PLL clock source is HSI/2. */
#define STM32_PLLSRC_HSE        (1 << 16)   /**< PLL clock source is
                                                 HSE/PREDIV.                */

#define STM32_USBPRE_DIV1P5     (0 << 22)   /**< USB clock is PLLCLK/1.5.   */
#define STM32_USBPRE_DIV1       (1 << 22)   /**< USB clock is PLLCLK/1.     */

#define STM32_MCOSEL_NOCLOCK    (0 << 24)   /**< No clock on MCO pin.       */
#define STM32_MCOSEL_LSI        (2 << 24)   /**< LSI clock on MCO pin.      */
#define STM32_MCOSEL_LSE        (3 << 24)   /**< LSE clock on MCO pin.      */
#define STM32_MCOSEL_SYSCLK     (4 << 24)   /**< SYSCLK on MCO pin.         */
#define STM32_MCOSEL_HSI        (5 << 24)   /**< HSI clock on MCO pin.      */
#define STM32_MCOSEL_HSE        (6 << 24)   /**< HSE clock on MCO pin.      */
#define STM32_MCOSEL_PLLDIV2    (7 << 24)   /**< PLL/2 clock on MCO pin.    */
/** @} */

/**
 * @name    RCC_BDCR register bits definitions
 * @{
 */
#define STM32_RTCSEL_MASK       (3 << 8)    /**< RTC clock source mask.     */
#define STM32_RTCSEL_NOCLOCK    (0 << 8)    /**< No clock.                  */
#define STM32_RTCSEL_LSE        (1 << 8)    /**< LSE used as RTC clock.     */
#define STM32_RTCSEL_LSI        (2 << 8)    /**< LSI used as RTC clock.     */
#define STM32_RTCSEL_HSEDIV     (3 << 8)    /**< HSE divided by 32 used as
                                                 RTC clock.                 */
/** @} */

/**
 * @name    RCC_CFGR2 register bits definitions
 * @{
 */
#define STM32_PREDIV_MASK       (15 << 0)   /**< PREDIV divisor mask.       */
#define STM32_ADC12PRES_MASK    (31 << 4)   /**< ADC12 clock source mask.   */
#define STM32_ADC12PRES_NOCLOCK (0 << 4)    /**< ADC12 clock is disabled.   */
#define STM32_ADC12PRES_DIV1    (16 << 4)   /**< ADC12 clock is PLL/1.      */
#define STM32_ADC12PRES_DIV2    (17 << 4)   /**< ADC12 clock is PLL/2.      */
#define STM32_ADC12PRES_DIV4    (18 << 4)   /**< ADC12 clock is PLL/4.      */
#define STM32_ADC12PRES_DIV6    (19 << 4)   /**< ADC12 clock is PLL/6.      */
#define STM32_ADC12PRES_DIV8    (20 << 4)   /**< ADC12 clock is PLL/8.      */
#define STM32_ADC12PRES_DIV10   (21 << 4)   /**< ADC12 clock is PLL/10.     */
#define STM32_ADC12PRES_DIV12   (22 << 4)   /**< ADC12 clock is PLL/12.     */
#define STM32_ADC12PRES_DIV16   (23 << 4)   /**< ADC12 clock is PLL/16.     */
#define STM32_ADC12PRES_DIV32   (24 << 4)   /**< ADC12 clock is PLL/32.     */
#define STM32_ADC12PRES_DIV64   (25 << 4)   /**< ADC12 clock is PLL/64.     */
#define STM32_ADC12PRES_DIV128  (26 << 4)   /**< ADC12 clock is PLL/128.    */
#define STM32_ADC12PRES_DIV256  (27 << 4)   /**< ADC12 clock is PLL/256.    */
#define STM32_ADC34PRES_MASK    (31 << 9)   /**< ADC34 clock source mask.   */
#define STM32_ADC34PRES_NOCLOCK (0 << 9)    /**< ADC34 clock is disabled.   */
#define STM32_ADC34PRES_DIV1    (16 << 9)   /**< ADC34 clock is PLL/1.      */
#define STM32_ADC34PRES_DIV2    (17 << 9)   /**< ADC34 clock is PLL/2.      */
#define STM32_ADC34PRES_DIV4    (18 << 9)   /**< ADC34 clock is PLL/4.      */
#define STM32_ADC34PRES_DIV6    (19 << 9)   /**< ADC34 clock is PLL/6.      */
#define STM32_ADC34PRES_DIV8    (20 << 9)   /**< ADC34 clock is PLL/8.      */
#define STM32_ADC34PRES_DIV10   (21 << 9)   /**< ADC34 clock is PLL/10.     */
#define STM32_ADC34PRES_DIV12   (22 << 9)   /**< ADC34 clock is PLL/12.     */
#define STM32_ADC34PRES_DIV16   (23 << 9)   /**< ADC34 clock is PLL/16.     */
#define STM32_ADC34PRES_DIV32   (24 << 9)   /**< ADC34 clock is PLL/32.     */
#define STM32_ADC34PRES_DIV64   (25 << 9)   /**< ADC34 clock is PLL/64.     */
#define STM32_ADC34PRES_DIV128  (26 << 9)   /**< ADC34 clock is PLL/128.    */
#define STM32_ADC34PRES_DIV256  (27 << 9)   /**< ADC34 clock is PLL/256.    */
/** @} */

/**
 * @name    RCC_CFGR3 register bits definitions
 * @{
 */
#define STM32_USART1SW_MASK     (3 << 0)    /**< USART1 clock source mask. */
#define STM32_USART1SW_PCLK     (0 << 0)    /**< USART1 clock is PCLK.     */
#define STM32_USART1SW_SYSCLK   (1 << 0)    /**< USART1 clock is SYSCLK.   */
#define STM32_USART1SW_LSE      (2 << 0)    /**< USART1 clock is LSE.      */
#define STM32_USART1SW_HSI      (3 << 0)    /**< USART1 clock is HSI.      */
#define STM32_I2C1SW_MASK       (1 << 4)    /**< I2C1 clock source mask.   */
#define STM32_I2C1SW_HSI        (0 << 4)    /**< I2C1 clock is HSI.        */
#define STM32_I2C1SW_SYSCLK     (1 << 4)    /**< I2C1 clock is SYSCLK.     */
#define STM32_I2C2SW_MASK       (1 << 5)    /**< I2C2 clock source mask.   */
#define STM32_I2C2SW_HSI        (0 << 5)    /**< I2C2 clock is HSI.        */
#define STM32_I2C2SW_SYSCLK     (1 << 5)    /**< I2C2 clock is SYSCLK.     */
#define STM32_TIM1SW_MASK       (1 << 8)    /**< TIM1 clock source mask.   */
#define STM32_TIM1SW_PCLK2      (0 << 8)    /**< TIM1 clock is PCLK2.      */
#define STM32_TIM1SW_PLLX2      (1 << 8)    /**< TIM1 clock is PLL*2.      */
#define STM32_TIM8SW_MASK       (1 << 9)    /**< TIM8 clock source mask.   */
#define STM32_TIM8SW_PCLK2      (0 << 9)    /**< TIM8 clock is PCLK2.      */
#define STM32_TIM8SW_PLLX2      (1 << 9)    /**< TIM8 clock is PLL*2.      */
#define STM32_HRTIM1SW_MASK     (1 << 12)   /**< HRTIM1 clock source mask. */
#define STM32_HRTIM1SW_PCLK2    (0 << 12)   /**< HRTIM1 clock is PCLK2.    */
#define STM32_HRTIM1SW_PLLX2    (1 << 12)   /**< HRTIM1 clock is PLL*2.    */
#define STM32_USART2SW_MASK     (3 << 16)   /**< USART2 clock source mask. */
#define STM32_USART2SW_PCLK     (0 << 16)   /**< USART2 clock is PCLK.     */
#define STM32_USART2SW_SYSCLK   (1 << 16)   /**< USART2 clock is SYSCLK.   */
#define STM32_USART2SW_LSE      (2 << 16)   /**< USART2 clock is LSE.      */
#define STM32_USART2SW_HSI      (3 << 16)   /**< USART2 clock is HSI.      */
#define STM32_USART3SW_MASK     (3 << 18)   /**< USART3 clock source mask. */
#define STM32_USART3SW_PCLK     (0 << 18)   /**< USART3 clock is PCLK.     */
#define STM32_USART3SW_SYSCLK   (1 << 18)   /**< USART3 clock is SYSCLK.   */
#define STM32_USART3SW_LSE      (2 << 18)   /**< USART3 clock is LSE.      */
#define STM32_USART3SW_HSI      (3 << 18)   /**< USART3 clock is HSI.      */
#define STM32_UART4SW_MASK      (3 << 20)   /**< USART4 clock source mask. */
#define STM32_UART4SW_PCLK      (0 << 20)   /**< USART4 clock is PCLK.     */
#define STM32_UART4SW_SYSCLK    (1 << 20)   /**< USART4 clock is SYSCLK.   */
#define STM32_UART4SW_LSE       (2 << 20)   /**< USART4 clock is LSE.      */
#define STM32_UART4SW_HSI       (3 << 20)   /**< USART4 clock is HSI.      */
#define STM32_UART5SW_MASK      (3 << 22)   /**< USART5 clock source mask. */
#define STM32_UART5SW_PCLK      (0 << 22)   /**< USART5 clock is PCLK.     */
#define STM32_UART5SW_SYSCLK    (1 << 22)   /**< USART5 clock is SYSCLK.   */
#define STM32_UART5SW_LSE       (2 << 22)   /**< USART5 clock is LSE.      */
#define STM32_UART5SW_HSI       (3 << 22)   /**< USART5 clock is HSI.      */
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
 * @brief   Enables or disables the HSI clock source.
 */
#if !defined(STM32_HSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI_ENABLED                   TRUE
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
#define STM32_HSE_ENABLED                   TRUE
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
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using the PLL.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                            STM32_SW_PLL
#endif

/**
 * @brief   Clock source for the PLL.
 * @note    This setting has only effect if the PLL is selected as the
 *          system clock source.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using the PLL.
 */
#if !defined(STM32_PLLSRC) || defined(__DOXYGEN__)
#define STM32_PLLSRC                        STM32_PLLSRC_HSE
#endif

/**
 * @brief   Crystal PLL pre-divider.
 * @note    This setting has only effect if the PLL is selected as the
 *          system clock source.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using the PLL.
 */
#if !defined(STM32_PREDIV_VALUE) || defined(__DOXYGEN__)
#define STM32_PREDIV_VALUE                  1
#endif

/**
 * @brief   PLL multiplier value.
 * @note    The allowed range is 2...16.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using the PLL.
 */
#if !defined(STM32_PLLMUL_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLMUL_VALUE                  9
#endif

/**
 * @brief   AHB prescaler value.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using the PLL.
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
#define STM32_PPRE2                         STM32_PPRE2_DIV2
#endif

/**
 * @brief   MCO pin setting.
 */
#if !defined(STM32_MCOSEL) || defined(__DOXYGEN__)
#define STM32_MCOSEL                        STM32_MCOSEL_NOCLOCK
#endif

/**
 * @brief   ADC12 prescaler value.
 */
#if !defined(STM32_ADC12PRES) || defined(__DOXYGEN__)
#define STM32_ADC12PRES                     STM32_ADC12PRES_DIV1
#endif

/**
 * @brief   ADC34 prescaler value.
 */
#if !defined(STM32_ADC34PRES) || defined(__DOXYGEN__)
#define STM32_ADC34PRES                     STM32_ADC34PRES_DIV1
#endif

/**
 * @brief   USART1 clock source.
 */
#if !defined(STM32_USART1SW) || defined(__DOXYGEN__)
#define STM32_USART1SW                      STM32_USART1SW_PCLK
#endif

/**
 * @brief   USART2 clock source.
 */
#if !defined(STM32_USART2SW) || defined(__DOXYGEN__)
#define STM32_USART2SW                      STM32_USART2SW_PCLK
#endif

/**
 * @brief   USART3 clock source.
 */
#if !defined(STM32_USART3SW) || defined(__DOXYGEN__)
#define STM32_USART3SW                      STM32_USART3SW_PCLK
#endif

/**
 * @brief   UART4 clock source.
 */
#if !defined(STM32_UART4SW) || defined(__DOXYGEN__)
#define STM32_UART4SW                       STM32_UART4SW_PCLK
#endif

/**
 * @brief   UART5 clock source.
 */
#if !defined(STM32_UART5SW) || defined(__DOXYGEN__)
#define STM32_UART5SW                       STM32_UART5SW_PCLK
#endif

/**
 * @brief   I2C1 clock source.
 */
#if !defined(STM32_I2C1SW) || defined(__DOXYGEN__)
#define STM32_I2C1SW                        STM32_I2C1SW_SYSCLK
#endif

/**
 * @brief   I2C2 clock source.
 */
#if !defined(STM32_I2C2SW) || defined(__DOXYGEN__)
#define STM32_I2C2SW                        STM32_I2C2SW_SYSCLK
#endif

/**
 * @brief   TIM1 clock source.
 */
#if !defined(STM32_TIM1SW) || defined(__DOXYGEN__)
#define STM32_TIM1SW                        STM32_TIM1SW_PCLK2
#endif

/**
 * @brief   TIM8 clock source.
 */
#if !defined(STM32_TIM8SW) || defined(__DOXYGEN__)
#define STM32_TIM8SW                        STM32_TIM8SW_PCLK2
#endif

/**
 * @brief   HRTIM1 clock source.
 */
#if !defined(STM32_HRTIM1SW) || defined(__DOXYGEN__)
#define STM32_HRTIM1SW                      STM32_HRTIM1SW_PCLK2
#endif

/**
 * @brief   RTC clock source.
 */
#if !defined(STM32_RTCSEL) || defined(__DOXYGEN__)
#define STM32_RTCSEL                        STM32_RTCSEL_LSI
#endif

/**
 * @brief   USB clock setting.
 */
#if !defined(STM32_USB_CLOCK_REQUIRED) || defined(__DOXYGEN__)
#define STM32_USB_CLOCK_REQUIRED            TRUE
#endif

/**
 * @brief   USB prescaler initialization.
 */
#if !defined(STM32_USBPRE) || defined(__DOXYGEN__)
#define STM32_USBPRE                        STM32_USBPRE_DIV1P5
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(STM32F3xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F3xx_MCUCONF not defined"
#endif

/* Only some devices have strongly checked mcuconf.h files. Others will be
   added gradually.*/
#if (defined(STM32F303xC) || defined(STM32F303xE)) &&                       \
    !defined(STM32F303_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F303_MCUCONF not defined"
#endif

/*
 * HSI related checks.
 */
#if STM32_HSI_ENABLED
#else /* !STM32_HSI_ENABLED */

#if STM32_SW == STM32_SW_HSI
#error "HSI not enabled, required by STM32_SW"
#endif

#if STM32_USART1SW == STM32_USART1SW_HSI
#error "HSI not enabled, required by STM32_USART1SW"
#endif

#if STM32_USART2SW == STM32_USART2SW_HSI
#error "HSI not enabled, required by STM32_USART2SW"
#endif

#if STM32_USART3SW == STM32_USART3SW_HSI
#error "HSI not enabled, required by STM32_USART3SW"
#endif

#if STM32_UART4SW == STM32_UART4SW_HSI
#error "HSI not enabled, required by STM32_UART4SW"
#endif

#if STM32_UART5SW == STM32_UART5SW_HSI
#error "HSI not enabled, required by STM32_UART5SW"
#endif

#if STM32_I2C1SW == STM32_I2C1SW_HSI
#error "HSI not enabled, required by STM32_I2C1SW"
#endif

#if STM32_I2C2SW == STM32_I2C2SW_HSI
#error "HSI not enabled, required by STM32_I2C2SW"
#endif

#if (STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSI)
#error "HSI not enabled, required by STM32_SW and STM32_PLLSRC"
#endif

#if (STM32_MCOSEL == STM32_MCOSEL_HSI) ||                                   \
    ((STM32_MCOSEL == STM32_MCOSEL_PLLDIV2) &&                              \
     (STM32_PLLSRC == STM32_PLLSRC_HSI))
#error "HSI not enabled, required by STM32_MCOSEL"
#endif

#endif /* !STM32_HSI_ENABLED */

/*
 * HSE related checks.
 */
#if STM32_HSE_ENABLED

#if STM32_HSECLK == 0
#error "HSE frequency not defined"
#elif (STM32_HSECLK < STM32_HSECLK_MIN) || (STM32_HSECLK > STM32_HSECLK_MAX)
#error "STM32_HSECLK outside acceptable range (STM32_HSECLK_MIN...STM32_HSECLK_MAX)"
#endif

#else /* !STM32_HSE_ENABLED */

#if STM32_SW == STM32_SW_HSE
#error "HSE not enabled, required by STM32_SW"
#endif

#if (STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSE)
#error "HSE not enabled, required by STM32_SW and STM32_PLLSRC"
#endif

#if (STM32_MCOSEL == STM32_MCOSEL_HSE) ||                                   \
    ((STM32_MCOSEL == STM32_MCOSEL_PLLDIV2) &&                              \
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

#endif /* !STM32_LSI_ENABLED */

/*
 * LSE related checks.
 */
#if STM32_LSE_ENABLED

#if !defined(STM32_LSECLK) || (STM32_LSECLK == 0)
#error "STM32_LSECLK not defined"
#endif

#if (STM32_LSECLK < STM32_LSECLK_MIN) || (STM32_LSECLK > STM32_LSECLK_MAX)
#error "STM32_LSECLK outside acceptable range (STM32_LSECLK_MIN...STM32_LSECLK_MAX)"
#endif

#if !defined(STM32_LSEDRV)
#error "STM32_LSEDRV not defined"
#endif

#if (STM32_LSEDRV >> 3) > 3
#error "STM32_LSEDRV outside acceptable range ((0<<3)...(3<<3))"
#endif

#if STM32_USART1SW == STM32_USART1SW_LSE
#error "LSE not enabled, required by STM32_USART1SW"
#endif

#if STM32_USART2SW == STM32_USART2SW_LSE
#error "LSE not enabled, required by STM32_USART2SW"
#endif

#if STM32_USART3SW == STM32_USART3SW_LSE
#error "LSE not enabled, required by STM32_USART3SW"
#endif

#if STM32_UART4SW == STM32_UART4SW_LSE
#error "LSE not enabled, required by STM32_UART4SW"
#endif

#if STM32_UART5SW == STM32_UART5SW_LSE
#error "LSE not enabled, required by STM32_UART5SW"
#endif

#else /* !STM32_LSE_ENABLED */

#if STM32_RTCSEL == STM32_RTCSEL_LSE
#error "LSE not enabled, required by STM32_RTCSEL"
#endif

#endif /* !STM32_LSE_ENABLED */

/* PLL activation conditions.*/
#if (STM32_SW == STM32_SW_PLL) ||                                           \
    (STM32_MCOSEL == STM32_MCOSEL_PLLDIV2) ||                               \
    (STM32_TIM1SW == STM32_TIM1SW_PLLX2) ||                                 \
    (STM32_TIM8SW == STM32_TIM8SW_PLLX2) ||                                 \
    (STM32_ADC12PRES != STM32_ADC12PRES_NOCLOCK) ||                         \
    (STM32_ADC34PRES != STM32_ADC34PRES_NOCLOCK) ||                         \
    STM32_USB_CLOCK_REQUIRED ||                                             \
    defined(__DOXYGEN__)
/**
 * @brief   PLL activation flag.
 */
#define STM32_ACTIVATE_PLL          TRUE
#else
#define STM32_ACTIVATE_PLL          FALSE
#endif

/* HSE prescaler setting check.*/
#if ((STM32_PREDIV_VALUE >= 1) || (STM32_PREDIV_VALUE <= 16))
#define STM32_PREDIV                ((STM32_PREDIV_VALUE - 1) << 0)
#else
#error "invalid STM32_PREDIV value specified"
#endif

/**
 * @brief   PLLMUL field.
 */
#if ((STM32_PLLMUL_VALUE >= 2) && (STM32_PLLMUL_VALUE <= 16)) ||            \
    defined(__DOXYGEN__)
#define STM32_PLLMUL                ((STM32_PLLMUL_VALUE - 2) << 18)
#else
#error "invalid STM32_PLLMUL_VALUE value specified"
#endif

/**
 * @brief   PLL input clock frequency.
 */
#if (STM32_PLLSRC == STM32_PLLSRC_HSE) || defined(__DOXYGEN__)
#define STM32_PLLCLKIN              (STM32_HSECLK / STM32_PREDIV_VALUE)
#elif STM32_PLLSRC == STM32_PLLSRC_HSI
#define STM32_PLLCLKIN              (STM32_HSICLK / 2)
#else
#error "invalid STM32_PLLSRC value specified"
#endif

/* PLL input frequency range check.*/
#if (STM32_PLLCLKIN < STM32_PLLIN_MIN) || (STM32_PLLCLKIN > STM32_PLLIN_MAX)
#error "STM32_PLLCLKIN outside acceptable range (STM32_PLLIN_MIN...STM32_PLLIN_MAX)"
#endif

/**
 * @brief   PLL output clock frequency.
 */
#define STM32_PLLCLKOUT             (STM32_PLLCLKIN * STM32_PLLMUL_VALUE)

/* PLL output frequency range check.*/
#if (STM32_PLLCLKOUT < STM32_PLLOUT_MIN) || (STM32_PLLCLKOUT > STM32_PLLOUT_MAX)
#error "STM32_PLLCLKOUT outside acceptable range (STM32_PLLOUT_MIN...STM32_PLLOUT_MAX)"
#endif

/**
 * @brief   System clock source.
 */
#if (STM32_SW == STM32_SW_PLL) || defined(__DOXYGEN__)
#define STM32_SYSCLK                STM32_PLLCLKOUT
#elif (STM32_SW == STM32_SW_HSI)
#define STM32_SYSCLK                STM32_HSICLK
#elif (STM32_SW == STM32_SW_HSE)
#define STM32_SYSCLK                STM32_HSECLK
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
#define STM32_PCLK1                  (STM32_HCLK / 1)
#elif STM32_PPRE1 == STM32_PPRE1_DIV2
#define STM32_PCLK1                  (STM32_HCLK / 2)
#elif STM32_PPRE1 == STM32_PPRE1_DIV4
#define STM32_PCLK1                  (STM32_HCLK / 4)
#elif STM32_PPRE1 == STM32_PPRE1_DIV8
#define STM32_PCLK1                  (STM32_HCLK / 8)
#elif STM32_PPRE1 == STM32_PPRE1_DIV16
#define STM32_PCLK1                  (STM32_HCLK / 16)
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
#define STM32_PCLK2                  (STM32_HCLK / 1)
#elif STM32_PPRE2 == STM32_PPRE2_DIV2
#define STM32_PCLK2                  (STM32_HCLK / 2)
#elif STM32_PPRE2 == STM32_PPRE2_DIV4
#define STM32_PCLK2                  (STM32_HCLK / 4)
#elif STM32_PPRE2 == STM32_PPRE2_DIV8
#define STM32_PCLK2                  (STM32_HCLK / 8)
#elif STM32_PPRE2 == STM32_PPRE2_DIV16
#define STM32_PCLK2                  (STM32_HCLK / 16)
#else
#error "invalid STM32_PPRE2 value specified"
#endif

/* APB2 frequency check.*/
#if STM32_PCLK2 > STM32_PCLK2_MAX
#error "STM32_PCLK2 exceeding maximum frequency (STM32_PCLK2_MAX)"
#endif

/**
 * @brief   RTC clock.
 */
#if (STM32_RTCSEL == STM32_RTCSEL_LSE) || defined(__DOXYGEN__)
#define STM32_RTCCLK                STM32_LSECLK
#elif STM32_RTCSEL == STM32_RTCSEL_LSI
#define STM32_RTCCLK                STM32_LSICLK
#elif STM32_RTCSEL == STM32_RTCSEL_HSEDIV
#define STM32_RTCCLK                (STM32_HSECLK / 32)
#elif STM32_RTCSEL == STM32_RTCSEL_NOCLOCK
#define STM32_RTCCLK                0
#else
#error "invalid source selected for RTC clock"
#endif

/**
 * @brief   ADC12 frequency.
 */
#if (STM32_ADC12PRES == STM32_ADC12PRES_NOCLOCK) || defined(__DOXYGEN__)
#define STM32_ADC12CLK              0
#elif STM32_ADC12PRES == STM32_ADC12PRES_DIV1
#define STM32_ADC12CLK              (STM32_PLLCLKOUT / 1)
#elif STM32_ADC12PRES == STM32_ADC12PRES_DIV2
#define STM32_ADC12CLK              (STM32_PLLCLKOUT / 2)
#elif STM32_ADC12PRES == STM32_ADC12PRES_DIV4
#define STM32_ADC12CLK              (STM32_PLLCLKOUT / 4)
#elif STM32_ADC12PRES == STM32_ADC12PRES_DIV6
#define STM32_ADC12CLK              (STM32_PLLCLKOUT / 6)
#elif STM32_ADC12PRES == STM32_ADC12PRES_DIV8
#define STM32_ADC12CLK              (STM32_PLLCLKOUT / 8)
#elif STM32_ADC12PRES == STM32_ADC12PRES_DIV10
#define STM32_ADC12CLK              (STM32_PLLCLKOUT / 10)
#elif STM32_ADC12PRES == STM32_ADC12PRES_DIV12
#define STM32_ADC12CLK              (STM32_PLLCLKOUT / 12)
#elif STM32_ADC12PRES == STM32_ADC12PRES_DIV16
#define STM32_ADC12CLK              (STM32_PLLCLKOUT / 16)
#elif STM32_ADC12PRES == STM32_ADC12PRES_DIV32
#define STM32_ADC12CLK              (STM32_PLLCLKOUT / 32)
#elif STM32_ADC12PRES == STM32_ADC12PRES_DIV64
#define STM32_ADC12CLK              (STM32_PLLCLKOUT / 64)
#elif STM32_ADC12PRES == STM32_ADC12PRES_DIV128
#define STM32_ADC12CLK              (STM32_PLLCLKOUT / 128)
#elif STM32_ADC12PRES == STM32_ADC12PRES_DIV256
#define STM32_ADC12CLK              (STM32_PLLCLKOUT / 256)
#else
#error "invalid STM32_ADC12PRES value specified"
#endif

/**
 * @brief   ADC34 frequency.
 */
#if (STM32_ADC34PRES == STM32_ADC34PRES_NOCLOCK) || defined(__DOXYGEN__)
#define STM32_ADC34CLK              0
#elif STM32_ADC34PRES == STM32_ADC34PRES_DIV1
#define STM32_ADC34CLK              (STM32_PLLCLKOUT / 1)
#elif STM32_ADC34PRES == STM32_ADC34PRES_DIV2
#define STM32_ADC34CLK              (STM32_PLLCLKOUT / 2)
#elif STM32_ADC34PRES == STM32_ADC34PRES_DIV4
#define STM32_ADC34CLK              (STM32_PLLCLKOUT / 4)
#elif STM32_ADC34PRES == STM32_ADC34PRES_DIV6
#define STM32_ADC34CLK              (STM32_PLLCLKOUT / 6)
#elif STM32_ADC34PRES == STM32_ADC34PRES_DIV8
#define STM32_ADC34CLK              (STM32_PLLCLKOUT / 8)
#elif STM32_ADC34PRES == STM32_ADC34PRES_DIV10
#define STM32_ADC34CLK              (STM32_PLLCLKOUT / 10)
#elif STM32_ADC34PRES == STM32_ADC34PRES_DIV12
#define STM32_ADC34CLK              (STM32_PLLCLKOUT / 12)
#elif STM32_ADC34PRES == STM32_ADC34PRES_DIV16
#define STM32_ADC34CLK              (STM32_PLLCLKOUT / 16)
#elif STM32_ADC34PRES == STM32_ADC34PRES_DIV32
#define STM32_ADC34CLK              (STM32_PLLCLKOUT / 32)
#elif STM32_ADC34PRES == STM32_ADC34PRES_DIV64
#define STM32_ADC34CLK              (STM32_PLLCLKOUT / 64)
#elif STM32_ADC34PRES == STM32_ADC34PRES_DIV128
#define STM32_ADC34CLK              (STM32_PLLCLKOUT / 128)
#elif STM32_ADC34PRES == STM32_ADC34PRES_DIV256
#define STM32_ADC34CLK              (STM32_PLLCLKOUT / 256)
#else
#error "invalid STM32_ADC34PRES value specified"
#endif

/* ADC12 frequency check.*/
#if STM32_ADC12CLK > STM32_ADCCLK_MAX
#error "STM32_ADC12CLK exceeding maximum frequency (STM32_ADCCLK_MAX)"
#endif

/* ADC34 frequency check.*/
#if STM32_ADC34CLK > STM32_ADCCLK_MAX
#error "STM32_ADC34CLK exceeding maximum frequency (STM32_ADCCLK_MAX)"
#endif

/**
 * @brief   I2C1 frequency.
 */
#if STM32_I2C1SW == STM32_I2C1SW_HSI
#define STM32_I2C1CLK               STM32_HSICLK
#elif STM32_I2C1SW == STM32_I2C1SW_SYSCLK
#define STM32_I2C1CLK               STM32_SYSCLK
#else
#error "invalid source selected for I2C1 clock"
#endif

/**
 * @brief   I2C2 frequency.
 */
#if STM32_I2C2SW == STM32_I2C2SW_HSI
#define STM32_I2C2CLK               STM32_HSICLK
#elif STM32_I2C2SW == STM32_I2C2SW_SYSCLK
#define STM32_I2C2CLK               STM32_SYSCLK
#else
#error "invalid source selected for I2C2 clock"
#endif

/**
 * @brief   USART1 frequency.
 */
#if STM32_USART1SW == STM32_USART1SW_PCLK
#define STM32_USART1CLK             STM32_PCLK2
#elif STM32_USART1SW == STM32_USART1SW_SYSCLK
#define STM32_USART1CLK             STM32_SYSCLK
#elif STM32_USART1SW == STM32_USART1SW_LSE
#define STM32_USART1CLK             STM32_LSECLK
#elif STM32_USART1SW == STM32_USART1SW_HSI
#define STM32_USART1CLK             STM32_HSICLK
#else
#error "invalid source selected for USART1 clock"
#endif

/**
 * @brief   USART2 frequency.
 */
#if STM32_USART2SW == STM32_USART2SW_PCLK
#define STM32_USART2CLK             STM32_PCLK1
#elif STM32_USART2SW == STM32_USART2SW_SYSCLK
#define STM32_USART2CLK             STM32_SYSCLK
#elif STM32_USART2SW == STM32_USART2SW_LSE
#define STM32_USART2CLK             STM32_LSECLK
#elif STM32_USART2SW == STM32_USART2SW_HSI
#define STM32_USART2CLK             STM32_HSICLK
#else
#error "invalid source selected for USART2 clock"
#endif

/**
 * @brief   USART3 frequency.
 */
#if STM32_USART3SW == STM32_USART3SW_PCLK
#define STM32_USART3CLK             STM32_PCLK1
#elif STM32_USART3SW == STM32_USART3SW_SYSCLK
#define STM32_USART3CLK             STM32_SYSCLK
#elif STM32_USART3SW == STM32_USART3SW_LSE
#define STM32_USART3CLK             STM32_LSECLK
#elif STM32_USART3SW == STM32_USART3SW_HSI
#define STM32_USART3CLK             STM32_HSICLK
#else
#error "invalid source selected for USART3 clock"
#endif

/**
 * @brief   UART4 frequency.
 */
#if STM32_UART4SW == STM32_UART4SW_PCLK
#define STM32_UART4CLK             STM32_PCLK1
#elif STM32_UART4SW == STM32_UART4SW_SYSCLK
#define STM32_UART4CLK             STM32_SYSCLK
#elif STM32_UART4SW == STM32_UART4SW_LSE
#define STM32_UART4CLK             STM32_LSECLK
#elif STM32_UART4SW == STM32_UART4SW_HSI
#define STM32_UART4CLK             STM32_HSICLK
#else
#error "invalid source selected for UART4 clock"
#endif

/**
 * @brief   UART5 frequency.
 */
#if STM32_UART5SW == STM32_UART5SW_PCLK
#define STM32_UART5CLK             STM32_PCLK1
#elif STM32_UART5SW == STM32_UART5SW_SYSCLK
#define STM32_UART5CLK             STM32_SYSCLK
#elif STM32_UART5SW == STM32_UART5SW_LSE
#define STM32_UART5CLK             STM32_LSECLK
#elif STM32_UART5SW == STM32_UART5SW_HSI
#define STM32_UART5CLK             STM32_HSICLK
#else
#error "invalid source selected for UART5 clock"
#endif

/**
 * @brief   TIM1 frequency.
 */
#if STM32_TIM1SW == STM32_TIM1SW_PCLK2
#if STM32_PPRE2 == STM32_PPRE2_DIV1
#define STM32_TIM1CLK               STM32_PCLK2
#else
#define STM32_TIM1CLK               (STM32_PCLK2 * 2)
#endif

#elif STM32_TIM1SW == STM32_TIM1SW_PLLX2
#if (STM32_SW != STM32_SW_PLL) ||                                           \
    (STM32_HPRE != STM32_HPRE_DIV1) ||                                      \
    (STM32_PPRE2 != STM32_PPRE2_DIV1)
#error "double clock mode cannot be activated for TIM1 under the current settings"
#endif
#define STM32_TIM1CLK               (STM32_PLLCLKOUT * 2)

#else
#error "invalid source selected for TIM1 clock"
#endif

/**
 * @brief   TIM8 frequency.
 */
#if STM32_TIM8SW == STM32_TIM8SW_PCLK2
#if STM32_PPRE2 == STM32_PPRE2_DIV1
#define STM32_TIM8CLK               STM32_PCLK2
#else
#define STM32_TIM8CLK               (STM32_PCLK2 * 2)
#endif

#elif STM32_TIM8SW == STM32_TIM8SW_PLLX2
#if (STM32_SW != STM32_SW_PLL) ||                                           \
    (STM32_HPRE != STM32_HPRE_DIV1) ||                                      \
    (STM32_PPRE2 != STM32_PPRE2_DIV1)
#error "double clock mode cannot be activated for TIM8 under the current settings"
#endif
#define STM32_TIM8CLK               (STM32_PLLCLKOUT * 2)

#else
#error "invalid source selected for TIM8 clock"
#endif

/**
 * @brief   HRTIM1 frequency.
 */
#if STM32_HRTIM1SW == STM32_HRTIM1SW_PCLK2
#if STM32_PPRE2 == STM32_PPRE2_DIV1
#define STM32_HRTIM1CLK               STM32_PCLK2
#else
#define STM32_HRTIM1CLK               (STM32_PCLK2 * 2)
#endif

#elif STM32_HRTIM1SW == STM32_HRTIM1SW_PLLX2
#if (STM32_SW != STM32_SW_PLL) ||                                           \
    (STM32_HPRE != STM32_HPRE_DIV1) ||                                      \
    (STM32_PPRE2 != STM32_PPRE2_DIV1)
#error "double clock mode cannot be activated for HRTIM1 under the current settings"
#endif
#define STM32_HRTIM1CLK               (STM32_PLLCLKOUT * 2)

#else
#error "invalid source selected for HRTIM1 clock"
#endif

/**
 * @brief   Timers 2, 3, 4, 6, 7 frequency.
 */
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK1               (STM32_PCLK1 * 1)
#else
#define STM32_TIMCLK1               (STM32_PCLK1 * 2)
#endif

/**
 * @brief   Timers 1, 8, 15, 16, 17 frequency.
 */
#if (STM32_PPRE2 == STM32_PPRE2_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK2               (STM32_PCLK2 * 1)
#else
#define STM32_TIMCLK2               (STM32_PCLK2 * 2)
#endif

/**
 * @brief   USB frequency.
 */
#if (STM32_USBPRE == STM32_USBPRE_DIV1P5) || defined(__DOXYGEN__)
#define STM32_USBCLK                ((STM32_PLLCLKOUT * 2) / 3)
#elif (STM32_USBPRE == STM32_USBPRE_DIV1)
#define STM32_USBCLK                STM32_PLLCLKOUT
#else
#error "invalid STM32_USBPRE value specified"
#endif

/**
 * @brief   Flash settings.
 */
#if (STM32_HCLK <= 24000000) || defined(__DOXYGEN__)
#define STM32_FLASHBITS             0x00000010
#elif STM32_HCLK <= 48000000
#define STM32_FLASHBITS             0x00000011
#else
#define STM32_FLASHBITS             0x00000012
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
