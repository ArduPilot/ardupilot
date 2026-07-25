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
 * @file    STM32F7xx/hal_lld.h
 * @brief   STM32F7xx HAL subsystem low level driver header.
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
 *          - STM32F722xx, STM32F723xx very high-performance MCUs.
 *          - STM32F732xx, STM32F733xx very high-performance MCUs.
 *          - STM32F745xx, STM32F746xx, STM32F756xx very high-performance MCUs.
 *          - STM32F765xx, STM32F767xx, STM32F769xx very high-performance MCUs.
 *          - STM32F777xx, STM32F779xx very high-performance MCUs.
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
#if defined(STM32F722xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32F722 Very High Performance with DSP and FPU"

#elif defined(STM32F723xx)
#define PLATFORM_NAME           "STM32F723 Very High Performance with DSP and FPU"

#elif defined(STM32F732xx)
#define PLATFORM_NAME           "STM32F732 Very High Performance with DSP and FPU"

#elif defined(STM32F733xx)
#define PLATFORM_NAME           "STM32F733 Very High Performance with DSP and FPU"

#elif defined(STM32F745xx)
#define PLATFORM_NAME           "STM32F745 Very High Performance with DSP and FPU"

#elif defined(STM32F746xx)
#define PLATFORM_NAME           "STM32F746 Very High Performance with DSP and FPU"

#elif defined(STM32F756xx)
#define PLATFORM_NAME           "STM32F756 Very High Performance with DSP and FPU"

#elif defined(STM32F765xx)
#define PLATFORM_NAME           "STM32F767 Very High Performance with DSP and DP FPU"

#elif defined(STM32F767xx)
#define PLATFORM_NAME           "STM32F767 Very High Performance with DSP and DP FPU"

#elif defined(STM32F769xx)
#define PLATFORM_NAME           "STM32F769 Very High Performance with DSP and DP FPU"

#elif defined(STM32F777xx)
#define PLATFORM_NAME           "STM32F767 Very High Performance with DSP and DP FPU"

#elif defined(STM32F779xx)
#define PLATFORM_NAME           "STM32F769 Very High Performance with DSP and DP FPU"

#else
#error "STM32F7xx device not specified"
#endif
/** @} */

/**
 * @name    Sub-family identifier
 */
#if !defined(STM32F7XX) || defined(__DOXYGEN__)
#define STM32F7XX
#endif
/** @} */

/**
 * @name    Absolute Maximum Ratings
 * @{
 */
/**
 * @brief   Absolute maximum system clock.
 */
#define STM32_SYSCLK_MAX        216000000

/**
 * @brief   Maximum HSE clock frequency.
 */
#define STM32_HSECLK_MAX        26000000

/**
 * @brief   Maximum HSE clock frequency using an external source.
 */
#define STM32_HSECLK_BYP_MAX    50000000

/**
 * @brief   Minimum HSE clock frequency.
 */
#define STM32_HSECLK_MIN        4000000

/**
 * @brief   Minimum HSE clock frequency.
 */
#define STM32_HSECLK_BYP_MIN    1000000

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_MAX        32768

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_BYP_MAX    1000000

/**
 * @brief   Minimum LSE clock frequency.
 */
#define STM32_LSECLK_MIN        32768

/**
 * @brief   Maximum PLLs input clock frequency.
 */
#define STM32_PLLIN_MAX         2100000

/**
 * @brief   Minimum PLLs input clock frequency.
 */
#define STM32_PLLIN_MIN         950000

/**
 * @brief   Maximum PLLs VCO clock frequency.
 */
#define STM32_PLLVCO_MAX        432000000

/**
 * @brief   Minimum PLLs VCO clock frequency.
 * @note    This value is specified at 192MHz in the DS and 100MHz in the RM,
 *          using the least restrictive one.
 */
#define STM32_PLLVCO_MIN        100000000

/**
 * @brief   Maximum PLL output clock frequency.
 */
#define STM32_PLLOUT_MAX        216000000

/**
 * @brief   Minimum PLL output clock frequency.
 */
#define STM32_PLLOUT_MIN        24000000

/**
 * @brief   Maximum APB1 clock frequency.
 */
#define STM32_PCLK1_MAX         (STM32_PLLOUT_MAX / 4)

/**
 * @brief   Maximum APB2 clock frequency.
 */
#define STM32_PCLK2_MAX         (STM32_PLLOUT_MAX / 2)

/**
 * @brief   Maximum SPI/I2S clock frequency.
 */
#define STM32_SPII2S_MAX        54000000
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define STM32_HSICLK            16000000    /**< High speed internal clock. */
#define STM32_LSICLK            32000       /**< Low speed internal clock.  */
/** @} */

/**
 * @name    PWR_CR register bits definitions
 * @{
 */
#define STM32_VOS_SCALE3        (PWR_CR1_VOS_0)
#define STM32_VOS_SCALE2        (PWR_CR1_VOS_1)
#define STM32_VOS_SCALE1        (PWR_CR1_VOS_1 | PWR_CR1_VOS_0)

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
 * @name    RCC_PLLCFGR register bits definitions
 * @{
 */
#define STM32_PLLP_MASK         (3 << 16)   /**< PLLP mask.                 */
#define STM32_PLLP_DIV2         (0 << 16)   /**< PLL clock divided by 2.    */
#define STM32_PLLP_DIV4         (1 << 16)   /**< PLL clock divided by 4.    */
#define STM32_PLLP_DIV6         (2 << 16)   /**< PLL clock divided by 6.    */
#define STM32_PLLP_DIV8         (3 << 16)   /**< PLL clock divided by 8.    */

#define STM32_PLLSRC_HSI        (0 << 22)   /**< PLL clock source is HSI.   */
#define STM32_PLLSRC_HSE        (1 << 22)   /**< PLL clock source is HSE.   */
/** @} */

/**
 * @name    RCC_CFGR register bits definitions
 * @{
 */
#define STM32_SW_MASK           (3 << 0)    /**< SW mask.                   */
#define STM32_SW_HSI            (0 << 0)    /**< SYSCLK source is HSI.      */
#define STM32_SW_HSE            (1 << 0)    /**< SYSCLK source is HSE.      */
#define STM32_SW_PLL            (2 << 0)    /**< SYSCLK source is PLL.      */

#define STM32_HPRE_MASK         (15 << 4)   /**< HPRE mask.                 */
#define STM32_HPRE_DIV1         (0 << 4)    /**< SYSCLK divided by 1.       */
#define STM32_HPRE_DIV2         (8 << 4)    /**< SYSCLK divided by 2.       */
#define STM32_HPRE_DIV4         (9 << 4)    /**< SYSCLK divided by 4.       */
#define STM32_HPRE_DIV8         (10 << 4)   /**< SYSCLK divided by 8.       */
#define STM32_HPRE_DIV16        (11 << 4)   /**< SYSCLK divided by 16.      */
#define STM32_HPRE_DIV64        (12 << 4)   /**< SYSCLK divided by 64.      */
#define STM32_HPRE_DIV128       (13 << 4)   /**< SYSCLK divided by 128.     */
#define STM32_HPRE_DIV256       (14 << 4)   /**< SYSCLK divided by 256.     */
#define STM32_HPRE_DIV512       (15 << 4)   /**< SYSCLK divided by 512.     */

#define STM32_PPRE1_MASK        (7 << 10)   /**< PPRE1 mask.                */
#define STM32_PPRE1_DIV1        (0 << 10)   /**< HCLK divided by 1.         */
#define STM32_PPRE1_DIV2        (4 << 10)   /**< HCLK divided by 2.         */
#define STM32_PPRE1_DIV4        (5 << 10)   /**< HCLK divided by 4.         */
#define STM32_PPRE1_DIV8        (6 << 10)   /**< HCLK divided by 8.         */
#define STM32_PPRE1_DIV16       (7 << 10)   /**< HCLK divided by 16.        */

#define STM32_PPRE2_MASK        (7 << 13)   /**< PPRE2 mask.                */
#define STM32_PPRE2_DIV1        (0 << 13)   /**< HCLK divided by 1.         */
#define STM32_PPRE2_DIV2        (4 << 13)   /**< HCLK divided by 2.         */
#define STM32_PPRE2_DIV4        (5 << 13)   /**< HCLK divided by 4.         */
#define STM32_PPRE2_DIV8        (6 << 13)   /**< HCLK divided by 8.         */
#define STM32_PPRE2_DIV16       (7 << 13)   /**< HCLK divided by 16.        */

#define STM32_RTCPRE_MASK       (31 << 16)  /**< RTCPRE mask.               */

#define STM32_MCO1SEL_MASK      (3 << 21)   /**< MCO1 mask.                 */
#define STM32_MCO1SEL_HSI       (0 << 21)   /**< HSI clock on MCO1 pin.     */
#define STM32_MCO1SEL_LSE       (1 << 21)   /**< LSE clock on MCO1 pin.     */
#define STM32_MCO1SEL_HSE       (2 << 21)   /**< HSE clock on MCO1 pin.     */
#define STM32_MCO1SEL_PLL       (3 << 21)   /**< PLL clock on MCO1 pin.     */

#define STM32_I2SSRC_MASK       (1 << 23)   /**< I2CSRC mask.               */
#define STM32_I2SSRC_PLLI2S     (0 << 23)   /**< I2SSRC is PLLI2S.          */
#define STM32_I2SSRC_CKIN       (1 << 23)   /**< I2S_CKIN is PLLI2S.        */
#define STM32_I2SSRC_OFF        (1 << 23)   /**< ISS clock not required.    */

#define STM32_MCO1PRE_MASK      (7 << 24)   /**< MCO1PRE mask.              */
#define STM32_MCO1PRE_DIV1      (0 << 24)   /**< MCO1 divided by 1.         */
#define STM32_MCO1PRE_DIV2      (4 << 24)   /**< MCO1 divided by 2.         */
#define STM32_MCO1PRE_DIV3      (5 << 24)   /**< MCO1 divided by 3.         */
#define STM32_MCO1PRE_DIV4      (6 << 24)   /**< MCO1 divided by 4.         */
#define STM32_MCO1PRE_DIV5      (7 << 24)   /**< MCO1 divided by 5.         */

#define STM32_MCO2PRE_MASK      (7 << 27)   /**< MCO2PRE mask.              */
#define STM32_MCO2PRE_DIV1      (0 << 27)   /**< MCO2 divided by 1.         */
#define STM32_MCO2PRE_DIV2      (4 << 27)   /**< MCO2 divided by 2.         */
#define STM32_MCO2PRE_DIV3      (5 << 27)   /**< MCO2 divided by 3.         */
#define STM32_MCO2PRE_DIV4      (6 << 27)   /**< MCO2 divided by 4.         */
#define STM32_MCO2PRE_DIV5      (7 << 27)   /**< MCO2 divided by 5.         */

#define STM32_MCO2SEL_MASK      (3 << 30)   /**< MCO2 mask.                 */
#define STM32_MCO2SEL_SYSCLK    (0 << 30)   /**< SYSCLK clock on MCO2 pin.  */
#define STM32_MCO2SEL_PLLI2S    (1 << 30)   /**< PLLI2S clock on MCO2 pin.  */
#define STM32_MCO2SEL_HSE       (2 << 30)   /**< HSE clock on MCO2 pin.     */
#define STM32_MCO2SEL_PLL       (3 << 30)   /**< PLL clock on MCO2 pin.     */
/** @} */

/**
 * @name    RCC_PLLI2SCFGR register bits definitions
 * @{
 */
#define STM32_PLLI2SN_MASK      (511 << 6)  /**< PLLI2SN mask.              */
#define STM32_PLLI2SR_MASK      (7 << 28)   /**< PLLI2SR mask.              */
/** @} */

/**
 * @name    RCC_DCKCFGR1 register bits definitions
 * @{
 */
#define STM32_PLLSAIDIVR_MASK   (3 << 16)   /**< PLLSAIDIVR mask.           */
#define STM32_PLLSAIDIVR_DIV2   (0 << 16)   /**< LCD_CLK is R divided by 2. */
#define STM32_PLLSAIDIVR_DIV4   (1 << 16)   /**< LCD_CLK is R divided by 4. */
#define STM32_PLLSAIDIVR_DIV8   (2 << 16)   /**< LCD_CLK is R divided by 8. */
#define STM32_PLLSAIDIVR_DIV16  (3 << 16)   /**< LCD_CLK is R divided by 16.*/

#define STM32_SAI1SEL_MASK      (3 << 20)   /**< SAI1SEL mask.              */
#define STM32_SAI1SEL_SAIPLL    (0 << 20)   /**< SAI1 source is SAIPLL.     */
#define STM32_SAI1SEL_I2SPLL    (1 << 20)   /**< SAI1 source is I2SPLL.     */
#define STM32_SAI1SEL_CKIN      (2 << 20)   /**< SAI1 source is I2S_CKIN.   */
#define STM32_SAI1SEL_OFF       0xFFFFFFFFU /**< SAI1 clock is not required.*/

#define STM32_SAI2SEL_MASK      (3 << 22)   /**< SAI2SEL mask.              */
#define STM32_SAI2SEL_SAIPLL    (0 << 22)   /**< SAI2 source is SAIPLL.     */
#define STM32_SAI2SEL_I2SPLL    (1 << 22)   /**< SAI2 source is I2SPLL.     */
#define STM32_SAI2SEL_CKIN      (2 << 22)   /**< SAI2 source is I2S_CKIN.   */
#define STM32_SAI2SEL_OFF       0xFFFFFFFFU /**< SAI2 clock is not required.*/

#define STM32_TIMPRE_MASK       (1 << 24)   /**< TIMPRE mask.               */
#define STM32_TIMPRE_PCLK       (0 << 24)   /**< TIM clocks from PCLKx.     */
#define STM32_TIMPRE_HCLK       (1 << 24)   /**< TIM clocks from HCLK.      */
/** @} */

/**
 * @name    RCC_DCKCFGR2 register bits definitions
 * @{
 */
#define STM32_USART1SEL_MASK    (3 << 0)    /**< USART1SEL mask.            */
#define STM32_USART1SEL_PCLK2   (0 << 0)    /**< USART1 source is PCLK2.    */
#define STM32_USART1SEL_SYSCLK  (1 << 0)    /**< USART1 source is SYSCLK.   */
#define STM32_USART1SEL_HSI     (2 << 0)    /**< USART1 source is HSI.      */
#define STM32_USART1SEL_LSE     (3 << 0)    /**< USART1 source is LSE.      */

#define STM32_USART2SEL_MASK    (3 << 2)    /**< USART2 mask.               */
#define STM32_USART2SEL_PCLK1   (0 << 2)    /**< USART2 source is PCLK1.    */
#define STM32_USART2SEL_SYSCLK  (1 << 2)    /**< USART2 source is SYSCLK.   */
#define STM32_USART2SEL_HSI     (2 << 2)    /**< USART2 source is HSI.      */
#define STM32_USART2SEL_LSE     (3 << 2)    /**< USART2 source is LSE.      */

#define STM32_USART3SEL_MASK    (3 << 4)    /**< USART3 mask.               */
#define STM32_USART3SEL_PCLK1   (0 << 4)    /**< USART3 source is PCLK1.    */
#define STM32_USART3SEL_SYSCLK  (1 << 4)    /**< USART3 source is SYSCLK.   */
#define STM32_USART3SEL_HSI     (2 << 4)    /**< USART3 source is HSI.      */
#define STM32_USART3SEL_LSE     (3 << 4)    /**< USART3 source is LSE.      */

#define STM32_UART4SEL_MASK     (3 << 6)    /**< UART4 mask.                */
#define STM32_UART4SEL_PCLK1    (0 << 6)    /**< UART4 source is PCLK1.     */
#define STM32_UART4SEL_SYSCLK   (1 << 6)    /**< UART4 source is SYSCLK.    */
#define STM32_UART4SEL_HSI      (2 << 6)    /**< UART4 source is HSI.       */
#define STM32_UART4SEL_LSE      (3 << 6)    /**< UART4 source is LSE.       */

#define STM32_UART5SEL_MASK     (3 << 8)    /**< UART5 mask.                */
#define STM32_UART5SEL_PCLK1    (0 << 8)    /**< UART5 source is PCLK1.     */
#define STM32_UART5SEL_SYSCLK   (1 << 8)    /**< UART5 source is SYSCLK.    */
#define STM32_UART5SEL_HSI      (2 << 8)    /**< UART5 source is HSI.       */
#define STM32_UART5SEL_LSE      (3 << 8)    /**< UART5 source is LSE.       */

#define STM32_USART6SEL_MASK    (3 << 10)   /**< USART6SEL mask.            */
#define STM32_USART6SEL_PCLK2   (0 << 10)   /**< USART6 source is PCLK2.    */
#define STM32_USART6SEL_SYSCLK  (1 << 10)   /**< USART6 source is SYSCLK.   */
#define STM32_USART6SEL_HSI     (2 << 10)   /**< USART6 source is HSI.      */
#define STM32_USART6SEL_LSE     (3 << 10)   /**< USART6 source is LSE.      */

#define STM32_UART7SEL_MASK     (3 << 12)   /**< UART7 mask.                */
#define STM32_UART7SEL_PCLK1    (0 << 12)   /**< UART7 source is PCLK1.     */
#define STM32_UART7SEL_SYSCLK   (1 << 12)   /**< UART7 source is SYSCLK.    */
#define STM32_UART7SEL_HSI      (2 << 12)   /**< UART7 source is HSI.       */
#define STM32_UART7SEL_LSE      (3 << 12)   /**< UART7 source is LSE.       */

#define STM32_UART8SEL_MASK     (3 << 14)   /**< UART8 mask.                */
#define STM32_UART8SEL_PCLK1    (0 << 14)   /**< UART8 source is PCLK1.     */
#define STM32_UART8SEL_SYSCLK   (1 << 14)   /**< UART8 source is SYSCLK.    */
#define STM32_UART8SEL_HSI      (2 << 14)   /**< UART8 source is HSI.       */
#define STM32_UART8SEL_LSE      (3 << 14)   /**< UART8 source is LSE.       */

#define STM32_I2C1SEL_MASK      (3 << 16)   /**< I2C1SEL mask.              */
#define STM32_I2C1SEL_PCLK1     (0 << 16)   /**< I2C1 source is PCLK1.      */
#define STM32_I2C1SEL_SYSCLK    (1 << 16)   /**< I2C1 source is SYSCLK.     */
#define STM32_I2C1SEL_HSI       (2 << 16)   /**< I2C1 source is HSI.        */
#define STM32_I2C1SEL_LSE       (3 << 16)   /**< I2C1 source is LSE.        */

#define STM32_I2C2SEL_MASK      (3 << 18)   /**< I2C2SEL mask.              */
#define STM32_I2C2SEL_PCLK1     (0 << 18)   /**< I2C2 source is PCLK1.      */
#define STM32_I2C2SEL_SYSCLK    (1 << 18)   /**< I2C2 source is SYSCLK.     */
#define STM32_I2C2SEL_HSI       (2 << 18)   /**< I2C2 source is HSI.        */
#define STM32_I2C2SEL_LSE       (3 << 18)   /**< I2C2 source is LSE.        */

#define STM32_I2C3SEL_MASK      (3 << 20)   /**< I2C3SEL mask.              */
#define STM32_I2C3SEL_PCLK1     (0 << 20)   /**< I2C3 source is PCLK1.      */
#define STM32_I2C3SEL_SYSCLK    (1 << 20)   /**< I2C3 source is SYSCLK.     */
#define STM32_I2C3SEL_HSI       (2 << 20)   /**< I2C3 source is HSI.        */
#define STM32_I2C3SEL_LSE       (3 << 20)   /**< I2C3 source is LSE.        */

#define STM32_I2C4SEL_MASK      (3 << 22)   /**< I2C4SEL mask.              */
#define STM32_I2C4SEL_PCLK1     (0 << 22)   /**< I2C4 source is PCLK1.      */
#define STM32_I2C4SEL_SYSCLK    (1 << 22)   /**< I2C4 source is SYSCLK.     */
#define STM32_I2C4SEL_HSI       (2 << 22)   /**< I2C4 source is HSI.        */
#define STM32_I2C4SEL_LSE       (3 << 22)   /**< I2C4 source is LSE.        */

#define STM32_LPTIM1SEL_MASK    (3 << 24)   /**< LPTIM1SEL mask.            */
#define STM32_LPTIM1SEL_PCLK1   (0 << 24)   /**< LPTIM1 source is PCLK1.    */
#define STM32_LPTIM1SEL_LSI     (1 << 24)   /**< LPTIM1 source is LSI.   */
#define STM32_LPTIM1SEL_HSI     (2 << 24)   /**< LPTIM1 source is HSI.      */
#define STM32_LPTIM1SEL_LSE     (3 << 24)   /**< LPTIM1 source is LSE.      */

#define STM32_CECSEL_MASK       (1 << 26)   /**< CECSEL mask.               */
#define STM32_CECSEL_LSE        (0 << 26)   /**< CEC source is LSE.         */
#define STM32_CECSEL_HSIDIV488  (1 << 26)   /**< CEC source is HSI/488.     */

#define STM32_CK48MSEL_MASK     (1 << 27)   /**< CK48MSEL mask.             */
#define STM32_CK48MSEL_PLL      (0 << 27)   /**< PLL48CLK source is PLL.    */
#define STM32_CK48MSEL_PLLSAI   (1 << 27)   /**< PLL48CLK source is PLLSAI. */

#define STM32_SDMMC1SEL_MASK     (1 << 28)  /**< SDMMC1SEL mask.            */
#define STM32_SDMMC1SEL_PLL48CLK (0 << 28)  /**< SDMMC1 source is PLL48CLK. */
#define STM32_SDMMC1SEL_SYSCLK   (1 << 28)  /**< SDMMC1 source is SYSCLK.   */

#define STM32_SDMMC2SEL_MASK     (1 << 29)  /**< SDMMC2SEL mask.            */
#define STM32_SDMMC2SEL_PLL48CLK (0 << 29)  /**< SDMMC2 source is PLL48CLK. */
#define STM32_SDMMC2SEL_SYSCLK   (1 << 29)  /**< SDMMC2 source is SYSCLK.   */
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
 * @brief   Enables a no-cache RAM area using the MPU.
 */
#if !defined(STM32_NOCACHE_ENABLE) || defined(__DOXYGEN__)
#define STM32_NOCACHE_ENABLE                FALSE
#endif

/**
 * @brief   MPU region to be used for no-cache RAM area.
 */
#if !defined(STM32_NOCACHE_MPU_REGION) || defined(__DOXYGEN__)
#define STM32_NOCACHE_MPU_REGION            MPU_REGION_6
#endif

/**
 * @brief   Base address to be used for no-cache RAM area.
 */
#if !defined(STM32_NOCACHE_RBAR) || defined(__DOXYGEN__)
#define STM32_NOCACHE_RBAR                  0x2004C000U
#endif

/**
 * @brief   Size to be used for no-cache RAM area.
 */
#if !defined(STM32_NOCACHE_RASR) || defined(__DOXYGEN__)
#define STM32_NOCACHE_RASR                  MPU_RASR_SIZE_16K
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
 * @brief   Enables the backup RAM regulator.
 */
#if !defined(STM32_BKPRAM_ENABLE) || defined(__DOXYGEN__)
#define STM32_BKPRAM_ENABLE                 FALSE
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
 * @brief   USB/SDIO clock setting.
 */
#if !defined(STM32_CLOCK48_REQUIRED) || defined(__DOXYGEN__)
#define STM32_CLOCK48_REQUIRED              TRUE
#endif

/**
 * @brief   Main clock source selection.
 * @note    If the selected clock source is not the PLL then the PLL is not
 *          initialized and started.
 * @note    The default value is calculated for a 216MHz system clock from
 *          an external 25MHz HSE clock.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                            STM32_SW_PLL
#endif

/**
 * @brief   Clock source for the PLLs.
 * @note    This setting has only effect if the PLL is selected as the
 *          system clock source.
 * @note    The default value is calculated for a 216MHz system clock from
 *          an external 25MHz HSE clock.
 */
#if !defined(STM32_PLLSRC) || defined(__DOXYGEN__)
#define STM32_PLLSRC                        STM32_PLLSRC_HSE
#endif

/**
 * @brief   PLLM divider value.
 * @note    The allowed values are 2..63.
 * @note    The default value is calculated for a 216MHz system clock from
 *          an external 25MHz HSE clock.
 */
#if !defined(STM32_PLLM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLM_VALUE                    25
#endif

/**
 * @brief   PLLN multiplier value.
 * @note    The allowed values are 192..432.
 * @note    The default value is calculated for a 216MHz system clock from
 *          an external 25MHz HSE clock.
 */
#if !defined(STM32_PLLN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLN_VALUE                    432
#endif

/**
 * @brief   PLLP divider value.
 * @note    The allowed values are 2, 4, 6, 8.
 * @note    The default value is calculated for a 216MHz system clock from
 *          an external 25MHz HSE clock.
 */
#if !defined(STM32_PLLP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLP_VALUE                    2
#endif

/**
 * @brief   PLLQ divider value.
 * @note    The allowed values are 2..15.
 * @note    The default value is calculated for a 216MHz system clock from
 *          an external 25MHz HSE clock.
 */
#if !defined(STM32_PLLQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLQ_VALUE                    9
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
#define STM32_PPRE1                         STM32_PPRE1_DIV4
#endif

/**
 * @brief   APB2 prescaler value.
 */
#if !defined(STM32_PPRE2) || defined(__DOXYGEN__)
#define STM32_PPRE2                         STM32_PPRE2_DIV2
#endif

/**
 * @brief   RTC clock source.
 */
#if !defined(STM32_RTCSEL) || defined(__DOXYGEN__)
#define STM32_RTCSEL                        STM32_RTCSEL_LSE
#endif

/**
 * @brief   RTC HSE prescaler value.
 * @note    The allowed values are 2..31.
 */
#if !defined(STM32_RTCPRE_VALUE) || defined(__DOXYGEN__)
#define STM32_RTCPRE_VALUE                  25
#endif

/**
 * @brief   MCO1 clock source value.
 * @note    The default value outputs HSI clock on MCO1 pin.
 */
#if !defined(STM32_MCO1SEL) || defined(__DOXYGEN__)
#define STM32_MCO1SEL                       STM32_MCO1SEL_HSI
#endif

/**
 * @brief   MCO1 prescaler value.
 * @note    The default value outputs HSI clock on MCO1 pin.
 */
#if !defined(STM32_MCO1PRE) || defined(__DOXYGEN__)
#define STM32_MCO1PRE                       STM32_MCO1PRE_DIV1
#endif

/**
 * @brief   MCO2 clock source value.
 * @note    The default value outputs SYSCLK / 4 on MCO2 pin.
 */
#if !defined(STM32_MCO2SEL) || defined(__DOXYGEN__)
#define STM32_MCO2SEL                       STM32_MCO2SEL_SYSCLK
#endif

/**
 * @brief   MCO2 prescaler value.
 * @note    The default value outputs SYSCLK / 4 on MCO2 pin.
 */
#if !defined(STM32_MCO2PRE) || defined(__DOXYGEN__)
#define STM32_MCO2PRE                       STM32_MCO2PRE_DIV4
#endif

/**
 * @brief   TIM clock prescaler selection.
 */
#if !defined(STM32_TIMPRE_ENABLE) || defined(__DOXYGEN__)
#define STM32_TIMPRE_ENABLE                 FALSE
#endif

/**
 * @brief   I2S clock source.
 */
#if !defined(STM32_I2SSRC) || defined(__DOXYGEN__)
#define STM32_I2SSRC                        STM32_I2SSRC_PLLI2S
#endif

/**
 * @brief   PLLI2SN multiplier value.
 * @note    The allowed values are 49..432.
 */
#if !defined(STM32_PLLI2SN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLI2SN_VALUE                 192
#endif

/**
 * @brief   PLLI2SP divider value.
 * @note    The allowed values are 2, 4, 6 and 8.
 */
#if !defined(STM32_PLLI2SP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLI2SP_VALUE                 4
#endif

/**
 * @brief   PLLI2SQ divider value.
 * @note    The allowed values are 2..15.
 */
#if !defined(STM32_PLLI2SQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLI2SQ_VALUE                 4
#endif

/**
 * @brief   PLLI2SDIVQ divider value (SAI clock divider).
 */
#if !defined(STM32_PLLI2SDIVQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLI2SDIVQ_VALUE              2
#endif

/**
 * @brief   PLLI2SR divider value.
 * @note    The allowed values are 2..7.
 */
#if !defined(STM32_PLLI2SR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLI2SR_VALUE                 4
#endif

/**
 * @brief   PLLSAIN multiplier value.
 * @note    The allowed values are 49..432.
 */
#if !defined(STM32_PLLSAIN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAIN_VALUE                 192
#endif

/**
 * @brief   PLLSAIP divider value.
 * @note    The allowed values are 2, 4, 6 and 8.
 */
#if !defined(STM32_PLLSAIP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAIP_VALUE                 4
#endif

/**
 * @brief   PLLSAIQ divider value.
 * @note    The allowed values are 2..15.
 */
#if !defined(STM32_PLLSAIQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAIQ_VALUE                 4
#endif

/**
 * @brief   PLLSAIR divider value.
 * @note    The allowed values are 2..7.
 */
#if !defined(STM32_PLLSAIR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAIR_VALUE                 4
#endif

/**
 * @brief   PLLSAIDIVQ divider value (SAI clock divider).
 */
#if !defined(STM32_PLLSAIDIVQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAIDIVQ_VALUE              2
#endif

/**
 * @brief   PLLSAIDIVR divider value (LCD clock divider).
 */
#if !defined(STM32_PLLSAIDIVR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAIDIVR_VALUE              2
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
 * @brief   LCD-TFT clock enable switch.
 */
#if !defined(STM32_LCDTFT_REQUIRED) || defined(__DOXYGEN__)
#define STM32_LCDTFT_REQUIRED               FALSE
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
#define STM32_USART6SEL                     STM32_USART6SEL_PCLK2
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
 * @brief   CEC clock source.
 */
#if !defined(STM32_CECSEL) || defined(__DOXYGEN__)
#define STM32_CECSEL                        STM32_CECSEL_LSE
#endif

/**
 * @brief   PLL48CLK clock source.
 */
#if !defined(STM32_CK48MSEL) || defined(__DOXYGEN__)
#define STM32_CK48MSEL                      STM32_CK48MSEL_PLL
#endif

/**
 * @brief   SDMMC1 clock source.
 */
#if !defined(STM32_SDMMC1SEL) || defined(__DOXYGEN__)
#define STM32_SDMMC1SEL                     STM32_SDMMC1SEL_PLL48CLK
#endif

/**
 * @brief   SDMMC2 clock source.
 */
#if !defined(STM32_SDMMC2SEL) || defined(__DOXYGEN__)
#define STM32_SDMMC2SEL                     STM32_SDMMC2SEL_PLL48CLK
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(STM32F7xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F7xx_MCUCONF not defined"
#endif

#if defined(STM32F722xx) && !defined(STM32F722_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F722_MCUCONF not defined"
#endif

#if defined(STM32F732xx) && !defined(STM32F732_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F732_MCUCONF not defined"
#endif

#if defined(STM32F723xx) && !defined(STM32F723_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F723_MCUCONF not defined"
#endif

#if defined(STM32F733xx) && !defined(STM32F733_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F733_MCUCONF not defined"
#endif

#if defined(STM32F746xx) && !defined(STM32F746_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F746_MCUCONF not defined"
#endif

#if defined(STM32F756xx) && !defined(STM32F756_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F756_MCUCONF not defined"
#endif

#if defined(STM32F765xx) && !defined(STM32F765_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F765_MCUCONF not defined"
#endif

#if defined(STM32F767xx) && !defined(STM32F767_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F767_MCUCONF not defined"
#endif

#if defined(STM32F777xx) && !defined(STM32F777_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F777_MCUCONF not defined"
#endif

#if defined(STM32F769xx) && !defined(STM32F769_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F769_MCUCONF not defined"
#endif

#if defined(STM32F779xx) && !defined(STM32F779_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F779_MCUCONF not defined"
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
#if !defined(STM32_VDD)
#error "STM32_VDD not defined in board.h"
#endif

/**
 * @brief   Maximum frequency thresholds and wait states for flash access.
 * @note    The values are valid for 2.7V to 3.6V supply range.
 */
#if ((STM32_VDD >= 270) && (STM32_VDD <= 360)) || defined(__DOXYGEN__)
#define STM32_0WS_THRESHOLD         30000000
#define STM32_1WS_THRESHOLD         60000000
#define STM32_2WS_THRESHOLD         90000000
#define STM32_3WS_THRESHOLD         120000000
#define STM32_4WS_THRESHOLD         150000000
#define STM32_5WS_THRESHOLD         180000000
#define STM32_6WS_THRESHOLD         210000000
#define STM32_7WS_THRESHOLD         STM32_SYSCLK_MAX
#define STM32_8WS_THRESHOLD         0
#define STM32_9WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           2

#elif (STM32_VDD >= 240) && (STM32_VDD < 270)
#define STM32_0WS_THRESHOLD         24000000
#define STM32_1WS_THRESHOLD         48000000
#define STM32_2WS_THRESHOLD         72000000
#define STM32_3WS_THRESHOLD         96000000
#define STM32_4WS_THRESHOLD         120000000
#define STM32_5WS_THRESHOLD         144000000
#define STM32_6WS_THRESHOLD         168000000
#define STM32_7WS_THRESHOLD         192000000
#define STM32_8WS_THRESHOLD         STM32_SYSCLK_MAX
#define STM32_9WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           1

#elif (STM32_VDD >= 210) && (STM32_VDD < 240)
#define STM32_0WS_THRESHOLD         22000000
#define STM32_1WS_THRESHOLD         44000000
#define STM32_2WS_THRESHOLD         66000000
#define STM32_3WS_THRESHOLD         88000000
#define STM32_4WS_THRESHOLD         110000000
#define STM32_5WS_THRESHOLD         132000000
#define STM32_6WS_THRESHOLD         154000000
#define STM32_7WS_THRESHOLD         176000000
#define STM32_8WS_THRESHOLD         198000000
#define STM32_9WS_THRESHOLD         STM32_SYSCLK_MAX
#define STM32_FLASH_PSIZE           1

#elif (STM32_VDD >= 180) && (STM32_VDD < 210)
#define STM32_0WS_THRESHOLD         20000000
#define STM32_1WS_THRESHOLD         40000000
#define STM32_2WS_THRESHOLD         60000000
#define STM32_3WS_THRESHOLD         80000000
#define STM32_4WS_THRESHOLD         100000000
#define STM32_5WS_THRESHOLD         120000000
#define STM32_6WS_THRESHOLD         140000000
#define STM32_7WS_THRESHOLD         160000000
#define STM32_8WS_THRESHOLD         180000000
#define STM32_9WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           0

#else
#error "invalid VDD voltage specified"
#endif

/*
 * HSI related checks.
 */
#if STM32_HSI_ENABLED
#else /* !STM32_HSI_ENABLED */

#if STM32_SW == STM32_SW_HSI
#error "HSI not enabled, required by STM32_SW"
#endif

#if (STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSI)
#error "HSI not enabled, required by STM32_SW and STM32_PLLSRC"
#endif

#if (STM32_MCO1SEL == STM32_MCO1SEL_HSI) ||                                 \
    ((STM32_MCO1SEL == STM32_MCO1SEL_PLL) &&                                \
     (STM32_PLLSRC == STM32_PLLSRC_HSI))
#error "HSI not enabled, required by STM32_MCO1SEL"
#endif

#if (STM32_MCO2SEL == STM32_MCO2SEL_PLL) &&                                 \
    (STM32_PLLSRC == STM32_PLLSRC_HSI)
#error "HSI not enabled, required by STM32_MCO2SEL"
#endif

#if (STM32_I2SSRC == STM32_I2SSRC_PLLI2S) &&                                \
    (STM32_PLLSRC == STM32_PLLSRC_HSI)
#error "HSI not enabled, required by STM32_I2SSRC"
#endif

#if ((STM32_SAI1SEL == STM32_SAI1SEL_SAIPLL) ||                             \
     (STM32_SAI1SEL == STM32_SAI1SEL_I2SPLL)) &&                            \
    (STM32_PLLSRC == STM32_PLLSRC_HSI)
#error "HSI not enabled, required by STM32_SAI1SEL"
#endif

#if ((STM32_SAI2SEL == STM32_SAI2SEL_SAIPLL) ||                             \
     (STM32_SAI2SEL == STM32_SAI2SEL_I2SPLL)) &&                            \
    (STM32_PLLSRC == STM32_PLLSRC_HSI)
#error "HSI not enabled, required by STM32_SAI2SEL"
#endif

#if STM32_LCDTFT_REQUIRED &&                                                \
    (STM32_PLLSRC == STM32_PLLSRC_HSI)
#error "HSI not enabled, required by STM32_LCDTFT_REQUIRED"
#endif

#endif /* !STM32_HSI_ENABLED */

/*
 * HSE related checks.
 */
#if STM32_HSE_ENABLED

#if STM32_HSECLK == 0
#error "HSE frequency not defined"
#else /* STM32_HSECLK != 0 */
#if defined(STM32_HSE_BYPASS)
#if (STM32_HSECLK < STM32_HSECLK_MIN) || (STM32_HSECLK > STM32_HSECLK_BYP_MAX)
#error "STM32_HSECLK outside acceptable range (STM32_HSECLK_MIN...STM32_HSECLK_BYP_MAX)"
#endif
#else /* !defined(STM32_HSE_BYPASS) */
#if (STM32_HSECLK < STM32_HSECLK_MIN) || (STM32_HSECLK > STM32_HSECLK_MAX)
#error "STM32_HSECLK outside acceptable range (STM32_HSECLK_MIN...STM32_HSECLK_MAX)"
#endif
#endif /* !defined(STM32_HSE_BYPASS) */
#endif /* STM32_HSECLK != 0 */
#else /* !STM32_HSE_ENABLED */

#if STM32_SW == STM32_SW_HSE
#error "HSE not enabled, required by STM32_SW"
#endif

#if (STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSE)
#error "HSE not enabled, required by STM32_SW and STM32_PLLSRC"
#endif

#if (STM32_MCO1SEL == STM32_MCO1SEL_HSE) ||                                 \
    ((STM32_MCO1SEL == STM32_MCO1SEL_PLL) &&                                \
     (STM32_PLLSRC == STM32_PLLSRC_HSE))
#error "HSE not enabled, required by STM32_MCO1SEL"
#endif

#if (STM32_MCO2SEL == STM32_MCO2SEL_HSE) ||                                 \
    ((STM32_MCO2SEL == STM32_MCO2SEL_PLL) &&                                \
     (STM32_PLLSRC == STM32_PLLSRC_HSE))
#error "HSE not enabled, required by STM32_MCO2SEL"
#endif

#if (STM32_I2SSRC == STM32_I2SSRC_PLLI2S) &&                                \
    (STM32_PLLSRC == STM32_PLLSRC_HSE)
#error "HSE not enabled, required by STM32_I2SSRC"
#endif

#if ((STM32_SAI1SEL == STM32_SAI1SEL_SAIPLL) |                              \
     (STM32_SAI1SEL == STM32_SAI1SEL_I2SPLL)) &&                            \
    (STM32_PLLSRC == STM32_PLLSRC_HSE)
#error "HSE not enabled, required by STM32_SAI1SEL"
#endif

#if ((STM32_SAI2SEL == STM32_SAI2SEL_SAIPLL) |                              \
     (STM32_SAI2SEL == STM32_SAI2SEL_I2SPLL)) &&                            \
    (STM32_PLLSRC == STM32_PLLSRC_HSE)
#error "HSE not enabled, required by STM32_SAI2SEL"
#endif

#if STM32_LCDTFT_REQUIRED &&                                                \
    (STM32_PLLSRC == STM32_PLLSRC_HSE)
#error "HSE not enabled, required by STM32_LCDTFT_REQUIRED"
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

#if (STM32_LSECLK == 0)
#error "LSE frequency not defined"
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

#else /* !STM32_LSE_ENABLED */

#if STM32_RTCSEL == STM32_RTCSEL_LSE
#error "LSE not enabled, required by STM32_RTCSEL"
#endif

#if STM32_MCO1SEL == STM32_MCO1SEL_LSE
#error "LSE not enabled, required by STM32_MCO1SEL"
#endif

#endif /* !STM32_LSE_ENABLED */

/**
 * @brief   STM32_PLLM field.
 */
#if ((STM32_PLLM_VALUE >= 2) && (STM32_PLLM_VALUE <= 63)) ||                \
    defined(__DOXYGEN__)
#define STM32_PLLM                  (STM32_PLLM_VALUE << 0)
#else
#error "invalid STM32_PLLM_VALUE value specified"
#endif

/**
 * @brief   PLLs input clock frequency.
 */
#if (STM32_PLLSRC == STM32_PLLSRC_HSE) || defined(__DOXYGEN__)
#define STM32_PLLCLKIN              (STM32_HSECLK / STM32_PLLM_VALUE)

#elif STM32_PLLSRC == STM32_PLLSRC_HSI
#define STM32_PLLCLKIN              (STM32_HSICLK / STM32_PLLM_VALUE)

#else
#error "invalid STM32_PLLSRC value specified"
#endif

/*
 * PLLs input frequency range check.
 */
#if (STM32_PLLCLKIN < STM32_PLLIN_MIN) || (STM32_PLLCLKIN > STM32_PLLIN_MAX)
#error "STM32_PLLCLKIN outside acceptable range (STM32_PLLIN_MIN...STM32_PLLIN_MAX)"
#endif

/*
 * PLL enable check.
 */
#if (STM32_CLOCK48_REQUIRED && (STM32_CK48MSEL == STM32_CK48MSEL_PLL)) ||   \
    (STM32_SW == STM32_SW_PLL) ||                                           \
    (STM32_MCO1SEL == STM32_MCO1SEL_PLL) ||                                 \
    (STM32_MCO2SEL == STM32_MCO2SEL_PLL) ||                                 \
    defined(__DOXYGEN__)
/**
 * @brief   PLL activation flag.
 */
#define STM32_ACTIVATE_PLL          TRUE
#else
#define STM32_ACTIVATE_PLL          FALSE
#endif

/**
 * @brief   STM32_PLLN field.
 */
#if ((STM32_PLLN_VALUE >= 64) && (STM32_PLLN_VALUE <= 432)) ||              \
    defined(__DOXYGEN__)
#define STM32_PLLN                  (STM32_PLLN_VALUE << 6)
#else
#error "invalid STM32_PLLN_VALUE value specified"
#endif

/**
 * @brief   STM32_PLLP field.
 */
#if (STM32_PLLP_VALUE == 2) || defined(__DOXYGEN__)
#define STM32_PLLP                  (0 << 16)

#elif STM32_PLLP_VALUE == 4
#define STM32_PLLP                  (1 << 16)

#elif STM32_PLLP_VALUE == 6
#define STM32_PLLP                  (2 << 16)

#elif STM32_PLLP_VALUE == 8
#define STM32_PLLP                  (3 << 16)

#else
#error "invalid STM32_PLLP_VALUE value specified"
#endif

/**
 * @brief   STM32_PLLQ field.
 */
#if ((STM32_PLLQ_VALUE >= 2) && (STM32_PLLQ_VALUE <= 15)) ||                \
    defined(__DOXYGEN__)
#define STM32_PLLQ                  (STM32_PLLQ_VALUE << 24)
#else
#error "invalid STM32_PLLQ_VALUE value specified"
#endif

/**
 * @brief   PLL VCO frequency.
 */
#define STM32_PLLVCO                (STM32_PLLCLKIN * STM32_PLLN_VALUE)

/*
 * PLL VCO frequency range check.
 */
#if (STM32_PLLVCO < STM32_PLLVCO_MIN) || (STM32_PLLVCO > STM32_PLLVCO_MAX)
#error "STM32_PLLVCO outside acceptable range (STM32_PLLVCO_MIN...STM32_PLLVCO_MAX)"
#endif

/**
 * @brief   PLL P output clock frequency.
 */
#define STM32_PLL_P_CLKOUT          (STM32_PLLVCO / STM32_PLLP_VALUE)

/**
 * @brief   PLL Q output clock frequency.
 */
#define STM32_PLL_Q_CLKOUT          (STM32_PLLVCO / STM32_PLLQ_VALUE)

/*
 * PLL output frequency range check.
 */
#if (STM32_PLL_P_CLKOUT < STM32_PLLOUT_MIN) || (STM32_PLL_P_CLKOUT > STM32_PLLOUT_MAX)
#error "STM32_PLL_P_CLKOUT outside acceptable range (STM32_PLLOUT_MIN...STM32_PLLOUT_MAX)"
#endif

/**
 * @brief   System clock source.
 */
#if (STM32_SW == STM32_SW_HSI)
#define STM32_SYSCLK                STM32_HSICLK

#elif (STM32_SW == STM32_SW_HSE)
#define STM32_SYSCLK                STM32_HSECLK

#elif (STM32_SW == STM32_SW_PLL)
#define STM32_SYSCLK                STM32_PLL_P_CLKOUT

#else
#error "invalid STM32_SW value specified"
#endif

/* Check on the system clock.*/
#if STM32_SYSCLK > STM32_SYSCLK_MAX
#error "STM32_SYSCLK above maximum rated frequency (STM32_SYSCLK_MAX)"
#endif

/* Calculating VOS settings.*/
#if STM32_SYSCLK <= 144000000
#define STM32_VOS                   STM32_VOS_SCALE3
#define STM32_OVERDRIVE_REQUIRED    FALSE

#elif STM32_SYSCLK <= 168000000
#define STM32_VOS                   STM32_VOS_SCALE2
#define STM32_OVERDRIVE_REQUIRED    FALSE

#elif STM32_SYSCLK <= 180000000
#define STM32_VOS                   STM32_VOS_SCALE1
#define STM32_OVERDRIVE_REQUIRED    FALSE

#else
#define STM32_VOS                   STM32_VOS_SCALE1
#define STM32_OVERDRIVE_REQUIRED    TRUE
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

/*
 * AHB frequency check.
 */
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

/*
 * APB1 frequency check.
 */
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

/*
 * APB2 frequency check.
 */
#if STM32_PCLK2 > STM32_PCLK2_MAX
#error "STM32_PCLK2 exceeding maximum frequency (STM32_PCLK2_MAX)"
#endif

/*
 * PLLI2S enable check.
 */
#if (STM32_I2SSRC == STM32_I2SSRC_PLLI2S) ||                                \
    (STM32_SAI1SEL == STM32_SAI1SEL_I2SPLL) ||                              \
    (STM32_SAI2SEL == STM32_SAI2SEL_I2SPLL) ||                              \
    defined(__DOXYGEN__)
/**
 * @brief   PLLI2S activation flag.
 */
#define STM32_ACTIVATE_PLLI2S       TRUE
#else
#define STM32_ACTIVATE_PLLI2S       FALSE
#endif

/**
 * @brief   STM32_PLLI2SN field.
 */
#if ((STM32_PLLI2SN_VALUE >= 49) && (STM32_PLLI2SN_VALUE <= 432)) ||       \
    defined(__DOXYGEN__)
#define STM32_PLLI2SN               (STM32_PLLI2SN_VALUE << 6)
#else
#error "invalid STM32_PLLI2SN_VALUE value specified"
#endif

/**
 * @brief   STM32_PLLI2SQ field.
 */
#if ((STM32_PLLI2SQ_VALUE >= 2) && (STM32_PLLI2SQ_VALUE <= 15)) ||          \
    defined(__DOXYGEN__)
#define STM32_PLLI2SQ               (STM32_PLLI2SQ_VALUE << 24)
#else
#error "invalid STM32_PLLI2SQ_VALUE value specified"
#endif

/**
 * @brief   STM32_PLLI2SR field.
 */
#if ((STM32_PLLI2SR_VALUE >= 2) && (STM32_PLLI2SR_VALUE <= 7)) ||           \
    defined(__DOXYGEN__)
#define STM32_PLLI2SR               (STM32_PLLI2SR_VALUE << 28)
#else
#error "invalid STM32_PLLI2SR_VALUE value specified"
#endif

/**
 * @brief   STM32_PLLI2SP field.
 */
#if (STM32_PLLI2SP_VALUE == 2) || defined(__DOXYGEN__)
#define STM32_PLLI2SP               (0 << 16)

#elif STM32_PLLI2SP_VALUE == 4
#define STM32_PLLI2SP               (1 << 16)

#elif STM32_PLLI2SP_VALUE == 6
#define STM32_PLLI2SP               (2 << 16)

#elif STM32_PLLI2SP_VALUE == 8
#define STM32_PLLI2SP               (3 << 16)

#else
#error "invalid STM32_PLLI2SP_VALUE value specified"
#endif

/**
 * @brief   PLLI2S VCO frequency.
 */
#define STM32_PLLI2SVCO             (STM32_PLLCLKIN * STM32_PLLI2SN_VALUE)

/*
 * PLLI2S VCO frequency range check.
 */
#if (STM32_PLLI2SVCO < STM32_PLLVCO_MIN) ||                                 \
    (STM32_PLLI2SVCO > STM32_PLLVCO_MAX)
#error "STM32_PLLI2SVCO outside acceptable range (STM32_PLLVCO_MIN...STM32_PLLVCO_MAX)"
#endif

/**
 * @brief   PLLI2S P output clock frequency.
 */
#define STM32_PLLI2S_P_CLKOUT       (STM32_PLLI2SVCO / STM32_PLLI2SP_VALUE)

/**
 * @brief   PLLI2S Q output clock frequency.
 */
#define STM32_PLLI2S_Q_CLKOUT       (STM32_PLLI2SVCO / STM32_PLLI2SQ_VALUE)

/**
 * @brief   PLLI2S R output clock frequency.
 */
#define STM32_PLLI2S_R_CLKOUT       (STM32_PLLI2SVCO / STM32_PLLI2SR_VALUE)

/**
 * @brief   STM32_PLLI2SDIVQ field.
 */
#if (STM32_PLLI2SDIVQ_VALUE < 1) || (STM32_PLLI2SDIVQ_VALUE > 32)
#error "STM32_PLLI2SDIVQ_VALUE out of acceptable range"
#endif
#define STM32_PLLI2SDIVQ            ((STM32_PLLI2SDIVQ_VALUE - 1) << 0)

/**
 * @brief   PLLI2S Q output clock frequency after divisor.
 */
#define STM32_PLLI2SDIVQ_CLKOUT     (STM32_PLLI2S_Q_CLKOUT / STM32_PLLI2SDIVQ_VALUE)

/*
 * PLLSAI enable check.
 */
#if (STM32_CLOCK48_REQUIRED && (STM32_CK48MSEL == STM32_CK48MSEL_PLLSAI)) | \
    STM32_LCDTFT_REQUIRED ||                                                \
    (STM32_SAI1SEL == STM32_SAI1SEL_SAIPLL) ||                              \
    (STM32_SAI2SEL == STM32_SAI2SEL_SAIPLL) ||                              \
    defined(__DOXYGEN__)
/**
 * @brief   PLLSAI activation flag.
 */
#define STM32_ACTIVATE_PLLSAI       TRUE
#else
#define STM32_ACTIVATE_PLLSAI       FALSE
#endif

/**
 * @brief   STM32_PLLSAIN field.
 */
#if ((STM32_PLLSAIN_VALUE >= 49) && (STM32_PLLSAIN_VALUE <= 432)) ||        \
    defined(__DOXYGEN__)
#define STM32_PLLSAIN               (STM32_PLLSAIN_VALUE << 6)
#else
#error "invalid STM32_PLLSAIN_VALUE value specified"
#endif

/**
 * @brief   STM32_PLLSAIQ field.
 */
#if ((STM32_PLLSAIQ_VALUE >= 2) && (STM32_PLLSAIQ_VALUE <= 15)) ||          \
    defined(__DOXYGEN__)
#define STM32_PLLSAIQ               (STM32_PLLSAIQ_VALUE << 24)
#else
#error "invalid STM32_PLLSAIR_VALUE value specified"
#endif

/**
 * @brief   STM32_PLLSAIR field.
 */
#if ((STM32_PLLSAIR_VALUE >= 2) && (STM32_PLLSAIR_VALUE <= 7)) ||           \
    defined(__DOXYGEN__)
#define STM32_PLLSAIR               (STM32_PLLSAIR_VALUE << 28)
#else
#error "invalid STM32_PLLSAIR_VALUE value specified"
#endif

/**
 * @brief   STM32_PLLSAIP field.
 */
#if (STM32_PLLSAIP_VALUE == 2) || defined(__DOXYGEN__)
#define STM32_PLLSAIP               (0 << 16)

#elif STM32_PLLSAIP_VALUE == 4
#define STM32_PLLSAIP               (1 << 16)

#elif STM32_PLLSAIP_VALUE == 6
#define STM32_PLLSAIP               (2 << 16)

#elif STM32_PLLSAIP_VALUE == 8
#define STM32_PLLSAIP               (3 << 16)

#else
#error "invalid STM32_PLLSAIP_VALUE value specified"
#endif

/**
 * @brief   PLLSAI VCO frequency.
 */
#define STM32_PLLSAIVCO             (STM32_PLLCLKIN * STM32_PLLSAIN_VALUE)

/*
 * PLLSAI VCO frequency range check.
 */
#if (STM32_PLLSAIVCO < STM32_PLLVCO_MIN) ||                                 \
    (STM32_PLLSAIVCO > STM32_PLLVCO_MAX)
#error "STM32_PLLSAIVCO outside acceptable range (STM32_PLLVCO_MIN...STM32_PLLVCO_MAX)"
#endif

/**
 * @brief   PLLSAI P output clock frequency.
 */
#define STM32_PLLSAI_P_CLKOUT       (STM32_PLLSAIVCO / STM32_PLLSAIP_VALUE)

/**
 * @brief   PLLSAI Q output clock frequency.
 */
#define STM32_PLLSAI_Q_CLKOUT       (STM32_PLLSAIVCO / STM32_PLLSAIQ_VALUE)

/**
 * @brief   PLLSAI R output clock frequency.
 */
#define STM32_PLLSAI_R_CLKOUT       (STM32_PLLSAIVCO / STM32_PLLSAIR_VALUE)

/**
 * @brief   STM32_PLLSAIDIVQ field.
 */
#if (STM32_PLLSAIDIVQ_VALUE < 1) || (STM32_PLLSAIDIVQ_VALUE > 32)
#error "STM32_PLLSAIDIVQ_VALUE out of acceptable range"
#endif
#define STM32_PLLSAIDIVQ            ((STM32_PLLSAIDIVQ_VALUE - 1) << 8)

/**
 * @brief   PLLSAI Q output clock frequency after divisor.
 */
#define STM32_PLLSAIDIVQ_CLKOUT     (STM32_PLLSAI_Q_CLKOUT / STM32_PLLSAIDIVQ_VALUE)

/*
 * STM32_PLLSAIDIVR field.
 */
#if (STM32_PLLSAIDIVR_VALUE == 2) || defined(__DOXYGEN__)
#define STM32_PLLSAIDIVR            (0 << 16)

#elif STM32_PLLSAIDIVR_VALUE == 4
#define STM32_PLLSAIDIVR            (1 << 16)

#elif STM32_PLLSAIDIVR_VALUE == 8
#define STM32_PLLSAIDIVR            (2 << 16)

#elif STM32_PLLSAIDIVR_VALUE == 16
#define STM32_PLLSAIDIVR            (3 << 16)

#else
#error "invalid STM32_PLLSAIDIVR_VALUE value specified"
#endif

/**
 * @brief   PLLSAI R output clock frequency after divisor.
 */
#define STM32_PLLSAIDIVR_CLKOUT       (STM32_PLLSAI_R_CLKOUT / STM32_PLLSAIDIVR_VALUE)

/**
 * @brief   MCO1 divider clock.
 */
#if (STM32_MCO1SEL == STM32_MCO1SEL_HSI) || defined(__DOXYGEN__)
#define STM32_MCO1DIVCLK            STM32_HSICLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_LSE
#define STM32_MCO1DIVCLK            STM32_LSECLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_HSE
#define STM32_MCO1DIVCLK            STM32_HSECLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_PLL
#define STM32_MCO1DIVCLK            STM32_PLL_P_CLKOUT

#else
#error "invalid STM32_MCO1SEL value specified"
#endif

/**
 * @brief   MCO1 output pin clock.
 */
#if (STM32_MCO1PRE == STM32_MCO1PRE_DIV1) || defined(__DOXYGEN__)
#define STM32_MCO1CLK               STM32_MCO1DIVCLK

#elif STM32_MCO1PRE == STM32_MCO1PRE_DIV2
#define STM32_MCO1CLK               (STM32_MCO1DIVCLK / 2)

#elif STM32_MCO1PRE == STM32_MCO1PRE_DIV3
#define STM32_MCO1CLK               (STM32_MCO1DIVCLK / 3)

#elif STM32_MCO1PRE == STM32_MCO1PRE_DIV4
#define STM32_MCO1CLK               (STM32_MCO1DIVCLK / 4)

#elif STM32_MCO1PRE == STM32_MCO1PRE_DIV5
#define STM32_MCO1CLK               (STM32_MCO1DIVCLK / 5)

#else
#error "invalid STM32_MCO1PRE value specified"
#endif

/**
 * @brief   MCO2 divider clock.
 */
#if (STM32_MCO2SEL == STM32_MCO2SEL_HSE) || defined(__DOXYGEN__)
#define STM32_MCO2DIVCLK            STM32_HSECLK

#elif STM32_MCO2SEL == STM32_MCO2SEL_PLL
#define STM32_MCO2DIVCLK            STM32_PLL_P_CLKOUT

#elif STM32_MCO2SEL == STM32_MCO2SEL_SYSCLK
#define STM32_MCO2DIVCLK            STM32_SYSCLK

#elif STM32_MCO2SEL == STM32_MCO2SEL_PLLI2S
#define STM32_MCO2DIVCLK            STM32_PLLI2S

#else
#error "invalid STM32_MCO2SEL value specified"
#endif

/**
 * @brief   MCO2 output pin clock.
 */
#if (STM32_MCO2PRE == STM32_MCO2PRE_DIV1) || defined(__DOXYGEN__)
#define STM32_MCO2CLK               STM32_MCO2DIVCLK

#elif STM32_MCO2PRE == STM32_MCO2PRE_DIV2
#define STM32_MCO2CLK               (STM32_MCO2DIVCLK / 2)

#elif STM32_MCO2PRE == STM32_MCO2PRE_DIV3
#define STM32_MCO2CLK               (STM32_MCO2DIVCLK / 3)

#elif STM32_MCO2PRE == STM32_MCO2PRE_DIV4
#define STM32_MCO2CLK               (STM32_MCO2DIVCLK / 4)

#elif STM32_MCO2PRE == STM32_MCO2PRE_DIV5
#define STM32_MCO2CLK               (STM32_MCO2DIVCLK / 5)

#else
#error "invalid STM32_MCO2PRE value specified"
#endif

/**
 * @brief   RTC HSE divider setting.
 */
#if ((STM32_RTCPRE_VALUE >= 2) && (STM32_RTCPRE_VALUE <= 31)) ||            \
    defined(__DOXYGEN__)
#define STM32_RTCPRE                (STM32_RTCPRE_VALUE << 16)
#else
#error "invalid STM32_RTCPRE value specified"
#endif

/**
 * @brief   HSE divider toward RTC clock.
 */
#if ((STM32_RTCPRE_VALUE >= 2) && (STM32_RTCPRE_VALUE <= 31))  ||           \
    defined(__DOXYGEN__)
#define STM32_HSEDIVCLK             (STM32_HSECLK / STM32_RTCPRE_VALUE)
#else
#error "invalid STM32_RTCPRE value specified"
#endif

/**
 * @brief   RTC clock.
 */
#if (STM32_RTCSEL == STM32_RTCSEL_NOCLOCK) || defined(__DOXYGEN__)
#define STM32_RTCCLK                0

#elif STM32_RTCSEL == STM32_RTCSEL_LSE
#define STM32_RTCCLK                STM32_LSECLK

#elif STM32_RTCSEL == STM32_RTCSEL_LSI
#define STM32_RTCCLK                STM32_LSICLK

#elif STM32_RTCSEL == STM32_RTCSEL_HSEDIV
#define STM32_RTCCLK                STM32_HSEDIVCLK

#else
#error "invalid STM32_RTCSEL value specified"
#endif

/**
 * @brief   USART1 frequency.
 */
#if (STM32_USART1SEL == STM32_USART1SEL_PCLK2) || defined(__DOXYGEN__)
#define STM32_USART1CLK             STM32_PCLK2
#elif STM32_USART1SEL == STM32_USART1SEL_SYSCLK
#define STM32_USART1CLK             STM32_SYSCLK
#elif STM32_USART1SEL == STM32_USART1SEL_HSI
#define STM32_USART1CLK             STM32_HSICLK
#elif STM32_USART1SEL == STM32_USART1SEL_LSE
#define STM32_USART1CLK             STM32_LSECLK
#else
#error "invalid source selected for USART1 clock"
#endif

/**
 * @brief   USART2 frequency.
 */
#if (STM32_USART2SEL == STM32_USART2SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_USART2CLK             STM32_PCLK1
#elif STM32_USART2SEL == STM32_USART2SEL_SYSCLK
#define STM32_USART2CLK             STM32_SYSCLK
#elif STM32_USART2SEL == STM32_USART2SEL_HSI
#define STM32_USART2CLK             STM32_HSICLK
#elif STM32_USART2SEL == STM32_USART2SEL_LSE
#define STM32_USART2CLK             STM32_LSECLK
#else
#error "invalid source selected for USART2 clock"
#endif

/**
 * @brief   USART3 frequency.
 */
#if (STM32_USART3SEL == STM32_USART3SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_USART3CLK             STM32_PCLK1
#elif STM32_USART3SEL == STM32_USART3SEL_SYSCLK
#define STM32_USART3CLK             STM32_SYSCLK
#elif STM32_USART3SEL == STM32_USART3SEL_HSI
#define STM32_USART3CLK             STM32_HSICLK
#elif STM32_USART3SEL == STM32_USART3SEL_LSE
#define STM32_USART3CLK             STM32_LSECLK
#else
#error "invalid source selected for USART3 clock"
#endif

/**
 * @brief   UART4 frequency.
 */
#if (STM32_UART4SEL == STM32_UART4SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_UART4CLK              STM32_PCLK1
#elif STM32_UART4SEL == STM32_UART4SEL_SYSCLK
#define STM32_UART4CLK              STM32_SYSCLK
#elif STM32_UART4SEL == STM32_UART4SEL_HSI
#define STM32_UART4CLK              STM32_HSICLK
#elif STM32_UART4SEL == STM32_UART4SEL_LSE
#define STM32_UART4CLK              STM32_LSECLK
#else
#error "invalid source selected for UART4 clock"
#endif

/**
 * @brief   UART5 frequency.
 */
#if (STM32_UART5SEL == STM32_UART5SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_UART5CLK              STM32_PCLK1
#elif STM32_UART5SEL == STM32_UART5SEL_SYSCLK
#define STM32_UART5CLK              STM32_SYSCLK
#elif STM32_UART5SEL == STM32_UART5SEL_HSI
#define STM32_UART5CLK              STM32_HSICLK
#elif STM32_UART5SEL == STM32_UART5SEL_LSE
#define STM32_UART5CLK              STM32_LSECLK
#else
#error "invalid source selected for UART5 clock"
#endif

/**
 * @brief   USART6 frequency.
 */
#if (STM32_USART6SEL == STM32_USART6SEL_PCLK2) || defined(__DOXYGEN__)
#define STM32_USART6CLK             STM32_PCLK2
#elif STM32_USART6SEL == STM32_USART6SEL_SYSCLK
#define STM32_USART6CLK             STM32_SYSCLK
#elif STM32_USART6SEL == STM32_USART6SEL_HSI
#define STM32_USART6CLK             STM32_HSICLK
#elif STM32_USART6SEL == STM32_USART6SEL_LSE
#define STM32_USART6CLK             STM32_LSECLK
#else
#error "invalid source selected for USART6 clock"
#endif

/**
 * @brief   UART7 frequency.
 */
#if (STM32_UART7SEL == STM32_UART7SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_UART7CLK              STM32_PCLK1
#elif STM32_UART7SEL == STM32_UART7SEL_SYSCLK
#define STM32_UART7CLK              STM32_SYSCLK
#elif STM32_UART7SEL == STM32_UART7SEL_HSI
#define STM32_UART7CLK              STM32_HSICLK
#elif STM32_UART7SEL == STM32_UART7SEL_LSE
#define STM32_UART7CLK              STM32_LSECLK
#else
#error "invalid source selected for UART7 clock"
#endif

/**
 * @brief   UART8 frequency.
 */
#if (STM32_UART8SEL == STM32_UART8SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_UART8CLK              STM32_PCLK1
#elif STM32_UART8SEL == STM32_UART8SEL_SYSCLK
#define STM32_UART8CLK              STM32_SYSCLK
#elif STM32_UART8SEL == STM32_UART8SEL_HSI
#define STM32_UART8CLK              STM32_HSICLK
#elif STM32_UART8SEL == STM32_UART8SEL_LSE
#define STM32_UART8CLK              STM32_LSECLK
#else
#error "invalid source selected for UART8 clock"
#endif

/**
 * @brief   I2C1 frequency.
 */
#if (STM32_I2C1SEL == STM32_I2C1SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_I2C1CLK               STM32_PCLK1
#elif STM32_I2C1SEL == STM32_I2C1SEL_SYSCLK
#define STM32_I2C1CLK               STM32_SYSCLK
#elif STM32_I2C1SEL == STM32_I2C1SEL_HSI
#define STM32_I2C1CLK               STM32_HSICLK
#else
#error "invalid source selected for I2C1 clock"
#endif

/**
 * @brief   I2C2 frequency.
 */
#if (STM32_I2C2SEL == STM32_I2C2SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_I2C2CLK               STM32_PCLK1
#elif STM32_I2C2SEL == STM32_I2C2SEL_SYSCLK
#define STM32_I2C2CLK               STM32_SYSCLK
#elif STM32_I2C2SEL == STM32_I2C2SEL_HSI
#define STM32_I2C2CLK               STM32_HSICLK
#else
#error "invalid source selected for I2C2 clock"
#endif

/**
 * @brief   I2C3 frequency.
 */
#if (STM32_I2C3SEL == STM32_I2C3SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_I2C3CLK               STM32_PCLK1
#elif STM32_I2C3SEL == STM32_I2C3SEL_SYSCLK
#define STM32_I2C3CLK               STM32_SYSCLK
#elif STM32_I2C3SEL == STM32_I2C3SEL_HSI
#define STM32_I2C3CLK               STM32_HSICLK
#else
#error "invalid source selected for I2C3 clock"
#endif

/**
 * @brief   I2C4 frequency.
 */
#if (STM32_I2C4SEL == STM32_I2C4SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_I2C4CLK               STM32_PCLK1
#elif STM32_I2C4SEL == STM32_I2C4SEL_SYSCLK
#define STM32_I2C4CLK               STM32_SYSCLK
#elif STM32_I2C4SEL == STM32_I2C4SEL_HSI
#define STM32_I2C4CLK               STM32_HSICLK
#else
#error "invalid source selected for I2C4 clock"
#endif

/**
 * @brief   LPTIM1 frequency.
 */
#if (STM32_LPTIM1SEL == STM32_LPTIM1SEL_PCLK1) || defined(__DOXYGEN__)
#define STM32_LPTIM1CLK             STM32_PCLK1
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSI
#define STM32_LPTIM1CLK             STM32_LSICLK
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_HSI
#define STM32_LPTIM1CLK             STM32_HSICLK
#elif STM32_LPTIM1SEL == STM32_LPTIM1SEL_LSE
#define STM32_LPTIM1CLK             STM32_LSECLK
#else
#error "invalid source selected for LPTIM1 clock"
#endif

/**
 * @brief   48MHz frequency.
 */
#if STM32_CLOCK48_REQUIRED || defined(__DOXYGEN__)
#if (STM32_CK48MSEL == STM32_CK48MSEL_PLL) || defined(__DOXYGEN__)
#define STM32_PLL48CLK              (STM32_PLLVCO / STM32_PLLQ_VALUE)
#elif STM32_CK48MSEL == STM32_CK48MSEL_PLLSAI
#define STM32_PLL48CLK              (STM32_PLLSAIVCO / STM32_PLLSAIP_VALUE)
#else
#error "invalid source selected for PLL48CLK clock"
#endif
#else /* !STM32_CLOCK48_REQUIRED */
#define STM32_PLL48CLK              0
#endif /* !STM32_CLOCK48_REQUIRED */

/**
 * @brief   I2S frequency.
 */
#if (STM32_I2SSRC == STM32_I2SSRC_OFF) || defined(__DOXYGEN__)
#define STM32_I2SCLK                0
#elif STM32_I2SSRC == STM32_I2SSRC_CKIN
#define STM32_I2SCLK                0 /* Unknown, would require a board value */
#elif STM32_I2SSRC == STM32_I2SSRC_PLLI2S
#define STM32_I2SCLK                STM32_PLLI2S_R_CLKOUT
#else
#error "invalid source selected for I2S clock"
#endif

/**
 * @brief   SAI1 frequency.
 */
#if (STM32_SAI1SEL == STM32_SAI1SEL_OFF) || defined(__DOXYGEN__)
#define STM32_SAI1CLK               0
#elif STM32_SAI1SEL == STM32_SAI1SEL_SAIPLL
#define STM32_SAI1CLK               STM32_PLLSAIDIVQ_CLKOUT
#elif STM32_SAI1SEL == STM32_SAI1SEL_I2SPLL
#define STM32_SAI1CLK               STM32_PLLI2SDIVQ_CLKOUT
#elif STM32_SAI1SEL == STM32_SAI1SEL_CKIN
#define STM32_SAI1CLK               0 /* Unknown, would require a board value */
#else
#error "invalid source selected for SAI1 clock"
#endif

/**
 * @brief   SAI2 frequency.
 */
#if (STM32_SAI2SEL == STM32_SAI2SEL_OFF) || defined(__DOXYGEN__)
#define STM32_SAI2CLK               0
#elif STM32_SAI2SEL == STM32_SAI2SEL_SAIPLL
#define STM32_SAI2CLK               STM32_PLLSAIDIVQ_CLKOUT
#elif STM32_SAI2SEL == STM32_SAI2SEL_I2SPLL
#define STM32_SAI2CLK               STM32_PLLI2SDIVQ_CLKOUT
#elif STM32_SAI2SEL == STM32_SAI2SEL_CKIN
#define STM32_SAI2CLK               0 /* Unknown, would require a board value */
#else
#error "invalid source selected for SAI2 clock"
#endif

/**
 * @brief   SDMMC1 frequency.
 */
#if (STM32_SDMMC1SEL == STM32_SDMMC1SEL_PLL48CLK) || defined(__DOXYGEN__)
#define STM32_SDMMC1CLK              STM32_PLL48CLK
#elif STM32_SDMMC1SEL == STM32_SDMMC1SEL_SYSCLK
#define STM32_SDMMC1CLK              STM32_SYSCLK
#else
#error "invalid source selected for SDMMC1 clock"
#endif

/**
 * @brief   SDMMC2 frequency.
 */
#if (STM32_SDMMC2SEL == STM32_SDMMC2SEL_PLL48CLK) || defined(__DOXYGEN__)
#define STM32_SDMMC2CLK              STM32_PLL48CLK
#elif STM32_SDMMC2SEL == STM32_SDMMC2SEL_SYSCLK
#define STM32_SDMMC2CLK              STM32_SYSCLK
#else
#error "invalid source selected for SDMMC2 clock"
#endif

/**
 * @brief   Clock of timers connected to APB1
 */
#if (STM32_TIMPRE_ENABLE == FALSE) || defined(__DOXYGEN__)
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK1               (STM32_PCLK1 * 1)
#else
#define STM32_TIMCLK1               (STM32_PCLK1 * 2)
#endif
#else
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) ||                                    \
    (STM32_PPRE1 == STM32_PPRE1_DIV2) ||                                    \
    (STM32_PPRE1 == STM32_PPRE1_DIV4)
#define STM32_TIMCLK1               (STM32_HCLK * 1)
#else
#define STM32_TIMCLK1               (STM32_PCLK1 * 4)
#endif
#endif

/**
 * @brief   Clock of timers connected to APB2.
 */
#if (STM32_TIMPRE_ENABLE == FALSE) || defined(__DOXYGEN__)
#if (STM32_PPRE2 == STM32_PPRE2_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK2               (STM32_PCLK2 * 1)
#else
#define STM32_TIMCLK2               (STM32_PCLK2 * 2)
#endif
#else
#if (STM32_PPRE2 == STM32_PPRE2_DIV1) ||                                    \
    (STM32_PPRE2 == STM32_PPRE2_DIV2) ||                                    \
    (STM32_PPRE2 == STM32_PPRE2_DIV4)
#define STM32_TIMCLK2               (STM32_HCLK * 1)
#else
#define STM32_TIMCLK2               (STM32_PCLK2 * 4)
#endif
#endif

/**
 * @brief   RNG clock point.
 */
#define STM32_RNGCLK                STM32_PLL48CLK

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

#elif STM32_HCLK <= STM32_5WS_THRESHOLD
#define STM32_FLASHBITS             0x00000005

#elif STM32_HCLK <= STM32_6WS_THRESHOLD
#define STM32_FLASHBITS             0x00000006

#elif STM32_HCLK <= STM32_7WS_THRESHOLD
#define STM32_FLASHBITS             0x00000007

#elif STM32_HCLK <= STM32_8WS_THRESHOLD
#define STM32_FLASHBITS             0x00000008

#elif STM32_HCLK <= STM32_9WS_THRESHOLD
#define STM32_FLASHBITS             0x00000009

#else
#error "invalid frequency at specified VDD level"
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
