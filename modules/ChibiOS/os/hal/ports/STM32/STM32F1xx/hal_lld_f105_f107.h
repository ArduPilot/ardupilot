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
 * @defgroup STM32F10X_CL_HAL STM32F105/F107 HAL Support
 * @details HAL support for STM32 Connectivity Line sub-family.
 *
 * @ingroup HAL
 */

/**
 * @file    STM32F1xx/hal_lld_f105_f107.h
 * @brief   STM32F10x Connectivity Line HAL subsystem low level driver header.
 *
 * @addtogroup STM32F10X_CL_HAL
 * @{
 */

#ifndef _HAL_LLD_F105_F107_H_
#define _HAL_LLD_F105_F107_H_

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

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
#define STM32_HSECLK_MAX        50000000

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
#define STM32_PLL1IN_MAX        12000000

/**
 * @brief   Minimum PLL1 input clock frequency.
 */
#define STM32_PLL1IN_MIN        3000000

/**
 * @brief   Maximum PLL1 input clock frequency.
 */
#define STM32_PLL23IN_MAX       5000000

/**
 * @brief   Minimum PLL2 and PLL3 input clock frequency.
 */
#define STM32_PLL23IN_MIN       3000000

/**
 * @brief   Maximum PLL1 VCO clock frequency.
 */
#define STM32_PLL1VCO_MAX       144000000

/**
 * @brief   Minimum PLL1 VCO clock frequency.
 */
#define STM32_PLL1VCO_MIN       36000000

/**
 * @brief   Maximum PLL2 and PLL3 VCO clock frequency.
 */
#define STM32_PLL23VCO_MAX      148000000

/**
 * @brief   Minimum PLL2 and PLL3 VCO clock frequency.
 */
#define STM32_PLL23VCO_MIN      80000000

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
#define STM32_ADCCLK_MAX        14000000

/**
 * @brief   Maximum SPI/I2S clock frequency.
 */
#define STM32_SPII2S_MAX        18000000
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

#define STM32_ADCPRE_DIV2       (0 << 14)   /**< PPRE2 divided by 2.        */
#define STM32_ADCPRE_DIV4       (1 << 14)   /**< PPRE2 divided by 4.        */
#define STM32_ADCPRE_DIV6       (2 << 14)   /**< PPRE2 divided by 6.        */
#define STM32_ADCPRE_DIV8       (3 << 14)   /**< PPRE2 divided by 8.        */

#define STM32_PLLSRC_HSI        (0 << 16)   /**< PLL clock source is HSI.   */
#define STM32_PLLSRC_PREDIV1    (1 << 16)   /**< PLL clock source is
                                                 PREDIV1.                   */

#define STM32_OTGFSPRE_DIV2     (1 << 22)   /**< HCLK*2 divided by 2.       */
#define STM32_OTGFSPRE_DIV3     (0 << 22)   /**< HCLK*2 divided by 3.       */

#define STM32_MCOSEL_NOCLOCK    (0 << 24)   /**< No clock on MCO pin.       */
#define STM32_MCOSEL_SYSCLK     (4 << 24)   /**< SYSCLK on MCO pin.         */
#define STM32_MCOSEL_HSI        (5 << 24)   /**< HSI clock on MCO pin.      */
#define STM32_MCOSEL_HSE        (6 << 24)   /**< HSE clock on MCO pin.      */
#define STM32_MCOSEL_PLLDIV2    (7 << 24)   /**< PLL/2 clock on MCO pin.    */
#define STM32_MCOSEL_PLL2       (8 << 24)   /**< PLL2 clock on MCO pin.     */
#define STM32_MCOSEL_PLL3DIV2   (9 << 24)   /**< PLL3/2 clock on MCO pin.   */
#define STM32_MCOSEL_XT1        (10 << 24)  /**< XT1 clock on MCO pin.      */
#define STM32_MCOSEL_PLL3       (11 << 24)  /**< PLL3 clock on MCO pin.     */
/** @} */

/**
 * @name    RCC_BDCR register bits definitions
 * @{
 */
#define STM32_RTCSEL_MASK       (3 << 8)    /**< RTC clock source mask.     */
#define STM32_RTCSEL_NOCLOCK    (0 << 8)    /**< No clock.                  */
#define STM32_RTCSEL_LSE        (1 << 8)    /**< LSE used as RTC clock.     */
#define STM32_RTCSEL_LSI        (2 << 8)    /**< LSI used as RTC clock.     */
#define STM32_RTCSEL_HSEDIV     (3 << 8)    /**< HSE divided by 128 used as
                                                 RTC clock.                 */
/** @} */

/**
 * @name    RCC_CFGR2 register bits definitions
 * @{
 */
#define STM32_PREDIV1SRC_HSE    (0 << 16)   /**< PREDIV1 source is HSE.     */
#define STM32_PREDIV1SRC_PLL2   (1 << 16)   /**< PREDIV1 source is PLL2.    */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Main clock source selection.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 25MHz crystal using both PLL and PLL2.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                    STM32_SW_PLL
#endif

/**
 * @brief   Clock source for the PLL.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 25MHz crystal using both PLL and PLL2.
 */
#if !defined(STM32_PLLSRC) || defined(__DOXYGEN__)
#define STM32_PLLSRC                STM32_PLLSRC_PREDIV1
#endif

/**
 * @brief   PREDIV1 clock source.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 25MHz crystal using both PLL and PLL2.
 */
#if !defined(STM32_PREDIV1SRC) || defined(__DOXYGEN__)
#define STM32_PREDIV1SRC            STM32_PREDIV1SRC_HSE
#endif

/**
 * @brief   PREDIV1 division factor.
 * @note    The allowed range is 1...16.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 25MHz crystal using both PLL and PLL2.
 */
#if !defined(STM32_PREDIV1_VALUE) || defined(__DOXYGEN__)
#define STM32_PREDIV1_VALUE         5
#endif

/**
 * @brief   PLL multiplier value.
 * @note    The allowed range is 4...9.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 25MHz crystal using both PLL and PLL2.
 */
#if !defined(STM32_PLLMUL_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLMUL_VALUE          9
#endif

/**
 * @brief   PREDIV2 division factor.
 * @note    The allowed range is 1...16.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 25MHz crystal using both PLL and PLL2.
 */
#if !defined(STM32_PREDIV2_VALUE) || defined(__DOXYGEN__)
#define STM32_PREDIV2_VALUE         5
#endif

/**
 * @brief   PLL2 multiplier value.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 25MHz crystal using both PLL and PLL2.
 */
#if !defined(STM32_PLL2MUL_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL2MUL_VALUE         8
#endif

/**
 * @brief   PLL3 multiplier value.
 * @note    The default value is calculated for a 50MHz clock from
 *          a 25MHz crystal.
 */
#if !defined(STM32_PLL3MUL_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3MUL_VALUE         10
#endif

/**
 * @brief   AHB prescaler value.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 25MHz crystal using both PLL and PLL2.
 */
#if !defined(STM32_HPRE) || defined(__DOXYGEN__)
#define STM32_HPRE                  STM32_HPRE_DIV1
#endif

/**
 * @brief   APB1 prescaler value.
 */
#if !defined(STM32_PPRE1) || defined(__DOXYGEN__)
#define STM32_PPRE1                 STM32_PPRE1_DIV2
#endif

/**
 * @brief   APB2 prescaler value.
 */
#if !defined(STM32_PPRE2) || defined(__DOXYGEN__)
#define STM32_PPRE2                 STM32_PPRE2_DIV2
#endif

/**
 * @brief   ADC prescaler value.
 */
#if !defined(STM32_ADCPRE) || defined(__DOXYGEN__)
#define STM32_ADCPRE                STM32_ADCPRE_DIV4
#endif

/**
 * @brief   USB clock setting.
 */
#if !defined(STM32_OTG_CLOCK_REQUIRED) || defined(__DOXYGEN__)
#define STM32_OTG_CLOCK_REQUIRED    TRUE
#endif

/**
 * @brief   OTG prescaler initialization.
 */
#if !defined(STM32_OTGFSPRE) || defined(__DOXYGEN__)
#define STM32_OTGFSPRE              STM32_OTGFSPRE_DIV3
#endif

/**
 * @brief   Dedicated I2S clock setting.
 */
#if !defined(STM32_I2S_CLOCK_REQUIRED) || defined(__DOXYGEN__)
#define STM32_I2S_CLOCK_REQUIRED    FALSE
#endif

/**
 * @brief   MCO pin setting.
 */
#if !defined(STM32_MCOSEL) || defined(__DOXYGEN__)
#define STM32_MCOSEL                   STM32_MCOSEL_NOCLOCK
#endif

/**
 * @brief   RTC clock source.
 */
#if !defined(STM32_RTCSEL) || defined(__DOXYGEN__)
#define STM32_RTCSEL                   STM32_RTCSEL_HSEDIV
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(STM32F107_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F107_MCUCONF not defined"
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

#if (STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_PREDIV1)
#error "HSE not enabled, required by STM32_SW and STM32_PLLSRC"
#endif

#if (STM32_MCOSEL == STM32_MCOSEL_HSE) ||                                   \
    (((STM32_MCOSEL == STM32_MCOSEL_PLLDIV2) ||                             \
      (STM32_MCOSEL == STM32_MCOSEL_PLL2) ||                                \
      (STM32_MCOSEL == STM32_MCOSEL_PLL3) ||                                \
      (STM32_MCOSEL == STM32_MCOSEL_PLL3DIV2)) &&                           \
     (STM32_PLLSRC == STM32_PLLSRC_HSE)) ||                                 \
    (STM32_MCOSEL == STM32_MCOSEL_XT1)
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

#endif /* !STM32_LSE_ENABLED */

/* PLL1 activation conditions.*/
#if STM32_OTG_CLOCK_REQUIRED ||                                             \
    (STM32_SW == STM32_SW_PLL) ||                                           \
    (STM32_MCOSEL == STM32_MCOSEL_PLLDIV2) ||                               \
    defined(__DOXYGEN__)
/**
 * @brief   PLL1 activation flag.
 */
#define STM32_ACTIVATE_PLL1         TRUE
#else
#define STM32_ACTIVATE_PLL1         FALSE
#endif

/* PLL2 activation conditions.*/
#if ((STM32_PREDIV1SRC == STM32_PREDIV1SRC_PLL2) && STM32_ACTIVATE_PLL1) || \
    (STM32_MCOSEL == STM32_MCOSEL_PLL2) || defined(__DOXYGEN__)
/**
 * @brief   PLL2 activation flag.
 */
#define STM32_ACTIVATE_PLL2         TRUE
#else
#define STM32_ACTIVATE_PLL2         FALSE
#endif

/* PLL3 activation conditions.*/
#if STM32_I2S_CLOCK_REQUIRED ||                                             \
    (STM32_MCOSEL == STM32_MCOSEL_PLL3DIV2) ||                              \
    (STM32_MCOSEL == STM32_MCOSEL_PLL3) ||                                  \
    defined(__DOXYGEN__)
/**
 * @brief   PLL3 activation flag.
 */
#define STM32_ACTIVATE_PLL3         TRUE
#else
#define STM32_ACTIVATE_PLL3         FALSE
#endif

/**
 * @brief   PREDIV1 field.
 */
#if (STM32_PREDIV1_VALUE >= 1) && (STM32_PREDIV1_VALUE <= 16) ||            \
    defined(__DOXYGEN__)
#define STM32_PREDIV1               ((STM32_PREDIV1_VALUE - 1) << 0)
#else
#error "invalid STM32_PREDIV1_VALUE value specified"
#endif

/**
 * @brief   PREDIV2 field.
 */
#if (STM32_PREDIV2_VALUE >= 1) && (STM32_PREDIV2_VALUE <= 16) ||            \
    defined(__DOXYGEN__)
#define STM32_PREDIV2               ((STM32_PREDIV2_VALUE - 1) << 4)
#else
#error "invalid STM32_PREDIV2_VALUE value specified"
#endif

/**
 * @brief   PLLMUL field.
 */
#if ((STM32_PLLMUL_VALUE >= 4) && (STM32_PLLMUL_VALUE <= 9)) ||             \
    defined(__DOXYGEN__)
#define STM32_PLLMUL                ((STM32_PLLMUL_VALUE - 2) << 18)
#else
#error "invalid STM32_PLLMUL_VALUE value specified"
#endif

/**
 * @brief   PLL2MUL field.
 */
#if ((STM32_PLL2MUL_VALUE >= 8) && (STM32_PLL2MUL_VALUE <= 14)) ||          \
    defined(__DOXYGEN__)
#define STM32_PLL2MUL               ((STM32_PLL2MUL_VALUE - 2) << 8)
#elif (STM32_PLL2MUL_VALUE == 16)
#define STM32_PLL2MUL               (14 << 8)
#elif (STM32_PLL2MUL_VALUE == 20)
#define STM32_PLL2MUL               (15 << 8)
#else
#error "invalid STM32_PLL2MUL_VALUE value specified"
#endif

/**
 * @brief   PLL3MUL field.
 */
#if ((STM32_PLL3MUL_VALUE >= 8) && (STM32_PLL3MUL_VALUE <= 14)) ||          \
    defined(__DOXYGEN__)
#define STM32_PLL3MUL               ((STM32_PLL3MUL_VALUE - 2) << 12)
#elif (STM32_PLL3MUL_VALUE == 16)
#define STM32_PLL3MUL               (14 << 12)
#elif (STM32_PLL3MUL_VALUE == 20)
#define STM32_PLL3MUL               (15 << 12)
#else
#error "invalid STM32_PLL3MUL_VALUE value specified"
#endif

/**
 * @brief   PLL2 input frequency.
 */
#define STM32_PLL2CLKIN             (STM32_HSECLK / STM32_PREDIV2_VALUE)

/* PLL2 input frequency range check.*/
#if (STM32_PLL2CLKIN < STM32_PLL23IN_MIN) ||                                \
    (STM32_PLL2CLKIN > STM32_PLL23IN_MAX)
#error "STM32_PLL2CLKIN outside acceptable range (STM32_PLL23IN_MIN...STM32_PLL23IN_MAX)"
#endif

/**
 * @brief   PLL2 output clock frequency.
 */
#define STM32_PLL2CLKOUT            (STM32_PLL2CLKIN * STM32_PLL2MUL_VALUE)

/**
 * @brief   PLL2 VCO clock frequency.
 */
#define STM32_PLL2VCO               (STM32_PLL2CLKOUT * 2)

/* PLL2 output frequency range check.*/
#if (STM32_PLL2VCO < STM32_PLL23VCO_MIN) ||                                 \
    (STM32_PLL2VCO > STM32_PLL23VCO_MAX)
#error "STM32_PLL2VCO outside acceptable range (STM32_PLL23VCO_MIN...STM32_PLL23VCO_MAX)"
#endif

/**
 * @brief   PLL3 input frequency.
 */
#define STM32_PLL3CLKIN             (STM32_HSECLK / STM32_PREDIV2_VALUE)

/* PLL3 input frequency range check.*/
#if (STM32_PLL3CLKIN < STM32_PLL23IN_MIN) ||                                \
    (STM32_PLL3CLKIN > STM32_PLL23IN_MAX)
#error "STM32_PLL3CLKIN outside acceptable range (STM32_PLL23IN_MIN...STM32_PLL23IN_MAX)"
#endif

/**
 * @brief   PLL3 output clock frequency.
 */
#define STM32_PLL3CLKOUT            (STM32_PLL3CLKIN * STM32_PLL3MUL_VALUE)

/**
 * @brief   PLL3 VCO clock frequency.
 */
#define STM32_PLL3VCO               (STM32_PLL3CLKOUT * 2)

/* PLL3 output frequency range check.*/
#if (STM32_PLL3VCO < STM32_PLL23VCO_MIN) ||                                 \
    (STM32_PLL3VCO > STM32_PLL23VCO_MAX)
#error "STM32_PLL3CLKOUT outside acceptable range (STM32_PLL23VCO_MIN...STM32_PLL23VCO_MAX)"
#endif

/**
 * @brief   PREDIV1 input frequency.
 */
#if (STM32_PREDIV1SRC == STM32_PREDIV1SRC_HSE) || defined(__DOXYGEN__)
#define STM32_PREDIV1CLK            STM32_HSECLK
#elif STM32_PREDIV1SRC == STM32_PREDIV1SRC_PLL2
#define STM32_PREDIV1CLK            STM32_PLL2CLKOUT
#else
#error "invalid STM32_PREDIV1SRC value specified"
#endif

/**
 * @brief   PLL input clock frequency.
 */
#if (STM32_PLLSRC == STM32_PLLSRC_PREDIV1) || defined(__DOXYGEN__)
#define STM32_PLLCLKIN              (STM32_PREDIV1CLK / STM32_PREDIV1_VALUE)
#elif STM32_PLLSRC == STM32_PLLSRC_HSI
#define STM32_PLLCLKIN              (STM32_HSICLK / 2)
#else
#error "invalid STM32_PLLSRC value specified"
#endif

/* PLL input frequency range check.*/
#if (STM32_PLLCLKIN < STM32_PLL1IN_MIN) || (STM32_PLLCLKIN > STM32_PLL1IN_MAX)
#error "STM32_PLLCLKIN outside acceptable range (STM32_PLL1IN_MIN...STM32_PLL1IN_MAX)"
#endif

/**
 * @brief   PLL output clock frequency.
 */
#define STM32_PLLCLKOUT             (STM32_PLLCLKIN * STM32_PLLMUL_VALUE)

/**
 * @brief   PLL VCO clock frequency.
 */
#define STM32_PLLVCO                (STM32_PLLCLKOUT * 2)

/* PLL output frequency range check.*/
#if (STM32_PLLVCO < STM32_PLL1VCO_MIN) || (STM32_PLLVCO > STM32_PLL1VCO_MAX)
#error "STM32_PLLVCO outside acceptable range (STM32_PLL1VCO_MIN...STM32_PLL1VCO_MAX)"
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
 * @brief   RTC clock.
 */
#if (STM32_RTCSEL == STM32_RTCSEL_LSE) || defined(__DOXYGEN__)
#define STM32_RTCCLK                STM32_LSECLK
#elif STM32_RTCSEL == STM32_RTCSEL_LSI
#define STM32_RTCCLK                STM32_LSICLK
#elif STM32_RTCSEL == STM32_RTCSEL_HSEDIV
#define STM32_RTCCLK                (STM32_HSECLK / 128)
#elif STM32_RTCSEL == STM32_RTCSEL_NOCLOCK
#define STM32_RTCCLK                0
#else
#error "invalid source selected for RTC clock"
#endif

/**
 * @brief   ADC frequency.
 */
#if (STM32_ADCPRE == STM32_ADCPRE_DIV2) || defined(__DOXYGEN__)
#define STM32_ADCCLK                (STM32_PCLK2 / 2)
#elif STM32_ADCPRE == STM32_ADCPRE_DIV4
#define STM32_ADCCLK                (STM32_PCLK2 / 4)
#elif STM32_ADCPRE == STM32_ADCPRE_DIV6
#define STM32_ADCCLK                (STM32_PCLK2 / 6)
#elif STM32_ADCPRE == STM32_ADCPRE_DIV8
#define STM32_ADCCLK                (STM32_PCLK2 / 8)
#else
#error "invalid STM32_ADCPRE value specified"
#endif

/* ADC frequency check.*/
#if STM32_ADCCLK > STM32_ADCCLK_MAX
#error "STM32_ADCCLK exceeding maximum frequency (STM32_ADCCLK_MAX)"
#endif

/**
 * @brief   OTG frequency.
 */
#if (STM32_OTGFSPRE == STM32_OTGFSPRE_DIV3) || defined(__DOXYGEN__)
#define STM32_OTGFSCLK              (STM32_PLLVCO / 3)
#elif (STM32_OTGFSPRE == STM32_OTGFSPRE_DIV2)
#define STM32_OTGFSCLK              (STM32_PLLVCO / 2)
#else
#error "invalid STM32_OTGFSPRE value specified"
#endif

/**
 * @brief   Timers 2, 3, 4, 5, 6, 7 clock.
 */
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK1               (STM32_PCLK1 * 1)
#else
#define STM32_TIMCLK1               (STM32_PCLK1 * 2)
#endif

/**
 * @brief   Timers 1, 8 clock.
 */
#if (STM32_PPRE2 == STM32_PPRE2_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK2               (STM32_PCLK2 * 1)
#else
#define STM32_TIMCLK2               (STM32_PCLK2 * 2)
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

#endif /* _HAL_LLD_F105_F107_H_ */

/** @} */
