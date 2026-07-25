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
 * @file    STM32F4xx/hal_lld.h
 * @brief   STM32F4xx/STM32F2xx HAL subsystem low level driver header.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "stm32_registry.h"

#if defined(STM32F413xx)
#include "hal_lld_type2.h"
#else
#include "hal_lld_type1.h"
#endif

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "mpu_v7m.h"
#include "stm32_isr.h"
#include "stm32_dma.h"
#include "stm32_exti.h"
#include "stm32_rcc.h"
#include "stm32_tim.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/**
 * @brief   MCO1 divider clock.
 */
#if (STM32_MCO1SEL == STM32_MCO1SEL_HSI) || defined(__DOXYGEN__)
#define STM32_MCO1DIVCLK             STM32_HSICLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_LSE
#define STM32_MCO1DIVCLK             STM32_LSECLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_HSE
#define STM32_MCO1DIVCLK             STM32_HSECLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_PLL
#define STM32_MCO1DIVCLK             STM32_PLLCLKOUT

#else
#error "invalid STM32_MCO1SEL value specified"
#endif

/**
 * @brief   MCO1 output pin clock.
 */
#if (STM32_MCO1PRE == STM32_MCO1PRE_DIV1) || defined(__DOXYGEN__)
#define STM32_MCO1CLK                STM32_MCO1DIVCLK

#elif STM32_MCO1PRE == STM32_MCO1PRE_DIV2
#define STM32_MCO1CLK                (STM32_MCO1DIVCLK / 2)

#elif STM32_MCO1PRE == STM32_MCO1PRE_DIV3
#define STM32_MCO1CLK                (STM32_MCO1DIVCLK / 3)

#elif STM32_MCO1PRE == STM32_MCO1PRE_DIV4
#define STM32_MCO1CLK                (STM32_MCO1DIVCLK / 4)

#elif STM32_MCO1PRE == STM32_MCO1PRE_DIV5
#define STM32_MCO1CLK                (STM32_MCO1DIVCLK / 5)

#else
#error "invalid STM32_MCO1PRE value specified"
#endif

/**
 * @brief   MCO2 divider clock.
 */
#if (STM32_MCO2SEL == STM32_MCO2SEL_HSE) || defined(__DOXYGEN__)
#define STM32_MCO2DIVCLK             STM32_HSECLK

#elif STM32_MCO2SEL == STM32_MCO2SEL_PLL
#define STM32_MCO2DIVCLK             STM32_PLLCLKOUT

#elif STM32_MCO2SEL == STM32_MCO2SEL_SYSCLK
#define STM32_MCO2DIVCLK             STM32_SYSCLK

#elif STM32_MCO2SEL == STM32_MCO2SEL_PLLI2S
#define STM32_MCO2DIVCLK             STM32_PLLI2S

#else
#error "invalid STM32_MCO2SEL value specified"
#endif

/**
 * @brief   MCO2 output pin clock.
 */
#if (STM32_MCO2PRE == STM32_MCO2PRE_DIV1) || defined(__DOXYGEN__)
#define STM32_MCO2CLK                STM32_MCO2DIVCLK

#elif STM32_MCO2PRE == STM32_MCO2PRE_DIV2
#define STM32_MCO2CLK                (STM32_MCO2DIVCLK / 2)

#elif STM32_MCO2PRE == STM32_MCO2PRE_DIV3
#define STM32_MCO2CLK                (STM32_MCO2DIVCLK / 3)

#elif STM32_MCO2PRE == STM32_MCO2PRE_DIV4
#define STM32_MCO2CLK                (STM32_MCO2DIVCLK / 4)

#elif STM32_MCO2PRE == STM32_MCO2PRE_DIV5
#define STM32_MCO2CLK                (STM32_MCO2DIVCLK / 5)

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
 * @brief   48MHz frequency.
 */
#if STM32_CLOCK48_REQUIRED || defined(__DOXYGEN__)
#if STM32_HAS_RCC_CK48MSEL || defined(__DOXYGEN__)
#if (STM32_CK48MSEL == STM32_CK48MSEL_PLL) || defined(__DOXYGEN__)
#define STM32_PLL48CLK              (STM32_PLLVCO / STM32_PLLQ_VALUE)
#elif STM32_CK48MSEL == STM32_CK48MSEL_PLLALT
#define STM32_PLL48CLK              STM32_PLL48CLK_ALTSRC
#else
#error "invalid source selected for PLL48CLK clock"
#endif
#else /* !STM32_HAS_RCC_CK48MSEL */
#define STM32_PLL48CLK              (STM32_PLLVCO / STM32_PLLQ_VALUE)
#endif /* !STM32_HAS_RCC_CK48MSEL */
#else /* !STM32_CLOCK48_REQUIRED */
#define STM32_PLL48CLK              0
#endif /* STM32_CLOCK48_REQUIRED */

#if !STM32_HAS_RCC_DCKCFGR || (STM32_TIMPRE == STM32_TIMPRE_PCLK) ||        \
    defined(__DOXYGEN__)
/**
 * @brief   Clock of timers connected to APB1
 *          (Timers 2, 3, 4, 5, 6, 7, 12, 13, 14).
 */
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK1               (STM32_PCLK1 * 1)
#else
#define STM32_TIMCLK1               (STM32_PCLK1 * 2)
#endif

/**
 * @brief   Clock of timers connected to APB2 (Timers 1, 8, 9, 10, 11).
 */
#if (STM32_PPRE2 == STM32_PPRE2_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK2               (STM32_PCLK2 * 1)
#else
#define STM32_TIMCLK2               (STM32_PCLK2 * 2)
#endif

#else /* STM32_HAS_RCC_DCKCFGR && (STM32_TIMPRE == STM32_TIMPRE_HCLK) */
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) ||                                    \
    (STM32_PPRE1 == STM32_PPRE1_DIV2) ||                                    \
    ((STM32_PPRE1 == STM32_PPRE1_DIV4) &&                                   \
     (STM32_TIMPRE_PRESCALE4 == TRUE)) || defined(__DOXYGEN__)
#define STM32_TIMCLK1               STM32_HCLK
#else
#define STM32_TIMCLK1               (STM32_PCLK1 * 4)
#endif

#if (STM32_PPRE2 == STM32_PPRE2_DIV1) ||                                    \
    (STM32_PPRE2 == STM32_PPRE2_DIV2) ||                                    \
    ((STM32_PPRE2 == STM32_PPRE2_DIV4) &&                                   \
     (STM32_TIMPRE_PRESCALE4 == TRUE)) || defined(__DOXYGEN__)
#define STM32_TIMCLK2               STM32_HCLK
#else
#define STM32_TIMCLK2               (STM32_PCLK2 * 4)
#endif
#endif /* STM32_HAS_RCC_DCKCFGR && (STM32_TIMPRE == STM32_TIMPRE_HCLK) */

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

#endif /* HAL_LLD_H */

/** @} */
