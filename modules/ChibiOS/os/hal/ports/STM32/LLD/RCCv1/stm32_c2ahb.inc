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
 * @file    RCCv1/stm32_c2ahb.inc
 * @brief   Shared Core 2 AHB clock handler.
 *
 * @addtogroup STM32_C2AHB_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @name    C2HPRE field bits definitions
 * @{
 */
#define STM32_C2HPRE_MASK       (15U << RCC_EXTCFGR_C2HPRE_Pos)
#define STM32_C2HPRE_FIELD(n)   ((n) << RCC_EXTCFGR_C2HPRE_Pos)
#define STM32_C2HPRE_DIV1       STM32_C2HPRE_FIELD(0U)
#define STM32_C2HPRE_DIV2       STM32_C2HPRE_FIELD(8U)
#define STM32_C2HPRE_DIV3       STM32_C2HPRE_FIELD(1U)
#define STM32_C2HPRE_DIV4       STM32_C2HPRE_FIELD(9U)
#define STM32_C2HPRE_DIV5       STM32_C2HPRE_FIELD(2U)
#define STM32_C2HPRE_DIV6       STM32_C2HPRE_FIELD(5U)
#define STM32_C2HPRE_DIV8       STM32_C2HPRE_FIELD(10U)
#define STM32_C2HPRE_DIV10      STM32_C2HPRE_FIELD(6U)
#define STM32_C2HPRE_DIV16      STM32_C2HPRE_FIELD(11U)
#define STM32_C2HPRE_DIV32      STM32_C2HPRE_FIELD(7U)
#define STM32_C2HPRE_DIV64      STM32_C2HPRE_FIELD(12U)
#define STM32_C2HPRE_DIV128     STM32_C2HPRE_FIELD(13U)
#define STM32_C2HPRE_DIV256     STM32_C2HPRE_FIELD(14U)
#define STM32_C2HPRE_DIV512     STM32_C2HPRE_FIELD(15U)
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/

/* Checks on configurations.*/
#if !defined(STM32_C2HPRE)
#error "STM32_C2HPRE not defined in mcuconf.h"
#endif

/* Input checks.*/
#if !defined(STM32_SYSCLK_MAX)
#error "STM32_SYSCLK_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_SYSCLK)
#error "STM32_SYSCLK not defined in hal_lld.h"
#endif

/**
 * @brief   HCLK2 frequency.
 */
#if (STM32_C2HPRE == STM32_C2HPRE_DIV1) || defined(__DOXYGEN__)
#define STM32_HCLK2                 (STM32_SYSCLK / 1)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV2
#define STM32_HCLK2                 (STM32_SYSCLK / 2)

#elif STM32_C2HPRE == STM32_C2HPRE_DIV4
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
 * HCLK2 frequency check.
 */
#if STM32_HCLK2 > STM32_SYSCLK_MAX
#error "STM32_HCLK2 exceeding maximum frequency (STM32_SYSCLK_MAX)"
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
