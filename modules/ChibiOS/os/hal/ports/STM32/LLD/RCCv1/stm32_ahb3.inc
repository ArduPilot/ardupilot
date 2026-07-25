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
 * @file    RCCv1/stm32_ahb3.inc
 * @brief   Shared AHB3 clock handler.
 *
 * @addtogroup STM32_AHB3_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @name    SHDHPRE field bits definitions
 * @{
 */
#define STM32_SHDHPRE_MASK      (15U << RCC_EXTCFGR_SHDHPRE_Pos)
#define STM32_SHDHPRE_FIELD(n)  ((n) << RCC_EXTCFGR_SHDHPRE_Pos)
#define STM32_SHDHPRE_DIV1      STM32_SHDHPRE_FIELD(0U)
#define STM32_SHDHPRE_DIV2      STM32_SHDHPRE_FIELD(8U)
#define STM32_SHDHPRE_DIV3      STM32_SHDHPRE_FIELD(1U)
#define STM32_SHDHPRE_DIV4      STM32_SHDHPRE_FIELD(9U)
#define STM32_SHDHPRE_DIV5      STM32_SHDHPRE_FIELD(2U)
#define STM32_SHDHPRE_DIV6      STM32_SHDHPRE_FIELD(5U)
#define STM32_SHDHPRE_DIV8      STM32_SHDHPRE_FIELD(10U)
#define STM32_SHDHPRE_DIV10     STM32_SHDHPRE_FIELD(6U)
#define STM32_SHDHPRE_DIV16     STM32_SHDHPRE_FIELD(11U)
#define STM32_SHDHPRE_DIV32     STM32_SHDHPRE_FIELD(7U)
#define STM32_SHDHPRE_DIV64     STM32_SHDHPRE_FIELD(12U)
#define STM32_SHDHPRE_DIV128    STM32_SHDHPRE_FIELD(13U)
#define STM32_SHDHPRE_DIV256    STM32_SHDHPRE_FIELD(14U)
#define STM32_SHDHPRE_DIV512    STM32_SHDHPRE_FIELD(15U)

#define STM32_SHDHPREF_MASK     (1U << RCC_EXTCFGR_SHDHPREF_Pos)

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/

/* Checks on configurations.*/
#if !defined(STM32_SHDHPRE)
#error "STM32_SHDHPRE not defined in mcuconf.h"
#endif

/**
 * @brief   AHB3 HCLK3/PCLK3 frequency.
 */
#if (STM32_SHDHPRE == STM32_SHDHPRE_DIV1) || defined(__DOXYGEN__)
#define STM32_HCLK3                 (STM32_SYSCLK / 1)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV2
#define STM32_HCLK3                 (STM32_SYSCLK / 2)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV4
#define STM32_HCLK3                 (STM32_SYSCLK / 3)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV4
#define STM32_HCLK3                 (STM32_SYSCLK / 4)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV5
#define STM32_HCLK3                 (STM32_SYSCLK / 5)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV6
#define STM32_HCLK3                 (STM32_SYSCLK / 6)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV8
#define STM32_HCLK3                 (STM32_SYSCLK / 8)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV10
#define STM32_HCLK3                 (STM32_SYSCLK / 10)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV16
#define STM32_HCLK3                 (STM32_SYSCLK / 16)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV32
#define STM32_HCLK3                 (STM32_SYSCLK / 32)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV64
#define STM32_HCLK3                 (STM32_SYSCLK / 64)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV128
#define STM32_HCLK3                 (STM32_SYSCLK / 128)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV256
#define STM32_HCLK3                 (STM32_SYSCLK / 256)

#elif STM32_SHDHPRE == STM32_SHDHPRE_DIV512
#define STM32_HCLK3                 (STM32_SYSCLK / 512)

#else
#error "invalid STM32_SHDHPRE value specified"
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
