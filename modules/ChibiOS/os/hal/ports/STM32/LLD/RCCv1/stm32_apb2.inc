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
 * @file    RCCv1/stm32_apb2.inc
 * @brief   Shared APB2 clock handler.
 *
 * @addtogroup STM32_APB2_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @name    PPRE2 field bits definitions (legacy)
 * @{
 */
#if defined(RCC_CFGR_PPRE2_Pos) || defined(__DOXYGEN__)
#define STM32_PPRE2_POS                     RCC_CFGR_PPRE2_Pos
#define STM32_PPRE2_MASK                    RCC_CFGR_PPRE2_Msk

#elif defined(RCC_CFGR2_PPRE2_Pos)
#define STM32_PPRE2_POS                     RCC_CFGR2_PPRE2_Pos
#define STM32_PPRE2_MASK                    RCC_CFGR2_PPRE2_Msk

#else
#error "unknown register name"
#endif

#define STM32_PPRE2_FIELD(n)                ((n) << STM32_PPRE2_POS)
#define STM32_PPRE2_DIV1                    STM32_PPRE2_FIELD(0U)
#define STM32_PPRE2_DIV2                    STM32_PPRE2_FIELD(4U)
#define STM32_PPRE2_DIV4                    STM32_PPRE2_FIELD(5U)
#define STM32_PPRE2_DIV8                    STM32_PPRE2_FIELD(6U)
#define STM32_PPRE2_DIV16                   STM32_PPRE2_FIELD(7U)
/** @} */


/**
 * @name    PPRE2 field helpers
 * @{
 */
#if defined(RCC_CFGR_PPRE2_Pos) || defined(__DOXYGEN__)
#define RCC_CFGR_PPRE2_FIELD(n)             ((n) << RCC_CFGR_PPRE2_Pos)
#if !defined(RCC_CFGR_PPRE2_DIV1)
#define RCC_CFGR_PPRE2_DIV1                 RCC_CFGR_PPRE2_FIELD(0U)
#define RCC_CFGR_PPRE2_DIV2                 RCC_CFGR_PPRE2_FIELD(4U)
#define RCC_CFGR_PPRE2_DIV4                 RCC_CFGR_PPRE2_FIELD(5U)
#define RCC_CFGR_PPRE2_DIV8                 RCC_CFGR_PPRE2_FIELD(6U)
#define RCC_CFGR_PPRE2_DIV16                RCC_CFGR_PPRE2_FIELD(7U)
#endif

#elif defined(RCC_CFGR2_PPRE2_Pos)
#define RCC_CFGR2_PPRE2_FIELD(n)            ((n) << RCC_CFGR2_PPRE2_Pos)
#if !defined(RCC_CFGR2_PPRE2_DIV1)
#define RCC_CFGR2_PPRE2_DIV1                RCC_CFGR2_PPRE2_FIELD(0U)
#define RCC_CFGR2_PPRE2_DIV2                RCC_CFGR2_PPRE2_FIELD(4U)
#define RCC_CFGR2_PPRE2_DIV4                RCC_CFGR2_PPRE2_FIELD(5U)
#define RCC_CFGR2_PPRE2_DIV8                RCC_CFGR2_PPRE2_FIELD(6U)
#define RCC_CFGR2_PPRE2_DIV16               RCC_CFGR2_PPRE2_FIELD(7U)
#endif

#else
#error "unknown register name"
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/

/* Checks on configurations.*/
#if !defined(STM32_PPRE2)
#error "STM32_PPRE2 not defined in mcuconf.h"
#endif

/* Input checks.*/
#if !defined(STM32_PCLK2_MAX)
#error "STM32_PCLK2_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_HCLK)
#error "STM32_HCLK not defined in hal_lld.h"
#endif

/**
 * @brief   APB2 frequency.
 */
#if (STM32_PPRE2 == STM32_PPRE2_DIV1) || defined(__DOXYGEN__)
#define STM32_PCLK2                 (STM32_HCLK / 1U)

#elif STM32_PPRE2 == STM32_PPRE2_DIV2
#define STM32_PCLK2                 (STM32_HCLK / 2U)

#elif STM32_PPRE2 == STM32_PPRE2_DIV4
#define STM32_PCLK2                 (STM32_HCLK / 4U)

#elif STM32_PPRE2 == STM32_PPRE2_DIV8
#define STM32_PCLK2                 (STM32_HCLK / 8U)

#elif STM32_PPRE2 == STM32_PPRE2_DIV16
#define STM32_PCLK2                 (STM32_HCLK / 16U)

#else
#error "invalid STM32_PPRE2 value specified"
#endif

/*
 * APB2 frequency check.
 */
#if STM32_PCLK2 > STM32_PCLK2_MAX
#error "STM32_PCLK2 exceeding maximum frequency (STM32_PCLK2_MAX)"
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
