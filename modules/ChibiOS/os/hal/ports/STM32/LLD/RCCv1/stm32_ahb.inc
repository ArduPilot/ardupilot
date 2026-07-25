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
 * @file    RCCv1/stm32_ahb.inc
 * @brief   Shared AHB clock handler.
 *
 * @addtogroup STM32_AHB_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @name    HPRE field bits definitions (legacy)
 * @{
 */
#if defined(RCC_CFGR_HPRE_Pos)
#define STM32_HPRE_POS                      RCC_CFGR_HPRE_Pos
#define STM32_HPRE_MASK                     RCC_CFGR_HPRE_Msk

#elif defined(RCC_CFGR2_HPRE_Pos)
#define STM32_HPRE_POS                      RCC_CFGR2_HPRE_Pos
#define STM32_HPRE_MASK                     RCC_CFGR2_HPRE_Msk

#else
#error "unknown register name"
#endif

#define STM32_HPRE_FIELD(n)                 ((n) << STM32_HPRE_POS)
#define STM32_HPRE_DIV1                     STM32_HPRE_FIELD(0U)
#define STM32_HPRE_DIV2                     STM32_HPRE_FIELD(8U)
#define STM32_HPRE_DIV4                     STM32_HPRE_FIELD(9U)
#define STM32_HPRE_DIV8                     STM32_HPRE_FIELD(10U)
#define STM32_HPRE_DIV16                    STM32_HPRE_FIELD(11U)
#define STM32_HPRE_DIV64                    STM32_HPRE_FIELD(12U)
#define STM32_HPRE_DIV128                   STM32_HPRE_FIELD(13U)
#define STM32_HPRE_DIV256                   STM32_HPRE_FIELD(14U)
#define STM32_HPRE_DIV512                   STM32_HPRE_FIELD(15U)
/** @} */

/**
 * @name    HPRE field helpers
 * @{
 */
#if defined(RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_FIELD(n)              ((n) << RCC_CFGR_HPRE_Pos)
#if !defined(RCC_CFGR_HPRE_DIV1)
#define RCC_CFGR_HPRE_DIV1                  RCC_CFGR_HPRE_FIELD(0U)
#define RCC_CFGR_HPRE_DIV2                  RCC_CFGR_HPRE_FIELD(8U)
#define RCC_CFGR_HPRE_DIV4                  RCC_CFGR_HPRE_FIELD(9U)
#define RCC_CFGR_HPRE_DIV8                  RCC_CFGR_HPRE_FIELD(10U)
#define RCC_CFGR_HPRE_DIV16                 RCC_CFGR_HPRE_FIELD(11U)
#define RCC_CFGR_HPRE_DIV64                 RCC_CFGR_HPRE_FIELD(12U)
#define RCC_CFGR_HPRE_DIV128                RCC_CFGR_HPRE_FIELD(13U)
#define RCC_CFGR_HPRE_DIV256                RCC_CFGR_HPRE_FIELD(14U)
#define RCC_CFGR_HPRE_DIV512                RCC_CFGR_HPRE_FIELD(15U)
#endif

#elif defined(RCC_CFGR2_HPRE_Pos)
#define RCC_CFGR2_HPRE_FIELD(n)             ((n) << RCC_CFGR2_HPRE_Pos)
#if !defined(RCC_CFGR2_HPRE_DIV1)
#define RCC_CFGR2_HPRE_DIV1                 RCC_CFGR2_HPRE_FIELD(0U)
#define RCC_CFGR2_HPRE_DIV2                 RCC_CFGR2_HPRE_FIELD(8U)
#define RCC_CFGR2_HPRE_DIV4                 RCC_CFGR2_HPRE_FIELD(9U)
#define RCC_CFGR2_HPRE_DIV8                 RCC_CFGR2_HPRE_FIELD(10U)
#define RCC_CFGR2_HPRE_DIV16                RCC_CFGR2_HPRE_FIELD(11U)
#define RCC_CFGR2_HPRE_DIV64                RCC_CFGR2_HPRE_FIELD(12U)
#define RCC_CFGR2_HPRE_DIV128               RCC_CFGR2_HPRE_FIELD(13U)
#define RCC_CFGR2_HPRE_DIV256               RCC_CFGR2_HPRE_FIELD(14U)
#define RCC_CFGR2_HPRE_DIV512               RCC_CFGR2_HPRE_FIELD(15U)
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
#if !defined(STM32_HPRE)
#error "STM32_HPRE not defined in mcuconf.h"
#endif

/* Input checks.*/
#if !defined(STM32_SYSCLK_MAX)
#error "STM32_SYSCLK_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_SYSCLK)
#error "STM32_SYSCLK not defined in hal_lld.h"
#endif

/**
 * @brief   AHB frequency.
 */
#if (STM32_HPRE == STM32_HPRE_DIV1) || defined(__DOXYGEN__)
#define STM32_HCLK                  (STM32_SYSCLK / 1U)

#elif STM32_HPRE == STM32_HPRE_DIV2
#define STM32_HCLK                  (STM32_SYSCLK / 2U)

#elif STM32_HPRE == STM32_HPRE_DIV4
#define STM32_HCLK                  (STM32_SYSCLK / 4U)

#elif STM32_HPRE == STM32_HPRE_DIV8
#define STM32_HCLK                  (STM32_SYSCLK / 8U)

#elif STM32_HPRE == STM32_HPRE_DIV16
#define STM32_HCLK                  (STM32_SYSCLK / 16U)

#elif STM32_HPRE == STM32_HPRE_DIV64
#define STM32_HCLK                  (STM32_SYSCLK / 64U)

#elif STM32_HPRE == STM32_HPRE_DIV128
#define STM32_HCLK                  (STM32_SYSCLK / 128U)

#elif STM32_HPRE == STM32_HPRE_DIV256
#define STM32_HCLK                  (STM32_SYSCLK / 256U)

#elif STM32_HPRE == STM32_HPRE_DIV512
#define STM32_HCLK                  (STM32_SYSCLK / 512U)

#else
#error "invalid STM32_HPRE value specified"
#endif

/*
 * SYSCLK frequency check.
 */
#if STM32_SYSCLK > STM32_SYSCLK_MAX
#error "STM32_SYSCLK above maximum rated frequency (STM32_SYSCLK_MAX)"
#endif

/*
 * HCLK frequency check.
 */
#if STM32_HCLK > STM32_SYSCLK_MAX
#error "STM32_HCLK exceeding maximum frequency (STM32_SYSCLK_MAX)"
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
