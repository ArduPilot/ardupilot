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
 * @file    RCCv1/stm32_lsi.inc
 * @brief   Shared LSI clock handler.
 *
 * @addtogroup STM32_LSI_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   LSI clock frequency.
 */
#define STM32_LSIRCCLK          32000U

/**
 * @name    RCC_CSR register bits definitions
 * @{
 */
#define STM32_LSIPRE_MASK       (1U << RCC_CSR_LSIPRE_Pos)
#define STM32_LSIPRE_NODIV      (0U << RCC_CSR_LSIPRE_Pos)
#define STM32_LSIPRE_DIV128     (1U << RCC_CSR_LSIPRE_Pos)
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_RCC_HAS_LSI)
#error "STM32_RCC_HAS_LSI not defined in stm32_registry.h"
#endif

#if !defined(STM32_RCC_HAS_LSI_PRESCALER)
#error "STM32_RCC_HAS_LSI_PRESCALER not defined in stm32_registry.h"
#endif

/* Checks on configurations.*/
#if !defined(STM32_LSI_ENABLED)
#error "STM32_LSI_ENABLED not defined in mcuconf.h"
#endif

#if STM32_RCC_HAS_LSI_PRESCALER || defined(__DOXYGEN__)

#if !defined(STM32_LSIPRE)
#error "STM32_LSIPRE not defined in mcuconf.h"
#endif

/**
 * @brief   LSI frequency.
 */
#if (STM32_LSIPRE == STM32_LSIPRE_NODIV) || defined(__DOXYGEN__)
#define STM32_LSICLK                (STM32_LSIRCCLK)
#elif STM32_LSIPRE == STM32_LSIPRE_DIV128
#define STM32_LSICLK                (STM32_LSIRCCLK / 128U)
#else
#error "invalid STM32_LSIPRE value specified"
#endif

#else /* !STM32_RCC_HAS_LSI_PRESCALER */
#define STM32_LSIPRE                0U
#define STM32_LSICLK                (STM32_LSIRCCLK)
#endif /* !STM32_RCC_HAS_LSI_PRESCALER */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

__STATIC_INLINE void lsi_init(void) {

#if STM32_LSI_ENABLED
#if defined(RCC_CSR_LSION)
  /* LSI activation.*/
  RCC->CSR |= STM32_LSIPRE | RCC_CSR_LSION;
  while ((RCC->CSR & RCC_CSR_LSIRDY) == 0U) {
  }
#elif defined(RCC_BDCR_LSION)
  /* LSI activation.*/
  RCC->BDCR |= STM32_LSIPRE | RCC_BDCR_LSION;
  while ((RCC->BDCR & RCC_BDCR_LSIRDY) == 0U) {
  }
#else
#error "unknown register name"
#endif
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
