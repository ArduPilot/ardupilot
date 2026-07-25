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
 * @file    RCCv1/stm32_lsi12.inc
 * @brief   Shared LSI12 clock handler.
 *
 * @addtogroup STM32_LSI12_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   LSI clock frequency.
 */
#define STM32_LSICLK                        32000U

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_RCC_HAS_LSI1)
#error "STM32_RCC_HAS_LSI1 not defined in stm32_registry.h"
#endif

#if !defined(STM32_RCC_HAS_LSI2)
#error "STM32_RCC_HAS_LSI2 not defined in stm32_registry.h"
#elif !defined(STM32_RCC_LSI2_TRIM_ADDR)
#error "STM32_RCC_LSI2_TRIM_ADDR not defined in stm32_registry.h"
#endif

/* Checks on configurations.*/
#if !defined(STM32_LSI1_ENABLED)
#error "STM32_LSI1_ENABLED not defined in mcuconf.h"
#endif

#if !defined(STM32_LSI2_ENABLED)
#error "STM32_LSI2_ENABLED not defined in mcuconf.h"
#endif

#if defined(STM32_LSI_ENABLED)
#error "STM32_LSI_ENABLED should not be defined in mcuconf.h"
#endif

/* Define STM32_LSI_ENABLED for compatibility with other drivers (xWDG) */
#if STM32_LSI1_ENABLED || STM32_LSI2_ENABLED
#define STM32_LSI_ENABLED                   TRUE
#else
#define STM32_LSI_ENABLED                   FALSE
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

__STATIC_INLINE void lsi_init(void) {

#if STM32_LSI1_ENABLED
  /* LSI1 activation.*/
  RCC->CSR |= RCC_CSR_LSI1ON;
  while ((RCC->CSR & RCC_CSR_LSI1RDY) == 0U) {
  }
#endif
#if STM32_LSI2_ENABLED
  /* Set LSI2 trimming.*/
  uint32_t trim = ((*(uint32_t *)(STM32_RCC_LSI2_TRIM_ADDR)) & 0xFUL);
  RCC->CSR |= (trim << RCC_CSR_LSI2TRIM_Pos);
  /* LSI2 activation.*/
  RCC->CSR |= RCC_CSR_LSI2ON;
  while ((RCC->CSR & RCC_CSR_LSI2RDY) == 0U) {
  }
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
