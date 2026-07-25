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
 * @file    RCCv2/stm32_lsi.inc
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
#define STM32_LSICLK            32000U

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_RCC_HAS_LSI)
#error "STM32_RCC_HAS_LSI not defined in stm32_registry.h"
#endif

/* Checks on configurations.*/
#if !defined(STM32_LSI_ENABLED)
#error "STM32_LSI_ENABLED not defined in mcuconf.h"
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

__STATIC_FORCEINLINE void lsi_init(void) {

#if STM32_LSI_ENABLED
  /* LSI activation.*/
  RCC->RDLSICR |= RCC_RDLSICR_LSION;
  while ((RCC->RDLSICR & RCC_RDLSICR_LSIRDY) == 0U) {
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
