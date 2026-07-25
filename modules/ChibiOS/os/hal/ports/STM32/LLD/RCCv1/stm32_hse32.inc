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
 * @file    RCCv1/stm32_hse32.inc
 * @brief   Shared HSE32 clock handler.
 *
 * @addtogroup STM32_HSE32_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   HSE32 clock frequency.
 */
#define STM32_HSE32CLK          32000000U

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_RCC_HAS_HSE32)
#error "STM32_RCC_HAS_HSE32 not defined in stm32_registry.h"
#endif

/* Checks on configurations.*/
#if !defined(STM32_HSE32_ENABLED)
#error "STM32_HSE32_ENABLED not defined in mcuconf.h"
#endif

#if defined(STM32_HSECLK)
#error "STM32_HSECLK should not be defined in board.h"
#endif

#if defined(STM32_HSE_ENABLED)
#error "STM32_HSE_ENABLED should not be defined in mcuconf.h"
#endif

/* HSE clock frequency.*/
#if STM32_HSE32_ENABLED
  #if STM32_HSE32PRE == STM32_HSE32PRE_DIV1
    #define STM32_HSECLK                STM32_HSE32CLK
  #elif STM32_HSE32PRE == STM32_HSE32PRE_DIV2
    #define STM32_HSECLK                (STM32_HSE32CLK / 2U)
  #else
    #error "invalid STM32_HSE32PRE value specified"
  #endif
#else
  #define STM32_HSECLK          0U
#endif /* STM32_HSE32_ENABLED */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

__STATIC_INLINE void hse32_enable(void) {

  /* Set HSE32 SYSCLK prescaler.*/
  RCC->CR |= STM32_HSE32PRE;

  /* HSE32 activation.*/
  RCC->CR |= RCC_CR_HSEON;

  while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
    /* Wait until HSE32 is stable.    */
  }
}

__STATIC_INLINE void hse32_init(void) {

#if STM32_HSE32_ENABLED
  hse32_enable();
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
