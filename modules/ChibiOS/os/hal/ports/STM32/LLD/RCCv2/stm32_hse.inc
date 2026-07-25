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
 * @file    RCCv2/stm32_hse.inc
 * @brief   Shared HSE clock handler.
 *
 * @addtogroup STM32_HSE_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_RCC_HAS_HSE)
#error "STM32_RCC_HAS_HSE not defined in stm32_registry.h"
#endif

/* Checks on configurations.*/
#if !defined(STM32_HSE_ENABLED)
#error "STM32_HSE_ENABLED not defined in mcuconf.h"
#endif

#if !defined(STM32_HSECLK)
#error "STM32_HSECLK not defined in board.h"
#endif

/* Check on limits.*/
#if !defined(STM32_HSECLK_MAX)
#error "STM32_HSECLK_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_HSECLK_MIN)
#error "STM32_HSECLK_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_HSECLK_BYP_MAX)
#error "STM32_HSECLK_BYP_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_HSECLK_BYP_MIN)
#error "STM32_HSECLK_BYP_MIN not defined in hal_lld.h"
#endif

#if STM32_HSE_ENABLED
  #if STM32_HSECLK == 0
    #error "HSE frequency not defined"
  #else /* STM32_HSECLK != 0 */
    #if defined(STM32_HSE_BYPASS)
      #if (STM32_HSECLK < STM32_HSECLK_BYP_MIN) || (STM32_HSECLK > STM32_HSECLK_BYP_MAX)
        #error "STM32_HSECLK outside acceptable range (STM32_HSECLK_BYP_MIN...STM32_HSECLK_BYP_MAX)"
      #endif
    #else /* !defined(STM32_HSE_BYPASS) */
      #if (STM32_HSECLK < STM32_HSECLK_MIN) || (STM32_HSECLK > STM32_HSECLK_MAX)
        #error "STM32_HSECLK outside acceptable range (STM32_HSECLK_MIN...STM32_HSECLK_MAX)"
      #endif
    #endif /* !defined(STM32_HSE_BYPASS) */
  #endif /* STM32_HSECLK != 0 */
#endif /* STM32_HSE_ENABLED */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

__STATIC_FORCEINLINE void hse_enable(void) {

#if 0
#if defined(STM32_HSE_BYPASS)
  /* HSE Bypass case.*/
  RCC->CR |= RCC_CR_HSEON | RCC_CR_HSEBYP;
#else
  RCC->CR |= RCC_CR_HSEON;
#endif
  while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
    /* Waiting for HSE activation.*/
  }
#endif

#if defined(STM32_HSE_BYPASS) && defined(STM32_HSE_DIGBYPASS)
  /* HSE digital bypass case.*/
  RCC->OCENSETR = RCC_OCENSETR_HSEBYP | RCC_OCENSETR_DIGBYP | RCC_OCENSETR_HSEON;
#elif defined(STM32_HSE_BYPASS)
  /* HSE analog bypass case.*/
  RCC->OCENSETR = RCC_OCENSETR_HSEBYP | RCC_OCENSETR_HSEON;
#else
  /* HSE no bypass case.*/
  RCC->OCENSETR = RCC_OCENSETR_HSEON;
#endif
  while ((RCC->OCRDYR & RCC_OCRDYR_HSERDY) == 0U) {
    /* Waiting for HSE activation.*/
  }
}

__STATIC_FORCEINLINE void hse_disable(void) {

  RCC->OCENCLRR = RCC_OCENSETR_HSEON;
}

__STATIC_FORCEINLINE void hse_init(void) {

#if STM32_HSE_ENABLED
  hse_enable();
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
