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
 * @file    RCCv1/stm32_hsi48.inc
 * @brief   Shared HSI48 clock handler.
 *
 * @addtogroup STM32_HSI48_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   HSI48 clock frequency.
 */
#define STM32_HSI48CLK          48000000U

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_RCC_HAS_HSI48)
#error "STM32_RCC_HAS_HSI48 not defined in stm32_registry.h"
#endif

/* Checks on configurations.*/
#if !defined(STM32_HSI48_ENABLED)
#error "STM32_HSI48_ENABLED not defined in mcuconf.h"
#endif

#if (STM32_RCC_HAS_HSI48 == FALSE) && (STM32_HSI48_ENABLED == TRUE)
#error "HSI48 not present on this device"
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

#if STM32_RCC_HAS_HSI48 == TRUE
#if defined(RCC_CRRCR_HSI48ON)
__STATIC_INLINE void hsi48_enable(void) {

  RCC->CRRCR |= RCC_CRRCR_HSI48ON;
  while ((RCC->CRRCR & RCC_CRRCR_HSI48RDY) == 0U) {
    /* Waiting for HSI48 activation.*/
  }
}

__STATIC_INLINE void hsi48_disable(void) {

  RCC->CRRCR &= ~RCC_CRRCR_HSI48ON;
}
#endif /* defined(RCC_CRRCR_HSI48ON) */

#if defined(RCC_CR_HSI48ON)
__STATIC_INLINE void hsi48_enable(void) {

  RCC->CR |= RCC_CR_HSI48ON;
  while ((RCC->CR & RCC_CR_HSI48RDY) == 0U) {
    /* Waiting for HSI48 activation.*/
  }
}

__STATIC_INLINE void hsi48_disable(void) {

  RCC->CR &= ~RCC_CR_HSI48ON;
}
#endif /* defined(RCC_CR_HSI48ON) */
#endif /* STM32_RCC_HAS_HSI48 == TRUE */

__STATIC_INLINE void hsi48_init(void) {

#if (defined(RCC_CRRCR_HSI48ON) || defined(RCC_CR_HSI48ON)) && STM32_HSI48_ENABLED
  /* HSI activation.*/
  hsi48_enable();
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
