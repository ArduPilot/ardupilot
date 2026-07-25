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
 * @file    RCCv1/stm32_hsi16.inc
 * @brief   Shared HSI16 clock handler.
 *
 * @addtogroup STM32_HSI16_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   HSI16 clock frequency.
 */
#define STM32_HSI16CLK          16000000U

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_RCC_HAS_HSI16)
#error "STM32_RCC_HAS_HSI16 not defined in stm32_registry.h"
#endif

/* Checks on configurations.*/
#if !defined(STM32_HSI16_ENABLED)
#error "STM32_HSI16_ENABLED not defined in mcuconf.h"
#endif

/* Special case where HSI16 is not the default clock.*/
#if !defined(STM32_RCC_HSI16_IS_DEFAULT) || defined(__DOXYGEN__)
#define STM32_RCC_HSI16_IS_DEFAULT  TRUE
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

__STATIC_INLINE void hsi16_enable(void) {

  RCC->CR |= RCC_CR_HSION;
  while ((RCC->CR & RCC_CR_HSIRDY) == 0U) {
    /* Waiting for HSI16 activation.*/
  }
}

__STATIC_INLINE void hsi16_disable(void) {

  RCC->CR &= ~RCC_CR_HSION;
}

#if STM32_RCC_HSI16_IS_DEFAULT == TRUE
__STATIC_INLINE void hsi16_reset(void) {

  /* Making sure HSI is active and ready.*/
  hsi16_enable();

  /* Clocking from HSI, in case HSI was not the default source.*/
  RCC->CFGR = RCC_CFGR_SW_HSI;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {
    /* Wait until HSI is selected.*/
  }
}
#endif

__STATIC_INLINE void hsi16_init(void) {

#if STM32_HSI16_ENABLED
  /* HSI activation.*/
  hsi16_enable();
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
