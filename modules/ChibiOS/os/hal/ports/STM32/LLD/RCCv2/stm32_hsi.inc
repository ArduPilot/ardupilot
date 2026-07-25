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
 * @file    RCCv2/stm32_hsi.inc
 * @brief   Shared HSI clock handler.
 *
 * @addtogroup STM32_HSI_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   HSI64 clock frequency.
 */
#define STM32_HSI64CLK          64000000U

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_RCC_HAS_HSI)
#error "STM32_RCC_HAS_HSI not defined in stm32_registry.h"
#endif

/* Checks on configurations.*/
#if !defined(STM32_HSI_ENABLED)
#error "STM32_HSI_ENABLED not defined in mcuconf.h"
#endif

#if !defined(STM32_HSIDIV)
#error "STM32_HSIDIV not defined in mcuconf.h"
#endif

/**
 * @brief   Final HSI clock.
 */
#if (STM32_HSIDIV == STM32_HSIDIV_DIV1) || defined(__DOXYGEN__)
  #define STM32_HSICLK              (STM32_HSI64CLK / 1U)

#elif STM32_HSIDIV == STM32_HSIDIV_DIV2
  #define STM32_HSICLK              (STM32_HSI64CLK / 2U)

#elif STM32_HSIDIV == STM32_HSIDIV_DIV4
  #define STM32_HSICLK              (STM32_HSI64CLK / 4U)

#elif STM32_HSIDIV == STM32_HSIDIV_DIV8
  #define STM32_HSICLK              (STM32_HSI64CLK / 8U)

#else
  #error "invalid STM32_HSIDIV value specified"
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

__STATIC_FORCEINLINE void hsi_enable(void) {

  RCC->OCENSETR = RCC_OCENSETR_HSION;
  while ((RCC->OCRDYR & RCC_OCRDYR_HSIRDY) == 0U) {
    /* Waiting for HSI activation.*/
  }
}

__STATIC_FORCEINLINE void hsi_disable(void) {

  RCC->OCENCLRR = RCC_OCENCLRR_HSION;
}

__STATIC_FORCEINLINE void hsi_init(void) {

#if STM32_HSI_ENABLED
  /* HSI activation.*/
  hsi_enable();
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
