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
 * @file    RCCv1/stm32_hsidiv.inc
 * @brief   Shared HSI with divider clock handler.
 *
 * @addtogroup STM32_HSIDIV_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   HSIDIV base clock frequency.
 */
#define STM32_HSI64CLK              64000000U

/**
 * @brief   HSI clock frequency after reset.
 */
#define STM32_HSICLK_RESET          32000000U

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

#if !defined(STM32_HSIDIV_VALUE)
#error "STM32_HSIDIV_VALUE not defined in mcuconf.h"
#endif

#if (STM32_HSIDIV_VALUE == 1) || defined(__DOXYGEN__)
#define STM32_HSIDIV                (0U << RCC_CR_HSIDIV_Pos)
#elif STM32_HSIDIV_VALUE == 2
#define STM32_HSIDIV                (1U << RCC_CR_HSIDIV_Pos)
#elif STM32_HSIDIV_VALUE == 4
#define STM32_HSIDIV                (2U << RCC_CR_HSIDIV_Pos)
#elif STM32_HSIDIV_VALUE == 8
#define STM32_HSIDIV                (3U << RCC_CR_HSIDIV_Pos)
#else
#error "invalid STM32_HSIDIV_VALUE value"
#endif

/**
 * @brief   Divided HSI clock.
 */
#if STM32_HSI_ENABLED || defined(__DOXYGEN__)
#define STM32_HSICLK                (STM32_HSI64CLK / STM32_HSIDIV_VALUE)
#else
#define STM32_HSICLK                0U
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

__STATIC_INLINE void hsi_enable(void) {

  RCC->CR |= RCC_CR_HSION;
  while ((RCC->CR & RCC_CR_HSIRDY) == 0U) {
    /* Waiting for HSI activation.*/
  }

  /* Setting up the divider.*/
  RCC->CR = (RCC->CR & ~RCC_CR_HSIDIV_Msk) | STM32_HSIDIV;
  while ((RCC->CR & RCC_CR_HSIDIVF) == 0U) {
    /* Waiting for HSIDIV activation.*/
  }
}

__STATIC_INLINE void hsi_disable(void) {

  RCC->CR &= ~RCC_CR_HSION;
}

__STATIC_INLINE void hsi_reset(void) {

  /* Making sure HSI is active and ready.*/
  hsi_enable();

  /* Clocking from HSI, in case HSI was not the default source.*/
  RCC->CFGR1 = RCC_CFGR1_SW_HSI;
  while ((RCC->CFGR1 & RCC_CFGR1_SWS_Msk) != RCC_CFGR1_SWS_HSI) {
    /* Wait until HSI is selected.*/
  }
}

__STATIC_INLINE void hsi_init(void) {

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
