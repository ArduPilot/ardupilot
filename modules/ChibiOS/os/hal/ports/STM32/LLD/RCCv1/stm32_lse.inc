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
 * @file    RCCv1/stm32_lse.inc
 * @brief   Shared LSE clock handler.
 *
 * @addtogroup STM32_LSE_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_RCC_HAS_LSE)
#error "STM32_RCC_HAS_LSE not defined in stm32_registry.h"
#endif

/* Checks on configurations.*/
#if !defined(STM32_LSE_ENABLED)
#error "STM32_LSE_ENABLED not defined in mcuconf.h"
#endif

#if !defined(STM32_LSECLK)
#error "STM32_LSECLK not defined in board.h"
#endif

#if !defined(STM32_LSEDRV)
#error "STM32_LSEDRV not defined in board.h"
#endif

/* Check on limits.*/
#if !defined(STM32_LSECLK_MAX)
#error "STM32_LSECLK_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_LSECLK_MIN)
#error "STM32_LSECLK_MIN not defined in hal_lld.h"
#endif

#if STM32_LSE_ENABLED
  #if (STM32_LSECLK == 0)
    #error "LSE frequency not defined"
  #endif

  #if (STM32_LSECLK < STM32_LSECLK_MIN) || (STM32_LSECLK > STM32_LSECLK_MAX)
    #error "STM32_LSECLK outside acceptable range (STM32_LSECLK_MIN...STM32_LSECLK_MAX)"
  #endif
#endif

#if !defined(RCC_BDCR_LSESYSEN)
#define RCC_BDCR_LSESYSEN               0U
#endif

#if !defined(RCC_BDCR_LSESYSRDY)
#define RCC_BDCR_LSESYSRDY              0U
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

__STATIC_INLINE void lse_init(void) {

#if STM32_LSE_ENABLED
  /* LSE activation.*/
#if defined(STM32_LSE_BYPASS)
  /* LSE Bypass.*/
  RCC->BDCR |= STM32_LSEDRV | RCC_BDCR_LSEON | RCC_BDCR_LSESYSEN | RCC_BDCR_LSEBYP;
#else
  /* No LSE Bypass.*/
  RCC->BDCR |= STM32_LSEDRV | RCC_BDCR_LSEON | RCC_BDCR_LSESYSEN;
  while ((RCC->BDCR & (RCC_BDCR_LSESYSRDY | RCC_BDCR_LSERDY)) !=
         (RCC_BDCR_LSESYSRDY | RCC_BDCR_LSERDY)) {
  }
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
