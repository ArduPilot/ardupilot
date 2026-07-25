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
 * @file    ICACHEv1/stm32_icache.inc
 * @brief   Shared ICACHE handler.
 *
 * @addtogroup STM32_ICACHE_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_HAS_ICACHE)
#define STM32_HAS_ICACHE                FALSE
#endif

#if !defined(STM32_ICACHE_HAS_REGIONS)
#define STM32_ICACHE_HAS_REGIONS        TRUE
#endif

#if STM32_HAS_ICACHE

/* Checks on configurations.*/
#if !defined(STM32_ICACHE_CR)
#error "STM32_ICACHE_CR not defined in mcuconf.h"
#endif

#if STM32_ICACHE_HAS_REGIONS
#if !defined(STM32_ICACHE_CRR0)
#error "STM32_ICACHE_CRR0 not defined in mcuconf.h"
#endif

#if !defined(STM32_ICACHE_CRR1)
#error "STM32_ICACHE_CRR1 not defined in mcuconf.h"
#endif

#if !defined(STM32_ICACHE_CRR2)
#error "STM32_ICACHE_CRR2 not defined in mcuconf.h"
#endif

#if !defined(STM32_ICACHE_CRR3)
#error "STM32_ICACHE_CRR3 not defined in mcuconf.h"
#endif
#endif

/* Check on limits.*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void icache_init(void) {

#if (STM32_ICACHE_CR & ICACHE_CR_EN) != 0
  /* Waiting for the invalidation operation to finish, if any.*/
  while ((ICACHE->SR & ICACHE_SR_BUSYF) != 0U) {
  }
  ICACHE->CR   = STM32_ICACHE_CR;
#if STM32_ICACHE_HAS_REGIONS
  ICACHE->CRR0 = STM32_ICACHE_CRR0;
  ICACHE->CRR1 = STM32_ICACHE_CRR1;
  ICACHE->CRR2 = STM32_ICACHE_CRR2;
  ICACHE->CRR3 = STM32_ICACHE_CRR3;
#endif
#endif
}

static inline void icache_deinit(void) {

#if (STM32_ICACHE_CR & ICACHE_CR_EN) != 0
  /* Waiting for the invalidation operation to finish, if any.*/
  while ((ICACHE->SR & ICACHE_SR_BUSYF) != 0U) {
  }
  ICACHE->CR   = 0U;
#if STM32_ICACHE_HAS_REGIONS
  ICACHE->CRR0 = 0U;
  ICACHE->CRR1 = 0U;
  ICACHE->CRR2 = 0U;
  ICACHE->CRR3 = 0U;
#endif
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

#endif /* STM32_HAS_ICACHE */

/** @} */
