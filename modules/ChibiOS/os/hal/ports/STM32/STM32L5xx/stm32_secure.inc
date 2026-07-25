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
 * @file    STM32L5xx/stm32_secure.inc
 * @brief   Secure mode handler.
 *
 * @addtogroup STM32_SECURE_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @name    FLASH_SECWMn registers bits definitions
 * @{
 */
#define STM32_FLASH_SECWM_START(n)      ((n) << 0)
#define STM32_FLASH_SECWM_END(n)        ((n) << 16)
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_SECURE_MODE)
#define STM32_SECURE_MODE               FALSE
#endif

#if STM32_SECURE_MODE

/* Configuration data inclusion.*/
#include "secconf.h"

/* Checks on configurations.*/
#if !defined(STM32_FLASH_SECBB1R1)
#error "STM32_FLASH_SECBB1R1 not defined in secconf.h"
#endif

#if !defined(STM32_FLASH_SECBB1R2)
#error "STM32_FLASH_SECBB1R2 not defined in secconf.h"
#endif

#if !defined(STM32_FLASH_SECBB1R3)
#error "STM32_FLASH_SECBB1R3 not defined in secconf.h"
#endif

#if !defined(STM32_FLASH_SECBB1R4)
#error "STM32_FLASH_SECBB1R4 not defined in secconf.h"
#endif

#if !defined(STM32_FLASH_SECBB2R1)
#error "STM32_FLASH_SECBB2R1 not defined in secconf.h"
#endif

#if !defined(STM32_FLASH_SECBB2R2)
#error "STM32_FLASH_SECBB2R2 not defined in secconf.h"
#endif

#if !defined(STM32_FLASH_SECBB2R3)
#error "STM32_FLASH_SECBB2R3 not defined in secconf.h"
#endif

#if !defined(STM32_FLASH_SECBB2R4)
#error "STM32_FLASH_SECBB2R4 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_CR)
#error "STM32_MPCBB1_CR not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_LCKVTR1)
#error "STM32_MPCBB1_LCKVTR1 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR0)
#error "STM32_MPCBB1_VCTR0 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR1)
#error "STM32_MPCBB1_VCTR1 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR2)
#error "STM32_MPCBB1_VCTR2 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR3)
#error "STM32_MPCBB1_VCTR3 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR4)
#error "STM32_MPCBB1_VCTR4 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR5)
#error "STM32_MPCBB1_VCTR5 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR6)
#error "STM32_MPCBB1_VCTR6 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR7)
#error "STM32_MPCBB1_VCTR7 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR8)
#error "STM32_MPCBB1_VCTR8 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR9)
#error "STM32_MPCBB1_VCTR9 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR10)
#error "STM32_MPCBB1_VCTR10 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR11)
#error "STM32_MPCBB1_VCTR11 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR12)
#error "STM32_MPCBB1_VCTR12 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR13)
#error "STM32_MPCBB1_VCTR13 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR14)
#error "STM32_MPCBB1_VCTR14 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR15)
#error "STM32_MPCBB1_VCTR15 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR16)
#error "STM32_MPCBB1_VCTR16 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR17)
#error "STM32_MPCBB1_VCTR17 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR18)
#error "STM32_MPCBB1_VCTR18 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR19)
#error "STM32_MPCBB1_VCTR19 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR20)
#error "STM32_MPCBB1_VCTR20 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR21)
#error "STM32_MPCBB1_VCTR21 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR22)
#error "STM32_MPCBB1_VCTR22 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB1_VCTR23)
#error "STM32_MPCBB1_VCTR23 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB2_CR)
#error "STM32_MPCBB2_CR not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB2_LCKVTR1)
#error "STM32_MPCBB2_LCKVTR1 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB2_VCTR0)
#error "STM32_MPCBB2_VCTR0 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB2_VCTR1)
#error "STM32_MPCBB2_VCTR1 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB2_VCTR2)
#error "STM32_MPCBB2_VCTR2 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB2_VCTR3)
#error "STM32_MPCBB2_VCTR3 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB2_VCTR4)
#error "STM32_MPCBB2_VCTR4 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB2_VCTR5)
#error "STM32_MPCBB2_VCTR5 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB2_VCTR6)
#error "STM32_MPCBB2_VCTR6 not defined in secconf.h"
#endif

#if !defined(STM32_MPCBB2_VCTR7)
#error "STM32_MPCBB2_VCTR7 not defined in secconf.h"
#endif

/* Check on limits.*/

#endif /* STM32_SECURE_MODE */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void secure_init(void) {

#if STM32_SECURE_MODE
  rccEnableGTZC(false);

  FLASH->SECBB1R1 = STM32_FLASH_SECBB1R1;
  FLASH->SECBB1R2 = STM32_FLASH_SECBB1R2;
  FLASH->SECBB1R3 = STM32_FLASH_SECBB1R3;
  FLASH->SECBB1R4 = STM32_FLASH_SECBB1R4;
  FLASH->SECBB2R1 = STM32_FLASH_SECBB2R1;
  FLASH->SECBB2R2 = STM32_FLASH_SECBB2R2;
  FLASH->SECBB2R3 = STM32_FLASH_SECBB2R3;
  FLASH->SECBB2R4 = STM32_FLASH_SECBB2R4;

  GTZC_MPCBB1->VCTR[0] = STM32_MPCBB1_VCTR0;
  GTZC_MPCBB1->VCTR[1] = STM32_MPCBB1_VCTR1;
  GTZC_MPCBB1->VCTR[2] = STM32_MPCBB1_VCTR2;
  GTZC_MPCBB1->VCTR[3] = STM32_MPCBB1_VCTR3;
  GTZC_MPCBB1->VCTR[4] = STM32_MPCBB1_VCTR4;
  GTZC_MPCBB1->VCTR[5] = STM32_MPCBB1_VCTR5;
  GTZC_MPCBB1->VCTR[6] = STM32_MPCBB1_VCTR6;
  GTZC_MPCBB1->VCTR[7] = STM32_MPCBB1_VCTR7;
  GTZC_MPCBB1->VCTR[8] = STM32_MPCBB1_VCTR8;
  GTZC_MPCBB1->VCTR[9] = STM32_MPCBB1_VCTR9;
  GTZC_MPCBB1->VCTR[10] = STM32_MPCBB1_VCTR10;
  GTZC_MPCBB1->VCTR[11] = STM32_MPCBB1_VCTR11;
  GTZC_MPCBB1->VCTR[12] = STM32_MPCBB1_VCTR12;
  GTZC_MPCBB1->VCTR[13] = STM32_MPCBB1_VCTR13;
  GTZC_MPCBB1->VCTR[14] = STM32_MPCBB1_VCTR14;
  GTZC_MPCBB1->VCTR[15] = STM32_MPCBB1_VCTR15;
  GTZC_MPCBB1->VCTR[16] = STM32_MPCBB1_VCTR16;
  GTZC_MPCBB1->VCTR[17] = STM32_MPCBB1_VCTR17;
  GTZC_MPCBB1->VCTR[18] = STM32_MPCBB1_VCTR18;
  GTZC_MPCBB1->VCTR[19] = STM32_MPCBB1_VCTR19;
  GTZC_MPCBB1->VCTR[20] = STM32_MPCBB1_VCTR20;
  GTZC_MPCBB1->VCTR[21] = STM32_MPCBB1_VCTR21;
  GTZC_MPCBB1->VCTR[22] = STM32_MPCBB1_VCTR22;
  GTZC_MPCBB1->VCTR[23] = STM32_MPCBB1_VCTR23;
  GTZC_MPCBB1->LCKVTR1  = STM32_MPCBB1_LCKVTR1;
  GTZC_MPCBB1->CR       = STM32_MPCBB1_CR;

  GTZC_MPCBB2->VCTR[0]  = STM32_MPCBB2_VCTR0;
  GTZC_MPCBB2->VCTR[1]  = STM32_MPCBB2_VCTR1;
  GTZC_MPCBB2->VCTR[2]  = STM32_MPCBB2_VCTR2;
  GTZC_MPCBB2->VCTR[3]  = STM32_MPCBB2_VCTR3;
  GTZC_MPCBB2->VCTR[4]  = STM32_MPCBB2_VCTR4;
  GTZC_MPCBB2->VCTR[5]  = STM32_MPCBB2_VCTR5;
  GTZC_MPCBB2->VCTR[6]  = STM32_MPCBB2_VCTR6;
  GTZC_MPCBB2->VCTR[7]  = STM32_MPCBB2_VCTR7;
  GTZC_MPCBB2->LCKVTR1  = STM32_MPCBB2_LCKVTR1;
  GTZC_MPCBB2->CR       = STM32_MPCBB2_CR;
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
