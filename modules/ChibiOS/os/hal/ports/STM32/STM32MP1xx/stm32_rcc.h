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
 * @file    STM32MP1xx/stm32_rcc.h
 * @brief   RCC helper driver header.
 * @note    This file requires definitions from the ST header file
 *          @p stm32mp1xx.h.
 *
 * @addtogroup STM32MP1xx_RCC
 * @{
 */
#ifndef STM32_RCC_H
#define STM32_RCC_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Generic RCC operations
 * @{
 */
/**
 * @brief   Enables the clock of one or more peripheral on the APB1 bus.
 *
 * @param[in] mask      APB1 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
__STATIC_FORCEINLINE void rccEnableAPB1(uint32_t mask, bool lp) {

  RCC->MC_APB1ENSETR = mask;
  if (lp) {
    RCC->MC_APB1LPENSETR = mask;
  }
  else {
    RCC->MC_APB1LPENCLRR = mask;
  }
  (void) RCC->MC_APB1ENSETR;
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB1 bus).
 *
 * @param[in] mask      APB1 peripherals mask
 *
 * @api
 */
__STATIC_FORCEINLINE void rccDisableAPB1(uint32_t mask) {

  RCC->MC_APB1ENCLRR = mask;
  RCC->MC_APB1LPENCLRR = mask;
  (void) RCC->MC_APB1LPENCLRR;
}

/**
 * @brief   Resets one or more peripheral on the APB1 bus.
 *
 * @param[in] mask      APB1 peripherals mask
 *
 * @api
 */
__STATIC_FORCEINLINE void rccResetAPB1(uint32_t mask) {

  RCC->APB1RSTSETR = mask;
  RCC->APB1RSTCLRR = mask;
  (void) RCC->APB1RSTCLRR;
}

/**
 * @brief   Enables the clock of one or more peripheral on the APB3 bus.
 *
 * @param[in] mask      APB3 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
__STATIC_FORCEINLINE void rccEnableAPB3(uint32_t mask, bool lp) {

  RCC->MC_APB3ENSETR = mask;
  if (lp) {
    RCC->MC_APB3LPENSETR = mask;
  }
  else {
    RCC->MC_APB3LPENCLRR = mask;
  }
  (void) RCC->MC_APB3ENSETR;
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB3 bus).
 *
 * @param[in] mask      APB3 peripherals mask
 *
 * @api
 */
__STATIC_FORCEINLINE void rccDisableAPB3(uint32_t mask) {

  RCC->MC_APB3ENCLRR = mask;
  RCC->MC_APB3LPENCLRR = mask;
  (void) RCC->MC_APB3LPENCLRR;
}

/**
 * @brief   Resets one or more peripheral on the APB3 bus.
 *
 * @param[in] mask      APB3 peripherals mask
 *
 * @api
 */
__STATIC_FORCEINLINE void rccResetAPB3(uint32_t mask) {

  RCC->APB3RSTSETR = mask;
  RCC->APB3RSTCLRR = mask;
  (void) RCC->APB3RSTCLRR;
}

/**
 * @brief   Enables the clock of one or more peripheral on the AHB4 bus.
 *
 * @param[in] mask      AHB4 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
__STATIC_FORCEINLINE void rccEnableAHB4(uint32_t mask, bool lp) {

  RCC->MC_AHB4ENSETR = mask;
  if (lp) {
    RCC->MC_AHB4LPENSETR = mask;
  }
  else {
    RCC->MC_AHB4LPENCLRR = mask;
  }
  (void) RCC->MC_AHB4ENSETR;
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB4 bus).
 *
 * @param[in] mask      AHB4 peripherals mask
 *
 * @api
 */
__STATIC_FORCEINLINE void rccDisableAHB4(uint32_t mask) {

  RCC->MC_AHB4ENCLRR = mask;
  RCC->MC_AHB4LPENCLRR = mask;
  (void) RCC->MC_AHB4LPENCLRR;
}

/**
 * @brief   Resets one or more peripheral on the AHB4 bus.
 *
 * @param[in] mask      AHB4 peripherals mask
 *
 * @api
 */
__STATIC_FORCEINLINE void rccResetAHB4(uint32_t mask) {

  RCC->AHB4RSTSETR = mask;
  RCC->AHB4RSTCLRR = mask;
  (void) RCC->AHB4RSTCLRR;
}
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

#endif /* STM32_RCC_H */

/** @} */
