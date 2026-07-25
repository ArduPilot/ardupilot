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
 * @file    hal_trng_lld.h
 * @brief   STM32 TRNG subsystem low level driver header.
 *
 * @addtogroup TRNG
 * @{
 */

#ifndef HAL_TRNG_LLD_H
#define HAL_TRNG_LLD_H

#if (HAL_USE_TRNG == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    STM32 configuration options
 * @{
 */
/**
 * @brief   TRNGD1 driver enable switch.
 * @details If set to @p TRUE the support for TRNGD1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_TRNG_USE_RNG1) || defined(__DOXYGEN__)
#define STM32_TRNG_USE_RNG1                 FALSE
#endif

/**
 * @brief   TRNGD1 error clear timeout counter.
 * @details Number of status register fetches before failing.
 */
#if !defined(STM32_TRNG_ERROR_CLEAR_ATTEMPTS) || defined(__DOXYGEN__)
#define STM32_TRNG_ERROR_CLEAR_ATTEMPTS     1000
#endif

/**
 * @brief   TRNGD1 data available timeout counter.
 * @details Number of status register fetches before failing.
 */
#if !defined(STM32_DATA_FETCH_ATTEMPTS) || defined(__DOXYGEN__)
#define STM32_DATA_FETCH_ATTEMPTS           1000
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !defined(STM32_HAS_RNG1)
#define STM32_HAS_RNG1                      FALSE
#endif

#if STM32_TRNG_USE_RNG1 && !STM32_HAS_RNG1
#error "RNG1 not present in the selected device"
#endif

#if !STM32_TRNG_USE_RNG1
#error "TRNG driver activated but no RNG peripheral assigned"
#endif

#if !defined(STM32_RNGCLK)
#error "STM32_RNGCLK not defined in this HAL"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the TRNG configuration structure.
 */
#define trng_lld_config_fields                                              \
  /* CR register initialization value.*/                                    \
  uint32_t                   cr;

/**
 * @brief   Low level fields of the TRNG driver structure.
 */
#define trng_lld_driver_fields                                              \
  /* Pointer to the RNG registers block.*/                                  \
  RNG_TypeDef                *rng;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (STM32_TRNG_USE_RNG1 == TRUE) && !defined(__DOXYGEN__)
extern TRNGDriver TRNGD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void trng_lld_init(void);
  void trng_lld_start(TRNGDriver *trngp);
  void trng_lld_stop(TRNGDriver *trngp);
  bool trng_lld_generate(TRNGDriver *trngp, size_t size, uint8_t *out);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_TRNG == TRUE */

#endif /* HAL_TRNG_LLD_H */

/** @} */
