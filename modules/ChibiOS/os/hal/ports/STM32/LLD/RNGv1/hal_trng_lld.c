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
 * @file    hal_trng_lld.c
 * @brief   STM32 TRNG subsystem low level driver source.
 *
 * @addtogroup TRNG
 * @{
 */

#include "hal.h"

#if (HAL_USE_TRNG == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   TRNGD1 driver identifier.
 */
#if (STM32_TRNG_USE_RNG1 == TRUE) || defined(__DOXYGEN__)
TRNGDriver TRNGD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static const TRNGConfig default_cfg = {.cr = 0};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level TRNG driver initialization.
 *
 * @notapi
 */
void trng_lld_init(void) {

#if STM32_TRNG_USE_RNG1 == TRUE
  /* Driver initialization.*/
  trngObjectInit(&TRNGD1);
  TRNGD1.rng = RNG;
#endif
}

/**
 * @brief   Configures and activates the TRNG peripheral.
 *
 * @param[in] trngp      pointer to the @p TRNGDriver object
 *
 * @notapi
 */
void trng_lld_start(TRNGDriver *trngp) {

#if !defined(STM32_DISABLE_RNG_CLOCK_CHECK)
  osalDbgAssert(((STM32_RNGCLK >= 47000000) && (STM32_RNGCLK <= 49000000)) ||
                ((STM32_RNGCLK >= 3500000)  && (STM32_RNGCLK <= 4500000)),
                "invalid RNG frequency");
#endif

  /* There is no real configuration but setting up a valid pointer anyway.*/
  if (trngp->config == NULL) {
    trngp->config = &default_cfg;
  }

  if (trngp->state == TRNG_STOP) {
    /* Enables the peripheral.*/
#if STM32_TRNG_USE_RNG1 == TRUE
    if (&TRNGD1 == trngp) {
      rccEnableRNG(false);
    }
#endif
  }
  /* Configures the peripheral.*/
  trngp->rng->CR |= RNG_CR_RNGEN;
}

/**
 * @brief   Deactivates the TRNG peripheral.
 *
 * @param[in] trngp      pointer to the @p TRNGDriver object
 *
 * @notapi
 */
void trng_lld_stop(TRNGDriver *trngp) {

  if (trngp->state == TRNG_READY) {
    /* Resets the peripheral.*/
    trngp->rng->CR &= ~RNG_CR_RNGEN;

    /* Disables the peripheral.*/
#if STM32_TRNG_USE_RNG1 == TRUE
    if (&TRNGD1 == trngp) {
      rccDisableRNG();
    }
#endif
  }
}

/**
 * @brief   True random numbers generator.
 * @note    The function is blocking and likely performs polled waiting
 *          inside the low level implementation.
 *
 * @param[in] trngp             pointer to the @p TRNGDriver object
 * @param[in] size              size of output buffer
 * @param[out] out              output buffer
 * @return                      The operation status.
 * @retval false                if a random number has been generated.
 * @retval true                 if an HW error occurred.
 *
 * @api
 */
bool trng_lld_generate(TRNGDriver *trngp, size_t size, uint8_t *out) {

  while (true) {
    uint32_t r, tmo;
    size_t i;

    /* Waiting for error conditions to be cleared.*/
    tmo = STM32_TRNG_ERROR_CLEAR_ATTEMPTS;
    while ((tmo > 0) && ((trngp->rng->SR & (RNG_SR_CECS | RNG_SR_SECS)) != 0)) {
      tmo--;
      if (tmo == 0) {
        return true;
      }
    }

    /* Waiting for a random number in data register.*/
    tmo = STM32_DATA_FETCH_ATTEMPTS;
    while ((tmo > 0) && ((trngp->rng->SR & RNG_SR_DRDY) == 0)) {
      tmo--;
      if (tmo == 0) {
        return true;
      }
    }

    /* Getting the generated random number.*/
    r = trngp->rng->DR;

    /* Writing in the output buffer.*/
    for (i = 0; i < sizeof (uint32_t) / sizeof (uint8_t); i++) {
      *out++ = (uint8_t)r;
      r = r >> 8;
      size--;
      if (size == 0) {
        return false;
      }
    }
  }
}

#endif /* HAL_USE_TRNG == TRUE */

/** @} */
