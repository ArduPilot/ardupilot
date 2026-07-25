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
 * @brief   PLATFORM TRNG subsystem low level driver source.
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
#if (PLATFORM_TRNG_USE_TRNG1 == TRUE) || defined(__DOXYGEN__)
TRNGDriver TRNGD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

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

#if PLATFORM_TRNG_USE_TRNG1 == TRUE
  /* Driver initialization.*/
  trngObjectInit(&TRNGD1);
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

  if (trngp->state == TRNG_STOP) {
    /* Enables the peripheral.*/
#if PLATFORM_TRNG_USE_TRNG1 == TRUE
    if (&TRNGD1 == trngp) {

    }
#endif
  }
  /* Configures the peripheral.*/

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

    /* Disables the peripheral.*/
#if PLATFORM_TRNG_USE_TRNG1 == TRUE
    if (&TRNGD1 == trngp) {

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

  (void)trngp;
  (void)size;
  (void)out;

  return true;
}

#endif /* HAL_USE_TRNG == TRUE */

/** @} */
