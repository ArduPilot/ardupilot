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
 * @file    hal_trng.h
 * @brief   TRNG Driver macros and structures.
 *
 * @addtogroup TRNG
 * @{
 */

#ifndef HAL_TRNG_H
#define HAL_TRNG_H

#if (HAL_USE_TRNG == TRUE) || defined(__DOXYGEN__)

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

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  TRNG_UNINIT = 0,                   /**< Not initialized.                   */
  TRNG_STOP = 1,                     /**< Stopped.                           */
  TRNG_READY = 2,                    /**< Ready.                             */
  TRNG_RUNNING = 3                   /**< Generating random number.          */
} trngstate_t;

/**
 * @brief   Type of a structure representing a TRNG driver.
 */
typedef struct hal_trng_driver TRNGDriver;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct hal_trng_config TRNGConfig;

/* Including the low level driver header, it exports information required
   for completing types.*/
#include "hal_trng_lld.h"

/**
 * @brief   Driver configuration structure.
 */
struct hal_trng_config {
  /* End of the mandatory fields.*/
  trng_lld_config_fields
};

/**
 * @brief   Structure representing a TRNG driver.
 */
struct hal_trng_driver {
  /**
   * @brief Driver state.
   */
  trngstate_t                state;
  /**
   * @brief Current configuration data.
   */
  const TRNGConfig           *config;
#if defined(TRNG_DRIVER_EXT_FIELDS)
  TRNG_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  trng_lld_driver_fields
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void trngInit(void);
  void trngObjectInit(TRNGDriver *trngp);
  msg_t trngStart(TRNGDriver *trngp, const TRNGConfig *config);
  void trngStop(TRNGDriver *trngp);
  bool trngGenerate(TRNGDriver *trngp, size_t size, uint8_t *out);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_TRNG == TRUE */

#endif /* HAL_TRNG_H */

/** @} */
