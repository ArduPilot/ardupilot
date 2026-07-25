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
 * @file    hal_efl.h
 * @brief   Embedded Flash Driver macros and structures.
 *
 * @addtogroup HAL_EFL
 * @{
 */

#ifndef HAL_EFL_H
#define HAL_EFL_H

#if (HAL_USE_EFL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Enables the @p flashAcquireExclusive() and
 *          @p flashReleaseExclusive() APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(EFL_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define EFL_USE_MUTUAL_EXCLUSION            TRUE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @extends BaseFlash
 *
 * @brief   Type of embedded flash driver class.
 */
typedef struct hal_efl_driver EFlashDriver;

#include "hal_efl_lld.h"

/**
 * @brief   @p EFlashDriver specific methods.
 */
#define _efl_flash_methods_alone

/**
 * @brief   @p EFlashDriver specific methods with inherited ones.
 */
#define _efl_flash_methods                                                  \
  _base_flash_methods                                                       \
  _efl_flash_methods_alone

/**
 * @brief   @p EFlashDriver specific data.
 */
#define _efl_driver_data                                                    \
  _base_flash_data                                                          \
  /* Current configuration data.*/                                          \
  const EFlashConfig            *config;

/**
 * @extends BaseFlashVMT
 *
 * @brief   @p EFlash virtual methods table.
 */
struct EFlashDriverVMT {
  _efl_flash_methods
};

/**
 * @brief   Type of a structure representing a flash driver configuration.
 */
typedef struct hal_efl_config {
  /* End of the mandatory fields.*/
  efl_lld_config_fields
} EFlashConfig;

/**
 * @extends BaseFlash
 *
 * @brief   Structure representing an embedded flash driver.
 */
struct hal_efl_driver {
  /**
   * @brief   EFlashDriver Virtual Methods Table.
   */
  const struct EFlashDriverVMT  *vmt;
  _efl_driver_data
#if (EFL_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting EFL.
   */
  mutex_t                   mutex;
#endif /* EFL_USE_MUTUAL_EXCLUSION == TRUE */
  /* End of the mandatory fields.*/
  efl_lld_driver_fields
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
  void  eflInit(void);
  void  eflObjectInit(EFlashDriver *eflp);
  msg_t eflStart(EFlashDriver *eflp, const EFlashConfig *config);
  void  eflStop(EFlashDriver *eflp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EFL == TRUE */

#endif /* HAL_EFL_H */

/** @} */
