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
 * @file    simulator/hal_efl_lld.h
 * @brief   Simulator Embedded Flash subsystem low level driver header.
 *
 * @addtogroup HAL_EFL
 * @{
 */

#ifndef HAL_EFL_LLD_H
#define HAL_EFL_LLD_H

#if (HAL_USE_EFL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Simulator EFL configuration options
 * @{
 */
/**
 * @brief   Simulated flash total size.
 */
#if !defined(SIM_EFL_TOTAL_SIZE) || defined(__DOXYGEN__)
#define SIM_EFL_TOTAL_SIZE                  (256U * 1024U)
#endif

/**
 * @brief   Simulated flash sector size.
 */
#if !defined(SIM_EFL_SECTOR_SIZE) || defined(__DOXYGEN__)
#define SIM_EFL_SECTOR_SIZE                 4096U
#endif

/**
 * @brief   Simulated flash write page size.
 */
#if !defined(SIM_EFL_PAGE_SIZE) || defined(__DOXYGEN__)
#define SIM_EFL_PAGE_SIZE                   16U
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if SIM_EFL_TOTAL_SIZE == 0
#error "invalid SIM_EFL_TOTAL_SIZE value"
#endif

#if SIM_EFL_SECTOR_SIZE == 0
#error "invalid SIM_EFL_SECTOR_SIZE value"
#endif

#if SIM_EFL_PAGE_SIZE == 0
#error "invalid SIM_EFL_PAGE_SIZE value"
#endif

#if (SIM_EFL_TOTAL_SIZE % SIM_EFL_SECTOR_SIZE) != 0
#error "SIM_EFL_TOTAL_SIZE is not a multiple of SIM_EFL_SECTOR_SIZE"
#endif

#if SIM_EFL_PAGE_SIZE > SIM_EFL_SECTOR_SIZE
#error "SIM_EFL_PAGE_SIZE exceeds SIM_EFL_SECTOR_SIZE"
#endif

#if (SIM_EFL_PAGE_SIZE & (SIM_EFL_PAGE_SIZE - 1U)) != 0U
#error "SIM_EFL_PAGE_SIZE is not a power of two"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the embedded flash driver structure.
 */
#define efl_lld_driver_fields                                               \
  uint8_t                  *memory;                                        \
  const flash_descriptor_t *descriptor;

/**
 * @brief   Low level fields of the embedded flash configuration structure.
 */
#define efl_lld_config_fields                                               \
  uint32_t                  dummy;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern EFlashDriver EFLD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void efl_lld_init(void);
  void efl_lld_start(EFlashDriver *eflp);
  void efl_lld_stop(EFlashDriver *eflp);
  const flash_descriptor_t *efl_lld_get_descriptor(void *instance);
  flash_error_t efl_lld_read(void *instance, flash_offset_t offset,
                             size_t n, uint8_t *rp);
  flash_error_t efl_lld_program(void *instance, flash_offset_t offset,
                                size_t n, const uint8_t *pp);
  flash_error_t efl_lld_start_erase_all(void *instance);
  flash_error_t efl_lld_start_erase_sector(void *instance,
                                           flash_sector_t sector);
  flash_error_t efl_lld_query_erase(void *instance, uint32_t *msec);
  flash_error_t efl_lld_verify_erase(void *instance, flash_sector_t sector);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EFL == TRUE */

#endif /* HAL_EFL_LLD_H */

/** @} */
