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
 * @file        hal_xsnor_device_template.h
 * @brief       Generated SNOR Device Template header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  HAL_XSNOR_DEVICE_TEMPLATE
 * @brief       SNOR device template driver.
 * @details     Template module for SNOR flash devices. Can be used by
 *              duplicating the XML file or working directly on cloned source
 *              files.
 * @{
 */

#ifndef HAL_XSNOR_DEVICE_TEMPLATE_H
#define HAL_XSNOR_DEVICE_TEMPLATE_H

#include "oop_base_object.h"
#include "hal_xsnor_base.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @class       hal_device_template_c
 * @extends     hal_xsnor_base_c
 *
 *
 * @name        Class @p hal_device_template_c structures
 * @{
 */

/**
 * @brief       Type of a SNOR device template driver class.
 */
typedef struct hal_device_template hal_device_template_c;

/**
 * @brief       Class @p hal_device_template_c virtual methods table.
 */
struct hal_device_template_vmt {
  /* From base_object_c.*/
  void (*dispose)(void *ip);
  /* From hal_xsnor_base_c.*/
  flash_error_t (*init)(void *ip);
  flash_error_t (*read)(void *ip, flash_offset_t offset, size_t n, uint8_t *rp);
  flash_error_t (*program)(void *ip, flash_offset_t offset, size_t n, const uint8_t *pp);
  flash_error_t (*start_erase_all)(void *ip);
  flash_error_t (*start_erase_sector)(void *ip, flash_sector_t sector);
  flash_error_t (*query_erase)(void *ip, unsigned *msec);
  flash_error_t (*verify_erase)(void *ip, flash_sector_t sector);
  flash_error_t (*mmap_on)(void *ip, uint8_t **addrp);
  void (*mmap_off)(void *ip);
  /* From hal_device_template_c.*/
};

/**
 * @brief       Structure representing a SNOR device template driver class.
 */
struct hal_device_template {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct hal_device_template_vmt *vmt;
  /**
   * @brief       Implemented interface @p flash_interface_i.
   */
  flash_interface_i         fls;
  /**
   * @brief       Driver state.
   */
  flash_state_t             state;
  /**
   * @brief       Driver configuration.
   */
  const xsnor_config_t      *config;
#if (XSNOR_USE_WSPI == TRUE) || defined (__DOXYGEN__)
  /**
   * @brief       Current commands configuration.
   * @note        This field is meant to be initialized by subclasses on object
   *              creation.
   */
  const xsnor_commands_t    *commands;
#endif /* XSNOR_USE_WSPI == TRUE */
  /**
   * @brief       Flash access mutex.
   */
  mutex_t                   mutex;
  /**
   * @brief       Flash descriptor.
   * @note        This field is meant to be initialized by subclasses on memory
   *              initialization.
   */
  flash_descriptor_t        descriptor;
};
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  /* Methods of hal_device_template_c.*/
  void *__tmpl_objinit_impl(void *ip, const void *vmt);
  void __tmpl_dispose_impl(void *ip);
  flash_error_t __tmpl_init_impl(void *ip);
  flash_error_t __tmpl_read_impl(void *ip, flash_offset_t offset, size_t n,
                                 uint8_t *rp);
  flash_error_t __tmpl_program_impl(void *ip, flash_offset_t offset, size_t n,
                                    const uint8_t *pp);
  flash_error_t __tmpl_start_erase_all_impl(void *ip);
  flash_error_t __tmpl_start_erase_sector_impl(void *ip, flash_sector_t sector);
  flash_error_t __tmpl_query_erase_impl(void *ip, unsigned *msec);
  flash_error_t __tmpl_verify_erase_impl(void *ip, flash_sector_t sector);
  flash_error_t __tmpl_mmap_on_impl(void *ip, uint8_t **addrp);
  void __tmpl_mmap_off_impl(void *ip);
  /* Regular functions.*/
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Default constructor of hal_device_template_c
 * @{
 */
/**
 * @brief       Default initialization function of @p hal_device_template_c.
 *
 * @param[out]    self          Pointer to a @p hal_device_template_c instance
 *                              to be initialized.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline hal_device_template_c *tmplObjectInit(hal_device_template_c *self) {
  extern const struct hal_device_template_vmt __hal_device_template_vmt;

  return __tmpl_objinit_impl(self, &__hal_device_template_vmt);
}
/** @} */

#endif /* HAL_XSNOR_DEVICE_TEMPLATE_H */

/** @} */
