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
 * @file        hal_xsnor_macronix_mx25.h
 * @brief       Generated SNOR Macronix MX25 header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  HAL_XSNOR_MACRONIX_MX25
 * @brief       SNOR Macronix MX25 driver.
 * @details     Module for SNOR Macronix MX25 flash devices.
 * @{
 */

#ifndef HAL_XSNOR_MACRONIX_MX25_H
#define HAL_XSNOR_MACRONIX_MX25_H

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

/**
 * @name    Device options
 * @{
 */
/**
 * @brief       Mask of the dummy cycles field.
 */
#define MX25_OPT_DUMMY_CYCLES_MASK          (15U << 0)

/**
 * @brief       Number of dummy cycles.
 *
 * @param         n             Number of dummy cycles (2..15)
 */
#define MX25_OPT_DUMMY_CYCLES(n)            ((n) << 0)

/**
 * @brief       Switch bus width on initialization.
 * @details     If @p MX25_OPT_NO_WIDTH_SWITCH is specified then this is the
 *              bus mode that the device is expected to be using else this is
 *              the bus mode that the device will be switched in.
 * @note        This option is only valid in WSPI bus mode.
 */
#define MX25_OPT_NO_WIDTH_SWITCH            (1U << 4)

/**
 * @brief       Use 4kB sub-sectors rather than 64kB sectors.
 */
#define MX25_OPT_USE_SUBSECTORS             (1U << 5)

/**
 * @brief       Delays insertion.
 * @details     If enabled this option inserts delays into the flash waiting
 *              routines releasing some extra CPU time for threads with lower
 *              priority, this may slow down the driver a bit however.
 */
#define MX25_OPT_NICE_WAITING               (1U << 6)

/**
 * @brief       Enables DTR in 8 lines mode.
 */
#define MX25_OPT_USE_DTR                    (1U << 7)
/** @} */

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @class       hal_xsnor_macronix_mx25_c
 * @extends     hal_xsnor_base_c
 *
 *
 * @name        Class @p hal_xsnor_macronix_mx25_c structures
 * @{
 */

/**
 * @brief       Type of a SNOR Macronix MX25 driver class.
 */
typedef struct hal_xsnor_macronix_mx25 hal_xsnor_macronix_mx25_c;

/**
 * @brief       Class @p hal_xsnor_macronix_mx25_c virtual methods table.
 */
struct hal_xsnor_macronix_mx25_vmt {
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
  /* From hal_xsnor_macronix_mx25_c.*/
};

/**
 * @brief       Structure representing a SNOR Macronix MX25 driver class.
 */
struct hal_xsnor_macronix_mx25 {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct hal_xsnor_macronix_mx25_vmt *vmt;
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
  /* Methods of hal_xsnor_macronix_mx25_c.*/
  void *__mx25_objinit_impl(void *ip, const void *vmt);
  void __mx25_dispose_impl(void *ip);
  flash_error_t __mx25_init_impl(void *ip);
  flash_error_t __mx25_read_impl(void *ip, flash_offset_t offset, size_t n,
                                 uint8_t *rp);
  flash_error_t __mx25_program_impl(void *ip, flash_offset_t offset, size_t n,
                                    const uint8_t *pp);
  flash_error_t __mx25_start_erase_all_impl(void *ip);
  flash_error_t __mx25_start_erase_sector_impl(void *ip, flash_sector_t sector);
  flash_error_t __mx25_query_erase_impl(void *ip, unsigned *msec);
  flash_error_t __mx25_verify_erase_impl(void *ip, flash_sector_t sector);
  flash_error_t __mx25_mmap_on_impl(void *ip, uint8_t **addrp);
  void __mx25_mmap_off_impl(void *ip);
  /* Regular functions.*/
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Default constructor of hal_xsnor_macronix_mx25_c
 * @{
 */
/**
 * @brief       Default initialization function of @p
 *              hal_xsnor_macronix_mx25_c.
 *
 * @param[out]    self          Pointer to a @p hal_xsnor_macronix_mx25_c
 *                              instance to be initialized.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline hal_xsnor_macronix_mx25_c *mx25ObjectInit(hal_xsnor_macronix_mx25_c *self) {
  extern const struct hal_xsnor_macronix_mx25_vmt __hal_xsnor_macronix_mx25_vmt;

  return __mx25_objinit_impl(self, &__hal_xsnor_macronix_mx25_vmt);
}
/** @} */

#endif /* HAL_XSNOR_MACRONIX_MX25_H */

/** @} */
