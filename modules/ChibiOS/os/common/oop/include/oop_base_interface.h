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
 * @file        oop_base_interface.h
 * @brief       Generated Base Interface header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  OOP_BASE_INTERFACE
 * @brief       Common interfaces ancestor.
 * @{
 */

#ifndef OOP_BASE_INTERFACE_H
#define OOP_BASE_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ccportab.h"

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

/**
 * @brief       Returns an object pointer starting by an interface pointer.
 * @details     A pointer to an object of class type @p c is returned starting
 *              from a pointer to any of the interfaces implemented by the
 *              class.
 *
 * @param         c             Class type name.
 * @param[in]     ip            Pointer to the interface field within the class
 *                              structure.
 * @return                      A pointer to an object of type @p c.
 *
 * @api
 */
#define oopIfGetOwner(c, ip)                                                \
  (c *)(((size_t)(ip)) - ((base_interface_i *)(ip))->vmt->instance_offset)

/**
 * @brief       Initialization of an interface structure.
 * @details     An interface structure contains only a VMT pointer and no data,
 *              the purpose of this macro is VMT initialization.
 *
 * @param[out]    ip            Pointer to the interface structure to be
 *                              initialized.
 * @param[in]     vmtp          VMT pointer to be assigned to the interface
 *                              structure.
 *
 * @api
 */
#define oopIfObjectInit(ip, vmtp)                                           \
  do {                                                                      \
    (ip)->vmt = (vmtp);                                                     \
  } while (false)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @interface   base_interface_i
 *
 * @brief       Common interfaces ancestor.
 * @details     There are no methods in this interface. This interface is just
 *              meant to be the common ancestor of all interfaces used in
 *              ChibiOS.
 *
 * @name        Interface @p base_interface_i structures
 * @{
 */

/**
 * @brief       Type of a base interface interface.
 */
typedef struct base_interface base_interface_i;

/**
 * @brief       Interface @p base_interface_i virtual methods table.
 */
struct base_interface_vmt {
  /* Memory offset between this interface structure and begin of
     the implementing class structure.*/
  size_t instance_offset;
  /* From base_interface_i.*/
};

/**
 * @brief       Structure representing a base interface interface.
 */
struct base_interface {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct base_interface_vmt *vmt;
};
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

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* OOP_BASE_INTERFACE_H */

/** @} */
