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
 * @file    oop_base_object.h
 * @brief   Base object.
 * @details This header defines a base class that is the root class of
 *          the ChibiOS Object Model.
 *
 * @addtogroup OOP_BASE_OBJECT
 * @details ChibiOS uses concepts of Object Oriented Programming even if
 *          it is written in C. Things like simple inheritance, multiple
 *          inheritance and interfaces are used through the system.
 *          This module defines a "base object" class that is the ancestor
 *          of all classes in the system.<br>
 *          Class types are denoted by a "_c" suffix, classes contain a
 *          virtual methods table and data encapsulated into a normal C
 *          structure.<br>
 *          Interfaces are denoted by a "_i" suffix, interfaces just have
 *          a virtual methods table as single member of their C structure.<br>
 *          The first field of a VMT is the offset between the container
 *          object and the VMT pointer.<br>
 *          Multiple inheritance is implemented by composing a class
 *          structure with the structures of implemented classes or
 *          interfaces.<br>
 *          Example:
 *          <code>
 *          // Defining a counter interface.
 *          typedef struct {
 *            const struct __counter_interface_vmt  *vmt;
 *          } counter_interface_i;
 *
 *          // Defining a beans interface.
 *          typedef struct {
 *            const struct __beans_interface_vmt    *vmt;
 *          } beans_interface_i;
 *
 *          // Definition of a class myclass_c implementing interfaces
 *          // counter_interface_i and beans_interface_i.
 *          typedef struct {
 *            const struct __myclass_vmt            *vmt;
 *            // Fields of myclass.
 *            counter_interface_i                   counter;
 *            beans_interface_i                     beans;
 *          } myclass_c;
 *          </code>
 * @{
 */

#ifndef OOP_BASE_OBJECT_H
#define OOP_BASE_OBJECT_H

#include "ccportab.h"
#include "osal.h"

/**
 * @brief   Type of a base object class.
 * @details This class represents a generic object including a virtual
 *          methods table (VMT).
 * @note    This class is compatible with the legacy HAL @p BaseObject class.
 */
typedef struct base_object base_object_c;

/**
 * @brief   @p base_object_c specific methods.
 * @note    This object defines no methods.
 */
#define __base_object_methods                                               \
  /* Instance offset, used for multiple inheritance, normally zero. It
     represents the offset between the current object and the container
     object*/                                                               \
  size_t instance_offset;

/**
 * @brief   @p base_object_c specific data.
 * @note    This object defines no data.
 */
#define __base_object_data

/**
 * @brief   @p base_object_c virtual methods table.
 */
struct __base_object_vmt {
  __base_object_methods
};

/**
 * @brief   Structure representing a base object class.
 */
struct base_object {
  /**
   * @brief   Virtual Methods Table.
   */
  const struct __base_object_vmt *vmt;
  __base_object_data
};

/**
 * @name    Methods implementations
 * @{
 */
/**
 * @brief   Object creation implementation.
 *
 * @param[in] ip        Pointer to a @p base_object_c structure to be
 *                      initialized.
 * @param[in] vmt       VMT pointer for the new object.
 * @return              A new reference to the object.
 */
CC_FORCE_INLINE
static inline void *__base_object_objinit_impl(void *ip, const void *vmt) {
  base_object_c *objp = (base_object_c *)ip;

  objp->vmt = (struct __base_object_vmt *)vmt;

  return ip;
}

/**
 * @brief   Object finalization implementation.
 *
 * @param[in] ip        Pointer to a @p base_object_c structure to be
 *                      disposed.
 */
CC_FORCE_INLINE
static inline void __base_object_dispose_impl(void *ip) {

  (void) ip;

  /* Nothing.*/
}
/** @} */

/**
 * @name    OOP Utility macros
 * @{
 */
/**
 * @brief   Returns the object instance pointer starting from an interface
 *          pointer.
 * @details Because multiple inheritance, an object can be composed by
 *          multiple interfaces and/or classes (its ancestors).<br>
 *          This function returns the pointer to the base object starting
 *          from a pointer to any of its composing classes or interfaces.<br>
 *          This is done by leveraging the offset field into each VMT table.
 *
 * @param[in] c     The class type of the object.
 * @param[in] ip    A pointer to one of the object composing classes or
 *                  interfaces.
 * @return          A pointer to an object of type @p type implementing
 *                  the interface @p ip.
 */
#define oopGetInstance(c, ip)                                               \
  (c)(((size_t)(ip)) - (ip)->vmt->instance_offset)
/** @} */

#endif /* OOP_BASE_OBJECT_H */

/** @} */
