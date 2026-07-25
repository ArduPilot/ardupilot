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
 * @file    hal_objects.h
 * @brief   Base object.
 * @details This header defines a base object that is the root for the
 *          inheritance system.
 *
 * @addtogroup HAL_BASE_OBJECT
 * @details HAL uses concepts of Object Oriented Programming even if it
 *          is written in C. Things like simple inheritance, multiple
 *          inheritance and interfaces are used through the system.
 *          This module defines a "base object" that is the ancestor of
 *          all classes in the system.
 * @{
 */

#ifndef HAL_OBJECTS_H
#define HAL_OBJECTS_H

/**
 * @brief   @p BaseObject specific methods.
 * @note    This object defines no methods.
 */
#define _base_object_methods                                                \
  /* Instance offset, used for multiple inheritance, normally zero. It
     represents the offset between the current object and the container
     object*/                                                               \
  size_t instance_offset;

/**
 * @brief   @p BaseObject specific data.
 * @note    This object defines no data.
 */
#define _base_object_data

/**
 * @brief   @p BaseObject virtual methods table.
 */
struct BaseObjectVMT {
  _base_object_methods
};

/**
 * @brief   Base object class.
 * @details This class represents a generic object including a virtual
 *          methods table (VMT).
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BaseObjectVMT *vmt;
  _base_object_data
} BaseObject;

/**
 * @name    Macro Functions (BaseObject)
 * @{
 */
/**
 * @brief   Returns the instance pointer starting from an interface pointer.
 *
 * @param[in] type  the type of the instance pointer, it is used for casting
 * @param[in] ip    the interface pointer
 * @return          A pointer to the object implementing the interface
 */
#define objGetInstance(type, ip)                                            \
  (type)(((size_t)(ip)) - (ip)->vmt->instance_offset)
/** @} */

#endif /* HAL_OBJECTS_H */

/** @} */
