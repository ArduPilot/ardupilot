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
 * @file        oop_base_object.h
 * @brief       Generated Base Object header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  OOP_BASE_OBJECT
 * @brief       Common ancestor abstract class.
 * @{
 */

#ifndef OOP_BASE_OBJECT_H
#define OOP_BASE_OBJECT_H

#if (defined(OOP_USE_CHIBIOS)) || defined (__DOXYGEN__)
#include "ch.h"
#elif defined(OOP_USE_NOTHING)
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#else
#include "ccportab.h"
#include "osal.h"
#endif

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
 * @brief       Returns an object pointer starting from a pointer to a class
 *              member.
 *
 * @param         c             Class type name.
 * @param         m             Class member field name.
 * @param[in]     p             Class member field pointer.
 * @return                      A pointer to an object of type @p c containing
 *                              the field @p m.
 *
 * @api
 */
#define oopGetOwner(c, m, p)                                                \
  (c *)(((size_t)(p)) - (size_t)offsetof(c, m))

/**
 * @brief       Returns a pointer to one of the implemented interfaces.
 * @note        The interface pointer is returned as a <tt>void *</tt> in order
 *              to be directly usable as any of the interface's ancestors
 *              pointers.
 *
 * @param[in]     ip            Pointer to the class instance.
 * @param         ifns          Implemented interface namespace.
 * @return                      A void pointer to the interface within the
 *                              class instance.
 *
 * @api
 */
#define oopGetIf(ip, ifns)                                                  \
  (void *)(&(ip)->ifns)

#if (defined(OOP_USE_CHIBIOS)) || defined (__DOXYGEN__)
/**
 * @brief       Condition assertion.
 *
 * @param         c             Condition to be proven true.
 * @param         r             Remark associated to the assertion.
 */
#define oopAssert(c, r)                                                     \
  chDbgAssert(c, r)

/**
 * @brief       Critical section enter.
 * @note        The critical section nature is not specified, implementation
 *              depends on the chosen underlying OS.
 */
#define oopLock()                                                           \
  chSysLock()

/**
 * @brief       Critical section leave.
 * @note        The critical section nature is not specified, implementation
 *              depends on the chosen underlying OS.
 */
#define oopUnlock()                                                         \
  chSysUnlock()

#elif defined(OOP_USE_NOTHING)
#define oopAssert(c, r)                                                     \
  do {                                                                      \
    (void)c;                                                                \
    (void)r;                                                                \
  } while (false)
#define oopLock()
#define oopUnlock()

#else
#define oopAssert(c, r)                                                     \
  osalDbgAssert(c, r)
#define oopLock()                                                           \
  osalSysLock()
#define oopUnlock()                                                         \
  osalSysUnlock()
#endif /* defined(OOP_USE_CHIBIOS) */

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @class       base_object_c
 *
 * @brief       Common ancestor abstract class.
 * @details     This abstract class is the common ancestor of all classes used
 *              in ChibiOS. This class just defines the position of the VMT
 *              pointer inside the structure.
 *
 * @name        Class @p base_object_c structures
 * @{
 */

/**
 * @brief       Type of a base object class.
 */
typedef struct base_object base_object_c;

/**
 * @brief       Class @p base_object_c virtual methods table.
 */
struct base_object_vmt {
  /* From base_object_c.*/
  void (*dispose)(void *ip);
};

/**
 * @brief       Structure representing a base object class.
 */
struct base_object {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct base_object_vmt *vmt;
};
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  /* Methods of base_object_c.*/
  void *__bo_objinit_impl(void *ip, const void *vmt);
  void __bo_dispose_impl(void *ip);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Virtual methods of base_object_c
 * @{
 */
/**
 * @brief       Object finalization.
 *
 * @param[in,out] ip            Pointer to a @p base_object_c instance.
 *
 * @dispose
 */
CC_FORCE_INLINE
static inline void boDispose(void *ip) {
  base_object_c *self = (base_object_c *)ip;

  self->vmt->dispose(ip);
}
/** @} */

/**
 * @name        Inline methods of base_object_c
 * @{
 */
/**
 * @brief       Conditional object finalization.
 * @details     The object is disposed if the passed reference is different
 *              from @p NULL.
 *
 * @param[in,out] ip            Pointer to a @p base_object_c instance.
 */
CC_FORCE_INLINE
static inline void boFree(void *ip) {
  base_object_c *self = (base_object_c *)ip;

  if (self != NULL) {
    boDispose(self);
  }
}
/** @} */

#endif /* OOP_BASE_OBJECT_H */

/** @} */
