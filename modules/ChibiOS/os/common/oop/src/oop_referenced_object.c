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
 * @file        oop_referenced_object.c
 * @brief       Generated Referenced Object source.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  OOP_REFERENCED_OBJECT
 * @{
 */

#include "oop_referenced_object.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module local macros.                                                      */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module class "referenced_object_c" methods.                               */
/*===========================================================================*/

/**
 * @name        Methods implementations of referenced_object_c
 * @{
 */
/**
 * @brief       Implementation of object creation.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[out]    ip            Pointer to a @p referenced_object_c instance to
 *                              be initialized.
 * @param[in]     vmt           VMT pointer for the new object.
 * @return                      A new reference to the object.
 */
void *__ro_objinit_impl(void *ip, const void *vmt) {
  referenced_object_c *self = (referenced_object_c *)ip;

  /* Initialization of the ancestors-defined parts.*/
  __bo_objinit_impl(self, vmt);

  /* Initialization code.*/
  self->references = (object_references_t)1;

  return self;
}

/**
 * @brief       Implementation of object finalization.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[in,out] ip            Pointer to a @p referenced_object_c instance to
 *                              be disposed.
 */
void __ro_dispose_impl(void *ip) {
  referenced_object_c *self = (referenced_object_c *)ip;

  /* No finalization code.*/
  (void)self;

  /* Finalization of the ancestors-defined parts.*/
  __bo_dispose_impl(self);
}

/**
 * @brief       Implementation of method @p roAddRef().
 * @note        This function is meant to be used by derived classes.
 *
 * @param[in,out] ip            Pointer to a @p referenced_object_c instance.
 * @return                      A new reference pointer.
 */
void *__ro_addref_impl(void *ip) {
  referenced_object_c *self = (referenced_object_c *)ip;

  oopLock();
  self->references++;
  oopAssert(self->references != (object_references_t)0, "overflow");
  oopUnlock();

  return self;
}

/**
 * @brief       Implementation of method @p roRelease().
 * @note        This function is meant to be used by derived classes.
 *
 * @param[in,out] ip            Pointer to a @p referenced_object_c instance.
 * @return                      The value of the reference counter.
 */
object_references_t __ro_release_impl(void *ip) {
  referenced_object_c *self = (referenced_object_c *)ip;

  oopLock();
  oopAssert(self->references > 0U, "zero references");
  if (--self->references == 0U) {
    oopUnlock();
    boDispose(self);
  }
  else {
    oopUnlock();
  }

  return self->references;
}
/** @} */

/** @} */
