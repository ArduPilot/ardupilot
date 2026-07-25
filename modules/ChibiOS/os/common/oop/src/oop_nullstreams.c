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
 * @file        oop_nullstreams.c
 * @brief       Generated Null Streams source.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  OOP_NULLSTREAMS
 * @{
 */

#include "oop_nullstreams.h"

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
/* Module class "null_stream_c" methods.                                     */
/*===========================================================================*/

/**
 * @name        Interfaces implementation of null_stream_c
 * @{
 */
/**
 * @brief       Implementation of interface method @p stmWrite().
 *
 * @param[in,out] ip            Pointer to the @p sequential_stream_i class
 *                              interface.
 * @param[in]     bp            Pointer to the data buffer.
 * @param[in]     n             The maximum amount of data to be transferred.
 * @return                      The number of bytes transferred. The returned
 *                              value can be less than the specified number of
 *                              bytes if an end-of-file condition has been met.
 */
static size_t __nullstm_stm_write_impl(void *ip, const uint8_t *bp, size_t n) {
  null_stream_c *self = oopIfGetOwner(null_stream_c, ip);

  (void)self;
  (void)bp;

  return n;
}

/**
 * @brief       Implementation of interface method @p stmRead().
 *
 * @param[in,out] ip            Pointer to the @p sequential_stream_i class
 *                              interface.
 * @param[out]    bp            Pointer to the data buffer.
 * @param[in]     n             The maximum amount of data to be transferred.
 * @return                      The number of bytes transferred. The returned
 *                              value can be less than the specified number of
 *                              bytes if an end-of-file condition has been met.
 */
static size_t __nullstm_stm_read_impl(void *ip, uint8_t *bp, size_t n) {
  null_stream_c *self = oopIfGetOwner(null_stream_c, ip);

  (void)self;
  (void)bp;

  return n;
}

/**
 * @brief       Implementation of interface method @p stmPut().
 *
 * @param[in,out] ip            Pointer to the @p sequential_stream_i class
 *                              interface.
 * @param[in]     b             The byte value to be written to the stream.
 * @return                      The operation status.
 */
static int __nullstm_stm_put_impl(void *ip, uint8_t b) {
  null_stream_c *self = oopIfGetOwner(null_stream_c, ip);

  (void)self;
  (void)b;

  return STM_OK;
}

/**
 * @brief       Implementation of interface method @p stmGet().
 *
 * @param[in,out] ip            Pointer to the @p sequential_stream_i class
 *                              interface.
 * @return                      A byte value from the stream.
 */
static int __nullstm_stm_get_impl(void *ip) {
  null_stream_c *self = oopIfGetOwner(null_stream_c, ip);

  (void)self;

  return 4;
}

/**
 * @brief       Implementation of interface method @p stmUnget().
 *
 * @param[in,out] ip            Pointer to the @p sequential_stream_i class
 *                              interface.
 * @param[in]     b             The byte value to be pushed back to the stream.
 * @return                      The operation status.
 */
static int __nullstm_stm_unget_impl(void *ip, int b) {
  null_stream_c *self = oopIfGetOwner(null_stream_c, ip);

  (void)self;
  (void)b;

  return STM_RESET;
}
/** @} */

/**
 * @name        Methods implementations of null_stream_c
 * @{
 */
/**
 * @brief       Implementation of object creation.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[out]    ip            Pointer to a @p null_stream_c instance to be
 *                              initialized.
 * @param[in]     vmt           VMT pointer for the new object.
 * @return                      A new reference to the object.
 */
void *__nullstm_objinit_impl(void *ip, const void *vmt) {
  null_stream_c *self = (null_stream_c *)ip;

  /* Initialization of the ancestors-defined parts.*/
  __bo_objinit_impl(self, vmt);

  /* Initialization of interface sequential_stream_i.*/
  {
    static const struct sequential_stream_vmt nullstm_stm_vmt = {
      .instance_offset      = offsetof(null_stream_c, stm),
      .write                = __nullstm_stm_write_impl,
      .read                 = __nullstm_stm_read_impl,
      .put                  = __nullstm_stm_put_impl,
      .get                  = __nullstm_stm_get_impl,
      .unget                = __nullstm_stm_unget_impl
    };
    oopIfObjectInit(&self->stm, &nullstm_stm_vmt);
  }

  /* No initialization code.*/

  return self;
}

/**
 * @brief       Implementation of object finalization.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[in,out] ip            Pointer to a @p null_stream_c instance to be
 *                              disposed.
 */
void __nullstm_dispose_impl(void *ip) {
  null_stream_c *self = (null_stream_c *)ip;

  /* No finalization code.*/
  (void)self;

  /* Finalization of the ancestors-defined parts.*/
  __bo_dispose_impl(self);
}
/** @} */

/**
 * @brief       VMT structure of null stream class.
 * @note        It is public because accessed by the inlined constructor.
 */
const struct null_stream_vmt __null_stream_vmt = {
  .dispose                  = __nullstm_dispose_impl
};

/** @} */
