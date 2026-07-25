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
 * @file        oop_memstreams.c
 * @brief       Generated Memory Streams source.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  OOP_MEMSTREAMS
 * @{
 */

#include "oop_memstreams.h"

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
/* Module class "memory_stream_c" methods.                                   */
/*===========================================================================*/

/**
 * @name        Interfaces implementation of memory_stream_c
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
static size_t __memstm_stm_write_impl(void *ip, const uint8_t *bp, size_t n) {
  memory_stream_c *self = oopIfGetOwner(memory_stream_c, ip);

  if (self->size - self->eos < n) {
    n = self->size - self->eos;
  }

  memcpy(self->buffer + self->eos, bp, n);
  self->eos += n;

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
static size_t __memstm_stm_read_impl(void *ip, uint8_t *bp, size_t n) {
  memory_stream_c *self = oopIfGetOwner(memory_stream_c, ip);

  if (self->eos - self->offset < n) {
    n = self->eos - self->offset;
  }

  memcpy(bp, self->buffer + self->offset, n);
  self->offset += n;

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
static int __memstm_stm_put_impl(void *ip, uint8_t b) {
  memory_stream_c *self = oopIfGetOwner(memory_stream_c, ip);

  if (self->size - self->eos <= 0U) {
    return STM_RESET;
  }

  *(self->buffer + self->eos) = b;
  self->eos++;

  return STM_OK;
}

/**
 * @brief       Implementation of interface method @p stmGet().
 *
 * @param[in,out] ip            Pointer to the @p sequential_stream_i class
 *                              interface.
 * @return                      A byte value from the stream.
 */
static int __memstm_stm_get_impl(void *ip) {
  memory_stream_c *self = oopIfGetOwner(memory_stream_c, ip);
  uint8_t b;

  if (self->eos - self->offset <= 0U) {
    return STM_RESET;
  }

  b = *(self->buffer + self->offset);
  self->offset++;

  return b;
}

/**
 * @brief       Implementation of interface method @p stmUnget().
 *
 * @param[in,out] ip            Pointer to the @p sequential_stream_i class
 *                              interface.
 * @param[in]     b             The byte value to be pushed back to the stream.
 * @return                      The operation status.
 */
static int __memstm_stm_unget_impl(void *ip, int b) {
  memory_stream_c *self = oopIfGetOwner(memory_stream_c, ip);

  if ((b == STM_RESET) || (self->offset <= 0U)) {
    return STM_RESET;
  }

  self->offset--;
  *(self->buffer + self->offset) = (uint8_t)b;

  return STM_OK;
}
/** @} */

/**
 * @name        Methods implementations of memory_stream_c
 * @{
 */
/**
 * @brief       Implementation of object creation.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[out]    ip            Pointer to a @p memory_stream_c instance to be
 *                              initialized.
 * @param[in]     vmt           VMT pointer for the new object.
 * @param         buffer        Pointer to the memory buffer for the memory
 *                              stream.
 * @param         size          Size of the memory stream buffer.
 * @param         eos           Initial End Of Stream offset. Normally you need
 *                              to put this to zero for RAM buffers or equal to
 *                              @p size for ROM streams.
 * @return                      A new reference to the object.
 */
void *__memstm_objinit_impl(void *ip, const void *vmt, uint8_t *buffer,
                            size_t size, size_t eos) {
  memory_stream_c *self = (memory_stream_c *)ip;

  /* Initialization of the ancestors-defined parts.*/
  __bo_objinit_impl(self, vmt);

  /* Initialization of interface sequential_stream_i.*/
  {
    static const struct sequential_stream_vmt memstm_stm_vmt = {
      .instance_offset      = offsetof(memory_stream_c, stm),
      .write                = __memstm_stm_write_impl,
      .read                 = __memstm_stm_read_impl,
      .put                  = __memstm_stm_put_impl,
      .get                  = __memstm_stm_get_impl,
      .unget                = __memstm_stm_unget_impl
    };
    oopIfObjectInit(&self->stm, &memstm_stm_vmt);
  }

  /* Initialization code.*/

  self->buffer = buffer;
  self->size   = size;
  self->eos    = eos;
  self->offset = 0U;

  return self;
}

/**
 * @brief       Implementation of object finalization.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[in,out] ip            Pointer to a @p memory_stream_c instance to be
 *                              disposed.
 */
void __memstm_dispose_impl(void *ip) {
  memory_stream_c *self = (memory_stream_c *)ip;

  /* No finalization code.*/
  (void)self;

  /* Finalization of the ancestors-defined parts.*/
  __bo_dispose_impl(self);
}
/** @} */

/**
 * @brief       VMT structure of memory stream class.
 * @note        It is public because accessed by the inlined constructor.
 */
const struct memory_stream_vmt __memory_stream_vmt = {
  .dispose                  = __memstm_dispose_impl
};

/** @} */
