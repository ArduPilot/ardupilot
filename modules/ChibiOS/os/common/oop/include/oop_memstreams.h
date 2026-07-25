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
 * @file        oop_memstreams.h
 * @brief       Generated Memory Streams header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  OOP_MEMSTREAMS
 * @brief       Memory streams class.
 * @{
 */

#ifndef OOP_MEMSTREAMS_H
#define OOP_MEMSTREAMS_H

#include <string.h>
#include "oop_base_object.h"
#include "oop_sequential_stream.h"

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
 * @class       memory_stream_c
 * @extends     base_object_c
 * @implements  sequential_stream_i
 *
 * @brief       Memory streams class.
 * @details     This class allows to manage a memory buffer using a stream
 *              interface.
 *
 * @name        Class @p memory_stream_c structures
 * @{
 */

/**
 * @brief       Type of a memory stream class.
 */
typedef struct memory_stream memory_stream_c;

/**
 * @brief       Class @p memory_stream_c virtual methods table.
 */
struct memory_stream_vmt {
  /* From base_object_c.*/
  void (*dispose)(void *ip);
  /* From memory_stream_c.*/
};

/**
 * @brief       Structure representing a memory stream class.
 */
struct memory_stream {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct memory_stream_vmt *vmt;
  /**
   * @brief       Implemented interface @p sequential_stream_i.
   */
  sequential_stream_i       stm;
  /**
   * @brief       Pointer to the memory buffer.
   */
  uint8_t                   *buffer;
  /**
   * @brief       Size of the memory buffer.
   */
  size_t                    size;
  /**
   * @brief       Current end of the stream.
   */
  size_t                    eos;
  /**
   * @brief       Current read offset.
   */
  size_t                    offset;
};
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  /* Methods of memory_stream_c.*/
  void *__memstm_objinit_impl(void *ip, const void *vmt, uint8_t *buffer,
                              size_t size, size_t eos);
  void __memstm_dispose_impl(void *ip);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Default constructor of memory_stream_c
 * @{
 */
/**
 * @brief       Default initialization function of @p memory_stream_c.
 *
 * @param[out]    self          Pointer to a @p memory_stream_c instance to be
 *                              initialized.
 * @param         buffer        Pointer to the memory buffer for the memory
 *                              stream.
 * @param         size          Size of the memory stream buffer.
 * @param         eos           Initial End Of Stream offset. Normally you need
 *                              to put this to zero for RAM buffers or equal to
 *                              @p size for ROM streams.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline memory_stream_c *memstmObjectInit(memory_stream_c *self,
                                                uint8_t *buffer, size_t size,
                                                size_t eos) {
  extern const struct memory_stream_vmt __memory_stream_vmt;

  return __memstm_objinit_impl(self, &__memory_stream_vmt, buffer, size, eos);
}
/** @} */

#endif /* OOP_MEMSTREAMS_H */

/** @} */
