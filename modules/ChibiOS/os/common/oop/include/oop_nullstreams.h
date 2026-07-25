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
 * @file        oop_nullstreams.h
 * @brief       Generated Null Streams header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  OOP_NULLSTREAMS
 * @brief       Null streams class.
 * @{
 */

#ifndef OOP_NULLSTREAMS_H
#define OOP_NULLSTREAMS_H

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
 * @class       null_stream_c
 * @extends     base_object_c
 * @implements  sequential_stream_i
 *
 * @brief       Null streams class.
 * @details     This class implements a null stream.
 *
 * @name        Class @p null_stream_c structures
 * @{
 */

/**
 * @brief       Type of a null stream class.
 */
typedef struct null_stream null_stream_c;

/**
 * @brief       Class @p null_stream_c virtual methods table.
 */
struct null_stream_vmt {
  /* From base_object_c.*/
  void (*dispose)(void *ip);
  /* From null_stream_c.*/
};

/**
 * @brief       Structure representing a null stream class.
 */
struct null_stream {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct null_stream_vmt *vmt;
  /**
   * @brief       Implemented interface @p sequential_stream_i.
   */
  sequential_stream_i       stm;
};
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  /* Methods of null_stream_c.*/
  void *__nullstm_objinit_impl(void *ip, const void *vmt);
  void __nullstm_dispose_impl(void *ip);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Default constructor of null_stream_c
 * @{
 */
/**
 * @brief       Default initialization function of @p null_stream_c.
 *
 * @param[out]    self          Pointer to a @p null_stream_c instance to be
 *                              initialized.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline null_stream_c *nullstmObjectInit(null_stream_c *self) {
  extern const struct null_stream_vmt __null_stream_vmt;

  return __nullstm_objinit_impl(self, &__null_stream_vmt);
}
/** @} */

#endif /* OOP_NULLSTREAMS_H */

/** @} */
