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
 * @file        oop_sequential_stream.h
 * @brief       Generated Sequential Stream Interface header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  OOP_SEQUENTIAL_STREAM
 * @brief       Sequential data streams interface.
 * @{
 */

#ifndef OOP_SEQUENTIAL_STREAM_H
#define OOP_SEQUENTIAL_STREAM_H

#include "oop_base_interface.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/* Temporary, in order to avoid conflicts with the same definitions in old
   HAL streams.*/
#undef STM_OK
#undef STM_TIMEOUT
#undef STM_RESET

/**
 * @name    Streams return codes
 * @{
 */
/**
 * @brief       Returned code for operation successful.
 * @note        It is an alias of @p MSG_OK.
 */
#define STM_OK                              0

/**
 * @brief       Returned code for operation timeout.
 * @note        It is an alias of @p MSG_TIMEOUT.
 */
#define STM_TIMEOUT                         -1

/**
 * @brief       Returned code for stream reset or End-of-File.
 * @note        It is an alias of @p MSG_RESET.
 */
#define STM_RESET                           -2
/** @} */

#if (defined(OOP_USE_LEGACY)) || defined (__DOXYGEN__)
/**
 * @name    Legacy interface method names
 * @{
 */
#define streamWrite                         stmWrite
#define streamRead                          stmRead
#define streamPut                           stmPut
#define streamGet                           stmGet
/** @} */
#endif /* defined(OOP_USE_LEGACY) */

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
 * @interface   sequential_stream_i
 * @extends     base_interface_i
 *
 * @brief       Sequential data streams interface.
 * @details     This module define an interface for generic sequential data
 *              streams.
 *              This interface is meant to be implemented in classes requiring
 *              streaming capability.
 * @note        This interface is meant to be compatible with legacy HAL @p
 *              BaseSequentialStream interface.
 *
 * @name        Interface @p sequential_stream_i structures
 * @{
 */

/**
 * @brief       Type of a sequential stream interface.
 */
typedef struct sequential_stream sequential_stream_i;

/**
 * @brief       Interface @p sequential_stream_i virtual methods table.
 */
struct sequential_stream_vmt {
  /* Memory offset between this interface structure and begin of
     the implementing class structure.*/
  size_t instance_offset;
  /* From base_interface_i.*/
  /* From sequential_stream_i.*/
  size_t (*write)(void *ip, const uint8_t *bp, size_t n);
  size_t (*read)(void *ip, uint8_t *bp, size_t n);
  int (*put)(void *ip, uint8_t b);
  int (*get)(void *ip);
  int (*unget)(void *ip, int b);
};

/**
 * @brief       Structure representing a sequential stream interface.
 */
struct sequential_stream {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct sequential_stream_vmt *vmt;
};
/** @} */

#if (defined(OOP_USE_LEGACY)) || defined (__DOXYGEN__)
/**
 * @brief       For compatibility with legacy @p BaseSequentialStream.
 */
typedef sequential_stream_i BaseSequentialStream;
#endif /* defined(OOP_USE_LEGACY) */

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

/**
 * @name        Virtual methods of sequential_stream_i
 * @{
 */
/**
 * @brief       Sequential Stream write.
 * @details     This function writes data from a buffer to a stream.
 *
 * @param[in,out] ip            Pointer to a @p sequential_stream_i instance.
 * @param[in]     bp            Pointer to the data buffer.
 * @param[in]     n             The maximum amount of data to be transferred.
 * @return                      The number of bytes transferred. The returned
 *                              value can be less than the specified number of
 *                              bytes if an end-of-file condition has been met.
 */
CC_FORCE_INLINE
static inline size_t stmWrite(void *ip, const uint8_t *bp, size_t n) {
  sequential_stream_i *self = (sequential_stream_i *)ip;

  return self->vmt->write(ip, bp, n);
}

/**
 * @brief       Sequential Stream read.
 * @details     This function reads data from a stream into a buffer.
 *
 * @param[in,out] ip            Pointer to a @p sequential_stream_i instance.
 * @param[out]    bp            Pointer to the data buffer.
 * @param[in]     n             The maximum amount of data to be transferred.
 * @return                      The number of bytes transferred. The returned
 *                              value can be less than the specified number of
 *                              bytes if an end-of-file condition has been met.
 */
CC_FORCE_INLINE
static inline size_t stmRead(void *ip, uint8_t *bp, size_t n) {
  sequential_stream_i *self = (sequential_stream_i *)ip;

  return self->vmt->read(ip, bp, n);
}

/**
 * @brief       Sequential Stream blocking byte write.
 * @details     This function writes a byte value to a stream. If the stream is
 *              not ready to accept data then the calling thread is suspended.
 *
 * @param[in,out] ip            Pointer to a @p sequential_stream_i instance.
 * @param[in]     b             The byte value to be written to the stream.
 * @return                      The operation status.
 * @retval STM_OK               If the byte has been written.
 * @retval STM_RESET            If an end-of-file condition has been met.
 */
CC_FORCE_INLINE
static inline int stmPut(void *ip, uint8_t b) {
  sequential_stream_i *self = (sequential_stream_i *)ip;

  return self->vmt->put(ip, b);
}

/**
 * @brief       Sequential Stream blocking byte read.
 * @details     This function reads a byte value from a stream. If the data is
 *              not available then the calling thread is suspended.
 *
 * @param[in,out] ip            Pointer to a @p sequential_stream_i instance.
 * @return                      A byte value from the stream.
 * @retval STM_RESET            If an end-of-file condition has been met.
 */
CC_FORCE_INLINE
static inline int stmGet(void *ip) {
  sequential_stream_i *self = (sequential_stream_i *)ip;

  return self->vmt->get(ip);
}

/**
 * @brief       Pushes back a byte into the stream.
 * @details     The specified byte is pushed back into the stream.
 * @note        Implementing classes may have limited push-back buffer capacity
 *              or not be able to push-back at all.
 *
 * @param[in,out] ip            Pointer to a @p sequential_stream_i instance.
 * @param[in]     b             The byte value to be pushed back to the stream.
 * @return                      The operation status.
 * @retval STM_OK               If the byte has been pushed back.
 * @retval STM_RESET            If there is no push-back capacity left.
 */
CC_FORCE_INLINE
static inline int stmUnget(void *ip, int b) {
  sequential_stream_i *self = (sequential_stream_i *)ip;

  return self->vmt->unget(ip, b);
}
/** @} */

#endif /* OOP_SEQUENTIAL_STREAM_H */

/** @} */
