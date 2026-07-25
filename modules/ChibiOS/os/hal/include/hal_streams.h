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
 * @file    hal_streams.h
 * @brief   Data streams.
 * @details This header defines abstract interfaces useful to access generic
 *          data streams in a standardized way.
 *
 * @addtogroup HAL_STREAMS
 * @details This module define an abstract interface for generic data streams.
 *          Note that no code is present, just abstract interfaces-like
 *          structures, you should look at the system as to a set of
 *          abstract C++ classes (even if written in C). This system
 *          has then advantage to make the access to data streams
 *          independent from the implementation logic.<br>
 *          The stream interface can be used as base class for high level
 *          object types such as files, sockets, serial ports, pipes etc.
 * @{
 */

#ifndef HAL_STREAMS_H
#define HAL_STREAMS_H


/* Temporary, in order to avoid conflicts with the same definitions in new
   OOP streams.*/
#undef STM_OK
#undef STM_TIMEOUT
#undef STM_RESET

/**
 * @name    Streams return codes
 * @{
 */
#define STM_OK               MSG_OK
#define STM_TIMEOUT          MSG_TIMEOUT
#define STM_RESET            MSG_RESET
/** @} */

/**
 * @brief   BaseSequentialStream specific methods.
 */
#define _base_sequential_stream_methods                                     \
  _base_object_methods                                                      \
  /* Stream write buffer method.*/                                          \
  size_t (*write)(void *instance, const uint8_t *bp, size_t n);             \
  /* Stream read buffer method.*/                                           \
  size_t (*read)(void *instance, uint8_t *bp, size_t n);                    \
  /* Channel put method, blocking.*/                                        \
  msg_t (*put)(void *instance, uint8_t b);                                  \
  /* Channel get method, blocking.*/                                        \
  msg_t (*get)(void *instance);

/**
 * @brief   @p BaseSequentialStream specific data.
 * @note    It is empty because @p BaseSequentialStream is only an interface
 *          without implementation.
 */
#define _base_sequential_stream_data                                        \
  _base_object_data

/**
 * @brief   @p BaseSequentialStream virtual methods table.
 */
struct BaseSequentialStreamVMT {
  _base_sequential_stream_methods
};

/**
 * @extends BaseObject
 *
 * @brief   Base stream class.
 * @details This class represents a generic blocking unbuffered sequential
 *          data stream.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BaseSequentialStreamVMT *vmt;
  _base_sequential_stream_data
} BaseSequentialStream;

/**
 * @name    Macro Functions (BaseSequentialStream)
 * @{
 */
/**
 * @brief   Sequential Stream write.
 * @details The function writes data from a buffer to a stream.
 *
 * @param[in] ip        pointer to a @p BaseSequentialStream or derived class
 * @param[in] bp        pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred
 * @return              The number of bytes transferred. The return value can
 *                      be less than the specified number of bytes if an
 *                      end-of-file condition has been met.
 *
 * @api
 */
#define streamWrite(ip, bp, n) ((ip)->vmt->write(ip, bp, n))

/**
 * @brief   Sequential Stream read.
 * @details The function reads data from a stream into a buffer.
 *
 * @param[in] ip        pointer to a @p BaseSequentialStream or derived class
 * @param[out] bp       pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred
 * @return              The number of bytes transferred. The return value can
 *                      be less than the specified number of bytes if an
 *                      end-of-file condition has been met.
 *
 * @api
 */
#define streamRead(ip, bp, n) ((ip)->vmt->read(ip, bp, n))

/**
 * @brief   Sequential Stream blocking byte write.
 * @details This function writes a byte value to a channel. If the channel
 *          is not ready to accept data then the calling thread is suspended.
 *
 * @param[in] ip        pointer to a @p BaseChannel or derived class
 * @param[in] b         the byte value to be written to the channel
 *
 * @return              The operation status.
 * @retval STM_OK       if the operation succeeded.
 * @retval STM_RESET    if an end-of-file condition has been met.
 *
 * @api
 */
#define streamPut(ip, b) ((ip)->vmt->put(ip, b))

/**
 * @brief   Sequential Stream blocking byte read.
 * @details This function reads a byte value from a channel. If the data
 *          is not available then the calling thread is suspended.
 *
 * @param[in] ip        pointer to a @p BaseChannel or derived class
 *
 * @return              A byte value from the queue.
 * @retval STM_RESET    if an end-of-file condition has been met.
 *
 * @api
 */
#define streamGet(ip) ((ip)->vmt->get(ip))
/** @} */

/**
 * @brief   @p BaseBufferedStream specific methods.
 */
#define _base_buffered_stream_methods                                       \
  _base_sequential_stream_methods                                           \
  /* Channel unget method */                                                \
  msg_t (*unget)(void *instance, uint8_t b);

/**
 * @brief   @p BaseBufferedStream specific data.
 * @note    It is empty because @p BaseBufferedStream is only an interface
 *          without implementation.
 */
#define _base_buffered_stream_data                                          \
  _base_sequential_stream_data

/**
 * @extends BaseSequentialStreamVMT
 *
 * @brief   @p BaseBufferedStream virtual methods table.
 */
struct BaseBufferedStreamVMT {
  _base_buffered_stream_methods
};

/**
 * @extends BaseSequentialStream
 *
 * @brief   Buffered stream class.
 * @details This class @p extends BaseSequentialStream to represent a generic
 *          blocking buffered sequential data stream.
 */
typedef struct {
  /** @brief Virtual Methods Table. */
  const struct BaseBufferedStreamVMT *vmt;
  _base_buffered_stream_data
} BaseBufferedStream;

/**
 * @name    Macro Functions (BaseBufferedStream)
 * @{
 */
/**
 * @brief   Buffered Stream unget.
 * @details This function replaces a byte value to a stream. streamUnget
 *          only guarantees a single byte can be replaced, and multiple
 *          calls without intervening calls to streamGet or streamRead may fail
 *
 * @param[in] ip        pointer to a @p BaseBufferedStream or derived class
 * @param[in] b         the byte value to be written to the channel
 *
 * @post
 *
 * @return              The operation status.
 * @retval STM_OK       if the operation succeeded.
 * @retval STM_RESET    if the operation failed
 *
 * @api
 */
#define streamUnget(ip, b) ((ip)->vmt->unget(ip, b))
/** @} */

#endif /* HAL_STREAMS_H */

/** @} */
