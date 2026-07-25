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
 * @file    hal_files.h
 * @brief   Data files.
 * @details This header defines abstract interfaces useful to access generic
 *          data files in a standardized way.
 *
 * @addtogroup HAL_FILES
 * @details This module define an abstract interface for generic data files by
 *          extending the @p BaseSequentialStream interface. Note that no code
 *          is present, data files are just abstract interface-like structures,
 *          you should look at the systems as to a set of abstract C++ classes
 *          (even if written in C). This system has the advantage to make the
 *          access to streams independent from the implementation logic.<br>
 *          The data files interface can be used as base class for high level
 *          object types such as an API for a File System implementation.
 * @{
 */

#ifndef HAL_FILES_H
#define HAL_FILES_H

/**
 * @name    Files return codes
 * @{
 */
/**
 * @brief   No error return code.
 */
#define FILE_OK         STM_OK

/**
 * @brief   Error code from the file stream methods.
 */
#define FILE_ERROR      STM_TIMEOUT

/**
 * @brief   End-of-file condition for file get/put methods.
 */
#define FILE_EOF        STM_RESET
/** @} */

/**
 * @brief   File offset type.
 */
typedef uint32_t fileoffset_t;

/**
 * @brief   FileStream specific methods.
 */
#define _file_stream_methods                                                \
  _base_sequential_stream_methods                                           \
  /* File close method.*/                                                   \
  msg_t (*close)(void *instance);                                           \
  /* Get last error code method.*/                                          \
  msg_t (*geterror)(void *instance);                                        \
  /* File get size method.*/                                                \
  msg_t (*getsize)(void *instance, fileoffset_t *offset);                   \
  /* File get current position method.*/                                    \
  msg_t (*getposition)(void *instance, fileoffset_t *offset);               \
  /* File set current position method.*/                                    \
  msg_t (*setposition)(void *instance, fileoffset_t offset);

/**
 * @brief   @p FileStream specific data.
 * @note    It is empty because @p FileStream is only an interface
 *          without implementation.
 */
#define _file_stream_data                                                   \
  _base_sequential_stream_data

/**
 * @extends BaseSequentialStreamVMT
 *
 * @brief   @p FileStream virtual methods table.
 */
struct FileStreamVMT {
  _file_stream_methods
};

/**
 * @extends BaseSequentialStream
 *
 * @brief   Base file stream class.
 * @details This class represents a generic file data stream.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct FileStreamVMT *vmt;
  _file_stream_data
} FileStream;

/**
 * @name    Macro Functions (FileStream)
 * @{
 */
/**
 * @brief   File stream write.
 * @details The function writes data from a buffer to a file stream.
 *
 * @param[in] ip        pointer to a @p FileStream or derived class
 * @param[in] bp        pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred
 * @return              The number of bytes transferred. The return value can
 *                      be less than the specified number of bytes if an
 *                      end-of-file condition has been met.
 * @retval FILE_ERROR   operation failed.
 *
 * @api
 */
#define fileStreamWrite(ip, bp, n) streamWrite(ip, bp, n)

/**
 * @brief   File stream read.
 * @details The function reads data from a file stream into a buffer.
 *
 * @param[in] ip        pointer to a @p FileStream or derived class
 * @param[out] bp       pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred
 * @return              The number of bytes transferred. The return value can
 *                      be less than the specified number of bytes if an
 *                      end-of-file condition has been met.
 * @retval FILE_ERROR   operation failed.
 *
 * @api
 */
#define fileStreamRead(ip, bp, n) streamRead(ip, bp, n)

/**
 * @brief   File stream blocking byte write.
 * @details This function writes a byte value to a channel. If the channel
 *          is not ready to accept data then the calling thread is suspended.
 *
 * @param[in] ip        pointer to a @p FileStream or derived class
 * @param[in] b         the byte value to be written to the channel
 *
 * @return              The operation status.
 * @retval FILE_OK      if the operation succeeded.
 * @retval FILE_ERROR   operation failed.
 * @retval FILE_EOF     if an end-of-file condition has been met.
 *
 * @api
 */
#define fileStreamPut(ip, b) streamPut(ip, b)

/**
 * @brief   File stream blocking byte read.
 * @details This function reads a byte value from a channel. If the data
 *          is not available then the calling thread is suspended.
 *
 * @param[in] ip        pointer to a @p FileStream or derived class
 *
 * @return              A byte value from the queue.
 * @retval FILE_ERROR   operation failed.
 * @retval FILE_EOF     if an end-of-file condition has been met.
 *
 * @api
 */
#define fileStreamGet(ip) streamGet(ip)

/**
 * @brief   File Stream close.
 * @details The function closes a file stream.
 *
 * @param[in] ip        pointer to a @p FileStream or derived class
 * @return              The operation status.
 * @retval FILE_OK      no error.
 * @retval FILE_ERROR   operation failed.
 *
 * @api
 */
#define fileStreamClose(ip) ((ip)->vmt->close(ip))

/**
 * @brief   Returns an implementation dependent error code.
 * @pre     The previously called function must have returned @p FILE_ERROR.
 *
 * @param[in] ip        pointer to a @p FileStream or derived class
 * @return              Implementation dependent error code.
 *
 * @api
 */
#define fileStreamGetError(ip) ((ip)->vmt->geterror(ip))

/**
 * @brief   Returns the current file size.
 *
 * @param[in] ip        pointer to a @p FileStream or derived class
 * @param[out] offset   current size of the file
 * @return              The file size.
 * @retval FILE_ERROR   operation failed.
 *
 * @api
 */
#define fileStreamGetSize(ip, offset) ((ip)->vmt->getsize(ip, offset))

/**
 * @brief   Returns the current file pointer position.
 *
 * @param[in] ip        pointer to a @p FileStream or derived class
 * @param[out] offset   current position in the file
 * @return              The current position inside the file.
 * @retval FILE_ERROR   operation failed.
 *
 * @api
 */
#define fileStreamGetPosition(ip, offset) ((ip)->vmt->getposition(ip, offset))

/**
 * @brief   Moves the file current pointer to an absolute position.
 *
 * @param[in] ip        pointer to a @p FileStream or derived class
 * @param[in] offset    new absolute position
 * @return              The operation status.
 * @retval FILE_OK      no error.
 * @retval FILE_ERROR   operation failed.
 *
 * @api
 */
#define fileStreamSetPosition(ip, offset) ((ip)->vmt->setposition(ip, offset))
/** @} */

#endif /* HAL_FILES_H */

/** @} */
