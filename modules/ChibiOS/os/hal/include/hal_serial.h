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
 * @file    hal_serial.h
 * @brief   Serial Driver macros and structures.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_H
#define HAL_SERIAL_H

#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Serial status flags (legacy)
 * @{
 */
#define SD_PARITY_ERROR         CHN_PARITY_ERROR
#define SD_FRAMING_ERROR        CHN_FRAMING_ERROR
#define SD_OVERRUN_ERROR        CHN_OVERRUN_ERROR
#define SD_NOISE_ERROR          CHN_NOISE_ERROR
#define SD_BREAK_DETECTED       CHN_BREAK_DETECTED
#define SD_QUEUE_FULL_ERROR     CHN_BUFFER_FULL_ERROR
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Serial configuration options
 * @{
 */
/**
 * @brief   Default bit rate.
 * @details Configuration parameter, this is the baud rate selected for the
 *          default configuration.
 */
#if !defined(SERIAL_DEFAULT_BITRATE) || defined(__DOXYGEN__)
#define SERIAL_DEFAULT_BITRATE      38400
#endif

/**
 * @brief   Serial buffers size.
 * @details Configuration parameter, you can change the depth of the queue
 *          buffers depending on the requirements of your application.
 * @note    The default is 16 bytes for both the transmission and receive
 *          buffers.
 * @note    This is a global setting and it can be overridden by low level
 *          driver specific settings.
 */
#if !defined(SERIAL_BUFFERS_SIZE) || defined(__DOXYGEN__)
#define SERIAL_BUFFERS_SIZE         16
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief Driver state machine possible states.
 */
typedef enum {
  SD_UNINIT = 0,                    /**< Not initialized.                   */
  SD_STOP = 1,                      /**< Stopped.                           */
  SD_READY = 2                      /**< Ready.                             */
} sdstate_t;

/**
 * @brief   Structure representing a serial driver.
 */
typedef struct hal_serial_driver SerialDriver;

#include "hal_serial_lld.h"

/**
 * @brief   @p SerialDriver specific methods.
 */
#define _serial_driver_methods                                              \
  _base_asynchronous_channel_methods

/**
 * @extends BaseAsynchronousChannelVMT
 *
 * @brief   @p SerialDriver virtual methods table.
 */
struct SerialDriverVMT {
  _serial_driver_methods
};

/**
 * @extends BaseAsynchronousChannel
 *
 * @brief   Full duplex serial driver class.
 * @details This class extends @p BaseAsynchronousChannel by adding physical
 *          I/O queues.
 */
struct hal_serial_driver {
  /** @brief Virtual Methods Table.*/
  const struct SerialDriverVMT *vmt;
  _serial_driver_data
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Direct write to a @p SerialDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly on the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] b         the byte value to be written in the output queue
 * @return              The operation status.
 * @retval MSG_OK       if the operation succeeded.
 * @retval MSG_TIMEOUT  if the queue is full.
 *
 * @iclass
 */
#define sdPutI(sdp, b) oqPutI(&(sdp)->oqueue, b)

/**
 * @brief   Direct write to a @p SerialDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly on the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] b         the byte value to be written in the output queue
 * @return              The operation status.
 * @retval MSG_OK       if the operation succeeded.
 * @retval MSG_RESET    if the @p SerialDriver has been stopped.
 *
 * @api
 */
#define sdPut(sdp, b) oqPut(&(sdp)->oqueue, b)

/**
 * @brief   Direct write to a @p SerialDriver with timeout specification.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly on the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] b         the byte value to be written in the output queue
 * @param[in] t         the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 * @return              The operation status.
 * @retval MSG_OK       if the operation succeeded.
 * @retval MSG_TIMEOUT  if the specified time expired.
 * @retval MSG_RESET    if the @p SerialDriver has been stopped.
 *
 * @api
 */
#define sdPutTimeout(sdp, b, t) oqPutTimeout(&(sdp)->oqueue, b, t)

/**
 * @brief   Direct read from a @p SerialDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @return              A byte value from the input queue.
 * @retval MSG_TIMEOUT  if the queue is empty.
 *
 * @iclass
 */
#define sdGetI(sdp) iqGetI(&(sdp)->iqueue)

/**
 * @brief   Direct read from a @p SerialDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @return              A byte value from the input queue.
 * @retval MSG_RESET    if the @p SerialDriver has been stopped.
 *
 * @api
 */
#define sdGet(sdp) iqGet(&(sdp)->iqueue)

/**
 * @brief   Direct read from a @p SerialDriver with timeout specification.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] t         the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 * @return              A byte value from the input queue.
 * @retval MSG_TIMEOUT  if the specified time expired.
 * @retval MSG_RESET    if the @p SerialDriver has been stopped.
 *
 * @api
 */
#define sdGetTimeout(sdp, t) iqGetTimeout(&(sdp)->iqueue, t)

/**
 * @brief   Direct non-blocking write to a @p SerialDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write from different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] b         pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @return              The number of bytes effectively transferred.
 *
 * @iclass
 */
#define sdWriteI(sdp, b, n) oqWriteI(&(sdp)->oqueue, b, n)

/**
 * @brief   Direct blocking write to a @p SerialDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write from different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] b         pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 *
 * @api
 */
#define sdWrite(sdp, b, n) oqWriteTimeout(&(sdp)->oqueue, b, n, TIME_INFINITE)

/**
 * @brief   Direct blocking write to a @p SerialDriver with timeout
 *          specification.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] b         pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @param[in] t         the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 * @return              The number of bytes effectively transferred.
 *
 * @api
 */
#define sdWriteTimeout(sdp, b, n, t)                                        \
  oqWriteTimeout(&(sdp)->oqueue, b, n, t)

/**
 * @brief   Direct non-blocking write to a @p SerialDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] b         pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 *
 * @api
 */
#define sdAsynchronousWrite(sdp, b, n)                                      \
  oqWriteTimeout(&(sdp)->oqueue, b, n, TIME_IMMEDIATE)

/**
 * @brief   Direct non-blocking read from a @p SerialDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] b         pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @return              The number of bytes effectively transferred.
 *
 * @iclass
 */
#define sdReadI(sdp, b, n) iqReadI(&(sdp)->iqueue, b, n)

/**
 * @brief   Direct blocking read from a @p SerialDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] b         pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 *
 * @api
 */
#define sdRead(sdp, b, n) iqReadTimeout(&(sdp)->iqueue, b, n, TIME_INFINITE)

/**
 * @brief   Direct blocking read from a @p SerialDriver with timeout
 *          specification.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] b         pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @param[in] t         the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 * @return              The number of bytes effectively transferred.
 *
 * @api
 */
#define sdReadTimeout(sdp, b, n, t) iqReadTimeout(&(sdp)->iqueue, b, n, t)

/**
 * @brief   Direct non-blocking read from a @p SerialDriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] b         pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @return              The number of bytes effectively transferred.
 *
 * @api
 */
#define sdAsynchronousRead(sdp, b, n)                                       \
  iqReadTimeout(&(sdp)->iqueue, b, n, TIME_IMMEDIATE)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void sdInit(void);
#if !defined(SERIAL_ADVANCED_BUFFERING_SUPPORT) ||                          \
    (SERIAL_ADVANCED_BUFFERING_SUPPORT == FALSE)
  void sdObjectInit(SerialDriver *sdp, qnotify_t inotify, qnotify_t onotify);
#else
  void sdObjectInit(SerialDriver *sdp);
#endif
  msg_t sdStart(SerialDriver *sdp, const SerialConfig *config);
  void sdStop(SerialDriver *sdp);
  void sdIncomingDataI(SerialDriver *sdp, uint8_t b);
  msg_t sdRequestDataI(SerialDriver *sdp);
  bool sdPutWouldBlock(SerialDriver *sdp);
  bool sdGetWouldBlock(SerialDriver *sdp);
  msg_t sdControl(SerialDriver *sdp, unsigned int operation, void *arg);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL == TRUE */

#endif /* HAL_SERIAL_H */

/** @} */
