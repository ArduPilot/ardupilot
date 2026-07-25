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
 * @file    hal_buffered_sio.h
 * @brief   Buffered SIO Driver macros and structures.
 *
 * @addtogroup HAL_BUFFERED_SIO
 * @{
 */

#ifndef HAL_BUFFERED_SIO_H
#define HAL_BUFFERED_SIO_H

#include "hal.h"

#if (HAL_USE_SIO == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   @p BufferedSIODriver specific methods.
 */
#define __buffered_sio_driver_methods                                       \
  __buffered_serial_methods

/**
 * @extends BufferedSerialVMT
 *
 * @brief   @p BufferedSIODriver virtual methods table.
 */
struct BufferedSIODriverVMT {
  __buffered_sio_driver_methods
};

/**
 * @brief   @p SerialDriver specific data.
 */
#define __buffered_sio_driver_data                                          \
  __buffered_serial_data                                                    \
  SIODriver             *siop;

/**
 * @extends BufferedSerial
 *
 * @brief   Buffered SIO driver class.
 */
typedef struct hal_buffered_siol_driver {
  /** @brief Virtual Methods Table.*/
  const struct BufferedSIODriverVMT *vmt;
  __buffered_sio_driver_data
} BufferedSIODriver;

/**
 * @brief   Type of a buffered SIO configuration.
 */
typedef SIOConfig BufferedSIOConfig;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Direct write to a @p BufferedSIODriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly on the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @iclass
 */
#define bsioPutI(bsiop, b) oqPutI(&(bsiop)->oqueue, b)

/**
 * @brief   Direct write to a @p BufferedSIODriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly on the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @api
 */
#define bsioPut(bsiop, b) oqPut(&(bsiop)->oqueue, b)

/**
 * @brief   Direct write to a @p BufferedSIODriver with timeout specification.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly on the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @api
 */
#define bsioPutTimeout(bsiop, b, t) oqPutTimeout(&(bsiop)->oqueue, b, t)

/**
 * @brief   Direct read from a @p BufferedSIODriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @iclass
 */
#define bsioGetI(bsiop) iqGetI(&(bsiop)->iqueue)

/**
 * @brief   Direct read from a @p BufferedSIODriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @api
 */
#define bsioGet(bsiop) iqGet(&(bsiop)->iqueue)

/**
 * @brief   Direct read from a @p BufferedSIODriver with timeout specification.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @api
 */
#define bsioGetTimeout(bsiop, t) iqGetTimeout(&(bsiop)->iqueue, t)

/**
 * @brief   Direct blocking write to a @p BufferedSIODriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write from different channels implementations.
 *
 * @iclass
 */
#define bsioWriteI(bsiop, b, n) oqWriteI(&(bsiop)->oqueue, b, n)

/**
 * @brief   Direct blocking write to a @p BufferedSIODriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write from different channels implementations.
 *
 * @api
 */
#define bsioWrite(bsiop, b, n) oqWriteTimeout(&(bsiop)->oqueue, b, n, TIME_INFINITE)

/**
 * @brief   Direct blocking write to a @p BufferedSIODriver with timeout
 *          specification.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @api
 */
#define bsioWriteTimeout(bsiop, b, n, t)                                        \
  oqWriteTimeout(&(bsiop)->oqueue, b, n, t)

/**
 * @brief   Direct non-blocking write to a @p BufferedSIODriver.
 * @note    This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @api
 */
#define bsioAsynchronousWrite(bsiop, b, n)                                      \
  oqWriteTimeout(&(bsiop)->oqueue, b, n, TIME_IMMEDIATE)

/**
 * @brief   Direct blocking read from a @p BufferedSIODriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @iclass
 */
#define bsioReadI(bsiop, b, n) iqReadI(&(bsiop)->iqueue, b, n)

/**
 * @brief   Direct blocking read from a @p BufferedSIODriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @api
 */
#define bsioRead(bsiop, b, n) iqReadTimeout(&(bsiop)->iqueue, b, n, TIME_INFINITE)

/**
 * @brief   Direct blocking read from a @p BufferedSIODriver with timeout
 *          specification.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @api
 */
#define bsioReadTimeout(bsiop, b, n, t) iqReadTimeout(&(bsiop)->iqueue, b, n, t)

/**
 * @brief   Direct non-blocking read from a @p BufferedSIODriver.
 * @note    This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @api
 */
#define bsioAsynchronousRead(bsiop, b, n)                                       \
  iqReadTimeout(&(bsiop)->iqueue, b, n, TIME_IMMEDIATE)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void bsioObjectInit(BufferedSIODriver *bsiop, SIODriver *siop,
                      uint8_t *ib, size_t ibsize,
                      uint8_t *ob, size_t obsize);
  msg_t bsioStart(BufferedSIODriver *bsiop, const BufferedSIOConfig *config);
  void bsioStop(BufferedSIODriver *bsiop);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SIO == TRUE */

#endif /* HAL_BUFFERED_SIO */

/** @} */
