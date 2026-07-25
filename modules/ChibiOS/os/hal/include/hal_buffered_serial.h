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
 * @file    hal_buffered_serial.h
 * @brief   Buffered Serial Driver macros and structures.
 *
 * @addtogroup HAL_BUFFERED_SERIAL
 * @{
 */

#ifndef HAL_BUFFERED_SERIAL_H
#define HAL_BUFFERED_SERIAL_H

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
 * @brief   @p BufferedSerial state machine states.
 */
typedef enum {
  BS_UNINIT = 0,                    /**< Not initialized.                   */
  BS_STOP = 1,                      /**< Stopped.                           */
  BS_READY = 2                      /**< Ready.                             */
} bsstate_t;

/**
 * @brief   Structure representing a buffered serial class.
 */
typedef struct hal_buffered_serial BufferedSerial;

/**
 * @brief   @p BufferedSerial specific methods.
 */
#define __buffered_serial_methods                                           \
  _base_asynchronous_channel_methods

/**
 * @extends BaseAsynchronousChannelVMT
 *
 * @brief   @p BufferedSerial virtual methods table.
 */
struct BufferedSerialVMT {
  __buffered_serial_methods
};

/**
 * @brief   @p BufferedSerial specific data.
 */
#define __buffered_serial_data                                              \
  _base_asynchronous_channel_data                                           \
  /* Driver state.*/                                                        \
  bsstate_t                 state;                                          \
  /* Input queue.*/                                                         \
  input_queue_t             iqueue;                                         \
  /* Output queue.*/                                                        \
  output_queue_t            oqueue;

/**
 * @extends BaseAsynchronousChannel
 *
 * @brief   Buffered serial channel class.
 * @details This class extends @p BaseAsynchronousChannel by adding physical
 *          I/O queues.
 */
struct hal_buffered_serial {
  /** @brief Virtual Methods Table.*/
  const struct BufferedSerialVMT *vmt;
  __buffered_serial_data
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void bsIncomingDataI(BufferedSerial *bsp, uint8_t b);
  msg_t bsRequestDataI(BufferedSerial *bsp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Inline functions.                                                         */
/*===========================================================================*/

/**
 * @name    Methods implementations
 * @{
 */
/**
 * @brief   Object initialization implementation.
 *
 * @param[in] ip        Pointer to a @p BufferedSerial structure to be
 *                      initialized.
 * @param[in] vmt       VMT pointer for the new object.
 * @param[in] ib        pointer to the input buffer
 * @param[in] ibsize    size of the input buffer
 * @param[in] inotify   pointer to a callback function that is invoked when
 *                      some data is read from the input queue. The value
 *                      can be @p NULL.
 * @param[in] iarg      parameter for the input notification callback
 * @param[in] ob        pointer to the output buffer
 * @param[in] obsize    size of the output buffer
 * @param[in] onotify   pointer to a callback function that is invoked when
 *                      some data is written in the output queue. The value
 *                      can be @p NULL.
 * @param[in] oarg      parameter for the output notification callback
 *
 * @init
 */
CC_FORCE_INLINE
static inline void __buffered_serial_objinit_impl(void *ip, const void *vmt,
                                                  uint8_t *ib, size_t ibsize,
                                                  qnotify_t inotify, void *iarg,
                                                  uint8_t *ob, size_t obsize,
                                                  qnotify_t onotify, void *oarg) {
  BufferedSerial *bsp = (BufferedSerial *)ip;

  bsp->vmt = (const struct BufferedSerialVMT *)vmt;  /* TODO use new obj model.*/
  osalEventObjectInit(&bsp->event);  /* TODO super class should do this.*/
  bsp->state = BS_STOP;
  iqObjectInit(&bsp->iqueue, ib, ibsize, inotify, iarg);
  oqObjectInit(&bsp->oqueue, ob, obsize, onotify, oarg);
}

CC_FORCE_INLINE
static inline size_t __buffered_serial_write_impl(void *ip,
                                                  const uint8_t *bp,
                                                  size_t n) {

  return oqWriteTimeout(&((BufferedSerial *)ip)->oqueue, bp,
                        n, TIME_INFINITE);
}

CC_FORCE_INLINE
static inline size_t __buffered_serial_read_impl(void *ip,
                                                 uint8_t *bp,
                                                 size_t n) {

  return iqReadTimeout(&((BufferedSerial *)ip)->iqueue, bp,
                       n, TIME_INFINITE);
}

CC_FORCE_INLINE
static inline msg_t __buffered_serial_put_impl(void *ip,
                                               uint8_t b) {

  return oqPutTimeout(&((BufferedSerial *)ip)->oqueue, b, TIME_INFINITE);
}

CC_FORCE_INLINE
static inline msg_t __buffered_serial_get_impl(void *ip) {

  return iqGetTimeout(&((BufferedSerial *)ip)->iqueue, TIME_INFINITE);
}

CC_FORCE_INLINE
static inline msg_t __buffered_serial_put_timeout_impl(void *ip,
                                                       uint8_t b,
                                                       sysinterval_t timeout) {

  return oqPutTimeout(&((BufferedSerial *)ip)->oqueue, b, timeout);
}

CC_FORCE_INLINE
static inline msg_t __buffered_serial_get_timeout_impl(void *ip,
                                                       sysinterval_t timeout) {

  return iqGetTimeout(&((BufferedSerial *)ip)->iqueue, timeout);
}

CC_FORCE_INLINE
static inline size_t __buffered_serial_write_timeout_impl(void *ip,
                                                          const uint8_t *bp,
                                                          size_t n,
                                                          sysinterval_t timeout) {

  return oqWriteTimeout(&((BufferedSerial *)ip)->oqueue, bp, n, timeout);
}

CC_FORCE_INLINE
static inline size_t __buffered_serial_read_timeout_impl(void *ip,
                                                         uint8_t *bp,
                                                         size_t n,
                                                         sysinterval_t timeout) {

  return iqReadTimeout(&((BufferedSerial *)ip)->iqueue, bp, n, timeout);
}

CC_FORCE_INLINE
static inline msg_t __buffered_serial_ctl_impl(void *ip,
                                               unsigned int operation,
                                               void *arg) {
  BufferedSerial *bsp = (BufferedSerial *)ip;

  osalDbgCheck(bsp != NULL);

  switch (operation) {
  case CHN_CTL_NOP:
    osalDbgCheck(arg == NULL);
    break;
  case CHN_CTL_INVALID:
    return HAL_RET_UNKNOWN_CTL;
  default:
    return HAL_RET_UNKNOWN_CTL;
  }

  return HAL_RET_SUCCESS;
}
/** @} */

#endif /* HAL_BUFFERED_SERIAL_H */

/** @} */
