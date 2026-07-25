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
 * @file    hal_buffers.h
 * @brief   I/O Buffers macros and structures.
 *
 * @addtogroup HAL_BUFFERS
 * @{
 */

#ifndef HAL_BUFFERS_H
#define HAL_BUFFERS_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Maximum size of blocks copied in critical sections.
 * @note    Increasing this value increases performance at expense of
 *          IRQ servicing efficiency.
 * @note    It must be a power of two.
 */
#if !defined(BUFFERS_CHUNKS_SIZE) || defined(__DOXYGEN__)
#define BUFFERS_CHUNKS_SIZE                 64
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*lint -save -e9027 [10.1] It is meant to be this way, not an error.*/
#if (BUFFERS_CHUNKS_SIZE & (BUFFERS_CHUNKS_SIZE - 1)) != 0
/*lint -restore*/
#error "BUFFERS_CHUNKS_SIZE must be a power of two"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a generic queue of buffers.
 */
typedef struct io_buffers_queue io_buffers_queue_t;

/**
 * @brief   Double buffer notification callback type.
 *
 * @param[in] bqp       the buffers queue pointer
 */
typedef void (*bqnotify_t)(io_buffers_queue_t *bqp);

/**
 * @brief   Structure of a generic buffers queue.
 */
struct io_buffers_queue {
  /**
   * @brief   Queue of waiting threads.
   */
  threads_queue_t       waiting;
  /**
   * @brief   Queue suspended state flag.
   */
  bool                  suspended;
  /**
   * @brief   Active buffers counter.
   */
  volatile size_t       bcounter;
  /**
   * @brief   Buffer write pointer.
   */
  uint8_t               *bwrptr;
  /**
   * @brief   Buffer read pointer.
   */
  uint8_t               *brdptr;
  /**
   * @brief   Pointer to the buffers boundary.
   */
  uint8_t               *btop;
  /**
   * @brief   Size of buffers.
   * @note    The buffer size must be not lower than <tt>sizeof(size_t) + 2</tt>
   *          because the first bytes are used to store the used size of the
   *          buffer.
   */
  size_t                bsize;
  /**
   * @brief   Number of buffers.
   */
  size_t                bn;
  /**
   * @brief   Queue of buffer objects.
   */
  uint8_t               *buffers;
  /**
   * @brief   Pointer for R/W sequential access.
   * @note    It is @p NULL if a new buffer must be fetched from the queue.
   */
  uint8_t               *ptr;
  /**
   * @brief   Boundary for R/W sequential access.
   */
  uint8_t               *top;
  /**
   * @brief   Data notification callback.
   */
  bqnotify_t            notify;
  /**
   * @brief   Application defined field.
   */
  void                  *link;
};

/**
 * @brief   Type of an input buffers queue.
 */
typedef io_buffers_queue_t input_buffers_queue_t;

/**
 * @brief   Type of an output buffers queue.
 */
typedef io_buffers_queue_t output_buffers_queue_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Computes the size of a buffers queue buffer size.
 *
 * @param[in] n         number of buffers in the queue
 * @param[in] size      size of the buffers
 */
#define BQ_BUFFER_SIZE(n, size)                                             \
  (((size_t)(size) + sizeof (size_t)) * (size_t)(n))

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Returns the queue's number of buffers.
 *
 * @param[in] bqp       pointer to an @p io_buffers_queue_t structure
 * @return              The number of buffers.
 *
 * @xclass
 */
#define bqSizeX(bqp) ((bqp)->bn)

/**
 * @brief   Return the ready buffers number.
 * @details Returns the number of filled buffers if used on an input queue
 *          or the number of empty buffers if used on an output queue.
 *
 * @param[in] bqp       pointer to an @p io_buffers_queue_t structure
 * @return              The number of ready buffers.
 *
 * @iclass
 */
#define bqSpaceI(bqp) ((bqp)->bcounter)

/**
 * @brief   Returns the queue application-defined link.
 *
 * @param[in] bqp       pointer to an @p io_buffers_queue_t structure
 * @return              The application-defined link.
 *
 * @special
 */
#define bqGetLinkX(bqp) ((bqp)->link)

/**
 * @brief   Sets the queue application-defined link.
 *
 * @param[in] bqp       pointer to an @p io_buffers_queue_t structure
 * @param[in] lk        The application-defined link.
 *
 * @special
 */
#define bqSetLinkX(bqp, lk) ((bqp)->link = lk)

/**
 * @brief   Return the suspended state of the queue.
 *
 * @param[in] bqp       pointer to an @p io_buffers_queue_t structure
 * @return              The suspended state.
 * @retval false        if blocking access to the queue is enabled.
 * @retval true         if blocking access to the queue is suspended.
 *
 * @xclass
 */
#define bqIsSuspendedX(bqp) ((bqp)->suspended)

/**
 * @brief   Puts the queue in suspended state.
 * @details When the queue is put in suspended state all waiting threads are
 *          woken with message @p MSG_RESET and subsequent attempt at waiting
 *          on the queue will result in an immediate return with @p MSG_RESET
 *          message.
 * @note    The content of the queue is not altered, queues can be accessed
 *          is suspended state until a blocking operation is met then a
 *          @p MSG_RESET occurs.
 *
 * @param[in] bqp       pointer to an @p io_buffers_queue_t structure
 *
 * @iclass
 */
#define bqSuspendI(bqp) {                                                   \
  (bqp)->suspended = true;                                                  \
  osalThreadDequeueAllI(&(bqp)->waiting, MSG_RESET);                        \
}

/**
 * @brief   Resumes normal queue operations.
 *
 * @param[in] bqp       pointer to an @p io_buffers_queue_t structure
 *
 * @xclass
 */
#define bqResumeX(bqp) {                                                    \
  (bqp)->suspended = false;                                                 \
}

/**
 * @brief   Evaluates to @p true if the specified input buffers queue is empty.
 *
 * @param[in] ibqp      pointer to an @p input_buffers_queue_t structure
 * @return              The queue status.
 * @retval false        if the queue is not empty.
 * @retval true         if the queue is empty.
 *
 * @iclass
 */
#define ibqIsEmptyI(ibqp) ((bool)(bqSpaceI(ibqp) == 0U))

/**
 * @brief   Evaluates to @p true if the specified input buffers queue is full.
 *
 * @param[in] ibqp      pointer to an @p input_buffers_queue_t structure
 * @return              The queue status.
 * @retval false        if the queue is not full.
 * @retval true         if the queue is full.
 *
 * @iclass
 */
#define ibqIsFullI(ibqp)                                                    \
  /*lint -save -e9007 [13.5] No side effects, a pointer is passed.*/        \
  ((bool)(((ibqp)->bwrptr == (ibqp)->brdptr) && ((ibqp)->bcounter != 0U)))  \
  /*lint -restore*/

/**
 * @brief   Evaluates to @p true if the specified output buffers queue is empty.
 *
 * @param[in] obqp      pointer to an @p output_buffers_queue_t structure
 * @return              The queue status.
 * @retval false        if the queue is not empty.
 * @retval true         if the queue is empty.
 *
 * @iclass
 */
#define obqIsEmptyI(obqp)                                                   \
  /*lint -save -e9007 [13.5] No side effects, a pointer is passed.*/        \
  ((bool)(((obqp)->bwrptr == (obqp)->brdptr) && ((obqp)->bcounter != 0U)))  \
  /*lint -restore*/

/**
 * @brief   Evaluates to @p true if the specified output buffers queue is full.
 *
 * @param[in] obqp      pointer to an @p output_buffers_queue_t structure
 * @return              The queue status.
 * @retval false        if the queue is not full.
 * @retval true         if the queue is full.
 *
 * @iclass
 */
#define obqIsFullI(obqp) ((bool)(bqSpaceI(obqp) == 0U))
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void ibqObjectInit(input_buffers_queue_t *ibqp, bool suspended, uint8_t *bp,
                     size_t size, size_t n, bqnotify_t infy, void *link);
  void ibqResetI(input_buffers_queue_t *ibqp);
  uint8_t *ibqGetEmptyBufferI(input_buffers_queue_t *ibqp);
  void ibqPostFullBufferI(input_buffers_queue_t *ibqp, size_t size);
  msg_t ibqGetFullBufferTimeout(input_buffers_queue_t *ibqp,
                                sysinterval_t timeout);
  msg_t ibqGetFullBufferTimeoutS(input_buffers_queue_t *ibqp,
                                 sysinterval_t timeout);
  void ibqReleaseEmptyBuffer(input_buffers_queue_t *ibqp);
  void ibqReleaseEmptyBufferS(input_buffers_queue_t *ibqp);
  msg_t ibqGetTimeout(input_buffers_queue_t *ibqp, sysinterval_t timeout);
  size_t ibqReadTimeout(input_buffers_queue_t *ibqp, uint8_t *bp,
                        size_t n, sysinterval_t timeout);
  void obqObjectInit(output_buffers_queue_t *obqp, bool suspended, uint8_t *bp,
                     size_t size, size_t n, bqnotify_t onfy, void *link);
  void obqResetI(output_buffers_queue_t *obqp);
  uint8_t *obqGetFullBufferI(output_buffers_queue_t *obqp,
                             size_t *sizep);
  void obqReleaseEmptyBufferI(output_buffers_queue_t *obqp);
  msg_t obqGetEmptyBufferTimeout(output_buffers_queue_t *obqp,
                                 sysinterval_t timeout);
  msg_t obqGetEmptyBufferTimeoutS(output_buffers_queue_t *obqp,
                                  sysinterval_t timeout);
  void obqPostFullBuffer(output_buffers_queue_t *obqp, size_t size);
  void obqPostFullBufferS(output_buffers_queue_t *obqp, size_t size);
  msg_t obqPutTimeout(output_buffers_queue_t *obqp, uint8_t b,
                      sysinterval_t timeout);
  size_t obqWriteTimeout(output_buffers_queue_t *obqp, const uint8_t *bp,
                         size_t n, sysinterval_t timeout);
  bool obqTryFlushI(output_buffers_queue_t *obqp);
  void obqFlush(output_buffers_queue_t *obqp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_BUFFERS_H */

/** @} */
