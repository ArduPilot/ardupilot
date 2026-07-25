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
 * @file    hal_queues.h
 * @brief   I/O Queues macros and structures.
 *
 * @addtogroup HAL_QUEUES
 * @{
 */

#ifndef HAL_QUEUES_H
#define HAL_QUEUES_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Queue functions returned status value
 * @{
 */
#define Q_OK            MSG_OK      /**< @brief Operation successful.       */
#define Q_TIMEOUT       MSG_TIMEOUT /**< @brief Timeout condition.          */
#define Q_RESET         MSG_RESET   /**< @brief Queue has been reset.       */
#define Q_EMPTY         MSG_TIMEOUT /**< @brief Queue empty.                */
#define Q_FULL          MSG_TIMEOUT /**< @brief Queue full.                 */
/** @} */

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
 * @brief   Type of a generic I/O queue structure.
 */
typedef struct io_queue io_queue_t;

/**
 * @brief   Queue notification callback type.
 *
 * @param[in] qp        the queue pointer
 */
typedef void (*qnotify_t)(io_queue_t *qp);

/**
 * @brief   Generic I/O queue structure.
 * @details This structure represents a generic Input or Output asymmetrical
 *          queue. The queue is asymmetrical because one end is meant to be
 *          accessed from a thread context, and thus can be blocking, the other
 *          end is accessible from interrupt handlers or from within a kernel
 *          lock zone and is non-blocking.
 */
struct io_queue {
  threads_queue_t       q_waiting;  /**< @brief Queue of waiting threads.   */
  volatile size_t       q_counter;  /**< @brief Resources counter.          */
  uint8_t               *q_buffer;  /**< @brief Pointer to the queue buffer.*/
  uint8_t               *q_top;     /**< @brief Pointer to the first
                                         location after the buffer.         */
  uint8_t               *q_wrptr;   /**< @brief Write pointer.              */
  uint8_t               *q_rdptr;   /**< @brief Read pointer.               */
  qnotify_t             q_notify;   /**< @brief Data notification callback. */
  void                  *q_link;    /**< @brief Application defined field.  */
};

/**
 * @extends io_queue_t
 *
 * @brief   Type of an input queue structure.
 * @details This structure represents a generic asymmetrical input queue.
 *          Writing to the queue is non-blocking and can be performed from
 *          interrupt handlers or from within a kernel lock zone.
 *          Reading the queue can be a blocking operation and is supposed to
 *          be performed by a system thread.
 */
typedef io_queue_t input_queue_t;

/**
 * @extends io_queue_t
 *
 * @brief   Type of an output queue structure.
 * @details This structure represents a generic asymmetrical output queue.
 *          Reading from the queue is non-blocking and can be performed from
 *          interrupt handlers or from within a kernel lock zone.
 *          Writing the queue can be a blocking operation and is supposed to
 *          be performed by a system thread.
 */
typedef io_queue_t output_queue_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Returns the queue's buffer size.
 *
 * @param[in] qp        pointer to a @p io_queue_t structure
 * @return              The buffer size.
 *
 * @xclass
 */
#define qSizeX(qp)                                                          \
  /*lint -save -e9033 [10.8] The cast is safe.*/                            \
  ((size_t)((qp)->q_top - (qp)->q_buffer))                                  \
  /*lint -restore*/

/**
 * @brief   Queue space.
 * @details Returns the used space if used on an input queue or the empty
 *          space if used on an output queue.
 *
 * @param[in] qp        pointer to a @p io_queue_t structure
 * @return              The buffer space.
 *
 * @iclass
 */
#define qSpaceI(qp) ((qp)->q_counter)

/**
 * @brief   Returns the queue application-defined link.
 * @note    This function can be called in any context.
 *
 * @param[in] qp        pointer to a @p io_queue_t structure
 * @return              The application-defined link.
 *
 * @special
 */
#define qGetLink(qp) ((qp)->q_link)

/**
 * @brief   Sets the queue application-defined link.
 * @note    This function can be called in any context.
 *
 * @param[in] qp        pointer to a @p io_queue_t structure
 * @param[in] lk        The application-defined link.
 *
 * @special
 */
#define qSetLink(qp, lk) ((qp)->q_link = lk)

/**
 * @brief   Returns the filled space into an input queue.
 *
 * @param[in] iqp       pointer to an @p input_queue_t structure
 * @return              The number of full bytes in the queue.
 * @retval 0            if the queue is empty.
 *
 * @iclass
 */
#define iqGetFullI(iqp) qSpaceI(iqp)

/**
 * @brief   Returns the empty space into an input queue.
 *
 * @param[in] iqp       pointer to an @p input_queue_t structure
 * @return              The number of empty bytes in the queue.
 * @retval 0            if the queue is full.
 *
 * @iclass
 */
#define iqGetEmptyI(iqp) (qSizeX(iqp) - qSpaceI(iqp))

/**
 * @brief   Evaluates to @p true if the specified input queue is empty.
 *
 * @param[in] iqp       pointer to an @p input_queue_t structure
 * @return              The queue status.
 * @retval false        if the queue is not empty.
 * @retval true         if the queue is empty.
 *
 * @iclass
 */
#define iqIsEmptyI(iqp) ((bool)(qSpaceI(iqp) == 0U))

/**
 * @brief   Evaluates to @p true if the specified input queue is full.
 *
 * @param[in] iqp       pointer to an @p input_queue_t structure
 * @return              The queue status.
 * @retval false        if the queue is not full.
 * @retval true         if the queue is full.
 *
 * @iclass
 */
#define iqIsFullI(iqp)                                                      \
  /*lint -save -e9007 [13.5] No side effects, a pointer is passed.*/        \
  ((bool)(((iqp)->q_wrptr == (iqp)->q_rdptr) && ((iqp)->q_counter != 0U)))  \
  /*lint -restore*/

/**
 * @brief   Input queue read.
 * @details This function reads a byte value from an input queue. If the queue
 *          is empty then the calling thread is suspended until a byte arrives
 *          in the queue.
 *
 * @param[in] iqp       pointer to an @p input_queue_t structure
 * @return              A byte value from the queue.
 * @retval MSG_RESET    if the queue has been reset.
 *
 * @api
 */
#define iqGet(iqp) iqGetTimeout(iqp, TIME_INFINITE)

/**
 * @brief   Returns the filled space into an output queue.
 *
 * @param[in] oqp       pointer to an @p output_queue_t structure
 * @return              The number of full bytes in the queue.
 * @retval 0            if the queue is empty.
 *
 * @iclass
 */
#define oqGetFullI(oqp) (qSizeX(oqp) - qSpaceI(oqp))

/**
 * @brief   Returns the empty space into an output queue.
 *
 * @param[in] oqp       pointer to an @p output_queue_t structure
 * @return              The number of empty bytes in the queue.
 * @retval 0            if the queue is full.
 *
 * @iclass
 */
#define oqGetEmptyI(oqp) qSpaceI(oqp)

/**
 * @brief   Evaluates to @p true if the specified output queue is empty.
 *
 * @param[in] oqp       pointer to an @p output_queue_t structure
 * @return              The queue status.
 * @retval false        if the queue is not empty.
 * @retval true         if the queue is empty.
 *
 * @iclass
 */
#define oqIsEmptyI(oqp)                                                     \
  /*lint -save -e9007 [13.5] No side effects, a pointer is passed.*/        \
  ((bool)(((oqp)->q_wrptr == (oqp)->q_rdptr) && ((oqp)->q_counter != 0U)))  \
  /*lint -restore*/

/**
 * @brief   Evaluates to @p true if the specified output queue is full.
 *
 * @param[in] oqp       pointer to an @p output_queue_t structure
 * @return              The queue status.
 * @retval false        if the queue is not full.
 * @retval true         if the queue is full.
 *
 * @iclass
 */
#define oqIsFullI(oqp) ((bool)(qSpaceI(oqp) == 0U))

/**
 * @brief   Output queue write.
 * @details This function writes a byte value to an output queue. If the queue
 *          is full then the calling thread is suspended until there is space
 *          in the queue.
 *
 * @param[in] oqp       pointer to an @p output_queue_t structure
 * @param[in] b         the byte value to be written in the queue
 * @return              The operation status.
 * @retval MSG_OK       if the operation succeeded.
 * @retval MSG_RESET    if the queue has been reset.
 *
 * @api
 */
#define oqPut(oqp, b) oqPutTimeout(oqp, b, TIME_INFINITE)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void iqObjectInit(input_queue_t *iqp, uint8_t *bp, size_t size,
                    qnotify_t infy, void *link);
  void iqResetI(input_queue_t *iqp);
  msg_t iqPutI(input_queue_t *iqp, uint8_t b);
  msg_t iqGetI(input_queue_t *iqp);
  msg_t iqGetTimeout(input_queue_t *iqp, sysinterval_t timeout);
  size_t iqReadI(input_queue_t *iqp, uint8_t *bp, size_t n);
  size_t iqReadTimeout(input_queue_t *iqp, uint8_t *bp,
                       size_t n, sysinterval_t timeout);

  void oqObjectInit(output_queue_t *oqp, uint8_t *bp, size_t size,
                    qnotify_t onfy, void *link);
  void oqResetI(output_queue_t *oqp);
  msg_t oqPutI(output_queue_t *oqp, uint8_t b);
  msg_t oqPutTimeout(output_queue_t *oqp, uint8_t b, sysinterval_t timeout);
  msg_t oqGetI(output_queue_t *oqp);
  size_t oqWriteI(output_queue_t *oqp, const uint8_t *bp, size_t n);
  size_t oqWriteTimeout(output_queue_t *oqp, const uint8_t *bp,
                        size_t n, sysinterval_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* HAL_QUEUES_H */

/** @} */
