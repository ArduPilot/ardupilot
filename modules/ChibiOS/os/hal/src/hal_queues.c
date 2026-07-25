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
 * @file    hal_queues.c
 * @brief   I/O Queues code.
 *
 * @addtogroup HAL_QUEUES
 * @details Queues are mostly used in serial-like device drivers.
 *          Serial device drivers are usually designed to have a lower side
 *          (lower driver, it is usually an interrupt service routine) and an
 *          upper side (upper driver, accessed by the application threads).<br>
 *          There are several kinds of queues:<br>
 *          - <b>Input queue</b>, unidirectional queue where the writer is the
 *            lower side and the reader is the upper side.
 *          - <b>Output queue</b>, unidirectional queue where the writer is the
 *            upper side and the reader is the lower side.
 *          - <b>Full duplex queue</b>, bidirectional queue. Full duplex queues
 *            are implemented by pairing an input queue and an output queue
 *            together.
 *          .
 * @{
 */

#include <string.h>

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Non-blocking input queue read.
 * @details The function reads data from an input queue into a buffer. The
 *          operation completes when the specified amount of data has been
 *          transferred or when the input queue has been emptied.
 *
 * @param[in] iqp       pointer to an @p input_queue_t structure
 * @param[out] bp       pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @return              The number of bytes effectively transferred.
 *
 * @notapi
 */
static size_t iq_read(input_queue_t *iqp, uint8_t *bp, size_t n) {
  size_t s1, s2;

  osalDbgCheck(n > 0U);

  /* Number of bytes that can be read in a single atomic operation.*/
  if (n > iqGetFullI(iqp)) {
    n = iqGetFullI(iqp);
  }

  /* Number of bytes before buffer limit.*/
  /*lint -save -e9033 [10.8] Checked to be safe.*/
  s1 = (size_t)(iqp->q_top - iqp->q_rdptr);
  /*lint -restore*/
  if (n < s1) {
    memcpy((void *)bp, (void *)iqp->q_rdptr, n);
    iqp->q_rdptr += n;
  }
  else if (n > s1) {
    memcpy((void *)bp, (void *)iqp->q_rdptr, s1);
    bp += s1;
    s2 = n - s1;
    memcpy((void *)bp, (void *)iqp->q_buffer, s2);
    iqp->q_rdptr = iqp->q_buffer + s2;
  }
  else {
    memcpy((void *)bp, (void *)iqp->q_rdptr, n);
    iqp->q_rdptr = iqp->q_buffer;
  }

  iqp->q_counter -= n;
  return n;
}

/**
 * @brief   Non-blocking output queue write.
 * @details The function writes data from a buffer to an output queue. The
 *          operation completes when the specified amount of data has been
 *          transferred or when the output queue has been filled.
 *
 * @param[in] oqp       pointer to an @p output_queue_t structure
 * @param[in] bp        pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @return              The number of bytes effectively transferred.
 *
 * @notapi
 */
static size_t oq_write(output_queue_t *oqp, const uint8_t *bp, size_t n) {
  size_t s1, s2;

  osalDbgCheck(n > 0U);

  /* Number of bytes that can be written in a single atomic operation.*/
  if (n > oqGetEmptyI(oqp)) {
    n = oqGetEmptyI(oqp);
  }

  /* Number of bytes before buffer limit.*/
  /*lint -save -e9033 [10.8] Checked to be safe.*/
  s1 = (size_t)(oqp->q_top - oqp->q_wrptr);
  /*lint -restore*/
  if (n < s1) {
    memcpy((void *)oqp->q_wrptr, (const void *)bp, n);
    oqp->q_wrptr += n;
  }
  else if (n > s1) {
    memcpy((void *)oqp->q_wrptr, (const void *)bp, s1);
    bp += s1;
    s2 = n - s1;
    memcpy((void *)oqp->q_buffer, (const void *)bp, s2);
    oqp->q_wrptr = oqp->q_buffer + s2;
  }
  else {
    memcpy((void *)oqp->q_wrptr, (const void *)bp, n);
    oqp->q_wrptr = oqp->q_buffer;
  }

  oqp->q_counter -= n;
  return n;
}

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes an input queue.
 * @details A Semaphore is internally initialized and works as a counter of
 *          the bytes contained in the queue.
 * @note    The callback is invoked from within the S-Locked system state.
 *
 * @param[out] iqp      pointer to an @p input_queue_t structure
 * @param[in] bp        pointer to a memory area allocated as queue buffer
 * @param[in] size      size of the queue buffer
 * @param[in] infy      pointer to a callback function that is invoked when
 *                      data is read from the queue. The value can be @p NULL.
 * @param[in] link      application defined pointer
 *
 * @init
 */
void iqObjectInit(input_queue_t *iqp, uint8_t *bp, size_t size,
                  qnotify_t infy, void *link) {

  osalThreadQueueObjectInit(&iqp->q_waiting);
  iqp->q_counter = 0;
  iqp->q_buffer  = bp;
  iqp->q_rdptr   = bp;
  iqp->q_wrptr   = bp;
  iqp->q_top     = bp + size;
  iqp->q_notify  = infy;
  iqp->q_link    = link;
}

/**
 * @brief   Resets an input queue.
 * @details All the data in the input queue is erased and lost, any waiting
 *          thread is resumed with status @p MSG_RESET.
 * @note    A reset operation can be used by a low level driver in order to
 *          obtain immediate attention from the high level layers.
 *
 * @param[in] iqp       pointer to an @p input_queue_t structure
 *
 * @iclass
 */
void iqResetI(input_queue_t *iqp) {

  osalDbgCheckClassI();

  iqp->q_rdptr = iqp->q_buffer;
  iqp->q_wrptr = iqp->q_buffer;
  iqp->q_counter = 0;
  osalThreadDequeueAllI(&iqp->q_waiting, MSG_RESET);
}

/**
 * @brief   Input queue write.
 * @details A byte value is written into the low end of an input queue. The
 *          operation completes immediately.
 *
 * @param[in] iqp       pointer to an @p input_queue_t structure
 * @param[in] b         the byte value to be written in the queue
 * @return              The operation status.
 * @retval MSG_OK       if the operation has been completed with success.
 * @retval MSG_TIMEOUT  if the queue is full.
 *
 * @iclass
 */
msg_t iqPutI(input_queue_t *iqp, uint8_t b) {

  osalDbgCheckClassI();

  /* Queue space check.*/
  if (!iqIsFullI(iqp)) {
    iqp->q_counter++;
    *iqp->q_wrptr++ = b;
    if (iqp->q_wrptr >= iqp->q_top) {
      iqp->q_wrptr = iqp->q_buffer;
    }

    osalThreadDequeueNextI(&iqp->q_waiting, MSG_OK);

    return MSG_OK;
  }

  return MSG_TIMEOUT;
}

/**
 * @brief   Input queue non-blocking read.
 * @details This function reads a byte value from an input queue. The
 *          operation completes immediately.
 * @note    The callback is invoked after removing a character from the
 *          queue.
 *
 * @param[in] iqp       pointer to an @p input_queue_t structure
 * @return              A byte value from the queue.
 * @retval MSG_TIMEOUT  if the queue is empty.
 *
 * @iclass
 */
msg_t iqGetI(input_queue_t *iqp) {

  osalDbgCheckClassI();

  /* Queue data check.*/
  if (!iqIsEmptyI(iqp)) {
    uint8_t b;

    /* Getting the character from the queue.*/
    iqp->q_counter--;
    b = *iqp->q_rdptr++;
    if (iqp->q_rdptr >= iqp->q_top) {
      iqp->q_rdptr = iqp->q_buffer;
    }

    /* Inform the low side that the queue has at least one slot available.*/
    if (iqp->q_notify != NULL) {
      iqp->q_notify(iqp);
    }

    return (msg_t)b;
  }

  return MSG_TIMEOUT;
}

/**
 * @brief   Input queue read with timeout.
 * @details This function reads a byte value from an input queue. If the queue
 *          is empty then the calling thread is suspended until a byte arrives
 *          in the queue or a timeout occurs.
 * @note    The callback is invoked after removing a character from the
 *          queue.
 *
 * @param[in] iqp       pointer to an @p input_queue_t structure
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 * @return              A byte value from the queue.
 * @retval MSG_TIMEOUT  if the specified time expired.
 * @retval MSG_RESET    if the queue has been reset.
 *
 * @api
 */
msg_t iqGetTimeout(input_queue_t *iqp, sysinterval_t timeout) {
  uint8_t b;

  osalSysLock();

  /* Waiting until there is a character available or a timeout occurs.*/
  while (iqIsEmptyI(iqp)) {
    msg_t msg = osalThreadEnqueueTimeoutS(&iqp->q_waiting, timeout);
    if (msg < MSG_OK) {
      osalSysUnlock();
      return msg;
    }
  }

  /* Getting the character from the queue.*/
  iqp->q_counter--;
  b = *iqp->q_rdptr++;
  if (iqp->q_rdptr >= iqp->q_top) {
    iqp->q_rdptr = iqp->q_buffer;
  }

  /* Inform the low side that the queue has at least one slot available.*/
  if (iqp->q_notify != NULL) {
    iqp->q_notify(iqp);
  }

  osalSysUnlock();

  return (msg_t)b;
}

/**
 * @brief   Input queue non-blocking read.
 * @details The function reads data from an input queue into a buffer. The
 *          operation completes immediately.
 *
 * @param[in] iqp       pointer to an @p input_queue_t structure
 * @param[out] bp       pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @return              The number of bytes effectively transferred.
 *
 * @iclass
 */
size_t iqReadI(input_queue_t *iqp, uint8_t *bp, size_t n) {
  qnotify_t nfy = iqp->q_notify;
  size_t rd;

  osalDbgCheckClassI();

  rd = iq_read(iqp, bp, n);

  /* Inform the low side that the queue has at least one character
     available.*/
  if ((rd > (size_t)0) && (nfy != NULL)) {
    nfy(iqp);
  }

  return rd;
}

/**
 * @brief   Input queue read with timeout.
 * @details The function reads data from an input queue into a buffer. The
 *          operation completes when the specified amount of data has been
 *          transferred or after the specified timeout or if the queue has
 *          been reset.
 * @note    The function is not atomic, if you need atomicity it is suggested
 *          to use a semaphore or a mutex for mutual exclusion.
 * @note    The callback is invoked after removing each character from the
 *          queue.
 *
 * @param[in] iqp       pointer to an @p input_queue_t structure
 * @param[out] bp       pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 * @return              The number of bytes effectively transferred.
 *
 * @api
 */
size_t iqReadTimeout(input_queue_t *iqp, uint8_t *bp,
                     size_t n, sysinterval_t timeout) {
  qnotify_t nfy = iqp->q_notify;
  size_t max = n;

  osalDbgCheck(n > 0U);

  osalSysLock();

  while (n > 0U) {
    size_t done;

    done = iq_read(iqp, bp, n);
    if (done == (size_t)0) {
      msg_t msg = osalThreadEnqueueTimeoutS(&iqp->q_waiting, timeout);

      /* Anything except MSG_OK causes the operation to stop.*/
      if (msg != MSG_OK) {
        break;
      }
    }
    else {
      /* Inform the low side that the queue has at least one empty slot
         available.*/
      if (nfy != NULL) {
        nfy(iqp);
      }

      /* Giving a preemption chance in a controlled point.*/
      osalSysUnlock();

      n  -= done;
      bp += done;

      osalSysLock();
    }
  }

  osalSysUnlock();
  return max - n;
}

/**
 * @brief   Initializes an output queue.
 * @details A Semaphore is internally initialized and works as a counter of
 *          the free bytes in the queue.
 * @note    The callback is invoked from within the S-Locked system state.
 *
 * @param[out] oqp      pointer to an @p output_queue_t structure
 * @param[in] bp        pointer to a memory area allocated as queue buffer
 * @param[in] size      size of the queue buffer
 * @param[in] onfy      pointer to a callback function that is invoked when
 *                      data is written to the queue. The value can be @p NULL.
 * @param[in] link      application defined pointer
 *
 * @init
 */
void oqObjectInit(output_queue_t *oqp, uint8_t *bp, size_t size,
                  qnotify_t onfy, void *link) {

  osalThreadQueueObjectInit(&oqp->q_waiting);
  oqp->q_counter = size;
  oqp->q_buffer  = bp;
  oqp->q_rdptr   = bp;
  oqp->q_wrptr   = bp;
  oqp->q_top     = bp + size;
  oqp->q_notify  = onfy;
  oqp->q_link    = link;
}

/**
 * @brief   Resets an output queue.
 * @details All the data in the output queue is erased and lost, any waiting
 *          thread is resumed with status @p MSG_RESET.
 * @note    A reset operation can be used by a low level driver in order to
 *          obtain immediate attention from the high level layers.
 *
 * @param[in] oqp       pointer to an @p output_queue_t structure
 *
 * @iclass
 */
void oqResetI(output_queue_t *oqp) {

  osalDbgCheckClassI();

  oqp->q_rdptr = oqp->q_buffer;
  oqp->q_wrptr = oqp->q_buffer;
  oqp->q_counter = qSizeX(oqp);
  osalThreadDequeueAllI(&oqp->q_waiting, MSG_RESET);
}

/**
 * @brief   Output queue non-blocking write.
 * @details This function writes a byte value to an output queue. The
 *          operation completes immediately.
 *
 * @param[in] oqp       pointer to an @p output_queue_t structure
 * @param[in] b         the byte value to be written in the queue
 * @return              The operation status.
 * @retval MSG_OK       if the operation succeeded.
 * @retval MSG_TIMEOUT  if the queue is full.
 *
 * @iclass
 */
msg_t oqPutI(output_queue_t *oqp, uint8_t b) {

  osalDbgCheckClassI();

  /* Queue space check.*/
  while (!oqIsFullI(oqp)) {
    /* Putting the character into the queue.*/
    oqp->q_counter--;
    *oqp->q_wrptr++ = b;
    if (oqp->q_wrptr >= oqp->q_top) {
      oqp->q_wrptr = oqp->q_buffer;
    }

    /* Inform the low side that the queue has at least one character available.*/
    if (oqp->q_notify != NULL) {
      oqp->q_notify(oqp);
    }

    return MSG_OK;
  }

  return MSG_TIMEOUT;
}

/**
 * @brief   Output queue write with timeout.
 * @details This function writes a byte value to an output queue. If the queue
 *          is full then the calling thread is suspended until there is space
 *          in the queue or a timeout occurs.
 * @note    The callback is invoked after putting the character into the
 *          queue.
 *
 * @param[in] oqp       pointer to an @p output_queue_t structure
 * @param[in] b         the byte value to be written in the queue
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 * @return              The operation status.
 * @retval MSG_OK       if the operation succeeded.
 * @retval MSG_TIMEOUT  if the specified time expired.
 * @retval MSG_RESET    if the queue has been reset.
 *
 * @api
 */
msg_t oqPutTimeout(output_queue_t *oqp, uint8_t b, sysinterval_t timeout) {

  osalSysLock();

  /* Waiting until there is a slot available or a timeout occurs.*/
  while (oqIsFullI(oqp)) {
    msg_t msg = osalThreadEnqueueTimeoutS(&oqp->q_waiting, timeout);
    if (msg < MSG_OK) {
      osalSysUnlock();
      return msg;
    }
  }

  /* Putting the character into the queue.*/
  oqp->q_counter--;
  *oqp->q_wrptr++ = b;
  if (oqp->q_wrptr >= oqp->q_top) {
    oqp->q_wrptr = oqp->q_buffer;
  }

  /* Inform the low side that the queue has at least one character available.*/
  if (oqp->q_notify != NULL) {
    oqp->q_notify(oqp);
  }

  osalSysUnlock();

  return MSG_OK;
}

/**
 * @brief   Output queue read.
 * @details A byte value is read from the low end of an output queue. The
 *          operation completes immediately.
 *
 * @param[in] oqp       pointer to an @p output_queue_t structure
 * @return              The byte value from the queue.
 * @retval MSG_TIMEOUT  if the queue is empty.
 *
 * @iclass
 */
msg_t oqGetI(output_queue_t *oqp) {

  osalDbgCheckClassI();

  /* Queue data check.*/
  if (!oqIsEmptyI(oqp)) {
    uint8_t b;

    oqp->q_counter++;
    b = *oqp->q_rdptr++;
    if (oqp->q_rdptr >= oqp->q_top) {
      oqp->q_rdptr = oqp->q_buffer;
    }

    osalThreadDequeueNextI(&oqp->q_waiting, MSG_OK);

    return (msg_t)b;
  }

  return MSG_TIMEOUT;
}

/**
 * @brief   Output queue non-blocking write.
 * @details The function writes data from a buffer to an output queue. The
 *          operation completes immediately.
 *
 * @param[in] oqp       pointer to an @p output_queue_t structure
 * @param[in] bp        pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @return              The number of bytes effectively transferred.
 *
 * @iclass
 */
size_t oqWriteI(output_queue_t *oqp, const uint8_t *bp, size_t n) {
  qnotify_t nfy = oqp->q_notify;
  size_t wr;

  osalDbgCheckClassI();

  wr = oq_write(oqp, bp, n);

  /* Inform the low side that the queue has at least one character
     available.*/
  if ((wr > (size_t)0) && (nfy != NULL)) {
    nfy(oqp);
  }

  return wr;
}

/**
 * @brief   Output queue write with timeout.
 * @details The function writes data from a buffer to an output queue. The
 *          operation completes when the specified amount of data has been
 *          transferred or after the specified timeout or if the queue has
 *          been reset.
 * @note    The function is not atomic, if you need atomicity it is suggested
 *          to use a semaphore or a mutex for mutual exclusion.
 * @note    The callback is invoked after putting each character into the
 *          queue.
 *
 * @param[in] oqp       pointer to an @p output_queue_t structure
 * @param[in] bp        pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 * @return              The number of bytes effectively transferred.
 *
 * @api
 */
size_t oqWriteTimeout(output_queue_t *oqp, const uint8_t *bp,
                      size_t n, sysinterval_t timeout) {
  qnotify_t nfy = oqp->q_notify;
  size_t max = n;

  osalDbgCheck(n > 0U);

  osalSysLock();

  while (n > 0U) {
    size_t done;

    done = oq_write(oqp, bp, n);
    if (done == (size_t)0) {
      msg_t msg = osalThreadEnqueueTimeoutS(&oqp->q_waiting, timeout);

      /* Anything except MSG_OK causes the operation to stop.*/
      if (msg != MSG_OK) {
        break;
      }
    }
    else {
      /* Inform the low side that the queue has at least one character
         available.*/
      if (nfy != NULL) {
        nfy(oqp);
      }

      /* Giving a preemption chance in a controlled point.*/
      osalSysUnlock();

      n  -= done;
      bp += done;

      osalSysLock();
    }
  }

  osalSysUnlock();
  return max - n;
}

/** @} */
