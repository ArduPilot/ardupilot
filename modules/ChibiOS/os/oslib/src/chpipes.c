/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    oslib/src/chpipes.c
 * @brief   Pipes code.
 * @details Byte pipes.
 *          <h2>Operation mode</h2>
 *          A pipe is an asynchronous communication mechanism.<br>
 *          Operations defined for mailboxes:
 *          - <b>Write</b>: Writes a buffer of data in the pipe in FIFO order.
 *          - <b>Read</b>: A buffer of data is read from the read and removed.
 *          - <b>Reset</b>: The pipe is emptied and all the stored data
 *            is lost.
 *          .
 * @pre     In order to use the pipes APIs the @p CH_CFG_USE_PIPES
 *          option must be enabled in @p chconf.h.
 * @note    Compatible with RT and NIL.
 *
 * @addtogroup oslib_pipes
 * @{
 */

#include <string.h>

#include "ch.h"

#if (CH_CFG_USE_PIPES == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*
 * Defaults on the best synchronization mechanism available.
 */
#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)
#define PC_INIT(p)       chMtxObjectInit(&(p)->cmtx)
#define PC_LOCK(p)       chMtxLock(&(p)->cmtx)
#define PC_UNLOCK(p)     chMtxUnlock(&(p)->cmtx)
#define PW_INIT(p)       chMtxObjectInit(&(p)->wmtx)
#define PW_LOCK(p)       chMtxLock(&(p)->wmtx)
#define PW_UNLOCK(p)     chMtxUnlock(&(p)->wmtx)
#define PR_INIT(p)       chMtxObjectInit(&(p)->rmtx)
#define PR_LOCK(p)       chMtxLock(&(p)->rmtx)
#define PR_UNLOCK(p)     chMtxUnlock(&(p)->rmtx)
#else
#define PC_INIT(p)       chSemObjectInit(&(p)->csem, (cnt_t)1)
#define PC_LOCK(p)       (void) chSemWait(&(p)->csem)
#define PC_UNLOCK(p)     chSemSignal(&(p)->csem)
#define PW_INIT(p)       chSemObjectInit(&(p)->wsem, (cnt_t)1)
#define PW_LOCK(p)       (void) chSemWait(&(p)->wsem)
#define PW_UNLOCK(p)     chSemSignal(&(p)->wsem)
#define PR_INIT(p)       chSemObjectInit(&(p)->rsem, (cnt_t)1)
#define PR_LOCK(p)       (void) chSemWait(&(p)->rsem)
#define PR_UNLOCK(p)     chSemSignal(&(p)->rsem)
#endif

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Non-blocking pipe write.
 * @details The function writes data from a buffer to a pipe. The
 *          operation completes when the specified amount of data has been
 *          transferred or when the pipe buffer has been filled.
 *
 * @param[in] pp        the pointer to an initialized @p pipe_t object
 * @param[in] bp        pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @return              The number of bytes effectively transferred.
 *
 * @notapi
 */
static size_t pipe_write(pipe_t *pp, const uint8_t *bp, size_t n) {
  size_t s1, s2;

  PC_LOCK(pp);

  /* Number of bytes that can be written in a single atomic operation.*/
  if (n > chPipeGetFreeCount(pp)) {
    n = chPipeGetFreeCount(pp);
  }
  pp->cnt += n;

  /* Number of bytes before buffer limit.*/
  /*lint -save -e9033 [10.8] Checked to be safe.*/
  s1 = (size_t)(pp->top - pp->wrptr);
  /*lint -restore*/

  if (n < s1) {
    memcpy((void *)pp->wrptr, (const void *)bp, n);
    pp->wrptr += n;
  }
  else if (n > s1) {
    memcpy((void *)pp->wrptr, (const void *)bp, s1);
    bp += s1;
    s2 = n - s1;
    memcpy((void *)pp->buffer, (const void *)bp, s2);
    pp->wrptr = pp->buffer + s2;
  }
  else {
    memcpy((void *)pp->wrptr, (const void *)bp, n);
    pp->wrptr = pp->buffer;
  }

  PC_UNLOCK(pp);

  return n;
}

/**
 * @brief   Non-blocking pipe read.
 * @details The function reads data from a pipe into a buffer. The
 *          operation completes when the specified amount of data has been
 *          transferred or when the pipe buffer has been emptied.
 *
 * @param[in] pp        the pointer to an initialized @p pipe_t object
 * @param[out] bp       pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @return              The number of bytes effectively transferred.
 *
 * @notapi
 */
static size_t pipe_read(pipe_t *pp, uint8_t *bp, size_t n) {
  size_t s1, s2;

  PC_LOCK(pp);

  /* Number of bytes that can be read in a single atomic operation.*/
  if (n > chPipeGetUsedCount(pp)) {
    n = chPipeGetUsedCount(pp);
  }
  pp->cnt -= n;

  /* Number of bytes before buffer limit.*/
  /*lint -save -e9033 [10.8] Checked to be safe.*/
  s1 = (size_t)(pp->top - pp->rdptr);
  /*lint -restore*/

  if (n < s1) {
    memcpy((void *)bp, (void *)pp->rdptr, n);
    pp->rdptr += n;
  }
  else if (n > s1) {
    memcpy((void *)bp, (void *)pp->rdptr, s1);
    bp += s1;
    s2 = n - s1;
    memcpy((void *)bp, (void *)pp->buffer, s2);
    pp->rdptr = pp->buffer + s2;
  }
  else {
    memcpy((void *)bp, (void *)pp->rdptr, n);
    pp->rdptr = pp->buffer;
  }

  PC_UNLOCK(pp);

  return n;
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes a @p mailbox_t object.
 *
 * @param[out] pp       the pointer to the @p pipe_t structure to be
 *                      initialized
 * @param[in] buf       pointer to the pipe buffer as an array of @p uint8_t
 * @param[in] n         number of elements in the buffer array
 *
 * @init
 */
void chPipeObjectInit(pipe_t *pp, uint8_t *buf, size_t n) {

  chDbgCheck((pp != NULL) && (buf != NULL) && (n > (size_t)0));

  pp->buffer = buf;
  pp->rdptr  = buf;
  pp->wrptr  = buf;
  pp->top    = &buf[n];
  pp->cnt    = (size_t)0;
  pp->reset  = false;
  pp->wtr    = NULL;
  pp->rtr    = NULL;
  PC_INIT(pp);
  PW_INIT(pp);
  PR_INIT(pp);
}

/**
 * @brief   Resets a @p pipe_t object.
 * @details All the waiting threads are resumed with status @p MSG_RESET and
 *          the queued data is lost.
 * @post    The pipe is in reset state, all operations will fail and
 *          return @p MSG_RESET until the mailbox is enabled again using
 *          @p chPipeResumeX().
 *
 * @param[in] pp        the pointer to an initialized @p pipe_t object
 *
 * @api
 */
void chPipeReset(pipe_t *pp) {

  chDbgCheck(pp != NULL);

  PC_LOCK(pp);

  pp->wrptr = pp->buffer;
  pp->rdptr = pp->buffer;
  pp->cnt   = (size_t)0;
  pp->reset = true;

  chSysLock();
  chThdResumeI(&pp->wtr, MSG_RESET);
  chThdResumeI(&pp->rtr, MSG_RESET);
  chSchRescheduleS();
  chSysUnlock();

  PC_UNLOCK(pp);
}

/**
 * @brief   Pipe write with timeout.
 * @details The function writes data from a buffer to a pipe. The
 *          operation completes when the specified amount of data has been
 *          transferred or after the specified timeout or if the pipe has
 *          been reset.
 *
 * @param[in] pp        the pointer to an initialized @p pipe_t object
 * @param[in] bp        pointer to the data buffer
 * @param[in] n         the number of bytes to be written, the value 0 is
 *                      reserved
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The number of bytes effectively transferred. A number
 *                      lower than @p n means that a timeout occurred or the
 *                      pipe went in reset state.
 *
 * @api
 */
size_t chPipeWriteTimeout(pipe_t *pp, const uint8_t *bp,
                          size_t n, sysinterval_t timeout) {
  size_t max = n;

  chDbgCheck(n > 0U);

  /* If the pipe is in reset state then returns immediately.*/
  if (pp->reset) {
    return (size_t)0;
  }

  PW_LOCK(pp);

  while (n > 0U) {
    size_t done;

    done = pipe_write(pp, bp, n);
    if (done == (size_t)0) {
      msg_t msg;

      chSysLock();
      msg = chThdSuspendTimeoutS(&pp->wtr, timeout);
      chSysUnlock();

      /* Anything except MSG_OK causes the operation to stop.*/
      if (msg != MSG_OK) {
        break;
      }
    }
    else {
      n  -= done;
      bp += done;

      /* Resuming the reader, if present.*/
      chThdResume(&pp->rtr, MSG_OK);
    }
  }

  PW_UNLOCK(pp);

  return max - n;
}

/**
 * @brief   Pipe read with timeout.
 * @details The function reads data from a pipe into a buffer. The
 *          operation completes when the specified amount of data has been
 *          transferred or after the specified timeout or if the pipe has
 *          been reset.
 *
 * @param[in] pp        the pointer to an initialized @p pipe_t object
 * @param[out] bp       pointer to the data buffer
 * @param[in] n         the number of bytes to be read, the value 0 is
 *                      reserved
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The number of bytes effectively transferred. A number
 *                      lower than @p n means that a timeout occurred or the
 *                      pipe went in reset state.
 *
 * @api
 */
size_t chPipeReadTimeout(pipe_t *pp, uint8_t *bp,
                         size_t n, sysinterval_t timeout) {
  size_t max = n;

  chDbgCheck(n > 0U);

  /* If the pipe is in reset state then returns immediately.*/
  if (pp->reset) {
    return (size_t)0;
  }

  PR_LOCK(pp);

  while (n > 0U) {
    size_t done;

    done = pipe_read(pp, bp, n);
    if (done == (size_t)0) {
      msg_t msg;

      chSysLock();
      msg = chThdSuspendTimeoutS(&pp->rtr, timeout);
      chSysUnlock();

      /* Anything except MSG_OK causes the operation to stop.*/
      if (msg != MSG_OK) {
        break;
      }
    }
    else {
      n  -= done;
      bp += done;

      /* Resuming the writer, if present.*/
      chThdResume(&pp->wtr, MSG_OK);
    }
  }

  PR_UNLOCK(pp);

  return max - n;
}

#endif /* CH_CFG_USE_PIPES == TRUE */

/** @} */
