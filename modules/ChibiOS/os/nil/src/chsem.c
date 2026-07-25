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
 * @file    nil/src/chsem.c
 * @brief   Nil RTOS semaphores source file.
 *
 * @addtogroup NIL_SEMAPHORES
 * @{
 */

#include "ch.h"

#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Performs a wait operation on a semaphore with timeout specification.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              A message specifying how the invoking thread has been
 *                      released from the semaphore.
 * @retval NIL_MSG_OK   if the thread has not stopped on the semaphore or the
 *                      semaphore has been signaled.
 * @retval NIL_MSG_RST  if the semaphore has been reset using @p chSemReset().
 * @retval NIL_MSG_TMO  if the semaphore has not been signaled or reset within
 *                      the specified timeout.
 *
 * @api
 */
msg_t chSemWaitTimeout(semaphore_t *sp, sysinterval_t timeout) {
  msg_t msg;

  chSysLock();
  msg = chSemWaitTimeoutS(sp, timeout);
  chSysUnlock();

  return msg;
}

/**
 * @brief   Performs a wait operation on a semaphore with timeout specification.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              A message specifying how the invoking thread has been
 *                      released from the semaphore.
 * @retval NIL_MSG_OK   if the thread has not stopped on the semaphore or the
 *                      semaphore has been signaled.
 * @retval NIL_MSG_RST  if the semaphore has been reset using @p chSemReset().
 * @retval NIL_MSG_TMO  if the semaphore has not been signaled or reset within
 *                      the specified timeout.
 *
 * @sclass
 */
msg_t chSemWaitTimeoutS(semaphore_t *sp, sysinterval_t timeout) {

  chDbgCheckClassS();
  chDbgCheck(sp != NULL);

  /* Note, the semaphore counter is a volatile variable so accesses are
     manually optimized.*/
  cnt_t cnt = sp->cnt;
  if (cnt <= (cnt_t)0) {
    if (TIME_IMMEDIATE == timeout) {

      return MSG_TIMEOUT;
    }
    sp->cnt = cnt - (cnt_t)1;
    nil.current->u1.semp = sp;

    return chSchGoSleepTimeoutS(NIL_STATE_WTQUEUE, timeout);
  }
  sp->cnt = cnt - (cnt_t)1;

  return MSG_OK;
}

/**
 * @brief   Performs a signal operation on a semaphore.
 *
 * @param[in] sp    pointer to a @p semaphore_t structure
 *
 * @api
 */
void chSemSignal(semaphore_t *sp) {

  chSysLock();
  chSemSignalI(sp);
  chSchRescheduleS();
  chSysUnlock();
}

/**
 * @brief   Performs a signal operation on a semaphore.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel. Note that
 *          interrupt handlers always reschedule on exit so an explicit
 *          reschedule must not be performed in ISRs.
 *
 * @param[in] sp    pointer to a @p semaphore_t structure
 *
 * @iclass
 */
void chSemSignalI(semaphore_t *sp) {

  chDbgCheckClassI();
  chDbgCheck(sp != NULL);

  if (++sp->cnt <= (cnt_t)0) {
    thread_t *tp = nil_find_thread(NIL_STATE_WTQUEUE, (void *)sp);

    chDbgAssert(tp != NULL, "thread not found");

    (void) chSchReadyI(tp, MSG_OK);
  }
}

/**
 * @brief   Performs a reset operation on the semaphore.
 * @post    After invoking this function all the threads waiting on the
 *          semaphore, if any, are released and the semaphore counter is set
 *          to the specified, non negative, value.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel. Note that
 *          interrupt handlers always reschedule on exit so an explicit
 *          reschedule must not be performed in ISRs.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 * @param[in] n         the new value of the semaphore counter. The value must
 *                      be non-negative.
 * @param[in] msg       message to be sent
 *
 * @api
 */
void chSemResetWithMessage(semaphore_t *sp, cnt_t n, msg_t msg) {

  chSysLock();
  chSemResetWithMessageI(sp, n, msg);
  chSchRescheduleS();
  chSysUnlock();
}

/**
 * @brief   Performs a reset operation on the semaphore.
 * @post    After invoking this function all the threads waiting on the
 *          semaphore, if any, are released and the semaphore counter is set
 *          to the specified, non negative, value.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel. Note that
 *          interrupt handlers always reschedule on exit so an explicit
 *          reschedule must not be performed in ISRs.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 * @param[in] n         the new value of the semaphore counter. The value must
 *                      be non-negative.
 * @param[in] msg       message to be sent
 *
 * @iclass
 */
void chSemResetWithMessageI(semaphore_t *sp, cnt_t n, msg_t msg) {
  cnt_t cnt;

  chDbgCheckClassI();
  chDbgCheck((sp != NULL) && (n >= (cnt_t)0));

  cnt = sp->cnt;
  sp->cnt = n;

  /* Does nothing for cnt >= 0, calling anyway.*/
  (void) nil_ready_all((void *)sp, cnt, msg);
}

#endif /* CH_CFG_USE_SEMAPHORES == TRUE */

/** @} */
