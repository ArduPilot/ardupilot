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
 * @file    nil/src/chmsg.c
 * @brief   Nil RTOS synchronous messages source file.
 *
 * @addtogroup NIL_MESSAGES
 * @{
 */

#include "ch.h"

#if (CH_CFG_USE_MESSAGES == TRUE) || defined(__DOXYGEN__)

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
 * @brief   Sends a message to the specified thread.
 * @details The sender is stopped until the receiver executes a
 *          @p chMsgRelease()after receiving the message.
 *
 * @param[in] tp        the pointer to the thread
 * @param[in] msg       the message
 * @return              The answer message from @p chMsgRelease().
 *
 * @api
 */
msg_t chMsgSend(thread_t *tp, msg_t msg) {
  thread_t *ctp = nil.current;

  chDbgCheck(tp != NULL);

  chSysLock();
  ctp->sntmsg = msg;
  ctp->u1.tp =  tp;
  if (NIL_THD_IS_WTMSG(tp)) {
    (void) chSchReadyI(tp, (msg_t)ctp);
  }
  msg = chSchGoSleepTimeoutS(NIL_STATE_SNDMSGQ, TIME_INFINITE);
  chSysUnlock();

  return msg;
}

/**
 * @brief   Suspends the thread and waits for an incoming message.
 * @post    After receiving a message the function @p chMsgGet() must be
 *          called in order to retrieve the message and then @p chMsgRelease()
 *          must be invoked in order to acknowledge the reception and send
 *          the answer.
 * @note    If the message is a pointer then you can assume that the data
 *          pointed by the message is stable until you invoke @p chMsgRelease()
 *          because the sending thread is suspended until then.
 * @note    The reference counter of the sender thread is not increased, the
 *          returned pointer is a temporary reference.
 *
 * @return              A pointer to the thread carrying the message.
 *
 * @api
 */
thread_t *chMsgWait(void) {
  thread_t *tp;

  chSysLock();
  tp = chMsgWaitS();
  chSysUnlock();

  return tp;
}

/**
 * @brief   Suspends the thread and waits for an incoming message or a
 *          timeout to occur.
 * @post    After receiving a message the function @p chMsgGet() must be
 *          called in order to retrieve the message and then @p chMsgRelease()
 *          must be invoked in order to acknowledge the reception and send
 *          the answer.
 * @note    If the message is a pointer then you can assume that the data
 *          pointed by the message is stable until you invoke @p chMsgRelease()
 *          because the sending thread is suspended until then.
 * @note    The reference counter of the sender thread is not increased, the
 *          returned pointer is a temporary reference.
 *
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              A pointer to the thread carrying the message.
 * @retval NULL         if a timeout occurred.
 *
 * @api
 */
thread_t *chMsgWaitTimeout(sysinterval_t timeout) {
  thread_t *tp;

  chSysLock();
  tp = chMsgWaitTimeoutS(timeout);
  chSysUnlock();

  return tp;
}

/**
 * @brief   Suspends the thread and waits for an incoming message or a
 *          timeout to occur.
 * @post    After receiving a message the function @p chMsgGet() must be
 *          called in order to retrieve the message and then @p chMsgRelease()
 *          must be invoked in order to acknowledge the reception and send
 *          the answer.
 * @note    If the message is a pointer then you can assume that the data
 *          pointed by the message is stable until you invoke @p chMsgRelease()
 *          because the sending thread is suspended until then.
 * @note    The reference counter of the sender thread is not increased, the
 *          returned pointer is a temporary reference.
 *
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              A pointer to the thread carrying the message.
 * @retval NULL         if a timeout occurred.
 *
 * @sclass
 */
thread_t *chMsgWaitTimeoutS(sysinterval_t timeout) {
  thread_t *tp;

  chDbgCheckClassS();

  tp = nil_find_thread(NIL_STATE_SNDMSGQ, nil.current);
  if (tp == NULL) {
    msg_t msg = chSchGoSleepTimeoutS(NIL_STATE_WTMSG, timeout);
    if (msg != MSG_TIMEOUT) {
      return (thread_t *)msg;
    }
  }

  return tp;
}

/**
 * @brief   Releases a sender thread specifying a response message.
 * @pre     Invoke this function only after a message has been received
 *          using @p chMsgWait().
 *
 * @param[in] tp        pointer to the thread
 * @param[in] msg       message to be returned to the sender
 *
 * @api
 */
void chMsgRelease(thread_t *tp, msg_t msg) {

  chSysLock();
  chDbgAssert(tp->state == NIL_STATE_SNDMSGQ, "invalid state");
  chMsgReleaseS(tp, msg);
  chSysUnlock();
}

#endif /* CH_CFG_USE_MESSAGES == TRUE */

/** @} */
