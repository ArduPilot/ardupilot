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
 * @file    rt/include/chmsg.h
 * @brief   Messages macros and structures.
 *
 * @addtogroup messages
 * @{
 */

#ifndef CHMSG_H
#define CHMSG_H

#if (CH_CFG_USE_MESSAGES == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

#if CH_CFG_USE_MESSAGES_PRIORITY == TRUE
#define __ch_msg_insert(qp, tp) ch_sch_prio_insert(qp, &tp->hdr.queue)
#else
#define __ch_msg_insert(qp, tp) ch_queue_insert(qp, &tp->hdr.queue)
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  msg_t chMsgSend(thread_t *tp, msg_t msg);
  thread_t *chMsgWaitS(void);
  thread_t *chMsgWaitTimeoutS(sysinterval_t timeout);
  thread_t *chMsgPollS(void);
  void chMsgRelease(thread_t *tp, msg_t msg);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

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
static inline thread_t *chMsgWait(void) {
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
 *                      the following special values are handled:
 *                      - @a TIME_INFINITE no timeout.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 * @return              A pointer to the thread carrying the message.
 * @retval NULL         if a timeout occurred.
 *
 * @api
 */
static inline thread_t *chMsgWaitTimeout(sysinterval_t timeout) {
  thread_t *tp;

  chSysLock();
  tp = chMsgWaitTimeoutS(timeout);
  chSysUnlock();

  return tp;
}

/**
 * @brief   Poll to check for an incoming message.
 * @post    If a message is available the function @p chMsgGet() must be
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
 * @retval  NULL        if no incoming message waiting.
 *
 * @api
 */
static inline thread_t *chMsgPoll(void) {
  thread_t *tp;

  chSysLock();
  tp = chMsgPollS();
  chSysUnlock();

  return tp;
}

/**
 * @brief   Evaluates to @p true if the thread has pending messages.
 *
 * @param[in] tp        pointer to the thread
 * @return              The pending messages status.
 *
 * @iclass
 */
static inline bool chMsgIsPendingI(thread_t *tp) {

  chDbgCheckClassI();

  return (bool)(tp->msgqueue.next != &tp->msgqueue);
}

/**
 * @brief   Returns the message carried by the specified thread.
 * @pre     This function must be invoked immediately after exiting a call
 *          to @p chMsgWait().
 *
 * @param[in] tp        pointer to the thread
 * @return              The message carried by the sender.
 *
 * @api
 */
static inline msg_t chMsgGet(thread_t *tp) {

  chDbgAssert(tp->state == CH_STATE_SNDMSG, "invalid state");

  return tp->sentmsg;
}

/**
 * @brief   Releases the thread waiting on top of the messages queue.
 * @pre     Invoke this function only after a message has been received
 *          using @p chMsgWait().
 *
 * @param[in] tp        pointer to the thread
 * @param[in] msg       message to be returned to the sender
 *
 * @sclass
 */
static inline void chMsgReleaseS(thread_t *tp, msg_t msg) {

  chDbgCheckClassS();

  chSchWakeupS(tp, msg);
}

#endif /* CH_CFG_USE_MESSAGES == TRUE */

#endif /* CHMSG_H */

/** @} */
