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
 * @file    nil/include/chmsg.h
 * @brief   Nil RTOS synchronous messages header file.
 *
 * @addtogroup NIL_MESSAGES
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

/**
 * @name    Macro Functions
 * @{
 */
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
 * @sclass
 */
#define chMsgWaitS() chMsgWaitTimeoutS(TIME_INFINITE)

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
#define chMsgGet(tp) ((tp)->sntmsg)

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
#define chMsgReleaseS(tp, msg) do {                                         \
  (void) chSchReadyI(tp, msg);                                              \
  chSchRescheduleS();                                                       \
  } while (false)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  msg_t chMsgSend(thread_t *tp, msg_t msg);
  thread_t *chMsgWait(void);
  thread_t *chMsgWaitTimeout(sysinterval_t timeout);
  thread_t *chMsgWaitTimeoutS(sysinterval_t timeout);
  void chMsgRelease(thread_t *tp, msg_t msg);
#ifdef __cplusplus
}
#endif

#endif /* CH_CFG_USE_MESSAGES == TRUE */

#endif /* CHMSG_H */

/** @} */
