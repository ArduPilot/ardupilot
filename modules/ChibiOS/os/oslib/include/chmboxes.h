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
 * @file    oslib/include/chmboxes.h
 * @brief   Mailboxes macros and structures.
 *
 * @addtogroup oslib_mailboxes
 * @{
 */

#ifndef CHMBOXES_H
#define CHMBOXES_H

#if (CH_CFG_USE_MAILBOXES == TRUE) || defined(__DOXYGEN__)

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

/**
 * @brief   Structure representing a mailbox object.
 */
typedef struct {
  msg_t                 *buffer;        /**< @brief Pointer to the mailbox
                                                    buffer.                 */
  msg_t                 *top;           /**< @brief Pointer to the location
                                                    after the buffer.       */
  msg_t                 *wrptr;         /**< @brief Write pointer.          */
  msg_t                 *rdptr;         /**< @brief Read pointer.           */
  size_t                cnt;            /**< @brief Messages in queue.      */
  bool                  reset;          /**< @brief True in reset state.    */
  threads_queue_t       qw;             /**< @brief Queued writers.         */
  threads_queue_t       qr;             /**< @brief Queued readers.         */
} mailbox_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Data part of a static mailbox initializer.
 * @details This macro should be used when statically initializing a
 *          mailbox that is part of a bigger structure.
 *
 * @param[in] name      the name of the mailbox variable
 * @param[in] buffer    pointer to the mailbox buffer array of @p msg_t
 * @param[in] size      number of @p msg_t elements in the buffer array
 */
#define __MAILBOX_DATA(name, buffer, size) {                                \
  (msg_t *)(buffer),                                                        \
  (msg_t *)(buffer) + size,                                                 \
  (msg_t *)(buffer),                                                        \
  (msg_t *)(buffer),                                                        \
  (size_t)0,                                                                \
  false,                                                                    \
  __THREADS_QUEUE_DATA(name.qw),                                            \
  __THREADS_QUEUE_DATA(name.qr),                                            \
}

/**
 * @brief   Static mailbox initializer.
 * @details Statically initialized mailboxes require no explicit
 *          initialization using @p chMBObjectInit().
 *
 * @param[in] name      the name of the mailbox variable
 * @param[in] buffer    pointer to the mailbox buffer array of @p msg_t
 * @param[in] size      number of @p msg_t elements in the buffer array
 */
#define MAILBOX_DECL(name, buffer, size)                                    \
  mailbox_t name = __MAILBOX_DATA(name, buffer, size)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void chMBObjectInit(mailbox_t *mbp, msg_t *buf, size_t n);
  void chMBReset(mailbox_t *mbp);
  void chMBResetI(mailbox_t *mbp);
  msg_t chMBPostTimeout(mailbox_t *mbp, msg_t msg, sysinterval_t timeout);
  msg_t chMBPostTimeoutS(mailbox_t *mbp, msg_t msg, sysinterval_t timeout);
  msg_t chMBPostI(mailbox_t *mbp, msg_t msg);
  msg_t chMBPostAheadTimeout(mailbox_t *mbp, msg_t msg, sysinterval_t timeout);
  msg_t chMBPostAheadTimeoutS(mailbox_t *mbp, msg_t msg, sysinterval_t timeout);
  msg_t chMBPostAheadI(mailbox_t *mbp, msg_t msg);
  msg_t chMBFetchTimeout(mailbox_t *mbp, msg_t *msgp, sysinterval_t timeout);
  msg_t chMBFetchTimeoutS(mailbox_t *mbp, msg_t *msgp, sysinterval_t timeout);
  msg_t chMBFetchI(mailbox_t *mbp, msg_t *msgp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Returns the mailbox buffer size as number of messages.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @return              The size of the mailbox.
 *
 * @iclass
 */
static inline size_t chMBGetSizeI(const mailbox_t *mbp) {

  /*lint -save -e9033 [10.8] Perfectly safe pointers
    arithmetic.*/
  return (size_t)(mbp->top - mbp->buffer);
  /*lint -restore*/
}

/**
 * @brief   Returns the number of used message slots into a mailbox.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @return              The number of queued messages.
 *
 * @iclass
 */
static inline size_t chMBGetUsedCountI(const mailbox_t *mbp) {

  chDbgCheckClassI();

  return mbp->cnt;
}

/**
 * @brief   Returns the number of free message slots into a mailbox.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @return              The number of empty message slots.
 *
 * @iclass
 */
static inline size_t chMBGetFreeCountI(const mailbox_t *mbp) {

  chDbgCheckClassI();

  return chMBGetSizeI(mbp) - chMBGetUsedCountI(mbp);
}

/**
 * @brief   Returns the next message in the queue without removing it.
 * @pre     A message must be waiting in the queue for this function to work
 *          or it would return garbage. The correct way to use this macro is
 *          to use @p chMBGetUsedCountI() and then use this macro, all within
 *          a lock state.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @return              The next message in queue.
 *
 * @iclass
 */
static inline msg_t chMBPeekI(const mailbox_t *mbp) {

  chDbgCheckClassI();

  return *mbp->rdptr;
}

/**
 * @brief   Terminates the reset state.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 *
 * @xclass
 */
static inline void chMBResumeX(mailbox_t *mbp) {

  mbp->reset = false;
}

#endif /* CH_CFG_USE_MAILBOXES == TRUE */

#endif /* CHMBOXES_H */

/** @} */
