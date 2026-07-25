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
 * @file    oslib/src/chmboxes.c
 * @brief   Mailboxes code.
 *
 * @addtogroup oslib_mailboxes
 * @details Asynchronous messages.
 *          <h2>Operation mode</h2>
 *          A mailbox is an asynchronous communication mechanism.<br>
 *          Operations defined for mailboxes:
 *          - <b>Post</b>: Posts a message on the mailbox in FIFO order.
 *          - <b>Post Ahead</b>: Posts a message on the mailbox with urgent
 *            priority.
 *          - <b>Fetch</b>: A message is fetched from the mailbox and removed
 *            from the queue.
 *          - <b>Reset</b>: The mailbox is emptied and all the stored messages
 *            are lost.
 *          .
 *          A message is a variable of type msg_t that is guaranteed to have
 *          the same size of and be compatible with (data) pointers (anyway an
 *          explicit cast is needed).
 *          If larger messages need to be exchanged then a pointer to a
 *          structure can be posted in the mailbox but the posting side has
 *          no predefined way to know when the message has been processed. A
 *          possible approach is to allocate memory (from a memory pool for
 *          example) from the posting side and free it on the fetching side.
 *          Another approach is to set a "done" flag into the structure pointed
 *          by the message.
 * @pre     In order to use the mailboxes APIs the @p CH_CFG_USE_MAILBOXES
 *          option must be enabled in @p chconf.h.
 * @note    Compatible with RT and NIL.
 * @{
 */

#include "ch.h"

#if (CH_CFG_USE_MAILBOXES == TRUE) || defined(__DOXYGEN__)

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

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes a @p mailbox_t object.
 *
 * @param[out] mbp      the pointer to the @p mailbox_t structure to be
 *                      initialized
 * @param[in] buf       pointer to the messages buffer as an array of @p msg_t
 * @param[in] n         number of elements in the buffer array
 *
 * @init
 */
void chMBObjectInit(mailbox_t *mbp, msg_t *buf, size_t n) {

  chDbgCheck((mbp != NULL) && (buf != NULL) && (n > (size_t)0));

  mbp->buffer = buf;
  mbp->rdptr  = buf;
  mbp->wrptr  = buf;
  mbp->top    = &buf[n];
  mbp->cnt    = (size_t)0;
  mbp->reset  = false;
  chThdQueueObjectInit(&mbp->qw);
  chThdQueueObjectInit(&mbp->qr);
}

/**
 * @brief   Resets a @p mailbox_t object.
 * @details All the waiting threads are resumed with status @p MSG_RESET and
 *          the queued messages are lost.
 * @post    The mailbox is in reset state, all operations will fail and
 *          return @p MSG_RESET until the mailbox is enabled again using
 *          @p chMBResumeX().
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 *
 * @api
 */
void chMBReset(mailbox_t *mbp) {

  chSysLock();
  chMBResetI(mbp);
  chSchRescheduleS();
  chSysUnlock();
}

/**
 * @brief   Resets a @p mailbox_t object.
 * @details All the waiting threads are resumed with status @p MSG_RESET and
 *          the queued messages are lost.
 * @post    The mailbox is in reset state, all operations will fail and
 *          return @p MSG_RESET until the mailbox is enabled again using
 *          @p chMBResumeX().
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 *
 * @api
 */
void chMBResetI(mailbox_t *mbp) {

  chDbgCheckClassI();
  chDbgCheck(mbp != NULL);

  mbp->wrptr = mbp->buffer;
  mbp->rdptr = mbp->buffer;
  mbp->cnt   = (size_t)0;
  mbp->reset = true;
  chThdDequeueAllI(&mbp->qw, MSG_RESET);
  chThdDequeueAllI(&mbp->qr, MSG_RESET);
}

/**
 * @brief   Posts a message into a mailbox.
 * @details The invoking thread waits until a empty slot in the mailbox becomes
 *          available or the specified time runs out.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @param[in] msg       the message to be posted on the mailbox
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if a message has been correctly posted.
 * @retval MSG_RESET    if the mailbox has been reset.
 * @retval MSG_TIMEOUT  if the operation has timed out.
 *
 * @api
 */
msg_t chMBPostTimeout(mailbox_t *mbp, msg_t msg, sysinterval_t timeout) {
  msg_t rdymsg;

  chSysLock();
  rdymsg = chMBPostTimeoutS(mbp, msg, timeout);
  chSysUnlock();

  return rdymsg;
}

/**
 * @brief   Posts a message into a mailbox.
 * @details The invoking thread waits until a empty slot in the mailbox becomes
 *          available or the specified time runs out.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @param[in] msg       the message to be posted on the mailbox
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if a message has been correctly posted.
 * @retval MSG_RESET    if the mailbox has been reset.
 * @retval MSG_TIMEOUT  if the operation has timed out.
 *
 * @sclass
 */
msg_t chMBPostTimeoutS(mailbox_t *mbp, msg_t msg, sysinterval_t timeout) {
  msg_t rdymsg;

  chDbgCheckClassS();
  chDbgCheck(mbp != NULL);

  do {
    /* If the mailbox is in reset state then returns immediately.*/
    if (mbp->reset) {
      return MSG_RESET;
    }

    /* Is there a free message slot in queue? if so then post.*/
    if (chMBGetFreeCountI(mbp) > (size_t)0) {
      *mbp->wrptr++ = msg;
      if (mbp->wrptr >= mbp->top) {
        mbp->wrptr = mbp->buffer;
      }
      mbp->cnt++;

      /* If there is a reader waiting then makes it ready.*/
      chThdDequeueNextI(&mbp->qr, MSG_OK);
      chSchRescheduleS();

      return MSG_OK;
    }

    /* No space in the queue, waiting for a slot to become available.*/
    rdymsg = chThdEnqueueTimeoutS(&mbp->qw, timeout);
  } while (rdymsg == MSG_OK);

  return rdymsg;
}

/**
 * @brief   Posts a message into a mailbox.
 * @details This variant is non-blocking, the function returns a timeout
 *          condition if the queue is full.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @param[in] msg       the message to be posted on the mailbox
 * @return              The operation status.
 * @retval MSG_OK       if a message has been correctly posted.
 * @retval MSG_RESET    if the mailbox has been reset.
 * @retval MSG_TIMEOUT  if the mailbox is full and the message cannot be
 *                      posted.
 *
 * @iclass
 */
msg_t chMBPostI(mailbox_t *mbp, msg_t msg) {

  chDbgCheckClassI();
  chDbgCheck(mbp != NULL);

  /* If the mailbox is in reset state then returns immediately.*/
  if (mbp->reset) {
    return MSG_RESET;
  }

  /* Is there a free message slot in queue? if so then post.*/
  if (chMBGetFreeCountI(mbp) > (size_t)0) {
    *mbp->wrptr++ = msg;
    if (mbp->wrptr >= mbp->top) {
      mbp->wrptr = mbp->buffer;
    }
    mbp->cnt++;

    /* If there is a reader waiting then makes it ready.*/
    chThdDequeueNextI(&mbp->qr, MSG_OK);

    return MSG_OK;
  }

  /* No space, immediate timeout.*/
  return MSG_TIMEOUT;
}

/**
 * @brief   Posts an high priority message into a mailbox.
 * @details The invoking thread waits until a empty slot in the mailbox becomes
 *          available or the specified time runs out.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @param[in] msg       the message to be posted on the mailbox
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if a message has been correctly posted.
 * @retval MSG_RESET    if the mailbox has been reset.
 * @retval MSG_TIMEOUT  if the operation has timed out.
 *
 * @api
 */
msg_t chMBPostAheadTimeout(mailbox_t *mbp, msg_t msg, sysinterval_t timeout) {
  msg_t rdymsg;

  chSysLock();
  rdymsg = chMBPostAheadTimeoutS(mbp, msg, timeout);
  chSysUnlock();

  return rdymsg;
}

/**
 * @brief   Posts an high priority message into a mailbox.
 * @details The invoking thread waits until a empty slot in the mailbox becomes
 *          available or the specified time runs out.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @param[in] msg       the message to be posted on the mailbox
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if a message has been correctly posted.
 * @retval MSG_RESET    if the mailbox has been reset.
 * @retval MSG_TIMEOUT  if the operation has timed out.
 *
 * @sclass
 */
msg_t chMBPostAheadTimeoutS(mailbox_t *mbp, msg_t msg, sysinterval_t timeout) {
  msg_t rdymsg;

  chDbgCheckClassS();
  chDbgCheck(mbp != NULL);

  do {
    /* If the mailbox is in reset state then returns immediately.*/
    if (mbp->reset) {
      return MSG_RESET;
    }

    /* Is there a free message slot in queue? if so then post.*/
    if (chMBGetFreeCountI(mbp) > (size_t)0) {
      if (--mbp->rdptr < mbp->buffer) {
        mbp->rdptr = mbp->top - 1;
      }
      *mbp->rdptr = msg;
      mbp->cnt++;

      /* If there is a reader waiting then makes it ready.*/
      chThdDequeueNextI(&mbp->qr, MSG_OK);
      chSchRescheduleS();

      return MSG_OK;
    }

    /* No space in the queue, waiting for a slot to become available.*/
    rdymsg = chThdEnqueueTimeoutS(&mbp->qw, timeout);
  } while (rdymsg == MSG_OK);

  return rdymsg;
}

/**
 * @brief   Posts an high priority message into a mailbox.
 * @details This variant is non-blocking, the function returns a timeout
 *          condition if the queue is full.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @param[in] msg       the message to be posted on the mailbox
 * @return              The operation status.
 * @retval MSG_OK       if a message has been correctly posted.
 * @retval MSG_RESET    if the mailbox has been reset.
 * @retval MSG_TIMEOUT  if the mailbox is full and the message cannot be
 *                      posted.
 *
 * @iclass
 */
msg_t chMBPostAheadI(mailbox_t *mbp, msg_t msg) {

  chDbgCheckClassI();
  chDbgCheck(mbp != NULL);

  /* If the mailbox is in reset state then returns immediately.*/
  if (mbp->reset) {
    return MSG_RESET;
  }

  /* Is there a free message slot in queue? if so then post.*/
  if (chMBGetFreeCountI(mbp) > (size_t)0) {
    if (--mbp->rdptr < mbp->buffer) {
      mbp->rdptr = mbp->top - 1;
    }
    *mbp->rdptr = msg;
    mbp->cnt++;

    /* If there is a reader waiting then makes it ready.*/
    chThdDequeueNextI(&mbp->qr, MSG_OK);

    return MSG_OK;
  }

  /* No space, immediate timeout.*/
  return MSG_TIMEOUT;
}

/**
 * @brief   Retrieves a message from a mailbox.
 * @details The invoking thread waits until a message is posted in the mailbox
 *          or the specified time runs out.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @param[out] msgp     pointer to a message variable for the received message
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if a message has been correctly fetched.
 * @retval MSG_RESET    if the mailbox has been reset.
 * @retval MSG_TIMEOUT  if the operation has timed out.
 *
 * @api
 */
msg_t chMBFetchTimeout(mailbox_t *mbp, msg_t *msgp, sysinterval_t timeout) {
  msg_t rdymsg;

  chSysLock();
  rdymsg = chMBFetchTimeoutS(mbp, msgp, timeout);
  chSysUnlock();

  return rdymsg;
}

/**
 * @brief   Retrieves a message from a mailbox.
 * @details The invoking thread waits until a message is posted in the mailbox
 *          or the specified time runs out.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @param[out] msgp     pointer to a message variable for the received message
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if a message has been correctly fetched.
 * @retval MSG_RESET    if the mailbox has been reset.
 * @retval MSG_TIMEOUT  if the operation has timed out.
 *
 * @sclass
 */
msg_t chMBFetchTimeoutS(mailbox_t *mbp, msg_t *msgp, sysinterval_t timeout) {
  msg_t rdymsg;

  chDbgCheckClassS();
  chDbgCheck((mbp != NULL) && (msgp != NULL));

  do {
    /* If the mailbox is in reset state then returns immediately.*/
    if (mbp->reset) {
      return MSG_RESET;
    }

    /* Is there a message in queue? if so then fetch.*/
    if (chMBGetUsedCountI(mbp) > (size_t)0) {
      *msgp = *mbp->rdptr++;
      if (mbp->rdptr >= mbp->top) {
        mbp->rdptr = mbp->buffer;
      }
      mbp->cnt--;

      /* If there is a writer waiting then makes it ready.*/
      chThdDequeueNextI(&mbp->qw, MSG_OK);
      chSchRescheduleS();

      return MSG_OK;
    }

    /* No message in the queue, waiting for a message to become available.*/
    rdymsg = chThdEnqueueTimeoutS(&mbp->qr, timeout);
  } while (rdymsg == MSG_OK);

  return rdymsg;
}

/**
 * @brief   Retrieves a message from a mailbox.
 * @details This variant is non-blocking, the function returns a timeout
 *          condition if the queue is empty.
 *
 * @param[in] mbp       the pointer to an initialized @p mailbox_t object
 * @param[out] msgp     pointer to a message variable for the received message
 * @return              The operation status.
 * @retval MSG_OK       if a message has been correctly fetched.
 * @retval MSG_RESET    if the mailbox has been reset.
 * @retval MSG_TIMEOUT  if the mailbox is empty and a message cannot be
 *                      fetched.
 *
 * @iclass
 */
msg_t chMBFetchI(mailbox_t *mbp, msg_t *msgp) {

  chDbgCheckClassI();
  chDbgCheck((mbp != NULL) && (msgp != NULL));

  /* If the mailbox is in reset state then returns immediately.*/
  if (mbp->reset) {
    return MSG_RESET;
  }

  /* Is there a message in queue? if so then fetch.*/
  if (chMBGetUsedCountI(mbp) > (size_t)0) {
    *msgp = *mbp->rdptr++;
    if (mbp->rdptr >= mbp->top) {
      mbp->rdptr = mbp->buffer;
    }
    mbp->cnt--;

    /* If there is a writer waiting then makes it ready.*/
    chThdDequeueNextI(&mbp->qw, MSG_OK);

    return MSG_OK;
  }

  /* No message, immediate timeout.*/
  return MSG_TIMEOUT;
}
#endif /* CH_CFG_USE_MAILBOXES == TRUE */

/** @} */
