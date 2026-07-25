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
 * @file    oslib/include/chobjfifos.h
 * @brief   Objects FIFO structures and macros.
 * @details This module implements a generic FIFO queue of objects by
 *          coupling a Guarded Memory Pool (for objects storage) and
 *          a MailBox.<br>
 *          On the sender side free objects are taken from the pool, filled
 *          and then sent to the receiver, on the receiver side objects are
 *          fetched, used and then returned to the pool.
 *          Operations defined for object FIFOs:
 *          - <b>Take</b>: An object is taken from the pool of the free
 *            objects, can be blocking.
 *          - <b>Return</b>: An object is returned to the pool of the
 *            free objects, it is guaranteed to be non-blocking.
 *          - <b>Send</b>: An object is sent through the mailbox, it is
 *            guaranteed to be non-blocking
 *          - <b>Receive</b>: An object is received from the mailbox,
 *            can be blocking.
 *          .
 *
 * @addtogroup oslib_objects_fifos
 * @{
 */

#ifndef CHOBJFIFOS_H
#define CHOBJFIFOS_H

#if (CH_CFG_USE_OBJ_FIFOS == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if CH_CFG_USE_MEMPOOLS == FALSE
#error "CH_CFG_USE_OBJ_FIFOS requires CH_CFG_USE_MEMPOOLS"
#endif

#if CH_CFG_USE_SEMAPHORES == FALSE
#error "CH_CFG_USE_OBJ_FIFOS requires CH_CFG_USE_SEMAPHORES"
#endif

#if CH_CFG_USE_MAILBOXES == FALSE
#error "CH_CFG_USE_OBJ_FIFOS requires CH_CFG_USE_MAILBOXES"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of an objects FIFO.
 */
typedef struct ch_objects_fifo {
  /**
   * @brief   Pool of the free objects.
   */
  guarded_memory_pool_t     free;
  /**
   * @brief   Mailbox of the sent objects.
   */
  mailbox_t                 mbx;
} objects_fifo_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Initializes a FIFO object.
 * @pre     The objects size must be a multiple of the alignment
 *          requirement.
 *
 * @param[out] ofp      pointer to a @p objects_fifo_t structure
 * @param[in] objsize   object size
 * @param[in] objn      number of objects available
 * @param[in] objalign  required objects alignment
 * @param[in] objbuf    pointer to the buffer of objects, it must be able
 *                      to hold @p objn objects of @p objsize size with
 *                      @p objalign alignment
 * @param[in] msgbuf    pointer to the buffer of messages, it must be able
 *                      to hold @p objn messages
 *
 * @init
 */
static inline void chFifoObjectInitAligned(objects_fifo_t *ofp, size_t objsize,
                                           size_t objn, unsigned objalign,
                                           void *objbuf, msg_t *msgbuf) {

  chDbgCheck((objsize >= objalign) && ((objsize % objalign) == 0U));

  chGuardedPoolObjectInitAligned(&ofp->free, objsize, objalign);
  chGuardedPoolLoadArray(&ofp->free, objbuf, objn);
  chMBObjectInit(&ofp->mbx, msgbuf, objn);
}

/**
 * @brief   Initializes a FIFO object.
 * @pre     The objects size must be a multiple of the alignment
 *          requirement.
 *
 * @param[out] ofp      pointer to a @p objects_fifo_t structure
 * @param[in] objsize   object size
 * @param[in] objn      number of objects available
 * @param[in] objbuf    pointer to the buffer of objects, it must be able
 *                      to hold @p objn objects of @p objsize size
 * @param[in] msgbuf    pointer to the buffer of messages, it must be able
 *                      to hold @p objn messages
 *
 * @init
 */
static inline void chFifoObjectInit(objects_fifo_t *ofp, size_t objsize,
                                    size_t objn, void *objbuf,
                                    msg_t *msgbuf) {

  chFifoObjectInitAligned(ofp, objsize, objn,
                          PORT_NATURAL_ALIGN,
                          objbuf, msgbuf);
}

/**
 * @brief   Allocates a free object.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @return              The pointer to the allocated object.
 * @retval NULL         if an object is not immediately available.
 *
 * @iclass
 */
static inline void *chFifoTakeObjectI(objects_fifo_t *ofp) {

  return chGuardedPoolAllocI(&ofp->free);
}

/**
 * @brief   Allocates a free object.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The pointer to the allocated object.
 * @retval NULL         if an object is not available within the specified
 *                      timeout.
 *
 * @sclass
 */
static inline void *chFifoTakeObjectTimeoutS(objects_fifo_t *ofp,
                                             sysinterval_t timeout) {

  return chGuardedPoolAllocTimeoutS(&ofp->free, timeout);
}

/**
 * @brief   Allocates a free object.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The pointer to the allocated object.
 * @retval NULL         if an object is not available within the specified
 *                      timeout.
 *
 * @api
 */
static inline void *chFifoTakeObjectTimeout(objects_fifo_t *ofp,
                                            sysinterval_t timeout) {

  return chGuardedPoolAllocTimeout(&ofp->free, timeout);
}

/**
 * @brief   Releases a fetched object.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] objp      pointer to the object to be released
 *
 * @iclass
 */
static inline void chFifoReturnObjectI(objects_fifo_t *ofp,
                                       void *objp) {

  chGuardedPoolFreeI(&ofp->free, objp);
}

/**
 * @brief   Releases a fetched object.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] objp      pointer to the object to be released
 *
 * @sclass
 */
static inline void chFifoReturnObjectS(objects_fifo_t *ofp,
                                       void *objp) {

  chGuardedPoolFreeS(&ofp->free, objp);
}

/**
 * @brief   Releases a fetched object.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] objp      pointer to the object to be released
 *
 * @api
 */
static inline void chFifoReturnObject(objects_fifo_t *ofp,
                                      void *objp) {

  chGuardedPoolFree(&ofp->free, objp);
}

/**
 * @brief   Posts an object.
 * @note    By design the object can be always immediately posted.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] objp      pointer to the object to be posted
 *
 * @iclass
 */
static inline void chFifoSendObjectI(objects_fifo_t *ofp,
                                     void *objp) {
  msg_t msg;

  msg = chMBPostI(&ofp->mbx, (msg_t)objp);
  chDbgAssert(msg == MSG_OK, "post failed");
}

/**
 * @brief   Posts an object.
 * @note    By design the object can be always immediately posted.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] objp      pointer to the object to be posted
 *
 * @sclass
 */
static inline void chFifoSendObjectS(objects_fifo_t *ofp,
                                     void *objp) {
  msg_t msg;

  msg = chMBPostTimeoutS(&ofp->mbx, (msg_t)objp, TIME_IMMEDIATE);
  chDbgAssert(msg == MSG_OK, "post failed");
}

/**
 * @brief   Posts an object.
 * @note    By design the object can be always immediately posted.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] objp      pointer to the object to be released
 *
 * @api
 */
static inline void chFifoSendObject(objects_fifo_t *ofp, void *objp) {

  msg_t msg;

  msg = chMBPostTimeout(&ofp->mbx, (msg_t)objp, TIME_IMMEDIATE);
  chDbgAssert(msg == MSG_OK, "post failed");
}

/**
 * @brief   Posts an high priority object.
 * @note    By design the object can be always immediately posted.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] objp      pointer to the object to be posted
 *
 * @iclass
 */
static inline void chFifoSendObjectAheadI(objects_fifo_t *ofp,
                                          void *objp) {
  msg_t msg;

  msg = chMBPostAheadI(&ofp->mbx, (msg_t)objp);
  chDbgAssert(msg == MSG_OK, "post failed");
}

/**
 * @brief   Posts an high priority object.
 * @note    By design the object can be always immediately posted.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] objp      pointer to the object to be posted
 *
 * @sclass
 */
static inline void chFifoSendObjectAheadS(objects_fifo_t *ofp,
                                          void *objp) {
  msg_t msg;

  msg = chMBPostAheadTimeoutS(&ofp->mbx, (msg_t)objp, TIME_IMMEDIATE);
  chDbgAssert(msg == MSG_OK, "post failed");
}

/**
 * @brief   Posts an high priority object.
 * @note    By design the object can be always immediately posted.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] objp      pointer to the object to be released
 *
 * @api
 */
static inline void chFifoSendObjectAhead(objects_fifo_t *ofp, void *objp) {

  msg_t msg;

  msg = chMBPostAheadTimeout(&ofp->mbx, (msg_t)objp, TIME_IMMEDIATE);
  chDbgAssert(msg == MSG_OK, "post failed");
}

/**
 * @brief   Fetches an object.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] objpp     pointer to the fetched object reference
 * @return              The operation status.
 * @retval MSG_OK       if an object has been correctly fetched.
 * @retval MSG_TIMEOUT  if the FIFO is empty and a message cannot be fetched.
 *
 * @iclass
 */
static inline msg_t chFifoReceiveObjectI(objects_fifo_t *ofp,
                                         void **objpp) {

  return chMBFetchI(&ofp->mbx, (msg_t *)objpp);
}

/**
 * @brief   Fetches an object.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] objpp     pointer to the fetched object reference
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if an object has been correctly fetched.
 * @retval MSG_TIMEOUT  if the operation has timed out.
 *
 * @sclass
 */
static inline msg_t chFifoReceiveObjectTimeoutS(objects_fifo_t *ofp,
                                                void **objpp,
                                                sysinterval_t timeout) {

  return chMBFetchTimeoutS(&ofp->mbx, (msg_t *)objpp, timeout);
}

/**
 * @brief   Fetches an object.
 *
 * @param[in] ofp       pointer to a @p objects_fifo_t structure
 * @param[in] objpp     pointer to the fetched object reference
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if an object has been correctly fetched.
 * @retval MSG_TIMEOUT  if the operation has timed out.
 *
 * @api
 */
static inline msg_t chFifoReceiveObjectTimeout(objects_fifo_t *ofp,
                                               void **objpp,
                                               sysinterval_t timeout) {

  return chMBFetchTimeout(&ofp->mbx, (msg_t *)objpp, timeout);
}

#endif /* CH_CFG_USE_OBJ_FIFOS == TRUE */

#endif /* CHOBJFIFOS_H */

/** @} */
