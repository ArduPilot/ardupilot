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
/*
   Concepts and parts of this file have been contributed by Leon Woestenberg.
 */

/**
 * @file    rt/src/chcond.c
 * @brief   Condition Variables code.
 *
 * @addtogroup condvars
 * @details This module implements the Condition Variables mechanism. Condition
 *          variables are an extensions to the mutex subsystem and cannot
 *          work alone.
 *          <h2>Operation mode</h2>
 *          The condition variable is a synchronization object meant to be
 *          used inside a zone protected by a mutex. Mutexes and condition
 *          variables together can implement a Monitor construct.
 * @pre     In order to use the condition variable APIs the @p CH_CFG_USE_CONDVARS
 *          option must be enabled in @p chconf.h.
 * @{
 */

#include "ch.h"

#if (CH_CFG_USE_CONDVARS == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

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
 * @brief   Initializes s @p condition_variable_t structure.
 *
 * @param[out] cp       pointer to a @p condition_variable_t structure
 *
 * @init
 */
void chCondObjectInit(condition_variable_t *cp) {

  chDbgCheck(cp != NULL);

  ch_queue_init(&cp->queue);
}

/**
 * @brief   Signals one thread that is waiting on the condition variable.
 *
 * @param[in] cp        pointer to the @p condition_variable_t structure
 *
 * @api
 */
void chCondSignal(condition_variable_t *cp) {

  chDbgCheck(cp != NULL);

  chSysLock();
  if (ch_queue_notempty(&cp->queue)) {
    chSchWakeupS(threadref(ch_queue_fifo_remove(&cp->queue)), MSG_OK);
  }
  chSysUnlock();
}

/**
 * @brief   Signals one thread that is waiting on the condition variable.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel. Note that
 *          interrupt handlers always reschedule on exit so an explicit
 *          reschedule must not be performed in ISRs.
 *
 * @param[in] cp        pointer to the @p condition_variable_t structure
 *
 * @iclass
 */
void chCondSignalI(condition_variable_t *cp) {

  chDbgCheckClassI();
  chDbgCheck(cp != NULL);

  if (ch_queue_notempty(&cp->queue)) {
    thread_t *tp = threadref(ch_queue_fifo_remove(&cp->queue));
    tp->u.rdymsg = MSG_OK;
    (void) chSchReadyI(tp);
  }
}

/**
 * @brief   Signals all threads that are waiting on the condition variable.
 *
 * @param[in] cp        pointer to the @p condition_variable_t structure
 *
 * @api
 */
void chCondBroadcast(condition_variable_t *cp) {

  chSysLock();
  chCondBroadcastI(cp);
  chSchRescheduleS();
  chSysUnlock();
}

/**
 * @brief   Signals all threads that are waiting on the condition variable.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel. Note that
 *          interrupt handlers always reschedule on exit so an explicit
 *          reschedule must not be performed in ISRs.
 *
 * @param[in] cp        pointer to the @p condition_variable_t structure
 *
 * @iclass
 */
void chCondBroadcastI(condition_variable_t *cp) {

  chDbgCheckClassI();
  chDbgCheck(cp != NULL);

  /* Empties the condition variable queue and inserts all the threads into the
     ready list in FIFO order. The wakeup message is set to @p MSG_RESET in
     order to make a chCondBroadcast() detectable from a chCondSignal().*/
  while (ch_queue_notempty(&cp->queue)) {
    chSchReadyI(threadref(ch_queue_fifo_remove(&cp->queue)))->u.rdymsg = MSG_RESET;
  }
}

/**
 * @brief   Waits on the condition variable releasing the mutex lock.
 * @details Releases the currently owned mutex, waits on the condition
 *          variable, and finally acquires the mutex again. All the sequence
 *          is performed atomically.
 * @pre     The invoking thread <b>must</b> have at least one owned mutex.
 *
 * @param[in] cp        pointer to the @p condition_variable_t structure
 * @return              A message specifying how the invoking thread has been
 *                      released from the condition variable.
 * @retval MSG_OK       if the condition variable has been signaled using
 *                      @p chCondSignal().
 * @retval MSG_RESET    if the condition variable has been signaled using
 *                      @p chCondBroadcast().
 *
 * @api
 */
msg_t chCondWait(condition_variable_t *cp) {
  msg_t msg;

  chSysLock();
  msg = chCondWaitS(cp);
  chSysUnlock();
  return msg;
}

/**
 * @brief   Waits on the condition variable releasing the mutex lock.
 * @details Releases the currently owned mutex, waits on the condition
 *          variable, and finally acquires the mutex again. All the sequence
 *          is performed atomically.
 * @pre     The invoking thread <b>must</b> have at least one owned mutex.
 *
 * @param[in] cp        pointer to the @p condition_variable_t structure
 * @return              A message specifying how the invoking thread has been
 *                      released from the condition variable.
 * @retval MSG_OK       if the condition variable has been signaled using
 *                      @p chCondSignal().
 * @retval MSG_RESET    if the condition variable has been signaled using
 *                      @p chCondBroadcast().
 *
 * @sclass
 */
msg_t chCondWaitS(condition_variable_t *cp) {
  thread_t *currtp = chThdGetSelfX();
  mutex_t *mp = chMtxGetNextMutexX();
  msg_t msg;

  chDbgCheckClassS();
  chDbgCheck(cp != NULL);
  chDbgAssert(mp != NULL, "not owning a mutex");

  /* Releasing "current" mutex.*/
  chMtxUnlockS(mp);

  /* Start waiting on the condition variable, on exit the mutex is taken
     again.*/
  currtp->u.wtobjp = cp;
  ch_sch_prio_insert(&cp->queue, &currtp->hdr.queue);
  chSchGoSleepS(CH_STATE_WTCOND);
  msg = currtp->u.rdymsg;
  chMtxLockS(mp);

  return msg;
}

#if (CH_CFG_USE_CONDVARS_TIMEOUT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Waits on the condition variable releasing the mutex lock.
 * @details Releases the currently owned mutex, waits on the condition
 *          variable, and finally acquires the mutex again. All the sequence
 *          is performed atomically.
 * @pre     The invoking thread <b>must</b> have at least one owned mutex.
 * @pre     The configuration option @p CH_CFG_USE_CONDVARS_TIMEOUT must be enabled
 *          in order to use this function.
 * @post    Exiting the function because a timeout does not re-acquire the
 *          mutex, the mutex ownership is lost.
 *
 * @param[in] cp        pointer to the @p condition_variable_t structure
 * @param[in] timeout   the number of ticks before the operation timeouts, the
 *                      special values are handled as follow:
 *                      - @a TIME_INFINITE no timeout.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 *                      .
 * @return              A message specifying how the invoking thread has been
 *                      released from the condition variable.
 * @retval MSG_OK       if the condition variable has been signaled using
 *                      @p chCondSignal().
 * @retval MSG_RESET    if the condition variable has been signaled using
 *                      @p chCondBroadcast().
 * @retval MSG_TIMEOUT  if the condition variable has not been signaled within
 *                      the specified timeout.
 *
 * @api
 */
msg_t chCondWaitTimeout(condition_variable_t *cp, sysinterval_t timeout) {
  msg_t msg;

  chSysLock();
  msg = chCondWaitTimeoutS(cp, timeout);
  chSysUnlock();

  return msg;
}

/**
 * @brief   Waits on the condition variable releasing the mutex lock.
 * @details Releases the currently owned mutex, waits on the condition
 *          variable, and finally acquires the mutex again. All the sequence
 *          is performed atomically.
 * @pre     The invoking thread <b>must</b> have at least one owned mutex.
 * @pre     The configuration option @p CH_CFG_USE_CONDVARS_TIMEOUT must be enabled
 *          in order to use this function.
 * @post    Exiting the function because a timeout does not re-acquire the
 *          mutex, the mutex ownership is lost.
 *
 * @param[in] cp        pointer to the @p condition_variable_t structure
 * @param[in] timeout   the number of ticks before the operation timeouts, the
 *                      special values are handled as follow:
 *                      - @a TIME_INFINITE no timeout.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 *                      .
 * @return              A message specifying how the invoking thread has been
 *                      released from the condition variable.
 * @retval MSG_OK       if the condition variable has been signaled using
 *                      @p chCondSignal().
 * @retval MSG_RESET    if the condition variable has been signaled using
 *                      @p chCondBroadcast().
 * @retval MSG_TIMEOUT  if the condition variable has not been signaled within
 *                      the specified timeout.
 *
 * @sclass
 */
msg_t chCondWaitTimeoutS(condition_variable_t *cp, sysinterval_t timeout) {
  thread_t *currtp = chThdGetSelfX();
  mutex_t *mp = chMtxGetNextMutexX();
  msg_t msg;

  chDbgCheckClassS();
  chDbgCheck((cp != NULL) && (timeout != TIME_IMMEDIATE));
  chDbgAssert(mp != NULL, "not owning a mutex");

  /* Releasing "current" mutex.*/
  chMtxUnlockS(mp);

  /* Start waiting on the condition variable, on exit the mutex is taken
     again.*/
  currtp->u.wtobjp = cp;
  ch_sch_prio_insert(&cp->queue, &currtp->hdr.queue);
  msg = chSchGoSleepTimeoutS(CH_STATE_WTCOND, timeout);
  if (msg != MSG_TIMEOUT) {
    chMtxLockS(mp);
  }

  return msg;
}
#endif /* CH_CFG_USE_CONDVARS_TIMEOUT == TRUE */

#endif /* CH_CFG_USE_CONDVARS == TRUE */

/** @} */
