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
 * @file    rt/include/chsem.h
 * @brief   Semaphores macros and structures.
 *
 * @addtogroup semaphores
 * @{
 */

#ifndef CHSEM_H
#define CHSEM_H

#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)

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
 * @brief   Semaphore structure.
 */
typedef struct ch_semaphore {
  ch_queue_t            queue;      /**< @brief Queue of the threads sleeping
                                                on this semaphore.          */
  cnt_t                 cnt;        /**< @brief The semaphore counter.      */
} semaphore_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Data part of a static semaphore initializer.
 * @details This macro should be used when statically initializing a semaphore
 *          that is part of a bigger structure.
 *
 * @param[in] name      the name of the semaphore variable
 * @param[in] n         the counter initial value, this value must be
 *                      non-negative
 */
#define __SEMAPHORE_DATA(name, n) {__CH_QUEUE_DATA(name.queue), n}

/**
 * @brief   Static semaphore initializer.
 * @details Statically initialized semaphores require no explicit
 *          initialization using @p chSemInit().
 *
 * @param[in] name      the name of the semaphore variable
 * @param[in] n         the counter initial value, this value must be
 *                      non-negative
 */
#define SEMAPHORE_DECL(name, n) semaphore_t name = __SEMAPHORE_DATA(name, n)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void chSemObjectInit(semaphore_t *sp, cnt_t n);
  void chSemResetWithMessage(semaphore_t *sp, cnt_t n, msg_t msg);
  void chSemResetWithMessageI(semaphore_t *sp, cnt_t n, msg_t msg);
  msg_t chSemWait(semaphore_t *sp);
  msg_t chSemWaitS(semaphore_t *sp);
  msg_t chSemWaitTimeout(semaphore_t *sp, sysinterval_t timeout);
  msg_t chSemWaitTimeoutS(semaphore_t *sp, sysinterval_t timeout);
  void chSemSignal(semaphore_t *sp);
  void chSemSignalI(semaphore_t *sp);
  void chSemAddCounterI(semaphore_t *sp, cnt_t n);
  msg_t chSemSignalWait(semaphore_t *sps, semaphore_t *spw);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Performs a reset operation on the semaphore.
 * @post    After invoking this function all the threads waiting on the
 *          semaphore, if any, are released and the semaphore counter is set
 *          to the specified, non negative, value.
 * @note    This function implicitly sends @p MSG_RESET as message.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 * @param[in] n         the new value of the semaphore counter. The value must
 *                      be non-negative.
 *
 * @api
 */
static inline void chSemReset(semaphore_t *sp, cnt_t n) {

  chSemResetWithMessage(sp, n, MSG_RESET);
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
 * @note    This function implicitly sends @p MSG_RESET as message.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 * @param[in] n         the new value of the semaphore counter. The value must
 *                      be non-negative.
 *
 * @iclass
 */
static inline void chSemResetI(semaphore_t *sp, cnt_t n) {

  chSemResetWithMessageI(sp, n, MSG_RESET);
}

/**
 * @brief   Decreases the semaphore counter.
 * @details This macro can be used when the counter is known to be positive.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 *
 * @iclass
 */
static inline void chSemFastWaitI(semaphore_t *sp) {

  chDbgCheckClassI();

  sp->cnt--;
}

/**
 * @brief   Increases the semaphore counter.
 * @details This macro can be used when the counter is known to be not
 *          negative.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 *
 * @iclass
 */
static inline void chSemFastSignalI(semaphore_t *sp) {

  chDbgCheckClassI();

  sp->cnt++;
}

/**
 * @brief   Returns the semaphore counter current value.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 * @return              The semaphore counter value.
 *
 * @iclass
 */
static inline cnt_t chSemGetCounterI(const semaphore_t *sp) {

  chDbgCheckClassI();

  return sp->cnt;
}

#endif /* CH_CFG_USE_SEMAPHORES == TRUE */

#endif /* CHSEM_H */

/** @} */
