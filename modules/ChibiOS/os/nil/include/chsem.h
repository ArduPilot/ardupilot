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
 * @file    nil/include/chsem.h
 * @brief   Nil RTOS semaphores header file.
 *
 * @addtogroup NIL_SEMAPHORES
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

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @name    Semaphores macros
 * @{
 */
/**
 * @brief   Data part of a static semaphore initializer.
 * @details This macro should be used when statically initializing a semaphore
 *          that is part of a bigger structure.
 *
 * @param[in] name      the name of the semaphore variable
 * @param[in] n         the counter initial value, this value must be
 *                      non-negative
 */
#define __SEMAPHORE_DATA(name, n) {n}

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
/** @} */

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Initializes a semaphore with the specified counter value.
 *
 * @param[out] sp       pointer to a @p semaphore_t structure
 * @param[in] n         initial value of the semaphore counter. Must be
 *                      non-negative.
 *
 * @init
 */
#define chSemObjectInit(sp, n) ((sp)->cnt = (n))

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
#define chSemReset(sp, n) chSemResetWithMessage(sp, n, MSG_RESET)

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
#define chSemResetI(sp, n) chSemResetWithMessageI(sp, n, MSG_RESET)

/**
 * @brief   Performs a wait operation on a semaphore.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 * @return              A message specifying how the invoking thread has been
 *                      released from the semaphore.
 * @retval CH_MSG_OK   if the thread has not stopped on the semaphore or the
 *                      semaphore has been signaled.
 * @retval CH_MSG_RST  if the semaphore has been reset using @p chSemReset().
 *
 * @api
 */
#define chSemWait(sp) chSemWaitTimeout(sp, TIME_INFINITE)

/**
 * @brief   Performs a wait operation on a semaphore.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 * @return              A message specifying how the invoking thread has been
 *                      released from the semaphore.
 * @retval CH_MSG_OK   if the thread has not stopped on the semaphore or the
 *                      semaphore has been signaled.
 * @retval CH_MSG_RST  if the semaphore has been reset using @p chSemReset().
 *
 * @sclass
 */
#define chSemWaitS(sp) chSemWaitTimeoutS(sp, TIME_INFINITE)

/**
 * @brief   Decreases the semaphore counter.
 * @details This macro can be used when the counter is known to be positive.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 *
 * @iclass
 */
#define chSemFastWaitI(sp) ((sp)->cnt--)

/**
 * @brief   Increases the semaphore counter.
 * @details This macro can be used when the counter is known to be not
 *          negative.
 *
 * @param[in] sp        pointer to a @p semaphore_t structure
 *
 * @iclass
 */
#define chSemFastSignalI(sp) ((sp)->cnt++)

/**
 * @brief   Returns the semaphore counter current value.
 *
 * @iclass
 */
#define chSemGetCounterI(sp) ((sp)->cnt)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  msg_t chSemWaitTimeout(semaphore_t *sp, sysinterval_t timeout);
  msg_t chSemWaitTimeoutS(semaphore_t *sp, sysinterval_t timeout);
  void chSemSignal(semaphore_t *sp);
  void chSemSignalI(semaphore_t *sp);
  void chSemResetWithMessage(semaphore_t *sp, cnt_t n, msg_t msg);
  void chSemResetWithMessageI(semaphore_t *sp, cnt_t n, msg_t msg);
#ifdef __cplusplus
}
#endif

#endif /* CH_CFG_USE_SEMAPHORES == TRUE */

#endif /* CHSEM_H */

/** @} */
