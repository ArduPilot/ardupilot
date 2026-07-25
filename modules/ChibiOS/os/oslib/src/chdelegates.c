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
 * @file    oslib/src/chdelegates.c
 * @brief   Delegate threads code.
 * @details Delegate threads.
 *          <h2>Operation mode</h2>
 *          A delegate thread is a thread performing function calls triggered
 *          by other threads. This functionality is especially useful when
 *          encapsulating a library not designed for threading into a
 *          delegate thread. Other threads have access to the library without
 *          having to worry about mutual exclusion.
 * @pre     In order to use the delegates APIs the @p CH_CFG_USE_DELEGATES
 *          option must be enabled in @p chconf.h.
 * @note    Compatible with RT and NIL.
 *
 * @addtogroup oslib_delegates
 * @{
 */

#include "ch.h"

#if (CH_CFG_USE_DELEGATES == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing a delegate call.
 */
typedef struct {
  /**
   * @brief   The delegate veneer function.
   */
  delegate_veneer_t veneer;
  /**
   * @brief   Pointer to the caller @p va_list object.
   */
  va_list           *argsp;
} call_message_t;

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/*lint -save -e586 [17.1] Required by design.*/

/**
 * @brief   Veneer for functions with no parameters.
 *
 * @param[in] argsp     the list of arguments
 * @return              The function return value.
 */
msg_t __ch_delegate_fn0(va_list *argsp) {
  delegate_fn0_t fn0 = (delegate_fn0_t)va_arg(*argsp, delegate_fn0_t);
  return fn0();
}

/**
 * @brief   Veneer for functions with one parameter.
 *
 * @param[in] argsp     the list of arguments
 * @return              The function return value.
 */
msg_t __ch_delegate_fn1(va_list *argsp) {
  delegate_fn1_t fn1 = (delegate_fn1_t)va_arg(*argsp, delegate_fn1_t);
  msg_t p1 = (msg_t)va_arg(*argsp, msg_t);
  return fn1(p1);
}

/**
 * @brief   Veneer for functions with two parameters.
 *
 * @param[in] argsp     the list of arguments
 * @return              The function return value.
 */
msg_t __ch_delegate_fn2(va_list *argsp) {
  delegate_fn2_t fn2 = (delegate_fn2_t)va_arg(*argsp, delegate_fn2_t);
  msg_t p1 = (msg_t)va_arg(*argsp, msg_t);
  msg_t p2 = (msg_t)va_arg(*argsp, msg_t);
  return fn2(p1, p2);
}

/**
 * @brief   Veneer for functions with three parameters.
 *
 * @param[in] argsp     the list of arguments
 * @return              The function return value.
 */
msg_t __ch_delegate_fn3(va_list *argsp) {
  delegate_fn3_t fn3 = (delegate_fn3_t)va_arg(*argsp, delegate_fn3_t);
  msg_t p1 = (msg_t)va_arg(*argsp, msg_t);
  msg_t p2 = (msg_t)va_arg(*argsp, msg_t);
  msg_t p3 = (msg_t)va_arg(*argsp, msg_t);
  return fn3(p1, p2, p3);
}

/**
 * @brief   Veneer for functions with four parameters.
 *
 * @param[in] argsp     the list of arguments
 * @return              The function return value.
 */
msg_t __ch_delegate_fn4(va_list *argsp) {
  delegate_fn4_t fn4 = (delegate_fn4_t)va_arg(*argsp, delegate_fn4_t);
  msg_t p1 = (msg_t)va_arg(*argsp, msg_t);
  msg_t p2 = (msg_t)va_arg(*argsp, msg_t);
  msg_t p3 = (msg_t)va_arg(*argsp, msg_t);
  msg_t p4 = (msg_t)va_arg(*argsp, msg_t);
  return fn4(p1, p2, p3, p4);
}

/**
 * @brief   Triggers a function call on a delegate thread.
 * @note    The thread must be executing @p chDelegateDispatchTimeout() in
 *          order to have the functions called.
 *
 * @param[in] tp        pointer to the delegate thread
 * @param[in] veneer    pointer to the veneer function to be called
 * @param[in] ...       variable number of parameters
 * @return              The function return value casted to msg_t. It is
 *                      garbage for functions returning @p void.
 */
msg_t chDelegateCallVeneer(thread_t *tp, delegate_veneer_t veneer, ...) {
  va_list args;
  call_message_t cm;
  msg_t msg;

  chDbgCheck((tp != NULL) && (veneer != NULL));

  va_start(args, veneer);

  /* Preparing the call message.*/
  cm.veneer = veneer;
  cm.argsp  = &args;

  /* Sending the message to the dispatcher thread, the return value is
     contained in the returned message.*/
  msg = chMsgSend(tp, (msg_t)&cm);

  va_end(args);

  return msg;
}

/*lint -restore*/

/**
 * @brief   Call messages dispatching.
 * @details The function awaits for an incoming call messages and calls the
 *          specified functions, then it returns. In case multiple threads
 *          are sending messages then the requests are served in priority
 *          order.
 *
 * @api
 */
void chDelegateDispatch(void) {
  thread_t *tp;
  const call_message_t *cmp;
  msg_t ret;

  tp = chMsgWait();
  cmp = (const call_message_t *)chMsgGet(tp);
  ret = cmp->veneer(cmp->argsp);

  chMsgRelease(tp, ret);
}

/**
 * @brief   Call messages dispatching with timeout.
 * @details The function awaits for an incoming call messages and calls the
 *          specified functions, then it returns. In case multiple threads
 *          are sending messages then the requests are served in priority
 *          order.
 *
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The function outcome.
 * @retval MSG_OK       if a function has been called.
 * @retval MSG_TIMEOUT  if a timeout occurred.
 *
 * @api
 */
msg_t chDelegateDispatchTimeout(sysinterval_t timeout) {
  thread_t *tp;
  const call_message_t *cmp;
  msg_t ret;

  tp = chMsgWaitTimeout(timeout);
  if (tp == NULL) {
    return MSG_TIMEOUT;
  }

  cmp = (const call_message_t *)chMsgGet(tp);
  ret = cmp->veneer(cmp->argsp);

  chMsgRelease(tp, ret);

  return MSG_OK;
}

#endif /* CH_CFG_USE_DELEGATES == TRUE */

/** @} */
