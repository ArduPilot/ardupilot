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
 * @file    oslib/include/chdelegates.h
 * @brief   Delegate threads macros and structures.
 *
 * @addtogroup oslib_delegates
 * @{
 */

#ifndef CHDELEGATES_H
#define CHDELEGATES_H

#if (CH_CFG_USE_DELEGATES == TRUE) || defined(__DOXYGEN__)

/*lint -save -e829 [17.1] Required by design.*/
#include <stdarg.h>
/*lint -restore*/

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if CH_CFG_USE_MESSAGES == FALSE
#error "CH_CFG_USE_DELEGATES requires CH_CFG_USE_MESSAGES"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a delegate veneer function.
 */
typedef msg_t (*delegate_veneer_t)(va_list *argsp);

/**
 * @brief   Type of a delegate function with no parameters.
 */
typedef msg_t (*delegate_fn0_t)(void);

/**
 * @brief   Type of a delegate function with one parameter.
 */
typedef msg_t (*delegate_fn1_t)(msg_t p1);

/**
 * @brief   Type of a delegate function with two parameters.
 */
typedef msg_t (*delegate_fn2_t)(msg_t p1, msg_t p2);

/**
 * @brief   Type of a delegate function with three parameters.
 */
typedef msg_t (*delegate_fn3_t)(msg_t p1, msg_t p2, msg_t p3);

/**
 * @brief   Type of a delegate function with four parameters.
 */
typedef msg_t (*delegate_fn4_t)(msg_t p1, msg_t p2, msg_t p3, msg_t p4);

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  msg_t __ch_delegate_fn0(va_list *argsp);
  msg_t __ch_delegate_fn1(va_list *argsp);
  msg_t __ch_delegate_fn2(va_list *argsp);
  msg_t __ch_delegate_fn3(va_list *argsp);
  msg_t __ch_delegate_fn4(va_list *argsp);
  void chDelegateDispatch(void);
  msg_t chDelegateDispatchTimeout(sysinterval_t timeout);
  msg_t chDelegateCallVeneer(thread_t *tp, delegate_veneer_t veneer, ...);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Direct call to a function with no parameters.
 * @note    The return value is assumed to be not larger than a data
 *          pointer type. If you need a portable function then use
 *          @p chDelegateCallVeneer() instead.
 *
 * @param[in] tp        pointer to the delegate thread
 * @param[in] func      pointer to the function to be called
 * @return              The function return value as a @p msg_t.
 */
static inline msg_t chDelegateCallDirect0(thread_t *tp, delegate_fn0_t func) {

  return chDelegateCallVeneer(tp, __ch_delegate_fn0, func);
}

/**
 * @brief   Direct call to a function with one parameter.
 * @note    The return value and parameters are assumed to be not larger
 *          than a data pointer type. If you need a portable function then use
 *          @p chDelegateCallVeneer() instead.
 *
 * @param[in] tp        pointer to the delegate thread
 * @param[in] func      pointer to the function to be called
 * @param[in] p1        parameter 1 passed as a @p msg_t
 * @return              The function return value as a @p msg_t.
 */
static inline msg_t chDelegateCallDirect1(thread_t *tp, delegate_fn1_t func,
                                          msg_t p1) {

  return chDelegateCallVeneer(tp, __ch_delegate_fn1, func, p1);
}

/**
 * @brief   Direct call to a function with two parameters.
 * @note    The return value and parameters are assumed to be not larger
 *          than a data pointer type. If you need a portable function then use
 *          @p chDelegateCallVeneer() instead.
 *
 * @param[in] tp        pointer to the delegate thread
 * @param[in] func      pointer to the function to be called
 * @param[in] p1        parameter 1 passed as a @p msg_t
 * @param[in] p2        parameter 2 passed as a @p msg_t
 * @return              The function return value as a @p msg_t.
 */
static inline msg_t chDelegateCallDirect2(thread_t *tp, delegate_fn2_t func,
                                          msg_t p1, msg_t p2) {

  return chDelegateCallVeneer(tp, __ch_delegate_fn2, func, p1, p2);
}

/**
 * @brief   Direct call to a function with three parameters.
 * @note    The return value and parameters are assumed to be not larger
 *          than a data pointer type. If you need a portable function then use
 *          @p chDelegateCallVeneer() instead.
 *
 * @param[in] tp        pointer to the delegate thread
 * @param[in] func      pointer to the function to be called
 * @param[in] p1        parameter 1 passed as a @p msg_t
 * @param[in] p2        parameter 2 passed as a @p msg_t
 * @param[in] p3        parameter 3 passed as a @p msg_t
 * @return              The function return value as a @p msg_t.
 */
static inline msg_t chDelegateCallDirect3(thread_t *tp, delegate_fn3_t func,
                                          msg_t p1, msg_t p2, msg_t p3) {

  return chDelegateCallVeneer(tp, __ch_delegate_fn3, func, p1, p2, p3);
}

/**
 * @brief   Direct call to a function with four parameters.
 * @note    The return value and parameters are assumed to be not larger
 *          than a data pointer type. If you need a portable function then use
 *          @p chDelegateCallVeneer() instead.
 *
 * @param[in] tp        pointer to the delegate thread
 * @param[in] func      pointer to the function to be called
 * @param[in] p1        parameter 1 passed as a @p msg_t
 * @param[in] p2        parameter 2 passed as a @p msg_t
 * @param[in] p3        parameter 3 passed as a @p msg_t
 * @param[in] p4        parameter 4 passed as a @p msg_t
 * @return              The function return value as a @p msg_t.
 */
static inline msg_t chDelegateCallDirect4(thread_t *tp, delegate_fn4_t func,
                                          msg_t p1, msg_t p2, msg_t p3,
                                          msg_t p4) {

  return chDelegateCallVeneer(tp, __ch_delegate_fn4, func, p1, p2, p3, p4);
}

#endif /* CH_CFG_USE_DELEGATES == TRUE */

#endif /* CHDELEGATES_H */

/** @} */
