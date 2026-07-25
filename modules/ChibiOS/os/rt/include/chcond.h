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
 * @file    rt/include/chcond.h
 * @brief   Condition Variables macros and structures.
 *
 * @addtogroup condvars
 * @{
 */

#ifndef CHCOND_H
#define CHCOND_H

#if (CH_CFG_USE_CONDVARS == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if CH_CFG_USE_MUTEXES == FALSE
#error "CH_CFG_USE_CONDVARS requires CH_CFG_USE_MUTEXES"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   condition_variable_t structure.
 */
typedef struct condition_variable {
  ch_queue_t            queue;              /**< @brief Condition variable
                                                 threads queue.             */
} condition_variable_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief Data part of a static condition variable initializer.
 * @details This macro should be used when statically initializing a condition
 *          variable that is part of a bigger structure.
 *
 * @param[in] name      the name of the condition variable
 */
#define __CONDVAR_DATA(name) {__CH_QUEUE_DATA(name.queue)}

/**
 * @brief Static condition variable initializer.
 * @details Statically initialized condition variables require no explicit
 *          initialization using @p chCondInit().
 *
 * @param[in] name      the name of the condition variable
 */
#define CONDVAR_DECL(name) condition_variable_t name = __CONDVAR_DATA(name)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void chCondObjectInit(condition_variable_t *cp);
  void chCondSignal(condition_variable_t *cp);
  void chCondSignalI(condition_variable_t *cp);
  void chCondBroadcast(condition_variable_t *cp);
  void chCondBroadcastI(condition_variable_t *cp);
  msg_t chCondWait(condition_variable_t *cp);
  msg_t chCondWaitS(condition_variable_t *cp);
#if CH_CFG_USE_CONDVARS_TIMEOUT == TRUE
  msg_t chCondWaitTimeout(condition_variable_t *cp, sysinterval_t timeout);
  msg_t chCondWaitTimeoutS(condition_variable_t *cp, sysinterval_t timeout);
#endif
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* CH_CFG_USE_CONDVARS == TRUE */

#endif /* CHCOND_H */

/** @} */
