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
 * @file    rt/include/chmtx.h
 * @brief   Mutexes macros and structures.
 *
 * @addtogroup mutexes
 * @{
 */

#ifndef CHMTX_H
#define CHMTX_H

#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)

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
 * @brief   Type of a mutex structure.
 */
typedef struct ch_mutex mutex_t;

/**
 * @brief   Mutex structure.
 */
struct ch_mutex {
  ch_queue_t            queue;      /**< @brief Queue of the threads sleeping
                                                on this mutex.              */
  thread_t              *owner;     /**< @brief Owner @p thread_t pointer or
                                                @p NULL.                    */
  mutex_t               *next;      /**< @brief Next @p mutex_t into an
                                                owner-list or @p NULL.      */
#if (CH_CFG_USE_MUTEXES_RECURSIVE == TRUE) || defined(__DOXYGEN__)
  cnt_t                 cnt;        /**< @brief Mutex recursion counter.    */
#endif
};

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Data part of a static mutex initializer.
 * @details This macro should be used when statically initializing a mutex
 *          that is part of a bigger structure.
 *
 * @param[in] name      the name of the mutex variable
 */
#if (CH_CFG_USE_MUTEXES_RECURSIVE == TRUE) || defined(__DOXYGEN__)
#define __MUTEX_DATA(name) {__CH_QUEUE_DATA(name.queue), NULL, NULL, 0}
#else
#define __MUTEX_DATA(name) {__CH_QUEUE_DATA(name.queue), NULL, NULL}
#endif

/**
 * @brief   Static mutex initializer.
 * @details Statically initialized mutexes require no explicit initialization
 *          using @p chMtxInit().
 *
 * @param[in] name      the name of the mutex variable
 */
#define MUTEX_DECL(name) mutex_t name = __MUTEX_DATA(name)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void chMtxObjectInit(mutex_t *mp);
  void chMtxLock(mutex_t *mp);
  void chMtxLockS(mutex_t *mp);
  bool chMtxTryLock(mutex_t *mp);
  bool chMtxTryLockS(mutex_t *mp);
  void chMtxUnlock(mutex_t *mp);
  void chMtxUnlockS(mutex_t *mp);
  void chMtxUnlockAll(void);
  void chMtxUnlockAllS(void);
  void chMtxForceReleaseS(mutex_t *mp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Returns @p true if the mutex queue contains at least a waiting
 *          thread.
 *
 * @param[out] mp       pointer to a @p mutex_t structure
 * @return              The mutex queue status.
 *
 * @sclass
 */
static inline bool chMtxQueueNotEmptyS(mutex_t *mp) {

  chDbgCheckClassS();

  return ch_queue_notempty(&mp->queue);
}

/**
 * @brief   Returns the mutex owner thread.
 *
 * @param[out] mp       pointer to a @p mutex_t structure
 * @return              The owner thread.
 * @retval NULL         if the mutex is not owned.
 *
 * @iclass
 */
static inline thread_t *chMtxGetOwnerI(mutex_t *mp) {

  chDbgCheckClassI();

  return mp->owner;
}

/**
 * @brief   Returns the next mutex in the mutexes stack of the current thread.
 *
 * @return              A pointer to the next mutex in the stack.
 * @retval NULL         if the stack is empty.
 *
 * @xclass
 */
static inline mutex_t *chMtxGetNextMutexX(void) {

  return chThdGetSelfX()->mtxlist;
}

#endif /* CH_CFG_USE_MUTEXES == TRUE */

#endif /* CHMTX_H */

/** @} */
