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
 * @file    rt/src/chmtx.c
 * @brief   Mutexes code.
 *
 * @addtogroup mutexes
 * @details Mutexes related APIs and services.
 *          <h2>Operation mode</h2>
 *          A mutex is a threads synchronization object that can be in two
 *          distinct states:
 *          - Not owned (unlocked).
 *          - Owned by a thread (locked).
 *          .
 *          Operations defined for mutexes:
 *          - <b>Lock</b>: The mutex is checked, if the mutex is not owned by
 *            some other thread then it is associated to the locking thread
 *            else the thread is queued on the mutex in a list ordered by
 *            priority.
 *          - <b>Unlock</b>: The mutex is released by the owner and the highest
 *            priority thread waiting in the queue, if any, is resumed and made
 *            owner of the mutex.
 *          .
 *          <h2>Constraints</h2>
 *          In ChibiOS/RT the Unlock operations must always be performed
 *          in lock-reverse order. This restriction both improves the
 *          performance and is required for an efficient implementation
 *          of the priority inheritance mechanism.<br>
 *          Operating under this restriction also ensures that deadlocks
 *          are no possible.
 *
 *          <h2>Recursive mode</h2>
 *          By default mutexes are not recursive, this mean that it is not
 *          possible to take a mutex already owned by the same thread.
 *          It is possible to enable the recursive behavior by enabling the
 *          option @p CH_CFG_USE_MUTEXES_RECURSIVE.
 *
 *          <h2>The priority inversion problem</h2>
 *          The mutexes in ChibiOS/RT implements the <b>full</b> priority
 *          inheritance mechanism in order handle the priority inversion
 *          problem.<br>
 *          When a thread is queued on a mutex, any thread, directly or
 *          indirectly, holding the mutex gains the same priority of the
 *          waiting thread (if their priority was not already equal or higher).
 *          The mechanism works with any number of nested mutexes and any
 *          number of involved threads. The algorithm complexity (worst case)
 *          is N with N equal to the number of nested mutexes.
 * @pre     In order to use the mutex APIs the @p CH_CFG_USE_MUTEXES option
 *          must be enabled in @p chconf.h.
 * @post    Enabling mutexes requires 5-12 (depending on the architecture)
 *          extra bytes in the @p thread_t structure.
 * @{
 */

#include "ch.h"

#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)

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
 * @brief   Initializes s @p mutex_t structure.
 *
 * @param[out] mp       pointer to a @p mutex_t structure
 *
 * @init
 */
void chMtxObjectInit(mutex_t *mp) {

  chDbgCheck(mp != NULL);

  ch_queue_init(&mp->queue);
  mp->owner = NULL;
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
  mp->cnt = (cnt_t)0;
#endif
}

/**
 * @brief   Locks the specified mutex.
 * @post    The mutex is locked and inserted in the per-thread stack of owned
 *          mutexes.
 *
 * @param[in] mp        pointer to the @p mutex_t structure
 *
 * @api
 */
void chMtxLock(mutex_t *mp) {

  chSysLock();
  chMtxLockS(mp);
  chSysUnlock();
}

/**
 * @brief   Locks the specified mutex.
 * @post    The mutex is locked and inserted in the per-thread stack of owned
 *          mutexes.
 *
 * @param[in] mp        pointer to the @p mutex_t structure
 *
 * @sclass
 */
void chMtxLockS(mutex_t *mp) {
  thread_t *currtp = chThdGetSelfX();

  chDbgCheckClassS();
  chDbgCheck(mp != NULL);

  /* Is the mutex already locked? */
  if (mp->owner != NULL) {
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE

    chDbgAssert(mp->cnt >= (cnt_t)1, "counter is not positive");

    /* If the mutex is already owned by this thread, the counter is increased
       and there is no need of more actions.*/
    if (mp->owner == currtp) {
      mp->cnt++;
    }
    else {
#endif
      /* Priority inheritance protocol; explores the thread-mutex dependencies
         boosting the priority of all the affected threads to equal the
         priority of the running thread requesting the mutex.*/
      thread_t *tp = mp->owner;

      /* Does the running thread have higher priority than the mutex
         owning thread? */
      while (tp->hdr.pqueue.prio < currtp->hdr.pqueue.prio) {
        /* Make priority of thread tp match the running thread's priority.*/
        tp->hdr.pqueue.prio = currtp->hdr.pqueue.prio;

        /* The following states need priority queues reordering.*/
        switch (tp->state) {
        case CH_STATE_WTMTX:
          /* Re-enqueues the mutex owner with its new priority.*/
          ch_sch_prio_insert(&tp->u.wtmtxp->queue,
                             ch_queue_dequeue(&tp->hdr.queue));
          tp = tp->u.wtmtxp->owner;
          /*lint -e{9042} [16.1] Continues the while.*/
          continue;
#if (CH_CFG_USE_CONDVARS == TRUE) ||                                        \
    ((CH_CFG_USE_SEMAPHORES == TRUE) &&                                     \
     (CH_CFG_USE_SEMAPHORES_PRIORITY == TRUE))
#if CH_CFG_USE_CONDVARS == TRUE
        case CH_STATE_WTCOND:
#endif
#if (CH_CFG_USE_SEMAPHORES == TRUE) &&                                      \
    (CH_CFG_USE_SEMAPHORES_PRIORITY == TRUE)
        case CH_STATE_WTSEM:
#endif
          /* Re-enqueues tp with its new priority on the queue.*/
          ch_sch_prio_insert(&tp->u.wtmtxp->queue,
                             ch_queue_dequeue(&tp->hdr.queue));
          break;
#endif
#if (CH_CFG_USE_MESSAGES == TRUE) &&                                        \
    (CH_CFG_USE_MESSAGES_PRIORITY == TRUE)
        case CH_STATE_SNDMSGQ:
          /* Re-enqueues tp using the receiver message queue back-pointer. */
          ch_sch_prio_insert((ch_queue_t *)tp->u.wtobjp,
                             ch_queue_dequeue(&tp->hdr.queue));
          break;
#endif
        case CH_STATE_READY:
#if CH_DBG_ENABLE_ASSERTS == TRUE
          /* Prevents an assertion in chSchReadyI().*/
          tp->state = CH_STATE_CURRENT;
#endif
          /* Re-enqueues tp with its new priority on the ready list.*/
          (void) chSchReadyI(threadref(ch_queue_dequeue(&tp->hdr.queue)));
          break;
        default:
          /* Nothing to do for other states.*/
          break;
        }
        break;
      }

      /* Sleep on the mutex.*/
      ch_sch_prio_insert(&mp->queue, &currtp->hdr.queue);
      currtp->u.wtmtxp = mp;
      chSchGoSleepS(CH_STATE_WTMTX);

      /* It is assumed that the thread performing the unlock operation assigns
         the mutex to this thread.*/
      chDbgAssert(mp->owner == currtp, "not owner");
      chDbgAssert(currtp->mtxlist == mp, "not owned");
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
      chDbgAssert(mp->cnt == (cnt_t)1, "counter is not one");
    }
#endif
  }
  else {
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
    chDbgAssert(mp->cnt == (cnt_t)0, "counter is not zero");

    mp->cnt++;
#endif
    /* It was not owned, inserted in the owned mutexes list.*/
    mp->owner = currtp;
    mp->next = currtp->mtxlist;
    currtp->mtxlist = mp;
  }
}

/**
 * @brief   Tries to lock a mutex.
 * @details This function attempts to lock a mutex, if the mutex is already
 *          locked by another thread then the function exits without waiting.
 * @post    The mutex is locked and inserted in the per-thread stack of owned
 *          mutexes.
 * @note    This function does not have any overhead related to the
 *          priority inheritance mechanism because it does not try to
 *          enter a sleep state.
 *
 * @param[in] mp        pointer to the @p mutex_t structure
 * @return              The operation status.
 * @retval true         if the mutex has been successfully acquired
 * @retval false        if the lock attempt failed.
 *
 * @api
 */
bool chMtxTryLock(mutex_t *mp) {
  bool b;

  chSysLock();
  b = chMtxTryLockS(mp);
  chSysUnlock();

  return b;
}

/**
 * @brief   Tries to lock a mutex.
 * @details This function attempts to lock a mutex, if the mutex is already
 *          taken by another thread then the function exits without waiting.
 * @post    The mutex is locked and inserted in the per-thread stack of owned
 *          mutexes.
 * @note    This function does not have any overhead related to the
 *          priority inheritance mechanism because it does not try to
 *          enter a sleep state.
 *
 * @param[in] mp        pointer to the @p mutex_t structure
 * @return              The operation status.
 * @retval true         if the mutex has been successfully acquired
 * @retval false        if the lock attempt failed.
 *
 * @sclass
 */
bool chMtxTryLockS(mutex_t *mp) {
  thread_t *currtp = chThdGetSelfX();

  chDbgCheckClassS();
  chDbgCheck(mp != NULL);

  if (mp->owner != NULL) {
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE

    chDbgAssert(mp->cnt >= (cnt_t)1, "counter is not positive");

    if (mp->owner == currtp) {
      mp->cnt++;
      return true;
    }
#endif
    return false;
  }
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE

  chDbgAssert(mp->cnt == (cnt_t)0, "counter is not zero");

  mp->cnt++;
#endif
  mp->owner = currtp;
  mp->next = currtp->mtxlist;
  currtp->mtxlist = mp;
  return true;
}

/**
 * @brief   Unlocks the specified mutex.
 * @note    Mutexes must be unlocked in reverse lock order. Violating this
 *          rules will result in a panic if assertions are enabled.
 * @pre     The invoking thread <b>must</b> have at least one owned mutex.
 * @post    The mutex is unlocked and removed from the per-thread stack of
 *          owned mutexes.
 *
 * @param[in] mp        pointer to the @p mutex_t structure
 *
 * @api
 */
void chMtxUnlock(mutex_t *mp) {
  thread_t *currtp = chThdGetSelfX();
  mutex_t *lmp;

  if (mp->owner != currtp) {
      /*
        this can only happen if we have force relased a mutex
       */
      return;
  }

  chDbgCheck(mp != NULL);

  chSysLock();

  chDbgAssert(currtp->mtxlist != NULL, "owned mutexes list empty");
  chDbgAssert(currtp->mtxlist->owner == currtp, "ownership failure");
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
  chDbgAssert(mp->cnt >= (cnt_t)1, "counter is not positive");

  if (--mp->cnt == (cnt_t)0) {
#endif

    chDbgAssert(currtp->mtxlist == mp, "not next in list");

    /* Removes the top mutex from the thread's owned mutexes list and marks
       it as not owned. Note, it is assumed to be the same mutex passed as
       parameter of this function.*/
    currtp->mtxlist = mp->next;

    /* If a thread is waiting on the mutex then the fun part begins.*/
    if (chMtxQueueNotEmptyS(mp)) {
      thread_t *tp;

      /* Recalculates the optimal thread priority by scanning the owned
         mutexes list.*/
      tprio_t newprio = currtp->realprio;
      lmp = currtp->mtxlist;
      while (lmp != NULL) {
        /* If the highest priority thread waiting in the mutexes list has a
           greater priority than the current thread base priority then the
           final priority will have at least that priority.*/
        if (chMtxQueueNotEmptyS(lmp) &&
            ((threadref(lmp->queue.next))->hdr.pqueue.prio > newprio)) {
          newprio = (threadref(lmp->queue.next))->hdr.pqueue.prio;
        }
        lmp = lmp->next;
      }

      /* Assigns to the current thread the highest priority among all the
         waiting threads.*/
      currtp->hdr.pqueue.prio = newprio;

      /* Awakens the highest priority thread waiting for the unlocked mutex and
         assigns the mutex to it.*/
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
      mp->cnt = (cnt_t)1;
#endif
      tp = threadref(ch_queue_fifo_remove(&mp->queue));
      mp->owner = tp;
      mp->next = tp->mtxlist;
      tp->mtxlist = mp;

      /* Note, not using chSchWakeupS() because that function expects the
         current thread to have the higher or equal priority than the ones
         in the ready list. This is not necessarily true here because we
         just changed priority.*/
      (void) chSchReadyI(tp);
      chSchRescheduleS();
    }
    else {
      mp->owner = NULL;
    }
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
  }
#endif

  chSysUnlock();
}

/**
 * @brief   Unlocks the specified mutex.
 * @note    Mutexes must be unlocked in reverse lock order. Violating this
 *          rules will result in a panic if assertions are enabled.
 * @pre     The invoking thread <b>must</b> have at least one owned mutex.
 * @post    The mutex is unlocked and removed from the per-thread stack of
 *          owned mutexes.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel.
 *
 * @param[in] mp        pointer to the @p mutex_t structure
 *
 * @sclass
 */
void chMtxUnlockS(mutex_t *mp) {
  thread_t *currtp = chThdGetSelfX();
  mutex_t *lmp;

  if (mp->owner != currtp) {
      /*
        this can only happen if we have force relased a mutex
       */
      return;
  }

  chDbgCheckClassS();
  chDbgCheck(mp != NULL);

  chDbgAssert(currtp->mtxlist != NULL, "owned mutexes list empty");
  chDbgAssert(currtp->mtxlist->owner == currtp, "ownership failure");
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
  chDbgAssert(mp->cnt >= (cnt_t)1, "counter is not positive");

  if (--mp->cnt == (cnt_t)0) {
#endif

    chDbgAssert(currtp->mtxlist == mp, "not next in list");

    /* Removes the top mutex from the thread's owned mutexes list and marks
       it as not owned. Note, it is assumed to be the same mutex passed as
       parameter of this function.*/
    currtp->mtxlist = mp->next;

    /* If a thread is waiting on the mutex then the fun part begins.*/
    if (chMtxQueueNotEmptyS(mp)) {
      thread_t *tp;

      /* Recalculates the optimal thread priority by scanning the owned
         mutexes list.*/
      tprio_t newprio = currtp->realprio;
      lmp = currtp->mtxlist;
      while (lmp != NULL) {
        /* If the highest priority thread waiting in the mutexes list has a
           greater priority than the current thread base priority then the
           final priority will have at least that priority.*/
        if (chMtxQueueNotEmptyS(lmp) &&
            ((threadref(lmp->queue.next))->hdr.pqueue.prio > newprio)) {
          newprio = threadref(lmp->queue.next)->hdr.pqueue.prio;
        }
        lmp = lmp->next;
      }

      /* Assigns to the current thread the highest priority among all the
         waiting threads.*/
      currtp->hdr.pqueue.prio = newprio;

      /* Awakens the highest priority thread waiting for the unlocked mutex and
         assigns the mutex to it.*/
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
      mp->cnt = (cnt_t)1;
#endif
      tp = threadref(ch_queue_fifo_remove(&mp->queue));
      mp->owner = tp;
      mp->next = tp->mtxlist;
      tp->mtxlist = mp;
      (void) chSchReadyI(tp);
    }
    else {
      mp->owner = NULL;
    }
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
  }
#endif
}

/**
 * @brief   Unlocks all mutexes owned by the invoking thread.
 * @post    The stack of owned mutexes is emptied and all the found
 *          mutexes are unlocked.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel.
 * @note    This function is <b>MUCH MORE</b> efficient than releasing the
 *          mutexes one by one and not just because the call overhead,
 *          this function does not have any overhead related to the priority
 *          inheritance mechanism.
 *
 * @sclass
 */
void chMtxUnlockAllS(void) {
  thread_t *currtp = chThdGetSelfX();

  if (currtp->mtxlist != NULL) {
    do {
      mutex_t *mp = currtp->mtxlist;
      currtp->mtxlist = mp->next;
      if (chMtxQueueNotEmptyS(mp)) {
        thread_t *tp;
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
        mp->cnt = (cnt_t)1;
#endif
        tp = threadref(ch_queue_fifo_remove(&mp->queue));
        mp->owner   = tp;
        mp->next    = tp->mtxlist;
        tp->mtxlist = mp;
        (void) chSchReadyI(tp);
      }
      else {
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
        mp->cnt = (cnt_t)0;
#endif
        mp->owner = NULL;
      }
    } while (currtp->mtxlist != NULL);
    currtp->hdr.pqueue.prio = currtp->realprio;
    chSchRescheduleS();
  }
}

/**
 * @brief   Unlocks all mutexes owned by the invoking thread.
 * @post    The stack of owned mutexes is emptied and all the found
 *          mutexes are unlocked.
 * @note    This function is <b>MUCH MORE</b> efficient than releasing the
 *          mutexes one by one and not just because the call overhead,
 *          this function does not have any overhead related to the priority
 *          inheritance mechanism.
 *
 * @api
 */
void chMtxUnlockAll(void) {

  chSysLock();
  chMtxUnlockAllS();
  chSysUnlock();
}

/**
 * @brief  Force release a mutex
 * @note   Use of this function violates locking principles and can result in data corruption.
 *         The purpose of this function is to allow some level of recovery from a deadlock
 *         caused by a lock ordering violation. Unlike other chMtxUnlockX calls this function
 *         may be called from a thread that does not own the mutex. It is intended to be called from
 *         a high priority supervisor thread that has detected an unrecoverable deadlock on systems
 *         where watchdog recovery is not appropriate.
 *         If recursive locks are enabled then the lock counter is zeroed, forcing complete release
 */
void chMtxForceReleaseS(mutex_t *mp)
{
  chDbgCheckClassS();
  thread_t *currtp = mp->owner;
  chDbgAssert(currtp->mtxlist != NULL, "owned mutexes list empty");
  chDbgAssert(currtp->mtxlist->owner == currtp, "ownership failure");

#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
  mp->cnt = 0;
#endif

  /*
	we need to de-link the mutex from this threads mutex list
  */
  if (currtp->mtxlist == mp) {
	// simple case, this mutex was most recently acquired by the thread
	currtp->mtxlist = mp->next;
  } else {
	for (mutex_t *m = currtp->mtxlist; m; m = m->next) {
	  if (m->next == mp) {
		m->next = mp->next;
	  }
	}
  }

  /* If a thread is waiting on the mutex then the fun part begins.*/
  if (chMtxQueueNotEmptyS(mp)) {
	thread_t *tp;

	/* Recalculates the optimal thread priority by scanning the owned
	   mutexes list.*/
	tprio_t newprio = currtp->realprio;
	mutex_t *lmp = currtp->mtxlist;
	while (lmp != NULL) {
	  /* If the highest priority thread waiting in the mutexes list has a
		 greater priority than the current thread base priority then the
		 final priority will have at least that priority.*/
	  if (chMtxQueueNotEmptyS(lmp) &&
		  ((threadref(lmp->queue.next))->hdr.pqueue.prio > newprio)) {
		newprio = (threadref(lmp->queue.next))->hdr.pqueue.prio;
	  }
	  lmp = lmp->next;
	}

	/* Assigns to the current thread the highest priority among all the
	   waiting threads.*/
	currtp->hdr.pqueue.prio = newprio;

	/* Awakens the highest priority thread waiting for the unlocked mutex and
	   assigns the mutex to it.*/
#if CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
	mp->cnt = (cnt_t)1;
#endif
	tp = threadref(ch_queue_fifo_remove(&mp->queue));
	mp->owner = tp;
	mp->next = tp->mtxlist;
	tp->mtxlist = mp;

	/* Note, not using chSchWakeupS() because that function expects the
	   current thread to have the higher or equal priority than the ones
	   in the ready list. This is not necessarily true here because we
	   just changed priority.*/
	(void) chSchReadyI(tp);
	chSchRescheduleS();
  }
  else {
	mp->owner = NULL;
  }
}

#endif /* CH_CFG_USE_MUTEXES == TRUE */

/** @} */
