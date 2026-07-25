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
 * @file    rt/src/chregistry.c
 * @brief   Threads registry code.
 *
 * @addtogroup registry
 * @details Threads Registry related APIs and services.
 *          <h2>Operation mode</h2>
 *          The Threads Registry is a double linked list that holds all the
 *          active threads in the system.<br>
 *          Operations defined for the registry:
 *          - <b>First</b>, returns the first, in creation order, active thread
 *            in the system.
 *          - <b>Next</b>, returns the next, in creation order, active thread
 *            in the system.
 *          .
 *          The registry is meant to be mainly a debug feature, for example,
 *          using the registry a debugger can enumerate the active threads
 *          in any given moment or the shell can print the active threads
 *          and their state.<br>
 *          Another possible use is for centralized threads memory management,
 *          terminating threads can pulse an event source and an event handler
 *          can perform a scansion of the registry in order to recover the
 *          memory.
 * @pre     In order to use the threads registry the @p CH_CFG_USE_REGISTRY
 *          option must be enabled in @p chconf.h.
 * @{
 */

#include <string.h>

#include "ch.h"

#if (CH_CFG_USE_REGISTRY == TRUE) || defined(__DOXYGEN__)

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

/*
 * OS signature in ROM plus debug-related information.
 */
ROMCONST chdebug_t ch_debug = {
  .identifier               = {'m', 'a', 'i', 'n'},
  .zero                     = (uint8_t)0,
  .size                     = (uint8_t)sizeof (chdebug_t),
  .version                  = (uint16_t)(((unsigned)CH_KERNEL_MAJOR << 11U) |
                                         ((unsigned)CH_KERNEL_MINOR << 6U) |
                                         ((unsigned)CH_KERNEL_PATCH << 0U)),
  .ptrsize                  = (uint8_t)sizeof (void *),
  .timesize                 = (uint8_t)sizeof (systime_t),
  .intervalsize             = (uint8_t)sizeof (sysinterval_t),
  .threadsize               = (uint8_t)sizeof (thread_t),
  .intctxsize               = (uint8_t)sizeof (struct port_intctx),
  .off_prio                 = (uint8_t)__CH_OFFSETOF(thread_t, hdr.pqueue.prio),
  .off_ctx                  = (uint8_t)__CH_OFFSETOF(thread_t, ctx),
  .off_newer                = (uint8_t)__CH_OFFSETOF(thread_t, rqueue.next),
  .off_older                = (uint8_t)__CH_OFFSETOF(thread_t, rqueue.prev),
  .off_name                 = (uint8_t)__CH_OFFSETOF(thread_t, name),
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
  .off_stklimit             = (uint8_t)__CH_OFFSETOF(thread_t, wabase),
#else
  .off_stklimit             = (uint8_t)0,
#endif
  .off_state                = (uint8_t)__CH_OFFSETOF(thread_t, state),
  .off_flags                = (uint8_t)__CH_OFFSETOF(thread_t, flags),
#if CH_CFG_USE_DYNAMIC == TRUE
  .off_refs                 = (uint8_t)__CH_OFFSETOF(thread_t, refs),
#else
  .off_refs                 = (uint8_t)0,
#endif
#if CH_CFG_TIME_QUANTUM > 0
  .off_preempt              = (uint8_t)__CH_OFFSETOF(thread_t, ticks),
#else
  .off_preempt              = (uint8_t)0,
#endif
#if CH_DBG_THREADS_PROFILING == TRUE
  .off_time                 = (uint8_t)__CH_OFFSETOF(thread_t, time),
#else
  .off_time                 = (uint8_t)0,
#endif
  .off_reserved             = {(uint8_t)0, (uint8_t)0, (uint8_t)0, (uint8_t)0},
  .instancesnum             = (uint8_t)PORT_CORES_NUMBER,
  .off_sys_state            = (uint8_t)__CH_OFFSETOF(ch_system_t, state),
  .off_sys_instances        = (uint8_t)__CH_OFFSETOF(ch_system_t, instances[0]),
#if (CH_CFG_USE_REGISTRY == TRUE) && (CH_CFG_SMP_MODE == TRUE)
  .off_sys_reglist          = (uint8_t)__CH_OFFSETOF(ch_system_t, reglist),
#else
  .off_sys_reglist          = (uint8_t)0,
#endif
#if CH_CFG_SMP_MODE == TRUE
  .off_sys_rfcu             = (uint8_t)__CH_OFFSETOF(ch_system_t, rfcu),
#else
  .off_sys_rfcu             = (uint8_t)0,
#endif
  .off_sys_reserved         = {(uint8_t)0, (uint8_t)0, (uint8_t)0, (uint8_t)0},
  .off_inst_rlist_current   = (uint8_t)__CH_OFFSETOF(os_instance_t, rlist.current),
  .off_inst_rlist           = (uint8_t)__CH_OFFSETOF(os_instance_t, rlist),
  .off_inst_vtlist          = (uint8_t)__CH_OFFSETOF(os_instance_t, vtlist),
#if ((CH_CFG_USE_REGISTRY == TRUE) && (CH_CFG_SMP_MODE == FALSE))
  .off_inst_reglist         = (uint8_t)__CH_OFFSETOF(os_instance_t, reglist),
#else
  .off_inst_reglist         = (uint8_t)0,
#endif
  .off_inst_core_id         = (uint8_t)__CH_OFFSETOF(os_instance_t, core_id),
#if CH_CFG_SMP_MODE == FALSE
  .off_inst_rfcu            = (uint8_t)__CH_OFFSETOF(os_instance_t, rfcu)
#else
  .off_inst_rfcu            = (uint8_t)0
#endif
};

/**
 * @brief   Returns the first thread in the system.
 * @details Returns the most ancient thread in the system, usually this is
 *          the main thread unless it terminated. A reference is added to the
 *          returned thread in order to make sure its status is not lost.
 * @note    This function cannot return @p NULL because there is always at
 *          least one thread in the system.
 *
 * @return              A reference to the most ancient thread.
 *
 * @api
 */
thread_t *chRegFirstThread(void) {
  thread_t *tp;
  uint8_t *p;

  chSysLock();
  p = (uint8_t *)REG_HEADER(currcore)->next;
  /*lint -save -e413 [1.3] Safe to subtract a calculated offset.*/
  tp = threadref((p - __CH_OFFSETOF(thread_t, rqueue)));
  /*lint -restore*/
#if CH_CFG_USE_DYNAMIC == TRUE
  tp->refs++;
#endif
  chSysUnlock();

  return tp;
}

/**
 * @brief   Returns the thread next to the specified one.
 * @details The reference counter of the specified thread is decremented and
 *          the reference counter of the returned thread is incremented.
 *
 * @param[in] tp        pointer to the thread
 * @return              A reference to the next thread.
 * @retval NULL         if there is no next thread.
 *
 * @api
 */
thread_t *chRegNextThread(thread_t *tp) {
  thread_t *ntp;
  ch_queue_t *nqp;

  chSysLock();

  /* Next element in the registry queue.*/
  nqp = tp->rqueue.next;
  if (nqp == REG_HEADER(currcore)) {
    ntp = NULL;
  }
  else {
    uint8_t *p = (uint8_t *)nqp;
    /*lint -save -e413 [1.3] Safe to subtract a calculated offset.*/
    ntp = threadref((p - __CH_OFFSETOF(thread_t, rqueue)));
    /*lint -restore*/

#if CH_CFG_USE_DYNAMIC == TRUE
    chDbgAssert(ntp->refs < (trefs_t)255, "too many references");

    ntp->refs++;
#endif
  }
  chSysUnlock();
#if CH_CFG_USE_DYNAMIC == TRUE
  chThdRelease(tp);
#endif

  return ntp;
}

/**
 * @brief   Retrieves a thread pointer by name.
 * @note    The reference counter of the found thread is increased by one so
 *          it cannot be disposed incidentally after the pointer has been
 *          returned.
 *
 * @param[in] name      the thread name
 * @return              A pointer to the found thread.
 * @retval NULL         if a matching thread has not been found.
 *
 * @api
 */
thread_t *chRegFindThreadByName(const char *name) {
  thread_t *ctp;

  /* Scanning registry.*/
  ctp = chRegFirstThread();
  do {
    if (strcmp(chRegGetThreadNameX(ctp), name) == 0) {
      return ctp;
    }
    ctp = chRegNextThread(ctp);
  } while (ctp != NULL);

  return NULL;
}

/**
 * @brief   Confirms that a pointer is a valid thread pointer.
 * @note    The reference counter of the found thread is increased by one so
 *          it cannot be disposed incidentally after the pointer has been
 *          returned.
 *
 * @param[in] tp        pointer to the thread
 * @return              A pointer to the found thread.
 * @retval NULL         if a matching thread has not been found.
 *
 * @api
 */
thread_t *chRegFindThreadByPointer(thread_t *tp) {
  thread_t *ctp;

  /* Scanning registry.*/
  ctp = chRegFirstThread();
  do {
    if (ctp == tp) {
      return ctp;
    }
    ctp = chRegNextThread(ctp);
  } while (ctp != NULL);

  return NULL;
}

#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE) ||  \
    defined(__DOXYGEN__)
/**
 * @brief   Confirms that a working area is being used by some active thread.
 * @note    The reference counter of the found thread is increased by one so
 *          it cannot be disposed incidentally after the pointer has been
 *          returned.
 *
 * @param[in] wa        pointer to a static working area
 * @return              A pointer to the found thread.
 * @retval NULL         if a matching thread has not been found.
 *
 * @api
 */
thread_t *chRegFindThreadByWorkingArea(stkalign_t *wa) {
  thread_t *ctp;

  /* Scanning registry.*/
  ctp = chRegFirstThread();
  do {
    if (chThdGetWorkingAreaX(ctp) == wa) {
      return ctp;
    }
    ctp = chRegNextThread(ctp);
  } while (ctp != NULL);

  return NULL;
}
#endif

#endif /* CH_CFG_USE_REGISTRY == TRUE */

/** @} */
