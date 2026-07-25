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
 * @file    rt/src/chvt.c
 * @brief   Time and Virtual Timers module code.
 *
 * @addtogroup time
 * @details Time and Virtual Timers related APIs and services.
 * @{
 */

#include <string.h>

#include "ch.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#if CH_CFG_INTERVALS_SIZE > CH_CFG_ST_RESOLUTION
#define VT_MAX_DELAY                                                        \
  (((sysinterval_t)TIME_MAX_SYSTIME) &                                      \
   ~(sysinterval_t)(((sysinterval_t)1 << (CH_CFG_ST_RESOLUTION / 2)) - (sysinterval_t)1))
#endif

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

#if (CH_CFG_ST_TIMEDELTA > 0) || defined(__DOXYGEN__)
/**
 * @brief   Alarm time setup.
 * @note    An RFCU fault is registered if the system time skips past
 *          <tt>(now + delay)</tt>, the deadline is skipped forward
 *          in order to compensate for the event.
 *
 * @param[in] now       last known system time
 * @param[in] delay     delay over @p now
 */
static void vt_set_alarm(systime_t now, sysinterval_t delay) {
  sysinterval_t currdelta;

  /* Initial delta is what is configured statically.*/
  currdelta = (sysinterval_t)CH_CFG_ST_TIMEDELTA;

  if (delay < currdelta) {
    /* We need to avoid that the system time goes past the alarm we are
       going to set before the alarm is actually set.*/
    delay = currdelta;
  }
#if CH_CFG_INTERVALS_SIZE > CH_CFG_ST_RESOLUTION
  else if (delay > VT_MAX_DELAY) {
    /* The delta could be too large for the physical timer to handle
       this can happen when: sizeof (systime_t) < sizeof (sysinterval_t).*/
    delay = VT_MAX_DELAY;
  }
#endif

  /* Deadline skip detection and correction loop.*/
  while (true) {
    sysinterval_t nowdelta;
    systime_t newnow;

    /* Setting up the alarm on the next deadline.*/
    port_timer_set_alarm(chTimeAddX(now, delay));

    /* Check on current time, we need to detect the error condition where
       current time skipped past the calculated deadline.
       Note that the "<" condition is intentional, we want to make sure
       that the alarm is set before the deadline is reached because the
       comparison could happen on the transition depending on the timer
       architecture.*/
    newnow = chVTGetSystemTimeX();
    nowdelta = chTimeDiffX(now, newnow);
    if (likely(nowdelta < delay)) {
      break;
    }

    /* Trying again with a more relaxed minimum delta.*/
    currdelta += (sysinterval_t)1;

    /* Current time becomes the new "base" time.*/
    now = newnow;
    delay = currdelta;
  }

#if !defined(CH_VT_RFCU_DISABLED)
  /* Checking if a skip occurred.*/
  if (currdelta > CH_CFG_ST_TIMEDELTA) {
    chRFCUCollectFaultsI(CH_RFCU_VT_INSUFFICIENT_DELTA);
  }
#else
  /* Assertions as fallback.*/
  chDbgAssert(currdelta <= CH_CFG_ST_TIMEDELTA, "insufficient delta");
#endif
}

/**
 * @brief   Inserts a timer as first element in a delta list.
 * @note    This is the special case when the delta list is initially empty.
 */
static void vt_insert_first(virtual_timers_list_t *vtlp,
                            virtual_timer_t *vtp,
                            systime_t now,
                            sysinterval_t delay) {
  sysinterval_t currdelta;

  /* The delta list is empty, the current time becomes the new
     delta list base time, the timer is inserted.*/
  vtlp->lasttime = now;
  ch_dlist_insert_after(&vtlp->dlist, &vtp->dlist, delay);

  /* Initial delta is what is configured statically.*/
  currdelta = (sysinterval_t)CH_CFG_ST_TIMEDELTA;

  /* If the requested delay is lower than the minimum safe delta then it
     is raised to the minimum safe value.*/
  if (delay < currdelta) {
    /* We need to avoid that the system time goes past the alarm we are
       going to set before the alarm is actually set.*/
    delay = currdelta;
  }
#if CH_CFG_INTERVALS_SIZE > CH_CFG_ST_RESOLUTION
  else if (delay > VT_MAX_DELAY) {
    /* The delta could be too large for the physical timer to handle
       this can happen when: sizeof (systime_t) < sizeof (sysinterval_t).*/
    delay = VT_MAX_DELAY;
  }
#endif

  /* Being the first element inserted in the list the alarm timer
     is started.*/
  port_timer_start_alarm(chTimeAddX(vtlp->lasttime, delay));

  /* Deadline skip detection and correction loop.*/
  while (true) {
    systime_t newnow;

    /* Check on current time, we need to detect the error condition where
       current time skipped past the calculated deadline.
       Note that the "<" condition is intentional, we want to make sure
       that the alarm is set before the deadline is reached because the
       comparison could happen on the transition depending on the timer
       architecture.*/
    newnow = chVTGetSystemTimeX();
    if (likely(chTimeDiffX(now, newnow) < delay)) {
      break;
    }

    /* Trying again with a more relaxed minimum delta.*/
    currdelta += (sysinterval_t)1;

    /* Setting up the alarm on the next deadline.*/
    port_timer_set_alarm(chTimeAddX(now, currdelta));

    /* Current time becomes the new "base" time.*/
    now = newnow;
    delay = currdelta;
  }

#if !defined(CH_VT_RFCU_DISABLED)
  /* Checking if a skip occurred.*/
  if (currdelta > CH_CFG_ST_TIMEDELTA) {
    chRFCUCollectFaultsI(CH_RFCU_VT_INSUFFICIENT_DELTA);
  }
#else
  /* Assertions as fallback.*/
  chDbgAssert(currdelta <= CH_CFG_ST_TIMEDELTA, "insufficient delta");
#endif
}
#endif /* CH_CFG_ST_TIMEDELTA > 0 */

/**
 * @brief   Enqueues a virtual timer in a virtual timers list.
 */
static void vt_enqueue(virtual_timers_list_t *vtlp,
                       virtual_timer_t *vtp,
                       sysinterval_t delay) {
  sysinterval_t delta;

#if CH_CFG_ST_TIMEDELTA > 0
  {
    sysinterval_t nowdelta;
    systime_t now = chVTGetSystemTimeX();

    /* Special case where the timers list is empty.*/
    if (ch_dlist_isempty(&vtlp->dlist)) {

      vt_insert_first(vtlp, vtp, now, delay);

      return;
    }

    /* Delay as delta from 'lasttime'. Note, it can overflow and the value
       becomes lower than 'deltanow'.*/
    nowdelta = chTimeDiffX(vtlp->lasttime, now);
    delta    = nowdelta + delay;

    /* Scenario where a very large delay exceeded the numeric range, the
       delta is shortened to make it fit the numeric range, the timer
       will be triggered "deltanow" cycles earlier.*/
    if (delta < nowdelta) {
      delta = delay;
    }

    /* Checking if this timer would become the first in the delta list, this
       requires changing the current alarm setting.*/
    if (delta < vtlp->dlist.next->delta) {

      vt_set_alarm(now, delay);
    }
  }
#else /* CH_CFG_ST_TIMEDELTA == 0 */

  /* Delta is initially equal to the specified delay.*/
  delta = delay;
#endif /* CH_CFG_ST_TIMEDELTA == 0 */

  ch_dlist_insert(&vtlp->dlist, &vtp->dlist, delta);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Enables a one-shot virtual timer.
 * @details The timer is enabled and programmed to trigger after the delay
 *          specified as parameter.
 * @pre     The timer must not be already armed before calling this function.
 * @note    The callback function is invoked from interrupt context.
 *
 * @param[out] vtp      pointer to a @p virtual_timer_t structure
 * @param[in] delay     the number of ticks before the operation timeouts, the
 *                      special values are handled as follow:
 *                      - @a TIME_INFINITE is allowed but interpreted as a
 *                        normal time specification.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 *                      .
 * @param[in] vtfunc    the timer callback function. After invoking the
 *                      callback the timer is disabled and the structure can
 *                      be disposed or reused.
 * @param[in] par       a parameter that will be passed to the callback
 *                      function
 *
 * @iclass
 */
void chVTDoSetI(virtual_timer_t *vtp, sysinterval_t delay,
                vtfunc_t vtfunc, void *par) {
  virtual_timers_list_t *vtlp = &currcore->vtlist;

  chDbgCheckClassI();
  chDbgCheck((vtp != NULL) && (vtfunc != NULL) && (delay != TIME_IMMEDIATE));

  /* Timer initialization.*/
  vtp->par     = par;
  vtp->func    = vtfunc;
  vtp->reload  = (sysinterval_t)0;

  /* Inserting the timer in the delta list.*/
  vt_enqueue(vtlp, vtp, delay);
}

/**
 * @brief   Enables a continuous virtual timer.
 * @details The timer is enabled and programmed to trigger after the delay
 *          specified as parameter.
 * @pre     The timer must not be already armed before calling this function.
 * @note    The callback function is invoked from interrupt context.
 *
 * @param[out] vtp      pointer to a @p virtual_timer_t structure
 * @param[in] delay     the number of ticks before the operation timeouts, the
 *                      special values are handled as follow:
 *                      - @a TIME_INFINITE is allowed but interpreted as a
 *                        normal time specification.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 *                      .
 * @param[in] vtfunc    the timer callback function. After invoking the
 *                      callback the timer is restarted.
 * @param[in] par       a parameter that will be passed to the callback
 *                      function
 *
 * @iclass
 */
void chVTDoSetContinuousI(virtual_timer_t *vtp, sysinterval_t delay,
                          vtfunc_t vtfunc, void *par) {
  virtual_timers_list_t *vtlp = &currcore->vtlist;

  chDbgCheckClassI();
  chDbgCheck((vtp != NULL) && (vtfunc != NULL) && (delay != TIME_IMMEDIATE));

  /* Timer initialization.*/
  vtp->par     = par;
  vtp->func    = vtfunc;
  vtp->reload  = delay;

  /* Inserting the timer in the delta list.*/
  vt_enqueue(vtlp, vtp, delay);
}

/**
 * @brief   Disables a Virtual Timer.
 * @pre     The timer must be in armed state before calling this function.
 *
 * @param[in] vtp       pointer to a @p virtual_timer_t structure
 *
 * @iclass
 */
void chVTDoResetI(virtual_timer_t *vtp) {
  virtual_timers_list_t *vtlp = &currcore->vtlist;

  chDbgCheckClassI();
  chDbgCheck(vtp != NULL);
  chDbgAssert(chVTIsArmedI(vtp), "timer not armed");

#if CH_CFG_ST_TIMEDELTA == 0

  /* The delta of the timer is added to the next timer.*/
  vtp->dlist.next->delta += vtp->dlist.delta;

 /* Removing the element from the delta list, marking it as not armed.*/
  (void) ch_dlist_dequeue(&vtp->dlist);
  vtp->dlist.next = NULL;

  /* The above code changes the value in the header when the removed element
     is the last of the list, restoring it.*/
  vtlp->dlist.delta = (sysinterval_t)-1;
#else /* CH_CFG_ST_TIMEDELTA > 0 */
  systime_t now;
  sysinterval_t nowdelta, delta;

  /* If the timer is not the first of the list then it is simply unlinked
     else the operation is more complex.*/
  if (!ch_dlist_isfirst(&vtlp->dlist, &vtp->dlist)) {

    /* Removing the element from the delta list.*/
    (void) ch_dlist_dequeue(&vtp->dlist);

    /* Adding delta to the next element, if it is not the last one.*/
    vtp->dlist.next->delta += vtp->dlist.delta;

    /* Marking timer as not armed.*/
    vtp->dlist.next = NULL;

    /* Special case when the removed element from the last position in the list,
       the value in the header must be restored, just doing it is faster than
       checking then doing.*/
    vtlp->dlist.delta = (sysinterval_t)-1;

    return;
  }

  /* Removing the first timer from the list, marking it as not armed.*/
  ch_dlist_remove_first(&vtlp->dlist);
  vtp->dlist.next = NULL;

  /* If the list become empty then the alarm timer is stopped and done.*/
  if (ch_dlist_isempty(&vtlp->dlist)) {

    port_timer_stop_alarm();

    return;
  }

  /* The delta of the removed timer is added to the new first timer.*/
  vtlp->dlist.next->delta += vtp->dlist.delta;

  /* Distance in ticks between the last alarm event and current time.*/
  now = chVTGetSystemTimeX();
  nowdelta = chTimeDiffX(vtlp->lasttime, now);

  /* If the current time surpassed the time of the next element in list
     then the event interrupt is already pending, just return.*/
  if (nowdelta >= vtlp->dlist.next->delta) {
    return;
  }

  /* Distance from the next scheduled event and now.*/
  delta = vtlp->dlist.next->delta - nowdelta;

  /* Setting up the alarm.*/
  vt_set_alarm(now, delta);
#endif /* CH_CFG_ST_TIMEDELTA > 0 */
}

/**
 * @brief   Returns the remaining time interval before next timer trigger.
 * @note    This function can be called while the timer is active.
 *
 * @param[in] vtp       pointer to a @p virtual_timer_t structure
 * @return              The remaining time interval.
 *
 * @iclass
 */
sysinterval_t chVTGetRemainingIntervalI(virtual_timer_t *vtp) {
  virtual_timers_list_t *vtlp = &currcore->vtlist;
  sysinterval_t delta;
  ch_delta_list_t *dlp;

  chDbgCheckClassI();

  delta = (sysinterval_t)0;
  dlp = vtlp->dlist.next;
  do {
    delta += dlp->delta;
    if (dlp == &vtp->dlist) {
#if CH_CFG_ST_TIMEDELTA > 0
      systime_t now = chVTGetSystemTimeX();
      sysinterval_t nowdelta = chTimeDiffX(vtlp->lasttime, now);
      if (nowdelta > delta) {
        return (sysinterval_t)0;
      }
      return delta - nowdelta;
#else
      return delta;
#endif
    }
    dlp = dlp->next;
  } while (dlp != &vtlp->dlist);

  chDbgAssert(false, "timer not in list");

  return (sysinterval_t)-1;
}

/**
 * @brief   Virtual timers ticker.
 * @note    The system lock is released before entering the callback and
 *          re-acquired immediately after. It is callback's responsibility
 *          to acquire the lock if needed. This is done in order to reduce
 *          interrupts jitter when many timers are in use.
 *
 * @iclass
 */
void chVTDoTickI(void) {
  virtual_timers_list_t *vtlp = &currcore->vtlist;

  chDbgCheckClassI();

#if CH_CFG_ST_TIMEDELTA == 0
  vtlp->systime++;
  if (ch_dlist_notempty(&vtlp->dlist)) {
    /* The list is not empty, processing elements on top.*/
    --vtlp->dlist.next->delta;
    while (vtlp->dlist.next->delta == (sysinterval_t)0) {
      virtual_timer_t *vtp;

      /* Triggered timer.*/
      vtp = (virtual_timer_t *)vtlp->dlist.next;

      /* Removing the element from the delta list, marking it as not armed.*/
      (void) ch_dlist_dequeue(&vtp->dlist);
      vtp->dlist.next = NULL;

      chSysUnlockFromISR();
      vtp->func(vtp, vtp->par);
      chSysLockFromISR();

      /* If a reload is defined the timer needs to be restarted.*/
      if (vtp->reload > (sysinterval_t)0) {
        ch_dlist_insert(&vtlp->dlist, &vtp->dlist, vtp->reload);
      }
    }
  }
#else /* CH_CFG_ST_TIMEDELTA > 0 */
  virtual_timer_t *vtp;
  sysinterval_t nowdelta;
  systime_t now;

  /* Looping through timers consuming all timers with deltas lower or equal
     than the interval between "now" and "lasttime".*/
  while (true) {
    systime_t lasttime;

    /* First timer in the delta list.*/
    vtp = (virtual_timer_t *)vtlp->dlist.next;

    /* Delta between current time and last execution time.*/
    now = chVTGetSystemTimeX();
    nowdelta = chTimeDiffX(vtlp->lasttime, now);

    /* Loop break condition.
       Note that the list scan is limited by the delta list header having
       "vtlp->dlist.delta == (sysinterval_t)-1" which is greater than all
       deltas*/
    if (nowdelta < vtp->dlist.delta) {
      break;
    }

    /* Last time deadline is updated to the next timer's time.*/
    lasttime = chTimeAddX(vtlp->lasttime, vtp->dlist.delta);
    vtlp->lasttime = lasttime;

    /* Removing the timer from the list, marking it as not armed.*/
    (void) ch_dlist_dequeue(&vtp->dlist);
    vtp->dlist.next = NULL;

    /* If the list becomes empty then the alarm is disabled.*/
    if (ch_dlist_isempty(&vtlp->dlist)) {
      port_timer_stop_alarm();
    }

    /* The callback is invoked outside the kernel critical section, it
       is re-entered on the callback return. Note that "lasttime" can be
       modified within the callback if some timer function is called.*/
    chSysUnlockFromISR();

    vtp->func(vtp, vtp->par);

    chSysLockFromISR();

    /* If a reload is defined the timer needs to be restarted.*/
    if (unlikely(vtp->reload > (sysinterval_t)0)) {
      sysinterval_t delta, delay;

      /* Refreshing the now delta after spending time in the callback for
         a more accurate detection of too fast reloads.*/
      now = chVTGetSystemTimeX();
      nowdelta = chTimeDiffX(lasttime, now);

#if !defined(CH_VT_RFCU_DISABLED)
      /* Checking if the required reload is feasible.*/
      if (nowdelta > vtp->reload) {
        /* System time is already past the deadline, logging the fault and
           proceeding with a minimum delay.*/

        chDbgAssert(false, "skipped deadline");
        chRFCUCollectFaultsI(CH_RFCU_VT_SKIPPED_DEADLINE);

        delay = (sysinterval_t)0;
      }
      else {
        /* Enqueuing the timer again using the calculated delta.*/
        delay = vtp->reload - nowdelta;
      }
#else
      /* Assertions as fallback.*/
      chDbgAssert(nowdelta <= vtp->reload, "skipped deadline");

      /* Enqueuing the timer again using the calculated delta.*/
      delay = vtp->reload - nowdelta;
#endif

      /* Special case where the timers list is empty.*/
      if (ch_dlist_isempty(&vtlp->dlist)) {

        vt_insert_first(vtlp, vtp, now, delay);

        return;
      }

      /* Delay as delta from 'lasttime'. Note, it can overflow and the value
         becomes lower than 'nowdelta'. In that case the delta is shortened
         to make it fit the numeric range and the timer will be triggered
         "nowdelta" cycles earlier.*/
      delta = nowdelta + delay;
      if (delta < nowdelta) {
        delta = delay;
      }

      /* Insert into delta list. */
      ch_dlist_insert(&vtlp->dlist, &vtp->dlist, delta);
    }
  }

  /* If the list is empty, nothing else to do.*/
  if (ch_dlist_isempty(&vtlp->dlist)) {
    return;
  }

  /* The "unprocessed nowdelta" time slice is added to "last time"
     and subtracted to next timer's delta.*/
  vtlp->lasttime += nowdelta;
  vtp->dlist.delta -= nowdelta;

  /* Update alarm time to next timer.*/
  vt_set_alarm(now, vtp->dlist.delta);
#endif /* CH_CFG_ST_TIMEDELTA > 0 */
}

#if (CH_CFG_USE_TIMESTAMP == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Generates a monotonic time stamp.
 * @details This function generates a monotonic time stamp synchronized with
 *          the system time. The time stamp has the same resolution of
 *          system time.
 * @note    There is an assumption, this function must be called at
 *          least once before the system time wraps back to zero or
 *          synchronization is lost. You may use a periodic virtual timer with
 *          a very large interval in order to keep time stamps synchronized
 *          by calling this function.
 *
 * @return              The time stamp.
 *
 * @iclass
 */
systimestamp_t chVTGetTimeStampI(void) {
  os_instance_t * oip = currcore;
  systimestamp_t last, stamp;
  systime_t now;

  chDbgCheckClassI();

  /* Current system time.*/
  now = chVTGetSystemTimeX();

  /* Last time stamp generated.*/
  last = oip->vtlist.laststamp;

  /* Interval between the last time stamp and current time used for a new
     time stamp. Note that this fails if the interval is larger than a
     systime_t type.*/
  stamp = last + (systimestamp_t)chTimeDiffX((systime_t)last, now);

  chDbgAssert(oip->vtlist.laststamp <= stamp, "wrapped");

  /* Storing the new stamp.*/
  oip->vtlist.laststamp = stamp;

  return stamp;
}

/**
 * @brief   Resets and re-synchronizes the time stamps monotonic counter.
 *
 * @iclass
 */
void chVTResetTimeStampI(void) {

  chDbgCheckClassI();

  currcore->vtlist.laststamp = (systimestamp_t)chVTGetSystemTimeX();
}

#endif /* CH_CFG_USE_TIMESTAMP == TRUE */

/** @} */
