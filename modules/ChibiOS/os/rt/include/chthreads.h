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
 * @file    rt/include/chthreads.h
 * @brief   Threads module macros and structures.
 *
 * @addtogroup threads
 * @{
 */

#ifndef CHTHREADS_H
#define CHTHREADS_H

/*lint -sem(chThdExit, r_no) -sem(chThdExitS, r_no)*/

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
 * @brief   Thread function.
 */
typedef void (*tfunc_t)(void *p);

/**
 * @brief   Type of a thread descriptor.
 */
typedef struct {
  /**
   * @brief   Thread name.
   */
  const char        *name;
  /**
   * @brief   Pointer to the working area base.
   */
  stkalign_t        *wbase;
  /**
   * @brief   Pointer to the working area end.
   */
  stkalign_t        *wend;
  /**
   * @brief   Thread priority.
   */
  tprio_t           prio;
  /**
   * @brief   Thread function pointer.
   */
  tfunc_t           funcp;
  /**
   * @brief   Thread argument.
   */
  void              *arg;
#if (CH_CFG_SMP_MODE != FALSE) || defined(__DOXYGEN__)
  /**
   * @brief         OS instance affinity or @p NULL for current one.
   */
  os_instance_t     *instance;
#endif
} thread_descriptor_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @name    Threads queues
 * @{
 */
/**
 * @brief   Data part of a static threads queue object initializer.
 * @details This macro should be used when statically initializing a threads
 *          queue that is part of a bigger structure.
 *
 * @param[in] name      the name of the threads queue variable
 */
#define __THREADS_QUEUE_DATA(name) {__CH_QUEUE_DATA(name)}

/**
 * @brief   Static threads queue object initializer.
 * @details Statically initialized threads queues require no explicit
 *          initialization using @p queue_init().
 *
 * @param[in] name      the name of the threads queue variable
 */
#define THREADS_QUEUE_DECL(name)                                            \
  threads_queue_t name = __THREADS_QUEUE_DATA(name)
/** @} */

/**
 * @name    Working Areas
 * @{
 */
/**
 * @brief   Calculates the total Working Area size.
 *
 * @param[in] n         the stack size to be assigned to the thread
 * @return              The total used memory in bytes.
 *
 * @api
 */
#define THD_WORKING_AREA_SIZE(n)                                            \
  MEM_ALIGN_NEXT(sizeof(thread_t) + PORT_WA_SIZE(n), PORT_STACK_ALIGN)

/**
 * @brief   Static working area allocation.
 * @details This macro is used to allocate a static thread working area
 *          aligned as both position and size.
 *
 * @param[in] s         the name to be assigned to the stack array
 * @param[in] n         the stack size to be assigned to the thread
 *
 * @api
 */
#define THD_WORKING_AREA(s, n) PORT_WORKING_AREA(s, n)

/**
 * @brief   Base of a working area casted to the correct type.
 *
 * @param[in] s         name of the working area
 */
#define THD_WORKING_AREA_BASE(s) ((stkalign_t *)(s))

/**
 * @brief   End of a working area casted to the correct type.
 *
 * @param[in] s         name of the working area
 */
#define THD_WORKING_AREA_END(s) (THD_WORKING_AREA_BASE(s) +                 \
                                 (sizeof (s) / sizeof (stkalign_t)))
/** @} */

/**
 * @name    Threads abstraction macros
 * @{
 */
/**
 * @brief   Thread declaration macro.
 * @note    Thread declarations should be performed using this macro because
 *          the port layer could define optimizations for thread functions.
 */
#define THD_FUNCTION(tname, arg) PORT_THD_FUNCTION(tname, arg)
/** @} */

/**
 * @name    Threads initializers
 * @{
 */
#if (CH_CFG_SMP_MODE != FALSE) || defined(__DOXYGEN__)
/**
 * @brief   Thread descriptor initializer with no affinity.
 *
 * @param[in] name      thread name
 * @param[in] wbase     pointer to the working area base
 * @param[in] wend      pointer to the working area end
 * @param[in] prio      thread priority
 * @param[in] funcp     thread function pointer
 * @param[in] arg       thread argument
 */
#define THD_DESCRIPTOR(name, wbase, wend, prio, funcp, arg) {               \
  (name),                                                                   \
  (wbase),                                                                  \
  (wend),                                                                   \
  (prio),                                                                   \
  (funcp),                                                                  \
  (arg),                                                                    \
  NULL                                                                      \
}
#else
#define THD_DESCRIPTOR(name, wbase, wend, prio, funcp, arg) {               \
  (name),                                                                   \
  (wbase),                                                                  \
  (wend),                                                                   \
  (prio),                                                                   \
  (funcp),                                                                  \
  (arg)                                                                     \
}
#endif

/**
 * @brief   Thread descriptor initializer with no affinity.
 *
 * @param[in] name      thread name
 * @param[in] wbase     pointer to the working area base
 * @param[in] wend      pointer to the working area end
 * @param[in] prio      thread priority
 * @param[in] funcp     thread function pointer
 * @param[in] arg       thread argument
 * @param[in] oip       instance affinity
 */
#define THD_DESCRIPTOR_AFFINITY(name, wbase, wend, prio, funcp, arg, oip) { \
  (name),                                                                   \
  (wbase),                                                                  \
  (wend),                                                                   \
  (prio),                                                                   \
  (funcp),                                                                  \
  (arg),                                                                    \
  (oip)                                                                     \
}
/** @} */

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Delays the invoking thread for the specified number of seconds.
 * @note    The specified time is rounded up to a value allowed by the real
 *          system tick clock.
 * @note    The maximum specifiable value is implementation dependent.
 * @note    Use of this macro for large values is not secure because
 *          integer overflows, make sure your value can be correctly
 *          converted.
 *
 * @param[in] sec       time in seconds, must be different from zero
 *
 * @api
 */
#define chThdSleepSeconds(sec) chThdSleep(TIME_S2I(sec))

/**
 * @brief   Delays the invoking thread for the specified number of
 *          milliseconds.
 * @note    The specified time is rounded up to a value allowed by the real
 *          system tick clock.
 * @note    The maximum specifiable value is implementation dependent.
 * @note    Use of this macro for large values is not secure because
 *          integer overflows, make sure your value can be correctly
 *          converted.
 *
 * @param[in] msec      time in milliseconds, must be different from zero
 *
 * @api
 */
#define chThdSleepMilliseconds(msec) chThdSleep(TIME_MS2I(msec))

/**
 * @brief   Delays the invoking thread for the specified number of
 *          microseconds.
 * @note    The specified time is rounded up to a value allowed by the real
 *          system tick clock.
 * @note    The maximum specifiable value is implementation dependent.
 * @note    Use of this macro for large values is not secure because
 *          integer overflows, make sure your value can be correctly
 *          converted.
 *
 * @param[in] usec      time in microseconds, must be different from zero
 *
 * @api
 */
#define chThdSleepMicroseconds(usec) chThdSleep(TIME_US2I(usec))
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
   thread_t *__thd_object_init(os_instance_t *oip,
                               thread_t *tp,
                               const char *name,
                               tprio_t prio);
#if CH_DBG_FILL_THREADS == TRUE
  void __thd_stackfill(uint8_t *startp, uint8_t *endp);
#endif
  thread_t *chThdCreateSuspendedI(const thread_descriptor_t *tdp);
  thread_t *chThdCreateSuspended(const thread_descriptor_t *tdp);
  thread_t *chThdCreateI(const thread_descriptor_t *tdp);
  thread_t *chThdCreate(const thread_descriptor_t *tdp);
  thread_t *chThdCreateStatic(void *wsp, size_t size,
                              tprio_t prio, tfunc_t pf, void *arg);
  thread_t *chThdStart(thread_t *tp);
#if CH_CFG_USE_REGISTRY == TRUE
  thread_t *chThdAddRef(thread_t *tp);
  void chThdRelease(thread_t *tp);
#endif
  void chThdExit(msg_t msg);
  void chThdExitS(msg_t msg);
#if CH_CFG_USE_WAITEXIT == TRUE
  msg_t chThdWait(thread_t *tp);
#endif
  tprio_t chThdSetPriority(tprio_t newprio);
  void chThdTerminate(thread_t *tp);
  msg_t chThdSuspendS(thread_reference_t *trp);
  msg_t chThdSuspendTimeoutS(thread_reference_t *trp, sysinterval_t timeout);
  void chThdResumeI(thread_reference_t *trp, msg_t msg);
  void chThdResumeS(thread_reference_t *trp, msg_t msg);
  void chThdResume(thread_reference_t *trp, msg_t msg);
  msg_t chThdEnqueueTimeoutS(threads_queue_t *tqp, sysinterval_t timeout);
  void chThdDequeueNextI(threads_queue_t *tqp, msg_t msg);
  void chThdDequeueAllI(threads_queue_t *tqp, msg_t msg);
  void chThdSleep(sysinterval_t time);
  void chThdSleepUntil(systime_t time);
  systime_t chThdSleepUntilWindowed(systime_t prev, systime_t next);
  void chThdYield(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Returns a pointer to the current @p thread_t.
 *
 * @return             A pointer to the current thread.
 *
 * @xclass
 */
static inline thread_t *chThdGetSelfX(void) {

  return __sch_get_currthread();
}

/**
 * @brief   Returns the current thread priority.
 * @note    Can be invoked in any context.
 *
 * @return              The current thread priority.
 *
 * @xclass
 */
static inline tprio_t chThdGetPriorityX(void) {

  return chThdGetSelfX()->hdr.pqueue.prio;
}

/**
 * @brief   Returns the number of ticks consumed by the specified thread.
 * @note    This function is only available when the
 *          @p CH_DBG_THREADS_PROFILING configuration option is enabled.
 *
 * @param[in] tp        pointer to the thread
 * @return              The number of consumed system ticks.
 *
 * @xclass
 */
#if (CH_DBG_THREADS_PROFILING == TRUE) || defined(__DOXYGEN__)
static inline systime_t chThdGetTicksX(thread_t *tp) {

  return tp->time;
}
#endif

#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE) ||  \
    defined(__DOXYGEN__)
/**
 * @brief   Returns the working area base of the specified thread.
 *
 * @param[in] tp        pointer to the thread
 * @return              The working area base pointer.
 *
 * @xclass
 */
static inline stkalign_t *chThdGetWorkingAreaX(thread_t *tp) {

  return tp->wabase;
}
#endif /* CH_DBG_ENABLE_STACK_CHECK == TRUE */

/**
 * @brief   Verifies if the specified thread is in the @p CH_STATE_FINAL state.
 *
 * @param[in] tp        pointer to the thread
 * @retval true         thread terminated.
 * @retval false        thread not terminated.
 *
 * @xclass
 */
static inline bool chThdTerminatedX(thread_t *tp) {

  return (bool)(tp->state == CH_STATE_FINAL);
}

/**
 * @brief   Verifies if the current thread has a termination request pending.
 *
 * @retval true         termination request pending.
 * @retval false        termination request not pending.
 *
 * @xclass
 */
static inline bool chThdShouldTerminateX(void) {

  return (bool)((chThdGetSelfX()->flags & CH_FLAG_TERMINATE) != (tmode_t)0);
}

/**
 * @brief   Resumes a thread created with @p chThdCreateI().
 *
 * @param[in] tp        pointer to the thread
 * @return              The pointer to the @p thread_t structure allocated for
 *                      the thread into the working space area.
 *
 * @iclass
 */
static inline thread_t *chThdStartI(thread_t *tp) {

  chDbgAssert(tp->state == CH_STATE_WTSTART, "wrong state");

  return chSchReadyI(tp);
}

/**
 * @brief   Suspends the invoking thread for the specified number of ticks.
 *
 * @param[in] ticks     the delay in system ticks, the special values are
 *                      handled as follow:
 *                      - @a TIME_INFINITE the thread enters an infinite sleep
 *                        state.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 *                      .
 *
 * @sclass
 */
static inline void chThdSleepS(sysinterval_t ticks) {

  chDbgCheck(ticks != TIME_IMMEDIATE);

  (void) chSchGoSleepTimeoutS(CH_STATE_SLEEPING, ticks);
}

/**
 * @brief   Initializes a threads queue object.
 *
 * @param[out] tqp      pointer to the threads queue object
 *
 * @init
 */
static inline void chThdQueueObjectInit(threads_queue_t *tqp) {

  ch_queue_init(&tqp->queue);
}

/**
 * @brief   Evaluates to @p true if the specified queue is empty.
 *
 * @param[out] tqp      pointer to the threads queue object
 * @return              The queue status.
 * @retval false        if the queue is not empty.
 * @retval true         if the queue is empty.
 *
 * @iclass
 */
static inline bool chThdQueueIsEmptyI(threads_queue_t *tqp) {

  chDbgCheckClassI();

  return ch_queue_isempty(&tqp->queue);
}

/**
 * @brief   Dequeues and wakes up one thread from the threads queue object.
 * @details Dequeues one thread from the queue without checking if the queue
 *          is empty.
 * @pre     The queue must contain at least an object.
 *
 * @param[in] tqp       pointer to the threads queue object
 * @param[in] msg       the message code
 *
 * @iclass
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
static inline void chThdDoDequeueNextI(threads_queue_t *tqp, msg_t msg) {
  thread_t *tp;

  chDbgAssert(ch_queue_notempty(&tqp->queue), "empty queue");

  tp = threadref(ch_queue_fifo_remove(&tqp->queue));

  chDbgAssert(tp->state == CH_STATE_QUEUED, "invalid state");

  tp->u.rdymsg = msg;
  (void) chSchReadyI(tp);
}
#pragma GCC diagnostic pop

#endif /* CHTHREADS_H */

/** @} */
