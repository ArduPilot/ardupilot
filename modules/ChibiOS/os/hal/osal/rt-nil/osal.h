/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    osal.h
 * @brief   OSAL module header.
 *
 * @addtogroup OSAL
 * @{
 */

#ifndef OSAL_H
#define OSAL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "ch.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @name    Common constants
 * @{
 */
#if !defined(FALSE) || defined(__DOXYGEN__)
#define FALSE                               0
#endif

#if !defined(TRUE) || defined(__DOXYGEN__)
#define TRUE                                1
#endif

#define OSAL_SUCCESS                        false
#define OSAL_FAILED                         true
/** @} */

#if 0
/**
 * @name    Messages
 * @{
 */
#define MSG_OK                              (msg_t)0
#define MSG_TIMEOUT                         (msg_t)-1
#define MSG_RESET                           (msg_t)-2
/** @} */
#endif

#if 0
/**
 * @name    Special time constants
 * @{
 */
#define TIME_IMMEDIATE                      ((sysinterval_t)0)
#define TIME_INFINITE                       ((sysinterval_t)-1)
/** @} */
#endif

/**
 * @name    Systick modes.
 * @{
 */
#define OSAL_ST_MODE_NONE                   0
#define OSAL_ST_MODE_PERIODIC               1
#define OSAL_ST_MODE_FREERUNNING            2
/** @} */

/**
 * @name    Systick parameters.
 * @{
 */
/**
 * @brief   Size in bits of the @p systick_t type.
 */
#define OSAL_ST_RESOLUTION                  CH_CFG_ST_RESOLUTION

/**
 * @brief   Required systick frequency or resolution.
 */
#define OSAL_ST_FREQUENCY                   CH_CFG_ST_FREQUENCY

/**
 * @brief   Systick mode required by the underlying OS.
 */
#if (CH_CFG_ST_TIMEDELTA == 0) || defined(__DOXYGEN__)
#define OSAL_ST_MODE                        OSAL_ST_MODE_PERIODIC
#else
#define OSAL_ST_MODE                        OSAL_ST_MODE_FREERUNNING
#endif
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !(OSAL_ST_MODE == OSAL_ST_MODE_NONE) &&                                 \
    !(OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC) &&                             \
    !(OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING)
#error "invalid OSAL_ST_MODE setting in osal.h"
#endif

#if (OSAL_ST_RESOLUTION != 16) && (OSAL_ST_RESOLUTION != 32) &&             \
    (OSAL_ST_RESOLUTION != 64)
#error "invalid OSAL_ST_RESOLUTION, must be 16, 32 or 64"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

#if 0
/**
 * @brief   Type of a system status word.
 */
typedef uint32_t syssts_t;
#endif

#if 0
/**
 * @brief   Type of a message.
 */
typedef int32_t msg_t;
#endif

#if 0
/**
 * @brief   Type of system time counter.
 */
typedef uint32_t systime_t;
#endif

#if 0
/**
 * @brief   Type of system time interval.
 */
typedef uint32_t sysinterval_t;
#endif

#if 0
/**
 * @brief   Type of time conversion variable.
 * @note    This type must have double width than other time types, it is
 *          only used internally for conversions.
 */
typedef uint64_t time_conv_t;
#endif

#if 0
/**
 * @brief   Type of realtime counter.
 */
typedef uint32_t rtcnt_t;
#endif

#if 0
/**
 * @brief   Type of a thread reference.
 */
typedef thread_t * thread_reference_t;
#endif

#if 0
/**
 * @brief   Type of an event flags mask.
 */
typedef uint32_t eventflags_t;
#endif

#if (CH_CFG_USE_EVENTS == FALSE) || defined(__DOXYGEN__)
/**
 * @brief   Type of an event flags object.
 * @note    The content of this structure is not part of the API and should
 *          not be relied upon. Implementers may define this structure in
 *          an entirely different way.
 * @note    Retrieval and clearing of the flags are not defined in this
 *          API and are implementation-dependent.
 */
typedef struct event_source event_source_t;

/**
 * @brief   Type of an event source callback.
 * @note    This type is not part of the OSAL API and is provided
 *          exclusively as an example and for convenience.
 */
typedef void (*eventcallback_t)(event_source_t *esp);

/**
 * @brief   Events source object.
 * @note    The content of this structure is not part of the API and should
 *          not be relied upon. Implementers may define this structure in
 *          an entirely different way.
 * @note    Retrieval and clearing of the flags are not defined in this
 *          API and are implementation-dependent.
 */
struct event_source {
  volatile eventflags_t flags;      /**< @brief Stored event flags.         */
  eventcallback_t       cb;         /**< @brief Event source callback.      */
  void                  *param;     /**< @brief User defined field.         */
};
#endif /* CH_CFG_USE_EVENTS == FALSE */

/**
 * @brief   Type of a mutex.
 * @note    If the OS does not support mutexes or there is no OS then the
 *          mechanism can be simulated.
 */
#if CH_CFG_USE_MUTEXES || defined(__DOXYGEN__)
#elif CH_CFG_USE_SEMAPHORES
typedef semaphore_t mutex_t;
#else
typedef uint32_t mutex_t;
#endif

#if 0
/**
 * @brief   Type of a thread queue.
 * @details A thread queue is a queue of sleeping threads, queued threads
 *          can be dequeued one at time or all together.
 * @note    In this implementation it is implemented as a single reference
 *          because there are no real threads.
 */
typedef struct {
  thread_reference_t    tr;
} threads_queue_t;
#endif

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @name    Debug related macros
 * @{
 */
/**
 * @brief   Condition assertion.
 * @details If the condition check fails then the OSAL panics with a
 *          message and halts.
 * @note    The condition is tested only if the @p OSAL_ENABLE_ASSERTIONS
 *          switch is enabled.
 * @note    The remark string is not currently used except for putting a
 *          comment in the code about the assertion.
 *
 * @param[in] c         the condition to be verified to be true
 * @param[in] remark    a remark string
 *
 * @api
 */
#ifndef osalDbgAssert
#define osalDbgAssert(c, remark) chDbgAssert(c, remark)
#endif

/**
 * @brief   Function parameters check.
 * @details If the condition check fails then the OSAL panics and halts.
 * @note    The condition is tested only if the @p OSAL_ENABLE_CHECKS switch
 *          is enabled.
 *
 * @param[in] c         the condition to be verified to be true
 *
 * @api
 */
#define osalDbgCheck(c) chDbgCheck(c)

/**
 * @brief   I-Class state check.
 * @note    Not implemented in this simplified OSAL.
 */
#define osalDbgCheckClassI() chDbgCheckClassI()

/**
 * @brief   S-Class state check.
 * @note    Not implemented in this simplified OSAL.
 */
#define osalDbgCheckClassS() chDbgCheckClassS()
/** @} */

/**
 * @name    IRQ service routines wrappers
 * @{
 */
/**
 * @brief   Priority level verification macro.
 */
#define OSAL_IRQ_IS_VALID_PRIORITY(n) CH_IRQ_IS_VALID_KERNEL_PRIORITY(n)

/**
 * @brief   IRQ prologue code.
 * @details This macro must be inserted at the start of all IRQ handlers.
 */
#define OSAL_IRQ_PROLOGUE() CH_IRQ_PROLOGUE()

/**
 * @brief   IRQ epilogue code.
 * @details This macro must be inserted at the end of all IRQ handlers.
 */
#define OSAL_IRQ_EPILOGUE() CH_IRQ_EPILOGUE()

/**
 * @brief   IRQ handler function declaration.
 * @details This macro hides the details of an ISR function declaration.
 *
 * @param[in] id        a vector name as defined in @p vectors.s
 */
#define OSAL_IRQ_HANDLER(id) CH_IRQ_HANDLER(id)
/** @} */

/**
 * @name    Time conversion utilities
 * @{
 */
/**
 * @brief   Seconds to time interval.
 * @details Converts from seconds to system ticks number.
 * @note    The result is rounded upward to the next tick boundary.
 * @note    Use of this macro for large values is not secure because
 *          integer overflows, make sure your value can be correctly
 *          converted.
 *
 * @param[in] secs      number of seconds
 * @return              The number of ticks.
 *
 * @api
 */
#define OSAL_S2I(secs) TIME_S2I(secs)

/**
 * @brief   Milliseconds to time interval.
 * @details Converts from milliseconds to system ticks number.
 * @note    The result is rounded upward to the next tick boundary.
 * @note    Use of this macro for large values is not secure because
 *          integer overflows, make sure your value can be correctly
 *          converted.
 *
 * @param[in] msecs     number of milliseconds
 * @return              The number of ticks.
 *
 * @api
 */
#define OSAL_MS2I(msecs) TIME_MS2I(msecs)

/**
 * @brief   Microseconds to time interval.
 * @details Converts from microseconds to system ticks number.
 * @note    The result is rounded upward to the next tick boundary.
 * @note    Use of this macro for large values is not secure because
 *          integer overflows, make sure your value can be correctly
 *          converted.
 *
 * @param[in] usecs     number of microseconds
 * @return              The number of ticks.
 *
 * @api
 */
#define OSAL_US2I(usecs) TIME_US2I(usecs)

/**
 * @brief   Time interval to seconds.
 * @details Converts from system ticks number to seconds.
 * @note    The result is rounded up to the next second boundary.
 * @note    Use of this macro for large values is not secure because
 *          integer overflows, make sure your value can be correctly
 *          converted.
 *
 * @param[in] interval  interval in ticks
 * @return              The number of seconds.
 *
 * @api
 */
#define OSAL_I2S(interval) TIME_I2S(interval)

/**
 * @brief   Time interval to milliseconds.
 * @details Converts from system ticks number to milliseconds.
 * @note    The result is rounded up to the next millisecond boundary.
 * @note    Use of this macro for large values is not secure because
 *          integer overflows, make sure your value can be correctly
 *          converted.
 *
 * @param[in] interval  interval in ticks
 * @return              The number of milliseconds.
 *
 * @api
 */
#define OSAL_I2MS(interval) TIME_I2MS(interval)

/**
 * @brief   Time interval to microseconds.
 * @details Converts from system ticks number to microseconds.
 * @note    The result is rounded up to the next microsecond boundary.
 * @note    Use of this macro for large values is not secure because
 *          integer overflows, make sure your value can be correctly
 *          converted.
 *
 * @param[in] interval  interval in ticks
 * @return              The number of microseconds.
 *
 * @api
 */
#define OSAL_I2US(interval) TIME_I2US(interval)
/** @} */

/**
 * @name    Time conversion utilities for the realtime counter
 * @{
 */
/**
 * @brief   Seconds to realtime counter.
 * @details Converts from seconds to realtime counter cycles.
 * @note    The macro assumes that @p freq >= @p 1.
 *
 * @param[in] freq      clock frequency, in Hz, of the realtime counter
 * @param[in] sec       number of seconds
 * @return              The number of cycles.
 *
 * @api
 */
#define OSAL_S2RTC(freq, sec) S2RTC(freq, sec)

/**
 * @brief   Milliseconds to realtime counter.
 * @details Converts from milliseconds to realtime counter cycles.
 * @note    The result is rounded upward to the next millisecond boundary.
 * @note    The macro assumes that @p freq >= @p 1000.
 *
 * @param[in] freq      clock frequency, in Hz, of the realtime counter
 * @param[in] msec      number of milliseconds
 * @return              The number of cycles.
 *
 * @api
 */
#define OSAL_MS2RTC(freq, msec) MS2RTC(freq, msec)

/**
 * @brief   Microseconds to realtime counter.
 * @details Converts from microseconds to realtime counter cycles.
 * @note    The result is rounded upward to the next microsecond boundary.
 * @note    The macro assumes that @p freq >= @p 1000000.
 *
 * @param[in] freq      clock frequency, in Hz, of the realtime counter
 * @param[in] usec      number of microseconds
 * @return              The number of cycles.
 *
 * @api
 */
#define OSAL_US2RTC(freq, usec) US2RTC(freq, usec)
/** @} */

/**
 * @name    Sleep macros using absolute time
 * @{
 */
/**
 * @brief   Delays the invoking thread for the specified number of seconds.
 * @note    The specified time is rounded up to a value allowed by the real
 *          system tick clock.
 * @note    The maximum specifiable value is implementation dependent.
 *
 * @param[in] secs      time in seconds, must be different from zero
 *
 * @api
 */
#define osalThreadSleepSeconds(secs) osalThreadSleep(OSAL_S2I(secs))

/**
 * @brief   Delays the invoking thread for the specified number of
 *          milliseconds.
 * @note    The specified time is rounded up to a value allowed by the real
 *          system tick clock.
 * @note    The maximum specifiable value is implementation dependent.
 *
 * @param[in] msecs     time in milliseconds, must be different from zero
 *
 * @api
 */
#define osalThreadSleepMilliseconds(msecs) osalThreadSleep(OSAL_MS2I(msecs))

/**
 * @brief   Delays the invoking thread for the specified number of
 *          microseconds.
 * @note    The specified time is rounded up to a value allowed by the real
 *          system tick clock.
 * @note    The maximum specifiable value is implementation dependent.
 *
 * @param[in] usecs     time in microseconds, must be different from zero
 *
 * @api
 */
#define osalThreadSleepMicroseconds(usecs) osalThreadSleep(OSAL_US2I(usecs))
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   OSAL module initialization.
 *
 * @api
 */
static inline void osalInit(void) {

}

/**
 * @brief   System halt with error message.
 *
 * @param[in] reason    the halt message pointer
 *
 * @api
 */
static inline void osalSysHalt(const char *reason) {

  chSysHalt(reason);
}

/**
 * @brief   Disables interrupts globally.
 *
 * @special
 */
static inline void osalSysDisable(void) {

  chSysDisable();
}

/**
 * @brief   Enables interrupts globally.
 *
 * @special
 */
static inline void osalSysEnable(void) {

  chSysEnable();
}

/**
 * @brief   Enters a critical zone from thread context.
 * @note    This function cannot be used for reentrant critical zones.
 *
 * @special
 */
static inline void osalSysLock(void) {

  chSysLock();
}

/**
 * @brief   Leaves a critical zone from thread context.
 * @note    This function cannot be used for reentrant critical zones.
 *
 * @special
 */
static inline void osalSysUnlock(void) {

  chSysUnlock();
}

/**
 * @brief   Enters a critical zone from ISR context.
 * @note    This function cannot be used for reentrant critical zones.
 *
 * @special
 */
static inline void osalSysLockFromISR(void) {

  chSysLockFromISR();
}

/**
 * @brief   Leaves a critical zone from ISR context.
 * @note    This function cannot be used for reentrant critical zones.
 *
 * @special
 */
static inline void osalSysUnlockFromISR(void) {

  chSysUnlockFromISR();
}

/**
 * @brief   Returns the execution status and enters a critical zone.
 * @details This functions enters into a critical zone and can be called
 *          from any context. Because its flexibility it is less efficient
 *          than @p chSysLock() which is preferable when the calling context
 *          is known.
 * @post    The system is in a critical zone.
 *
 * @return              The previous system status, the encoding of this
 *                      status word is architecture-dependent and opaque.
 *
 * @xclass
 */
static inline syssts_t osalSysGetStatusAndLockX(void) {

  return chSysGetStatusAndLockX();
}

/**
 * @brief   Restores the specified execution status and leaves a critical zone.
 * @note    A call to @p chSchRescheduleS() is automatically performed
 *          if exiting the critical zone and if not in ISR context.
 *
 * @param[in] sts       the system status to be restored.
 *
 * @xclass
 */
static inline void osalSysRestoreStatusX(syssts_t sts) {

  chSysRestoreStatusX(sts);
}

/**
 * @brief   Polled delay.
 * @note    The real delay is always few cycles in excess of the specified
 *          value.
 *
 * @param[in] cycles    number of cycles
 *
 * @xclass
 */
#if (PORT_SUPPORTS_RT == TRUE) || defined(__DOXYGEN__)
static inline void osalSysPolledDelayX(rtcnt_t cycles) {

  chSysPolledDelayX(cycles);
}
#endif

/**
 * @brief   Systick callback for the underlying OS.
 * @note    This callback is only defined if the OSAL requires such a
 *          service from the HAL.
 */
#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)
static inline void osalOsTimerHandlerI(void) {

  chSysTimerHandlerI();
}
#endif

/**
 * @brief   Checks if a reschedule is required and performs it.
 * @note    I-Class functions invoked from thread context must not reschedule
 *          by themselves, an explicit reschedule using this function is
 *          required in this scenario.
 * @note    Not implemented in this simplified OSAL.
 *
 * @sclass
 */
static inline void osalOsRescheduleS(void) {

  chSchRescheduleS();
}

/**
 * @brief   Current system time.
 * @details Returns the number of system ticks since the @p osalInit()
 *          invocation.
 * @note    The counter can reach its maximum and then restart from zero.
 * @note    This function can be called from any context but its atomicity
 *          is not guaranteed on architectures whose word size is less than
 *          @p systime_t size.
 *
 * @return              The system time in ticks.
 *
 * @xclass
 */
static inline systime_t osalOsGetSystemTimeX(void) {

  return chVTGetSystemTimeX();
}

/**
 * @brief   Adds an interval to a system time returning a system time.
 *
 * @param[in] systime   base system time
 * @param[in] interval  interval to be added
 * @return              The new system time.
 *
 * @xclass
 */
static inline systime_t osalTimeAddX(systime_t systime,
                                     sysinterval_t interval) {

  return chTimeAddX(systime, interval);
}

/**
 * @brief   Subtracts two system times returning an interval.
 *
 * @param[in] start     first system time
 * @param[in] end       second system time
 * @return              The interval representing the time difference.
 *
 * @xclass
 */
static inline sysinterval_t osalTimeDiffX(systime_t start, systime_t end) {

  return chTimeDiffX(start, end);
}

/**
 * @brief   Checks if the specified time is within the specified time window.
 * @note    When start==end then the function returns always true because the
 *          whole time range is specified.
 * @note    This function can be called from any context.
 *
 * @param[in] time      the time to be verified
 * @param[in] start     the start of the time window (inclusive)
 * @param[in] end       the end of the time window (non inclusive)
 * @retval true         current time within the specified time window.
 * @retval false        current time not within the specified time window.
 *
 * @xclass
 */
static inline bool osalTimeIsInRangeX(systime_t time,
                                      systime_t start,
                                      systime_t end) {

  return chTimeIsInRangeX(time, start, end);
}

/**
 * @brief   Suspends the invoking thread for the specified time.
 *
 * @param[in] delay     the delay in system ticks, the special values are
 *                      handled as follow:
 *                      - @a TIME_INFINITE is allowed but interpreted as a
 *                        normal time specification.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 *                      .
 *
 * @sclass
 */
static inline void osalThreadSleepS(sysinterval_t delay) {

  chThdSleepS(delay);
}

/**
 * @brief   Suspends the invoking thread for the specified time.
 *
 * @param[in] delay     the delay in system ticks, the special values are
 *                      handled as follow:
 *                      - @a TIME_INFINITE is allowed but interpreted as a
 *                        normal time specification.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 *                      .
 *
 * @api
 */
static inline void osalThreadSleep(sysinterval_t delay) {

  chThdSleep(delay);
}

/**
 * @brief   Sends the current thread sleeping and sets a reference variable.
 * @note    This function must reschedule, it can only be called from thread
 *          context.
 *
 * @param[in] trp       a pointer to a thread reference object
 * @return              The wake up message.
 *
 * @sclass
 */
static inline msg_t osalThreadSuspendS(thread_reference_t *trp) {

  return chThdSuspendTimeoutS(trp, TIME_INFINITE);
}

/**
 * @brief   Sends the current thread sleeping and sets a reference variable.
 * @note    This function must reschedule, it can only be called from thread
 *          context.
 *
 * @param[in] trp       a pointer to a thread reference object
 * @param[in] timeout   the timeout in system ticks, the special values are
 *                      handled as follow:
 *                      - @a TIME_INFINITE the thread enters an infinite sleep
 *                        state.
 *                      - @a TIME_IMMEDIATE the thread is not enqueued and
 *                        the function returns @p MSG_TIMEOUT as if a timeout
 *                        occurred.
 *                      .
 * @return              The wake up message.
 * @retval MSG_TIMEOUT  if the operation timed out.
 *
 * @sclass
 */
static inline msg_t osalThreadSuspendTimeoutS(thread_reference_t *trp,
                                              sysinterval_t timeout) {

  return chThdSuspendTimeoutS(trp, timeout);
}

/**
 * @brief   Wakes up a thread waiting on a thread reference object.
 * @note    This function must not reschedule because it can be called from
 *          ISR context.
 *
 * @param[in] trp       a pointer to a thread reference object
 * @param[in] msg       the message code
 *
 * @iclass
 */
static inline void osalThreadResumeI(thread_reference_t *trp, msg_t msg) {

  chThdResumeI(trp, msg);
}

/**
 * @brief   Wakes up a thread waiting on a thread reference object.
 * @note    This function must reschedule, it can only be called from thread
 *          context.
 *
 * @param[in] trp       a pointer to a thread reference object
 * @param[in] msg       the message code
 *
 * @iclass
 */
static inline void osalThreadResumeS(thread_reference_t *trp, msg_t msg) {

  chThdResumeS(trp, msg);
}

/**
 * @brief   Initializes a threads queue object.
 *
 * @param[out] tqp      pointer to the threads queue object
 *
 * @init
 */
static inline void osalThreadQueueObjectInit(threads_queue_t *tqp) {

  chThdQueueObjectInit(tqp);
}

/**
 * @brief   Enqueues the caller thread.
 * @details The caller thread is enqueued and put to sleep until it is
 *          dequeued or the specified timeouts expires.
 *
 * @param[in] tqp       pointer to the threads queue object
 * @param[in] timeout   the timeout in system ticks, the special values are
 *                      handled as follow:
 *                      - @a TIME_INFINITE the thread enters an infinite sleep
 *                        state.
 *                      - @a TIME_IMMEDIATE the thread is not enqueued and
 *                        the function returns @p MSG_TIMEOUT as if a timeout
 *                        occurred.
 *                      .
 * @return              The message from @p osalQueueWakeupOneI() or
 *                      @p osalQueueWakeupAllI() functions.
 * @retval MSG_TIMEOUT  if the thread has not been dequeued within the
 *                      specified timeout or if the function has been
 *                      invoked with @p TIME_IMMEDIATE as timeout
 *                      specification.
 *
 * @sclass
 */
static inline msg_t osalThreadEnqueueTimeoutS(threads_queue_t *tqp,
                                              sysinterval_t timeout) {

  return chThdEnqueueTimeoutS(tqp, timeout);
}

/**
 * @brief   Dequeues and wakes up one thread from the queue, if any.
 *
 * @param[in] tqp       pointer to the threads queue object
 * @param[in] msg       the message code
 *
 * @iclass
 */
static inline void osalThreadDequeueNextI(threads_queue_t *tqp, msg_t msg) {

  chThdDequeueNextI(tqp, msg);
}

/**
 * @brief   Dequeues and wakes up all threads from the queue.
 *
 * @param[in] tqp       pointer to the threads queue object
 * @param[in] msg       the message code
 *
 * @iclass
 */
static inline void osalThreadDequeueAllI(threads_queue_t *tqp, msg_t msg) {

  chThdDequeueAllI(tqp, msg);
}

#if (CH_CFG_USE_EVENTS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Initializes an event source object.
 *
 * @param[out] esp      pointer to the event source object
 *
 * @init
 */
static inline void osalEventObjectInit(event_source_t *esp) {

  chEvtObjectInit(esp);
}
#else
static inline void osalEventObjectInit(event_source_t *esp) {

  osalDbgCheck(esp != NULL);

  esp->flags = (eventflags_t)0;
  esp->cb    = NULL;
  esp->param = NULL;
}
#endif

#if (CH_CFG_USE_EVENTS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Add flags to an event source object.
 *
 * @param[in] esp       pointer to the event flags object
 * @param[in] flags     flags to be ORed to the flags mask
 *
 * @iclass
 */
static inline void osalEventBroadcastFlagsI(event_source_t *esp,
                                            eventflags_t flags) {

  chEvtBroadcastFlagsI(esp, flags);
}
#else
static inline void osalEventBroadcastFlagsI(event_source_t *esp,
                                            eventflags_t flags) {

  osalDbgCheck(esp != NULL);

  esp->flags |= flags;
  if (esp->cb != NULL) {
    esp->cb(esp);
  }
}
#endif

#if (CH_CFG_USE_EVENTS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Add flags to an event source object.
 *
 * @param[in] esp       pointer to the event flags object
 * @param[in] flags     flags to be ORed to the flags mask
 *
 * @iclass
 */
static inline void osalEventBroadcastFlags(event_source_t *esp,
                                           eventflags_t flags) {

  chEvtBroadcastFlags(esp, flags);
}
#else
static inline void osalEventBroadcastFlags(event_source_t *esp,
                                           eventflags_t flags) {

  osalDbgCheck(esp != NULL);

  osalSysLock();
  esp->flags |= flags;
  if (esp->cb != NULL) {
    esp->cb(esp);
  }
  osalSysUnlock();
}
#endif

#if (CH_CFG_USE_EVENTS == FALSE) || defined(__DOXYGEN__)
/**
 * @brief   Event callback setup.
 * @note    The callback is invoked from ISR context and can
 *          only invoke I-Class functions. The callback is meant
 *          to wakeup the task that will handle the event by
 *          calling @p osalEventGetAndClearFlagsI().
 *
 * @param[in] esp       pointer to the event flags object
 * @param[in] cb        pointer to the callback function
 * @param[in] param     parameter to be passed to the callback function
 *
 * @api
 */
static inline void osalEventSetCallback(event_source_t *esp,
                                        eventcallback_t cb,
                                        void *param) {

  osalDbgCheck(esp != NULL);

  esp->cb    = cb;
  esp->param = param;
}
#endif

/**
 * @brief   Initializes a @p mutex_t object.
 *
 * @param[out] mp       pointer to the @p mutex_t object
 *
 * @init
 */
static inline void osalMutexObjectInit(mutex_t *mp) {

#if CH_CFG_USE_MUTEXES
  chMtxObjectInit(mp);
#elif CH_CFG_USE_SEMAPHORES
  chSemObjectInit((semaphore_t *)mp, 1);
#else
 *mp = 0;
#endif
}

/**
 * @brief   Locks the specified mutex.
 * @post    The mutex is locked and inserted in the per-thread stack of owned
 *          mutexes.
 *
 * @param[in,out] mp    pointer to the @p mutex_t object
 *
 * @api
 */
static inline void osalMutexLock(mutex_t *mp) {

#if CH_CFG_USE_MUTEXES
  chMtxLock(mp);
#elif CH_CFG_USE_SEMAPHORES
  chSemWait((semaphore_t *)mp);
#else
  *mp = 1;
#endif
}

/**
 * @brief   Unlocks the specified mutex.
 * @note    The HAL guarantees to release mutex in reverse lock order. The
 *          mutex being unlocked is guaranteed to be the last locked mutex
 *          by the invoking thread.
 *          The implementation can rely on this behavior and eventually
 *          ignore the @p mp parameter which is supplied in order to support
 *          those OSes not supporting a stack of the owned mutexes.
 *
 * @param[in,out] mp    pointer to the @p mutex_t object
 *
 * @api
 */
static inline void osalMutexUnlock(mutex_t *mp) {

#if CH_CFG_USE_MUTEXES
  chMtxUnlock(mp);
#elif CH_CFG_USE_SEMAPHORES
  chSemSignal((semaphore_t *)mp);
#else
  *mp = 0;
#endif
}

#endif /* OSAL_H */

/** @} */
