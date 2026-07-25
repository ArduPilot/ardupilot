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

#include "cmparams.h"

#include "osalconf.h"

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

/**
 * @name    Messages
 * @{
 */
#define MSG_OK                              (msg_t)0
#define MSG_RESET                           (msg_t)-1
#define MSG_TIMEOUT                         (msg_t)-2
#define MSG_WAIT                            (msg_t)-10
/** @} */

/**
 * @name    Special time constants
 * @{
 */
#define TIME_IMMEDIATE                      ((sysinterval_t)0)
#define TIME_INFINITE                       ((sysinterval_t)-1)
/** @} */

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
#define OSAL_ST_RESOLUTION                  32

/**
 * @brief   Systick mode required by the underlying OS.
 */
#define OSAL_ST_MODE                        OSAL_ST_MODE_PERIODIC
/** @} */

/**
 * @name    IRQ-related constants
 * @{
 */
/**
 * @brief   Total priority levels.
 */
#define OSAL_IRQ_PRIORITY_LEVELS            (1U << CORTEX_PRIORITY_BITS)

/**
 * @brief   Highest IRQ priority for HAL drivers.
 */
#if (CORTEX_MODEL == 0) || defined(__DOXYGEN__)
#define OSAL_IRQ_MAXIMUM_PRIORITY           0
#else
#define OSAL_IRQ_MAXIMUM_PRIORITY           1
#endif

/**
 * @brief   Converts from numeric priority to BASEPRI register value.
 */
#define OSAL_BASEPRI(priority)              ((priority) << (8U - CORTEX_PRIORITY_BITS))
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Frequency in Hertz of the system tick.
 */
#if !defined(OSAL_ST_FREQUENCY) || defined(__DOXYGEN__)
#define OSAL_ST_FREQUENCY                   1000
#endif

/**
 * @brief   Enables OSAL assertions.
 */
#if !defined(OSAL_DBG_ENABLE_ASSERTS) || defined(__DOXYGEN__)
#define OSAL_DBG_ENABLE_ASSERTS             FALSE
#endif

/**
 * @brief   Enables OSAL functions parameters checks.
 */
#if !defined(OSAL_DBG_ENABLE_CHECKS) || defined(__DOXYGEN__)
#define OSAL_DBG_ENABLE_CHECKS              FALSE
#endif

/**
 * @brief   OSAL initialization hook.
 */
#if !defined(OSAL_INIT_HOOK) || defined(__DOXYGEN__)
#define OSAL_INIT_HOOK()
#endif

/**
 * @brief   Idle loop hook macro.
 */
#if !defined(OSAL_IDLE_HOOK) || defined(__DOXYGEN__)
#define OSAL_IDLE_HOOK()
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a system status word.
 */
typedef uint32_t syssts_t;

/**
 * @brief   Type of a message.
 */
typedef int32_t msg_t;

/**
 * @brief   Type of system time counter.
 */
typedef uint32_t systime_t;

/**
 * @brief   Type of system time interval.
 */
typedef uint32_t sysinterval_t;

/**
 * @brief   Type of time conversion variable.
 * @note    This type must have double width than other time types, it is
 *          only used internally for conversions.
 */
typedef uint64_t time_conv_t;

/**
 * @brief   Type of realtime counter.
 */
typedef uint32_t rtcnt_t;

/**
 * @brief   Type of a thread.
 * @note    The content of this structure is not part of the API and should
 *          not be relied upon. Implementers may define this structure in
 *          an entirely different way.
 */
typedef struct {
  volatile msg_t        message;
} thread_t;

/**
 * @brief   Type of a thread reference.
 */
typedef thread_t * thread_reference_t;

/**
 * @brief   Type of an event flags mask.
 */
typedef uint32_t eventflags_t;

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

/**
 * @brief   Type of a mutex.
 * @note    If the OS does not support mutexes or there is no OS then them
 *          mechanism can be simulated.
 */
typedef uint32_t mutex_t;

/**
 * @brief   Type of a thread queue.
 * @details A thread queue is a queue of sleeping threads, queued threads
 *          can be dequeued one at time or all together.
 * @note    If the OSAL is implemented on a bare metal machine without RTOS
 *          then the queue can be implemented as a single thread reference.
 */
typedef struct {
  thread_reference_t    tr;
} threads_queue_t;

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
#define osalDbgAssert(c, remark) do {                                       \
  /*lint -save -e506 -e774 [2.1, 14.3] Can be a constant by design.*/       \
  if (OSAL_DBG_ENABLE_ASSERTS != FALSE) {                                   \
    if (!(c)) {                                                             \
  /*lint -restore*/                                                         \
      osalSysHalt(__func__);                                                \
    }                                                                       \
  }                                                                         \
} while (false)

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
#define osalDbgCheck(c) do {                                                \
  /*lint -save -e506 -e774 [2.1, 14.3] Can be a constant by design.*/       \
  if (OSAL_DBG_ENABLE_CHECKS != FALSE) {                                    \
    if (!(c)) {                                                             \
  /*lint -restore*/                                                         \
      osalSysHalt(__func__);                                                \
    }                                                                       \
  }                                                                         \
} while (false)

/**
 * @brief   I-Class state check.
 * @note    Implementation is optional.
 */
#define osalDbgCheckClassI()

/**
 * @brief   S-Class state check.
 * @note    Implementation is optional.
 */
#define osalDbgCheckClassS()
/** @} */

/**
 * @name    IRQ service routines wrappers
 * @{
 */
/**
 * @brief   Priority level verification macro.
 */
#define OSAL_IRQ_IS_VALID_PRIORITY(n)                                       \
  (((n) >= OSAL_IRQ_MAXIMUM_PRIORITY) && ((n) < OSAL_IRQ_PRIORITY_LEVELS))

/**
 * @brief   IRQ prologue code.
 * @details This macro must be inserted at the start of all IRQ handlers.
 */
#define OSAL_IRQ_PROLOGUE()

/**
 * @brief   IRQ epilogue code.
 * @details This macro must be inserted at the end of all IRQ handlers.
 */
#define OSAL_IRQ_EPILOGUE()

/**
 * @brief   IRQ handler function declaration.
 * @details This macro hides the details of an ISR function declaration.
 *
 * @param[in] id        a vector name as defined in @p vectors.s
 */
#define OSAL_IRQ_HANDLER(id) void id(void)
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
#define OSAL_S2I(secs)                                                      \
  ((sysinterval_t)((time_conv_t)(secs) * (time_conv_t)OSAL_ST_FREQUENCY))

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
#define OSAL_MS2I(msecs)                                                    \
  ((sysinterval_t)((((time_conv_t)(msecs) *                                 \
                     (time_conv_t)OSAL_ST_FREQUENCY) +                      \
                    (time_conv_t)999) / (time_conv_t)1000))

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
#define OSAL_US2I(usecs)                                                    \
  ((sysinterval_t)((((time_conv_t)(usecs) *                                 \
                     (time_conv_t)OSAL_ST_FREQUENCY) +                      \
                    (time_conv_t)999999) / (time_conv_t)1000000))

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
#define OSAL_I2S(interval)                                                  \
  (time_secs_t)(((time_conv_t)(interval) +                                  \
                 (time_conv_t)OSAL_ST_FREQUENCY -                           \
                 (time_conv_t)1) / (time_conv_t)OSAL_ST_FREQUENCY)

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
#define OSAL_I2MS(interval)                                                 \
  (time_msecs_t)((((time_conv_t)(interval) * (time_conv_t)1000) +           \
                  (time_conv_t)OSAL_ST_FREQUENCY - (time_conv_t)1) /        \
                 (time_conv_t)OSAL_ST_FREQUENCY)

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
#define OSAL_I2US(interval)                                                 \
  (time_msecs_t)((((time_conv_t)(interval) * (time_conv_t)1000000) +        \
                  (time_conv_t)OSAL_ST_FREQUENCY - (time_conv_t)1) /        \
                 (time_conv_t)OSAL_ST_FREQUENCY)
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
#define OSAL_S2RTC(freq, sec) ((freq) * (sec))

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
#define OSAL_MS2RTC(freq, msec) (rtcnt_t)((((freq) + 999UL) / 1000UL) * (msec))

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
#define OSAL_US2RTC(freq, usec) (rtcnt_t)((((freq) + 999999UL) / 1000000UL) * (usec))
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

extern const char *osal_halt_msg;

#ifdef __cplusplus
extern "C" {
#endif
  void osalInit(void);
  void osalSysHalt(const char *reason);
  void osalSysPolledDelayX(rtcnt_t cycles);
  void osalOsTimerHandlerI(void);
  void osalOsRescheduleS(void);
  systime_t osalOsGetSystemTimeX(void);
  void osalThreadSleepS(sysinterval_t time);
  void osalThreadSleep(sysinterval_t time);
  msg_t osalThreadSuspendS(thread_reference_t *trp);
  msg_t osalThreadSuspendTimeoutS(thread_reference_t *trp, sysinterval_t timeout);
  void osalThreadResumeI(thread_reference_t *trp, msg_t msg);
  void osalThreadResumeS(thread_reference_t *trp, msg_t msg);
  msg_t osalThreadEnqueueTimeoutS(threads_queue_t *tqp, sysinterval_t timeout);
  void osalThreadDequeueNextI(threads_queue_t *tqp, msg_t msg);
  void osalThreadDequeueAllI(threads_queue_t *tqp, msg_t msg);
  void osalEventBroadcastFlagsI(event_source_t *esp, eventflags_t flags);
  void osalEventBroadcastFlags(event_source_t *esp, eventflags_t flags);
  void osalEventSetCallback(event_source_t *esp,
                            eventcallback_t cb,
                            void *param);
  void osalMutexLock(mutex_t *mp);
  void osalMutexUnlock(mutex_t *mp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Disables interrupts globally.
 *
 * @special
 */
static inline void osalSysDisable(void) {

  __disable_irq();
}

/**
 * @brief   Enables interrupts globally.
 *
 * @special
 */
static inline void osalSysEnable(void) {

  __enable_irq();
}

/**
 * @brief   Enters a critical zone from thread context.
 * @note    This function cannot be used for reentrant critical zones.
 *
 * @special
 */
static inline void osalSysLock(void) {

#if CORTEX_MODEL == 0
  __disable_irq();
#else
  __set_BASEPRI(OSAL_BASEPRI(OSAL_IRQ_MAXIMUM_PRIORITY));
#endif
}

/**
 * @brief   Leaves a critical zone from thread context.
 * @note    This function cannot be used for reentrant critical zones.
 *
 * @special
 */
static inline void osalSysUnlock(void) {

#if CORTEX_MODEL == 0
  __enable_irq();
#else
  __set_BASEPRI(0);
#endif
}

/**
 * @brief   Enters a critical zone from ISR context.
 * @note    This function cannot be used for reentrant critical zones.
 *
 * @special
 */
static inline void osalSysLockFromISR(void) {

#if CORTEX_MODEL == 0
  __disable_irq();
#else
  __set_BASEPRI(OSAL_BASEPRI(OSAL_IRQ_MAXIMUM_PRIORITY));
#endif
}

/**
 * @brief   Leaves a critical zone from ISR context.
 * @note    This function cannot be used for reentrant critical zones.
 *
 * @special
 */
static inline void osalSysUnlockFromISR(void) {

#if CORTEX_MODEL == 0
  __enable_irq();
#else
  __set_BASEPRI(0);
#endif
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
  syssts_t sts;

#if CORTEX_MODEL == 0
  sts = (syssts_t)__get_PRIMASK();
  __disable_irq();
#else
  sts = (syssts_t)__get_BASEPRI();
  __set_BASEPRI(OSAL_BASEPRI(OSAL_IRQ_MAXIMUM_PRIORITY));
#endif
  return sts;
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

#if CORTEX_MODEL == 0
  if ((sts & (syssts_t)1) == (syssts_t)0) {
    __enable_irq();
  }
#else
  __set_BASEPRI(sts);
#endif
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

  return systime + (systime_t)interval;
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

  return (sysinterval_t)((systime_t)(end - start));
}

/**
 * @brief   Checks if the specified time is within the specified time window.
 * @note    When start==end then the function returns always false because the
 *          time window has zero size.
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

  return (bool)((systime_t)((systime_t)time - (systime_t)start) <
                (systime_t)((systime_t)end - (systime_t)start));
}

/**
 * @brief   Initializes a threads queue object.
 *
 * @param[out] tqp      pointer to the threads queue object
 *
 * @init
 */
static inline void osalThreadQueueObjectInit(threads_queue_t *tqp) {

  osalDbgCheck(tqp != NULL);
}

/**
 * @brief   Initializes an event source object.
 *
 * @param[out] esp      pointer to the event source object
 *
 * @init
 */
static inline void osalEventObjectInit(event_source_t *esp) {

  osalDbgCheck(esp != NULL);

  esp->flags = (eventflags_t)0;
  esp->cb    = NULL;
  esp->param = NULL;
}

/**
 * @brief   Initializes a @p mutex_t object.
 *
 * @param[out] mp       pointer to the @p mutex_t object
 *
 * @init
 */
static inline void osalMutexObjectInit(mutex_t *mp) {

  osalDbgCheck(mp != NULL);

  *mp = 0;
}

#endif /* OSAL_H */

/** @} */
