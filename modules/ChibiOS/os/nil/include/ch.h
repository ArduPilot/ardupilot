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
 * @file    nil/include/ch.h
 * @brief   Nil RTOS main header file.
 * @details This header includes all the required kernel headers so it is the
 *          only header you usually need to include in your application.
 *
 * @addtogroup NIL_KERNEL
 * @{
 */

#ifndef CH_H
#define CH_H

#include "chtypes.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @brief   ChibiOS/NIL identification macro.
 */
#define __CHIBIOS_NIL__

/**
 * @brief   Stable release flag.
 */
#define CH_KERNEL_STABLE        1

/**
 * @name    ChibiOS/NIL version identification
 * @{
 */
/**
 * @brief   Kernel version string.
 */
#define CH_KERNEL_VERSION       "4.1.4"

/**
 * @brief   Kernel version major number.
 */
#define CH_KERNEL_MAJOR         4

/**
 * @brief   Kernel version minor number.
 */
#define CH_KERNEL_MINOR         1

/**
 * @brief   Kernel version patch number.
 */
#define CH_KERNEL_PATCH         4
/** @} */

/**
 * @name    Constants for configuration options
 * @{
 */
/**
 * @brief   Generic 'false' preprocessor boolean constant.
 * @note    It is meant to be used in configuration files as switch.
 */
#if !defined(FALSE) || defined(__DOXYGEN__)
#define FALSE                   0
#endif

/**
 * @brief   Generic 'true' preprocessor boolean constant.
 * @note    It is meant to be used in configuration files as switch.
 */
#if !defined(TRUE) || defined(__DOXYGEN__)
#define TRUE                    1
#endif
/** @} */

/**
 * @name    Wakeup messages
 * @{
 */
#define MSG_OK                  (msg_t)0    /**< @brief OK wakeup message.  */
#define MSG_TIMEOUT             (msg_t)-1   /**< @brief Wake-up caused by
                                                 a timeout condition.       */
#define MSG_RESET               (msg_t)-2   /**< @brief Wake-up caused by
                                                 a reset condition.         */
/** @} */

/**
 * @name    Special time constants
 * @{
 */
/**
 * @brief   Zero time specification for some functions with a timeout
 *          specification.
 * @note    Not all functions accept @p TIME_IMMEDIATE as timeout parameter,
 *          see the specific function documentation.
 */
#define TIME_IMMEDIATE          ((sysinterval_t)-1)

/**
 * @brief   Infinite time specification for all functions with a timeout
 *          specification.
 */
#define TIME_INFINITE           ((sysinterval_t)0)

/**
 * @brief   Maximum interval constant usable as timeout.
 */
#define TIME_MAX_INTERVAL       ((sysinterval_t)-2)

/**
 * @brief   Maximum system of system time before it wraps.
 */
#define TIME_MAX_SYSTIME        ((systime_t)-1)
/** @} */

/**
 * @name    Thread state related macros
 * @{
 */
#define NIL_STATE_WTSTART       (tstate_t)0 /**< @brief Thread not yet
                                                 started or terminated.     */
#define NIL_STATE_READY         (tstate_t)1 /**< @brief Thread ready or
                                                 executing.                 */
#define NIL_STATE_SLEEPING      (tstate_t)2 /**< @brief Thread sleeping.    */
#define NIL_STATE_SUSPENDED     (tstate_t)3 /**< @brief Thread suspended.   */
#define NIL_STATE_WTEXIT        (tstate_t)4 /**< @brief Waiting a thread.   */
#define NIL_STATE_WTQUEUE       (tstate_t)5 /**< @brief On queue or semaph. */
#define NIL_STATE_WTOREVT       (tstate_t)6 /**< @brief Waiting for events. */
#define NIL_STATE_WTANDEVT      (tstate_t)7 /**< @brief Waiting for events. */
#define NIL_STATE_SNDMSGQ       (tstate_t)8 /**< @brief Sending a message,
                                                        in queue.           */
#define NIL_STATE_WTMSG         (tstate_t)10/**< @brief Waiting for a
                                                        message.            */
#define NIL_STATE_FINAL         (tstate_t)11/**< @brief Thread terminated.  */

#define NIL_THD_IS_WTSTART(tp)      ((tp)->state == NIL_STATE_WTSTART)
#define NIL_THD_IS_READY(tp)        ((tp)->state == NIL_STATE_READY)
#define NIL_THD_IS_SLEEPING(tp)     ((tp)->state == NIL_STATE_SLEEPING)
#define NIL_THD_IS_SUSPENDED(tp)    ((tp)->state == NIL_STATE_SUSPENDED)
#define NIL_THD_IS_WTEXIT(tp)       ((tp)->state == NIL_STATE_WTEXIT)
#define NIL_THD_IS_WTQUEUE(tp)      ((tp)->state == NIL_STATE_WTQUEUE)
#define NIL_THD_IS_WTOREVT(tp)      ((tp)->state == NIL_STATE_WTOREVT)
#define NIL_THD_IS_WTANDEVT(tp)     ((tp)->state == NIL_STATE_WTANDEVT)
#define NIL_THD_IS_SNDMSGQ(tp)      ((tp)->state == NIL_STATE_SNDMSGQ)
#define NIL_THD_IS_WTMSG(tp)        ((tp)->state == NIL_STATE_WTMSG)
#define NIL_THD_IS_FINAL(tp)        ((tp)->state == NIL_STATE_FINAL)

#define CH_STATE_NAMES                                                      \
  "WTSTART", "READY", "SLEEPING", "SUSPENDED", "WTEXIT", "WTQUEUE",         \
  "WTOREVT", "WTANDEVT", "SNDMSGQ", "SNDMSG", "WTMSG", "FINAL"
/** @} */

/**
 * @name    RT options not existing in NIL
 * @{
 */
#define CH_CFG_USE_REGISTRY         FALSE
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

#include "chconf.h"
#include "chlicense.h"

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Checks on configuration options.*/
#if !defined(CH_CFG_MAX_THREADS) || defined(__DOXYGEN__)
#error "CH_CFG_MAX_THREADS not defined in chconf.h"
#endif

#if !defined(CH_CFG_AUTOSTART_THREADS) || defined(__DOXYGEN__)
#error "CH_CFG_AUTOSTART_THREADS not defined in chconf.h"
#endif

#if !defined(CH_CFG_ST_RESOLUTION) || defined(__DOXYGEN__)
#error "CH_CFG_ST_RESOLUTION not defined in chconf.h"
#endif

#if !defined(CH_CFG_ST_FREQUENCY) || defined(__DOXYGEN__)
#error "CH_CFG_ST_FREQUENCY not defined in chconf.h"
#endif

#if !defined(CH_CFG_ST_TIMEDELTA) || defined(__DOXYGEN__)
#error "CH_CFG_ST_TIMEDELTA not defined in chconf.h"
#endif

#if !defined(CH_CFG_USE_WAITEXIT)
#error "CH_CFG_USE_WAITEXIT not defined in chconf.h"
#endif

#if !defined(CH_CFG_USE_MESSAGES) || defined(__DOXYGEN__)
#error "CH_CFG_USE_MESSAGES not defined in chconf.h"
#endif

#if !defined(CH_CFG_USE_SEMAPHORES) || defined(__DOXYGEN__)
#error "CH_CFG_USE_SEMAPHORES not defined in chconf.h"
#endif

#if !defined(CH_CFG_USE_EVENTS)
#error "CH_CFG_USE_EVENTS not defined in chconf.h"
#endif

#if !defined(CH_CFG_USE_MUTEXES) || defined(__DOXYGEN__)
#error "CH_CFG_USE_MUTEXES not defined in chconf.h"
#endif

#if !defined(CH_DBG_STATISTICS) || defined(__DOXYGEN__)
#error "CH_DBG_STATISTICS not defined in chconf.h"
#endif

#if !defined(CH_DBG_SYSTEM_STATE_CHECK) || defined(__DOXYGEN__)
#error "CH_DBG_SYSTEM_STATE_CHECK not defined in chconf.h"
#endif

#if !defined(CH_DBG_ENABLE_CHECKS) || defined(__DOXYGEN__)
#error "CH_DBG_ENABLE_CHECKS not defined in chconf.h"
#endif

#if !defined(CH_DBG_ENABLE_ASSERTS) || defined(__DOXYGEN__)
#error "CH_DBG_ENABLE_ASSERTS not defined in chconf.h"
#endif

#if !defined(CH_DBG_ENABLE_STACK_CHECK) || defined(__DOXYGEN__)
#error "CH_DBG_ENABLE_STACK_CHECK not defined in chconf.h"
#endif

#if !defined(CH_CFG_SYSTEM_INIT_HOOK) || defined(__DOXYGEN__)
#error "CH_CFG_SYSTEM_INIT_HOOK not defined in chconf.h"
#endif

#if !defined(CH_CFG_THREAD_EXT_FIELDS) || defined(__DOXYGEN__)
#error "CH_CFG_THREAD_EXT_FIELDS not defined in chconf.h"
#endif

#if !defined(CH_CFG_THREAD_EXT_INIT_HOOK) || defined(__DOXYGEN__)
#error "CH_CFG_THREAD_EXT_INIT_HOOK not defined in chconf.h"
#endif

#if !defined(CH_CFG_THREAD_EXIT_HOOK) || defined(__DOXYGEN__)
#error "CH_CFG_THREAD_EXIT_HOOK not defined in chconf.h"
#endif

#if !defined(CH_CFG_IDLE_ENTER_HOOK) || defined(__DOXYGEN__)
#error "CH_CFG_IDLE_ENTER_HOOK not defined in chconf.h"
#endif

#if !defined(CH_CFG_IDLE_LEAVE_HOOK) || defined(__DOXYGEN__)
#error "CH_CFG_IDLE_LEAVE_HOOK not defined in chconf.h"
#endif

#if !defined(CH_CFG_SYSTEM_HALT_HOOK) || defined(__DOXYGEN__)
#error "CH_CFG_SYSTEM_HALT_HOOK not defined in chconf.h"
#endif

/* License checks.*/
#if !defined(CH_CUSTOMER_LIC_NIL) || !defined(CH_LICENSE_FEATURES)
#error "malformed chlicense.h"
#endif

#if CH_CUSTOMER_LIC_NIL == FALSE
#error "ChibiOS/NIL not licensed"
#endif

#if (CH_LICENSE_FEATURES != CH_FEATURES_FULL) &&                            \
    (CH_LICENSE_FEATURES != CH_FEATURES_INTERMEDIATE) &&                    \
    (CH_LICENSE_FEATURES != CH_FEATURES_BASIC)
#error "invalid CH_LICENSE_FEATURES setting"
#endif

/* Restrictions in basic and intermediate modes.*/
#if (CH_LICENSE_FEATURES == CH_FEATURES_INTERMEDIATE) ||                    \
    (CH_LICENSE_FEATURES == CH_FEATURES_BASIC)

/* System tick limited to 1000hz.*/
#if CH_CFG_ST_FREQUENCY > 1000
#undef CH_CFG_ST_FREQUENCY
#define CH_CFG_ST_FREQUENCY                 1000
#endif

#endif /* (CH_LICENSE_FEATURES == CH_FEATURES_INTERMEDIATE) ||
          (CH_LICENSE_FEATURES == CH_FEATURES_BASIC) */

/* Restrictions in basic mode.*/
#if CH_LICENSE_FEATURES == CH_FEATURES_BASIC

/* Tick-Less mode restricted.*/
#undef CH_CFG_ST_TIMEDELTA
#define CH_CFG_ST_TIMEDELTA                 0

/* Messages restricted.*/
#undef CH_CFG_USE_MESSAGES
#define CH_CFG_USE_MESSAGES                 FALSE

#endif /* CH_LICENSE_FEATURES == CH_FEATURES_BASIC */

#if !defined(_CHIBIOS_NIL_CONF_)
#error "missing or wrong configuration file"
#endif

#if !defined(_CHIBIOS_NIL_CONF_VER_4_0_)
#error "obsolete or unknown configuration file"
#endif

#if CH_CFG_MAX_THREADS < 1
#error "at least one thread must be defined"
#endif

#if CH_CFG_MAX_THREADS > 16
#error "ChibiOS/NIL is not recommended for thread-intensive applications,"  \
       "consider ChibiOS/RT instead"
#endif

#if (CH_CFG_ST_RESOLUTION != 16) && (CH_CFG_ST_RESOLUTION != 32)
#error "invalid CH_CFG_ST_RESOLUTION specified, must be 16 or 32"
#endif

#if CH_CFG_ST_FREQUENCY <= 0
#error "invalid CH_CFG_ST_FREQUENCY specified, must be greater than zero"
#endif

#if (CH_CFG_ST_TIMEDELTA < 0) || (CH_CFG_ST_TIMEDELTA == 1)
#error "invalid CH_CFG_ST_TIMEDELTA specified, must "                       \
       "be zero or greater than one"
#endif

#if CH_CFG_USE_MUTEXES == TRUE
#error "mutexes not yet supported"
#endif

#if CH_DBG_STATISTICS == TRUE
#error "statistics not yet supported"
#endif

#if (CH_DBG_SYSTEM_STATE_CHECK == TRUE) ||                                  \
    (CH_DBG_ENABLE_CHECKS == TRUE)      ||                                  \
    (CH_DBG_ENABLE_ASSERTS == TRUE)     ||                                  \
    (CH_DBG_ENABLE_STACK_CHECK == TRUE)
#define NIL_DBG_ENABLED                 TRUE
#else
#define NIL_DBG_ENABLED                 FALSE
#endif

/** Boundaries of the idle thread boundaries, only required if stack checking
    is enabled.*/
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || defined(__DOXYGEN__)
#define THD_IDLE_BASE                   (&__main_thread_stack_base__)
#define THD_IDLE_END                    (&__main_thread_stack_end__)
#else
#define THD_IDLE_BASE                   NULL
#define THD_IDLE_END                    NULL
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

#if defined(PORT_DOES_NOT_PROVIDE_TYPES) || defined(__DOXYGEN__)
/**
 * @name    Kernel types
 * @{
 */
typedef port_rtcnt_t    rtcnt_t;            /**< Realtime counter.          */
typedef port_syssts_t   syssts_t;           /**< System status word.        */
typedef port_stkalign_t stkalign_t;         /**< Stack alignment type.      */

#if (PORT_ARCH_REGISTERS_WIDTH == 32) || defined(__DOXYGEN__)
typedef uint8_t         tstate_t;           /**< Thread state.              */
typedef uint32_t        tprio_t;            /**< Thread priority.           */
typedef int32_t         msg_t;              /**< Inter-thread message.      */
typedef int32_t         eventid_t;          /**< Numeric event identifier.  */
typedef uint32_t        eventmask_t;        /**< Mask of event identifiers. */
typedef uint32_t        eventflags_t;       /**< Mask of event flags.       */
typedef int32_t         cnt_t;              /**< Generic signed counter.    */
typedef uint32_t        ucnt_t;             /**< Generic unsigned counter.  */
#elif PORT_ARCH_REGISTERS_WIDTH == 16
typedef uint8_t         tstate_t;           /**< Thread state.              */
typedef uint16_t        tprio_t;            /**< Thread priority.           */
typedef int16_t         msg_t;              /**< Inter-thread message.      */
typedef int16_t         eventid_t;          /**< Numeric event identifier.  */
typedef uint16_t        eventmask_t;        /**< Mask of event identifiers. */
typedef uint16_t        eventflags_t;       /**< Mask of event flags.       */
typedef int16_t         cnt_t;              /**< Generic signed counter.    */
typedef uint16_t        ucnt_t;             /**< Generic unsigned counter.  */
#elif PORT_ARCH_REGISTERS_WIDTH == 8
typedef uint8_t         tstate_t;           /**< Thread state.              */
typedef uint8_t         tprio_t;            /**< Thread priority.           */
typedef int16_t         msg_t;              /**< Inter-thread message.      */
typedef int8_t          eventid_t;          /**< Numeric event identifier.  */
typedef uint8_t         eventmask_t;        /**< Mask of event identifiers. */
typedef uint8_t         eventflags_t;       /**< Mask of event flags.       */
typedef int8_t          cnt_t;              /**< Generic signed counter.    */
typedef uint8_t         ucnt_t;             /**< Generic unsigned counter.  */
#else
#error "unsupported PORT_ARCH_REGISTERS_WIDTH value"
#endif
/** @} */
#endif

#if (CH_CFG_ST_RESOLUTION == 32) || defined(__DOXYGEN__)
/**
 * @brief   Type of system time.
 * @note    It is selectable in configuration between 16 or 32 bits.
 */
typedef uint32_t systime_t;

/**
 * @brief   Type of time interval.
 * @note    It is selectable in configuration between 16 or 32 bits.
 */
typedef uint32_t sysinterval_t;

/**
 * @brief   Type of time conversion variable.
 * @note    This type must have double width than other time types, it is
 *          only used internally for conversions.
 */
typedef uint64_t time_conv_t;

#else
typedef uint16_t systime_t;
typedef uint16_t sysinterval_t;
typedef uint32_t time_conv_t;
#endif

/**
 * @brief   Type of a structure representing the system.
 */
typedef struct nil_os_instance os_instance_t;

/**
 * @brief Thread function.
 */
typedef void (*tfunc_t)(void *p);

/**
 * @brief   Type of a thread descriptor.
 */
typedef struct nil_thread_descriptor thread_descriptor_t;

/**
 * @brief   Type of a structure representing a thread.
 * @note    It is required as an early definition.
 */
typedef struct nil_thread thread_t;

/**
 * @brief   Type of a thread reference.
 */
typedef thread_t * thread_reference_t;

/**
 * @brief   Type of a queue of threads.
 */
typedef struct nil_threads_queue threads_queue_t;

#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Type of a structure representing a semaphore.
 * @note    Semaphores are implemented on thread queues, the object is the
 *          same, the behavior is slightly different.
 */
typedef threads_queue_t semaphore_t;
#endif /* CH_CFG_USE_SEMAPHORES == TRUE */

/* Late inclusion of port core layer.*/
#include "chcore.h"

/* Recursive locks port capability assessed.*/
#if defined(port_get_lock_status) && defined(port_is_locked)
#define CH_PORT_SUPPORTS_RECURSIVE_LOCKS    TRUE
#else
#define CH_PORT_SUPPORTS_RECURSIVE_LOCKS    FALSE
#endif

/**
 * @brief   Structure representing a queue of threads.
 */
struct nil_threads_queue {
  volatile cnt_t    cnt;        /**< @brief Threads Queue counter.          */
};

/**
 * @brief   Structure representing a thread descriptor.
 */
struct nil_thread_descriptor {
  const char        *name;      /**< @brief Thread name, for debugging.     */
  stkalign_t        *wbase;     /**< @brief Thread working area base.       */
  stkalign_t        *wend;      /**< @brief Thread working area end.        */
  tprio_t           prio;       /**< @brief Thread priority slot.           */
  tfunc_t           funcp;      /**< @brief Thread function.                */
  void              *arg;       /**< @brief Thread function argument.       */
};

/**
 * @brief   Structure representing a thread.
 */
struct nil_thread {
  struct port_context   ctx;        /**< @brief Processor context.          */
  tstate_t              state;      /**< @brief Thread state.               */
  /* Note, the following union contains a pointer/value while the thread is
     in a sleeping state or a wake-up message when the thread is made ready.*/
  union {
    msg_t               msg;        /**< @brief Wake-up/exit message.       */
    void                *p;         /**< @brief Generic pointer.            */
    os_instance_t       *nsp;       /**< @brief Pointer to nil base struct. */
    thread_reference_t  *trp;       /**< @brief Pointer to thread reference.*/
    threads_queue_t     *tqp;       /**< @brief Pointer to thread queue.    */
    thread_t            *tp;        /**< @brief Pointer to thread.          */
#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
    semaphore_t         *semp;      /**< @brief Pointer to semaphore.       */
#endif
#if (CH_CFG_USE_EVENTS == TRUE) || defined(__DOXYGEN__)
    eventmask_t         ewmask;     /**< @brief Enabled events mask.        */
#endif
  } u1;
  volatile sysinterval_t timeout;   /**< @brief Timeout counter, zero
                                                if disabled.                */
#if (CH_CFG_USE_EVENTS == TRUE) || defined(__DOXYGEN__)
  eventmask_t           epmask;     /**< @brief Pending events mask.        */
#endif
#if (CH_CFG_USE_MESSAGES == TRUE) || defined(__DOXYGEN__)
  msg_t                 sntmsg;     /**< @brief Sent message.               */
#endif
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || defined(__DOXYGEN__)
  stkalign_t            *wabase;    /**< @brief Thread stack boundary.      */
#endif
  /* Optional extra fields.*/
  CH_CFG_THREAD_EXT_FIELDS
};

/**
 * @brief   System data structure.
 * @note    This structure contain all the data areas used by the OS except
 *          stacks.
 */
struct nil_os_instance {
  /**
   * @brief   Pointer to the running thread.
   */
  thread_t              *current;
  /**
   * @brief   Pointer to the next thread to be executed.
   * @note    This pointer must point at the same thread pointed by @p current
   *          or to an higher priority thread if a switch is required.
   */
  thread_t              *next;
#if (CH_CFG_ST_TIMEDELTA == 0) || defined(__DOXYGEN__)
  /**
   * @brief   System time.
   */
  volatile systime_t    systime;
#endif
#if (CH_CFG_ST_TIMEDELTA > 0) || defined(__DOXYGEN__)
  /**
   * @brief   System time of the last tick event.
   */
  systime_t             lasttime;
  /**
   * @brief   Time of the next scheduled tick event.
   */
  systime_t             nexttime;
#endif
#if (CH_DBG_SYSTEM_STATE_CHECK == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   ISR nesting level.
   */
  cnt_t                 isr_cnt;
  /**
   * @brief   Lock nesting level.
   */
  cnt_t                 lock_cnt;
#endif
#if (NIL_DBG_ENABLED == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Panic message.
   * @note    This field is only present if some debug options have been
   *          activated.
   * @note    Accesses to this pointer must never be optimized out so the
   *          field itself is declared volatile.
   */
  const char            * volatile dbg_panic_msg;
#endif
  /**
   * @brief   Thread structures for all the defined threads.
   */
  thread_t              threads[CH_CFG_MAX_THREADS + 1];
};

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

#if CH_DBG_SYSTEM_STATE_CHECK == TRUE
#define __dbg_enter_lock() (nil.lock_cnt = (cnt_t)1)
#define __dbg_leave_lock() (nil.lock_cnt = (cnt_t)0)
#endif

/**
 * @brief   Utility to make the parameter a quoted string.
 */
#define __CH_STRINGIFY(a) #a

/**
 * @name    Threads tables definition macros
 * @{
 */
/**
 * @brief   Start of user threads table.
 */
#define THD_TABLE_BEGIN                                                     \
  const thread_descriptor_t nil_thd_configs[] = {

/**
 * @brief   Entry of user threads table
 */
#define THD_TABLE_THREAD(_prio, _name, _wap, _funcp, _arg)                  \
  {                                                                         \
    .name  = (_name),                                                       \
    .wbase = (_wap),                                                        \
    .wend  = THD_WORKING_AREA_END(_wap),                                    \
    .prio  = (_prio),                                                       \
    .funcp = (_funcp),                                                      \
    .arg   = (_arg)                                                         \
  },

/**
 * @brief   End of user threads table.
 */
#define THD_TABLE_END                                                       \
  {                                                                         \
    .name  = "idle",                                                        \
    .wbase = THD_IDLE_BASE,                                                 \
    .wend  = THD_IDLE_END,                                                  \
    .prio  = CH_CFG_MAX_THREADS,                                            \
    .funcp = NULL,                                                          \
    .arg  = NULL                                                            \
  }                                                                         \
};
/** @} */

/**
 * @name    Memory alignment support macros
 * @{
 */
/**
 * @brief   Alignment mask constant.
 *
 * @param[in] a         alignment, must be a power of two
 */
#define MEM_ALIGN_MASK(a)       ((size_t)(a) - 1U)

/**
 * @brief   Aligns to the previous aligned memory address.
 *
 * @param[in] p         variable to be aligned
 * @param[in] a         alignment, must be a power of two
 */
#define MEM_ALIGN_PREV(p, a)    ((size_t)(p) & ~MEM_ALIGN_MASK(a))

/**
 * @brief   Aligns to the new aligned memory address.
 *
 * @param[in] p         variable to be aligned
 * @param[in] a         alignment, must be a power of two
 */
#define MEM_ALIGN_NEXT(p, a)    MEM_ALIGN_PREV((size_t)(p) +                \
                                               MEM_ALIGN_MASK(a), (a))

/**
 * @brief   Returns whatever a pointer or memory size is aligned.
 *
 * @param[in] p         variable to be aligned
 * @param[in] a         alignment, must be a power of two
 */
#define MEM_IS_ALIGNED(p, a)    (((size_t)(p) & MEM_ALIGN_MASK(a)) == 0U)

/**
 * @brief   Returns whatever a constant is a valid alignment.
 * @details Valid alignments are powers of two.
 *
 * @param[in] a         alignment to be checked, must be a constant
 */
#define MEM_IS_VALID_ALIGNMENT(a)                                           \
  (((size_t)(a) != 0U) && (((size_t)(a) & ((size_t)(a) - 1U)) == 0U))
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
#define THD_WORKING_AREA_SIZE(n) MEM_ALIGN_NEXT(PORT_WA_SIZE(n),            \
                                                PORT_STACK_ALIGN)

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
/** @} */

/**
 * @brief   Returns the top address of a working area.
 * @note    The parameter is assumed to be an array of @p stkalign_t. The
 *          macros is invalid for anything else.
 *
 * @param[in] wa        working area array
 *
 * @api
 */
#define THD_WORKING_AREA_END(wa)                                            \
  ((wa) + ((sizeof wa) / sizeof (stkalign_t)))

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
 * @name    ISRs abstraction macros
 * @{
 */
/**
 * @brief   Priority level validation macro.
 * @details This macro determines if the passed value is a valid priority
 *          level for the underlying architecture.
 *
 * @param[in] prio      the priority level
 * @return              Priority range result.
 * @retval false        if the priority is invalid or if the architecture
 *                      does not support priorities.
 * @retval true         if the priority is valid.
 */
#if defined(PORT_IRQ_IS_VALID_PRIORITY) || defined(__DOXYGEN__)
#define CH_IRQ_IS_VALID_PRIORITY(prio)                                      \
  PORT_IRQ_IS_VALID_PRIORITY(prio)
#else
#define CH_IRQ_IS_VALID_PRIORITY(prio) false
#endif

/**
 * @brief   Priority level validation macro.
 * @details This macro determines if the passed value is a valid priority
 *          level that cannot preempt the kernel critical zone.
 *
 * @param[in] prio      the priority level
 * @return              Priority range result.
 * @retval false        if the priority is invalid or if the architecture
 *                      does not support priorities.
 * @retval true         if the priority is valid.
 */
#if defined(PORT_IRQ_IS_VALID_KERNEL_PRIORITY) || defined(__DOXYGEN__)
#define CH_IRQ_IS_VALID_KERNEL_PRIORITY(prio)                               \
  PORT_IRQ_IS_VALID_KERNEL_PRIORITY(prio)
#else
#define CH_IRQ_IS_VALID_KERNEL_PRIORITY(prio) false
#endif

/**
 * @brief   IRQ handler enter code.
 * @note    Usually IRQ handlers functions are also declared naked.
 * @note    On some architectures this macro can be empty.
 *
 * @special
 */
#define CH_IRQ_PROLOGUE()                                                   \
  PORT_IRQ_PROLOGUE();                                                      \
  __dbg_check_enter_isr()

/**
 * @brief   IRQ handler exit code.
 * @note    Usually IRQ handlers function are also declared naked.
 *
 * @special
 */
#define CH_IRQ_EPILOGUE()                                                   \
  __dbg_check_leave_isr();                                                  \
  PORT_IRQ_EPILOGUE()

/**
 * @brief   Standard normal IRQ handler declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 *
 * @special
 */
#define CH_IRQ_HANDLER(id) PORT_IRQ_HANDLER(id)
/** @} */

/**
 * @name    Fast ISRs abstraction macros
 * @{
 */
/**
 * @brief   Standard fast IRQ handler declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 * @note    Not all architectures support fast interrupts.
 *
 * @special
 */
#define CH_FAST_IRQ_HANDLER(id) PORT_FAST_IRQ_HANDLER(id)
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
#define TIME_S2I(secs)                                                      \
  ((sysinterval_t)((time_conv_t)(secs) * (time_conv_t)CH_CFG_ST_FREQUENCY))

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
#define TIME_MS2I(msecs)                                                    \
  ((sysinterval_t)((((time_conv_t)(msecs) *                                 \
                     (time_conv_t)CH_CFG_ST_FREQUENCY) +                    \
                    (time_conv_t)999) / (time_conv_t)1000))

/**
 * @brief   Microseconds to time interval.
 * @details Converts from microseconds to system ticks number.
 * @note    The result is rounded upward to the next tick boundary.
 * @note    Use of this macro for large values is not secure because
 *          integer overflows, make sure your value can be correctly
 *          converted.
 *
 * @param[in] usecs      number of microseconds
 * @return              The number of ticks.
 *
 * @api
 */
#define TIME_US2I(usecs)                                                    \
  ((sysinterval_t)((((time_conv_t)(usecs) *                                 \
                     (time_conv_t)CH_CFG_ST_FREQUENCY) +                    \
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
#define TIME_I2S(interval)                                                  \
  (time_secs_t)(((time_conv_t)(interval) +                                  \
                 (time_conv_t)CH_CFG_ST_FREQUENCY -                         \
                 (time_conv_t)1) / (time_conv_t)CH_CFG_ST_FREQUENCY)

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
#define TIME_I2MS(interval)                                                 \
  (time_msecs_t)((((time_conv_t)(interval) * (time_conv_t)1000) +           \
                  (time_conv_t)CH_CFG_ST_FREQUENCY - (time_conv_t)1) /      \
                 (time_conv_t)CH_CFG_ST_FREQUENCY)

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
#define TIME_I2US(interval)                                                 \
    (time_usecs_t)((((time_conv_t)(interval) * (time_conv_t)1000000) +      \
                    (time_conv_t)CH_CFG_ST_FREQUENCY - (time_conv_t)1) /    \
                   (time_conv_t)CH_CFG_ST_FREQUENCY)
/** @} */

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
#define __THREADS_QUEUE_DATA(name) {(cnt_t)0}

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
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Returns the current value of the system real time counter.
 * @note    This function is only available if the port layer supports the
 *          option @p PORT_SUPPORTS_RT.
 *
 * @return              The value of the system realtime counter of
 *                      type rtcnt_t.
 *
 * @xclass
 */
#if (PORT_SUPPORTS_RT == TRUE) || defined(__DOXYGEN__)
#define chSysGetRealtimeCounterX() (rtcnt_t)port_rt_get_counter_value()
#endif

/**
 * @brief   Raises the system interrupt priority mask to the maximum level.
 * @details All the maskable interrupt sources are disabled regardless their
 *          hardware priority.
 * @note    Do not invoke this API from within a kernel lock.
 *
 * @special
 */
#define chSysDisable() {                                                    \
  port_disable();                                                           \
  __dbg_check_disable();                                                    \
}

/**
 * @brief   Raises the system interrupt priority mask to system level.
 * @details The interrupt sources that should not be able to preempt the kernel
 *          are disabled, interrupt sources with higher priority are still
 *          enabled.
 * @note    Do not invoke this API from within a kernel lock.
 * @note    This API is no replacement for @p chSysLock(), the @p chSysLock()
 *          could do more than just disable the interrupts.
 *
 * @special
 */
#define chSysSuspend() {                                                    \
  port_suspend();                                                           \
  __dbg_check_suspend();                                                    \
}

/**
 * @brief   Lowers the system interrupt priority mask to user level.
 * @details All the interrupt sources are enabled.
 * @note    Do not invoke this API from within a kernel lock.
 * @note    This API is no replacement for @p chSysUnlock(), the
 *          @p chSysUnlock() could do more than just enable the interrupts.
 *
 * @special
 */
#define chSysEnable() {                                                     \
  __dbg_check_enable();                                                     \
  port_enable();                                                            \
}

/**
 * @brief   Enters the kernel lock state.
 *
 * @special
 */
#define chSysLock() {                                                       \
  port_lock();                                                              \
  __dbg_check_lock();                                                       \
}

/**
 * @brief   Leaves the kernel lock state.
 *
 * @special
 */
#define chSysUnlock() {                                                     \
  __dbg_check_unlock();                                                     \
  port_unlock();                                                            \
}

/**
 * @brief   Enters the kernel lock state from within an interrupt handler.
 * @note    This API may do nothing on some architectures, it is required
 *          because on ports that support preemptable interrupt handlers
 *          it is required to raise the interrupt mask to the same level of
 *          the system mutual exclusion zone.<br>
 *          It is good practice to invoke this API before invoking any I-class
 *          syscall from an interrupt handler.
 * @note    This API must be invoked exclusively from interrupt handlers.
 *
 * @special
 */
#define chSysLockFromISR() {                                                \
  port_lock_from_isr();                                                     \
  __dbg_check_lock_from_isr();                                              \
}

/**
 * @brief   Leaves the kernel lock state from within an interrupt handler.
 *
 * @note    This API may do nothing on some architectures, it is required
 *          because on ports that support preemptable interrupt handlers
 *          it is required to raise the interrupt mask to the same level of
 *          the system mutual exclusion zone.<br>
 *          It is good practice to invoke this API after invoking any I-class
 *          syscall from an interrupt handler.
 * @note    This API must be invoked exclusively from interrupt handlers.
 *
 * @special
 */
#define chSysUnlockFromISR() {                                              \
  __dbg_check_unlock_from_isr();                                            \
  port_unlock_from_isr();                                                   \
}

/**
 * @brief   Puts the current thread to sleep into the specified state.
 *
 * @param[in] newstate  the new thread state or a semaphore pointer
 * @return              The wakeup message.
 *
 * @sclass
 */
#define chSchGoSleepS(newstate) chSchGoSleepTimeoutS(newstate, TIME_INFINITE)

/**
 * @brief   Wakes up a thread.
 *
 * @param[in] ntp       the thread to be made ready
 * @param[in] msg       the wakeup message
 *
 * @sclass
 */
#define chSchWakeupS(ntp, msg) do {                                         \
  chSchReadyI(ntp, msg);                                                    \
  chSchRescheduleS();                                                       \
} while (false)

/**
 * @brief   Evaluates if a reschedule is required.
 *
 * @retval true         if there is a thread that must go in running state
 *                      immediately.
 * @retval false        if preemption is not required.
 *
 * @iclass
 */
#define chSchIsRescRequiredI() ((bool)(nil.current != nil.next))

/**
 * @brief   Returns a pointer to the current @p thread_t.
 *
 * @xclass
 */
#define chThdGetSelfX() nil.current

/**
 * @brief   Returns the current thread priority.
 * @note    Can be invoked in any context.
 *
 * @return              The current thread priority.
 *
 * @xclass
 */
#define chThdGetPriorityX(void) (tprio_t)(nil.current - &nil.threads[0])

/**
 * @brief   Wakes up a thread waiting on a thread reference object.
 * @note    This function must reschedule, it can only be called from thread
 *          context.
 *
 * @param[in] trp       a pointer to a thread reference object
 * @param[in] msg       the message code
 *
 * @sclass
 */
#define chThdResumeS(trp, msg) do {                                         \
  chThdResumeI(trp, msg);                                                   \
  chSchRescheduleS();                                                       \
} while (false)

/**
 * @brief   Delays the invoking thread for the specified number of seconds.
 * @note    The specified time is rounded up to a value allowed by the real
 *          system clock.
 * @note    The maximum specified value is implementation dependent.
 *
 * @param[in] secs      time in seconds, must be different from zero
 *
 * @api
 */
#define chThdSleepSeconds(secs) chThdSleep(TIME_S2I(secs))

/**
 * @brief   Delays the invoking thread for the specified number of
 *          milliseconds.
 * @note    The specified time is rounded up to a value allowed by the real
 *          system clock.
 * @note    The maximum specified value is implementation dependent.
 *
 * @param[in] msecs     time in milliseconds, must be different from zero
 *
 * @api
 */
#define chThdSleepMilliseconds(msecs) chThdSleep(TIME_MS2I(msecs))

/**
 * @brief   Delays the invoking thread for the specified number of
 *          microseconds.
 * @note    The specified time is rounded up to a value allowed by the real
 *          system clock.
 * @note    The maximum specified value is implementation dependent.
 *
 * @param[in] usecs     time in microseconds, must be different from zero
 *
 * @api
 */
#define chThdSleepMicroseconds(usecs) chThdSleep(TIME_US2I(usecs))

/**
 * @brief   Suspends the invoking thread for the specified time.
 *
 * @param[in] timeout   the delay in system ticks
 *
 * @sclass
 */
#define chThdSleepS(timeout)                                                \
  (void) chSchGoSleepTimeoutS(NIL_STATE_SLEEPING, timeout)

/**
 * @brief   Suspends the invoking thread until the system time arrives to the
 *          specified value.
 *
 * @param[in] abstime   absolute system time
 *
 * @sclass
 */
#define chThdSleepUntilS(abstime)                                           \
  (void) chSchGoSleepTimeoutS(NIL_STATE_SLEEPING,                           \
                              chTimeDiffX(chVTGetSystemTimeX(), (abstime)))

/**
 * @brief   Initializes a threads queue object.
 *
 * @param[out] tqp      pointer to the threads queue object
 *
 * @init
 */
#define chThdQueueObjectInit(tqp) ((tqp)->cnt = (cnt_t)0)

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
#define chThdQueueIsEmptyI(tqp) ((bool)(tqp->cnt >= (cnt_t)0))

/**
 * @brief   Current system time.
 * @details Returns the number of system ticks since the @p chSysInit()
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
#if (CH_CFG_ST_TIMEDELTA == 0) || defined(__DOXYGEN__)
#define chVTGetSystemTimeX() (nil.systime)
#else
#define chVTGetSystemTimeX() port_timer_get_time()
#endif

/**
 * @brief   Returns the elapsed time since the specified start time.
 *
 * @param[in] start     start time
 * @return              The elapsed time.
 *
 * @xclass
 */
#define chVTTimeElapsedSinceX(start)                                        \
  chTimeDiffX((start), chVTGetSystemTimeX())

/**
 * @brief   Checks if the current system time is within the specified time
 *          window.
 * @note    When start==end then the function returns always false because the
 *          time window has zero size.
 *
 * @param[in] start     the start of the time window (inclusive)
 * @param[in] end       the end of the time window (non inclusive)
 * @retval true         current time within the specified time window.
 * @retval false        current time not within the specified time window.
 *
 * @xclass
 */
#define chVTIsSystemTimeWithinX(start, end)                                 \
  chTimeIsInRangeX(chVTGetSystemTimeX(), start, end)

/**
 * @brief   Adds an interval to a system time returning a system time.
 *
 * @param[in] systime   base system time
 * @param[in] interval  interval to be added
 * @return              The new system time.
 *
 * @xclass
 */
#define chTimeAddX(systime, interval)                                       \
  ((systime_t)(systime) + (systime_t)(interval))

/**
 * @brief   Subtracts two system times returning an interval.
 *
 * @param[in] start     first system time
 * @param[in] end       second system time
 * @return              The interval representing the time difference.
 *
 * @xclass
 */
#define chTimeDiffX(start, end)                                             \
  ((sysinterval_t)((systime_t)((systime_t)(end) - (systime_t)(start))))

/**
 * @brief   Function parameters check.
 * @details If the condition check fails then the kernel panics and halts.
 * @note    The condition is tested only if the @p CH_DBG_ENABLE_CHECKS switch
 *          is specified in @p chconf.h else the macro does nothing.
 *
 * @param[in] c         the condition to be verified to be true
 *
 * @api
 */
#if !defined(chDbgCheck)
#define chDbgCheck(c) do {                                                  \
  /*lint -save -e506 -e774 [2.1, 14.3] Can be a constant by design.*/       \
  if (CH_DBG_ENABLE_CHECKS != FALSE) {                                      \
    if (!(c)) {                                                             \
  /*lint -restore*/                                                         \
      chSysHalt(__func__);                                                  \
    }                                                                       \
  }                                                                         \
} while (false)
#endif /* !defined(chDbgCheck) */

/**
 * @brief   Condition assertion.
 * @details If the condition check fails then the kernel panics with a
 *          message and halts.
 * @note    The condition is tested only if the @p CH_DBG_ENABLE_ASSERTS
 *          switch is specified in @p chconf.h else the macro does nothing.
 * @note    The remark string is not currently used except for putting a
 *          comment in the code about the assertion.
 *
 * @param[in] c         the condition to be verified to be true
 * @param[in] r         a remark string
 *
 * @api
 */
#if !defined(chDbgAssert)
#define chDbgAssert(c, r) do {                                              \
  /*lint -save -e506 -e774 [2.1, 14.3] Can be a constant by design.*/       \
  if (CH_DBG_ENABLE_ASSERTS != FALSE) {                                     \
    if (!(c)) {                                                             \
  /*lint -restore*/                                                         \
      chSysHalt(__func__);                                                  \
    }                                                                       \
  }                                                                         \
} while (false)
#endif /* !defined(chDbgAssert) */
/** @} */

/* Empty macros if the state checker is not enabled.*/
#if CH_DBG_SYSTEM_STATE_CHECK == FALSE
#define __dbg_enter_lock()
#define __dbg_leave_lock()
#define __dbg_check_disable()
#define __dbg_check_suspend()
#define __dbg_check_enable()
#define __dbg_check_lock()
#define __dbg_check_unlock()
#define __dbg_check_lock_from_isr()
#define __dbg_check_unlock_from_isr()
#define __dbg_check_enter_isr()
#define __dbg_check_leave_isr()
#define chDbgCheckClassI()
#define chDbgCheckClassS()
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || defined(__DOXYGEN__)
extern stkalign_t __main_thread_stack_base__, __main_thread_stack_end__;
#endif
extern os_instance_t nil;
extern const thread_descriptor_t nil_thd_configs[];
#endif

#ifdef __cplusplus
extern "C" {
#endif
  thread_t *nil_find_thread(tstate_t state, void *p);
  cnt_t nil_ready_all(void *p, cnt_t cnt, msg_t msg);
  void chSysInit(void);
  void chSysHalt(const char *reason);
  void chSysTimerHandlerI(void);
  void chSysUnconditionalLock(void);
  void chSysUnconditionalUnlock(void);
  syssts_t chSysGetStatusAndLockX(void);
  bool chSysIsCounterWithinX(rtcnt_t cnt, rtcnt_t start, rtcnt_t end);
  void chSysPolledDelayX(rtcnt_t cycles);
  void chSysRestoreStatusX(syssts_t sts);
  thread_t *chSchReadyI(thread_t *tp, msg_t msg);
  bool chSchIsPreemptionRequired(void);
  void chSchDoPreemption(void);
  void chSchRescheduleS(void);
  msg_t chSchGoSleepTimeoutS(tstate_t newstate, sysinterval_t timeout);
  bool chTimeIsInRangeX(systime_t time, systime_t start, systime_t end);
  thread_t *chThdCreateI(const thread_descriptor_t *tdp);
  thread_t *chThdCreate(const thread_descriptor_t *tdp);
  void chThdExit(msg_t msg);
#if CH_CFG_USE_WAITEXIT == TRUE
  msg_t chThdWait(thread_t *tp);
#endif
  msg_t chThdSuspendTimeoutS(thread_reference_t *trp, sysinterval_t timeout);
  void chThdResumeI(thread_reference_t *trp, msg_t msg);
  void chThdResume(thread_reference_t *trp, msg_t msg);
  void chThdSleep(sysinterval_t timeout);
  void chThdSleepUntil(systime_t abstime);
  msg_t chThdEnqueueTimeoutS(threads_queue_t *tqp, sysinterval_t timeout);
  void chThdDoDequeueNextI(threads_queue_t *tqp, msg_t msg);
  void chThdDequeueNextI(threads_queue_t *tqp, msg_t msg);
  void chThdDequeueAllI(threads_queue_t *tqp, msg_t msg);
#if CH_DBG_SYSTEM_STATE_CHECK == TRUE
  void __dbg_check_disable(void);
  void __dbg_check_suspend(void);
  void __dbg_check_enable(void);
  void __dbg_check_lock(void);
  void __dbg_check_unlock(void);
  void __dbg_check_lock_from_isr(void);
  void __dbg_check_unlock_from_isr(void);
  void __dbg_check_enter_isr(void);
  void __dbg_check_leave_isr(void);
  void chDbgCheckClassI(void);
  void chDbgCheckClassS(void);
#endif
#ifdef __cplusplus
}
#endif

/* Optional modules.*/
#include "chsem.h"
#include "chevt.h"
#include "chmsg.h"
#include "chlib.h"

#endif /* CH_H */

/** @} */
