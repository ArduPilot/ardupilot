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
 * @file    chearly.h
 * @brief   Early forward types declarations header.
 *
 * @addtogroup os_structures
 * @{
 */

#ifndef CHEARLY_H
#define CHEARLY_H

/* Port architecture-related definitions.*/
#if !defined(PORT_NEW_TYPES)
#include "chtypes.h"
#else
#include "chporttypes.h"
#endif

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if defined(PORT_DOES_NOT_PROVIDE_TYPES)
#if !defined(PORT_ARCH_SIZEOF_DATA_PTR)
#error "PORT_ARCH_SIZEOF_DATA_PTR not defined in chtypes.h"
#endif

#if !defined(PORT_ARCH_SIZEOF_CODE_PTR)
#error "PORT_ARCH_SIZEOF_CODE_PTR not defined in chtypes.h"
#endif

#if !defined(PORT_ARCH_REGISTERS_WIDTH)
#error "PORT_ARCH_REGISTERS_WIDTH not defined in chtypes.h"
#endif

#if !defined(PORT_ARCH_REVERSE_ORDER)
#error "PORT_ARCH_REVERSE_ORDER not defined in chtypes.h"
#endif
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
typedef port_rttime_t   rttime_t;           /**< Realtime accumulator.      */
typedef port_syssts_t   syssts_t;           /**< System status word.        */
typedef port_stkalign_t stkalign_t;         /**< Stack alignment type.      */

#if (PORT_ARCH_REGISTERS_WIDTH == 32) || defined(__DOXYGEN__)
typedef uint8_t         tmode_t;            /**< Thread flags.              */
typedef uint8_t         tstate_t;           /**< Thread state.              */
typedef uint8_t         trefs_t;            /**< Thread references counter. */
typedef uint8_t         tslices_t;          /**< Thread time slices counter.*/
typedef uint32_t        tprio_t;            /**< Thread priority.           */
typedef int32_t         msg_t;              /**< Inter-thread message.      */
typedef int32_t         eventid_t;          /**< Numeric event identifier.  */
typedef uint32_t        eventmask_t;        /**< Mask of event identifiers. */
typedef uint32_t        eventflags_t;       /**< Mask of event flags.       */
typedef int32_t         cnt_t;              /**< Generic signed counter.    */
typedef uint32_t        ucnt_t;             /**< Generic unsigned counter.  */
#elif PORT_ARCH_REGISTERS_WIDTH == 16
typedef uint8_t         tmode_t;            /**< Thread flags.              */
typedef uint8_t         tstate_t;           /**< Thread state.              */
typedef uint8_t         trefs_t;            /**< Thread references counter. */
typedef uint8_t         tslices_t;          /**< Thread time slices counter.*/
typedef uint16_t        tprio_t;            /**< Thread priority.           */
typedef int16_t         msg_t;              /**< Inter-thread message.      */
typedef int16_t         eventid_t;          /**< Numeric event identifier.  */
typedef uint16_t        eventmask_t;        /**< Mask of event identifiers. */
typedef uint16_t        eventflags_t;       /**< Mask of event flags.       */
typedef int16_t         cnt_t;              /**< Generic signed counter.    */
typedef uint16_t        ucnt_t;             /**< Generic unsigned counter.  */
#elif PORT_ARCH_REGISTERS_WIDTH == 8
typedef uint8_t         tmode_t;            /**< Thread flags.              */
typedef uint8_t         tstate_t;           /**< Thread state.              */
typedef uint8_t         trefs_t;            /**< Thread references counter. */
typedef uint8_t         tslices_t;          /**< Thread time slices counter.*/
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

/**
 * @brief   Type of a core identifier.
 * @note    Core identifiers have ranges from 0 to @p PORT_CORES_NUMBER - 1.
 */
typedef unsigned core_id_t;

/**
 * @brief   Type of a thread structure.
 */
typedef struct ch_thread thread_t;

/**
 * @brief   Type of an OS instance structure.
 */
typedef struct ch_os_instance os_instance_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Utility to make the parameter a quoted string.
 *
 * @param[in] a        literal to be string-ified
 */
#define __CH_STRINGIFY(a)   #a

/**
 * @brief   Structure field offset utility.
 *
 * @param[in] st        structured type name
 * @param[in] m         field name in the structured type
 * @return              The offset of the field in the structured type.
 */
#define __CH_OFFSETOF(st, m)                                                \
  /*lint -save -e9005 -e9033 -e413 [11.8, 10.8, 1.3] Normal pointers
    arithmetic, it is safe.*/                                               \
  ((size_t)((char *)&((st *)0)->m - (char *)0))                             \
  /*lint -restore*/

/**
 * @brief   Marks an expression result as used.
 *
 * @param[in] x         a valid expression
 */
#define __CH_USED(x)    (void)(x)

/**
 * @brief   Marks a boolean expression as likely true.
 * @note    No namespace prefix for this macro because it is commonly defined
 *          by operating systems.
 *
 * @param[in] x         a valid expression
 */
#if defined(PORT_LIKELY) || defined(__DOXYGEN__)
#define likely(x)       PORT_LIKELY(x)
#else
#define likely(x)       x
#endif

/**
 * @brief   Marks a boolean expression as likely false.
 * @note    No namespace prefix for this macro because it is commonly defined
 *          by operating systems.
 *
 * @param[in] x         a valid expression
 */
#if defined(PORT_UNLIKELY) || defined(__DOXYGEN__)
#define unlikely(x)     PORT_UNLIKELY(x)
#else
#define unlikely(x)     x
#endif

/**
 * @brief   Safe cast of a queue pointer to a thread pointer.
 * @note    Casting to a thread pointer should always be performed using
 *          this macro. Casting to threads pointer is allowed by design
 *          and this is the single check point for this operation.
 *
 * @param[in] p         pointer to a queue/list structure
 * @return              The pointer to the thread containing the queue/list
 *                      element.
 */
#define threadref(p)    ((thread_t *)(void *)(p))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* Early function prototypes required by the following headers.*/
#ifdef __cplusplus
extern "C" {
#endif
  void chSysHalt(const char *reason);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* CHEARLY_H */

/** @} */
