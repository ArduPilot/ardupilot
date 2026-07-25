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
 * @file    rt/include/chdebug.h
 * @brief   Debug support macros and structures.
 *
 * @addtogroup checks_assertions
 * @{
 */

#ifndef CHDEBUG_H
#define CHDEBUG_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Debug related settings
 * @{
 */
/**
 * @brief   Fill value for thread stack area in debug mode.
 */
#if !defined(CH_DBG_STACK_FILL_VALUE) || defined(__DOXYGEN__)
#define CH_DBG_STACK_FILL_VALUE             0x55
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   System debug data structure.
 */
typedef struct ch_system_debug {
  /**
   * @brief   Pointer to the panic message.
   * @details This pointer is meant to be accessed through the debugger, it is
   *          written once and then the system is halted.
   * @note    Accesses to this pointer must never be optimized out so the
   *          field itself is declared volatile.
   */
  const char            * volatile panic_msg;
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
} system_debug_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/* When the state checker feature is disabled then the following functions
   are replaced by an empty macro.*/
#if CH_DBG_SYSTEM_STATE_CHECK == FALSE
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

/**
 * @name    Macro Functions
 * @{
 */
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
    if (unlikely(!(c))) {                                                   \
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
 * @note    The condition is tested only if the @p CH_DBG_ENABLE_ASSERTS switch
 *          is specified in @p chconf.h else the macro does nothing.
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
    if (unlikely(!(c))) {                                                   \
  /*lint -restore*/                                                         \
      chSysHalt(__func__);                                                  \
    }                                                                       \
  }                                                                         \
} while (false)
#endif /* !defined(chDbgAssert) */
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
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

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Debug support initialization.
 * @note    Internal use only.
 *
 * @param[out] sdp      pointer to the @p system_debug_t structure
 *
 * @notapi
 */
static inline void __dbg_object_init(system_debug_t *sdp) {

  sdp->panic_msg = NULL;

#if CH_DBG_SYSTEM_STATE_CHECK == TRUE
  /* The initial state is assumed to be within a critical zone.*/
  sdp->isr_cnt  = (cnt_t)0;
  sdp->lock_cnt = (cnt_t)1;
#endif
}

#endif /* CHDEBUG_H */

/** @} */
