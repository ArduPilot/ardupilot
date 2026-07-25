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
 * @file    osal_vt.h
 * @brief   OSAL Virtual Timers module header.
 *
 * @addtogroup OSAL_VT
 * @{
 */

#ifndef _OSAL_VT_H_
#define _OSAL_VT_H_

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
 * @brief   Type of a Virtual Timer callback function.
 */
typedef void (*vtfunc_t)(void *);

/**
 * @brief   Type of a Virtual Timer structure.
 */
typedef struct virtual_timer virtual_timer_t;

/**
 * @brief   Virtual timers list header.
 * @note    The content of this structure is not part of the API and should
 *          not be relied upon. Implementers may define this structure in
 *          an entirely different way.
 * @note    The delta list is implemented as a double link bidirectional list
 *          in order to make the unlink time constant, the reset of a virtual
 *          timer is often used in the code.
 */
typedef struct {
  virtual_timer_t       *vt_next;   /**< @brief Next timer in the timers
                                                list.                       */
  virtual_timer_t       *vt_prev;   /**< @brief Last timer in the timers
                                                list.                       */
  sysinterval_t         vt_delta;   /**< @brief Must be initialized to -1.  */
  volatile systime_t    vt_systime; /**< @brief System Time counter.        */
} virtual_timers_list_t;

/**
 * @extends virtual_timers_list_t
 *
 * @brief   Virtual Timer descriptor structure.
 * @note    The content of this structure is not part of the API and should
 *          not be relied upon. Implementers may define this structure in
 *          an entirely different way.
 */
struct virtual_timer {
  virtual_timer_t       *vt_next;   /**< @brief Next timer in the timers
                                                list.                       */
  virtual_timer_t       *vt_prev;   /**< @brief Previous timer in the timers
                                                list.                       */
  sysinterval_t         vt_delta;   /**< @brief Time delta before timeout.  */
  vtfunc_t              vt_func;    /**< @brief Timer callback function
                                                pointer.                    */
  void                  *vt_par;    /**< @brief Timer callback function
                                                parameter.                  */
};

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern virtual_timers_list_t vtlist;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void vtInit(void);
  bool vtIsArmedI(virtual_timer_t *vtp);
  void vtDoTickI(void);
  void vtSetI(virtual_timer_t *vtp, sysinterval_t timeout,
              vtfunc_t vtfunc, void *par);
  void vtResetI(virtual_timer_t *vtp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* _OSAL_VT_H_ */

/** @} */
