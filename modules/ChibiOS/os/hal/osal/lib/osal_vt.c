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
 * @file    osal_vt.c
 * @brief   OSAL Virtual Timers module code.
 * @details This module can be used in an OSAL implementation whenever an
 *          underlying RTOS is unable to provide timeout services or there
 *          is no underlying RTOS.
 *
 * @addtogroup OSAL_VT
 * @{
 */

#include "osal.h"
#include "osal_vt.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   Virtual timers delta list header.
 */
virtual_timers_list_t vtlist;

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
 * @brief   Timers initialization.
 *
 * @init
 */
void vtInit(void) {

  /* Virtual Timers initialization.*/
  vtlist.vt_next = vtlist.vt_prev = (void *)&vtlist;
  vtlist.vt_delta = (sysinterval_t)-1;
  vtlist.vt_systime = 0;
}

/**
 * @brief   Returns @p TRUE if the specified timer is armed.
 *
 * @param[out] vtp      the @p virtual_timer_t structure pointer
 *
 * @iclass
 */
bool vtIsArmedI(virtual_timer_t *vtp) {

  return vtp->vt_func != NULL;
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
void vtDoTickI(void) {

  vtlist.vt_systime++;
  if (&vtlist != (virtual_timers_list_t *)vtlist.vt_next) {
    virtual_timer_t *vtp;

    --vtlist.vt_next->vt_delta;
    while (!(vtp = vtlist.vt_next)->vt_delta) {
      vtfunc_t fn = vtp->vt_func;
      vtp->vt_func = (vtfunc_t)NULL;
      vtp->vt_next->vt_prev = (void *)&vtlist;
      (&vtlist)->vt_next = vtp->vt_next;
      osalSysUnlockFromISR();
      fn(vtp->vt_par);
      osalSysLockFromISR();
    }
  }
}

/**
 * @brief   Enables a virtual timer.
 * @note    The associated function is invoked from interrupt context.
 *
 * @param[out] vtp      the @p virtual_timer_t structure pointer
 * @param[in] timeout   the number of ticks before the operation timeouts, the
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
void vtSetI(virtual_timer_t *vtp, sysinterval_t timeout,
            vtfunc_t vtfunc, void *par) {
  virtual_timer_t *p;

  vtp->vt_par = par;
  vtp->vt_func = vtfunc;
  p = vtlist.vt_next;
  while (p->vt_delta < timeout) {
    timeout -= p->vt_delta;
    p = p->vt_next;
  }

  vtp->vt_prev = (vtp->vt_next = p)->vt_prev;
  vtp->vt_prev->vt_next = p->vt_prev = vtp;
  vtp->vt_delta = timeout;
  if (p != (void *)&vtlist)
    p->vt_delta -= timeout;
}

/**
 * @brief   Disables a Virtual Timer.
 * @note    The timer MUST be active when this function is invoked.
 *
 * @param[in] vtp       the @p virtual_timer_t structure pointer
 *
 * @iclass
 */
void vtResetI(virtual_timer_t *vtp) {

  if (vtp->vt_next != (void *)&vtlist)
    vtp->vt_next->vt_delta += vtp->vt_delta;
  vtp->vt_prev->vt_next = vtp->vt_next;
  vtp->vt_next->vt_prev = vtp->vt_prev;
  vtp->vt_func = (vtfunc_t)NULL;
}

/** @} */
