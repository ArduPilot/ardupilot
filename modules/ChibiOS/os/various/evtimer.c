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
 * @file    evtimer.c
 * @brief   Events Generator Timer code.
 *
 * @addtogroup event_timer
 * @{
 */

#include "ch.h"
#include "evtimer.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

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

static void tmrcb(virtual_timer_t *vtp, void *p) {
  event_timer_t *etp = p;

  (void)vtp;

  chSysLockFromISR();
  chEvtBroadcastI(&etp->et_es);
  chVTDoSetI(&etp->et_vt, etp->et_interval, tmrcb, etp);
  chSysUnlockFromISR();
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief Initializes an @p event_timer_t structure.
 *
 * @param[out] etp      the @p event_timer_t structure to be initialized
 * @param[in] time      the interval in system ticks
 */
void evtObjectInit(event_timer_t *etp, systime_t time) {

  chEvtObjectInit(&etp->et_es);
  chVTObjectInit(&etp->et_vt);
  etp->et_interval = time;
}

/**
 * @brief   Starts the timer
 * @details If the timer was already running then the function has no effect.
 *
 * @param[in] etp       pointer to an initialized @p event_timer_t structure.
 */
void evtStart(event_timer_t *etp) {

  chVTSet(&etp->et_vt, etp->et_interval, tmrcb, etp);
}

/** @} */
