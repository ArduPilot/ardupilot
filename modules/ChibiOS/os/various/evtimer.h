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
 * @file    evtimer.h
 * @brief   Events Generator Timer structures and macros.
 *
 * @addtogroup event_timer
 * @{
 */

#ifndef EVTIMER_H
#define EVTIMER_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Module dependencies check.
 */
#if !CH_CFG_USE_EVENTS
#error "Event Timers require CH_CFG_USE_EVENTS"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a event timer structure.
 */
typedef struct {
  virtual_timer_t       et_vt;
  event_source_t        et_es;
  systime_t             et_interval;
} event_timer_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void evtObjectInit(event_timer_t *etp, systime_t time);
  void evtStart(event_timer_t *etp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Stops the timer.
 * @details If the timer was already stopped then the function has no effect.
 *
 * @param[in] etp       pointer to an initialized @p event_timer_t structure.
 */
static inline void evtStop(event_timer_t *etp) {

  chVTReset(&etp->et_vt);
}

#endif /* EVTIMER_H */

/** @} */
