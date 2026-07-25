/*
    ChibiOS - Copyright (C) 2017 Theodore Ateba

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
 * @file    TIMv1/hal_st_lld.h
 * @brief   AVR XMEGA ST Driver subsystem low level driver header file.
 * @details This header is designed to be include-able without having to
 *          include other files from the HAL.
 *
 * @addtogroup AVR
 * @{
 */

#ifndef HAL_ST_LLD_H
#define HAL_ST_LLD_H

#include <avr/io.h>

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/*
 * TODO: for models that have many timers,
 *       could add AVR_XMEGA_ST_USE_TIMER.
 */

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

/* TODO: error checks for valid timer selected.                             */

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void st_lld_init(void);
#ifdef __cplusplus
}
#endif

#define TIFR_REG  TCC0.INTFLAGS

/*==========================================================================*/
/* Driver inline functions.                                                 */
/*==========================================================================*/

/**
 * @brief   Returns the time counter value.
 *
 * @return              The counter value.
 *
 * @notapi
 */
static inline systime_t st_lld_get_counter(void) {

  return (systime_t) TCC0.CNT;
}

/**
 * @brief   Starts the alarm.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 *
 * @param[in] time      the time to be set for the first alarm
 *
 * @notapi
 */
static inline void st_lld_start_alarm(systime_t time) {

  TCC0.INTCTRLA  = (TCC0.INTCTRLA & ~TC0_OVFINTLVL_gm) | TC_OVFINTLVL_MED_gc;
  PMIC.CTRL |= PMIC_MEDLVLEN_bm;
}

/**
 * @brief   Stops the alarm interrupt.
 *
 * @notapi
 */
static inline void st_lld_stop_alarm(void) {

  /* TODO: Implement this function. */
}

/**
 * @brief   Sets the alarm time.
 *
 * @param[in] time      the time to be set for the next alarm
 *
 * @notapi
 */
static inline void st_lld_set_alarm(systime_t time) {

  TCC0.PER       = (uint16_t) time;;
}

/**
 * @brief   Returns the current alarm time.
 *
 * @return              The currently set alarm time.
 *
 * @notapi
 */
static inline systime_t st_lld_get_alarm(void) {

  return (systime_t)TCC0.PER;
}

/**
 * @brief   Determines if the alarm is active.
 *
 * @return              The alarm status.
 * @retval false        if the alarm is not active.
 * @retval true         is the alarm is active.
 *
 * @notapi
 */
static inline bool st_lld_is_alarm_active(void) {

  /* TODO: Implement this function. */
  return (bool) 0;
}

#endif /* HAL_ST_LLD_H */

/** @} */
