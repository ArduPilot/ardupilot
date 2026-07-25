/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    ADUCM41x/hal_st_lld.h
 * @brief   ST Driver subsystem low level driver header.
 * @details This header is designed to be include-able without having to
 *          include other files from the HAL.
 *
 * @addtogroup ST
 * @{
 */

#ifndef HAL_ST_LLD_H
#define HAL_ST_LLD_H

#include "mcuconf.h"
#include "aducm_wut.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   SysTick timer IRQ priority.
 */
#if !defined(ADUCM_ST_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define ADUCM_ST_IRQ_PRIORITY               3
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING)
#define ADUCM_ST_TIM                        ADUCM_WUT
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void st_lld_init(void);
  void st_lld_serve_interrupt(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Driver inline functions.                                                  */
/*===========================================================================*/

#if (OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING) || defined(__DOXYGEN__)
/**
 * @brief   Returns the time counter value.
 *
 * @return              The counter value.
 *
 * @notapi
 */
static inline systime_t st_lld_get_counter(void) {

  ADUCM_ST_TIM->CON |= ADUCM_WUT_CON_FREEZE;
  systime_t cnt = (systime_t)((ADUCM_ST_TIM->VAL0 & 0X0000FFFF) |
                              ((ADUCM_ST_TIM->VAL1 & 0X0000FFFF) << 16));
  ADUCM_ST_TIM->CON &= ~ADUCM_WUT_CON_FREEZE;
  return cnt;
}

/**
 * @brief   Starts the alarm.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 *
 * @param[in] abstime   the time to be set for the first alarm
 *
 * @notapi
 */
static inline void st_lld_start_alarm(systime_t abstime) {
  ADUCM_ST_TIM->WUFB0 = (abstime & 0X0000FFFF);
  ADUCM_ST_TIM->WUFB1 = ((abstime >> 16) & 0X0000FFFF);
  ADUCM_ST_TIM->CLRI = ADUCM_WUT_CLRI_WUFA | ADUCM_WUT_CLRI_WUFB |
                       ADUCM_WUT_CLRI_WUFC | ADUCM_WUT_CLRI_WUFD |
                       ADUCM_WUT_CLRI_ROLL;
  ADUCM_ST_TIM->IEN = ADUCM_WUT_IER_WUFB;
}

/**
 * @brief   Stops the alarm interrupt.
 *
 * @notapi
 */
static inline void st_lld_stop_alarm(void) {
  ADUCM_ST_TIM->IEN = 0;
}

/**
 * @brief   Sets the alarm time.
 *
 * @param[in] abstime   the time to be set for the next alarm
 *
 * @notapi
 */
static inline void st_lld_set_alarm(systime_t abstime) {
  ADUCM_ST_TIM->WUFB0 = (abstime & 0X0000FFFF);
  ADUCM_ST_TIM->WUFB1 = ((abstime >> 16) & 0X0000FFFF);
}

/**
 * @brief   Returns the current alarm time.
 *
 * @return              The currently set alarm time.
 *
 * @notapi
 */
static inline systime_t st_lld_get_alarm(void) {

  return (systime_t)((ADUCM_ST_TIM->WUFB0 & 0X0000FFFF) |
                     ((ADUCM_ST_TIM->WUFB1 & 0X0000FFFF) << 16));
}

/**
 * @brief   Determines if the alarm is active.
 *
 * @return              The alarm status.
 * @retval false        if the alarm is not active.
 * @retval true         is the alarm is active
 *
 * @notapi
 */
static inline bool st_lld_is_alarm_active(void) {

  return (bool)((ADUCM_ST_TIM->IEN & ADUCM_WUT_IER_WUFB) != 0);
}

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#endif /* HAL_ST_LLD_H */

/** @} */
