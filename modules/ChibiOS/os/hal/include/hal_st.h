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
 * @file    hal_st.h
 * @brief   ST Driver macros and structures.
 * @details This header is designed to be include-able without having to
 *          include other files from the HAL.
 *
 * @addtogroup ST
 * @{
 */

#ifndef HAL_ST_H
#define HAL_ST_H

#include "hal_st_lld.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Compatibility with old LLDS.*/
#if !defined(ST_LLD_NUM_ALARMS)
#define ST_LLD_NUM_ALARMS                   1
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef void (*st_callback_t)(unsigned alarm);

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (ST_LLD_NUM_ALARMS > 1) && !defined(__DOXYGEN__)
extern st_callback_t st_callbacks[ST_LLD_NUM_ALARMS];
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void stInit(void);
#if defined(ST_LLD_MULTICORE_SUPPORT)
  void stBind(void);
#endif
#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING
  systime_t stGetCounter(void);
  void stStartAlarm(systime_t abstime);
  void stStopAlarm(void);
  void stSetAlarm(systime_t abstime);
  systime_t stGetAlarm(void);
  bool stIsAlarmActive(void);
#if ST_LLD_NUM_ALARMS > 1
  void stSetCallback(unsigned alarm, st_callback_t cb);
#if defined(ST_LLD_MULTICORE_SUPPORT)
  void stBindAlarmN(unsigned alarm);
#endif
  void stStartAlarmN(unsigned alarm, systime_t abstime);
  void stStopAlarmN(unsigned alarm);
  void stSetAlarmN(unsigned alarm, systime_t abstime);
  systime_t stGetAlarmN(unsigned alarm);
  bool stIsAlarmActiveN(unsigned alarm);
#endif /* ST_LLD_NUM_ALARMS > 1 */
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */
#ifdef __cplusplus
}
#endif

#endif /* HAL_ST_H */

/** @} */
