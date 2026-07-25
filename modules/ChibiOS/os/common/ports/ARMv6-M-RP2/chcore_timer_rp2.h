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
 * @file    ARMv6-M-RP2/chcore_timer.h
 * @brief   System timer header file.
 *
 * @addtogroup ARMV6M_RP2_TIMER
 * @{
 */

#ifndef CHCORE_TIMER_RP2_H
#define CHCORE_TIMER_RP2_H

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

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

#if (CH_CFG_ST_TIMEDELTA > 0) || defined(__DOXYGEN__)
/**
 * @brief   Enables the system timer for the specified OS instance.
 */
#define port_timer_enable(oip) stBindAlarmN((unsigned)(oip)->core_id)

/**
 * @brief   Disables the system timer for the specified OS instance.
 */
#define port_timer_disable(oip)

/**
 * @brief   Starts the alarm.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 *
 * @param[in] time      the time to be set for the first alarm
 *
 * @notapi
 */
#define port_timer_start_alarm(time) stStartAlarmN((unsigned)currcore->core_id, time)

/**
 * @brief   Stops the alarm interrupt.
 *
 * @notapi
 */
#define port_timer_stop_alarm() stStopAlarmN((unsigned)currcore->core_id)

/**
 * @brief   Sets the alarm time.
 *
 * @param[in] time      the time to be set for the next alarm
 *
 * @notapi
 */
#define port_timer_set_alarm(time) stSetAlarmN((unsigned)currcore->core_id, time)

/**
 * @brief   Returns the current alarm time.
 *
 * @return              The currently set alarm time.
 *
 * @notapi
 */
#define port_timer_get_alarm() stGetAlarmN((unsigned)currcore->core_id)

/**
 * @brief   Returns the system time.
 *
 * @return              The system time.
 *
 * @notapi
 */
#define port_timer_get_time() stGetCounter()

#else
#define port_timer_enable(oip) stBind()
#define port_timer_disable(oip)
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void stBind(void);
#if CH_CFG_ST_TIMEDELTA > 0
  void stBindAlarmN(unsigned alarm);
  void stStartAlarmN(unsigned alarm, systime_t time);
  void stStopAlarmN(unsigned alarm);
  void stSetAlarmN(unsigned alarm, systime_t time);
  systime_t stGetAlarmN(unsigned alarm);
  systime_t stGetCounter(void);
#endif
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* CHCORE_TIMER_RP2_H */

/** @} */
