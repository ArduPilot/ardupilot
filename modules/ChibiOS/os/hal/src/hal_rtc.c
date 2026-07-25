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
/*
   Concepts and parts of this file have been contributed by Uladzimir Pylinsky
   aka barthess.
 */

/**
 * @file    hal_rtc.c
 * @brief   RTC Driver code.
 *
 * @addtogroup RTC
 * @{
 */

#include "hal.h"

#if (HAL_USE_RTC == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*
 * Lookup table with months' length
 */
static const uint8_t month_len[12] = {
  31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

static const uint16_t accu_month_len[12] = {
  0, 31, 59,  90, 120, 151, 181, 212, 243, 273, 304, 334
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   RTC Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void rtcInit(void) {

  rtc_lld_init();
}

/**
 * @brief   Initializes a generic RTC driver object.
 * @details The HW dependent part of the initialization has to be performed
 *          outside, usually in the hardware initialization code.
 *
 * @param[out] rtcp     pointer to RTC driver structure
 *
 * @init
 */
void rtcObjectInit(RTCDriver *rtcp) {

#if RTC_HAS_STORAGE == TRUE
  rtcp->vmt = &_rtc_lld_vmt;
#else
  (void)rtcp;
#endif
}

/**
 * @brief   Set current time.
 * @note    This function can be called from any context but limitations
 *          could be imposed by the low level implementation. It is
 *          guaranteed that the function can be called from thread
 *          context.
 * @note    The function can be reentrant or not reentrant depending on
 *          the low level implementation.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCDateTime structure
 *
 * @special
 */
void rtcSetTime(RTCDriver *rtcp, const RTCDateTime *timespec) {

  osalDbgCheck((rtcp != NULL) && (timespec != NULL));

  rtc_lld_set_time(rtcp, timespec);
}

/**
 * @brief   Get current time.
 * @note    This function can be called from any context but limitations
 *          could be imposed by the low level implementation. It is
 *          guaranteed that the function can be called from thread
 *          context.
 * @note    The function can be reentrant or not reentrant depending on
 *          the low level implementation.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[out] timespec pointer to a @p RTCDateTime structure
 *
 * @special
 */
void rtcGetTime(RTCDriver *rtcp, RTCDateTime *timespec) {

  osalDbgCheck((rtcp != NULL) && (timespec != NULL));

  rtc_lld_get_time(rtcp, timespec);
}

#if (RTC_ALARMS > 0) || defined(__DOXYGEN__)
/**
 * @brief   Set alarm time.
 * @note    This function can be called from any context but limitations
 *          could be imposed by the low level implementation. It is
 *          guaranteed that the function can be called from thread
 *          context.
 * @note    The function can be reentrant or not reentrant depending on
 *          the low level implementation.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] alarm     alarm identifier
 * @param[in] alarmspec pointer to a @p RTCAlarm structure or @p NULL
 *
 * @special
 */
void rtcSetAlarm(RTCDriver *rtcp,
                 rtcalarm_t alarm,
                 const RTCAlarm *alarmspec) {

  osalDbgCheck((rtcp != NULL) && (alarm < (rtcalarm_t)RTC_ALARMS));

  rtc_lld_set_alarm(rtcp, alarm, alarmspec);
}

/**
 * @brief   Get current alarm.
 * @note    If an alarm has not been set then the returned alarm specification
 *          is not meaningful.
 * @note    This function can be called from any context but limitations
 *          could be imposed by the low level implementation. It is
 *          guaranteed that the function can be called from thread
 *          context.
 * @note    The function can be reentrant or not reentrant depending on
 *          the low level implementation.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] alarm     alarm identifier
 * @param[out] alarmspec pointer to a @p RTCAlarm structure
 *
 * @special
 */
void rtcGetAlarm(RTCDriver *rtcp,
                 rtcalarm_t alarm,
                 RTCAlarm *alarmspec) {

  osalDbgCheck((rtcp != NULL) &&
               (alarm < (rtcalarm_t)RTC_ALARMS) &&
               (alarmspec != NULL));

  rtc_lld_get_alarm(rtcp, alarm, alarmspec);
}
#endif /* RTC_ALARMS > 0 */

#if (RTC_SUPPORTS_CALLBACKS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Enables or disables RTC callbacks.
 * @details This function enables or disables the callback, use a @p NULL
 *          pointer in order to disable it.
 * @note    This function can be called from any context but limitations
 *          could be imposed by the low level implementation. It is
 *          guaranteed that the function can be called from thread
 *          context.
 * @note    The function can be reentrant or not reentrant depending on
 *          the low level implementation.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] callback  callback function pointer or @p NULL
 *
 * @special
 */
void rtcSetCallback(RTCDriver *rtcp, rtccb_t callback) {

  osalDbgCheck(rtcp != NULL);

  rtc_lld_set_callback(rtcp, callback);
}
#endif /* RTC_SUPPORTS_CALLBACKS == TRUE */

/**
 * @brief   Convert @p RTCDateTime to broken-down time structure.
 *
 * @param[in]  timespec   pointer to a @p RTCDateTime structure
 * @param[out] timp       pointer to a broken-down time structure
 * @param[out] tv_msec    pointer to milliseconds value or @p NULL
 *
 * @api
 */
void rtcConvertDateTimeToStructTm(const RTCDateTime *timespec,
                                  struct tm *timp,
                                  uint32_t *tv_msec) {
  int sec, year;
  bool is_leap_year;

  timp->tm_year  = (int)timespec->year + (int)(RTC_BASE_YEAR - 1900U);
  timp->tm_mon   = (int)timespec->month - 1;
  timp->tm_mday  = (int)timespec->day;
  timp->tm_isdst = (int)timespec->dstflag;
  timp->tm_wday  = (int)timespec->dayofweek % 7;

  sec = (int)timespec->millisecond / 1000;
  timp->tm_hour = sec / 3600;
  sec %= 3600;
  timp->tm_min = sec / 60;
  timp->tm_sec = sec % 60;

  if (NULL != tv_msec) {
    *tv_msec = (uint32_t)timespec->millisecond % 1000U;
  }

  /* Day of the year calculation.*/
  year = timp->tm_year + 1900;
  timp->tm_yday = timp->tm_mday - 1;
  timp->tm_yday += (int)accu_month_len[timp->tm_mon];
  is_leap_year = (((year % 4) == 0) && ((year % 100) != 0)) || ((year % 400) == 0);
  if (is_leap_year && (timp->tm_mon > 1)) {
    timp->tm_yday++;
  }
}

/**
 * @brief   Convert broken-down time structure to @p RTCDateTime.
 *
 * @param[in] timp        pointer to a broken-down time structure
 * @param[in] tv_msec     milliseconds value
 * @param[out] timespec   pointer to a @p RTCDateTime structure
 *
 * @api
 */
void rtcConvertStructTmToDateTime(const struct tm *timp,
                                  uint32_t tv_msec,
                                  RTCDateTime *timespec) {

  /*lint -save -e9034 [10.4] Verified assignments to bit fields.*/
  timespec->year      = (uint32_t)timp->tm_year - (RTC_BASE_YEAR - 1900U);
  timespec->month     = (uint32_t)timp->tm_mon + 1U;
  timespec->day       = (uint32_t)timp->tm_mday;
  timespec->dayofweek = (((uint32_t)timp->tm_wday + 6U) % 7U) + 1U;

  if (-1 == timp->tm_isdst) {
    timespec->dstflag = 0U;  /* Set zero if dst is unknown.*/
  }
  else {
    timespec->dstflag = (uint32_t)timp->tm_isdst;
  }
  /*lint -restore*/
  /*lint -save -e9033 [10.8] Verified assignments to bit fields.*/
  timespec->millisecond = tv_msec + (uint32_t)(((timp->tm_hour * 3600) +
                                                (timp->tm_min * 60) +
                                                 timp->tm_sec) * 1000);
  /*lint -restore*/
}

/**
 * @brief   Get current time in format suitable for usage in FAT file system.
 * @note    The information about day of week and DST is lost in DOS
 *          format, the second field loses its least significant bit.
 *
 * @param[out] timespec pointer to a @p RTCDateTime structure
 * @return              FAT date/time value.
 *
 * @api
 */
uint32_t rtcConvertDateTimeToFAT(const RTCDateTime *timespec) {
  uint32_t fattime;
  uint32_t sec, min, hour, day, month, year;

  sec   = timespec->millisecond / 1000U;
  hour  = sec / 3600U;
  sec  %= 3600U;
  min   = sec / 60U;
  sec  %= 60U;
  day   = timespec->day;
  month = timespec->month;
  year  = timespec->year;

  /* Handle DST flag.*/
  if (1U == timespec->dstflag) {
    hour += 1U;
    if (hour == 24U) {
      hour = 0U;
      day += 1U;
      if (day > (uint32_t)month_len[month - 1U]) {
        day = 1U;
        month += 1U;
        if (month > 12U) {
          month = 1U;
          year += 1U;
        }
      }
    }
  }

  fattime  = sec   >> 1U;
  fattime |= min   << 5U;
  fattime |= hour  << 11U;
  fattime |= day   << 16U;
  fattime |= month << 21U;
  fattime |= year << 25U;

  return fattime;
}

#endif /* HAL_USE_RTC == TRUE */

/** @} */
