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
 * @file    hal_rtc_lld.h
 * @brief   RP2040 RTC subsystem low level driver header.
 *
 * @addtogroup RTC
 * @{
 */

#ifndef HAL_RTC_LLD_H
#define HAL_RTC_LLD_H

#if (HAL_USE_RTC == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Implementation capabilities
 * @{
 */
/**
 * @brief   Callback support in the driver.
 */
#define RTC_SUPPORTS_CALLBACKS      TRUE

/**
 * @brief   Number of alarms available.
 */
#define RTC_ALARMS                  1

/**
 * @brief   Presence of a local persistent storage.
 */
#define RTC_HAS_STORAGE             FALSE
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    PLATFORM configuration options
 * @{
 */
/* Priority settings checks.*/
#if !defined(RP_IRQ_RTC_PRIORITY)
#error "RP_IRQ_RTC_PRIORITY not defined in mcuconf.h"
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/**
 * @name    Date/time alarm setting values
 * @{
 */
#define RTC_DT_ALARM_SECOND   0U
#define RTC_DT_ALARM_MINUTE   1U
#define RTC_DT_ALARM_HOUR     2U
#define RTC_DT_ALARM_DAY      3U
#define RTC_DT_ALARM_MONTH    4U
#define RTC_DT_ALARM_YEAR     5U
#define RTC_DT_ALARM_DOTW     6U
/** @} */

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of an RTC event.
 */
typedef enum {
  RTC_EVENT_ALARM     = 0,            /** Alarm.                        */
 } rtcevent_t;

/**
 * @brief   Type of a generic RTC callback.
 */
typedef void (*rtccb_t)(RTCDriver *rtcp, rtcevent_t event);

/**
 * @brief   Type of a date/time mask.
 */
typedef uint8_t rtcdtmask_t;

/**
 * @brief   Type of a structure representing an RTC alarm time stamp.
 */
typedef struct {
  /* End of the mandatory fields.*/
  RTCDateTime       alarm;
  rtcdtmask_t       mask;
} RTCAlarm;

/**
 * @brief   Implementation-specific @p RTCDriver fields.
 */
#define rtc_lld_driver_fields                                               \
  /* Pointer to the RTC registers block.*/                                  \
  RTC_TypeDef       *rtc;                                                   \
  /* Callback pointer.*/                                                    \
  rtccb_t           callback;                                               \
  RTCDateTime       alarm;                                                  \
  rtcdtmask_t       mask;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#define RTC_ALARM_ENABLE_MATCH(n)         (1U << n)
#define RTC_ALARM_TEST_MATCH(a, n)        ((a & (1U << n)) != 0)
#define RTC_ALARM_DISABLE_ALL_MATCHING    0U
#define RTC_ALARM_ENABLE_ALL_MATCHING     0x7FU
/* DOTW is not taken into consideration in a non-repeating alarm. */
#define RTC_ALARM_NON_REPEATING           (RTC_ALARM_ENABLE_ALL_MATCHING     \
              & ~(RTC_ALARM_ENABLE_MATCH(RTC_DT_ALARM_DOTW)))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern RTCDriver RTCD1;

#ifdef __cplusplus
extern "C" {
#endif
  void rtc_lld_init(void);
  void rtc_lld_set_time(RTCDriver *rtcp, const RTCDateTime *timespec);
  void rtc_lld_get_time(RTCDriver *rtcp, RTCDateTime *timespec);
#if RTC_ALARMS > 0
  void rtc_lld_set_alarm(RTCDriver *rtcp,
                         rtcalarm_t alarm,
                         const RTCAlarm *alarmspec);
  void rtc_lld_get_alarm(RTCDriver *rtcp,
                         rtcalarm_t alarm,
                         RTCAlarm *alarmspec);
#endif
#if RTC_SUPPORTS_CALLBACKS == TRUE
  void rtc_lld_set_callback(RTCDriver *rtcp, rtccb_t callback);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_RTC == TRUE */

#endif /* HAL_RTC_LLD_H */

/** @} */
