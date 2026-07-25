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
 * @file    RTCv2/hal_rtc_lld.h
 * @brief   STM32 RTC low level driver header.
 *
 * @addtogroup RTC
 * @{
 */

#ifndef HAL_RTC_LLD_H
#define HAL_RTC_LLD_H

#if HAL_USE_RTC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Implementation capabilities
 * @{
 */
/**
 * @brief   Callback support int the driver.
 */
#define RTC_SUPPORTS_CALLBACKS      TRUE

/**
 * @brief   Number of alarms available.
 */
#define RTC_ALARMS                  STM32_RTC_NUM_ALARMS

/**
 * @brief   Presence of a local persistent storage.
 */
#define RTC_HAS_STORAGE             (STM32_RTC_STORAGE_SIZE > 0)
/** @} */

/**
 * @brief   RTC PRER register initializer.
 */
#define RTC_PRER(a, s)              ((((a) - 1) << 16) | ((s) - 1))

/**
 * @name    Alarm helper macros
 * @{
 */
#define RTC_ALRM_MSK4               (1U << 31)
#define RTC_ALRM_WDSEL              (1U << 30)
#define RTC_ALRM_DT(n)              ((n) << 28)
#define RTC_ALRM_DU(n)              ((n) << 24)
#define RTC_ALRM_MSK3               (1U << 23)
#define RTC_ALRM_HT(n)              ((n) << 20)
#define RTC_ALRM_HU(n)              ((n) << 16)
#define RTC_ALRM_MSK2               (1U << 15)
#define RTC_ALRM_MNT(n)             ((n) << 12)
#define RTC_ALRM_MNU(n)             ((n) << 8)
#define RTC_ALRM_MSK1               (1U << 7)
#define RTC_ALRM_ST(n)              ((n) << 4)
#define RTC_ALRM_SU(n)              ((n) << 0)
/** @} */

/* Requires services from the EXTI driver.*/
#if !defined(STM32_EXTI_REQUIRED)
#define STM32_EXTI_REQUIRED
#endif

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   RTC PRES register initialization.
 * @note    The default is calculated for a 32768Hz clock.
 */
#if !defined(STM32_RTC_PRESA_VALUE) || defined(__DOXYGEN__)
#define STM32_RTC_PRESA_VALUE               32
#endif

/**
 * @brief   RTC PRESS divider initialization.
 * @note    The default is calculated for a 32768Hz clock.
 */
#if !defined(STM32_RTC_PRESS_VALUE) || defined(__DOXYGEN__)
#define STM32_RTC_PRESS_VALUE               1024
#endif

/**
 * @brief   RTC CR register initialization value.
 * @note    Use this value to initialize features not directly handled by
 *          the RTC driver.
 */
#if !defined(STM32_RTC_CR_INIT) || defined(__DOXYGEN__)
#define STM32_RTC_CR_INIT                   0
#endif

/**
 * @brief   RTC TAMPCR register initialization value.
 * @note    Use this value to initialize features not directly handled by
 *          the RTC driver.
 * @note    On some devices this values goes in the similar TAFCR register.
 */
#if !defined(STM32_RTC_TAMPCR_INIT) || defined(__DOXYGEN__)
#define STM32_RTC_TAMPCR_INIT               0
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if HAL_USE_RTC && !STM32_HAS_RTC
#error "RTC not present in the selected device"
#endif

#if defined(STM32_RTC_CK) && !defined(STM32_RTCCLK)
#define STM32_RTCCLK                        STM32_RTC_CK
#endif

#if !defined(STM32_RTCCLK)
#error "RTC clock not exported by HAL layer"
#endif

#if STM32_PCLK1 < (STM32_RTCCLK * 7)
#error "STM32_PCLK1 frequency is too low"
#endif

/**
 * @brief   Initialization for the RTC_PRER register.
 */
#define STM32_RTC_PRER_BITS                 RTC_PRER(STM32_RTC_PRESA_VALUE, \
                                                     STM32_RTC_PRESS_VALUE)

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of an RTC event.
 */
typedef enum {
  RTC_EVENT_ALARM_A     = 0,            /** Alarm A.                        */
  RTC_EVENT_ALARM_B     = 1,            /** Alarm B.                        */
  RTC_EVENT_TS          = 2,            /** Time stamp.                     */
  RTC_EVENT_TS_OVF      = 3,            /** Time stamp overflow.            */
  RTC_EVENT_TAMP1       = 4,            /** Tamper 1.                       */
  RTC_EVENT_TAMP2       = 5,            /** Tamper 2-                       */
  RTC_EVENT_TAMP3       = 6,            /** Tamper 3.                       */
  RTC_EVENT_WAKEUP      = 7             /** Wakeup.                         */
 } rtcevent_t;

/**
 * @brief   Type of a generic RTC callback.
 */
typedef void (*rtccb_t)(RTCDriver *rtcp, rtcevent_t event);

/**
 * @brief   Type of a structure representing an RTC alarm time stamp.
 */
typedef struct hal_rtc_alarm {
  /**
   * @brief   Type of an alarm as encoded in RTC ALRMxR registers.
   */
  uint32_t                  alrmr;
} RTCAlarm;

#if STM32_RTC_HAS_PERIODIC_WAKEUPS
/**
 * @brief   Type of a wakeup as encoded in RTC WUTR register.
 */
typedef struct hal_rtc_wakeup {
  /**
   * @brief   Wakeup as encoded in RTC WUTR register.
   * @note    ((WUTR == 0) || (WUCKSEL == 3)) are a forbidden combination.
   * @note    Bits 16..18 are copied in the CR bits 0..2 (WUCKSEL).
   */
  uint32_t                  wutr;
} RTCWakeup;
#endif

/**
 * @brief   Implementation-specific @p RTCDriver fields.
 */
#define rtc_lld_driver_fields                                               \
  /* Pointer to the RTC registers block.*/                                  \
  RTC_TypeDef               *rtc;                                           \
  /* Callback pointer.*/                                                    \
  rtccb_t           callback;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void rtc_lld_init(void);
  void rtc_lld_set_time(RTCDriver *rtcp, const RTCDateTime *timespec);
  void rtc_lld_get_time(RTCDriver *rtcp, RTCDateTime *timespec);
#if RTC_SUPPORTS_CALLBACKS == TRUE
  void rtc_lld_set_callback(RTCDriver *rtcp, rtccb_t callback);
#endif
#if RTC_ALARMS > 0
  void rtc_lld_set_alarm(RTCDriver *rtcp,
                         rtcalarm_t alarm,
                         const RTCAlarm *alarmspec);
  void rtc_lld_get_alarm(RTCDriver *rtcp,
                         rtcalarm_t alarm,
                         RTCAlarm *alarmspec);
#endif
#if STM32_RTC_HAS_PERIODIC_WAKEUPS
  void rtcSTM32SetPeriodicWakeup(RTCDriver *rtcp, const RTCWakeup *wakeupspec);
  void rtcSTM32GetPeriodicWakeup(RTCDriver *rtcp, RTCWakeup *wakeupspec);
#endif /* STM32_RTC_HAS_PERIODIC_WAKEUPS */
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_RTC */

#endif /* HAL_RTC_LLD_H */

/** @} */
