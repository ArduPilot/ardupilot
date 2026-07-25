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
 * @file    RTCv4/hal_rtc_lld.c
 * @brief   STM32 RTCv4 low level driver.
 *
 * @addtogroup RTC
 * @{
 */

#include "hal.h"

#if HAL_USE_RTC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define RTC_TR_PM_OFFSET                    RTC_TR_PM_Pos
#define RTC_TR_HT_OFFSET                    RTC_TR_HT_Pos
#define RTC_TR_HU_OFFSET                    RTC_TR_HU_Pos
#define RTC_TR_MNT_OFFSET                   RTC_TR_MNT_Pos
#define RTC_TR_MNU_OFFSET                   RTC_TR_MNU_Pos
#define RTC_TR_ST_OFFSET                    RTC_TR_ST_Pos
#define RTC_TR_SU_OFFSET                    RTC_TR_SU_Pos

#define RTC_DR_YT_OFFSET                    RTC_DR_YT_Pos
#define RTC_DR_YU_OFFSET                    RTC_DR_YU_Pos
#define RTC_DR_WDU_OFFSET                   RTC_DR_WDU_Pos
#define RTC_DR_MT_OFFSET                    RTC_DR_MT_Pos
#define RTC_DR_MU_OFFSET                    RTC_DR_MU_Pos
#define RTC_DR_DT_OFFSET                    RTC_DR_DT_Pos
#define RTC_DR_DU_OFFSET                    RTC_DR_DU_Pos

#define RTC_CR_BKP_OFFSET                   RTC_CR_BKP_Pos

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief RTC driver identifier.
 */
RTCDriver RTCD1;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Beginning of configuration procedure.
 *
 * @notapi
 */
static void rtc_enter_init(void) {

  RTCD1.rtc->ICSR |= RTC_ICSR_INIT;
  while ((RTCD1.rtc->ICSR & RTC_ICSR_INITF) == 0U)
    ;
}

/**
 * @brief   Finalizing of configuration procedure.
 *
 * @notapi
 */
static inline void rtc_exit_init(void) {

  RTCD1.rtc->ICSR &= ~RTC_ICSR_INIT;
}

/**
 * @brief   Converts time from TR register encoding to timespec.
 *
 * @param[in] tr        TR register value
 * @param[out] timespec pointer to a @p RTCDateTime structure
 *
 * @notapi
 */
static void rtc_decode_time(uint32_t tr, RTCDateTime *timespec) {
  uint32_t n;

  n  = ((tr >> RTC_TR_HT_OFFSET) & 3U)   * 36000000U;
  n += ((tr >> RTC_TR_HU_OFFSET) & 15U)  * 3600000U;
  n += ((tr >> RTC_TR_MNT_OFFSET) & 7U)  * 600000U;
  n += ((tr >> RTC_TR_MNU_OFFSET) & 15U) * 60000U;
  n += ((tr >> RTC_TR_ST_OFFSET) & 7U)   * 10000U;
  n += ((tr >> RTC_TR_SU_OFFSET) & 15U)  * 1000U;
  timespec->millisecond = n;
}

/**
 * @brief   Converts date from DR register encoding to timespec.
 *
 * @param[in] dr        DR register value
 * @param[out] timespec pointer to a @p RTCDateTime structure
 *
 * @notapi
 */
static void rtc_decode_date(uint32_t dr, RTCDateTime *timespec) {

  timespec->year  = (((dr >> RTC_DR_YT_OFFSET) & 15U) * 10U) +
                     ((dr >> RTC_DR_YU_OFFSET) & 15U);
  timespec->month = (((dr >> RTC_DR_MT_OFFSET) & 1U) * 10U) +
                     ((dr >> RTC_DR_MU_OFFSET) & 15U);
  timespec->day   = (((dr >> RTC_DR_DT_OFFSET) & 3U) * 10U) +
                     ((dr >> RTC_DR_DU_OFFSET) & 15U);
  timespec->dayofweek = (dr >> RTC_DR_WDU_OFFSET) & 7U;
}

/**
 * @brief   Converts time from timespec to TR register encoding.
 *
 * @param[in] timespec  pointer to a @p RTCDateTime structure
 * @return              the TR register encoding.
 *
 * @notapi
 */
static uint32_t rtc_encode_time(const RTCDateTime *timespec) {
  uint32_t n, tr = 0U;

  /* Subseconds cannot be set.*/
  n = timespec->millisecond / 1000U;

  /* Seconds conversion.*/
  tr |= ((n % 10U) << RTC_TR_SU_OFFSET);
  n /= 10U;
  tr |= ((n % 6U) << RTC_TR_ST_OFFSET);
  n /= 6U;

  /* Minutes conversion.*/
  tr |= ((n % 10U) << RTC_TR_MNU_OFFSET);
  n /= 10U;
  tr |= ((n % 6U) << RTC_TR_MNT_OFFSET);
  n /= 6U;

  /* Hours conversion.*/
  tr |= ((n % 10U) << RTC_TR_HU_OFFSET);
  n /= 10U;
  tr |= (n << RTC_TR_HT_OFFSET);

  return tr;
}

/**
 * @brief   Converts a date from timespec to DR register encoding.
 *
 * @param[in] timespec  pointer to a @p RTCDateTime structure
 * @return              the DR register encoding.
 *
 * @notapi
 */
static uint32_t rtc_encode_date(const RTCDateTime *timespec) {
  uint32_t n, dr = 0U;

  /* Year conversion. Note, only years last two digits are considered.*/
  n = timespec->year;
  dr |= ((n % 10U) << RTC_DR_YU_OFFSET);
  n /= 10U;
  dr |= ((n % 10U) << RTC_DR_YT_OFFSET);

  /* Months conversion.*/
  n = timespec->month;
  dr |= ((n % 10U) << RTC_DR_MU_OFFSET);
  n /= 10U;
  dr |= ((n % 10U) << RTC_DR_MT_OFFSET);

  /* Days conversion.*/
  n = timespec->day;
  dr |= ((n % 10U) << RTC_DR_DU_OFFSET);
  n /= 10U;
  dr |= ((n % 10U) << RTC_DR_DT_OFFSET);

  /* Days of week conversion.*/
  dr |= (timespec->dayofweek << RTC_DR_WDU_OFFSET);

  return dr;
}

/**
 * @brief   RTC ISR service routine.
 *
 * @notapi
 */
static void rtc_lld_serve_interrupt(void) {
  uint32_t isr;

  /* Get and clear the RTC interrupts.*/
  isr = RTCD1.rtc->MISR;
  RTCD1.rtc->SCR = isr;

  /* Clear EXTI events.*/
  STM32_RTC_CLEAR_ALL_EXTI();

  if (RTCD1.callback != NULL) {
    uint32_t cr = RTCD1.rtc->CR;

#if defined(RTC_MISR_ALRAMF)
    if (((cr & RTC_CR_ALRAIE) != 0U) && ((isr & RTC_MISR_ALRAMF) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_ALARM_A);
    }
#endif
#if defined(RTC_MISR_TSMF)
    if (((cr & RTC_CR_TSIE) != 0U) && ((isr & RTC_MISR_TSMF) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_TS);
    }
#endif
#if defined(RTC_MISR_TSOVMF)
    if (((cr & RTC_CR_TSIE) != 0U) && ((isr & RTC_MISR_TSOVMF) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_TS_OVF);
    }
#endif
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_RTC_COMMON_HANDLER)
#if !defined(STM32_RTC_SUPPRESS_COMMON_ISR)
/**
 * @brief   RTC common interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_RTC_COMMON_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  rtc_lld_serve_interrupt();

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_RTC_SUPPRESS_COMMON_ISR) */
#else
#error "missing required RTC common handler definition in registry"
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Enable access to registers.
 *
 * @notapi
 */
void rtc_lld_init(void) {

  /* RTC object initialization.*/
  rtcObjectInit(&RTCD1);

  /* RTC pointer initialization.*/
  RTCD1.rtc = RTC;

  /* Disable write protection.*/
  RTCD1.rtc->WPR = 0xCAU;
  RTCD1.rtc->WPR = 0x53U;

  /* If calendar has not been initialized yet then proceed with the
     initial setup.*/
  if ((RTCD1.rtc->ICSR & RTC_ICSR_INITS) == 0U) {

    rtc_enter_init();

    RTCD1.rtc->CR = (STM32_RTC_CR_INIT & STM32_RTC_CR_MASK) | RTC_CR_BYPSHAD;
    /* Setting PRER has to be done as two writes. Write Sync part first
       then Sync + Async.*/
    RTCD1.rtc->PRER = STM32_RTC_PRER_BITS & 0x7FFFU;
    RTCD1.rtc->PRER = STM32_RTC_PRER_BITS;

    rtc_exit_init();
  }

  /* Callback initially disabled.*/
  RTCD1.callback = NULL;

  /* Enabling RTC-related EXTI lines.*/
  STM32_RTC_ENABLE_ALL_EXTI();

  /* IRQ vectors permanently assigned to this driver.*/
  STM32_RTC_IRQ_ENABLE();
}

/**
 * @brief   Set current time.
 * @note    Fractional part will be silently ignored. There is no possibility
 *          to set it on STM32 platform.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCDateTime structure
 *
 * @notapi
 */
void rtc_lld_set_time(RTCDriver *rtcp, const RTCDateTime *timespec) {
  uint32_t dr, tr;
  syssts_t sts;

  tr = rtc_encode_time(timespec);
  dr = rtc_encode_date(timespec);

  sts = osalSysGetStatusAndLockX();

  rtc_enter_init();
  rtcp->rtc->TR = tr;
  rtcp->rtc->DR = dr;
  rtcp->rtc->CR = (rtcp->rtc->CR & ~(1U << RTC_CR_BKP_OFFSET)) |
                  (timespec->dstflag << RTC_CR_BKP_OFFSET);
  rtc_exit_init();

  osalSysRestoreStatusX(sts);
}

/**
 * @brief   Get current time.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[out] timespec pointer to a @p RTCDateTime structure
 *
 * @notapi
 */
void rtc_lld_get_time(RTCDriver *rtcp, RTCDateTime *timespec) {
  uint32_t cr, dr, tr, ssr, prev_dr, prev_tr, prev_ssr;
  uint32_t subs;
  syssts_t sts;

  sts = osalSysGetStatusAndLockX();

  ssr = 0U;
  tr  = 0U;
  dr  = 0U;
  do {
    prev_ssr = ssr;
    prev_tr  = tr;
    prev_dr  = dr;
    ssr = rtcp->rtc->SSR;
    tr  = rtcp->rtc->TR;
    dr  = rtcp->rtc->DR;
  } while ((ssr != prev_ssr) || (tr != prev_tr) || (dr != prev_dr));

  cr = rtcp->rtc->CR;

  osalSysRestoreStatusX(sts);

  rtc_decode_time(tr, timespec);

  subs = (((STM32_RTC_PRESS_VALUE - 1U) - ssr) * 1000U) / STM32_RTC_PRESS_VALUE;
  timespec->millisecond += subs;

  rtc_decode_date(dr, timespec);

  timespec->dstflag = (cr >> RTC_CR_BKP_OFFSET) & 1U;
}

#if (RTC_ALARMS > 0) || defined(__DOXYGEN__)
/**
 * @brief   Set alarm time.
 * @note    Function does not performs any checks of alarm time validity.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] alarm     alarm identifier. Can be 0.
 * @param[in] alarmspec pointer to a @p RTCAlarm structure
 *
 * @notapi
 */
void rtc_lld_set_alarm(RTCDriver *rtcp,
                       rtcalarm_t alarm,
                       const RTCAlarm *alarmspec) {
  syssts_t sts;

  osalDbgAssert(alarm == 0U, "invalid alarm");

  sts = osalSysGetStatusAndLockX();

  if (alarmspec != NULL) {
    rtcp->rtc->CR &= ~RTC_CR_ALRAE;
#if defined(RTC_ICSR_ALRAWF)
    while ((rtcp->rtc->ICSR & RTC_ICSR_ALRAWF) == 0U)
      ;
#endif
    rtcp->rtc->ALRMAR = alarmspec->alrmr;
    rtcp->rtc->CR |= RTC_CR_ALRAE;
    rtcp->rtc->CR |= RTC_CR_ALRAIE;
  }
  else {
    rtcp->rtc->CR &= ~RTC_CR_ALRAIE;
    rtcp->rtc->CR &= ~RTC_CR_ALRAE;
  }

  osalSysRestoreStatusX(sts);
}

/**
 * @brief   Get alarm time.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp       pointer to RTC driver structure
 * @param[in] alarm      alarm identifier. Can be 0.
 * @param[out] alarmspec pointer to a @p RTCAlarm structure
 *
 * @notapi
 */
void rtc_lld_get_alarm(RTCDriver *rtcp,
                       rtcalarm_t alarm,
                       RTCAlarm *alarmspec) {

  osalDbgAssert(alarm == 0U, "invalid alarm");

  alarmspec->alrmr = rtcp->rtc->ALRMAR;
}
#endif /* RTC_ALARMS > 0 */

/**
 * @brief   Enables or disables RTC callbacks.
 * @details This function enables or disables callbacks, use a @p NULL pointer
 *          in order to disable a callback.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] callback  callback function pointer or @p NULL
 *
 * @notapi
 */
void rtc_lld_set_callback(RTCDriver *rtcp, rtccb_t callback) {

  rtcp->callback = callback;
}

#endif /* HAL_USE_RTC */

/** @} */
