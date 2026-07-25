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
 * @file    RTCv3/hal_rtc_lld.c
 * @brief   STM32 RTC low level driver.
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
/* Driver local variables and types.                                         */
/*===========================================================================*/

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
  while ((RTCD1.rtc->ICSR & RTC_ICSR_INITF) == 0)
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

  n  = ((tr >> RTC_TR_HT_OFFSET) & 3)   * 36000000;
  n += ((tr >> RTC_TR_HU_OFFSET) & 15)  * 3600000;
  n += ((tr >> RTC_TR_MNT_OFFSET) & 7)  * 600000;
  n += ((tr >> RTC_TR_MNU_OFFSET) & 15) * 60000;
  n += ((tr >> RTC_TR_ST_OFFSET) & 7)   * 10000;
  n += ((tr >> RTC_TR_SU_OFFSET) & 15)  * 1000;
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

  timespec->year  = (((dr >> RTC_DR_YT_OFFSET) & 15) * 10) +
                     ((dr >> RTC_DR_YU_OFFSET) & 15);
  timespec->month = (((dr >> RTC_TR_MNT_OFFSET) & 1) * 10) +
                     ((dr >> RTC_TR_MNU_OFFSET) & 15);
  timespec->day   = (((dr >> RTC_DR_DT_OFFSET) & 3) * 10) +
                     ((dr >> RTC_DR_DU_OFFSET) & 15);
  timespec->dayofweek = ((dr >> RTC_DR_WDU_OFFSET) & 7) + 1;
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
  uint32_t n, tr = 0;

  /* Subseconds cannot be set.*/
  n = timespec->millisecond / 1000;

  /* Seconds conversion.*/
  tr = tr | ((n % 10) << RTC_TR_SU_OFFSET);
  n /= 10;
  tr = tr | ((n % 6) << RTC_TR_ST_OFFSET);
  n /= 6;

  /* Minutes conversion.*/
  tr = tr | ((n % 10) << RTC_TR_MNU_OFFSET);
  n /= 10;
  tr = tr | ((n % 6) << RTC_TR_MNT_OFFSET);
  n /= 6;

  /* Hours conversion.*/
  tr = tr | ((n % 10) << RTC_TR_HU_OFFSET);
  n /= 10;
  tr = tr | (n << RTC_TR_HT_OFFSET);

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
  uint32_t n, dr = 0;

  /* Year conversion. Note, only years last two digits are considered.*/
  n = timespec->year;
  dr = dr | ((n % 10) << RTC_DR_YU_OFFSET);
  n /= 10;
  dr = dr | ((n % 10) << RTC_DR_YT_OFFSET);

  /* Months conversion.*/
  n = timespec->month;
  dr = dr | ((n % 10) << RTC_DR_MU_OFFSET);
  n /= 10;
  dr = dr | ((n % 10) << RTC_DR_MT_OFFSET);

  /* Days conversion.*/
  n = timespec->day;
  dr = dr | ((n % 10) << RTC_DR_DU_OFFSET);
  n /= 10;
  dr = dr | ((n % 10) << RTC_DR_DT_OFFSET);

  /* Days of week conversion.*/
  dr = dr | ((timespec->dayofweek) << RTC_DR_WDU_OFFSET);

  return dr;
}

#if RTC_HAS_STORAGE == TRUE
static size_t _getsize(void *instance) {

  (void)instance;

  return (size_t)STM32_RTC_STORAGE_SIZE;
}

static ps_error_t _read(void *instance, ps_offset_t offset,
                        size_t n, uint8_t *rp) {
  volatile uint32_t *bkpr = &((RTCDriver *)instance)->tamp->BKP0R;
  unsigned i;

  osalDbgCheck((instance != NULL) && (rp != NULL));
  osalDbgCheck((n > 0U) && (n <= STM32_RTC_STORAGE_SIZE));
  osalDbgCheck((offset < STM32_RTC_STORAGE_SIZE) &&
               (offset + n <= STM32_RTC_STORAGE_SIZE));

  for (i = 0; i < (unsigned)n; i++) {
    unsigned index = ((unsigned)offset + i) / sizeof (uint32_t);
    unsigned shift = ((unsigned)offset + i) % sizeof (uint32_t);
    *rp++ = (uint8_t)(bkpr[index] >> (shift * 8U));
  }

  return PS_NO_ERROR;
}

static ps_error_t _write(void *instance, ps_offset_t offset,
                         size_t n, const uint8_t *wp) {
  volatile uint32_t *bkpr = &((RTCDriver *)instance)->tamp->BKP0R;
  unsigned i;

  osalDbgCheck((instance != NULL) && (wp != NULL));
  osalDbgCheck((n > 0U) && (n <= STM32_RTC_STORAGE_SIZE));
  osalDbgCheck((offset < STM32_RTC_STORAGE_SIZE) &&
               (offset + n <= STM32_RTC_STORAGE_SIZE));

  for (i = 0; i < (unsigned)n; i++) {
    unsigned index = ((unsigned)offset + i) / sizeof (uint32_t);
    unsigned shift = ((unsigned)offset + i) % sizeof (uint32_t);
    uint32_t regval = bkpr[index];
    regval &= ~(0xFFU << (shift * 8U));
    regval |= (uint32_t)*wp++ << (shift * 8U);
    bkpr[index] = regval;
  }

  return PS_NO_ERROR;
}

/**
 * @brief   VMT for the RTC storage file interface.
 */
struct RTCDriverVMT _rtc_lld_vmt = {
  (size_t)0,
  _getsize, _read, _write
};
#endif /* RTC_HAS_STORAGE == TRUE */

/**
 * @brief   RTC ISR service routine.
 *
 */
static void rtc_lld_serve_interrupt(void) {

  uint32_t isr;

  /* Get and clear the RTC interrupts. */
  isr = RTCD1.rtc->MISR;
  RTCD1.rtc->SCR = isr;

  /* Clear EXTI events. */
  STM32_RTC_CLEAR_ALL_EXTI();

  /* Process call backs if enabled. */
  if (RTCD1.callback != NULL) {

#if defined(RTC_MISR_WUTMF)
    if ((isr & RTC_MISR_WUTMF) != 0U) {
      RTCD1.callback(&RTCD1, RTC_EVENT_WAKEUP);
    }
#endif

#if defined(RTC_MISR_ALRAMF)
    if ((isr & RTC_MISR_ALRAMF) != 0U) {
      RTCD1.callback(&RTCD1, RTC_EVENT_ALARM_A);
    }
#endif
#if defined(RTC_MISR_ALRBMF)
    if ((isr & RTC_MISR_ALRBMF) != 0U) {
      RTCD1.callback(&RTCD1, RTC_EVENT_ALARM_B);
    }
#endif
#if defined(RTC_MISR_ITSMF)
    if ((isr & RTC_MISR_ITSMF) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TS);
      }
#endif
#if defined(RTC_MISR_TSOVMF)
    if ((isr & RTC_MISR_TSOVMF) != 0U) {
      RTCD1.callback(&RTCD1, RTC_EVENT_TS_OVF);
    }
#endif

    /* Get and clear the TAMP interrupts. */
     isr = RTCD1.tamp->MISR;
     RTCD1.tamp->SCR = isr;
#if defined(TAMP_MISR_TAMP1MF)
     if ((isr & TAMP_MISR_TAMP1MF) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TAMP1);
      }
#endif
#if defined(TAMP_MISR_TAMP2MF)
     if ((isr & TAMP_MISR_TAMP2MF) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TAMP2);
      }
#endif
#if defined(TAMP_MISR_ITAMP3MF)
     if ((isr & TAMP_MISR_ITAMP3MF) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TAMP3);
      }
#endif
#if defined(TAMP_MISR_ITAMP4MF)
     if ((isr & TAMP_MISR_ITAMP4MF) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TAMP4);
      }
#endif
#if defined(TAMP_MISR_ITAMP5MF)
     if ((isr & TAMP_MISR_ITAMP5MF) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TAMP5);
      }
#endif
#if defined(TAMP_MISR_ITAMP6MF)
     if ((isr & TAMP_MISR_ITAMP6MF) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TAMP6);
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

#elif defined(STM32_RTC_GLOBAL_HANDLER) &&                                  \
      defined(STM32_RTC_TAMP_HANDLER)
/**
 * @brief   RTC global interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_RTC_GLOBAL_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  rtc_lld_serve_interrupt();

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   RTC TAMP interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_RTC_TAMP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  rtc_lld_serve_interrupt();

  OSAL_IRQ_EPILOGUE();
}

#elif defined(STM32_RTC_TAMP_STAMP_HANDLER) &&                              \
      defined(STM32_RTC_WKUP_HANDLER) &&                                    \
      defined(STM32_RTC_ALARM_HANDLER)
/**
 * @brief   RTC TAMP/STAMP interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_RTC_TAMP_STAMP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  rtc_lld_serve_interrupt();

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   RTC wakeup interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_RTC_WKUP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  rtc_lld_serve_interrupt();

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   RTC alarm interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_RTC_ALARM_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  rtc_lld_serve_interrupt();

  OSAL_IRQ_EPILOGUE();
}

#else
#error "missing required RTC handler definitions in registry"
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

  /* Disable write protection. */
  RTCD1.rtc->WPR = 0xCAU;
  RTCD1.rtc->WPR = 0x53U;

  /* If calendar has not been initialized yet then proceed with the
     initial setup.*/
  if (!(RTCD1.rtc->ICSR & RTC_ICSR_INITS)) {

    rtc_enter_init();

    RTCD1.rtc->CR   = (STM32_RTC_CR_INIT & STM32_RTC_CR_MASK) | RTC_CR_BYPSHAD;
    /* Setting PRER has to be done as two writes. Write Sync part first
       then Sync + Async. */
    RTCD1.rtc->PRER = STM32_RTC_PRER_BITS & 0x7FFFU;
    RTCD1.rtc->PRER = STM32_RTC_PRER_BITS;

    rtc_exit_init();
  }

  /* TAMP pointer initialization. */
  RTCD1.tamp = TAMP;

  /* Initialise TAMP registers, BYPSHAD is enforced.*/
  RTCD1.tamp->CR1   |= (STM32_TAMP_CR1_INIT & STM32_TAMP_CR1_MASK);
  RTCD1.tamp->CR2   |= (STM32_TAMP_CR2_INIT & STM32_TAMP_CR2_MASK);
  RTCD1.tamp->FLTCR |= (STM32_TAMP_FLTCR_INIT & STM32_TAMP_FLTCR_MASK);
  RTCD1.tamp->IER   |= (STM32_TAMP_IER_INIT & STM32_TAMP_IER_MASK);

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

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  /* Writing the registers.*/
  rtc_enter_init();
  rtcp->rtc->TR = tr;
  rtcp->rtc->DR = dr;
  rtcp->rtc->CR = (rtcp->rtc->CR & ~(1U << RTC_CR_BKP_OFFSET)) |
                  (timespec->dstflag << RTC_CR_BKP_OFFSET);
  rtc_exit_init();

  /* Leaving a reentrant critical zone.*/
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

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  /* Repeated registers read until 2 matching sets are found.*/
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

  /* DST bit is in CR, no need to poll on this one.*/
  cr  = rtcp->rtc->CR;

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);

  /* Decoding day time, this starts the atomic read sequence, see "Reading
     the calendar" in the RTC documentation.*/
  rtc_decode_time(tr, timespec);

  /* The value is normalized in milliseconds and added to the time.*/
  subs = (((STM32_RTC_PRESS_VALUE - 1U) - ssr) * 1000U) / STM32_RTC_PRESS_VALUE;
  timespec->millisecond += subs;

  /* Decoding date, this concludes the atomic read sequence.*/
  rtc_decode_date(dr, timespec);

  /* Retrieving the DST bit.*/
  timespec->dstflag = (cr >> RTC_CR_BKP_OFFSET) & 1;
}

#if (RTC_ALARMS > 0) || defined(__DOXYGEN__)
/**
 * @brief   Set alarm time.
 * @note    Default value after BKP domain reset for both comparators is 0.
 * @note    Function does not performs any checks of alarm time validity.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure.
 * @param[in] alarm     alarm identifier. Can be 0 or 1.
 * @param[in] alarmspec pointer to a @p RTCAlarm structure.
 *
 * @notapi
 */
void rtc_lld_set_alarm(RTCDriver *rtcp,
                       rtcalarm_t alarm,
                       const RTCAlarm *alarmspec) {
  syssts_t sts;

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  if (alarm == 0) {
    if (alarmspec != NULL) {
      rtcp->rtc->CR &= ~RTC_CR_ALRAE;
#if defined(RTC_ICSR_ALRAWF)
      while (!(rtcp->rtc->ICSR & RTC_ICSR_ALRAWF))
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
  }
#if RTC_ALARMS > 1
  else {
    if (alarmspec != NULL) {
      rtcp->rtc->CR &= ~RTC_CR_ALRBE;
#if defined(RTC_ICSR_ALRBWF)
      while (!(rtcp->rtc->ICSR & RTC_ICSR_ALRBWF))
        ;
#endif
      rtcp->rtc->ALRMBR = alarmspec->alrmr;
      rtcp->rtc->CR |= RTC_CR_ALRBE;
      rtcp->rtc->CR |= RTC_CR_ALRBIE;
    }
    else {
      rtcp->rtc->CR &= ~RTC_CR_ALRBIE;
      rtcp->rtc->CR &= ~RTC_CR_ALRBE;
    }
  }
#endif /* RTC_ALARMS > 1 */

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);
}

/**
 * @brief   Get alarm time.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp       pointer to RTC driver structure
 * @param[in] alarm      alarm identifier. Can be 0 or 1.
 * @param[out] alarmspec pointer to a @p RTCAlarm structure
 *
 * @notapi
 */
void rtc_lld_get_alarm(RTCDriver *rtcp,
                       rtcalarm_t alarm,
                       RTCAlarm *alarmspec) {

  if (alarm == 0)
    alarmspec->alrmr = rtcp->rtc->ALRMAR;
#if RTC_ALARMS > 1
  else
    alarmspec->alrmr = rtcp->rtc->ALRMBR;
#endif /* RTC_ALARMS > 1 */
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

#if STM32_RTC_HAS_PERIODIC_WAKEUPS || defined(__DOXYGEN__)
/**
 * @brief   Sets time of periodic wakeup.
 * @note    Default value after BKP domain reset is 0x0000FFFF
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp       pointer to RTC driver structure
 * @param[in] wakeupspec pointer to a @p RTCWakeup structure
 *
 * @api
 */
void rtcSTM32SetPeriodicWakeup(RTCDriver *rtcp, const RTCWakeup *wakeupspec) {
  syssts_t sts;

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  if (wakeupspec != NULL) {
    osalDbgCheck(wakeupspec->wutr != 0x30000);

    rtcp->rtc->CR &= ~RTC_CR_WUTE;
    rtcp->rtc->CR &= ~RTC_CR_WUTIE;
    while (!(rtcp->rtc->ICSR & RTC_ICSR_WUTWF))
      ;
    rtcp->rtc->WUTR = wakeupspec->wutr & 0xFFFF;
    rtcp->rtc->CR &= ~RTC_CR_WUCKSEL;
    rtcp->rtc->CR |= (wakeupspec->wutr >> 16) & RTC_CR_WUCKSEL;
    rtcp->rtc->CR |= RTC_CR_WUTIE;
    rtcp->rtc->CR |= RTC_CR_WUTE;
  }
  else {
    rtcp->rtc->CR &= ~RTC_CR_WUTE;
    rtcp->rtc->CR &= ~RTC_CR_WUTIE;
  }

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);
}

/**
 * @brief   Gets time of periodic wakeup.
 * @note    Default value after BKP domain reset is 0x0000FFFF
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp        pointer to RTC driver structure
 * @param[out] wakeupspec pointer to a @p RTCWakeup structure
 *
 * @api
 */
void rtcSTM32GetPeriodicWakeup(RTCDriver *rtcp, RTCWakeup *wakeupspec) {
  syssts_t sts;

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  wakeupspec->wutr  = 0;
  wakeupspec->wutr |= rtcp->rtc->WUTR;
  wakeupspec->wutr |= (((uint32_t)rtcp->rtc->CR) & 0x7) << 16;

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);
}
#endif /* STM32_RTC_HAS_PERIODIC_WAKEUPS */

#endif /* HAL_USE_RTC */

/** @} */
