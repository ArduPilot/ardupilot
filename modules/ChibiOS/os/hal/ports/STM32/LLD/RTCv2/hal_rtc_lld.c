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
 * @file    RTCv2/hal_rtc_lld.c
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

#define RTC_TR_PM_OFFSET                    22
#define RTC_TR_HT_OFFSET                    20
#define RTC_TR_HU_OFFSET                    16
#define RTC_TR_MNT_OFFSET                   12
#define RTC_TR_MNU_OFFSET                   8
#define RTC_TR_ST_OFFSET                    4
#define RTC_TR_SU_OFFSET                    0

#define RTC_DR_YT_OFFSET                    20
#define RTC_DR_YU_OFFSET                    16
#define RTC_DR_WDU_OFFSET                   13
#define RTC_DR_MT_OFFSET                    12
#define RTC_DR_MU_OFFSET                    8
#define RTC_DR_DT_OFFSET                    4
#define RTC_DR_DU_OFFSET                    0

#define RTC_CR_BKP_OFFSET                   18

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

  RTCD1.rtc->ISR |= RTC_ISR_INIT;
  while ((RTCD1.rtc->ISR & RTC_ISR_INITF) == 0)
    ;
}

/**
 * @brief   Finalizing of configuration procedure.
 *
 * @notapi
 */
static inline void rtc_exit_init(void) {

  RTCD1.rtc->ISR &= ~RTC_ISR_INIT;
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
  timespec->month = (((dr >> RTC_DR_MT_OFFSET) & 1) * 10) +
                     ((dr >> RTC_DR_MU_OFFSET) & 15);
  timespec->day   = (((dr >> RTC_DR_DT_OFFSET) & 3) * 10) +
                     ((dr >> RTC_DR_DU_OFFSET) & 15);
  timespec->dayofweek = (dr >> RTC_DR_WDU_OFFSET) & 7;
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
  dr = dr | (timespec->dayofweek << RTC_DR_WDU_OFFSET);

  return dr;
}

#if RTC_HAS_STORAGE == TRUE
static size_t _getsize(void *instance) {

  (void)instance;

  return (size_t)STM32_RTC_STORAGE_SIZE;
}

static ps_error_t _read(void *instance, ps_offset_t offset,
                        size_t n, uint8_t *rp) {
  volatile uint32_t *bkpr = &((RTCDriver *)instance)->rtc->BKP0R;
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
  volatile uint32_t *bkpr = &((RTCDriver *)instance)->rtc->BKP0R;
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
  uint32_t isr, clear;

  OSAL_IRQ_PROLOGUE();

  clear = (0U
           | RTC_ISR_TSF
           | RTC_ISR_TSOVF
#if defined(RTC_ISR_TAMP1F)
           | RTC_ISR_TAMP1F
#endif
#if defined(RTC_ISR_TAMP2F)
           | RTC_ISR_TAMP2F
#endif
#if defined(RTC_ISR_TAMP3F)
           | RTC_ISR_TAMP3F
#endif
#if defined(RTC_ISR_WUTF)
           | RTC_ISR_WUTF
#endif
#if defined(RTC_ISR_ALRAF)
           | RTC_ISR_ALRAF
#endif
#if defined(RTC_ISR_ALRBF)
           | RTC_ISR_ALRBF
#endif
          );

  isr = RTCD1.rtc->ISR;
  RTCD1.rtc->ISR = isr & ~clear;

  extiClearGroup1(EXTI_MASK1(STM32_RTC_ALARM_EXTI) |
                  EXTI_MASK1(STM32_RTC_TAMP_STAMP_EXTI) |
                  EXTI_MASK1(STM32_RTC_WKUP_EXTI));

  if (RTCD1.callback != NULL) {
    uint32_t cr = RTCD1.rtc->CR;
    uint32_t tcr;

#if defined(RTC_ISR_WUTF)
    if (((cr & RTC_CR_WUTIE) != 0U) && ((isr & RTC_ISR_WUTF) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_WAKEUP);
    }
#endif

#if defined(RTC_ISR_ALRAF)
    if (((cr & RTC_CR_ALRAIE) != 0U) && ((isr & RTC_ISR_ALRAF) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_ALARM_A);
    }
#endif
#if defined(RTC_ISR_ALRBF)
    if (((cr & RTC_CR_ALRBIE) != 0U) && ((isr & RTC_ISR_ALRBF) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_ALARM_B);
    }
#endif

    if ((cr & RTC_CR_TSIE) != 0U) {
      if ((isr & RTC_ISR_TSF) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TS);
      }
      if ((isr & RTC_ISR_TSOVF) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TS_OVF);
      }
    }

    /* This part is different depending on if the RTC has a TAMPCR or TAFCR
       register.*/
#if defined(RTC_TAFCR_TAMP1E)
    tcr = RTCD1.rtc->TAFCR;
    if ((tcr & RTC_TAFCR_TAMPIE) != 0U) {
#if defined(RTC_ISR_TAMP1F)
      if ((isr & RTC_ISR_TAMP1F) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TAMP1);
      }
#endif
#if defined(RTC_ISR_TAMP2F)
      if ((isr & RTC_ISR_TAMP2F) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TAMP2);
      }
#endif
    }

#else /* !defined(RTC_TAFCR_TAMP1E) */
    tcr = RTCD1.rtc->TAMPCR;
#if defined(RTC_ISR_TAMP1F)
    if (((tcr & RTC_TAMPCR_TAMP1IE) != 0U) &&
        ((isr & RTC_ISR_TAMP1F) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_TAMP1);
    }
#endif
#if defined(RTC_ISR_TAMP2F)
    if (((tcr & RTC_TAMPCR_TAMP2IE) != 0U) &&
        ((isr & RTC_ISR_TAMP2F) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_TAMP2);
    }
#endif
#if defined(RTC_ISR_TAMP3F)
    if (((tcr & RTC_TAMPCR_TAMP3IE) != 0U) &&
        ((isr & RTC_ISR_TAMP3F) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_TAMP3);
    }
#endif
#endif /* !defined(RTC_TAFCR_TAMP1E) */
  }

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_RTC_SUPPRESS_COMMON_ISR) */

#elif defined(STM32_RTC_TAMP_STAMP_HANDLER) &&                              \
      defined(STM32_RTC_WKUP_HANDLER) &&                                    \
      defined(STM32_RTC_ALARM_HANDLER)
/**
 * @brief   RTC TAMP/STAMP interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_RTC_TAMP_STAMP_HANDLER) {
  uint32_t isr, clear;

  OSAL_IRQ_PROLOGUE();

  clear = (0U
           | RTC_ISR_TSF
           | RTC_ISR_TSOVF
#if defined(RTC_ISR_TAMP1F)
           | RTC_ISR_TAMP1F
#endif
#if defined(RTC_ISR_TAMP2F)
           | RTC_ISR_TAMP2F
#endif
#if defined(RTC_ISR_TAMP3F)
           | RTC_ISR_TAMP3F
#endif
          );

  isr = RTCD1.rtc->ISR;
  RTCD1.rtc->ISR = isr & ~clear;

  extiClearGroup1(EXTI_MASK1(STM32_RTC_TAMP_STAMP_EXTI));

  if (RTCD1.callback != NULL) {
    uint32_t cr, tcr;

    cr = RTCD1.rtc->CR;
    if ((cr & RTC_CR_TSIE) != 0U) {
      if ((isr & RTC_ISR_TSF) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TS);
      }
      if ((isr & RTC_ISR_TSOVF) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TS_OVF);
      }
    }

    /* This part is different depending on if the RTC has a TAMPCR or TAFCR
       register.*/
#if defined(RTC_TAFCR_TAMP1E)
    tcr = RTCD1.rtc->TAFCR;
    if ((tcr & RTC_TAFCR_TAMPIE) != 0U) {
#if defined(RTC_ISR_TAMP1F)
      if ((isr & RTC_ISR_TAMP1F) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TAMP1);
      }
#endif
#if defined(RTC_ISR_TAMP2F)
      if ((isr & RTC_ISR_TAMP2F) != 0U) {
        RTCD1.callback(&RTCD1, RTC_EVENT_TAMP2);
      }
#endif
    }

#else /* !defined(RTC_TAFCR_TAMP1E) */
    tcr = RTCD1.rtc->TAMPCR;
#if defined(RTC_ISR_TAMP1F)
    if (((tcr & RTC_TAMPCR_TAMP1IE) != 0U) &&
        ((isr & RTC_ISR_TAMP1F) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_TAMP1);
    }
#endif
#if defined(RTC_ISR_TAMP2F)
    if (((tcr & RTC_TAMPCR_TAMP2IE) != 0U) &&
        ((isr & RTC_ISR_TAMP2F) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_TAMP2);
    }
#endif
#if defined(RTC_ISR_TAMP3F)
    if (((tcr & RTC_TAMPCR_TAMP3IE) != 0U) &&
        ((isr & RTC_ISR_TAMP3F) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_TAMP3);
    }
#endif
#endif /* !defined(RTC_TAFCR_TAMP1E) */
  }

  OSAL_IRQ_EPILOGUE();
}
/**
 * @brief   RTC wakeup interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_RTC_WKUP_HANDLER) {
  uint32_t isr;

  OSAL_IRQ_PROLOGUE();

  isr = RTCD1.rtc->ISR;
  RTCD1.rtc->ISR = isr & ~RTC_ISR_WUTF;

  extiClearGroup1(EXTI_MASK1(STM32_RTC_WKUP_EXTI));

  if (RTCD1.callback != NULL) {
    uint32_t cr = RTCD1.rtc->CR;

    if (((cr & RTC_CR_WUTIE) != 0U) && ((isr & RTC_ISR_WUTF) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_WAKEUP);
    }
  }

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   RTC alarm interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_RTC_ALARM_HANDLER) {
  uint32_t isr, clear;

  OSAL_IRQ_PROLOGUE();

  clear = (0U
#if defined(RTC_ISR_ALRAF)
           | RTC_ISR_ALRAF
#endif
#if defined(RTC_ISR_ALRBF)
           | RTC_ISR_ALRBF
#endif
          );

  isr = RTCD1.rtc->ISR;
  RTCD1.rtc->ISR = isr & ~clear;

  extiClearGroup1(EXTI_MASK1(STM32_RTC_ALARM_EXTI));

  if (RTCD1.callback != NULL) {
    uint32_t cr = RTCD1.rtc->CR;
#if defined(RTC_ISR_ALRAF)
    if (((cr & RTC_CR_ALRAIE) != 0U) && ((isr & RTC_ISR_ALRAF) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_ALARM_A);
    }
#endif
#if defined(RTC_ISR_ALRBF)
    if (((cr & RTC_CR_ALRBIE) != 0U) && ((isr & RTC_ISR_ALRBF) != 0U)) {
      RTCD1.callback(&RTCD1, RTC_EVENT_ALARM_B);
    }
#endif
  }

  OSAL_IRQ_EPILOGUE();
}

#else
#error "missing required RTC handlers definitions in registry"
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
  if (!(RTCD1.rtc->ISR & RTC_ISR_INITS)) {

    rtc_enter_init();

    RTCD1.rtc->CR       = STM32_RTC_CR_INIT | RTC_CR_BYPSHAD;
#if defined(RTC_TAFCR_TAMP1E)
    RTCD1.rtc->TAFCR    = STM32_RTC_TAMPCR_INIT;
#else
    RTCD1.rtc->TAMPCR   = STM32_RTC_TAMPCR_INIT;
#endif
    RTCD1.rtc->ISR      = RTC_ISR_INIT; /* Clearing all but RTC_ISR_INIT.   */
    RTCD1.rtc->PRER     = STM32_RTC_PRER_BITS & 0x7FFFU;
    RTCD1.rtc->PRER     = STM32_RTC_PRER_BITS;

    rtc_exit_init();
  }
  else {
    RTCD1.rtc->ISR &= ~RTC_ISR_RSF;
  }

  /* Callback initially disabled.*/
  RTCD1.callback = NULL;

  /* Enabling RTC-related EXTI lines.*/
  extiEnableGroup1(EXTI_MASK1(STM32_RTC_ALARM_EXTI) |
                   EXTI_MASK1(STM32_RTC_TAMP_STAMP_EXTI) |
                   EXTI_MASK1(STM32_RTC_WKUP_EXTI),
                   EXTI_MODE_RISING_EDGE | EXTI_MODE_ACTION_INTERRUPT);

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
  uint32_t cr, dr, tr, prev_dr, prev_tr;
  uint32_t subs;
#if STM32_RTC_HAS_SUBSECONDS
  uint32_t ssr, prev_ssr;
#endif /* STM32_RTC_HAS_SUBSECONDS */
  syssts_t sts;

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  /* Repeated registers read until 2 matching sets are found.*/
#if STM32_RTC_HAS_SUBSECONDS
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
#else /* !STM32_RTC_HAS_SUBSECONDS */
  tr  = 0U;
  dr  = 0U;
  do {
    prev_tr  = tr;
    prev_dr  = dr;
    tr  = rtcp->rtc->TR;
    dr  = rtcp->rtc->DR;
  } while ((tr != prev_tr) || (dr != prev_dr));
#endif /* !STM32_RTC_HAS_SUBSECONDS */

  /* DST bit is in CR, no need to poll on this one.*/
  cr  = rtcp->rtc->CR;

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);

  /* Decoding day time, this starts the atomic read sequence, see "Reading
     the calendar" in the RTC documentation.*/
  rtc_decode_time(tr, timespec);

  /* If the RTC is capable of sub-second counting then the value is
     normalized in milliseconds and added to the time.*/
#if STM32_RTC_HAS_SUBSECONDS
  subs = (((STM32_RTC_PRESS_VALUE - 1U) - ssr) * 1000U) / STM32_RTC_PRESS_VALUE;
#else
  subs = 0;
#endif /* STM32_RTC_HAS_SUBSECONDS */
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
      while (!(rtcp->rtc->ISR & RTC_ISR_ALRAWF))
        ;
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
      while (!(rtcp->rtc->ISR & RTC_ISR_ALRBWF))
        ;
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
 * @param[in] alarm     alarm identifier. Can be 0 or 1.
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
    while (!(rtcp->rtc->ISR & RTC_ISR_WUTWF))
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
