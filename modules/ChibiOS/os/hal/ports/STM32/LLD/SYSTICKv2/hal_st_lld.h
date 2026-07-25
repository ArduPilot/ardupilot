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
 * @file    SYSTICKv2/hal_st_lld.h
 * @brief   RTC V3 based ST Driver subsystem low level driver header.
 *
 * @addtogroup ST
 * @{
 */

#ifndef HAL_RTC_ST_LLD_H
#define HAL_RTC_ST_LLD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   RTC SSR registry initial value.
 */
#define STM32_RTC_SSR_INIT_VALUE            0xFFFFFFFFUL

/* Requires services from the EXTI driver.*/
#if !defined(STM32_EXTI_REQUIRED)
#define STM32_EXTI_REQUIRED
#endif

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   SysTick timer IRQ priority.
 */
#if !defined(STM32_ST_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ST_IRQ_PRIORITY               8
#endif

/**
 * @brief   RTC binary mode attribute default value.
 */
#if !defined(STM32_RTC_HAS_BINARY_MODE) || defined(__DOXYGEN__)
#define STM32_RTC_HAS_BINARY_MODE           FALSE
#endif

/**
 * @brief   RTC mixed mode attribute default value.
 */
#if !defined(STM32_RTC_HAS_MIXED_MODE) || defined(__DOXYGEN__)
#define STM32_RTC_HAS_MIXED_MODE            FALSE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if OSAL_ST_MODE != OSAL_ST_MODE_FREERUNNING
#error "ST based on RTC supports only free running mode. Change CH_CFG_ST_TIMEDELTA to enable tick-less mode."
#endif

#if STM32_HAS_RTC == FALSE
#error "RTC not present in the selected device"
#endif

#if HAL_USE_RTC == TRUE
#error "ST requires RTC but it is already used"
#endif

#if STM32_RTC_HAS_BINARY_MODE == FALSE
#error "RTC does not support binary mode"
#endif

#if (OSAL_ST_RESOLUTION != 32)
#error "ST based on RTC requires 32bits resolution. Set CH_CFG_ST_RESOLUTION to 32."
#endif

#if (STM32_RTCCLK % OSAL_ST_FREQUENCY) != 0
#error "the selected ST frequency is not obtainable because integer rounding"
#endif

#if (STM32_RTCCLK / OSAL_ST_FREQUENCY) > 128
#error "the selected ST frequency is not obtainable because RTC Prescaler A limits"
#endif

/**
 * @brief   ST Deep Sleep support attrubute.
 */
#define STM32_ST_DEEP_SLEEP_SUPPORT         TRUE

/**
 * @brief   ST Alarms number.
 */
#define ST_LLD_NUM_ALARMS                   1

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

/**
 * @brief   Returns the time counter value.
 *
 * @return              The counter value.
 *
 * @notapi
 */
static inline systime_t st_lld_get_counter(void) {

  return (systime_t)(STM32_RTC_SSR_INIT_VALUE - RTC->SSR);
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

  /* Disable RTC Alarm A.*/
  RTC->CR &= ~(RTC_CR_ALRAE | RTC_CR_ALRAIE);

  /* Set alarm time.*/
  RTC->ALRABINR = (STM32_RTC_SSR_INIT_VALUE - (uint32_t)abstime);

  /* Enabel RTC Alarm A.*/
  RTC->CR |= (RTC_CR_ALRAE | RTC_CR_ALRAIE);
}

/**
 * @brief   Sets the alarm time.
 *
 * @param[in] abstime   the time to be set for the next alarm
 *
 * @notapi
 */
static inline void st_lld_set_alarm(systime_t abstime) {

  st_lld_start_alarm(abstime);
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

  return (bool)((RTC->CR & RTC_CR_ALRAE) != 0);
}

/**
 * @brief   Stops the alarm interrupt.
 *
 * @notapi
 */
static inline void st_lld_stop_alarm(void) {

  RTC->CR &= ~(RTC_CR_ALRAE | RTC_CR_ALRAIE);
}

/**
 * @brief   Returns the current alarm time.
 *
 * @return              The currently set alarm time.
 *
 * @notapi
 */
static inline systime_t st_lld_get_alarm(void) {

  return (systime_t)(STM32_RTC_SSR_INIT_VALUE - RTC->ALRABINR);
}

#endif /* HAL_RTC_ST_LLD_H */

/** @} */
