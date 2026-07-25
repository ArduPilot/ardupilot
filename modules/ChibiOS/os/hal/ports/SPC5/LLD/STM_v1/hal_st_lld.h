/*
    SPC5 HAL - Copyright (C) 2013 STMicroelectronics

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
 * @file    SPC5xx/STMv1/hal_st_lld.h
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
#include "spc5_registry.h"
#include "registers.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    STM CR register definitions.
 * @{
 */
#define STM_CR_CNT_MASK                     (255U << 8U)
#define STM_CR_CNT(n)                       ((n) << 8U)
#define STM_CR_FRZ                          (1U << 1U)
#define STM_CR_TEN                          (1U << 0U)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   STM unit to be used by the ST driver.
 */
#if !defined(SPC5_STM_UNIT) || defined(__DOXYGEN__)
#define SPC5_STM_UNIT                       STM_2
#endif

/**
 * @brief   SysTick timer IRQ priority.
 */
#if !defined(SPC5_STM_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_STM_IRQ_PRIORITY               8
#endif

/**
 * @brief   Counter clock to be programmed in the STM unit.
 */
#if !defined(SPC5_STM_CNT_CLOCK) || defined(__DOXYGEN__)
#define SPC5_STM_CNT_CLOCK                  8000000U
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if 0
#if !defined(STM_HANDLER)
#error "STM_HANDLER not defined in registry"
#endif

#if !defined(SPC5_STM_CLK)
#error "SPC5_STM_CLK not defined in registry"
#endif

/**
 * @brief   Prescaler value.
 */
#define SPC5_STM_CPL_VALUE                  (SPC5_STM_CLK / SPC5_STM_CNT_CLOCK)

#if (SPC5_STM_CPL_VALUE * SPC5_STM_CNT_CLOCK) != SPC5_STM_CLK
#error "SPC5_STM_CNT_CLOCK is invalid"
#endif
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

  return (systime_t)0;
}

/**
 * @brief   Starts the alarm.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 *
 * @param[in] time      the time to be set for the first alarm
 *
 * @notapi
 */
static inline void st_lld_start_alarm(systime_t time) {

  (void)time;
}

/**
 * @brief   Stops the alarm interrupt.
 *
 * @notapi
 */
static inline void st_lld_stop_alarm(void) {

}

/**
 * @brief   Sets the alarm time.
 *
 * @param[in] time      the time to be set for the next alarm
 *
 * @notapi
 */
static inline void st_lld_set_alarm(systime_t time) {

  (void)time;
}

/**
 * @brief   Returns the current alarm time.
 *
 * @return              The currently set alarm time.
 *
 * @notapi
 */
static inline systime_t st_lld_get_alarm(void) {

  return (systime_t)0;
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

  return false;
}

#endif /* HAL_ST_LLD_H */

/** @} */
