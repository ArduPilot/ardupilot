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
 * @brief   ST Driver subsystem low level driver code.
 *
 * @addtogroup ST
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define ST_HANDLER                          STM32_RTC_ALARM_HANDLER
#define ST_NUMBER                           STM32_RTC_ALARM_NUMBER

#define STM32_ST_RTC_PREDIVA                (STM32_RTCCLK / OSAL_ST_FREQUENCY)

/**
 * Initialization for the RTC_PRER register.
 */
#define STM32_ST_RTC_PRER_BITS              ((STM32_ST_RTC_PREDIVA - 1) << RTC_PRER_PREDIV_A_Pos)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if !defined(STM32_SYSTICK_SUPPRESS_ISR)
/**
 * @brief   Interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(ST_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  st_lld_serve_interrupt();

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ST driver initialization.
 *
 * @notapi
 */
void st_lld_init(void) {

  /* Enabling the stop mode during debug for RTC.*/
  DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_RTC_STOP;

  /* Enable RTC kernel clock.*/
  RCC->BDCR |= RCC_BDCR_RTCEN;

  /* Enable RTC APB bus clock.*/
  rccEnableAPB1R1(RCC_APB1ENR1_RTCAPBEN, true);

  /* Disable RTC write protection.*/
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;

  /* Disable all alarms and interrupts.*/
  RTC->CR &= ~(RTC_CR_ALRAIE | RTC_CR_ALRBIE | RTC_CR_ALRAE | RTC_CR_ALRBE);

  /* Enter initialization mode.*/
  RTC->ICSR |= RTC_ICSR_INIT;
  while ((RTC->ICSR & RTC_ICSR_INITF) == 0U) {
    /* Waint for init flag.*/
  }

  /* Activate free running Binary mode.*/
  RTC->ICSR |= RTC_ICSR_BIN_0;
  /* Set RTC prescaler.
     Setting PRER has to be done as two writes. Write Sync part first
     then Sync + Async. */
  RTC->PRER = STM32_ST_RTC_PRER_BITS & 0x7FFF;
  RTC->PRER = STM32_ST_RTC_PRER_BITS;

  /* Exit initialization mode.*/
  RTC->ICSR &= ~RTC_ICSR_INIT;

  /* Wait for shadow reg. update.*/
  while ((RTC->ICSR & RTC_ICSR_RSF) == 0U) {
    /* Wait RSF flag.*/
  }

  /* Compare all Sub Seconds 32 bits for RTC Alarm A.*/
  RTC->ALRMASSR = (32UL << RTC_ALRMASSR_MASKSS_Pos);

  /* EXTI enable.*/
  extiEnableGroup1(EXTI_MASK1(STM32_RTC_ALARM_EXTI), EXTI_MODE_RISING_EDGE | EXTI_MODE_ACTION_INTERRUPT);
  /* IRQ enable.*/
  nvicEnableVector(ST_NUMBER, STM32_ST_IRQ_PRIORITY);

}

/**
 * @brief   IRQ handling code.
 */
void st_lld_serve_interrupt(void) {

  uint32_t isr;

  /* Get and clear the RTC interrupts. */
  isr = RTC->MISR;
  RTC->SCR = isr;

  if ((isr & RTC_MISR_ALRAMF) != 0U) {

    osalSysLockFromISR();
    osalOsTimerHandlerI();
    osalSysUnlockFromISR();
  }

}

/** @} */
