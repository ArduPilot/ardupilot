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
 * @file    RCCv1/stm32_bd.inc
 * @brief   Shared backup domain handler.
 *
 * @addtogroup STM32_BD_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Initializes the backup domain.
 */
__STATIC_INLINE void bd_init(void) {
  uint32_t bdcr;

  /* Current settings.*/
  bdcr = RCC->BDCR;

#if HAL_USE_RTC
  /* RTC enable.*/
  if ((bdcr & RCC_BDCR_RTCEN) == 0U) {
    bdcr |= RCC_BDCR_RTCEN;
  }
#endif

  /* Selectors.*/
  bdcr &= ~(RCC_BDCR_RTCSEL_Msk | RCC_BDCR_LSCOSEL_Msk);
  bdcr |= STM32_RTCSEL | STM32_LSCOSEL;

  /* Final settings.*/
  RCC->BDCR = bdcr;
}

/**
 * @brief   Resets the backup domain.
 * @note    WARNING! Changing RTC clock source impossible without reset
 *          of the whole BKP domain.
 */
__STATIC_INLINE void bd_reset(void) {

  /* Reset BKP domain if different clock source selected.*/
  if ((RCC->BDCR & RCC_BDCR_RTCSEL_Msk) != STM32_RTCSEL) {
    /* Backup domain reset.*/
#if defined(RCC_BDCR_BDRST)
    RCC->BDCR = RCC_BDCR_BDRST;
#elif defined(RCC_BDCR_VSWRST)
    RCC->BDCR = RCC_BDCR_VSWRST;
#else
#error "unknown bit name"
#endif
    RCC->BDCR = 0U;
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
