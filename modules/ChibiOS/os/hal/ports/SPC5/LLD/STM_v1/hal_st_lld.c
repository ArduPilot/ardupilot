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
 * @file    SPC5xx/hal_st_lld.c
 * @brief   ST Driver subsystem low level driver code.
 *
 * @addtogroup ST
 * @{
 */

#include "hal.h"

#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

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

#if defined(SPC5_USE_STM) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(STM_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  if (SPC5_STM_UNIT->CH[0].CIR.CIF != 0U) {
    SPC5_STM_UNIT->CH[0].CIR.CIF = 1U;

    osalSysLockFromISR();
    osalOsTimerHandlerI();
    osalSysUnlockFromISR();
  }

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

#if defined(SPC5_USE_STM)
  SPC5_STM_UNIT->CNT       = 0;
  SPC5_STM_UNIT->CH[0].CCR = 0;
  SPC5_STM_UNIT->CH[1].CCR = 0;
  SPC5_STM_UNIT->CH[2].CCR = 0;
  SPC5_STM_UNIT->CH[3].CCR = 0;
  SPC5_STM_UNIT->CR.R = STM_CR_CNT(SPC5_STM_CPL_VALUE - 1) |
                        STM_CR_FRZ | STM_CR_TEN;
#endif
}

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
