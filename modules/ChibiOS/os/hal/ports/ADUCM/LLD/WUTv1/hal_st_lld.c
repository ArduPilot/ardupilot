/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    ADUCM36x/hal_st_lld.c
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

#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING

#if !defined(ADUCM_PCLK0)
#define ADUCM_PCLK0                         ADUCM_PCLK
#endif

#define ST_HANDLER                          ADUCM_WUT_HANDLER
#define ST_NUMBER                           ADUCM_WUT_NUMBER
#define ST_CLOCK_SRC                        ADUCM_PCLK0

#if (((ST_CLOCK_SRC / OSAL_ST_FREQUENCY) % 4 == 0) &&                       \
     ((ST_CLOCK_SRC / OSAL_ST_FREQUENCY) / 4 == 1))
#define ST_CLOCK_PRESC                      ADUCM_WUT_CON_PRE_DIV4
#elif (((ST_CLOCK_SRC / OSAL_ST_FREQUENCY) % 16 == 0) &&                    \
       ((ST_CLOCK_SRC / OSAL_ST_FREQUENCY) / 16 == 1))
#define ST_CLOCK_PRESC                      ADUCM_WUT_CON_PRE_DIV16
#elif (((ST_CLOCK_SRC / OSAL_ST_FREQUENCY) % 256 == 0) &&                   \
       ((ST_CLOCK_SRC / OSAL_ST_FREQUENCY) / 256 == 1))
#define ST_CLOCK_PRESC                      ADUCM_WUT_CON_PRE_DIV256
#else
#error "the selected ST frequency is not obtainable prescaler limitations" 
#endif

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC

#define ST_HANDLER                          SysTick_Handler
#define ST_NUMBER                           HANDLER_SYSTICK
#define SYSTICK_CK                          ADUCM_HCLK


#if SYSTICK_CK % OSAL_ST_FREQUENCY != 0
#error "the selected ST frequency is not obtainable because integer rounding"
#endif

#if (SYSTICK_CK / OSAL_ST_FREQUENCY) - 1 > 0xFFFFFF
#error "the selected ST frequency is not obtainable because SysTick timer counter limits"
#endif

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */

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

#if !defined(ADUCM_SYSTICK_SUPPRESS_ISR)
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

#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING
  /* Free running counter mode.*/

  ADUCM_ST_TIM->CON = ST_CLOCK_PRESC | ADUCM_WUT_CON_TMODE_FREERUN;

  ADUCM_ST_TIM->WUFB0 = 0;
  ADUCM_ST_TIM->WUFB1 = 0;
  ADUCM_ST_TIM->CLRI = ADUCM_WUT_CLRI_WUFA | ADUCM_WUT_CLRI_WUFB |
                       ADUCM_WUT_CLRI_WUFC | ADUCM_WUT_CLRI_WUFD |
                       ADUCM_WUT_CLRI_ROLL;
  ADUCM_ST_TIM->IEN = 0;

  ADCUM_WUT_ENABLE(ADUCM_ST_TIM);

#if !defined(ADUCM_SYSTICK_SUPPRESS_ISR)
  /* IRQ enabled.*/
  nvicEnableVector(ST_NUMBER, ADUCM_ST_IRQ_PRIORITY);
#endif
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC
  /* Periodic systick mode, the Cortex-Mx internal systick timer is used
     in this mode.*/
  SysTick->LOAD = (SYSTICK_CK / OSAL_ST_FREQUENCY) - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_ENABLE_Msk |
                  SysTick_CTRL_TICKINT_Msk;

#if !defined(ADUCM_SYSTICK_SUPPRESS_ISR)
  /* IRQ enabled.*/
  nvicSetSystemHandlerPriority(ST_NUMBER, ADUCM_ST_IRQ_PRIORITY);
#endif
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */
}

/**
 * @brief   IRQ handling code.
 */
void st_lld_serve_interrupt(void) {
#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING
  uint32_t sr;
  sr = ADUCM_ST_TIM->STA;
  ADUCM_ST_TIM->CLRI = ADUCM_WUT_CLRI_WUFA | ADUCM_WUT_CLRI_WUFB |
                       ADUCM_WUT_CLRI_WUFC | ADUCM_WUT_CLRI_WUFD |
                       ADUCM_WUT_CLRI_ROLL;
  if((sr & ADUCM_WUT_STA_WUFB) != 0U)
#endif
  {
    osalSysLockFromISR();
    osalOsTimerHandlerI();
    osalSysUnlockFromISR();
  }
  __DSB();
}
#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
