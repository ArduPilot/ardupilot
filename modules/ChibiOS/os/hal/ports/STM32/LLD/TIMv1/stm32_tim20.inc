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
 * @file    TIMv1/stm32_tim20.inc
 * @brief   Shared TIM20 handler.
 *
 * @addtogroup STM32_TIM20_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_HAS_TIM20)
#error "STM32_HAS_TIM20 not defined in registry"
#endif

/* Driver checks for robustness, undefined USE macros are defaulted to
   FALSE. This makes this module independent from drivers implementation.*/
#if !defined(STM32_GPT_USE_TIM20)
#define STM32_GPT_USE_TIM20                 FALSE
#endif
#if !defined(STM32_ICU_USE_TIM20)
#define STM32_ICU_USE_TIM20                 FALSE
#endif
#if !defined(STM32_EICU_USE_TIM20)
#define STM32_EICU_USE_TIM20                FALSE
#endif
#if !defined(STM32_PWM_USE_TIM20)
#define STM32_PWM_USE_TIM20                 FALSE
#endif
#if !defined(STM32_ST_USE_TIM20)
#define STM32_ST_USE_TIM20                  FALSE
#endif

#if STM32_HAS_TIM20

/* Priority settings checks.*/
#if !defined(STM32_IRQ_TIM20_UP_PRIORITY)
#error "STM32_IRQ_TIM20_UP_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(STM32_IRQ_TIM20_CC_PRIORITY)
#error "STM32_IRQ_TIM20_CC_PRIORITY not defined in mcuconf.h"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM20_UP_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM20_UP_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM20_CC_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM20_CC_PRIORITY"
#endif

#endif /* STM32_HAS_TIM20 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void tim20_irq_init(void) {
#if defined(STM32_TIM20_IS_USED)
  nvicEnableVector(STM32_TIM20_UP_NUMBER, STM32_IRQ_TIM20_UP_PRIORITY);
  nvicEnableVector(STM32_TIM20_CC_NUMBER, STM32_IRQ_TIM20_CC_PRIORITY);
#endif
}

static inline void tim20_irq_deinit(void) {
#if defined(STM32_TIM20_IS_USED)
  nvicDisableVector(STM32_TIM20_UP_NUMBER);
  nvicDisableVector(STM32_TIM20_CC_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_TIM20_IS_USED) || defined(__DOXYGEN__)
/**
 * @brief   TIM20-UP interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM20_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
#if STM32_GPT_USE_TIM20
  pwm_lld_serve_interrupt(&GPTD20);
#endif
#endif
#if HAL_USE_ICU
#if STM32_ICU_USE_TIM20
  icu_lld_serve_interrupt(&ICUD20);
#endif
#endif
#if HAL_USE_EICU
#if STM32_EICU_USE_TIM20
  eicu_lld_serve_interrupt(&EICUD20);
#endif
#endif
#if HAL_USE_PWM
#if STM32_PWM_USE_TIM20
  pwm_lld_serve_interrupt(&PWMD20);
#endif
#endif
#if 1
#if STM32_ST_USE_TIM20
  st_lld_serve_interrupt();
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   TIM20-CC interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM20_CC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
  /* Not used by GPT.*/
#endif
#if STM32_ICU_USE_TIM20
  icu_lld_serve_interrupt(&ICUD20);
#endif
#if STM32_EICU_USE_TIM20
  eicu_lld_serve_interrupt(&EICUD20);
#endif
#if HAL_USE_PWM
#if STM32_PWM_USE_TIM20
  pwm_lld_serve_interrupt(&PWMD20);
#endif
#endif
#if 1
  /* Not used by ST.*/
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
