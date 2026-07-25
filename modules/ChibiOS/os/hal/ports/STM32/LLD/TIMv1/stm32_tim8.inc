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
 * @file    TIMv1/stm32_tim8.inc
 * @brief   Shared TIM8 handler.
 *
 * @addtogroup STM32_TIM8_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_HAS_TIM8)
#error "STM32_HAS_TIM8 not defined in registry"
#endif

/* Driver checks for robustness, undefined USE macros are defaulted to
   FALSE. This makes this module independent from drivers implementation.*/
#if !defined(STM32_GPT_USE_TIM8)
#define STM32_GPT_USE_TIM8                  FALSE
#endif
#if !defined(STM32_ICU_USE_TIM8)
#define STM32_ICU_USE_TIM8                  FALSE
#endif
#if !defined(STM32_EICU_USE_TIM8)
#define STM32_EICU_USE_TIM8                 FALSE
#endif
#if !defined(STM32_PWM_USE_TIM8)
#define STM32_PWM_USE_TIM8                  FALSE
#endif
#if !defined(STM32_ST_USE_TIM8)
#define STM32_ST_USE_TIM8                   FALSE
#endif

#if STM32_HAS_TIM8

/* Priority settings checks.*/
#if !defined(STM32_IRQ_TIM8_UP_PRIORITY)
#error "STM32_IRQ_TIM8_UP_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(STM32_IRQ_TIM8_CC_PRIORITY)
#error "STM32_IRQ_TIM8_CC_PRIORITY not defined in mcuconf.h"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM8_UP_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM8_UP_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM8_CC_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM8_CC_PRIORITY"
#endif

#endif /* STM32_HAS_TIM8 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void tim8_irq_init(void) {
#if defined(STM32_TIM8_IS_USED)
  nvicEnableVector(STM32_TIM8_UP_NUMBER, STM32_IRQ_TIM8_UP_PRIORITY);
  nvicEnableVector(STM32_TIM8_CC_NUMBER, STM32_IRQ_TIM8_CC_PRIORITY);
#endif
}

static inline void tim8_irq_deinit(void) {
#if defined(STM32_TIM8_IS_USED)
  nvicDisableVector(STM32_TIM8_UP_NUMBER);
  nvicDisableVector(STM32_TIM8_CC_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_TIM8_IS_USED) || defined(__DOXYGEN__)
/**
 * @brief   TIM8-UP interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM8_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
#if STM32_GPT_USE_TIM8
  gpt_lld_serve_interrupt(&GPTD8);
#endif
#endif
#if HAL_USE_ICU
#if STM32_ICU_USE_TIM8
  icu_lld_serve_interrupt(&ICUD8);
#endif
#endif
#if HAL_USE_EICU
#if STM32_EICU_USE_TIM8
  eicu_lld_serve_interrupt(&EICUD8);
#endif
#endif
#if HAL_USE_PWM
#if STM32_PWM_USE_TIM8
  pwm_lld_serve_interrupt(&PWMD8);
#endif
#endif
#if 1
#if STM32_ST_USE_TIM8
  st_lld_serve_interrupt();
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   TIM8-CC interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM8_CC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
  /* Not used by GPT.*/
#endif
#if HAL_USE_ICU
#if STM32_ICU_USE_TIM8
  icu_lld_serve_interrupt(&ICUD8);
#endif
#endif
#if HAL_USE_EICU
#if STM32_EICU_USE_TIM8
  eicu_lld_serve_interrupt(&EICUD8);
#endif
#endif
#if HAL_USE_PWM
#if STM32_PWM_USE_TIM8
  pwm_lld_serve_interrupt(&PWMD8);
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
