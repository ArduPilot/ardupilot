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
 * @file    TIMv1/stm32_tim3_4.inc
 * @brief   Shared TIM3, TIM4 handler.
 *
 * @addtogroup STM32_TIM3_4_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_HAS_TIM3)
#error "STM32_HAS_TIM3 not defined in registry"
#endif

#if !defined(STM32_HAS_TIM4)
#error "STM32_HAS_TIM4 not defined in registry"
#endif

/* HW compatibility check.*/
#if !STM32_HAS_TIM3
#error "TIM3 not present"
#endif

#if !STM32_HAS_TIM4
#error "TIM4 not present"
#endif

/* Driver checks for robustness, undefined USE macros are defaulted to
   FALSE. This makes this module independent from drivers implementation.*/
#if !defined(STM32_GPT_USE_TIM3)
#define STM32_GPT_USE_TIM3                  FALSE
#endif
#if !defined(STM32_ICU_USE_TIM3)
#define STM32_ICU_USE_TIM3                  FALSE
#endif
#if !defined(STM32_EICU_USE_TIM3)
#define STM32_EICU_USE_TIM3                 FALSE
#endif
#if !defined(STM32_PWM_USE_TIM3)
#define STM32_PWM_USE_TIM3                  FALSE
#endif
#if !defined(STM32_ST_USE_TIM3)
#define STM32_ST_USE_TIM3                   FALSE
#endif

#if !defined(STM32_GPT_USE_TIM4)
#define STM32_GPT_USE_TIM4                  FALSE
#endif
#if !defined(STM32_ICU_USE_TIM4)
#define STM32_ICU_USE_TIM4                  FALSE
#endif
#if !defined(STM32_EICU_USE_TIM4)
#define STM32_EICU_USE_TIM4                 FALSE
#endif
#if !defined(STM32_PWM_USE_TIM4)
#define STM32_PWM_USE_TIM4                  FALSE
#endif
#if !defined(STM32_ST_USE_TIM4)
#define STM32_ST_USE_TIM4                   FALSE
#endif

#if STM32_HAS_TIM3 || STM32_HAS_TIM4

/* Priority settings checks.*/
#if !defined(STM32_IRQ_TIM3_4_PRIORITY)
#error "STM32_IRQ_TIM3_4_PRIORITY not defined in mcuconf.h"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM3_4_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM3_4_PRIORITY"
#endif

#endif /* STM32_HAS_TIM3 || STM32_HAS_TIM4 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void tim3_tim4_irq_init(void) {

#if defined(STM32_TIM3_IS_USED) || defined(STM32_TIM4_IS_USED)
  nvicEnableVector(STM32_TIM3_4_NUMBER,
                   STM32_IRQ_TIM3_4_PRIORITY);
#endif
}

static inline void tim3_tim4_irq_deinit(void) {

#if defined(STM32_TIM3_IS_USED) || defined(STM32_TIM4_IS_USED)
  nvicDisableVector(STM32_TIM3_4_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_TIM3_IS_USED) || defined(STM32_TIM4_IS_USED)  ||          \
    defined(__DOXYGEN__)
/**
 * @brief   TIM3, TIM4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM3_4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
#if STM32_GPT_USE_TIM3
  gpt_lld_serve_interrupt(&GPTD3);
#endif
#if STM32_GPT_USE_TIM4
  gpt_lld_serve_interrupt(&GPTD4);
#endif
#endif
#if HAL_USE_ICU
#if STM32_ICU_USE_TIM3
  icu_lld_serve_interrupt(&ICUD3);
#endif
#if STM32_ICU_USE_TIM4
  icu_lld_serve_interrupt(&ICUD4);
#endif
#endif
#if HAL_USE_EICU
#if STM32_EICU_USE_TIM3
  eicu_lld_serve_interrupt(&EICUD3);
#endif
#if STM32_EICU_USE_TIM4
  eicu_lld_serve_interrupt(&EICUD4);
#endif
#endif
#if HAL_USE_PWM
#if STM32_PWM_USE_TIM3
  pwm_lld_serve_interrupt(&PWMD3);
#endif
#if STM32_PWM_USE_TIM4
  pwm_lld_serve_interrupt(&PWMD4);
#endif
#endif
#if 1
#if STM32_ST_USE_TIM3
  st_lld_serve_interrupt();
#endif
#if STM32_ST_USE_TIM4
  st_lld_serve_interrupt();
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
