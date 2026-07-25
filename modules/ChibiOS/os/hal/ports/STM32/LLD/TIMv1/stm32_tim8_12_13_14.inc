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
 * @file    TIMv1/stm32_tim8_12_13_14.inc
 * @brief   Shared TIM8, TIM12, TIM13, TIM14 handler.
 *
 * @addtogroup STM32_TIM8_TIM12_TIM13_TIM14_HANDLER
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

#if !defined(STM32_HAS_TIM12)
#error "STM32_HAS_TIM12 not defined in registry"
#endif

#if !defined(STM32_HAS_TIM13)
#error "STM32_HAS_TIM13 not defined in registry"
#endif

#if !defined(STM32_HAS_TIM14)
#error "STM32_HAS_TIM14 not defined in registry"
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

#if !defined(STM32_GPT_USE_TIM12)
#define STM32_GPT_USE_TIM12                 FALSE
#endif
#if !defined(STM32_ICU_USE_TIM12)
#define STM32_ICU_USE_TIM12                 FALSE
#endif
#if !defined(STM32_EICU_USE_TIM12)
#define STM32_EICU_USE_TIM12                FALSE
#endif
#if !defined(STM32_PWM_USE_TIM12)
#define STM32_PWM_USE_TIM12                 FALSE
#endif
#if !defined(STM32_ST_USE_TIM12)
#define STM32_ST_USE_TIM12                  FALSE
#endif

#if !defined(STM32_GPT_USE_TIM13)
#define STM32_GPT_USE_TIM13                 FALSE
#endif
#if !defined(STM32_ICU_USE_TIM13)
#define STM32_ICU_USE_TIM13                 FALSE
#endif
#if !defined(STM32_EICU_USE_TIM13)
#define STM32_EICU_USE_TIM13                FALSE
#endif
#if !defined(STM32_PWM_USE_TIM13)
#define STM32_PWM_USE_TIM13                 FALSE
#endif
#if !defined(STM32_ST_USE_TIM13)
#define STM32_ST_USE_TIM13                  FALSE
#endif

#if !defined(STM32_GPT_USE_TIM14)
#define STM32_GPT_USE_TIM14                 FALSE
#endif
#if !defined(STM32_ICU_USE_TIM14)
#define STM32_ICU_USE_TIM14                 FALSE
#endif
#if !defined(STM32_EICU_USE_TIM14)
#define STM32_EICU_USE_TIM14                FALSE
#endif
#if !defined(STM32_PWM_USE_TIM14)
#define STM32_PWM_USE_TIM14                 FALSE
#endif
#if !defined(STM32_ST_USE_TIM14)
#define STM32_ST_USE_TIM14                  FALSE
#endif

#if STM32_HAS_TIM8 || STM32_HAS_TIM12 || STM32_HAS_TIM13 || STM32_HAS_TIM14

/* Priority settings checks.*/
#if !defined(STM32_IRQ_TIM8_BRK_TIM12_PRIORITY)
#error "STM32_IRQ_TIM8_BRK_TIM12_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(STM32_IRQ_TIM8_UP_TIM13_PRIORITY)
#error "STM32_IRQ_TIM8_UP_TIM13_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY)
#error "STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(STM32_IRQ_TIM8_CC_PRIORITY)
#error "STM32_IRQ_TIM8_CC_PRIORITY not defined in mcuconf.h"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM8_BRK_TIM12_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM8_BRK_TIM12_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM8_UP_TIM13_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM8_UP_TIM13_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM8_CC_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM8_CC_PRIORITY"
#endif

#endif /* STM32_HAS_TIM8 || STM32_HAS_TIM12 || STM32_HAS_TIM13 || STM32_HAS_TIM14 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void tim8_tim12_tim13_tim14_irq_init(void) {
#if defined(STM32_TIM8_IS_USED) || defined(STM32_TIM12_IS_USED)
  nvicEnableVector(STM32_TIM8_BRK_TIM12_NUMBER,
                   STM32_IRQ_TIM8_BRK_TIM12_PRIORITY);
#endif
#if defined(STM32_TIM8_IS_USED) || defined(STM32_TIM13_IS_USED)
  nvicEnableVector(STM32_TIM8_UP_TIM13_NUMBER,
                   STM32_IRQ_TIM8_UP_TIM13_PRIORITY);
#endif
#if defined(STM32_TIM8_IS_USED) || defined(STM32_TIM14_IS_USED)
  nvicEnableVector(STM32_TIM8_TRGCO_TIM14_NUMBER,
                   STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY);
#endif
#if defined(STM32_TIM8_IS_USED)
  nvicEnableVector(STM32_TIM8_CC_NUMBER,
                   STM32_IRQ_TIM8_CC_PRIORITY);
#endif
}

static inline void tim8_tim12_tim13_tim14_irq_deinit(void) {
#if defined(STM32_TIM8_IS_USED) || defined(STM32_TIM12_IS_USED)
  nvicDisableVector(STM32_TIM8_BRK_TIM12_NUMBER);
#endif
#if defined(STM32_TIM8_IS_USED) || defined(STM32_TIM13_IS_USED)
  nvicDisableVector(STM32_TIM8_UP_TIM13_NUMBER);
#endif
#if defined(STM32_TIM8_IS_USED) || defined(STM32_TIM14_IS_USED)
  nvicDisableVector(STM32_TIM8_TRGCO_TIM14_NUMBER);
#endif
#if defined(STM32_TIM8_IS_USED)
  nvicDisableVector(STM32_TIM8_CC_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_TIM8_IS_USED) || defined(STM32_TIM12_IS_USED)  ||         \
    defined(__DOXYGEN__)
/**
 * @brief   TIM8-BRK, TIM12 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM8_BRK_TIM12_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
#if STM32_GPT_USE_TIM12
  gpt_lld_serve_interrupt(&GPTD12);
#endif
#endif
#if HAL_USE_ICU
#if STM32_ICU_USE_TIM12
  icu_lld_serve_interrupt(&ICUD12);
#endif
#endif
#if HAL_USE_EICU
#if STM32_EICU_USE_TIM12
  eicu_lld_serve_interrupt(&EICUD12);
#endif
#endif
#if HAL_USE_PWM
#if STM32_PWM_USE_TIM12
  pwm_lld_serve_interrupt(&PWMD12);
#endif
#endif
#if 1
#if STM32_ST_USE_TIM12
  st_lld_serve_interrupt();
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_TIM8_IS_USED) || defined(STM32_TIM13_IS_USED)  ||         \
    defined(__DOXYGEN__)
/**
 * @brief   TIM8-UP, TIM13 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM8_UP_TIM13_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
#if STM32_GPT_USE_TIM8
  gpt_lld_serve_interrupt(&GPTD8);
#endif
#if STM32_GPT_USE_TIM13
  gpt_lld_serve_interrupt(&GPTD13);
#endif
#endif
#if HAL_USE_ICU
#if STM32_ICU_USE_TIM8
  icu_lld_serve_interrupt(&ICUD8);
#endif
#if STM32_ICU_USE_TIM13
  icu_lld_serve_interrupt(&ICUD13);
#endif
#endif
#if HAL_USE_EICU
#if STM32_EICU_USE_TIM8
  eicu_lld_serve_interrupt(&EICUD8);
#endif
#if STM32_EICU_USE_TIM13
  eicu_lld_serve_interrupt(&EICUD13);
#endif
#endif
#if HAL_USE_PWM
#if STM32_PWM_USE_TIM8
  pwm_lld_serve_interrupt(&PWMD8);
#endif
#if STM32_PWM_USE_TIM13
  pwm_lld_serve_interrupt(&PWMD13);
#endif
#endif
#if 1
#if STM32_ST_USE_TIM8
  st_lld_serve_interrupt();
#endif
#if STM32_ST_USE_TIM13
  st_lld_serve_interrupt();
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_TIM8_IS_USED) || defined(STM32_TIM14_IS_USED)  ||         \
    defined(__DOXYGEN__)
/**
 * @brief   TIM8-TRG-COM, TIM14 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM8_TRGCO_TIM14_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
#if STM32_GPT_USE_TIM14
  gpt_lld_serve_interrupt(&GPTD14);
#endif
#endif
#if HAL_USE_ICU
#if STM32_ICU_USE_TIM14
  icu_lld_serve_interrupt(&ICUD14);
#endif
#endif
#if HAL_USE_EICU
#if STM32_EICU_USE_TIM14
  eicu_lld_serve_interrupt(&EICUD14);
#endif
#endif
#if HAL_USE_PWM
#if STM32_PWM_USE_TIM14
  pwm_lld_serve_interrupt(&PWMD14);
#endif
#endif
#if 1
#if STM32_ST_USE_TIM14
  st_lld_serve_interrupt();
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_TIM8_IS_USED) || defined(__DOXYGEN__)
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

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
