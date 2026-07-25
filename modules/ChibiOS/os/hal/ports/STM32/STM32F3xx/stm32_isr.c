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
 * @file    STM32F3xx/stm32_isr.c
 * @brief   STM32F3xx ISR handler code.
 *
 * @addtogroup STM32F3xx_ISR
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
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

#define exti_serve_irq(pr, channel) {                                       \
                                                                            \
  if ((pr) & (1U << (channel))) {                                           \
    _pal_isr_code(channel);                                                 \
  }                                                                         \
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (HAL_USE_PAL && (PAL_USE_WAIT || PAL_USE_CALLBACKS)) || defined(__DOXYGEN__)
#if !defined(STM32_DISABLE_EXTI0_HANDLER)
/**
 * @brief   EXTI[0] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector58) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & (1U << 0);
  EXTI->PR = pr;

  exti_serve_irq(pr, 0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI1_HANDLER)
/**
 * @brief   EXTI[1] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector5C) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & (1U << 1);
  EXTI->PR = pr;

  exti_serve_irq(pr, 1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI2_HANDLER)
/**
 * @brief   EXTI[2] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector60) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & (1U << 2);
  EXTI->PR = pr;

  exti_serve_irq(pr, 2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI3_HANDLER)
/**
 * @brief   EXTI[3] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector64) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & (1U << 3);
  EXTI->PR = pr;

  exti_serve_irq(pr, 3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI4_HANDLER)
/**
 * @brief   EXTI[4] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector68) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & (1U << 4);
  EXTI->PR = pr;

  exti_serve_irq(pr, 4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI5_9_HANDLER)
/**
 * @brief   EXTI[5]...EXTI[9] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector9C) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & ((1U << 5) | (1U << 6) | (1U << 7) | (1U << 8) |
                     (1U << 9));
  EXTI->PR = pr;

  exti_serve_irq(pr, 5);
  exti_serve_irq(pr, 6);
  exti_serve_irq(pr, 7);
  exti_serve_irq(pr, 8);
  exti_serve_irq(pr, 9);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI10_15_HANDLER)
/**
 * @brief   EXTI[10]...EXTI[15] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(VectorE0) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & ((1U << 10) | (1U << 11) | (1U << 12) | (1U << 13) |
                     (1U << 14) | (1U << 15));
  EXTI->PR = pr;

  exti_serve_irq(pr, 10);
  exti_serve_irq(pr, 11);
  exti_serve_irq(pr, 12);
  exti_serve_irq(pr, 13);
  exti_serve_irq(pr, 14);
  exti_serve_irq(pr, 15);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif /* HAL_USE_PAL && (PAL_USE_WAIT || PAL_USE_CALLBACKS) */

#if HAL_USE_GPT || HAL_USE_ICU || HAL_USE_PWM || defined(__DOXYGEN__)
/**
 * @brief   TIM1-BRK, TIM15 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(VectorA0) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
#if STM32_GPT_USE_TIM15
  gpt_lld_serve_interrupt(&GPTD15);
#endif
#endif
#if HAL_USE_ICU
#if STM32_ICU_USE_TIM15
  icu_lld_serve_interrupt(&ICUD15);
#endif
#endif
#if HAL_USE_PWM
#if STM32_PWM_USE_TIM15
  pwm_lld_serve_interrupt(&PWMD15);
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   TIM1-UP, TIM16 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(VectorA4) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
#if STM32_GPT_USE_TIM1
  gpt_lld_serve_interrupt(&GPTD1);
#endif
#if STM32_GPT_USE_TIM16
  gpt_lld_serve_interrupt(&GPTD16);
#endif
#endif
#if HAL_USE_ICU
#if STM32_ICU_USE_TIM1
  icu_lld_serve_interrupt(&ICUD1);
#endif
#endif
#if HAL_USE_PWM
#if STM32_PWM_USE_TIM1
  pwm_lld_serve_interrupt(&PWMD1);
#endif
#if STM32_PWM_USE_TIM16
  pwm_lld_serve_interrupt(&PWMD16);
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   TIM1-TRG-COM, TIM17 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(VectorA8) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
#if STM32_GPT_USE_TIM17
  gpt_lld_serve_interrupt(&GPTD17);
#endif
#endif
#if HAL_USE_ICU
  /* Not used by ICU.*/
#endif
#if HAL_USE_PWM
#if STM32_PWM_USE_TIM17
  pwm_lld_serve_interrupt(&PWMD17);
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   TIM1-CC interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(VectorAC) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
  /* Not used by GPT.*/
#endif
#if HAL_USE_ICU
#if STM32_ICU_USE_TIM1
  icu_lld_serve_interrupt(&ICUD1);
#endif
#endif
#if HAL_USE_PWM
#if STM32_PWM_USE_TIM1
  pwm_lld_serve_interrupt(&PWMD1);
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif /* HAL_USE_GPT || HAL_USE_ICU || HAL_USE_PWM */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Enables IRQ sources.
 *
 * @notapi
 */
void irqInit(void) {

#if HAL_USE_PAL
  nvicEnableVector(EXTI0_IRQn,              STM32_IRQ_EXTI0_PRIORITY);
  nvicEnableVector(EXTI1_IRQn,              STM32_IRQ_EXTI1_PRIORITY);
  nvicEnableVector(EXTI2_TSC_IRQn,          STM32_IRQ_EXTI2_PRIORITY);
  nvicEnableVector(EXTI3_IRQn,              STM32_IRQ_EXTI3_PRIORITY);
  nvicEnableVector(EXTI4_IRQn,              STM32_IRQ_EXTI4_PRIORITY);
  nvicEnableVector(EXTI9_5_IRQn,            STM32_IRQ_EXTI5_9_PRIORITY);
  nvicEnableVector(EXTI15_10_IRQn,          STM32_IRQ_EXTI10_15_PRIORITY);
#endif
#if HAL_USE_GPT || HAL_USE_ICU || HAL_USE_PWM || defined(__DOXYGEN__)
  nvicEnableVector(TIM1_BRK_TIM15_IRQn,     STM32_IRQ_TIM1_BRK_TIM15_PRIORITY);
  nvicEnableVector(TIM1_UP_TIM16_IRQn,      STM32_IRQ_TIM1_UP_TIM16_PRIORITY);
  nvicEnableVector(TIM1_TRG_COM_TIM17_IRQn, STM32_IRQ_TIM1_TRGCO_TIM17_PRIORITY);
  nvicEnableVector(TIM1_CC_IRQn,            STM32_IRQ_TIM1_CC_PRIORITY);
#endif
}

/**
 * @brief   Disables IRQ sources.
 *
 * @notapi
 */
void irqDeinit(void) {

#if HAL_USE_PAL
  nvicDisableVector(EXTI0_IRQn);
  nvicDisableVector(EXTI1_IRQn);
  nvicDisableVector(EXTI2_TSC_IRQn);
  nvicDisableVector(EXTI3_IRQn);
  nvicDisableVector(EXTI4_IRQn);
  nvicDisableVector(EXTI9_5_IRQn);
  nvicDisableVector(EXTI15_10_IRQn);
#endif
#if HAL_USE_GPT || HAL_USE_ICU || HAL_USE_PWM || defined(__DOXYGEN__)
  nvicDisableVector(TIM1_BRK_TIM15_IRQn);
  nvicDisableVector(TIM1_UP_TIM16_IRQn);
  nvicDisableVector(TIM1_TRG_COM_TIM17_IRQn);
  nvicDisableVector(TIM1_CC_IRQn);
#endif
}

/** @} */
