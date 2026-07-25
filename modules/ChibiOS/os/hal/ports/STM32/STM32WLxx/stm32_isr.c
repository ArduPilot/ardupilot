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
 * @file    STM32WLxx/stm32_isr.c
 * @brief   STM32WLxx ISR handler code.
 *
 * @addtogroup STM32WLxx_ISR
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

#if STM32_TARGET_CORE == 1

#include "stm32_exti0.inc"
#include "stm32_exti1.inc"
#include "stm32_exti2.inc"
#include "stm32_exti3.inc"
#include "stm32_exti4.inc"
#include "stm32_exti5_9.inc"
#include "stm32_exti10_15.inc"
#include "stm32_exti16_34.inc"
#include "stm32_exti45.inc"

#include "stm32_tim1.inc"

#else /* STM32_TARGET_CORE == 2 */

#include "stm32_dma1_ch123.inc"
#include "stm32_dma1_ch4567.inc"
#include "stm32_dma2_ch1-7.inc"

#include "stm32_exti0_1.inc"
#include "stm32_exti2_3.inc"
#include "stm32_exti4_15.inc"
#include "stm32_exti16_34.inc"
#include "stm32_exti45.inc"

#include "stm32_tim1_shared.inc"

#endif /* STM32_TARGET_CORE == 1 */

#include "stm32_usart1.inc"
#include "stm32_usart2.inc"
#include "stm32_lpuart1.inc"

#include "stm32_tim2.inc"
#include "stm32_tim16.inc"
#include "stm32_tim17.inc"

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Enables IRQ sources.
 *
 * @notapi
 */
void irqInit(void) {

#if STM32_TARGET_CORE == 1
  exti0_irq_init();
  exti1_irq_init();
  exti2_irq_init();
  exti3_irq_init();
  exti4_irq_init();
  exti5_9_irq_init();
  exti10_15_irq_init();
  exti16_exti34_irq_init();
  exti45_irq_init();
#else
  exti0_1_irq_init();
  exti2_3_irq_init();
  exti4_15_irq_init();
  exti16_exti34_irq_init();
  exti45_irq_init();
#endif /* STM32_TARGET_CORE == 1 */

  tim1_irq_init();
  tim2_irq_init();
  tim16_irq_init();
  tim17_irq_init();

  usart1_irq_init();
  usart2_irq_init();
  lpuart1_irq_init();
}

/**
 * @brief   Disables IRQ sources.
 *
 * @notapi
 */
void irqDeinit(void) {

#if STM32_TARGET_CORE == 1
  exti0_irq_deinit();
  exti1_irq_deinit();
  exti2_irq_deinit();
  exti3_irq_deinit();
  exti4_irq_deinit();
  exti5_9_irq_deinit();
  exti10_15_irq_deinit();
  exti16_exti34_irq_deinit();
  exti45_irq_deinit();
#else
  exti0_1_irq_deinit();
  exti2_3_irq_deinit();
  exti4_15_irq_deinit();
#endif /* STM32_TARGET_CORE == 1 */

  tim1_irq_deinit();
  tim2_irq_deinit();
  tim16_irq_deinit();
  tim17_irq_deinit();

  usart1_irq_deinit();
  usart2_irq_deinit();
  lpuart1_irq_deinit();
}

/** @} */
