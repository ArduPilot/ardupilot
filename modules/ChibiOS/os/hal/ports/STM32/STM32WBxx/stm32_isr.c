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
/*
    Concepts and parts of this file have been contributed by Ilya Kharin.
*/

/**
 * @file    STM32WBxx/stm32_isr.h
 * @brief   STM32WBxx ISR handler code.
 *
 * @addtogroup STM32WBxx_ISR
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

#include "stm32_exti0.inc"
#include "stm32_exti1.inc"
#include "stm32_exti2.inc"
#include "stm32_exti3.inc"
#include "stm32_exti4.inc"
#include "stm32_exti5_9.inc"
#include "stm32_exti10_15.inc"
#include "stm32_exti16-31-33.inc"
#include "stm32_exti17.inc"
#include "stm32_exti18.inc"
#include "stm32_exti19.inc"
#include "stm32_exti20_21.inc"

#include "stm32_usart1.inc"
#include "stm32_lpuart1.inc"

#include "stm32_tim1_16_17.inc"
#include "stm32_tim2.inc"

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Enables IRQ sources.
 *
 * @notapi
 */
void irqInit(void) {

  exti0_irq_init();
  exti1_irq_init();
  exti2_irq_init();
  exti3_irq_init();
  exti4_irq_init();
  exti5_9_irq_init();
  exti10_15_irq_init();
  exti16_exti31_exti33_irq_init();
  exti17_irq_init();
  exti18_irq_init();
  exti19_irq_init();
  exti20_exti21_irq_init();

  tim1_tim16_tim17_irq_init();
  tim2_irq_init();

  usart1_irq_init();
  lpuart1_irq_init();
}

/**
 * @brief   Disables IRQ sources.
 *
 * @notapi
 */
void irqDeinit(void) {

  exti0_irq_deinit();
  exti1_irq_deinit();
  exti2_irq_deinit();
  exti3_irq_deinit();
  exti4_irq_deinit();
  exti5_9_irq_deinit();
  exti10_15_irq_deinit();
  exti16_exti31_exti33_irq_deinit();
  exti17_irq_deinit();
  exti18_irq_deinit();
  exti19_irq_deinit();
  exti20_exti21_irq_deinit();

  tim1_tim16_tim17_irq_deinit();
  tim2_irq_deinit();

  usart1_irq_deinit();
  lpuart1_irq_deinit();
}

/** @} */
