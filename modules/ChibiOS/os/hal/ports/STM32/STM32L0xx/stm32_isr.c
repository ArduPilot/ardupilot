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
 * @file    STM32L0xx/stm32_isr.c
 * @brief   STM32L0xx ISR handler code.
 *
 * @addtogroup SRM32L0xx_ISR
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define exti_serve_irq(pr, channel) {                                       \
                                                                            \
  if ((pr) & (1U << (channel))) {                                           \
    _pal_isr_code(channel);                                                 \
  }                                                                         \
}

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#include "stm32_dma1_ch23.inc"
#include "stm32_dma1_ch4567.inc"

#include "stm32_exti0_1.inc"
#include "stm32_exti2_3.inc"
#include "stm32_exti4_15.inc"

#include "stm32_usart1.inc"
#include "stm32_usart2.inc"
#include "stm32_usart4_5.inc"
#include "stm32_lpuart1.inc"

#include "stm32_tim2.inc"
#include "stm32_tim3.inc"
#include "stm32_tim6.inc"
#include "stm32_tim7.inc"
#include "stm32_tim21.inc"
#include "stm32_tim22.inc"

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Enables IRQ sources.
 *
 * @notapi
 */
void irqInit(void) {

  exti0_1_irq_init();
  exti2_3_irq_init();
  exti4_15_irq_init();

  tim2_irq_init();
  tim3_irq_init();
  tim6_irq_init();
  tim7_irq_init();
  tim21_irq_init();
  tim22_irq_init();

  usart1_irq_init();
  usart2_irq_init();
  usart4_usart5_irq_init();
  lpuart1_irq_init();
}

/**
 * @brief   Disables IRQ sources.
 *
 * @notapi
 */
void irqDeinit(void) {

  exti0_1_irq_deinit();
  exti2_3_irq_deinit();
  exti4_15_irq_deinit();

  tim2_irq_deinit();
  tim3_irq_deinit();
  tim6_irq_deinit();
  tim7_irq_deinit();
  tim21_irq_deinit();
  tim22_irq_deinit();

  usart1_irq_deinit();
  usart2_irq_deinit();
  usart4_usart5_irq_deinit();
  lpuart1_irq_deinit();
}

/** @} */
