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
 * @file    STM32G4xx/stm32_isr.c
 * @brief   STM32G4xx ISR handler code.
 *
 * @addtogroup STM32G4xx_ISR
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

#include "stm32_exti0.inc"
#include "stm32_exti1.inc"
#include "stm32_exti2.inc"
#include "stm32_exti3.inc"
#include "stm32_exti4.inc"
#include "stm32_exti5_9.inc"
#include "stm32_exti10_15.inc"
#include "stm32_exti16-40_41.inc"
#include "stm32_exti17.inc"
#include "stm32_exti18.inc"
#include "stm32_exti19.inc"
#include "stm32_exti20.inc"
#include "stm32_exti21_22-29.inc"
#include "stm32_exti30_32.inc"
#include "stm32_exti33.inc"

#include "stm32_fdcan1.inc"
#include "stm32_fdcan2.inc"
#include "stm32_fdcan3.inc"

#include "stm32_i2c1.inc"
#include "stm32_i2c2.inc"
#include "stm32_i2c3.inc"
#include "stm32_i2c4.inc"

#include "stm32_usart1.inc"
#include "stm32_usart2.inc"
#include "stm32_usart3.inc"
#include "stm32_uart4.inc"
#include "stm32_uart5.inc"
#include "stm32_lpuart1.inc"

#include "stm32_tim1_15_16_17.inc"
#include "stm32_tim2.inc"
#include "stm32_tim3.inc"
#include "stm32_tim4.inc"
#include "stm32_tim5.inc"
#include "stm32_tim6.inc"
#include "stm32_tim7.inc"
#include "stm32_tim8.inc"
#include "stm32_tim20.inc"

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
  exti16_exti40_exti41_irq_init();
  exti17_irq_init();
  exti18_irq_init();
  exti19_irq_init();
  exti21_exti22_exti29_irq_init();
  exti30_32_irq_init();
  exti33_irq_init();

  fdcan1_irq_init();
  fdcan2_irq_init();
  fdcan3_irq_init();

  i2c1_irq_init();
  i2c2_irq_init();
  i2c3_irq_init();
  i2c4_irq_init();

  tim1_tim15_tim16_tim17_irq_init();
  tim2_irq_init();
  tim3_irq_init();
  tim4_irq_init();
  tim5_irq_init();
  tim6_irq_init();
  tim7_irq_init();
  tim8_irq_init();
  tim20_irq_init();

  usart1_irq_init();
  usart2_irq_init();
  usart3_irq_init();
  uart4_irq_init();
  uart5_irq_init();
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
  exti16_exti40_exti41_irq_deinit();
  exti17_irq_deinit();
  exti18_irq_deinit();
  exti19_irq_deinit();
  exti21_exti22_exti29_irq_deinit();
  exti30_32_irq_deinit();
  exti33_irq_deinit();

  fdcan1_irq_deinit();
  fdcan2_irq_deinit();
  fdcan3_irq_deinit();

  i2c1_irq_deinit();
  i2c2_irq_deinit();
  i2c3_irq_deinit();
  i2c4_irq_deinit();

  tim1_tim15_tim16_tim17_irq_deinit();
  tim2_irq_deinit();
  tim3_irq_deinit();
  tim4_irq_deinit();
  tim5_irq_deinit();
  tim6_irq_deinit();
  tim7_irq_deinit();
  tim8_irq_deinit();
  tim20_irq_deinit();

  usart1_irq_deinit();
  usart2_irq_deinit();
  usart3_irq_deinit();
  uart4_irq_deinit();
  uart5_irq_deinit();
  lpuart1_irq_deinit();
}

/** @} */
