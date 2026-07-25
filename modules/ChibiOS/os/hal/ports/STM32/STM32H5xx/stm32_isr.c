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
 * @file    STM32H5xx/stm32_isr.c
 * @brief   STM32H5xx ISR handler code.
 *
 * @addtogroup STM32H5xx_ISR
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

#include "stm32_dac1.inc"

#include "stm32_exti0.inc"
#include "stm32_exti1.inc"
#include "stm32_exti2.inc"
#include "stm32_exti3.inc"
#include "stm32_exti4.inc"
#include "stm32_exti5.inc"
#include "stm32_exti6.inc"
#include "stm32_exti7.inc"
#include "stm32_exti8.inc"
#include "stm32_exti9.inc"
#include "stm32_exti10.inc"
#include "stm32_exti11.inc"
#include "stm32_exti12.inc"
#include "stm32_exti13.inc"
#include "stm32_exti14.inc"
#include "stm32_exti15.inc"

#include "stm32_fdcan1.inc"
#include "stm32_fdcan2.inc"

#include "stm32_i2c1.inc"
#include "stm32_i2c2.inc"
#include "stm32_i2c3.inc"
#include "stm32_i2c4.inc"

#include "stm32_spi1.inc"
#include "stm32_spi2.inc"
#include "stm32_spi3.inc"
#include "stm32_spi4.inc"
#include "stm32_spi5.inc"
#include "stm32_spi6.inc"

#include "stm32_tim1.inc"
#include "stm32_tim2.inc"
#include "stm32_tim3.inc"
#include "stm32_tim4.inc"
#include "stm32_tim5.inc"
#include "stm32_tim6.inc"
#include "stm32_tim7.inc"
#include "stm32_tim8.inc"
#include "stm32_tim12.inc"
#include "stm32_tim13.inc"
#include "stm32_tim14.inc"
#include "stm32_tim15.inc"
#include "stm32_tim16.inc"
#include "stm32_tim17.inc"

#include "stm32_usart1.inc"
#include "stm32_usart2.inc"
#include "stm32_usart3.inc"
#include "stm32_uart4.inc"
#include "stm32_uart5.inc"
#include "stm32_usart6.inc"
#include "stm32_uart7.inc"
#include "stm32_uart8.inc"
#include "stm32_uart9.inc"
#include "stm32_usart10.inc"
#include "stm32_usart11.inc"
#include "stm32_uart12.inc"
#include "stm32_lpuart1.inc"

#include "stm32_usb1.inc"

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Enables IRQ sources.
 *
 * @notapi
 */
void irqInit(void) {

  dac1_irq_init();

  exti0_irq_init();
  exti1_irq_init();
  exti2_irq_init();
  exti3_irq_init();
  exti4_irq_init();
  exti5_irq_init();
  exti6_irq_init();
  exti7_irq_init();
  exti8_irq_init();
  exti9_irq_init();
  exti10_irq_init();
  exti11_irq_init();
  exti12_irq_init();
  exti13_irq_init();
  exti14_irq_init();
  exti15_irq_init();

  fdcan1_irq_init();
  fdcan2_irq_init();

  i2c1_irq_init();
  i2c2_irq_init();
  i2c3_irq_init();
  i2c4_irq_init();

  spi1_irq_init();
  spi2_irq_init();
  spi3_irq_init();
  spi4_irq_init();
  spi5_irq_init();
  spi6_irq_init();

  tim1_irq_init();
  tim2_irq_init();
  tim3_irq_init();
  tim4_irq_init();
  tim5_irq_init();
  tim6_irq_init();
  tim7_irq_init();
  tim8_irq_init();
  tim12_irq_init();
  tim13_irq_init();
  tim14_irq_init();
  tim15_irq_init();
  tim16_irq_init();
  tim17_irq_init();

  usart1_irq_init();
  usart2_irq_init();
  usart3_irq_init();
  uart4_irq_init();
  uart5_irq_init();
  usart6_irq_init();
  uart7_irq_init();
  uart8_irq_init();
  uart9_irq_init();
  usart10_irq_init();
  usart11_irq_init();
  uart12_irq_init();
  lpuart1_irq_init();

  usb1_irq_init();
}

/**
 * @brief   Disables IRQ sources.
 *
 * @notapi
 */
void irqDeinit(void) {

  dac1_irq_deinit();

  exti0_irq_deinit();
  exti1_irq_deinit();
  exti2_irq_deinit();
  exti3_irq_deinit();
  exti4_irq_deinit();
  exti5_irq_deinit();
  exti6_irq_deinit();
  exti7_irq_deinit();
  exti8_irq_deinit();
  exti9_irq_deinit();
  exti10_irq_deinit();
  exti11_irq_deinit();
  exti12_irq_deinit();
  exti13_irq_deinit();
  exti14_irq_deinit();
  exti15_irq_deinit();

  fdcan1_irq_deinit();
  fdcan2_irq_deinit();

  i2c1_irq_deinit();
  i2c2_irq_deinit();
  i2c3_irq_deinit();
  i2c4_irq_deinit();

  spi1_irq_deinit();
  spi2_irq_deinit();
  spi3_irq_deinit();
  spi4_irq_deinit();
  spi5_irq_deinit();
  spi6_irq_deinit();

  tim1_irq_deinit();
  tim2_irq_deinit();
  tim3_irq_deinit();
  tim4_irq_deinit();
  tim5_irq_deinit();
  tim6_irq_deinit();
  tim7_irq_deinit();
  tim8_irq_deinit();
  tim12_irq_deinit();
  tim13_irq_deinit();
  tim14_irq_deinit();
  tim15_irq_deinit();
  tim16_irq_deinit();
  tim17_irq_deinit();

  usart1_irq_deinit();
  usart2_irq_deinit();
  usart3_irq_deinit();
  uart4_irq_deinit();
  uart5_irq_deinit();
  usart6_irq_deinit();
  uart7_irq_deinit();
  uart8_irq_deinit();
  uart9_irq_deinit();
  usart10_irq_deinit();
  usart11_irq_deinit();
  uart12_irq_deinit();
  lpuart1_irq_deinit();

  usb1_irq_deinit();
}

/** @} */
