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
 * @file    STM32G0xx/stm32_isr.c
 * @brief   STM32G0xx ISR handler code.
 *
 * @addtogroup STM32G0xx_ISR
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
#if STM32_DMA2_NUM_CHANNELS > 0
#include "stm32_dma1_ch4567_dma2_ch12345.inc"
#else
#include "stm32_dma1_ch4567.inc"
#endif

#include "stm32_exti0_1.inc"
#include "stm32_exti2_3.inc"
#include "stm32_exti4_15.inc"
#include "stm32_exti19-21.inc"

#include "stm32_i2c1.inc"
#if STM32_HAS_I2C2 && STM32_HAS_I2C3
#include "stm32_i2c2_3.inc"
#elif STM32_HAS_I2C2
#include "stm32_i2c2.inc"
#else
#error "unknown I2Cs combination"
#endif

#include "stm32_tim1.inc"
#include "stm32_tim2.inc"
#if STM32_HAS_TIM3 && STM32_HAS_TIM4
#include "stm32_tim3_4.inc"
#elif STM32_HAS_TIM3
#include "stm32_tim3.inc"
#else
#error "unknown TIMs combination"
#endif
#include "stm32_tim6.inc"
#include "stm32_tim7.inc"
#include "stm32_tim14.inc"
#include "stm32_tim15.inc"
#include "stm32_tim16.inc"
#include "stm32_tim17.inc"

#include "stm32_usart1.inc"
#if STM32_HAS_USART2 && STM32_HAS_LPUART2
#include "stm32_usart2_lp2.inc"
#elif STM32_HAS_USART2
#include "stm32_usart2.inc"
#else
#error "unknown USARTs combination"
#endif
#if STM32_HAS_USART3 && STM32_HAS_UART4 && STM32_HAS_UART5 && STM32_HAS_USART6 && STM32_HAS_LPUART1
#include "stm32_usart3_4_5_6_lp1.inc"
#elif STM32_HAS_USART3 && STM32_HAS_UART4 && STM32_HAS_UART5 && STM32_HAS_USART6
#include "stm32_usart3_4_5_6.inc"
#elif STM32_HAS_USART3 && STM32_HAS_UART4 && STM32_HAS_LPUART1
#include "stm32_usart3_4_lp1.inc"
#elif STM32_HAS_LPUART1
#include "stm32_lpuart1.inc"
#else
#error "unknown USARTs combination"
#endif

#if STM32_HAS_UCPD1 && STM32_HAS_UCPD2
#include "stm32_usb1_ucpd1_2.inc"
#else
#include "stm32_usb1.inc"
#endif

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
  exti19_exti21_irq_init();

  i2c1_irq_init();
#if STM32_HAS_I2C2 && STM32_HAS_I2C3
  i2c2_i2c3_irq_init();
#elif STM32_HAS_I2C2
  i2c2_irq_init();
#endif

  tim1_irq_init();
  tim2_irq_init();
#if STM32_HAS_TIM3 && STM32_HAS_TIM4
  tim3_tim4_irq_init();
#elif STM32_HAS_TIM3
  tim3_irq_init();
#endif
  tim6_irq_init();
  tim7_irq_init();
  tim14_irq_init();
  tim15_irq_init();
  tim16_irq_init();
  tim17_irq_init();

  usart1_irq_init();
#if STM32_HAS_USART2 && STM32_HAS_LPUART2
  usart2_lpuart2_irq_init();
#elif STM32_HAS_USART2
  usart2_irq_init();
#endif
#if STM32_HAS_USART3 && STM32_HAS_UART4 && STM32_HAS_UART5 && STM32_HAS_USART6 && STM32_HAS_LPUART1
  usart3_usart4_usart5_usart6_lpuart1_irq_init();
#elif STM32_HAS_USART3 && STM32_HAS_UART4 && STM32_HAS_LPUART1
  usart3_usart4_lpuart1_irq_init();
#elif STM32_HAS_USART3 && STM32_HAS_UART4 && STM32_HAS_UART5 && STM32_HAS_USART6
  usart3_usart4_usart5_usart6_irq_init();
#elif STM32_HAS_LPUART1
  lpuart1_irq_init();
#endif

#if STM32_HAS_UCPD1 && STM32_HAS_UCPD2
  usb1_ucpd1_2_irq_init();
#else
  usb1_irq_init();
#endif
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
  exti19_exti21_irq_deinit();

  i2c1_irq_deinit();
#if STM32_HAS_I2C2 && STM32_HAS_I2C3
  i2c2_i2c3_irq_deinit();
#elif STM32_HAS_I2C2
  i2c2_irq_deinit();
#endif

  tim1_irq_deinit();
  tim2_irq_deinit();
#if STM32_HAS_TIM3 && STM32_HAS_TIM4
  tim3_tim4_irq_deinit();
#elif STM32_HAS_TIM3
  tim3_irq_deinit();
#endif
  tim6_irq_deinit();
  tim7_irq_deinit();
  tim14_irq_deinit();
  tim15_irq_deinit();
  tim16_irq_deinit();
  tim17_irq_deinit();

  usart1_irq_deinit();
#if STM32_HAS_USART2 && STM32_HAS_LPUART2
  usart2_lpuart2_irq_deinit();
#elif STM32_HAS_USART2
  usart2_irq_deinit();
#endif
#if STM32_HAS_USART3 && STM32_HAS_UART4 && STM32_HAS_UART5 && STM32_HAS_USART6 && STM32_HAS_LPUART1
  usart3_usart4_usart5_usart6_lpuart1_irq_deinit();
#elif STM32_HAS_USART3 && STM32_HAS_UART4 && STM32_HAS_LPUART1
  usart3_usart4_lpuart1_irq_deinit();
#elif STM32_HAS_LPUART1
  lpuart1_irq_deinit();
#endif

#if STM32_HAS_UCPD1 && STM32_HAS_UCPD2
  usb1_ucpd1_2_irq_deinit();
#else
  usb1_irq_deinit();
#endif
}

/** @} */
