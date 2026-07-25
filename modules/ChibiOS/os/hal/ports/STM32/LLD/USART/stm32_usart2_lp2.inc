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
 * @file    USART/stm32_usart2_lp2.inc
 * @brief   Shared USART2, LPUART2 handler.
 *
 * @addtogroup STM32_USART2_LP2_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_HAS_USART2)
#error "STM32_HAS_USART2 not defined in registry"
#endif

#if !defined(STM32_HAS_LPUART2)
#error "STM32_HAS_LPUART2 not defined in registry"
#endif

#if STM32_HAS_USART2 || STM32_HAS_LPUART2

/* Priority settings checks.*/
#if !defined(STM32_IRQ_USART2_LP2_PRIORITY)
#error "STM32_IRQ_USART2_LP2_PRIORITY not defined in mcuconf.h"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_USART2_LP2_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_USART2_LP2_PRIORITY"
#endif

#endif /* STM32_HAS_USART2 || STM32_HAS_LPUART2 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void usart2_lpuart2_irq_init(void) {
#if defined(STM32_USART2_IS_USED) || defined(STM32_LPUART2_IS_USED)
  nvicEnableVector(STM32_USART2_LP2_NUMBER, STM32_IRQ_USART2_LP2_PRIORITY);
#endif
}

static inline void usart2_lpuart2_irq_deinit(void) {
#if defined(STM32_USART2_IS_USED) || defined(STM32_LPUART2_IS_USED)
  nvicDisableVector(STM32_USART2_LP2_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_USART2_IS_USED)  || defined(STM32_LPUART2_IS_USED) ||     \
    defined(__DOXYGEN__)
/**
 * @brief   USART2, LPUART2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USART2_LP2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_SERIAL
#if STM32_SERIAL_USE_USART2
  sd_lld_serve_interrupt(&SD2);
#endif
#if STM32_SERIAL_USE_LPUART2
  sd_lld_serve_interrupt(&LPSD2);
#endif
#endif

#if HAL_USE_SIO
#if STM32_SIO_USE_USART2
  sio_lld_serve_interrupt(&SIOD2);
#endif
#if STM32_SIO_USE_LPUART2
  sio_lld_serve_interrupt(&LPSIOD2);
#endif
#endif

#if HAL_USE_UART
#if STM32_UART_USE_USART2
  uart_lld_serve_interrupt(&UARTD2);
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
