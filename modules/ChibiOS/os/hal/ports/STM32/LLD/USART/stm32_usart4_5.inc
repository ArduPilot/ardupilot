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
 * @file    USART/stm32_usart4_5.inc
 * @brief   Shared USART4, USART5 handler.
 *
 * @addtogroup STM32_USART4_5_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_HAS_UART4)
#error "STM32_HAS_UART4 not defined in registry"
#endif

#if !defined(STM32_HAS_UART5)
#error "STM32_HAS_UART5 not defined in registry"
#endif

#if STM32_HAS_UART4 || STM32_HAS_UART5

/* Priority settings checks.*/
#if !defined(STM32_IRQ_USART4_5_PRIORITY)
#error "STM32_IRQ_USART4_5_PRIORITY not defined in mcuconf.h"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_USART4_5_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_USART4_5_PRIORITY"
#endif

#endif /* STM32_HAS_UART4 || STM32_HAS_UART5 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void usart4_usart5_irq_init(void) {
#if defined(STM32_UART4_IS_USED) || defined(STM32_UART5_IS_USED)
  nvicEnableVector(STM32_USART4_5_NUMBER, STM32_IRQ_USART4_5_PRIORITY);
#endif
}

static inline void usart4_usart5_irq_deinit(void) {
#if defined(STM32_UART4_IS_USED) || defined(STM32_UART5_IS_USED)
  nvicDisableVector(STM32_USART4_5_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_UART4_IS_USED)  || defined(STM32_UART5_IS_USED) ||        \
    defined(__DOXYGEN__)
/**
 * @brief   USART4, USART5, LPUART1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USART4_5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_SERIAL
#if STM32_SERIAL_USE_UART4
  sd_lld_serve_interrupt(&SD4);
#endif
#if STM32_SERIAL_USE_UART5
  sd_lld_serve_interrupt(&SD5);
#endif
#endif

#if HAL_USE_SIO
#if STM32_SIO_USE_UART4
  sio_lld_serve_interrupt(&SIOD4);
#endif
#if STM32_SIO_USE_UART5
  sio_lld_serve_interrupt(&SIOD5);
#endif
#endif

#if HAL_USE_UART
#if STM32_UART_USE_UART4
  uart_lld_serve_interrupt(&UARTD4);
#endif
#if STM32_UART_USE_UART5
  uart_lld_serve_interrupt(&UARTD5);
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
