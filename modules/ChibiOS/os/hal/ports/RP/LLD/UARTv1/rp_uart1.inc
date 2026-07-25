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
 * @file    UARTv1/rp_uart1.inc
 * @brief   Shared UART1 handler.
 *
 * @addtogroup RP_UART1_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(RP_HAS_UART1)
#error "RP_HAS_UART1 not defined in registry"
#endif

#if RP_HAS_UART1

/* Priority settings checks.*/
#if !defined(RP_IRQ_UART1_PRIORITY)
#error "RP_IRQ_UART1_PRIORITY not defined in mcuconf.h"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(RP_IRQ_UART1_PRIORITY)
#error "Invalid IRQ priority assigned to RP_IRQ_UART1_PRIORITY"
#endif

#endif /* RP_HAS_UART1 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void uart1_irq_init(void) {
#if defined(RP_UART1_IS_USED)
  nvicEnableVector(RP_UART1_IRQ_NUMBER, RP_IRQ_UART1_PRIORITY);
#endif
}

static inline void uart1_irq_deinit(void) {
#if defined(RP_UART1_IS_USED)
  nvicDisableVector(RP_UART1_IRQ_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(RP_UART1_IS_USED) || defined(__DOXYGEN__)
/**
 * @brief   UART1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(RP_UART1_IRQ_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_SERIAL
#if RP_SERIAL_USE_UART1
  sd_lld_serve_interrupt(&SD1);
#endif
#endif

#if HAL_USE_SIO
#if RP_SIO_USE_UART1
  sio_lld_serve_interrupt(&SIOD1);
#endif
#endif

#if HAL_USE_UART
#if RP_UART_USE_UART1
  uart_lld_serve_interrupt(&UARTD1);
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
