/*
    ChibiOS - Copyright (C) 2017 Theodore Ateba

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
 * @file    USARTv1/hal_uart_lld.c
 * @brief   AVR Tiny low level UART driver source file.
 *
 * @addtogroup UART
 * @{
 */

#include "hal.h"

#if HAL_USE_UART || defined(__DOXYGEN__)

/*==========================================================================*/
/* Macros definition.                                                       */
/*==========================================================================*/
#define  UBRR(b)  (((F_CPU / b) >> 5) - 1)

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

/** @brief USART1 UART driver identifier. */
#if AVR_UART_USE_USART1 || defined(__DOXYGEN__)
UARTDriver UARTD1;
#endif

/** @brief USART2 UART driver identifier. */
#if AVR_UART_USE_USART2 || defined(__DOXYGEN__)
UARTDriver UARTD2;
#endif

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_stop(UARTDriver *uartp) {

#if AVR_UART_USE_USART1
  LINCR  &= ~(1 << LENA); /* Disable LIN/UART controller. */
#endif
}

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_start(UARTDriver *uartp) {

  /* Defensive programming, starting from a clean state.  */
  usart_stop(uartp);

#if AVR_UART_USE_USART1
  /* Compute bit time. */
  uint16_t BIT_TIME = UBRR(uartp->config->speed);

  /* Set UART baudrate. */
  LINBRRH  |= (uint8_t) (((BIT_TIME) >> 8) & 0x0F);
  LINBRRL  |= (uint8_t) (BIT_TIME);

  /* Enable LIN/UART controller. */
  LINCR  |= (1 << LENA);
#endif
}

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

#if AVR_UART_USE_USART1 || defined(__DOXYGEN__)
/**
 * @brief   USART1 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(LIN_TC_vect) {

  OSAL_IRQ_PROLOGUE();

  /* TODO: Manage the UART IRQ. */

  OSAL_IRQ_EPILOGUE();
}
#endif /* AVR_UART_USE_USART1 */

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Low level UART driver initialization.
 *
 * @notapi
 */
void uart_lld_init(void) {

#if AVR_UART_USE_USART1
  uartObjectInit(&UARTD1);
#endif
}

/**
 * @brief   Configures and activates the UART peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_start(UARTDriver *uartp) {

  if (uartp->state == UART_STOP) {
#if AVR_UART_USE_USART1
    if (&UARTD1 == uartp) {

      /* Set the full duplex. */
      LINCR  |= (1 << LCMD2);
      LINCR  |= (1 << LCMD1);
      LINCR  |= (1 << LCMD0);

      /* Set the Uart data format: 8-bit data, no parity, 1 stop-bit. */
      LINCR &= ~(1 << LCONF1);
      LINCR &= ~(1 << LCONF0);
    }
#endif
  }

  uartp->rxstate = UART_RX_IDLE;
  uartp->txstate = UART_TX_IDLE;
  usart_start(uartp);
}

/**
 * @brief   Deactivates the UART peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_stop(UARTDriver *uartp) {

  if (uartp->state == UART_READY) {
    usart_stop(uartp);

#if AVR_UART_USE_USART1
    if (&UARTD1 == uartp) {
      return;
    }
#endif
  }
}

/**
 * @brief   Starts a transmission on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void uart_lld_start_send(UARTDriver *uartp, size_t n, const uint8_t *txbuf) {

#if AVR_UART_USE_USART1
  if (&UARTD1  == uartp) {
    /* Starting transfer. */
    while (n--) {
      while (LINSIR & (1 << LBUSY));
      LINDAT  = *txbuf;
      txbuf++;
    }
  }
#endif
}

/**
 * @brief   Starts a receive operation on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in]   uartp     pointer to the @p UARTDriver object
 * @param[in]   n         number of data frames to send
 * @param[out]  rxbuf     the pointer to the receive buffer
 *
 * @notapi
 */
void uart_lld_start_receive(UARTDriver *uartp, size_t n, void *rxbuf) {

  /* Stopping previous activity (idle state). */
  /* TODO: Implement  this function. */
}

#endif /* HAL_USE_UART */

/** @} */
