/*
    ChibiOS - Copyright (C) 2016..2018 Theodore Ateba

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
 * @file    hal_uart_lld.c
 * @brief   AVR UART subsystem low level driver source.
 *
 * @addtogroup UART
 * @{
 */

#include "hal.h"

#if (HAL_USE_UART == TRUE) || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

/**
 * @brief   USART1 (USARTC0) driver identifier.
 */
#if (AVR_UART_USE_USART1 == TRUE) || defined(__DOXYGEN__)
UARTDriver USART1D;
#endif

/**
 * @brief   USART2 (USARTC1) driver identifier.
 */
#if (AVR_UART_USE_USART2 == TRUE) || defined(__DOXYGEN__)
UARTDriver USART2D;
#endif

/**
 * @brief   USART3 (USARTD0) driver identifier.
 */
#if (AVR_UART_USE_USART3 == TRUE) || defined(__DOXYGEN__)
UARTDriver USART3D;
#endif

/**
 * @brief   USART4 (USARTD1) driver identifier.
 */
#if (AVR_UART_USE_USART4 == TRUE) || defined(__DOXYGEN__)
UARTDriver USART4D;
#endif

/**
 * @brief   USARTD5 (USARTE0) driver identifier.
 */
#if (AVR_UART_USE_USART5 == TRUE) || defined(__DOXYGEN__)
UARTDriver USART5D;
#endif

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/**
 * @brief   Configure the multiprocessor communication mode.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_cfg_mpcm(UARTDriver *uartp) {

  if (uartp->config->mpcm)
    uartp->usart->CTRLB |= (USART_MPCM_bm);   /* Enable the MPCM.   */
  else
    uartp->usart->CTRLB &= ~(USART_MPCM_bm);  /* Disable the MPCM.  */
}

/**
 * @brief   Configure the double speed.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_cfg_clk2x(UARTDriver *uartp) {

  if (uartp->config->clk2x)
    uartp->usart->CTRLB |= (USART_CLK2X_bm);  /* Enable double speed. */
  else
    uartp->usart->CTRLB &= ~(USART_CLK2X_bm); /* Use normal speed.    */
}

/**
 * @brief   Configuration of transmission mode (8/9 bits).
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_cfg_txb8(UARTDriver *uartp) {

  if (uartp->config->txb8)
    uartp->usart->CTRLB |= (USART_TXB8_bm);   /* 9 bits transmission mode. */
  else
    uartp->usart->CTRLB &= ~(USART_TXB8_bm);  /* 8 bits transmission mode. */
}

/**
 * @brief   Configuration of communication mode.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_cfg_cmode(UARTDriver *uartp) {

  if (uartp->config->cmode == USART_CMODE_MSPI) {
    uartp->usart->CTRLC = (uartp->usart->CTRLC & ~USART_CMODE_gm) | \
                          (USART_CMODE_MSPI_gc);
  }
  else if (uartp->config->cmode == USART_CMODE_IRCOM) {
    uartp->usart->CTRLC = (uartp->usart->CTRLC & ~USART_CMODE_gm) | \
                          (USART_CMODE_IRDA_gc);
  }
  else if (uartp->config->cmode == USART_CMODE_SYNCHRONOUS) {
    uartp->usart->CTRLC = (uartp->usart->CTRLC & ~USART_CMODE_gm) | \
                          (USART_CMODE_SYNCHRONOUS_gc);
  }
  else {
    uartp->usart->CTRLC = (uartp->usart->CTRLC & ~USART_CMODE_gm) | \
                          (USART_CMODE_ASYNCHRONOUS_gc);
  }
}

/**
 * @brief   Configuration of the number of stop to use during transmission.
 * @details @true set 2 stop bit and @false set 1 stop bit.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_cfg_sbmode(UARTDriver *uartp) {

  if (uartp->config->sbmode) {
    uartp->usart->CTRLC |= USART_SBMODE_bm;
  }
  else {
    uartp->usart->CTRLC &= ~USART_SBMODE_bm;
  }
}

/**
 * @brief   Configuration of parity mode.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_cfg_pmode(UARTDriver *uartp) {

  if (uartp->config->pmode == USART_PMODE_EVEN) {
    uartp->usart->CTRLC = (uartp->usart->CTRLC & ~USART_PMODE_gm) | \
                          (USART_PMODE_EVEN_gc);
  }
  else if (uartp->config->chsize == USART_PMODE_ODD) {
    uartp->usart->CTRLC = (uartp->usart->CTRLC & ~USART_PMODE_gm) | \
                          (USART_PMODE_ODD_gc);
  }
  else {
    uartp->usart->CTRLC = (uartp->usart->CTRLC & ~USART_PMODE_gm) | \
                          (USART_PMODE_DISABLED_gc);
  }
}

/**
 * @brief   Configuration of caracter size.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_cfg_chsize(UARTDriver *uartp) {

  if (uartp->config->chsize == USART_CHSIZE_5BIT) {
    uartp->usart->CTRLC = (uartp->usart->CTRLC & ~USART_CHSIZE_gm) | \
                          (USART_CHSIZE_5BIT_gc);
  }
  else if (uartp->config->chsize == USART_CHSIZE_6BIT) {
    uartp->usart->CTRLC = (uartp->usart->CTRLC & ~USART_CHSIZE_gm) | \
                          (USART_CHSIZE_6BIT_gc);
  }
  else if (uartp->config->chsize == USART_CHSIZE_7BIT) {
    uartp->usart->CTRLC = (uartp->usart->CTRLC & ~USART_CHSIZE_gm) | \
                          (USART_CHSIZE_7BIT_gc);
  }
  else if (uartp->config->chsize == USART_CHSIZE_8BIT) {
    uartp->usart->CTRLC = (uartp->usart->CTRLC & ~USART_CHSIZE_gm) | \
                          (USART_CHSIZE_8BIT_gc);
  }
  else {
    uartp->usart->CTRLC = (uartp->usart->CTRLC & ~USART_CHSIZE_gm) | \
                          (USART_CHSIZE_9BIT_gc);
  }
}

/**
 * @brief   Configuration of the baud rate.
 * @note    BSCALE is set to 0 for the moment.
 * @TODO    Support all the BSCALE value
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_cfg_baudrate(UARTDriver *uartp) {

  /* BSCALE = 0. */
  #define BSCALE 0
  uint16_t br = get_bsel(uartp->config->speed);
  uartp->usart->BAUDCTRLA = (uint8_t)br;
  uartp->usart->BAUDCTRLB = (BSCALE << USART_BSCALE0_bp) | (br >> 8);
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_stop(UARTDriver *uartp) {

  /* Stops RX and TX DMA channels. */
  /* DmaStreamDisable(uartp->dmarx); */
  /* DmaStreamDisable(uartp->dmatx); */

  /* Stops USART operations. */
  uartp->usart->CTRLB &= ~(USART_RXEN_bm); /* Disable the USART receiver.    */
  uartp->usart->CTRLB &= ~(USART_TXEN_bm); /* Disable the USART transmitter. */
}

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_start(UARTDriver *uartp) {

  /* Defensive programming, starting from a clean state. */
  usart_stop(uartp);

  /* Resetting eventual pending status flags. */

  /* Starting the receiver idle loop. */
  /* Uart_enter_rx_idle_loop(uartp);  */

  usart_cfg_mpcm(uartp);  /* Set the multi processor communication mode.    */
  usart_cfg_clk2x(uartp); /* Set the USART speed (Normal/Double).           */
  usart_cfg_txb8(uartp);  /* Set the transmission mode (8/9bits).           */
  usart_cfg_cmode(uartp); /* Set the communication mode.                    */
  usart_cfg_sbmode(uartp);/* Set the stop bit mode.                         */
  usart_cfg_pmode(uartp); /* Set the parity mode.                           */
  usart_cfg_chsize(uartp);/* Set the character size.                        */
  usart_cfg_baudrate(uartp); /* Set the baud rate.                          */
  uartp->usart->CTRLB |= (USART_RXEN_bm); /* Enable the USART receiver.     */
  uartp->usart->CTRLB |= (USART_TXEN_bm); /* Enable the USART transmitter.  */
}

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

#if AVR_UART_USE_USART1 || defined(__DOXYGEN__)
/**
 * @brief   USART1 TX IRQ handler, transmission complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTC0_TXC_vect) {

  OSAL_IRQ_PROLOGUE();

  /* @TODO serve_usart_irq(&UARTD1);*/

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART1 RX IRQ handler, reception complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTC0_RXC_vect) {

  OSAL_IRQ_PROLOGUE();

  /* @TODO serve_usart_irq(&UARTD1);*/

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART1 DRE IRQ handler, data register empty interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTC0_DRE_vect) {

  OSAL_IRQ_PROLOGUE();

  /* @TODO serve_usart_irq(&UARTD1);*/

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

#if AVR_UART_USE_USART1 == TRUE
  /* Driver initialization. */
  uartObjectInit(&USART1D);
  USART1D.usart = &USARTC0;
  /* USART1D.usart->CTRLC = 0;      */
  /* USART1D.usart->BAUDCTRLA = 0;  */
#endif

#if AVR_UART_USE_USART2 == TRUE
  /* Driver initialization. */
  uartObjectInit(&USART2D);
#endif

#if AVR_UART_USE_USART3 == TRUE
  /* Driver initialization. */
  uartObjectInit(&USART3D);
#endif

#if AVR_UART_USE_USART4 == TRUE
  /* Driver initialization. */
  uartObjectInit(&USART4D);
#endif

#if AVR_UART_USE_USART5 == TRUE
  /* Driver initialization. */
  uartObjectInit(&USART5D);
#endif
}

/**
 * @brief   Configures and activates the UART peripheral.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_start(UARTDriver *uartp) {

  if (uartp->state == UART_STOP) {
    /* Enables the peripheral. */
#if AVR_UART_USE_USART1 == TRUE
    if (&USART1D == uartp) {
      /* TODO: Implement the DMA part here. */
    }
#endif
  }
  /* Configures the peripheral. */
  uartp->rxstate = UART_RX_IDLE;
  uartp->txstate = UART_TX_IDLE;
  usart_start(uartp);
}

/**
 * @brief   Deactivates the UART peripheral.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_stop(UARTDriver *uartp) {

  if (uartp->state == UART_READY) {
    /* Resets the peripheral.  */
    usart_stop(uartp);
    /* Disables the peripheral. */
#if AVR_UART_USE_USART1 == TRUE
    if (&USART1D == uartp) {
      /* TODO: Implement the DMA part here. */
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

  /* TODO: Add support of DMA. */
  while (n--) {
    while (!((uartp->usart->STATUS & USART_DREIF_bm) != 0));
    uartp->usart->DATA = *txbuf;
    txbuf++;
  }
}

/**
 * @brief   Stops any ongoing transmission.
 * @note    Stopping a transmission also suppresses the transmission callbacks.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not transmitted by the
 *                      stopped transmit operation.
 *
 * @notapi
 */
size_t uart_lld_stop_send(UARTDriver *uartp) {

  (void)uartp;

  return 0;
}

/**
 * @brief   Starts a receive operation on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void uart_lld_start_receive(UARTDriver *uartp, size_t n, void *rxbuf) {

  (void)uartp;
  (void)n;
  (void)rxbuf;
}

/**
 * @brief   Stops any ongoing receive operation.
 * @note    Stopping a receive operation also suppresses the receive callbacks.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not received by the
 *                      stopped receive operation.
 *
 * @notapi
 */
size_t uart_lld_stop_receive(UARTDriver *uartp) {

  (void)uartp;

  return 0;
}

#endif /* HAL_USE_UART == TRUE */

/** @} */
