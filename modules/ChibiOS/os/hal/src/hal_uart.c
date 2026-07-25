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
 * @file    hal_uart.c
 * @brief   UART Driver code.
 *
 * @addtogroup UART
 * @{
 */

#include "hal.h"

#if (HAL_USE_UART == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   UART Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void uartInit(void) {

  uart_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p UARTDriver structure.
 *
 * @param[out] uartp    pointer to the @p UARTDriver object
 *
 * @init
 */
void uartObjectInit(UARTDriver *uartp) {

  uartp->state      = UART_STOP;
  uartp->txstate    = UART_TX_IDLE;
  uartp->rxstate    = UART_RX_IDLE;
  uartp->config     = NULL;
#if UART_USE_WAIT == TRUE
  uartp->early      = false;
  uartp->threadrx   = NULL;
  uartp->threadtx   = NULL;
#endif /* UART_USE_WAIT */
#if UART_USE_MUTUAL_EXCLUSION == TRUE
  osalMutexObjectInit(&uartp->mutex);
#endif /* UART_USE_MUTUAL_EXCLUSION */

  /* Optional, user-defined initializer.*/
#if defined(UART_DRIVER_EXT_INIT_HOOK)
  UART_DRIVER_EXT_INIT_HOOK(uartp);
#endif
}

/**
 * @brief   Configures and activates the UART peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] config    pointer to the @p UARTConfig object
 * @return              The operation status.
 *
 * @api
 */
msg_t uartStart(UARTDriver *uartp, const UARTConfig *config) {
  msg_t msg;

  osalDbgCheck((uartp != NULL) && (config != NULL));

  osalSysLock();
  osalDbgAssert((uartp->state == UART_STOP) || (uartp->state == UART_READY),
                "invalid state");

  uartp->config = config;

#if defined(UART_LLD_ENHANCED_API)
  msg = uart_lld_start(uartp);
  if (msg == HAL_RET_SUCCESS) {
    uartp->state = UART_READY;
  }
  else {
    uartp->state = UART_STOP;
  }
#else
  uart_lld_start(uartp);
  uartp->state = UART_READY;
  msg = HAL_RET_SUCCESS;
#endif

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Deactivates the UART peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @api
 */
void uartStop(UARTDriver *uartp) {

  osalDbgCheck(uartp != NULL);

  osalSysLock();

  osalDbgAssert((uartp->state == UART_STOP) || (uartp->state == UART_READY),
                "invalid state");

  uart_lld_stop(uartp);
  uartp->config  = NULL;
  uartp->state   = UART_STOP;
  uartp->txstate = UART_TX_IDLE;
  uartp->rxstate = UART_RX_IDLE;

  osalSysUnlock();
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
 * @api
 */
void uartStartSend(UARTDriver *uartp, size_t n, const void *txbuf) {

  osalDbgCheck((uartp != NULL) && (n > 0U) && (txbuf != NULL));

  osalSysLock();
  osalDbgAssert(uartp->state == UART_READY, "is active");
  osalDbgAssert(uartp->txstate != UART_TX_ACTIVE, "tx active");

  uart_lld_start_send(uartp, n, txbuf);
  uartp->txstate = UART_TX_ACTIVE;
  osalSysUnlock();
}

/**
 * @brief   Starts a transmission on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 * @note    This function has to be invoked from a lock zone.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @iclass
 */
void uartStartSendI(UARTDriver *uartp, size_t n, const void *txbuf) {

  osalDbgCheckClassI();
  osalDbgCheck((uartp != NULL) && (n > 0U) && (txbuf != NULL));
  osalDbgAssert(uartp->state == UART_READY, "is active");
  osalDbgAssert(uartp->txstate != UART_TX_ACTIVE, "tx active");

  uart_lld_start_send(uartp, n, txbuf);
  uartp->txstate = UART_TX_ACTIVE;
}

/**
 * @brief   Stops any ongoing transmission.
 * @note    Stopping a transmission also suppresses the transmission callbacks.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not transmitted by the
 *                      stopped transmit operation.
 * @retval UART_ERR_NOT_ACTIVE if there was no transmit operation in progress.
 *
 * @api
 */
size_t uartStopSend(UARTDriver *uartp) {
  size_t n;

  osalDbgCheck(uartp != NULL);

  osalSysLock();
  osalDbgAssert(uartp->state == UART_READY, "not active");

  if (uartp->txstate == UART_TX_ACTIVE) {
    n = uart_lld_stop_send(uartp);
    uartp->txstate = UART_TX_IDLE;
  }
  else {
    n = UART_ERR_NOT_ACTIVE;
  }
  osalSysUnlock();

  return n;
}

/**
 * @brief   Stops any ongoing transmission.
 * @note    Stopping a transmission also suppresses the transmission callbacks.
 * @note    This function has to be invoked from a lock zone.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not transmitted by the
 *                      stopped transmit operation.
 * @retval UART_ERR_NOT_ACTIVE if there was no transmit operation in progress.
 *
 * @iclass
 */
size_t uartStopSendI(UARTDriver *uartp) {

  osalDbgCheckClassI();
  osalDbgCheck(uartp != NULL);
  osalDbgAssert(uartp->state == UART_READY, "not active");

  if (uartp->txstate == UART_TX_ACTIVE) {
    size_t n = uart_lld_stop_send(uartp);
    uartp->txstate = UART_TX_IDLE;
    return n;
  }
  return UART_ERR_NOT_ACTIVE;
}

/**
 * @brief   Starts a receive operation on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to receive
 * @param[in] rxbuf     the pointer to the receive buffer
 *
 * @api
 */
void uartStartReceive(UARTDriver *uartp, size_t n, void *rxbuf) {

  osalDbgCheck((uartp != NULL) && (n > 0U) && (rxbuf != NULL));

  osalSysLock();
  osalDbgAssert(uartp->state == UART_READY, "is active");
  osalDbgAssert(uartp->rxstate != UART_RX_ACTIVE, "rx active");

  uart_lld_start_receive(uartp, n, rxbuf);
  uartp->rxstate = UART_RX_ACTIVE;
  osalSysUnlock();
}

/**
 * @brief   Starts a receive operation on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 * @note    This function has to be invoked from a lock zone.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @iclass
 */
void uartStartReceiveI(UARTDriver *uartp, size_t n, void *rxbuf) {

  osalDbgCheckClassI();
  osalDbgCheck((uartp != NULL) && (n > 0U) && (rxbuf != NULL));
  osalDbgAssert(uartp->state == UART_READY, "is active");
  osalDbgAssert(uartp->rxstate != UART_RX_ACTIVE, "rx active");

  uart_lld_start_receive(uartp, n, rxbuf);
  uartp->rxstate = UART_RX_ACTIVE;
}

/**
 * @brief   Stops any ongoing receive operation.
 * @note    Stopping a receive operation also suppresses the receive callbacks.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not received by the
 *                      stopped receive operation.
 * @retval UART_ERR_NOT_ACTIVE if there was no receive operation in progress.
 *
 * @api
 */
size_t uartStopReceive(UARTDriver *uartp) {
  size_t n;

  osalDbgCheck(uartp != NULL);

  osalSysLock();
  osalDbgAssert(uartp->state == UART_READY, "not active");

  if (uartp->rxstate == UART_RX_ACTIVE) {
    n = uart_lld_stop_receive(uartp);
    uartp->rxstate = UART_RX_IDLE;
  }
  else {
    n = UART_ERR_NOT_ACTIVE;
  }
  osalSysUnlock();

  return n;
}

/**
 * @brief   Stops any ongoing receive operation.
 * @note    Stopping a receive operation also suppresses the receive callbacks.
 * @note    This function has to be invoked from a lock zone.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not received by the
 *                      stopped receive operation.
 * @retval UART_ERR_NOT_ACTIVE if there was no receive operation in progress.
 *
 * @iclass
 */
size_t uartStopReceiveI(UARTDriver *uartp) {

  osalDbgCheckClassI();
  osalDbgCheck(uartp != NULL);
  osalDbgAssert(uartp->state == UART_READY, "not active");

  if (uartp->rxstate == UART_RX_ACTIVE) {
    size_t n = uart_lld_stop_receive(uartp);
    uartp->rxstate = UART_RX_IDLE;
    return n;
  }
  return UART_ERR_NOT_ACTIVE;
}

#if (UART_USE_WAIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Performs a transmission on the UART peripheral.
 * @note    The function returns when the specified number of frames have been
 *          sent to the UART or on timeout.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 * @note    This function implements a software timeout, it does not use
 *          any underlying HW timeout mechanism.
 * @note    If the configured callback starts another transmission before this
 *          function returns then the behavior is undefined.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in,out] np    number of data frames to transmit, on exit the number
 *                      of frames actually transmitted
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[in] timeout   operation timeout
 * @return              The operation status.
 * @retval MSG_OK       if the operation completed successfully.
 * @retval MSG_TIMEOUT  if the operation timed out.
 *
 * @api
 */
msg_t uartSendTimeout(UARTDriver *uartp, size_t *np,
                      const void *txbuf, sysinterval_t timeout) {
  msg_t msg;

  osalDbgCheck((uartp != NULL) && (*np > 0U) && (txbuf != NULL));

  osalSysLock();
  osalDbgAssert(uartp->state == UART_READY, "is active");
  osalDbgAssert(uartp->txstate != UART_TX_ACTIVE, "tx active");

  /* Transmission start.*/
  uartp->early = true;
  uart_lld_start_send(uartp, *np, txbuf);
  uartp->txstate = UART_TX_ACTIVE;

  /* Waiting for result.*/
  msg = osalThreadSuspendTimeoutS(&uartp->threadtx, timeout);
  if (msg != MSG_OK) {
    *np -= uartStopSendI(uartp);
  }
  osalSysUnlock();

  return msg;
}

/**
 * @brief   Performs a transmission on the UART peripheral.
 * @note    The function returns when the specified number of frames have been
 *          physically transmitted or on timeout.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 * @note    This function implements a software timeout, it does not use
 *          any underlying HW timeout mechanism.
 * @note    If the configured callback starts another transmission before this
 *          function returns then the behavior is undefined.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in,out] np    number of data frames to transmit, on exit the number
 *                      of frames actually transmitted
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[in] timeout   operation timeout
 * @return              The operation status.
 * @retval MSG_OK       if the operation completed successfully.
 * @retval MSG_TIMEOUT  if the operation timed out.
 *
 * @api
 */
msg_t uartSendFullTimeout(UARTDriver *uartp, size_t *np,
                          const void *txbuf, sysinterval_t timeout) {
  msg_t msg;

  osalDbgCheck((uartp != NULL) && (*np > 0U) && (txbuf != NULL));

  osalSysLock();
  osalDbgAssert(uartp->state == UART_READY, "is active");
  osalDbgAssert(uartp->txstate != UART_TX_ACTIVE, "tx active");

  /* Transmission start.*/
  uartp->early = false;
  uart_lld_start_send(uartp, *np, txbuf);
  uartp->txstate = UART_TX_ACTIVE;

  /* Waiting for result.*/
  msg = osalThreadSuspendTimeoutS(&uartp->threadtx, timeout);
  if (msg != MSG_OK) {
    *np -= uartStopSendI(uartp);
  }
  osalSysUnlock();

  return msg;
}

/**
 * @brief   Performs a receive operation on the UART peripheral.
 * @note    The function returns when the specified number of frames have been
 *          received or on error/timeout.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 * @note    This function implements a software timeout, it does not use
 *          any underlying HW timeout mechanism.
 * @note    If the configured callback starts another receive operation before
 *          this function returns then the behavior is undefined.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in,out] np    number of data frames to receive, on exit the number
 *                      of frames actually received
 * @param[in] rxbuf     the pointer to the receive buffer
 * @param[in] timeout   operation timeout
 *
 * @return              The operation status.
 * @retval MSG_OK       if the operation completed successfully.
 * @retval MSG_TIMEOUT  if the operation timed out.
 * @retval MSG_RESET    in case of a receive error.
 *
 * @api
 */
msg_t uartReceiveTimeout(UARTDriver *uartp, size_t *np,
                         void *rxbuf, sysinterval_t timeout) {
  msg_t msg;

  osalDbgCheck((uartp != NULL) && (*np > 0U) && (rxbuf != NULL));

  osalSysLock();
  osalDbgAssert(uartp->state == UART_READY, "is active");
  osalDbgAssert(uartp->rxstate != UART_RX_ACTIVE, "rx active");

  /* Receive start.*/
  uart_lld_start_receive(uartp, *np, rxbuf);
  uartp->rxstate = UART_RX_ACTIVE;

  /* Waiting for result.*/
  msg = osalThreadSuspendTimeoutS(&uartp->threadrx, timeout);
  if (msg != MSG_OK) {
    *np -= uartStopReceiveI(uartp);
  }
  osalSysUnlock();

  return msg;
}
#endif

#if (UART_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Gains exclusive access to the UART bus.
 * @details This function tries to gain ownership to the UART bus, if the bus
 *          is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option @p UART_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @api
 */
void uartAcquireBus(UARTDriver *uartp) {

  osalDbgCheck(uartp != NULL);

  osalMutexLock(&uartp->mutex);
}

/**
 * @brief   Releases exclusive access to the UART bus.
 * @pre     In order to use this function the option @p UART_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @api
 */
void uartReleaseBus(UARTDriver *uartp) {

  osalDbgCheck(uartp != NULL);

  osalMutexUnlock(&uartp->mutex);
}
#endif

#endif /* HAL_USE_UART == TRUE */

/** @} */
