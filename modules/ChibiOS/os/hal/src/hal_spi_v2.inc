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
 * @file    hal_spi_v2.inc
 * @brief   SPI (v2) Driver code.
 *
 * @addtogroup SPI_V2
 * @{
 */

#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

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
 * @brief   SPI Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void spiInit(void) {

  spi_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p SPIDriver structure.
 *
 * @param[out] spip             pointer to the @p SPIDriver object
 *
 * @init
 */
void spiObjectInit(SPIDriver *spip) {

  spip->state           = SPI_STOP;
  spip->config          = NULL;
#if SPI_USE_SYNCHRONIZATION == TRUE
  spip->sync_transfer   = NULL;
#endif
#if SPI_USE_MUTUAL_EXCLUSION == TRUE
  osalMutexObjectInit(&spip->mutex);
#endif
#if defined(SPI_DRIVER_EXT_INIT_HOOK)
  SPI_DRIVER_EXT_INIT_HOOK(spip);
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] config            pointer to the @p SPIConfig object
 * @return                      The operation status.
 *
 * @api
 */
msg_t spiStart(SPIDriver *spip, const SPIConfig *config) {
  msg_t msg;

  osalDbgCheck((spip != NULL) && (config != NULL));

  osalSysLock();
  osalDbgAssert((spip->state == SPI_STOP) || (spip->state == SPI_READY),
                "invalid state");

  spip->config = config;

  msg = spi_lld_start(spip);
  if (msg == HAL_RET_SUCCESS) {
    spip->state = SPI_READY;
  }
  else {
    spip->state = SPI_STOP;
  }

  osalSysUnlock();

#if SPI_USE_ASSERT_ON_ERROR == TRUE
  osalDbgAssert(msg == HAL_RET_SUCCESS, "function failed");
#endif

  return msg;
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 *
 * @api
 */
void spiStop(SPIDriver *spip) {

  osalDbgCheck(spip != NULL);

  osalSysLock();

  osalDbgAssert((spip->state == SPI_STOP) || (spip->state == SPI_READY),
                "invalid state");

  spi_lld_stop(spip);
  spip->config = NULL;
  spip->state  = SPI_STOP;

  osalSysUnlock();
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 *
 * @api
 */
void spiSelect(SPIDriver *spip) {

  osalDbgCheck(spip != NULL);

  osalSysLock();
  osalDbgAssert(spip->state == SPI_READY, "not ready");
  spiSelectI(spip);
  osalSysUnlock();
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 *
 * @api
 */
void spiUnselect(SPIDriver *spip) {

  osalDbgCheck(spip != NULL);

  osalSysLock();
  osalDbgAssert(spip->state == SPI_READY, "not ready");
  spiUnselectI(spip);
  osalSysUnlock();
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] n                 number of words to be ignored
 * @return                      The operation status.
 *
 * @iclass
 */
msg_t spiStartIgnoreI(SPIDriver *spip, size_t n) {
  msg_t msg;

  osalDbgCheckClassI();

  osalDbgCheck((spip != NULL) && (n > 0U));
#if SPI_SUPPORTS_CIRCULAR
  osalDbgCheck((spip->config->circular == false) || ((n & 1U) == 0U));
#endif

  osalDbgAssert(spip->state == SPI_READY, "not ready");

  spip->state = SPI_ACTIVE;
  msg = spi_lld_ignore(spip, n);

#if SPI_USE_ASSERT_ON_ERROR == TRUE
  osalDbgAssert(msg == HAL_RET_SUCCESS, "function failed");
#endif

  return msg;
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] n                 number of words to be ignored
 * @return                      The operation status.
 *
 * @api
 */
msg_t spiStartIgnore(SPIDriver *spip, size_t n) {
  msg_t msg;

  osalSysLock();
  msg = spiStartIgnoreI(spip, n);
  osalSysUnlock();

  return msg;
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] n                 number of words to be exchanged
 * @param[in] txbuf             the pointer to the transmit buffer
 * @param[out] rxbuf            the pointer to the receive buffer
 * @return                      The operation status.
 *
 * @iclass
 */
msg_t spiStartExchangeI(SPIDriver *spip, size_t n,
                       const void *txbuf, void *rxbuf) {
  msg_t msg;

  osalDbgCheckClassI();

  osalDbgCheck((spip != NULL) && (n > 0U) &&
               (rxbuf != NULL) && (txbuf != NULL));
#if SPI_SUPPORTS_CIRCULAR
  osalDbgCheck((spip->config->circular == false) || ((n & 1U) == 0U));
#endif

  osalDbgAssert(spip->state == SPI_READY, "not ready");

  spip->state = SPI_ACTIVE;
  msg = spi_lld_exchange(spip, n, txbuf, rxbuf);

#if SPI_USE_ASSERT_ON_ERROR == TRUE
  osalDbgAssert(msg == HAL_RET_SUCCESS, "function failed");
#endif

  return msg;
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] n                 number of words to be exchanged
 * @param[in] txbuf             the pointer to the transmit buffer
 * @param[out] rxbuf            the pointer to the receive buffer
 * @return                      The operation status.
 *
 * @api
 */
msg_t spiStartExchange(SPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {
  msg_t msg;

  osalSysLock();
  msg = spiStartExchangeI(spip, n, txbuf, rxbuf);
  osalSysUnlock();

  return msg;
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] n                 number of words to send
 * @param[in] txbuf             the pointer to the transmit buffer
 * @return                      The operation status.
 *
 * @iclass
 */
msg_t spiStartSendI(SPIDriver *spip, size_t n, const void *txbuf) {
  msg_t msg;

  osalDbgCheckClassI();

  osalDbgCheck((spip != NULL) && (n > 0U) && (txbuf != NULL));
#if SPI_SUPPORTS_CIRCULAR
  osalDbgCheck((spip->config->circular == false) || ((n & 1U) == 0U));
#endif

  osalDbgAssert(spip->state == SPI_READY, "not ready");

  spip->state = SPI_ACTIVE;
  msg = spi_lld_send(spip, n, txbuf);

#if SPI_USE_ASSERT_ON_ERROR == TRUE
  osalDbgAssert(msg == HAL_RET_SUCCESS, "function failed");
#endif

  return msg;
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] n                 number of words to send
 * @param[in] txbuf             the pointer to the transmit buffer
 * @return                      The operation status.
 *
 * @api
 */
msg_t spiStartSend(SPIDriver *spip, size_t n, const void *txbuf) {
  msg_t msg;

  osalSysLock();
  msg = spiStartSendI(spip, n, txbuf);
  osalSysUnlock();

  return msg;
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] n                 number of words to receive
 * @param[out] rxbuf            the pointer to the receive buffer
 * @return                      The operation status.
 *
 * @iclass
 */
msg_t spiStartReceiveI(SPIDriver *spip, size_t n, void *rxbuf) {
  msg_t msg;

  osalDbgCheckClassI();

  osalDbgCheck((spip != NULL) && (n > 0U) && (rxbuf != NULL));
#if SPI_SUPPORTS_CIRCULAR
  osalDbgCheck((spip->config->circular == false) || ((n & 1U) == 0U));
#endif

  osalDbgAssert(spip->state == SPI_READY, "not ready");

  spip->state = SPI_ACTIVE;
  msg = spi_lld_receive(spip, n, rxbuf);

#if SPI_USE_ASSERT_ON_ERROR == TRUE
  osalDbgAssert(msg == HAL_RET_SUCCESS, "function failed");
#endif

  return msg;
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] n                 number of words to receive
 * @param[out] rxbuf            the pointer to the receive buffer
 * @return                      The operation status.
 *
 * @api
 */
msg_t spiStartReceive(SPIDriver *spip, size_t n, void *rxbuf) {
  msg_t msg;

  osalSysLock();
  msg = spiStartReceiveI(spip, n, rxbuf);
  osalSysUnlock();

  return msg;
}

/**
 * @brief   Stops the ongoing SPI operation.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[out] sizep            pointer to the counter of frames not yet
 *                              transferred or @p NULL
 * @return                      The operation status.
 *
 * @iclass
 */
msg_t spiStopTransferI(SPIDriver *spip, size_t *sizep) {
  msg_t msg;

  osalDbgCheckClassI();

  osalDbgCheck(spip != NULL);

  osalDbgAssert((spip->state == SPI_READY) ||
                (spip->state == SPI_ACTIVE) ||
                (spip->state == SPI_COMPLETE),
                "invalid state");

  if ((spip->state == SPI_ACTIVE) || (spip->state == SPI_COMPLETE)) {

    /* Stopping transfer at low level.*/
    msg = spi_lld_stop_transfer(spip, sizep);
    spip->state = SPI_READY;

#if SPI_USE_SYNCHRONIZATION == TRUE
    osalThreadResumeI(&spip->sync_transfer, MSG_RESET);
#endif
  }
  else {
    msg = HAL_RET_SUCCESS;
  }

  return msg;
}

/**
 * @brief   Stops the ongoing SPI operation, if any.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[out] sizep            pointer to the counter of frames not yet
 *                              transferred or @p NULL
 * @return                      The operation status.
 *
 * @api
 */
msg_t spiStopTransfer(SPIDriver *spip, size_t *sizep) {
  msg_t msg;

  osalSysLock();

  msg = spiStopTransferI(spip, sizep);
  osalOsRescheduleS();

  osalSysUnlock();

  return msg;
}

#if (SPI_USE_SYNCHRONIZATION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Synchronizes with current transfer completion.
 * @note    This function can only be called by a single thread at time.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] timeout           synchronization timeout
 * @return                      The synchronization result.
 * @retval MSG_OK               if operation completed without errors.
 * @retval MSG_TIMEOUT          if synchronization timed out.
 * @retval MSG_RESET            if the transfer has been stopped.
 *
 * @sclass
 */
msg_t spiSynchronizeS(SPIDriver *spip, sysinterval_t timeout) {
  msg_t msg;

  osalDbgCheck(spip != NULL);
  osalDbgAssert((spip->state == SPI_ACTIVE) || (spip->state == SPI_READY),
                "invalid state");

  if (spip->state == SPI_ACTIVE) {
    msg = osalThreadSuspendTimeoutS(&spip->sync_transfer, timeout);
  }
  else {
    msg = MSG_OK;
  }

  return msg;
}

/**
 * @brief   Synchronizes with current transfer completion.
 * @note    This function can only be called by a single thread at time.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] timeout           synchronization timeout
 * @return                      The synchronization result.
 * @retval MSG_OK               if operation completed without errors.
 * @retval MSG_TIMEOUT          if synchronization timed out.
 * @retval MSG_RESET            if the transfer has been stopped.
 *
 * @api
 */
msg_t spiSynchronize(SPIDriver *spip, sysinterval_t timeout) {
  msg_t msg;

  osalSysLock();
  msg = spiSynchronizeS(spip, timeout);
  osalSysUnlock();

  return msg;
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This synchronous function performs the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @pre     In order to use this function the option @p SPI_USE_SYNCHRONIZATION
 *          must be enabled.
 * @note    If the configured callback starts another transfer before this
 *          function returns then the behavior is undefined.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] n                 number of words to be ignored
 * @return                      The operation status.
 * @retval MSG_OK               if operation completed without errors.
 * @retval MSG_TIMEOUT          if synchronization timed out.
 * @retval MSG_RESET            if the transfer has been stopped.
 *
 * @api
 */
msg_t spiIgnore(SPIDriver *spip, size_t n) {
  msg_t msg;

  osalSysLock();

  msg = spiStartIgnoreI(spip, n);
  if (msg == MSG_OK) {
    msg = spiSynchronizeS(spip, TIME_INFINITE);
  }

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This synchronous function performs a simultaneous transmit/receive
 *          operation.
 * @pre     In order to use this function the option @p SPI_USE_SYNCHRONIZATION
 *          must be enabled.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 * @note    If the configured callback starts another transfer before this
 *          function returns then the behavior is undefined.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] n                 number of words to be exchanged
 * @param[in] txbuf             the pointer to the transmit buffer
 * @param[out] rxbuf            the pointer to the receive buffer
 * @return                      The operation status.
 * @retval MSG_OK               if operation completed without errors.
 * @retval MSG_TIMEOUT          if synchronization timed out.
 * @retval MSG_RESET            if the transfer has been stopped.
 *
 * @api
 */
msg_t spiExchange(SPIDriver *spip, size_t n,
                 const void *txbuf, void *rxbuf) {
  msg_t msg;

  osalSysLock();

  msg = spiStartExchangeI(spip, n, txbuf, rxbuf);
  if (msg == MSG_OK) {
    msg = spiSynchronizeS(spip, TIME_INFINITE);
  }

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This synchronous function performs a transmit operation.
 * @pre     In order to use this function the option @p SPI_USE_SYNCHRONIZATION
 *          must be enabled.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 * @note    If the configured callback starts another transfer before this
 *          function returns then the behavior is undefined.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] n                 number of words to send
 * @param[in] txbuf             the pointer to the transmit buffer
 * @return                      The operation status.
 * @retval MSG_OK               if operation completed without errors.
 * @retval MSG_TIMEOUT          if synchronization timed out.
 * @retval MSG_RESET            if the transfer has been stopped.
 *
 * @api
 */
msg_t spiSend(SPIDriver *spip, size_t n, const void *txbuf) {
  msg_t msg;

  osalSysLock();

  msg = spiStartSendI(spip, n, txbuf);
  if (msg == MSG_OK) {
    msg = spiSynchronizeS(spip, TIME_INFINITE);
  }

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This synchronous function performs a receive operation.
 * @pre     In order to use this function the option @p SPI_USE_SYNCHRONIZATION
 *          must be enabled.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 * @note    If the configured callback starts another transfer before this
 *          function returns then the behavior is undefined.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] n                 number of words to receive
 * @param[out] rxbuf            the pointer to the receive buffer
 * @return                      The operation status.
 * @retval MSG_OK               if operation completed without errors.
 * @retval MSG_TIMEOUT          if synchronization timed out.
 * @retval MSG_RESET            if the transfer has been stopped.
 *
 * @api
 */
msg_t spiReceive(SPIDriver *spip, size_t n, void *rxbuf) {
  msg_t msg;

  osalSysLock();

  msg = spiStartReceiveI(spip, n, rxbuf);
  if (msg == MSG_OK) {
    msg = spiSynchronizeS(spip, TIME_INFINITE);
  }

  osalSysUnlock();

  return msg;
}
#endif /* SPI_USE_SYNCHRONIZATION == TRUE */

#if (SPI_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Gains exclusive access to the SPI bus.
 * @details This function tries to gain ownership to the SPI bus, if the bus
 *          is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option @p SPI_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 *
 * @api
 */
void spiAcquireBus(SPIDriver *spip) {

  osalDbgCheck(spip != NULL);

  osalMutexLock(&spip->mutex);
}

/**
 * @brief   Releases exclusive access to the SPI bus.
 * @pre     In order to use this function the option @p SPI_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 *
 * @api
 */
void spiReleaseBus(SPIDriver *spip) {

  osalDbgCheck(spip != NULL);

  osalMutexUnlock(&spip->mutex);
}
#endif /* SPI_USE_MUTUAL_EXCLUSION == TRUE */

#endif /* HAL_USE_SPI == TRUE */

/** @} */
