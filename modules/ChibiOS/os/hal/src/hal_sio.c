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
 * @file    hal_sio.c
 * @brief   SIO Driver code.
 *
 * @addtogroup SIO
 * @{
 */

#include "hal.h"

#if (HAL_USE_SIO == TRUE) || defined(__DOXYGEN__)

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

#if (SIO_USE_STREAMS_INTERFACE == TRUE) || defined(__DOXYGEN__)
static size_t sync_write(void *ip, const uint8_t *bp, size_t n,
                         sysinterval_t timeout) {
  SIODriver *siop = (SIODriver *)ip;
  size_t i;

  i = 0U;
  while (i < n) {
    size_t written;
    msg_t msg;

    msg = sioSynchronizeTX(siop, timeout);
    if (msg != MSG_OK) {
      break;
    }

    written = sioAsyncWrite(siop, bp, n - i);
    i += written;
    bp += written;
  }
  return i;
}

static size_t sync_read(void *ip, uint8_t *bp, size_t n,
                        sysinterval_t timeout) {
  SIODriver *siop = (SIODriver *)ip;
  size_t i;

  i = 0U;
  while (i < n) {
    size_t read;
    msg_t msg;

    msg = sioSynchronizeRX(siop, timeout);
    if (msg == SIO_MSG_ERRORS) {
      (void) sioGetAndClearErrors(siop);
    }
    else if (msg != MSG_OK) {
      break;
    }

    read = sioAsyncRead(siop, bp, n - i);
    i += read;
    bp += read;
  }
  return i;
}

/*
 * Interface implementation, the following functions just invoke the equivalent
 * queue-level function or macro.
 */

static size_t __write(void *ip, const uint8_t *bp, size_t n) {

  return sync_write(ip, bp, n, TIME_INFINITE);
}

static size_t __read(void *ip, uint8_t *bp, size_t n) {

  return sync_read(ip, bp, n, TIME_INFINITE);
}

static msg_t __put(void *ip, uint8_t b) {
  SIODriver *siop = (SIODriver *)ip;
  msg_t msg;

  msg = sioSynchronizeTX(siop, TIME_INFINITE);
  if (msg != MSG_OK) {
    return msg;
  }

  sioPutX(siop, b);
  return MSG_OK;
}

static msg_t __get(void *ip) {
  SIODriver *siop = (SIODriver *)ip;
  msg_t msg;

  while (true) {
    msg = sioSynchronizeRX(siop, TIME_INFINITE);
    if (msg == SIO_MSG_ERRORS) {
      (void)sioGetAndClearErrors(siop);
      continue;
    }
    if (msg != MSG_OK) {
      return msg;
    }

    return sioGetX(siop);
  }
}

static msg_t __putt(void *ip, uint8_t b, sysinterval_t timeout) {
  SIODriver *siop = (SIODriver *)ip;
  msg_t msg;

  msg = sioSynchronizeTX(siop, timeout);
  if (msg != MSG_OK) {
    return msg;
  }

  sioPutX(siop, b);
  return MSG_OK;
}

static msg_t __gett(void *ip, sysinterval_t timeout) {
  SIODriver *siop = (SIODriver *)ip;
  msg_t msg;

  while (true) {
    msg = sioSynchronizeRX(siop, timeout);
    if (msg == SIO_MSG_ERRORS) {
      (void)sioGetAndClearErrors(siop);
      continue;
    }
    if (msg != MSG_OK) {
      return msg;
    }

    return sioGetX(siop);
  }
}

static size_t __writet(void *ip, const uint8_t *bp, size_t n,
                       sysinterval_t timeout) {

  return sync_write(ip, bp, n, timeout);
}

static size_t __readt(void *ip, uint8_t *bp, size_t n,
                      sysinterval_t timeout) {

  return sync_read(ip, bp, n, timeout);
}

static msg_t __ctl(void *ip, unsigned int operation, void *arg) {
  SIODriver *siop = (SIODriver *)ip;

  osalDbgCheck(siop != NULL);

  switch (operation) {
  case CHN_CTL_NOP:
    osalDbgCheck(arg == NULL);
    break;
  case CHN_CTL_INVALID:
    return HAL_RET_UNKNOWN_CTL;
  default:
    /* Delegating to the LLD if supported.*/
    return sio_lld_control(siop, operation, arg);
  }
  return HAL_RET_SUCCESS;
}

static const struct sio_driver_vmt vmt = {
  (size_t)0,
  __write, __read, __put,    __get,
  __putt,  __gett, __writet, __readt,
  __ctl
};
#endif /* SIO_USE_STREAMS_INTERFACE */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   SIO Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void sioInit(void) {

  sio_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p SIODriver structure.
 *
 * @param[out] siop     pointer to the @p SIODriver object
 *
 * @init
 */
void sioObjectInit(SIODriver *siop) {

#if SIO_USE_STREAMS_INTERFACE == TRUE
  siop->vmt         = &vmt;
#endif
  siop->state       = SIO_STOP;
  siop->config      = NULL;
  siop->enabled     = (sioevents_t)0;
  siop->cb          = NULL;
  siop->arg         = NULL;
#if SIO_USE_SYNCHRONIZATION == TRUE
  siop->sync_rx     = NULL;
  siop->sync_rxidle = NULL;
  siop->sync_tx     = NULL;
  siop->sync_txend  = NULL;
#endif

  /* Optional, user-defined initializer.*/
#if defined(SIO_DRIVER_EXT_INIT_HOOK)
  SIO_DRIVER_EXT_INIT_HOOK(siop);
#endif
}

/**
 * @brief   Configures and activates the SIO peripheral.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] config    pointer to the @p SIOConfig object, can be @p NULL
 *                      if the default configuration is desired
 * @return              The operation status.
 *
 * @api
 */
msg_t sioStart(SIODriver *siop, const SIOConfig *config) {
  msg_t msg;

  osalDbgCheck(siop != NULL);

  osalSysLock();

  osalDbgAssert((siop->state == SIO_STOP) || (siop->state == SIO_READY),
                "invalid state");
  siop->config = config;

  msg = sio_lld_start(siop);
  if (msg == HAL_RET_SUCCESS) {
    siop->state = SIO_READY;
  }
  else {
    siop->state = SIO_STOP;
  }

  /*lint -save -e9053 [12.2] MISRA seems to assume that the underlying type
    is 8 bits wide, it is not.*/
#if SIO_USE_SYNCHRONIZATION == TRUE
  /* If synchronization is enabled then all events by default.*/
  sioWriteEnableFlagsX(siop, (sioevents_t)SIO_EV_ALL_EVENTS);
#else
  /* If synchronization is disabled then no events by default.*/
  sioWriteEnableFlagsX(siop, (sioevents_t)SIO_EV_ALL_EVENTS);
  sioWriteEnableFlagsX(siop, (sioevents_t)SIO_EV_NONE);
#endif
  /*lint -restore*/

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Deactivates the SIO peripheral.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @api
 */
void sioStop(SIODriver *siop) {

  osalDbgCheck(siop != NULL);

  osalSysLock();

  osalDbgAssert((siop->state == SIO_STOP) || (siop->state == SIO_READY),
                "invalid state");

  sio_lld_stop(siop);
  siop->state   = SIO_STOP;
  siop->config  = NULL;
  siop->cb      = NULL;
  siop->arg     = NULL;

#if SIO_USE_SYNCHRONIZATION == TRUE
    /* Informing waiting threads, if any.*/
    osalThreadResumeI(&siop->sync_rx, MSG_RESET);
    osalThreadResumeI(&siop->sync_rxidle, MSG_RESET);
    osalThreadResumeI(&siop->sync_tx, MSG_RESET);
    osalThreadResumeI(&siop->sync_txend, MSG_RESET);
    osalOsRescheduleS();
#endif

  osalSysUnlock();
}

/**
 * @brief   Writes the enabled events flags mask.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] mask     enabled events mask to be written
 *
 * @api
 */
void sioWriteEnableFlags(SIODriver *siop, sioevents_t mask) {

  osalDbgCheck(siop != NULL);

  osalSysLock();

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  sioWriteEnableFlagsX(siop, mask);

  osalSysUnlock();
}

/**
 * @brief   Sets flags into the enabled events flags mask.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] mask     enabled events mask to be set
 *
 * @api
 */
void sioSetEnableFlags(SIODriver *siop, sioevents_t mask) {

  osalDbgCheck(siop != NULL);

  osalSysLock();

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  sioSetEnableFlagsX(siop, mask);

  osalSysUnlock();
}

/**
 * @brief   Clears flags from the enabled events flags mask.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] mask     enabled events mask to be cleared
 *
 * @api
 */
void sioClearEnableFlags(SIODriver *siop, sioevents_t mask) {

  osalDbgCheck(siop != NULL);

  osalSysLock();

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  sioClearEnableFlagsX(siop, mask);

  osalSysUnlock();
}

/**
 * @brief   Get and clears SIO error event flags.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The pending error event flags.
 *
 * @api
 */
sioevents_t sioGetAndClearErrors(SIODriver *siop) {
  sioevents_t errors;

  osalDbgCheck(siop != NULL);

  osalSysLock();

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  errors = sioGetAndClearErrorsX(siop);

  osalSysUnlock();

  return errors;
}

/**
 * @brief   Get and clears SIO event flags.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The pending event flags.
 *
 * @api
 */
sioevents_t sioGetAndClearEvents(SIODriver *siop) {
  sioevents_t events;

  osalDbgCheck(siop != NULL);

  osalSysLock();

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  events = sioGetAndClearEventsX(siop);

  osalSysUnlock();

  return events;
}

/**
 * @brief   Returns the pending SIO event flags.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The pending event flags.
 *
 * @api
 */
sioevents_t sioGetEvents(SIODriver *siop) {
  sioevents_t events;

  osalDbgCheck(siop != NULL);

  osalSysLock();

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  events = sioGetEventsX(siop);

  osalSysUnlock();

  return events;
}

/**
 * @brief   Reads data from the RX FIFO.
 * @details This function is non-blocking, data is read if present and the
 *          effective amount is returned.
 * @note    This function can be called from any context but it is meant to
 *          be called from the @p rxne_cb callback handler.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] buffer    buffer for the received data
 * @param[in] n         maximum number of frames to read
 * @return              The number of received frames.
 *
 * @api
 */
size_t sioAsyncRead(SIODriver *siop, uint8_t *buffer, size_t n) {

  osalDbgCheck((siop != NULL) && (buffer != NULL));

  osalSysLock();

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  n = sioAsyncReadX(siop, buffer, n);

  osalSysUnlock();

  return n;
}

/**
 * @brief   Writes data into the TX FIFO.
 * @details This function is non-blocking, data is written if there is space
 *          in the FIFO and the effective amount is returned.
 * @note    This function can be called from any context but it is meant to
 *          be called from the @p txnf_cb callback handler.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[out] buffer   buffer containing the data to be transmitted
 * @param[in] n         maximum number of frames to read
 * @return              The number of transmitted frames.
 *
 * @api
 */
size_t sioAsyncWrite(SIODriver *siop, const uint8_t *buffer, size_t n) {

  osalDbgCheck((siop != NULL) && (buffer != NULL));

  osalSysLock();

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  n = sioAsyncWriteX(siop, buffer, n);

  osalSysUnlock();

  return n;
}

#if (SIO_USE_SYNCHRONIZATION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Synchronizes with RX FIFO data availability.
 * @note    The exact behavior depends on low level FIFO settings such
 *          as thresholds, etc.
 * @note    This function can only be called by a single thread at time.
 *
 * @param[in] siop          pointer to an @p SIODriver structure
 * @param[in] timeout       synchronization timeout
 * @return                  The synchronization result.
 * @retval MSG_OK           if there is data in the RX FIFO.
 * @retval MSG_TIMEOUT      if synchronization timed out.
 * @retval MSG_RESET        it the operation has been stopped while waiting.
 * @retval SIO_MSG_ERRORS   if RX errors occurred before or during wait.
 *
 * @api
 */
msg_t sioSynchronizeRX(SIODriver *siop, sysinterval_t timeout) {
  msg_t msg;

  osalDbgCheck(siop != NULL);

  osalSysLock();

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  /* Checking for errors before going to sleep.*/
  if (sioHasRXErrorsX(siop)) {
    osalSysUnlock();
    return SIO_MSG_ERRORS;
  }

  msg = MSG_OK;
  /*lint -save -e506 -e681 [2.1] Silencing this error because it is
    tested with a template implementation of sio_lld_is_rx_empty() which
    is constant.*/
  while (sioIsRXEmptyX(siop)) {
  /*lint -restore*/
    msg = osalThreadSuspendTimeoutS(&siop->sync_rx, timeout);
    if (msg != MSG_OK) {
      break;
    }
  }

  osalSysUnlock();

  return msg;
}
/**
 * @brief   Synchronizes with RX going idle.
 * @note    This function can only be called by a single thread at time.
 *
 * @param[in] siop          pointer to an @p SIODriver structure
 * @param[in] timeout       synchronization timeout
 * @return                  The synchronization result.
 * @retval MSG_OK           if RW went idle.
 * @retval MSG_TIMEOUT      if synchronization timed out.
 * @retval MSG_RESET        it the operation has been stopped while waiting.
 * @retval SIO_MSG_ERRORS   if RX errors occurred before or during wait.
 *
 * @api
 */
msg_t sioSynchronizeRXIdle(SIODriver *siop, sysinterval_t timeout) {
  msg_t msg;

  osalDbgCheck(siop != NULL);

  osalSysLock();

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  /* Checking for errors before going to sleep.*/
  if (sioHasRXErrorsX(siop)) {
    osalSysUnlock();
    return SIO_MSG_ERRORS;
  }

  msg = MSG_OK;
  /*lint -save -e506 -e681 [2.1] Silencing this error because it is
    tested with a template implementation of sio_lld_is_rx_empty() which
    is constant.*/
  while (!sioIsRXIdleX(siop)) {
  /*lint -restore*/
    msg = osalThreadSuspendTimeoutS(&siop->sync_rxidle, timeout);
    if (msg != MSG_OK) {
      break;
    }
  }

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Synchronizes with TX FIFO space availability.
 * @note    The exact behavior depends on low level FIFO settings such
 *          as thresholds, etc.
 * @note    This function can only be called by a single thread at time.
 *
 * @param[in] siop          pointer to an @p SIODriver structure
 * @param[in] timeout       synchronization timeout
 * @return                  The synchronization result.
 * @retval MSG_OK           if there is space in the TX FIFO.
 * @retval MSG_TIMEOUT      if synchronization timed out.
 * @retval MSG_RESET        operation has been stopped while waiting.
 *
 * @api
 */
msg_t sioSynchronizeTX(SIODriver *siop, sysinterval_t timeout) {
  msg_t msg;

  osalDbgCheck(siop != NULL);

  osalSysLock();

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  msg = MSG_OK;
  /*lint -save -e506 -e681 [2.1] Silencing this error because it is
    tested with a template implementation of sio_lld_is_tx_full() which
    is constant.*/
  while (sioIsTXFullX(siop)) {
  /*lint -restore*/
    msg = osalThreadSuspendTimeoutS(&siop->sync_tx, timeout);
    if (msg != MSG_OK) {
      break;
    }
  }

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Synchronizes with TX completion.
 * @note    This function can only be called by a single thread at time.
 *
 * @param[in] siop          pointer to an @p SIODriver structure
 * @param[in] timeout       synchronization timeout
 * @return                  The synchronization result.
 * @retval MSG_OK           if TX operation finished.
 * @retval MSG_TIMEOUT      if synchronization timed out.
 *
 * @api
 */
msg_t sioSynchronizeTXEnd(SIODriver *siop, sysinterval_t timeout) {
  msg_t msg;

  osalDbgCheck(siop != NULL);

  osalSysLock();

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  /*lint -save -e506 -e774 [2.1, 14.3] Silencing this error because
    it is tested with a template implementation of sio_lld_is_tx_ongoing()
    which is constant.*/
  if (sioIsTXOngoingX(siop)) {
  /*lint -restore*/
    msg = osalThreadSuspendTimeoutS(&siop->sync_txend, timeout);
  }
  else {
    msg = MSG_OK;
  }

  osalSysUnlock();

  return msg;
}
#endif /* SIO_USE_SYNCHRONIZATION == TRUE */

#endif /* HAL_USE_SIO == TRUE */

/** @} */
