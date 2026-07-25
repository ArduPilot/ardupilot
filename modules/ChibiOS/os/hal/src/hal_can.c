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
 * @file    hal_can.c
 * @brief   CAN Driver code.
 *
 * @addtogroup CAN
 * @{
 */

#include "hal.h"

#if (HAL_USE_CAN == TRUE) || defined(__DOXYGEN__)

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
 * @brief   CAN Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void canInit(void) {

  can_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p CANDriver structure.
 *
 * @param[out] canp     pointer to the @p CANDriver object
 *
 * @init
 */
void canObjectInit(CANDriver *canp) {

  canp->state       = CAN_STOP;
  canp->config      = NULL;
  osalThreadQueueObjectInit(&canp->txqueue);
  osalThreadQueueObjectInit(&canp->rxqueue);
#if CAN_ENFORCE_USE_CALLBACKS == FALSE
  osalEventObjectInit(&canp->rxfull_event);
  osalEventObjectInit(&canp->txempty_event);
  osalEventObjectInit(&canp->error_event);
#if CAN_USE_SLEEP_MODE == TRUE
  osalEventObjectInit(&canp->sleep_event);
  osalEventObjectInit(&canp->wakeup_event);
#endif
#else /* CAN_ENFORCE_USE_CALLBACKS == TRUE */
  canp->rxfull_cb   = NULL;
  canp->txempty_cb  = NULL;
  canp->error_cb    = NULL;
#if CAN_USE_SLEEP_MODE == TRUE
  canp->wakeup_cb   = NULL;
#endif
#endif /* CAN_ENFORCE_USE_CALLBACKS == TRUE */
}

/**
 * @brief   Configures and activates the CAN peripheral.
 * @note    Activating the CAN bus can be a slow operation.
 * @note    Unlike other drivers it is not possible to restart the CAN
 *          driver without first stopping it using canStop().
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] config    pointer to the @p CANConfig object. Depending on
 *                      the implementation the value can be @p NULL.
 * @return              The operation status.
 *
 * @api
 */
msg_t canStart(CANDriver *canp, const CANConfig *config) {
  msg_t msg;

  osalDbgCheck(canp != NULL);

  osalSysLock();
  osalDbgAssert(canp->state == CAN_STOP, "invalid state");

  /* Entering initialization mode. */
  canp->state = CAN_STARTING;
  canp->config = config;

  /* Low level initialization, could be a slow process and sleeps could
     be performed inside.*/
#if defined(CAN_LLD_ENHANCED_API)
  msg = can_lld_start(canp);
  if (msg == HAL_RET_SUCCESS) {
    canp->state = CAN_READY;
  }
  else {
    canp->state = CAN_STOP;
  }
#else
  can_lld_start(canp);
  canp->state = CAN_READY;
  msg = HAL_RET_SUCCESS;
#endif

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @api
 */
void canStop(CANDriver *canp) {

  osalDbgCheck(canp != NULL);

  osalSysLock();
  osalDbgAssert((canp->state == CAN_STOP) || (canp->state == CAN_READY),
                "invalid state");

  /* The low level driver is stopped.*/
  can_lld_stop(canp);
  canp->config = NULL;
  canp->state  = CAN_STOP;

  /* Threads waiting on CAN APIs are notified that the driver has been
     stopped in order to not have stuck threads.*/
  osalThreadDequeueAllI(&canp->rxqueue, MSG_RESET);
  osalThreadDequeueAllI(&canp->txqueue, MSG_RESET);
  osalOsRescheduleS();
  osalSysUnlock();
}

/**
 * @brief   Can frame transmission attempt.
 * @details The specified frame is queued for transmission, if the hardware
 *          queue is full then the function fails.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 * @return              The operation result.
 * @retval false        Frame transmitted.
 * @retval true         Mailbox full.
 *
 * @iclass
 */
bool canTryTransmitI(CANDriver *canp,
                     canmbx_t mailbox,
                     const CANTxFrame *ctfp) {

  osalDbgCheckClassI();
  osalDbgCheck((canp != NULL) && (ctfp != NULL) &&
               (mailbox <= (canmbx_t)CAN_TX_MAILBOXES));
  osalDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
                "invalid state");

  /* If the RX mailbox is full then the function fails.*/
  if (!can_lld_is_tx_empty(canp, mailbox)) {
    return true;
  }

  /* Transmitting frame.*/
  can_lld_transmit(canp, mailbox, ctfp);

  return false;
}

/**
 * @brief   Can frame receive attempt.
 * @details The function tries to fetch a frame from a mailbox.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 * @param[out] crfp     pointer to the buffer where the CAN frame is copied
 * @return              The operation result.
 * @retval false        Frame fetched.
 * @retval true         Mailbox empty.
 *
 * @iclass
 */
bool canTryReceiveI(CANDriver *canp,
                    canmbx_t mailbox,
                    CANRxFrame *crfp) {

  osalDbgCheckClassI();
  osalDbgCheck((canp != NULL) && (crfp != NULL) &&
               (mailbox <= (canmbx_t)CAN_RX_MAILBOXES));
  osalDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
                "invalid state");

  /* If the RX mailbox is empty then the function fails.*/
  if (!can_lld_is_rx_nonempty(canp, mailbox)) {
    return true;
  }

  /* Fetching the frame.*/
  can_lld_receive(canp, mailbox, crfp);

  return false;
}

/**
 * @brief   Tries to abort an ongoing transmission.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number
 *
 * @xclass
 */
void canTryAbortX(CANDriver *canp,
                  canmbx_t mailbox) {

  osalDbgCheck((canp != NULL) &&
               (mailbox != CAN_ANY_MAILBOX) &&
               (mailbox <= (canmbx_t)CAN_TX_MAILBOXES));

  can_lld_abort(canp, mailbox);
}

/**
 * @brief   Can frame transmission.
 * @details The specified frame is queued for transmission, if the hardware
 *          queue is full then the invoking thread is queued.
 * @note    Trying to transmit while in sleep mode simply enqueues the thread.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 * @return              The operation result.
 * @retval MSG_OK       the frame has been queued for transmission.
 * @retval MSG_TIMEOUT  The operation has timed out.
 * @retval MSG_RESET    The driver has been stopped while waiting.
 *
 * @api
 */
msg_t canTransmitTimeout(CANDriver *canp,
                         canmbx_t mailbox,
                         const CANTxFrame *ctfp,
                         sysinterval_t timeout) {

  osalDbgCheck((canp != NULL) && (ctfp != NULL) &&
               (mailbox <= (canmbx_t)CAN_TX_MAILBOXES));

  osalSysLock();
  osalDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
                "invalid state");

  /*lint -save -e9007 [13.5] Right side is supposed to be pure.*/
  while ((canp->state == CAN_SLEEP) || !can_lld_is_tx_empty(canp, mailbox)) {
  /*lint -restore*/
   msg_t msg = osalThreadEnqueueTimeoutS(&canp->txqueue, timeout);
    if (msg != MSG_OK) {
      osalSysUnlock();
      return msg;
    }
  }
  can_lld_transmit(canp, mailbox, ctfp);
  osalSysUnlock();
  return MSG_OK;
}

/**
 * @brief   Can frame receive.
 * @details The function waits until a frame is received.
 * @note    Trying to receive while in sleep mode simply enqueues the thread.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 * @param[out] crfp     pointer to the buffer where the CAN frame is copied
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout (useful in an
 *                        event driven scenario where a thread never blocks
 *                        for I/O).
 *                      - @a TIME_INFINITE no timeout.
 * @return              The operation result.
 * @retval MSG_OK       a frame has been received and placed in the buffer.
 * @retval MSG_TIMEOUT  The operation has timed out.
 * @retval MSG_RESET    The driver has been stopped while waiting.
 *
 * @api
 */
msg_t canReceiveTimeout(CANDriver *canp,
                        canmbx_t mailbox,
                        CANRxFrame *crfp,
                        sysinterval_t timeout) {

  osalDbgCheck((canp != NULL) && (crfp != NULL) &&
               (mailbox <= (canmbx_t)CAN_RX_MAILBOXES));

  osalSysLock();
  osalDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
                "invalid state");

  /*lint -save -e9007 [13.5] Right side is supposed to be pure.*/
  while ((canp->state == CAN_SLEEP) || !can_lld_is_rx_nonempty(canp, mailbox)) {
  /*lint -restore*/
    msg_t msg = osalThreadEnqueueTimeoutS(&canp->rxqueue, timeout);
    if (msg != MSG_OK) {
      osalSysUnlock();
      return msg;
    }
  }
  can_lld_receive(canp, mailbox, crfp);
  osalSysUnlock();
  return MSG_OK;
}

#if (CAN_USE_SLEEP_MODE == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Enters the sleep mode.
 * @details This function puts the CAN driver in sleep mode and broadcasts
 *          the @p sleep_event event source.
 * @pre     In order to use this function the option @p CAN_USE_SLEEP_MODE must
 *          be enabled and the @p CAN_SUPPORTS_SLEEP mode must be supported
 *          by the low level driver.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @api
 */
void canSleep(CANDriver *canp) {

  osalDbgCheck(canp != NULL);

  osalSysLock();
  osalDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
                "invalid state");
  if (canp->state == CAN_READY) {
    can_lld_sleep(canp);
    canp->state = CAN_SLEEP;
#if CAN_ENFORCE_USE_CALLBACKS == FALSE
    osalEventBroadcastFlagsI(&canp->sleep_event, (eventflags_t)0);
    osalOsRescheduleS();
#endif
  }
  osalSysUnlock();
}

/**
 * @brief   Enforces leaving the sleep mode.
 * @note    The sleep mode is supposed to be usually exited automatically by
 *          an hardware event.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 */
void canWakeup(CANDriver *canp) {

  osalDbgCheck(canp != NULL);

  osalSysLock();
  osalDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
                "invalid state");
  if (canp->state == CAN_SLEEP) {
    can_lld_wakeup(canp);
    canp->state = CAN_READY;
#if CAN_ENFORCE_USE_CALLBACKS == FALSE
    osalEventBroadcastFlagsI(&canp->wakeup_event, (eventflags_t)0);
    osalOsRescheduleS();
#endif
  }
  osalSysUnlock();
}
#endif /* CAN_USE_SLEEP_MODE == TRUE */

#endif /* HAL_USE_CAN == TRUE */

/** @} */
