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
 * @file    hal_uart.h
 * @brief   UART Driver macros and structures.
 *
 * @addtogroup UART
 * @{
 */

#ifndef HAL_UART_H
#define HAL_UART_H

#if (HAL_USE_UART == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    UART status flags
 * @{
 */
#define UART_NO_ERROR           0   /**< @brief No pending conditions.      */
#define UART_PARITY_ERROR       4   /**< @brief Parity error happened.      */
#define UART_FRAMING_ERROR      8   /**< @brief Framing error happened.     */
#define UART_OVERRUN_ERROR      16  /**< @brief Overflow happened.          */
#define UART_NOISE_ERROR        32  /**< @brief Noise on the line.          */
#define UART_BREAK_DETECTED     64  /**< @brief Break detected.             */
/** @} */

/**
 * @name    UART error conditions
 * @{
 */
#define UART_ERR_NOT_ACTIVE     (size_t)-1
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    UART configuration options
 * @{
 */
/**
 * @brief   Enables synchronous APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(UART_USE_WAIT) || defined(__DOXYGEN__)
#define UART_USE_WAIT                       FALSE
#endif

/**
 * @brief   Enables the @p uartAcquireBus() and @p uartReleaseBus() APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(UART_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define UART_USE_MUTUAL_EXCLUSION           FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  UART_UNINIT = 0,                  /**< Not initialized.                   */
  UART_STOP = 1,                    /**< Stopped.                           */
  UART_READY = 2                    /**< Ready.                             */
} uartstate_t;

/**
 * @brief   Transmitter state machine states.
 */
typedef enum {
  UART_TX_IDLE = 0,                 /**< Not transmitting.                  */
  UART_TX_ACTIVE = 1                /**< Transmitting.                      */
} uarttxstate_t;

/**
 * @brief   Receiver state machine states.
 */
typedef enum {
  UART_RX_IDLE = 0,                 /**< Not receiving.                     */
  UART_RX_ACTIVE = 1                /**< Receiving.                         */
} uartrxstate_t;

#include "hal_uart_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Low level driver helper macros
 * @{
 */
#if (UART_USE_WAIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Wakes up the waiting thread in case of early TX complete.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
#define _uart_wakeup_tx1_isr(uartp) {                                       \
  if ((uartp)->early == true) {                                             \
    osalSysLockFromISR();                                                   \
    osalThreadResumeI(&(uartp)->threadtx, MSG_OK);                          \
    osalSysUnlockFromISR();                                                 \
  }                                                                         \
}
#else /* !UART_USE_WAIT */
#define _uart_wakeup_tx1_isr(uartp)
#endif /* !UART_USE_WAIT */

#if (UART_USE_WAIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Wakes up the waiting thread in case of late TX complete.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
#define _uart_wakeup_tx2_isr(uartp) {                                       \
  if ((uartp)->early == false) {                                            \
    osalSysLockFromISR();                                                   \
    osalThreadResumeI(&(uartp)->threadtx, MSG_OK);                          \
    osalSysUnlockFromISR();                                                 \
  }                                                                         \
}
#else /* !UART_USE_WAIT */
#define _uart_wakeup_tx2_isr(uartp)
#endif /* !UART_USE_WAIT */

#if (UART_USE_WAIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Wakes up the waiting thread in case of RX complete.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
#define _uart_wakeup_rx_complete_isr(uartp) {                               \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(uartp)->threadrx, MSG_OK);                            \
  osalSysUnlockFromISR();                                                   \
}
#else /* !UART_USE_WAIT */
#define _uart_wakeup_rx_complete_isr(uartp)
#endif /* !UART_USE_WAIT */

#if (UART_USE_WAIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Wakes up the waiting thread in case of RX error.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
#define _uart_wakeup_rx_error_isr(uartp) {                                  \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(uartp)->threadrx, MSG_RESET);                         \
  osalSysUnlockFromISR();                                                   \
}
#else /* !UART_USE_WAIT */
#define _uart_wakeup_rx_error_isr(uartp)
#endif /* !UART_USE_WAIT */

#if (UART_USE_WAIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Wakes up the waiting thread in case of RX character match.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
#define _uart_wakeup_rx_cm_isr(uartp) {                                     \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(uartp)->threadrx, MSG_TIMEOUT);                       \
  osalSysUnlockFromISR();                                                   \
}
#else /* !UART_USE_WAIT */
#define _uart_wakeup_rx_cm_isr(uartp)
#endif /* !UART_USE_WAIT */

#if (UART_USE_WAIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Wakes up the waiting thread in case of RX timeout.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
#define _uart_wakeup_rx_timeout_isr(uartp) {                                \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(uartp)->threadrx, MSG_TIMEOUT);                       \
  osalSysUnlockFromISR();                                                   \
}
#else /* !UART_USE_WAIT */
#define _uart_wakeup_rx_timeout_isr(uartp)
#endif /* !UART_USE_WAIT */

/**
 * @brief   Common ISR code for early TX.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
#define _uart_tx1_isr_code(uartp) {                                         \
  (uartp)->txstate = UART_TX_IDLE;                                          \
  if ((uartp)->config->txend1_cb != NULL) {                                 \
    (uartp)->config->txend1_cb(uartp);                                      \
  }                                                                         \
  _uart_wakeup_tx1_isr(uartp);                                              \
}

/**
 * @brief   Common ISR code for late TX.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
#define _uart_tx2_isr_code(uartp) {                                         \
  if ((uartp)->config->txend2_cb != NULL) {                                 \
    (uartp)->config->txend2_cb(uartp);                                      \
  }                                                                         \
  _uart_wakeup_tx2_isr(uartp);                                              \
}

/**
 * @brief   Common ISR code for RX complete.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
#define _uart_rx_complete_isr_code(uartp) {                                 \
  (uartp)->rxstate = UART_RX_IDLE;                                          \
  uart_enter_rx_idle_loop(uartp);                                           \
  if ((uartp)->config->rxend_cb != NULL) {                                  \
    (uartp)->config->rxend_cb(uartp);                                       \
  }                                                                         \
  _uart_wakeup_rx_complete_isr(uartp);                                      \
}

/**
 * @brief   Common ISR code for RX error.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] errors    mask of errors to be reported
 *
 * @notapi
 */
#define _uart_rx_error_isr_code(uartp, errors) {                            \
  if ((uartp)->config->rxerr_cb != NULL) {                                  \
    (uartp)->config->rxerr_cb(uartp, errors);                               \
  }                                                                         \
  _uart_wakeup_rx_error_isr(uartp);                                         \
}

/**
 * @brief   Common ISR code for RX on idle.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
#define _uart_rx_idle_code(uartp) {                                         \
  if ((uartp)->config->rxchar_cb != NULL)                                   \
    (uartp)->config->rxchar_cb(uartp, (uartp)->rxbuf);                      \
}

/**
 * @brief   Timeout ISR code for receiver.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
#define _uart_timeout_isr_code(uartp) {                                     \
  if ((uartp)->config->timeout_cb != NULL) {                                \
    (uartp)->config->timeout_cb(uartp);                                     \
  }                                                                         \
  _uart_wakeup_rx_timeout_isr(uartp);                                       \
}

/**
 * @brief   Character match ISR code for receiver.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
#define _uart_rx_char_match_isr_code(uartp) {                               \
  if ((uartp)->config->rx_cm_cb != NULL) {                                  \
    (uartp)->config->rx_cm_cb(uartp);                                       \
  }                                                                         \
  _uart_wakeup_rx_cm_isr(uartp);                                            \
}

/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void uartInit(void);
  void uartObjectInit(UARTDriver *uartp);
  msg_t uartStart(UARTDriver *uartp, const UARTConfig *config);
  void uartStop(UARTDriver *uartp);
  void uartStartSend(UARTDriver *uartp, size_t n, const void *txbuf);
  void uartStartSendI(UARTDriver *uartp, size_t n, const void *txbuf);
  size_t uartStopSend(UARTDriver *uartp);
  size_t uartStopSendI(UARTDriver *uartp);
  void uartStartReceive(UARTDriver *uartp, size_t n, void *rxbuf);
  void uartStartReceiveI(UARTDriver *uartp, size_t n, void *rxbuf);
  size_t uartStopReceive(UARTDriver *uartp);
  size_t uartStopReceiveI(UARTDriver *uartp);
#if UART_USE_WAIT == TRUE
  msg_t uartSendTimeout(UARTDriver *uartp, size_t *np,
                        const void *txbuf, sysinterval_t timeout);
  msg_t uartSendFullTimeout(UARTDriver *uartp, size_t *np,
                            const void *txbuf, sysinterval_t timeout);
  msg_t uartReceiveTimeout(UARTDriver *uartp, size_t *np,
                           void *rxbuf, sysinterval_t timeout);
#endif
#if UART_USE_MUTUAL_EXCLUSION == TRUE
  void uartAcquireBus(UARTDriver *uartp);
  void uartReleaseBus(UARTDriver *uartp);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_UART == TRUE */

#endif /* HAL_UART_H */

/** @} */
