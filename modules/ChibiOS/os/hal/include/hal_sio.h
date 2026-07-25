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
 * @file    hal_sio.h
 * @brief   SIO Driver macros and structures.
 *
 * @addtogroup SIO
 * @{
 */

#ifndef HAL_SIO_H
#define HAL_SIO_H

#if (HAL_USE_SIO == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    SIO masks offsets
 * @{
 */
#define SIO_EV_RXNOTEMPY_POS            2       /* CHN_INPUT_AVAILABLE */
#define SIO_EV_TXNOTFULL_POS            3       /* CHN_OUTPUT_EMPTY */
#define SIO_EV_TXDONE_POS               4       /* CHN_TRANSMISSION_END */
#define SIO_EV_ALL_ERRORS_POS           SIO_EV_PARITY_ERR_POS
#define SIO_EV_PARITY_ERR_POS           5       /* CHN_PARITY_ERROR */
#define SIO_EV_FRAMING_ERR_POS          6       /* CHN_FRAMING_ERROR */
#define SIO_EV_NOISE_ERR_POS            7       /* CHN_NOISE_ERROR */
#define SIO_EV_OVERRUN_ERR_POS          8       /* CHN_OVERRUN_ERROR */
#define SIO_EV_RXIDLE_POS               9       /* CHN_IDLE_DETECTED */
#define SIO_EV_RXBREAK_POS              10      /* CHN_BREAK_DETECTED */
/** @} */

/**
 * @name    Event flags (compatible with channel and serial events)
 * @{
 */
/*lint -save -e9053 [12.2] MISRA seems to assume that the underlying type
  is 8 bits wide, it is not.*/
#define SIO_EV_NONE                     0U
#define SIO_EV_RXNOTEMPY                (1U << SIO_EV_RXNOTEMPY_POS)
#define SIO_EV_TXNOTFULL                (1U << SIO_EV_TXNOTFULL_POS)
#define SIO_EV_ALL_DATA                 (SIO_EV_RXNOTEMPY | SIO_EV_TXNOTFULL)
#define SIO_EV_TXDONE                   (1U << SIO_EV_TXDONE_POS)
#define SIO_EV_PARITY_ERR               (1U << SIO_EV_PARITY_ERR_POS)
#define SIO_EV_FRAMING_ERR              (1U << SIO_EV_FRAMING_ERR_POS)
#define SIO_EV_NOISE_ERR                (1U << SIO_EV_NOISE_ERR_POS)
#define SIO_EV_OVERRUN_ERR              (1U << SIO_EV_OVERRUN_ERR_POS)
#define SIO_EV_RXIDLE                   (1U << SIO_EV_RXIDLE_POS)
#define SIO_EV_RXBREAK                  (1U << SIO_EV_RXBREAK_POS)
#define SIO_EV_ALL_ERRORS               (SIO_EV_PARITY_ERR  |               \
                                         SIO_EV_FRAMING_ERR |               \
                                         SIO_EV_OVERRUN_ERR |               \
                                         SIO_EV_NOISE_ERR   |               \
                                         SIO_EV_RXBREAK)
#define SIO_EV_ALL_EVENTS               (SIO_EV_ALL_DATA    |               \
                                         SIO_EV_ALL_ERRORS  |               \
                                         SIO_EV_TXDONE      |               \
                                         SIO_EV_RXIDLE)
/*lint -restore*/
/** @} */

/**
 * @name    Additional messages
 * @{
 */
#define SIO_MSG_ERRORS                  (msg_t)1
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    SIO configuration options
 * @{
 */
/**
 * @brief   Default bit rate.
 * @details Configuration parameter, this is the baud rate selected for the
 *          default configuration.
 */
#if !defined(SIO_DEFAULT_BITRATE) || defined(__DOXYGEN__)
#define SIO_DEFAULT_BITRATE                 38400
#endif

/**
 * @brief   Support for thread synchronization API.
 */
#if !defined(SIO_USE_SYNCHRONIZATION) || defined(__DOXYGEN__)
#define SIO_USE_SYNCHRONIZATION             TRUE
#endif

/**
 * @brief   Support for streams interfacwe.
 */
#if !defined(SIO_USE_STREAMS_INTERFACE) || defined(__DOXYGEN__)
#define SIO_USE_STREAMS_INTERFACE           SIO_USE_SYNCHRONIZATION
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (SIO_USE_STREAMS_INTERFACE == TRUE) && (SIO_USE_SYNCHRONIZATION == FALSE)
#error "SIO_USE_STREAMS_INTERFACE requires SIO_USE_SYNCHRONIZATION"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of event flags.
 */
typedef eventflags_t sioevents_t;

/**
 * @brief   Type of structure representing a SIO driver.
 */
typedef struct hal_sio_driver SIODriver;

/**
 * @brief   Type of structure representing a SIO configuration.
 */
typedef struct hal_sio_config SIOConfig;

/**
 * @brief   Generic SIO notification callback type.
 *
 * @param[in] siop     pointer to the @p SIODriver object
 */
typedef void (*siocb_t)(SIODriver *siop);

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  SIO_UNINIT = 0,                   /**< Not initialized.                   */
  SIO_STOP = 1,                     /**< Stopped.                           */
  SIO_READY = 2                     /**< Ready.                             */
} siostate_t;

#include "hal_sio_lld.h"

/**
 * @brief   Driver configuration structure.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
struct hal_sio_config {
  /* End of the mandatory fields.*/
  sio_lld_config_fields
#if defined(SIO_CONFIG_EXT_FIELDS)
  SIO_CONFIG_EXT_FIELDS
#endif
};

/**
 * @brief   @p SIODriver specific methods.
 */
#define _sio_driver_methods                                                 \
  _base_channel_methods

/**
 * @extends BaseChannelVMT
 *
 * @brief   @p SIODriver virtual methods table.
 */
struct sio_driver_vmt {
  _sio_driver_methods
};

/**
 * @brief   Structure representing a SIO driver.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
struct hal_sio_driver {
#if (SIO_USE_STREAMS_INTERFACE == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Virtual Methods Table.
   */
  const struct sio_driver_vmt *vmt;
#endif
  /**
   * @brief   Driver state.
   */
  siostate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const SIOConfig           *config;
  /**
   * @brief   Enabled event flags.
   */
  sioevents_t               enabled;
  /**
   * @brief   Events callback.
   * @note    Can be @p NULL.
   */
  siocb_t                   cb;
  /**
   * @brief   User argument.
   * @note    Can be retrieved through the @p siop argument of the callback.
   */
  void                      *arg;
#if (SIO_USE_SYNCHRONIZATION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Synchronization point for RX.
   */
  thread_reference_t        sync_rx;
  /**
   * @brief   Synchronization point for RX idle.
   */
  thread_reference_t        sync_rxidle;
  /**
   * @brief   Synchronization point for TX.
   */
  thread_reference_t        sync_tx;
  /**
   * @brief   Synchronization point for TX-end.
   */
  thread_reference_t        sync_txend;
#endif /* SIO_USE_SYNCHRONIZATION == TRUE */
#if defined(SIO_DRIVER_EXT_FIELDS)
  SIO_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  sio_lld_driver_fields
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Associates a callback to the SIO instance.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] f         callback to be associated
 */
#define sioSetCallbackX(siop, f) (siop)->cb = (f)

/**
 * @brief   Determines the state of the RX FIFO.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The RX FIFO state.
 * @retval false        if RX FIFO is not empty.
 * @retval true         if RX FIFO is empty.
 *
 * @xclass
 */
#define sioIsRXEmptyX(siop) sio_lld_is_rx_empty(siop)

/**
 * @brief   Determines the activity state of the receiver.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The RX activity state.
 * @retval false        if RX is in active state.
 * @retval true         if RX is in idle state.
 *
 * @xclass
 */
#define sioIsRXIdleX(siop) sio_lld_is_rx_idle(siop)

/**
 * @brief   Determines if RX has pending errors to be read and cleared.
 * @note    Only errors are handled, data and idle events are not considered.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The RX error events.
 * @retval false        if RX has no pending events.
 * @retval true         if RX has pending events.
 *
 * @xclass
 */
#define sioHasRXErrorsX(siop)  sio_lld_has_rx_errors(siop)

/**
 * @brief   Determines the state of the TX FIFO.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The TX FIFO state.
 * @retval false        if TX FIFO is not full.
 * @retval true         if TX FIFO is full.
 *
 * @xclass
 */
#define sioIsTXFullX(siop) sio_lld_is_tx_full(siop)

/**
 * @brief   Determines the transmission state.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The TX FIFO state.
 * @retval false        if transmission is over.
 * @retval true         if transmission is ongoing.
 *
 * @xclass
 */
#define sioIsTXOngoingX(siop) sio_lld_is_tx_ongoing(siop)

/**
 * @brief   Writes the enabled events mask.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] mask      enabled events mask to be written
 *
 * @xclass
 */
#define sioWriteEnableFlagsX(siop, mask) do {                               \
  (siop)->enabled = (mask);                                                 \
  sio_lld_update_enable_flags(siop);                                        \
} while (false)

/**
 * @brief   Sets flags into the enabled events flags mask.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] mask      enabled events mask to be set
 *
 * @xclass
 */
#define sioSetEnableFlagsX(siop, mask) do {                                 \
  (siop)->enabled |= (mask);                                                \
  sio_lld_update_enable_flags(siop);                                        \
} while (false)

/**
 * @brief   Clears flags from the enabled events flags mask.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] mask      enabled events mask to be cleared
 *
 * @xclass
 */
#define sioClearEnableFlagsX(siop, mask) do {                               \
  (siop)->enabled &= ~(mask);                                               \
  sio_lld_update_enable_flags(siop);                                        \
} while (false)

/**
 * @brief   Return the enabled condition flags mask.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @xclass
 */
#define sioGetEnableFlagsX(siop) (siop)->enabled

/**
 * @brief   Get and clears SIO error event flags.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The pending error event flags.
 *
 * @xclass
 */
#define sioGetAndClearErrorsX(siop) sio_lld_get_and_clear_errors(siop)

/**
 * @brief   Get and clears SIO event flags.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The pending event flags.
 *
 * @xclass
 */
#define sioGetAndClearEventsX(siop) sio_lld_get_and_clear_events(siop)

/**
 * @brief   Returns the pending SIO event flags.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The pending event flags.
 *
 * @xclass
 */
#define sioGetEventsX(siop) sio_lld_get_events(siop)

/**
 * @brief   Returns one frame from the RX FIFO.
 * @note    If the FIFO is empty then the returned value is unpredictable.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The frame from RX FIFO.
 *
 * @xclass
 */
#define sioGetX(siop) sio_lld_get(siop)

/**
 * @brief   Pushes one frame into the TX FIFO.
 * @note    If the FIFO is full then the behavior is unpredictable.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] data      frame to be written
 *
 * @xclass
 */
#define sioPutX(siop, data) sio_lld_put(siop, data)

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
 * @xclass
 */
#define sioAsyncReadX(siop, buffer, n) sio_lld_read(siop, buffer, n)

/**
 * @brief   Writes data into the TX FIFO.
 * @details This function is non-blocking, data is written if there is space
 *          in the FIFO and the effective amount is returned.
 * @note    This function can be called from any context but it is meant to
 *          be called from the @p txnf_cb callback handler.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] buffer    buffer containing the data to be transmitted
 * @param[in] n         maximum number of frames to read
 * @return              The number of transmitted frames.
 *
 * @xclass
 */
#define sioAsyncWriteX(siop, buffer, n) sio_lld_write(siop, buffer, n)

/**
 * @brief   Control operation on a serial port.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] operation control operation code
 * @param[in,out] arg   operation argument
 *
 * @return              The control operation status.
 * @retval MSG_OK       in case of success.
 * @retval MSG_TIMEOUT  in case of operation timeout.
 * @retval MSG_RESET    in case of operation reset.
 *
 * @xclass
 */
#define sioControlX(siop, operation, arg) sio_lld_control(siop, operation, arg)

/**
 * @name    Low level driver helper macros
 * @{
 */
/**
 * @brief   SIO callback.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @notapi
 */
#define __sio_callback(siop) do {                                           \
  if ((siop)->cb != NULL) {                                                 \
    (siop)->cb(siop);                                                       \
  }                                                                         \
} while (false)

#if (SIO_USE_SYNCHRONIZATION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Wakes up because RX errors.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @notapi
 */
#define __sio_wakeup_errors(siop) do {                                      \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(siop)->sync_rx, SIO_MSG_ERRORS);                      \
  osalThreadResumeI(&(siop)->sync_rxidle, SIO_MSG_ERRORS);                  \
  osalSysUnlockFromISR();                                                   \
} while (false)

/**
 * @brief   Wakes up the RX-waiting thread.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @notapi
 */
#define __sio_wakeup_rx(siop) do {                                          \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(siop)->sync_rx, MSG_OK);                              \
  osalSysUnlockFromISR();                                                   \
} while (false)

/**
 * @brief   Wakes up the RX-idle-waiting thread.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @notapi
 */
#define __sio_wakeup_rxidle(siop) do {                                      \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(siop)->sync_rxidle, MSG_OK);                          \
  osalSysUnlockFromISR();                                                   \
} while (false)

/**
 * @brief   Wakes up the TX-waiting thread.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @notapi
 */
#define __sio_wakeup_tx(siop) do {                                          \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(siop)->sync_tx, MSG_OK);                              \
  osalSysUnlockFromISR();                                                   \
} while (false)

/**
 * @brief   Wakes up the TXend-waiting thread.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @notapi
 */
#define __sio_wakeup_txend(siop) do {                                       \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(siop)->sync_txend, MSG_OK);                           \
  osalSysUnlockFromISR();                                                   \
} while (false)
#else /* !SIO_USE_SYNCHRONIZATION */
#define __sio_wakeup_errors(siop)
#define __sio_wakeup_rx(siop)
#define __sio_wakeup_rxidle(siop)
#define __sio_wakeup_tx(siop)
#define __sio_wakeup_txend(siop)
#endif /* !SIO_USE_SYNCHRONIZATION */
/** @} */

/**
 * @brief   Relocates a bit field.
 *
 * @param[in] v         value
 * @param[in] m         mask of the bit field
 * @param[in] s         source bit offset
 * @param[in] d         destination bit offset
 */
#define __sio_reloc_field(v, m, s, d)   ((((v) & m) >> (s)) << (d))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void sioInit(void);
  void sioObjectInit(SIODriver *siop);
  msg_t sioStart(SIODriver *siop, const SIOConfig *config);
  void sioStop(SIODriver *siop);
  void sioWriteEnableFlags(SIODriver *siop, sioevents_t mask);
  void sioSetEnableFlags(SIODriver *siop, sioevents_t mask);
  void sioClearEnableFlags(SIODriver *siop, sioevents_t mask);
  sioevents_t sioGetAndClearErrors(SIODriver *siop);
  sioevents_t sioGetAndClearEvents(SIODriver *siop);
  sioevents_t sioGetEvents(SIODriver *siop);
  size_t sioAsyncRead(SIODriver *siop, uint8_t *buffer, size_t n);
  size_t sioAsyncWrite(SIODriver *siop, const uint8_t *buffer, size_t n);
#if (SIO_USE_SYNCHRONIZATION == TRUE) || defined(__DOXYGEN__)
  msg_t sioSynchronizeRX(SIODriver *siop, sysinterval_t timeout);
  msg_t sioSynchronizeRXIdle(SIODriver *siop, sysinterval_t timeout);
  msg_t sioSynchronizeTX(SIODriver *siop, sysinterval_t timeout);
  msg_t sioSynchronizeTXEnd(SIODriver *siop, sysinterval_t timeout);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SIO == TRUE */

#endif /* HAL_SIO_H */

/** @} */
