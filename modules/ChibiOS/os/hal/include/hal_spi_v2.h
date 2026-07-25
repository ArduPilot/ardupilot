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
 * @file    hal_spi_v2.h
 * @brief   SPI (v2) Driver macros and structures.
 *
 * @addtogroup SPI_V2
 * @{
 */

#ifndef HAL_SPI_V2_H
#define HAL_SPI_V2_H

#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Chip Select modes
 * @{
 */
#define SPI_SELECT_MODE_NONE                0   /** @brief @p spiSelect() and
                                                    @p spiUnselect() do
                                                    nothing.                */
#define SPI_SELECT_MODE_PAD                 1   /** @brief Legacy mode.     */
#define SPI_SELECT_MODE_PORT                2   /** @brief Fastest mode.    */
#define SPI_SELECT_MODE_LINE                3   /** @brief Packed mode.     */
#define SPI_SELECT_MODE_LLD                 4   /** @brief LLD-defined mode.*/
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    SPI configuration options
 * @{
 */
/**
 * @brief   Support for thread synchronization API.
 */
#if !defined(SPI_USE_SYNCHRONIZATION) || defined(__DOXYGEN__)
#if !defined(SPI_USE_WAIT) || defined(__DOXYGEN__)
#define SPI_USE_SYNCHRONIZATION             FALSE
#else
#define SPI_USE_SYNCHRONIZATION             SPI_USE_WAIT
#endif
#endif

/**
 * @brief   Inserts an assertion on function errors before returning.
 */
#if !defined(SPI_USE_ASSERT_ON_ERROR) || defined(__DOXYGEN__)
#define SPI_USE_ASSERT_ON_ERROR             TRUE
#endif

/**
 * @brief   Enables the @p spiAcquireBus() and @p spiReleaseBus() APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(SPI_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define SPI_USE_MUTUAL_EXCLUSION            TRUE
#endif

/**
 * @brief   Handling method for SPI CS line.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(SPI_SELECT_MODE) || defined(__DOXYGEN__)
#define SPI_SELECT_MODE                     SPI_SELECT_MODE_PAD
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (SPI_SELECT_MODE != SPI_SELECT_MODE_NONE) &&                            \
    (SPI_SELECT_MODE != SPI_SELECT_MODE_PAD)  &&                            \
    (SPI_SELECT_MODE != SPI_SELECT_MODE_PORT) &&                            \
    (SPI_SELECT_MODE != SPI_SELECT_MODE_LINE) &&                            \
    (SPI_SELECT_MODE != SPI_SELECT_MODE_LLD)
#error "invalid SPI_SELECT_MODE setting"
#endif

/* Some modes have a dependency on the PAL driver, making the required
   checks here.*/
#if ((SPI_SELECT_MODE == SPI_SELECT_MODE_PAD)  ||                           \
     (SPI_SELECT_MODE == SPI_SELECT_MODE_PORT) ||                           \
     (SPI_SELECT_MODE == SPI_SELECT_MODE_LINE)) &&                          \
    (HAL_USE_PAL != TRUE)
#error "current SPI_SELECT_MODE requires HAL_USE_PAL"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  SPI_UNINIT = 0,                   /**< Not initialized.                   */
  SPI_STOP = 1,                     /**< Stopped.                           */
  SPI_READY = 2,                    /**< Ready.                             */
  SPI_ACTIVE = 3,                   /**< Exchanging data.                   */
  SPI_COMPLETE = 4                  /**< Asynchronous operation complete.   */
} spistate_t;

/**
 * @brief   Type of a structure representing an SPI driver.
 */
typedef struct hal_spi_driver SPIDriver;

/**
 * @brief   Type of a SPI driver configuration structure.
 */
typedef struct hal_spi_config SPIConfig;

/**
 * @brief   SPI notification callback type.
 * @details The callback is invoked from ISR context.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 *                              triggering the callback
 */
typedef void (*spicb_t)(SPIDriver *spip);

/* Including the low level driver header, it exports information required
   for completing types.*/
#include "hal_spi_v2_lld.h"

#if !defined(SPI_SUPPORTS_CIRCULAR)
#error "SPI_SUPPORTS_CIRCULAR not defined in LLD"
#endif

#if !defined(SPI_SUPPORTS_SLAVE_MODE)
#error "SPI_SUPPORTS_SLAVE_MODE not defined in LLD"
#endif

/**
 * @brief   Driver configuration structure.
 */
struct hal_spi_config {
#if (SPI_SUPPORTS_CIRCULAR == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Enables the circular buffer mode.
   */
  bool                      circular;
#endif
#if (SPI_SUPPORTS_SLAVE_MODE == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Enables the slave mode.
   */
  bool                      slave;
#endif
  /**
   * @brief   Operation data callback or @p NULL.
   * @details In linear mode the callback is invoked after the driver returns
   *          to the @p SPI_READY state, this allows chaining another transfer
   *          using I-Class APIs. In circular mode the callback is invoked in
   *          either @p SPI_ACTIVE state (half buffer) or @p SPI_COMPLETE
   *          state (full buffer).
   * @note    If a synchronous API is waiting for completion then starting a
   *          follow-on transfer from this callback is undefined.
   */
  spicb_t                   data_cb;
  /**
   * @brief   Operation error callback or @p NULL.
   */
  spicb_t                   error_cb;
#if (SPI_SELECT_MODE == SPI_SELECT_MODE_LINE) || defined(__DOXYGEN__)
  /**
   * @brief   The chip select line.
   * @note    Only used in master mode.
   */
  ioline_t                  ssline;
#elif SPI_SELECT_MODE == SPI_SELECT_MODE_PORT
  /**
   * @brief   The chip select port.
   * @note    Only used in master mode.
   */
  ioportid_t                ssport;
  /**
   * @brief   The chip select port mask.
   * @note    Only used in master mode.
   */
  ioportmask_t              ssmask;
#elif SPI_SELECT_MODE == SPI_SELECT_MODE_PAD
  /**
   * @brief   The chip select port.
   * @note    Only used in master mode.
   */
  ioportid_t                ssport;
  /**
   * @brief   The chip select pad number.
   * @note    Only used in master mode.
   */
  uint_fast8_t              sspad;
#endif
  /* End of the mandatory fields.*/
  spi_lld_config_fields
};

/**
 * @brief   Structure representing an SPI driver.
 */
struct hal_spi_driver {
  /**
   * @brief Driver state.
   */
  spistate_t                state;
  /**
   * @brief Current configuration data.
   */
  const SPIConfig           *config;
#if (SPI_USE_SYNCHRONIZATION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Synchronization point for transfer.
   */
  thread_reference_t        sync_transfer;
#endif /* SPI_USE_SYNCHRONIZATION == TRUE */
#if (SPI_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the peripheral.
   */
  mutex_t                   mutex;
#endif /* SPI_USE_MUTUAL_EXCLUSION == TRUE */
#if defined(SPI_DRIVER_EXT_FIELDS)
  SPI_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  spi_lld_driver_fields
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Buffer state.
 * @note    This function is meant to be called from the SPI callback only.
 * @note    This state is only meaningful for circular transfers, where it is
 *          used to distinguish the full buffer callback from the half buffer
 *          callback.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @return                      The buffer state.
 * @retval false                if the driver filled/sent the first half of
 *                              the buffer.
 * @retval true                 if the driver filled/sent the second half of
 *                              the buffer.
 *
 * @special
 */
#define spiIsBufferComplete(spip) ((bool)((spip)->state == SPI_COMPLETE))

#if (SPI_SELECT_MODE == SPI_SELECT_MODE_LLD) || defined(__DOXYGEN__)
/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 *
 * @iclass
 */
#define spiSelectI(spip)                                                    \
do {                                                                        \
  spi_lld_select(spip);                                                     \
} while (false)

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 *
 * @iclass
 */
#define spiUnselectI(spip)                                                  \
do {                                                                        \
  spi_lld_unselect(spip);                                                   \
} while (false)

#elif SPI_SELECT_MODE == SPI_SELECT_MODE_LINE
#define spiSelectI(spip)                                                    \
do {                                                                        \
  palClearLine((spip)->config->ssline);                                     \
} while (false)

#define spiUnselectI(spip)                                                  \
do {                                                                        \
  palSetLine((spip)->config->ssline);                                       \
} while (false)

#elif SPI_SELECT_MODE == SPI_SELECT_MODE_PORT
#define spiSelectI(spip)                                                    \
do {                                                                        \
  palClearPort((spip)->config->ssport, (spip)->config->ssmask);             \
} while (false)

#define spiUnselectI(spip)                                                  \
do {                                                                        \
  palSetPort((spip)->config->ssport, (spip)->config->ssmask);               \
} while (false)

#elif SPI_SELECT_MODE == SPI_SELECT_MODE_PAD
#define spiSelectI(spip)                                                    \
do {                                                                        \
  palClearPad((spip)->config->ssport, (spip)->config->sspad);               \
} while (false)

#define spiUnselectI(spip)                                                  \
do {                                                                        \
  palSetPad((spip)->config->ssport, (spip)->config->sspad);                 \
} while (false)

#elif SPI_SELECT_MODE == SPI_SELECT_MODE_NONE
#define spiSelectI(spip)

#define spiUnselectI(spip)
#endif

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 * @note    This API is implemented as a macro in order to minimize latency.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] frame             the data frame to send over the SPI bus
 * @return                      The received data frame from the SPI bus.
 */
#define spiPolledExchange(spip, frame) spi_lld_polled_exchange(spip, frame)

/**
 * @brief   Compatibility API with SPI driver v1.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 *
 * @iclass
 */
#define spiAbortI(spip)         spiStopTransferI(spip, NULL)

/**
 * @brief   Compatibility API with SPI driver v1.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 *
 * @api
 */
#define spiAbort(spip)          spiStopTransfer(spip, NULL)
/** @} */

/**
 * @name    Low level driver helper macros
 * @{
 */
#if (SPI_USE_SYNCHRONIZATION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Wakes up the waiting thread.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] msg       wakeup message
 *
 * @notapi
 */
#define __spi_wakeup_isr(spip, msg) {                                       \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(spip)->sync_transfer, msg);                           \
  osalSysUnlockFromISR();                                                   \
}
#else /* !SPI_USE_SYNCHRONIZATION */
#define __spi_wakeup_isr(spip, msg)
#endif /* !SPI_USE_SYNCHRONIZATION */

/**
 * @brief   Common ISR code in linear mode.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
#define __spi_isr_complete_code(spip) {                                     \
  (spip)->state = SPI_READY;                                                \
  if ((spip)->config->data_cb) {                                            \
    (spip)->config->data_cb(spip);                                          \
  }                                                                         \
  __spi_wakeup_isr(spip, MSG_OK);                                           \
}

/**
 * @brief   Half buffer filled ISR code in circular mode.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 *
 * @notapi
 */
#define __spi_isr_half_code(spip) {                                         \
  if ((spip)->config->data_cb) {                                            \
    (spip)->config->data_cb(spip);                                          \
  }                                                                         \
}

/**
 * @brief   Full buffer filled ISR code in circular mode.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 *
 * @notapi
 */
#define __spi_isr_full_code(spip) {                                         \
  if ((spip)->config->data_cb) {                                            \
    (spip)->state = SPI_COMPLETE;                                           \
    (spip)->config->data_cb(spip);                                          \
    if ((spip)->state == SPI_COMPLETE) {                                    \
      (spip)->state = SPI_ACTIVE;                                           \
    }                                                                       \
  }                                                                         \
}

/**
 * @brief   ISR error reporting code..
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] spip              pointer to the @p SPIDriver object
 * @param[in] msg               error code
 *
 * @notapi
 */
#define __spi_isr_error_code(spip, msg) {                                   \
  if ((spip)->config->error_cb) {                                           \
    (spip)->config->error_cb(spip);                                         \
  }                                                                         \
  __spi_wakeup_isr(spip, msg);                                              \
}
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void spiInit(void);
  void spiObjectInit(SPIDriver *spip);
  msg_t spiStart(SPIDriver *spip, const SPIConfig *config);
  void spiStop(SPIDriver *spip);
  void spiSelect(SPIDriver *spip);
  void spiUnselect(SPIDriver *spip);
  msg_t spiStartIgnoreI(SPIDriver *spip, size_t n);
  msg_t spiStartIgnore(SPIDriver *spip, size_t n);
  msg_t spiStartExchangeI(SPIDriver *spip, size_t n,
                          const void *txbuf, void *rxbuf);
  msg_t spiStartExchange(SPIDriver *spip, size_t n,
                         const void *txbuf, void *rxbuf);
  msg_t spiStartSendI(SPIDriver *spip, size_t n, const void *txbuf);
  msg_t spiStartSend(SPIDriver *spip, size_t n, const void *txbuf);
  msg_t spiStartReceiveI(SPIDriver *spip, size_t n, void *rxbuf);
  msg_t spiStartReceive(SPIDriver *spip, size_t n, void *rxbuf);
  msg_t spiStopTransferI(SPIDriver *spip, size_t *sizep);
  msg_t spiStopTransfer(SPIDriver *spip, size_t *sizep);
#if SPI_USE_SYNCHRONIZATION == TRUE
  msg_t spiSynchronizeS(SPIDriver *spip, sysinterval_t timeout);
  msg_t spiSynchronize(SPIDriver *spip, sysinterval_t timeout);
  msg_t spiIgnore(SPIDriver *spip, size_t n);
  msg_t spiExchange(SPIDriver *spip, size_t n, const void *txbuf, void *rxbuf);
  msg_t spiSend(SPIDriver *spip, size_t n, const void *txbuf);
  msg_t spiReceive(SPIDriver *spip, size_t n, void *rxbuf);
#endif
#if SPI_USE_MUTUAL_EXCLUSION == TRUE
  void spiAcquireBus(SPIDriver *spip);
  void spiReleaseBus(SPIDriver *spip);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI == TRUE */

#endif /* HAL_SPI_V2_H */

/** @} */
