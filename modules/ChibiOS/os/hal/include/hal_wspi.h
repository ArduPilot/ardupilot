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
 * @file    hal_wspi.h
 * @brief   WSPI Driver macros and structures.
 *
 * @addtogroup WSPI
 * @{
 */

#ifndef HAL_WSPI_H
#define HAL_WSPI_H

#if (HAL_USE_WSPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    WSPI configuration options
 * @{
 */
/**
 * @brief   Enables synchronous APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(WSPI_USE_WAIT) || defined(__DOXYGEN__)
#define WSPI_USE_WAIT                       TRUE
#endif

/**
 * @brief   Enables the @p wspiAcquireBus() and @p wspiReleaseBus() APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(WSPI_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define WSPI_USE_MUTUAL_EXCLUSION           TRUE
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
  WSPI_UNINIT = 0,                  /**< Not initialized.                   */
  WSPI_STOP = 1,                    /**< Stopped.                           */
  WSPI_READY = 2,                   /**< Ready.                             */
  WSPI_SEND = 3,                    /**< Sending data.                      */
  WSPI_RECEIVE = 4,                 /**< Receiving data.                    */
  WSPI_COMPLETE = 5,                /**< Asynchronous operation complete.   */
  WSPI_MEMMAP = 6                   /**< In memory mapped mode.             */
} wspistate_t;

/**
 * @brief   Type of a structure representing an WSPI driver.
 */
typedef struct hal_wspi_driver WSPIDriver;

/**
 * @brief   Type of a structure representing an WSPI driver configuration.
 */
typedef struct hal_wspi_config WSPIConfig;

/**
 * @brief   Type of a WSPI notification callback.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object triggering the
 *                      callback
 */
typedef void (*wspicallback_t)(WSPIDriver *wspip);

/**
 * @brief   Type of a WSPI command descriptor.
 */
typedef struct {
  /**
   * @brief   Transfer configuration field.
   */
  uint32_t              cfg;
  /**
   * @brief   Command phase data.
   */
  uint32_t              cmd;
  /**
   * @brief   Address phase data.
   */
  uint32_t              addr;
  /**
   * @brief   Alternate phase data.
   */
  uint32_t              alt;
  /**
   * @brief   Number of dummy cycles to be inserted.
   */
  uint32_t              dummy;
} wspi_command_t;

/* Including the low level driver header, it exports information required
   for completing types.*/
#include "hal_wspi_lld.h"

#if !defined(WSPI_SUPPORTS_MEMMAP)
#error "low level does not define WSPI_SUPPORTS_MEMMAP"
#endif

#if !defined(WSPI_DEFAULT_CFG_MASKS)
#error "low level does not define WSPI_DEFAULT_CFG_MASKS"
#endif

/**
 * @brief   Driver configuration structure.
 */
struct hal_wspi_config {
  /**
   * @brief   Operation complete callback or @p NULL.
   */
  wspicallback_t            end_cb;
  /**
   * @brief   Operation error callback or @p NULL.
   */
  wspicallback_t            error_cb;
  /* End of the mandatory fields.*/
  wspi_lld_config_fields
};

/**
 * @brief   Structure representing an WSPI driver.
 */
struct hal_wspi_driver {
  /**
   * @brief   Driver state.
   */
  wspistate_t               state;
  /**
   * @brief   Current configuration data.
   */
  const WSPIConfig          *config;
#if (WSPI_USE_WAIT == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Waiting thread.
   */
  thread_reference_t        thread;
#endif /* WSPI_USE_WAIT */
#if (WSPI_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the peripheral.
   */
  mutex_t                   mutex;
#endif /* WSPI_USE_MUTUAL_EXCLUSION */
#if defined(WSPI_DRIVER_EXT_FIELDS)
  WSPI_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  wspi_lld_driver_fields
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#if (WSPI_DEFAULT_CFG_MASKS == TRUE) || defined(__DOXYGEN__)
/**
 * @name    Transfer options
 * @note    The low level driver has the option to override the following
 *          definitions and use its own ones. It must take care to use
 *          the same name for the same function or compatibility is not
 *          ensured.
 * @{
 */
#define WSPI_CFG_CMD_MODE_MASK                  (7LU << 0LU)
#define WSPI_CFG_CMD_MODE_NONE                  (0LU << 0LU)
#define WSPI_CFG_CMD_MODE_ONE_LINE              (1LU << 0LU)
#define WSPI_CFG_CMD_MODE_TWO_LINES             (2LU << 0LU)
#define WSPI_CFG_CMD_MODE_FOUR_LINES            (3LU << 0LU)
#define WSPI_CFG_CMD_MODE_EIGHT_LINES           (4LU << 0LU)

#define WSPI_CFG_CMD_DTR                        (1LU << 3LU)

#define WSPI_CFG_CMD_SIZE_MASK                  (3LU << 4LU)
#define WSPI_CFG_CMD_SIZE_8                     (0LU << 4LU)
#define WSPI_CFG_CMD_SIZE_16                    (1LU << 4LU)
#define WSPI_CFG_CMD_SIZE_24                    (2LU << 4LU)
#define WSPI_CFG_CMD_SIZE_32                    (3LU << 4LU)

#define WSPI_CFG_ADDR_MODE_MASK                 (7LU << 8LU)
#define WSPI_CFG_ADDR_MODE_NONE                 (0LU << 8LU)
#define WSPI_CFG_ADDR_MODE_ONE_LINE             (1LU << 8LU)
#define WSPI_CFG_ADDR_MODE_TWO_LINES            (2LU << 8LU)
#define WSPI_CFG_ADDR_MODE_FOUR_LINES           (3LU << 8LU)
#define WSPI_CFG_ADDR_MODE_EIGHT_LINES          (4LU << 8LU)

#define WSPI_CFG_ADDR_DTR                       (1LU << 11LU)

#define WSPI_CFG_ADDR_SIZE_MASK                 (3LU << 12LU)
#define WSPI_CFG_ADDR_SIZE_8                    (0LU << 12LU)
#define WSPI_CFG_ADDR_SIZE_16                   (1LU << 12LU)
#define WSPI_CFG_ADDR_SIZE_24                   (2LU << 12LU)
#define WSPI_CFG_ADDR_SIZE_32                   (3LU << 12LU)

#define WSPI_CFG_ALT_MODE_MASK                  (7LU << 16LU)
#define WSPI_CFG_ALT_MODE_NONE                  (0LU << 16LU)
#define WSPI_CFG_ALT_MODE_ONE_LINE              (1LU << 16LU)
#define WSPI_CFG_ALT_MODE_TWO_LINES             (2LU << 16LU)
#define WSPI_CFG_ALT_MODE_FOUR_LINES            (3LU << 16LU)
#define WSPI_CFG_ALT_MODE_EIGHT_LINES           (4LU << 16LU)

#define WSPI_CFG_ALT_DTR                        (1LU << 19LU)

#define WSPI_CFG_ALT_SIZE_MASK                  (3LU << 20LU)
#define WSPI_CFG_ALT_SIZE_8                     (0LU << 20LU)
#define WSPI_CFG_ALT_SIZE_16                    (1LU << 20LU)
#define WSPI_CFG_ALT_SIZE_24                    (2LU << 20LU)
#define WSPI_CFG_ALT_SIZE_32                    (3LU << 20LU)

#define WSPI_CFG_DATA_MODE_MASK                 (7LU << 24LU)
#define WSPI_CFG_DATA_MODE_NONE                 (0LU << 24LU)
#define WSPI_CFG_DATA_MODE_ONE_LINE             (1LU << 24LU)
#define WSPI_CFG_DATA_MODE_TWO_LINES            (2LU << 24LU)
#define WSPI_CFG_DATA_MODE_FOUR_LINES           (3LU << 24LU)
#define WSPI_CFG_DATA_MODE_EIGHT_LINES          (4LU << 24LU)

#define WSPI_CFG_DATA_DTR                       (1LU << 27LU)

#define WSPI_CFG_DQS_ENABLE                     (1LU << 29LU)

#define WSPI_CFG_SIOO                           (1LU << 31LU)

#define WSPI_CFG_ALL_DTR                        (WSPI_CFG_CMD_DTR   |       \
                                                 WSPI_CFG_ADDR_DTR  |       \
                                                 WSPI_CFG_ALT_DTR   |       \
                                                 WSPI_CFG_DATA_DTR)
/** @} */
#endif /* WSPI_DEFAULT_CFG_MASKS == TRUE */

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Sends a command without data phase.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] cmdp      pointer to the command descriptor
 *
 * @iclass
 */
#define wspiStartCommandI(wspip, cmdp) {                                    \
  osalDbgAssert(((cmdp)->cfg & WSPI_CFG_DATA_MODE_MASK) ==                  \
                WSPI_CFG_DATA_MODE_NONE,                                    \
                "data mode specified");                                     \
  (wspip)->state = WSPI_SEND;                                               \
  wspi_lld_command(wspip, cmdp);                                            \
}

/**
 * @brief   Sends data over the WSPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] cmdp      pointer to the command descriptor
 * @param[in] n         number of bytes to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @iclass
 */
#define wspiStartSendI(wspip, cmdp, n, txbuf) {                             \
  osalDbgAssert(((cmdp)->cfg & WSPI_CFG_DATA_MODE_MASK) !=                  \
                WSPI_CFG_DATA_MODE_NONE,                                    \
                "data mode required");                                      \
  (wspip)->state = WSPI_SEND;                                               \
  wspi_lld_send(wspip, cmdp, n, txbuf);                                     \
}

/**
 * @brief   Receives data from the WSPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] cmdp      pointer to the command descriptor
 * @param[in] n         number of bytes to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @iclass
 */
#define wspiStartReceiveI(wspip, cmdp, n, rxbuf) {                          \
  osalDbgAssert(((cmdp)->cfg & WSPI_CFG_DATA_MODE_MASK) !=                  \
                WSPI_CFG_DATA_MODE_NONE,                                    \
                "data mode required");                                      \
  (wspip)->state = WSPI_RECEIVE;                                            \
  wspi_lld_receive(wspip, cmdp, n, rxbuf);                                  \
}

#if (WSPI_SUPPORTS_MEMMAP == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Maps in memory space a WSPI flash device.
 * @pre     The memory flash device must be initialized appropriately
 *          before mapping it in memory space.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] cmdp      pointer to the command descriptor
 * @param[out] addrp    pointer to the memory start address of the mapped
 *                      flash or @p NULL
 *
 * @iclass
 */
#define wspiMapFlashI(wspip, cmdp, addrp)                                   \
  wspi_lld_map_flash(wspip, cmdp, addrp)

/**
 * @brief   Maps in memory space a WSPI flash device.
 * @post    The memory flash device must be re-initialized for normal
 *          commands exchange.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 *
 * @iclass
 */
#define wspiUnmapFlashI(wspip)                                              \
  wspi_lld_unmap_flash(wspip)
#endif /* WSPI_SUPPORTS_MEMMAP == TRUE */
/** @} */

/**
 * @name    Low level driver helper macros
 * @{
 */
#if (WSPI_USE_WAIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Wakes up the waiting thread.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] msg       the wakeup message
 *
 * @notapi
 */
#define _wspi_wakeup_isr(wspip, msg) {                                      \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(wspip)->thread, msg);                                 \
  osalSysUnlockFromISR();                                                   \
}
#else /* !WSPI_USE_WAIT */
#define _wspi_wakeup_isr(wspip, msg)
#endif /* !WSPI_USE_WAIT */

/**
 * @brief   Common ISR code.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 *
 * @notapi
 */
#define _wspi_isr_code(wspip) {                                             \
  if ((wspip)->config->end_cb) {                                            \
    (wspip)->state = WSPI_COMPLETE;                                         \
    (wspip)->config->end_cb(wspip);                                         \
    if ((wspip)->state == WSPI_COMPLETE)                                    \
      (wspip)->state = WSPI_READY;                                          \
  }                                                                         \
  else                                                                      \
    (wspip)->state = WSPI_READY;                                            \
  _wspi_wakeup_isr(wspip, MSG_OK);                                          \
}

/**
 * @brief   Common error ISR code.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 *
 * @notapi
 */
#define _wspi_error_code(wspip) {                                           \
  if ((wspip)->config->error_cb) {                                          \
    (wspip)->state = WSPI_COMPLETE;                                         \
    (wspip)->config->error_cb(wspip);                                       \
    if ((wspip)->state == WSPI_COMPLETE)                                    \
      (wspip)->state = WSPI_READY;                                          \
  }                                                                         \
  else                                                                      \
    (wspip)->state = WSPI_READY;                                            \
  _wspi_wakeup_isr(wspip, MSG_RESET);                                       \
}
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void wspiInit(void);
  void wspiObjectInit(WSPIDriver *wspip);
  msg_t wspiStart(WSPIDriver *wspip, const WSPIConfig *config);
  void wspiStop(WSPIDriver *wspip);
  void wspiStartCommand(WSPIDriver *wspip, const wspi_command_t *cmdp);
  void wspiStartSend(WSPIDriver *wspip, const wspi_command_t *cmdp,
                     size_t n, const uint8_t *txbuf);
  void wspiStartReceive(WSPIDriver *wspip, const wspi_command_t *cmdp,
                        size_t n, uint8_t *rxbuf);
#if WSPI_USE_WAIT == TRUE
  bool wspiCommand(WSPIDriver *wspip, const wspi_command_t *cmdp);
  bool wspiSend(WSPIDriver *wspip, const wspi_command_t *cmdp,
                size_t n, const uint8_t *txbuf);
  bool wspiReceive(WSPIDriver *wspip, const wspi_command_t *cmdp,
                   size_t n, uint8_t *rxbuf);
#endif
#if WSPI_SUPPORTS_MEMMAP == TRUE
void wspiMapFlash(WSPIDriver *wspip,
                  const wspi_command_t *cmdp,
                  uint8_t **addrp);
void wspiUnmapFlash(WSPIDriver *wspip);
#endif
#if WSPI_USE_MUTUAL_EXCLUSION == TRUE
  void wspiAcquireBus(WSPIDriver *wspip);
  void wspiReleaseBus(WSPIDriver *wspip);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_WSPI == TRUE */

#endif /* HAL_WSPI_H */

/** @} */
