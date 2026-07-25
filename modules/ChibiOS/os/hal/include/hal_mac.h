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
 * @file    hal_mac.h
 * @brief   MAC Driver macros and structures.
 * @addtogroup MAC
 * @{
 */

#ifndef HAL_MAC_H
#define HAL_MAC_H

#if (HAL_USE_MAC == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    MAC event flags
 * @{
 */
#define MAC_FLAGS_TX                (1U << 0)
#define MAC_FLAGS_RX                (1U << 1)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    MAC configuration options
 * @{
 */
/**
 * @brief   Enables zero-copy support in the MAC API.
 */
#if !defined(MAC_USE_ZERO_COPY) || defined(__DOXYGEN__)
#define MAC_USE_ZERO_COPY           FALSE
#endif

/**
 * @brief   Enables an event source for incoming packets.
 */
#if !defined(MAC_USE_EVENTS) || defined(__DOXYGEN__)
#define MAC_USE_EVENTS              TRUE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing a MAC driver.
 */
typedef struct hal_mac_driver MACDriver;

/**
 * @brief   Type of structure representing a MAC configuration.
 */
typedef struct hal_mac_config MACConfig;

/**
 * @brief   Type of structure representing a MAC transmit descriptor.
 */
typedef struct hal_mac_transmit_descriptor MACTransmitDescriptor;

/**
 * @brief   Type of structure representing a MAC receive descriptor.
 */
typedef struct hal_mac_receive_descriptor MACReceiveDescriptor;

/**
 * @brief   Generic ETH notification callback type.
 *
 * @param[in] macp     pointer to the @p MACDriver object
 */
typedef void (*maccb_t)(MACDriver *macp);

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  MAC_UNINIT = 0,                   /**< Not initialized.                   */
  MAC_STOP = 1,                     /**< Stopped.                           */
  MAC_ACTIVE = 2                    /**< Active.                            */
} macstate_t;

#include "hal_mac_lld.h"

/**
 * @brief   Driver configuration structure.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
struct hal_mac_config {
  mac_lld_config_fields
#if defined(MAC_CONFIG_EXT_FIELDS)
  MAC_CONFIG_EXT_FIELDS
#endif
};

/**
 * @brief   Structure representing a MAC driver.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
struct hal_mac_driver {
  /**
   * @brief   Driver state.
   */
  macstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const MACConfig           *config;
  /**
   * @brief   Transmit threads queue.
   */
  threads_queue_t           tdqueue;
  /**
   * @brief   Receive threads queue.
   */
  threads_queue_t           rdqueue;
#if MAC_USE_EVENTS || defined(__DOXYGEN__)
  /**
   * @brief   MAC events source.
   */
  event_source_t            es;
#endif
  /**
   * @brief   Locally held event flags for callback use.
   */
  eventflags_t              flags;
  /**
   * @brief   Events callback.
   * @note    Can be @p NULL.
   */
  maccb_t                   cb;
  /**
   * @brief   User argument.
   * @note    Can be retrieved through the @p macp argument of the callback.
   */
  void                      *arg;
  /* End of the mandatory fields.*/
  mac_lld_driver_fields
#if defined(MAC_DRIVER_EXT_FIELDS)
  MAC_DRIVER_EXT_FIELDS
#endif
};

/**
 * @brief   Structure representing a MAC transmit descriptor.
 */
struct hal_mac_transmit_descriptor {
  /**
   * @brief   Current write offset.
   */
  size_t                    offset;
  /**
   * @brief   Available space size.
   */
  size_t                    size;
  /* End of the mandatory fields.*/
  mac_lld_transmit_descriptor_fields
};

/**
 * @brief   Structure representing a MAC receive descriptor.
 */
struct hal_mac_receive_descriptor {
  /**
   * @brief   Current read offset.
   */
  size_t                    offset;
  /**
   * @brief   Available data size.
   */
  size_t                    size;
  /* End of the mandatory fields.*/
  mac_lld_receive_descriptor_fields
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Low level driver helper macros
 * @{
 */
/**
 * @brief   MAC callback.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 *
 * @notapi
 */
#define __mac_callback(macp) do {                                           \
  if ((macp)->cb != NULL) {                                                 \
    (macp)->cb(macp);                                                       \
  }                                                                         \
} while (false)

/**
 * @brief   MAC TX wakeup and event.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 *
 * @notapi
 */
#if (MAC_USE_EVENTS == TRUE) || defined(__DOXYGEN__)
#define __mac_tx_wakeup(macp) do {                                          \
  osalSysLockFromISR();                                                     \
  osalThreadDequeueAllI(&(macp)->tdqueue, MSG_OK);                          \
  (macp)->flags |= MAC_FLAGS_TX;                                            \
  osalEventBroadcastFlagsI(&(macp)->es, MAC_FLAGS_TX);                      \
  osalSysUnlockFromISR();                                                   \
} while (false)
#else
#define __mac_tx_wakeup(macp) do {                                          \
  osalSysLockFromISR();                                                     \
  osalThreadDequeueAllI(&(macp)->tdqueue, MSG_OK);                          \
  (macp)->flags |= MAC_FLAGS_TX;                                            \
  osalSysUnlockFromISR();                                                   \
} while (false)
#endif

/**
 * @brief   MAC RX wakeup and event.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 *
 * @notapi
 */
#if (MAC_USE_EVENTS == TRUE) || defined(__DOXYGEN__)
#define __mac_rx_wakeup(macp) do {                                          \
  osalSysLockFromISR();                                                     \
  osalThreadDequeueAllI(&(macp)->rdqueue, MSG_OK);                          \
  (macp)->flags |= MAC_FLAGS_RX;                                            \
  osalEventBroadcastFlagsI(&(macp)->es, MAC_FLAGS_RX);                      \
  osalSysUnlockFromISR();                                                   \
} while (false)
#else
#define __mac_rx_wakeup(macp) do {                                          \
  osalSysLockFromISR();                                                     \
  osalThreadDequeueAllI(&(macp)->rdqueue, MSG_OK);                          \
  (macp)->flags |= MAC_FLAGS_RX;                                            \
  osalSysUnlockFromISR();                                                   \
} while (false)
#endif
/** @} */

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Associates a callback to the MAC instance.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 * @param[in] f         callback to be associated
 */
#define macSetCallbackX(macp, f) (macp)->cb = (f)

/**
 * @brief   Returns the driver events source.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 * @return              The pointer to the @p EventSource structure.
 *
 * @api
 */
#if (MAC_USE_EVENTS == TRUE) || defined(__DOXYGEN__)
#define macGetEventSource(macp)  (&(macp)->es)
#endif

/**
 * @brief   Returns a transmission descriptor.
 * @details One of the available transmission descriptors is locked and
 *          returned.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 * @param[out] tdp      pointer to a @p MACTransmitDescriptor structure
 * @return              The operation status.
 * @retval MSG_OK       the descriptor has been obtained.
 * @retval MSG_TIMEOUT  descriptor not available.
 *
 * @xclass
 */
#define macGetTransmitDescriptorX(macp, tdp)                                \
  mac_lld_get_transmit_descriptor(macp, tdp)

/**
 * @brief   Releases a transmit descriptor and starts the transmission of the
 *          enqueued data as a single frame.
 *
 * @param[in] tdp       the pointer to the @p MACTransmitDescriptor structure
 *
 * @notapi
 */
#define macReleaseTransmitDescriptorX(tdp)                                  \
  mac_lld_release_transmit_descriptor(tdp)

/**
 * @brief   Returns a receive descriptor.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 * @param[out] rdp      pointer to a @p MACReceiveDescriptor structure
 * @return              The operation status.
 * @retval MSG_OK       the descriptor has been obtained.
 * @retval MSG_TIMEOUT  descriptor not available.
 *
 * @xclass
 */
#define macGetReceiveDescriptorX(macp, rdp)                                 \
  mac_lld_get_receive_descriptor(macp, rdp)

/**
 * @brief   Releases a receive descriptor.
 * @details The descriptor and its buffer are made available for more incoming
 *          frames.
 *
 * @param[in] rdp       the pointer to the @p MACReceiveDescriptor structure
 *
 * @xclass
 */
#define macReleaseReceiveDescriptorX(rdp)                                   \
  mac_lld_release_receive_descriptor(rdp)

/**
 * @brief   Writes to a transmit descriptor's stream.
 *
 * @param[in] tdp       pointer to a @p MACTransmitDescriptor structure
 * @param[in] buf       pointer to the buffer containing the data to be written
 * @param[in] size      number of bytes to be written
 * @return              The number of bytes written into the descriptor's
 *                      stream, this value can be less than the amount
 *                      specified in the parameter @p size if the maximum frame
 *                      size is reached.
 *
 * @api
 */
#define macWriteTransmitDescriptor(tdp, buf, size)                          \
    mac_lld_write_transmit_descriptor(tdp, buf, size)

/**
 * @brief   Reads from a receive descriptor's stream.
 *
 * @param[in] rdp       pointer to a @p MACReceiveDescriptor structure
 * @param[in] buf       pointer to the buffer that will receive the read data
 * @param[in] size      number of bytes to be read
 * @return              The number of bytes read from the descriptor's stream,
 *                      this value can be less than the amount specified in the
 *                      parameter @p size if there are no more bytes to read.
 *
 * @api
 */
#define macReadReceiveDescriptor(rdp, buf, size)                            \
    mac_lld_read_receive_descriptor(rdp, buf, size)

#if (MAC_USE_ZERO_COPY == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Returns a pointer to the next transmit buffer in the descriptor
 *          chain.
 * @note    The API guarantees that enough buffers can be requested to fill
 *          a whole frame.
 *
 * @param[in] tdp       pointer to a @p MACTransmitDescriptor structure
 * @param[in] size      size of the requested buffer. Specify the frame size
 *                      on the first call then scale the value down subtracting
 *                      the amount of data already copied into the previous
 *                      buffers.
 * @param[out] sizep    pointer to variable receiving the real buffer size.
 *                      The returned value can be less than the amount
 *                      requested, this means that more buffers must be
 *                      requested in order to fill the frame data entirely.
 * @return              Pointer to the returned buffer.
 *
 * @api
 */
#define macGetNextTransmitBuffer(tdp, size, sizep)                          \
  mac_lld_get_next_transmit_buffer(tdp, size, sizep)

/**
 * @brief   Returns a pointer to the next receive buffer in the descriptor
 *          chain.
 * @note    The API guarantees that the descriptor chain contains a whole
 *          frame.
 *
 * @param[in] rdp       pointer to a @p MACReceiveDescriptor structure
 * @param[out] sizep    pointer to variable receiving the buffer size, it is
 *                      zero when the last buffer has already been returned.
 * @return              Pointer to the returned buffer.
 * @retval NULL         if the buffer chain has been entirely scanned.
 *
 * @api
 */
#define macGetNextReceiveBuffer(rdp, sizep)                                 \
  mac_lld_get_next_receive_buffer(rdp, sizep)
#endif /* MAC_USE_ZERO_COPY */
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  bool macInit(void);
  void macObjectInit(MACDriver *macp);
  msg_t macStart(MACDriver *macp, const MACConfig *config);
  void macStop(MACDriver *macp);
  eventflags_t macGetAndClearEventsI(MACDriver *macp);
  msg_t macWaitTransmitDescriptor(MACDriver *macp,
                                  MACTransmitDescriptor *tdp,
                                  sysinterval_t timeout);
  void macReleaseTransmitDescriptor(MACTransmitDescriptor *tdp);
  msg_t macWaitReceiveDescriptor(MACDriver *macp,
                                 MACReceiveDescriptor *rdp,
                                 sysinterval_t timeout);
  void macReleaseReceiveDescriptor(MACReceiveDescriptor *rdp);
  bool macPollLinkStatus(MACDriver *macp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_MAC == TRUE */

#endif /* HAL_MAC_H */

/** @} */
