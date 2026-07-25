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
 * @file    hal_usb.h
 * @brief   USB Driver macros and structures.
 *
 * @addtogroup USB
 * @{
 */

#ifndef HAL_USB_H
#define HAL_USB_H

#if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define USB_ENDPOINT_OUT(ep)                (ep)
#define USB_ENDPOINT_IN(ep)                 ((ep) | 0x80U)

#define USB_RTYPE_DIR_MASK                  0x80U
#define USB_RTYPE_DIR_HOST2DEV              0x00U
#define USB_RTYPE_DIR_DEV2HOST              0x80U
#define USB_RTYPE_TYPE_MASK                 0x60U
#define USB_RTYPE_TYPE_STD                  0x00U
#define USB_RTYPE_TYPE_CLASS                0x20U
#define USB_RTYPE_TYPE_VENDOR               0x40U
#define USB_RTYPE_TYPE_RESERVED             0x60U
#define USB_RTYPE_RECIPIENT_MASK            0x1FU
#define USB_RTYPE_RECIPIENT_DEVICE          0x00U
#define USB_RTYPE_RECIPIENT_INTERFACE       0x01U
#define USB_RTYPE_RECIPIENT_ENDPOINT        0x02U
#define USB_RTYPE_RECIPIENT_OTHER           0x03U

#define USB_REQ_GET_STATUS                  0U
#define USB_REQ_CLEAR_FEATURE               1U
#define USB_REQ_SET_FEATURE                 3U
#define USB_REQ_SET_ADDRESS                 5U
#define USB_REQ_GET_DESCRIPTOR              6U
#define USB_REQ_SET_DESCRIPTOR              7U
#define USB_REQ_GET_CONFIGURATION           8U
#define USB_REQ_SET_CONFIGURATION           9U
#define USB_REQ_GET_INTERFACE               10U
#define USB_REQ_SET_INTERFACE               11U
#define USB_REQ_SYNCH_FRAME                 12U

#define USB_DESCRIPTOR_DEVICE               1U
#define USB_DESCRIPTOR_CONFIGURATION        2U
#define USB_DESCRIPTOR_STRING               3U
#define USB_DESCRIPTOR_INTERFACE            4U
#define USB_DESCRIPTOR_ENDPOINT             5U
#define USB_DESCRIPTOR_DEVICE_QUALIFIER     6U
#define USB_DESCRIPTOR_OTHER_SPEED_CFG      7U
#define USB_DESCRIPTOR_INTERFACE_POWER      8U
#define USB_DESCRIPTOR_INTERFACE_ASSOCIATION 11U

#define USB_FEATURE_ENDPOINT_HALT           0U
#define USB_FEATURE_DEVICE_REMOTE_WAKEUP    1U
#define USB_FEATURE_TEST_MODE               2U

#define USB_EARLY_SET_ADDRESS               0
#define USB_LATE_SET_ADDRESS                1

#define USB_EP0_STATUS_STAGE_SW             0
#define USB_EP0_STATUS_STAGE_HW             1

#define USB_SET_ADDRESS_ACK_SW              0
#define USB_SET_ADDRESS_ACK_HW              1

/**
 * @name    Helper macros for USB descriptors
 * @{
 */
/**
 * @brief   Helper macro for index values into descriptor strings.
 */
#define USB_DESC_INDEX(i) ((uint8_t)(i))

/**
 * @brief   Helper macro for byte values into descriptor strings.
 */
#define USB_DESC_BYTE(b) ((uint8_t)(b))

/**
 * @brief   Helper macro for word values into descriptor strings.
 */
#define USB_DESC_WORD(w)                                                    \
  (uint8_t)((w) & 255U),                                                    \
  (uint8_t)(((w) >> 8) & 255U)

/**
 * @brief   Helper macro for BCD values into descriptor strings.
 */
#define USB_DESC_BCD(bcd)                                                   \
  (uint8_t)((bcd) & 255U),                                                  \
  (uint8_t)(((bcd) >> 8) & 255)

/*
 * @define  Device Descriptor size.
 */
#define USB_DESC_DEVICE_SIZE                18U

/**
 * @brief   Device Descriptor helper macro.
 */
#define USB_DESC_DEVICE(bcdUSB, bDeviceClass, bDeviceSubClass,              \
                        bDeviceProtocol, bMaxPacketSize, idVendor,          \
                        idProduct, bcdDevice, iManufacturer,                \
                        iProduct, iSerialNumber, bNumConfigurations)        \
  USB_DESC_BYTE(USB_DESC_DEVICE_SIZE),                                      \
  USB_DESC_BYTE(USB_DESCRIPTOR_DEVICE),                                     \
  USB_DESC_BCD(bcdUSB),                                                     \
  USB_DESC_BYTE(bDeviceClass),                                              \
  USB_DESC_BYTE(bDeviceSubClass),                                           \
  USB_DESC_BYTE(bDeviceProtocol),                                           \
  USB_DESC_BYTE(bMaxPacketSize),                                            \
  USB_DESC_WORD(idVendor),                                                  \
  USB_DESC_WORD(idProduct),                                                 \
  USB_DESC_BCD(bcdDevice),                                                  \
  USB_DESC_INDEX(iManufacturer),                                            \
  USB_DESC_INDEX(iProduct),                                                 \
  USB_DESC_INDEX(iSerialNumber),                                            \
  USB_DESC_BYTE(bNumConfigurations)

/**
 * @brief   Configuration Descriptor size.
 */
#define USB_DESC_CONFIGURATION_SIZE         9U

/**
 * @brief   Configuration Descriptor helper macro.
 */
#define USB_DESC_CONFIGURATION(wTotalLength, bNumInterfaces,                \
                               bConfigurationValue, iConfiguration,         \
                               bmAttributes, bMaxPower)                     \
  USB_DESC_BYTE(USB_DESC_CONFIGURATION_SIZE),                               \
  USB_DESC_BYTE(USB_DESCRIPTOR_CONFIGURATION),                              \
  USB_DESC_WORD(wTotalLength),                                              \
  USB_DESC_BYTE(bNumInterfaces),                                            \
  USB_DESC_BYTE(bConfigurationValue),                                       \
  USB_DESC_INDEX(iConfiguration),                                           \
  USB_DESC_BYTE(bmAttributes),                                              \
  USB_DESC_BYTE(bMaxPower)

/**
 * @brief   Interface Descriptor size.
 */
#define USB_DESC_INTERFACE_SIZE             9U

/**
 * @brief   Interface Descriptor helper macro.
 */
#define USB_DESC_INTERFACE(bInterfaceNumber, bAlternateSetting,             \
                           bNumEndpoints, bInterfaceClass,                  \
                           bInterfaceSubClass, bInterfaceProtocol,          \
                           iInterface)                                      \
  USB_DESC_BYTE(USB_DESC_INTERFACE_SIZE),                                   \
  USB_DESC_BYTE(USB_DESCRIPTOR_INTERFACE),                                  \
  USB_DESC_BYTE(bInterfaceNumber),                                          \
  USB_DESC_BYTE(bAlternateSetting),                                         \
  USB_DESC_BYTE(bNumEndpoints),                                             \
  USB_DESC_BYTE(bInterfaceClass),                                           \
  USB_DESC_BYTE(bInterfaceSubClass),                                        \
  USB_DESC_BYTE(bInterfaceProtocol),                                        \
  USB_DESC_INDEX(iInterface)

/**
 * @brief   Interface Association Descriptor size.
 */
#define USB_DESC_INTERFACE_ASSOCIATION_SIZE 8U

/**
 * @brief   Interface Association Descriptor helper macro.
 */
#define USB_DESC_INTERFACE_ASSOCIATION(bFirstInterface,                     \
                           bInterfaceCount, bFunctionClass,                 \
                           bFunctionSubClass, bFunctionProtocol,            \
                           iInterface)                                      \
  USB_DESC_BYTE(USB_DESC_INTERFACE_ASSOCIATION_SIZE),                       \
  USB_DESC_BYTE(USB_DESCRIPTOR_INTERFACE_ASSOCIATION),                      \
  USB_DESC_BYTE(bFirstInterface),                                           \
  USB_DESC_BYTE(bInterfaceCount),                                           \
  USB_DESC_BYTE(bFunctionClass),                                            \
  USB_DESC_BYTE(bFunctionSubClass),                                         \
  USB_DESC_BYTE(bFunctionProtocol),                                         \
  USB_DESC_INDEX(iInterface)

/**
 * @brief   Endpoint Descriptor size.
 */
#define USB_DESC_ENDPOINT_SIZE              7U

/**
 * @brief   Endpoint Descriptor helper macro.
 */
#define USB_DESC_ENDPOINT(bEndpointAddress, bmAttributes, wMaxPacketSize,   \
                          bInterval)                                        \
  USB_DESC_BYTE(USB_DESC_ENDPOINT_SIZE),                                    \
  USB_DESC_BYTE(USB_DESCRIPTOR_ENDPOINT),                                   \
  USB_DESC_BYTE(bEndpointAddress),                                          \
  USB_DESC_BYTE(bmAttributes),                                              \
  USB_DESC_WORD(wMaxPacketSize),                                            \
  USB_DESC_BYTE(bInterval)
/** @} */

/**
 * @name    Endpoint types and settings
 * @{
 */
#define USB_EP_MODE_TYPE                0x0003U /**< Endpoint type mask.    */
#define USB_EP_MODE_TYPE_CTRL           0x0000U /**< Control endpoint.      */
#define USB_EP_MODE_TYPE_ISOC           0x0001U /**< Isochronous endpoint.  */
#define USB_EP_MODE_TYPE_BULK           0x0002U /**< Bulk endpoint.         */
#define USB_EP_MODE_TYPE_INTR           0x0003U /**< Interrupt endpoint.    */
/** @} */

#define USB_IN_STATE                    0x08U
#define USB_OUT_STATE                   0x10U

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Enables synchronous APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(USB_USE_WAIT) || defined(__DOXYGEN__)
#define USB_USE_WAIT                        FALSE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an USB driver.
 */
typedef struct USBDriver USBDriver;

/**
 * @brief   Type of an endpoint identifier.
 */
typedef uint8_t usbep_t;

/**
 * @brief   Type of a driver state machine possible states.
 */
typedef enum {
  USB_UNINIT    = 0,                    /**< Not initialized.               */
  USB_STOP      = 1,                    /**< Stopped.                       */
  USB_READY     = 2,                    /**< Ready, after bus reset.        */
  USB_SELECTED  = 3,                    /**< Address assigned.              */
  USB_ACTIVE    = 4,                    /**< Active, configuration selected.*/
  USB_SUSPENDED = 5                     /**< Suspended, low power mode.     */
} usbstate_t;

/**
 * @brief   Type of an endpoint status.
 */
typedef enum {
  EP_STATUS_DISABLED = 0,               /**< Endpoint not active.           */
  EP_STATUS_STALLED = 1,                /**< Endpoint opened but stalled.   */
  EP_STATUS_ACTIVE = 2                  /**< Active endpoint.               */
} usbepstatus_t;

/**
 * @brief   Type of an endpoint zero state machine states.
 */
typedef enum {
  USB_EP0_STP_WAITING = 0U,                     /**< Waiting for SETUP data.*/
  USB_EP0_IN_TX = USB_IN_STATE | 1U,            /**< Transmitting.          */
  USB_EP0_IN_WAITING_TX0 = USB_IN_STATE | 2U,   /**< Waiting transmit 0.    */
  USB_EP0_IN_SENDING_STS = USB_IN_STATE | 3U,   /**< Sending status.        */
  USB_EP0_OUT_WAITING_STS = USB_OUT_STATE | 4U, /**< Waiting status.        */
  USB_EP0_OUT_RX = USB_OUT_STATE | 5U,          /**< Receiving.             */
  USB_EP0_ERROR = 6U                            /**< Error, EP0 stalled.    */
} usbep0state_t;

/**
 * @brief   Type of an enumeration of the possible USB events.
 */
typedef enum {
  USB_EVENT_RESET = 0,                  /**< Driver has been reset by host. */
  USB_EVENT_ADDRESS = 1,                /**< Address assigned.              */
  USB_EVENT_CONFIGURED = 2,             /**< Configuration selected.        */
  USB_EVENT_UNCONFIGURED = 3,           /**< Configuration removed.         */
  USB_EVENT_SUSPEND = 4,                /**< Entering suspend mode.         */
  USB_EVENT_WAKEUP = 5,                 /**< Leaving suspend mode.          */
  USB_EVENT_STALLED = 6                 /**< Endpoint 0 error, stalled.     */
} usbevent_t;

/**
 * @brief   Type of an USB descriptor.
 */
typedef struct {
  /**
   * @brief   Descriptor size in unicode characters.
   */
  size_t                        ud_size;
  /**
   * @brief   Pointer to the descriptor.
   */
  const uint8_t                 *ud_string;
} USBDescriptor;

/**
 * @brief   Type of an USB generic notification callback.
 *
 * @param[in] usbp      pointer to the @p USBDriver object triggering the
 *                      callback
 */
typedef void (*usbcallback_t)(USBDriver *usbp);

/**
 * @brief   Type of an USB endpoint callback.
 *
 * @param[in] usbp      pointer to the @p USBDriver object triggering the
 *                      callback
 * @param[in] ep        endpoint number
 */
typedef void (*usbepcallback_t)(USBDriver *usbp, usbep_t ep);

/**
 * @brief   Type of an USB event notification callback.
 *
 * @param[in] usbp      pointer to the @p USBDriver object triggering the
 *                      callback
 * @param[in] event     event type
 */
typedef void (*usbeventcb_t)(USBDriver *usbp, usbevent_t event);

/**
 * @brief   Type of a requests handler callback.
 * @details The request is encoded in the @p usb_setup buffer.
 *
 * @param[in] usbp      pointer to the @p USBDriver object triggering the
 *                      callback
 * @return              The request handling exit code.
 * @retval false        Request not recognized by the handler.
 * @retval true         Request handled.
 */
typedef bool (*usbreqhandler_t)(USBDriver *usbp);

/**
 * @brief   Type of an USB descriptor-retrieving callback.
 */
typedef const USBDescriptor * (*usbgetdescriptor_t)(USBDriver *usbp,
                                                    uint8_t dtype,
                                                    uint8_t dindex,
                                                    uint16_t lang);

#include "hal_usb_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Returns the driver state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The driver state.
 *
 * @iclass
 */
#define usbGetDriverStateI(usbp) ((usbp)->state)

/**
 * @brief   Connects the USB device.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @api
 */
#define usbConnectBus(usbp) usb_lld_connect_bus(usbp)

/**
 * @brief   Disconnect the USB device.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @api
 */
#define usbDisconnectBus(usbp) usb_lld_disconnect_bus(usbp)

/**
 * @brief   Returns the current frame number.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The current frame number.
 *
 * @xclass
 */
#define usbGetFrameNumberX(usbp) usb_lld_get_frame_number(usbp)

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The operation status.
 * @retval false        Endpoint ready.
 * @retval true         Endpoint transmitting.
 *
 * @iclass
 */
#define usbGetTransmitStatusI(usbp, ep)                                     \
  (((usbp)->transmitting & (uint16_t)((unsigned)1U << (unsigned)(ep))) != 0U)

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The operation status.
 * @retval false        Endpoint ready.
 * @retval true         Endpoint receiving.
 *
 * @iclass
 */
#define usbGetReceiveStatusI(usbp, ep)                                      \
  (((usbp)->receiving & (uint16_t)((unsigned)1U << (unsigned)(ep))) != 0U)

/**
 * @brief   Returns the exact size of a receive transaction.
 * @details The received size can be different from the size specified in
 *          @p usbStartReceiveI() because the last packet could have a size
 *          different from the expected one.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              Received data size.
 *
 * @xclass
 */
#define usbGetReceiveTransactionSizeX(usbp, ep)                             \
  usb_lld_get_transaction_size(usbp, ep)

/**
 * @brief   Request transfer setup.
 * @details This macro is used by the request handling callbacks in order to
 *          prepare a transaction over the endpoint zero.
 * @note    Endpoint zero transfers are managed by the control-transfer state
 *          machine; applications must not use the blocking wait APIs
 *          (usbTransmit()/usbReceive()) on EP0.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] buf       pointer to a buffer for the transaction data
 * @param[in] n         number of bytes to be transferred
 * @param[in] endcb     callback to be invoked after the transfer or @p NULL
 *
 * @special
 */
#define usbSetupTransfer(usbp, buf, n, endcb) {                             \
  (usbp)->ep0next  = (buf);                                                 \
  (usbp)->ep0n     = (n);                                                   \
  (usbp)->ep0endcb = (endcb);                                               \
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @note    This function can be invoked both in thread and IRQ context.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @special
 */
#define usbReadSetup(usbp, ep, buf) usb_lld_read_setup(usbp, ep, buf)

/**
 * @brief   Restores setup handling.
 * @details This function removes stall and restores setup state.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @note    This function can be invoked both in thread and IRQ context.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @special
 */
#define usbRestoreSetup(usbp, ep) {                                         \
  usb_lld_clear_in(usbp, ep);                                               \
  usb_lld_clear_out(usbp, ep);                                              \
  (usbp)->receiving &= ~1U;                                                 \
  (usbp)->transmitting &= ~1U;                                              \
  (usbp)->ep0n = 0;                                                         \
  (usbp)->ep0state = USB_EP0_STP_WAITING;                                   \
}
/** @} */

/**
 * @name    Low level driver helper macros
 * @{
 */
/**
 * @brief   Common ISR code, usb event callback.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] evt       USB event code
 *
 * @notapi
 */
#define _usb_isr_invoke_event_cb(usbp, evt) {                               \
  if (((usbp)->config->event_cb) != NULL) {                                 \
    (usbp)->config->event_cb(usbp, evt);                                    \
  }                                                                         \
}

/**
 * @brief   Common ISR code, SOF callback.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
#define _usb_isr_invoke_sof_cb(usbp) {                                      \
  if (((usbp)->config->sof_cb) != NULL) {                                   \
    (usbp)->config->sof_cb(usbp);                                           \
  }                                                                         \
}

/**
 * @brief   Common ISR code, setup packet callback.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
#define _usb_isr_invoke_setup_cb(usbp, ep) {                                \
  (usbp)->epc[ep]->setup_cb(usbp, ep);                                      \
}

/**
 * @brief   Common ISR code, IN endpoint callback.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
#if (USB_USE_WAIT == TRUE) || defined(__DOXYGEN__)
#define _usb_isr_invoke_in_cb(usbp, ep) {                                   \
  (usbp)->transmitting &= ~(uint16_t)((unsigned)1U << (unsigned)(ep));      \
  if ((usbp)->epc[ep]->in_cb != NULL) {                                     \
    (usbp)->epc[ep]->in_cb(usbp, ep);                                       \
  }                                                                         \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(usbp)->epc[ep]->in_state->thread, MSG_OK);            \
  osalSysUnlockFromISR();                                                   \
}
#else
#define _usb_isr_invoke_in_cb(usbp, ep) {                                   \
  (usbp)->transmitting &= ~(uint16_t)((unsigned)1U << (unsigned)(ep));      \
  if ((usbp)->epc[ep]->in_cb != NULL) {                                     \
    (usbp)->epc[ep]->in_cb(usbp, ep);                                       \
  }                                                                         \
}
#endif

/**
 * @brief   Common ISR code, OUT endpoint event.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
#if (USB_USE_WAIT == TRUE) || defined(__DOXYGEN__)
#define _usb_isr_invoke_out_cb(usbp, ep) {                                  \
  (usbp)->receiving &= ~(uint16_t)((unsigned)1U << (unsigned)(ep));         \
  if ((usbp)->epc[ep]->out_cb != NULL) {                                    \
    (usbp)->epc[ep]->out_cb(usbp, ep);                                      \
  }                                                                         \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(usbp)->epc[ep]->out_state->thread,                    \
                    usbGetReceiveTransactionSizeX(usbp, ep));               \
  osalSysUnlockFromISR();                                                   \
}
#else
#define _usb_isr_invoke_out_cb(usbp, ep) {                                  \
  (usbp)->receiving &= ~(uint16_t)((unsigned)1U << (unsigned)(ep));         \
  if ((usbp)->epc[ep]->out_cb != NULL) {                                    \
    (usbp)->epc[ep]->out_cb(usbp, ep);                                      \
  }                                                                         \
}
#endif
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void usbInit(void);
  void usbObjectInit(USBDriver *usbp);
  msg_t usbStart(USBDriver *usbp, const USBConfig *config);
  void usbStop(USBDriver *usbp);
  void usbInitEndpointI(USBDriver *usbp, usbep_t ep,
                        const USBEndpointConfig *epcp);
  void usbDisableEndpointsI(USBDriver *usbp);
  void usbReadSetupI(USBDriver *usbp, usbep_t ep, uint8_t *buf);
  void usbStartReceiveI(USBDriver *usbp, usbep_t ep,
                        uint8_t *buf, size_t n);
  void usbStartTransmitI(USBDriver *usbp, usbep_t ep,
                         const uint8_t *buf, size_t n);
#if USB_USE_WAIT == TRUE
  msg_t usbReceive(USBDriver *usbp, usbep_t ep, uint8_t *buf, size_t n);
  msg_t usbTransmit(USBDriver *usbp, usbep_t ep, const uint8_t *buf, size_t n);
#endif
  bool usbStallReceiveI(USBDriver *usbp, usbep_t ep);
  bool usbStallTransmitI(USBDriver *usbp, usbep_t ep);
  void usbWakeupHost(USBDriver *usbp);
  void _usb_reset(USBDriver *usbp);
  void _usb_suspend(USBDriver *usbp);
  void _usb_wakeup(USBDriver *usbp);
  void _usb_ep0setup(USBDriver *usbp, usbep_t ep);
  void _usb_ep0in(USBDriver *usbp, usbep_t ep);
  void _usb_ep0out(USBDriver *usbp, usbep_t ep);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_USB == TRUE */

#endif /* HAL_USB_H */

/** @} */
