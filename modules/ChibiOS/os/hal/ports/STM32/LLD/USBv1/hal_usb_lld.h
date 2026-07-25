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
 * @file    USBv1/hal_usb_lld.h
 * @brief   STM32 USB subsystem low level driver header.
 *
 * @addtogroup USB
 * @{
 */

#ifndef HAL_USB_LLD_H
#define HAL_USB_LLD_H

#if HAL_USE_USB || defined(__DOXYGEN__)

#include "stm32_usb.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Maximum endpoint address.
 */
#define USB_MAX_ENDPOINTS                   USB_ENDPOINTS_NUMBER

/**
 * @brief   Status stage handling method.
 */
#define USB_EP0_STATUS_STAGE                USB_EP0_STATUS_STAGE_SW

/**
 * @brief   This device requires the address change after the status packet.
 */
#define USB_SET_ADDRESS_MODE                USB_LATE_SET_ADDRESS

/**
 * @brief   Method for set address acknowledge.
 */
#define USB_SET_ADDRESS_ACK_HANDLING        USB_SET_ADDRESS_ACK_SW

/* Addressing differences in headers.*/
#if !defined(USB_CNTR_L2RES) && defined(USB_CNTR_RESUME)
#define USB_CNTR_L2RES  USB_CNTR_RESUME
#endif

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   USB1 driver enable switch.
 * @details If set to @p TRUE the support for USB1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_USB_USE_USB1) || defined(__DOXYGEN__)
#define STM32_USB_USE_USB1                  FALSE
#endif

/**
 * @brief   Enables the USB device low power mode on suspend.
 */
#if !defined(STM32_USB_LOW_POWER_ON_SUSPEND) || defined(__DOXYGEN__)
#define STM32_USB_LOW_POWER_ON_SUSPEND      FALSE
#endif

/**
 * @brief   USB1 interrupt priority level setting.
 */
#if (!defined(STM32_USB_USB1_HP_IRQ_PRIORITY) &&                           \
     (STM32_USB1_HP_NUMBER != STM32_USB1_LP_NUMBER)) || defined(__DOXYGEN__)
#define STM32_USB_USB1_HP_IRQ_PRIORITY      13
#endif

/**
 * @brief   USB1 interrupt priority level setting.
 */
#if !defined(STM32_USB_USB1_LP_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_USB_USB1_LP_IRQ_PRIORITY      14
#endif

/**
 * @brief   Enables isochronous support.
 * @note    Isochronous support requires special handling and this makes the
 *          code size increase significantly.
 */
#if !defined(STM32_USB_USE_ISOCHRONOUS) || defined(__DOXYGEN__)
#define STM32_USB_USE_ISOCHRONOUS           FALSE
#endif

/**
 * @brief   Use faster copy for packets.
 * @note    Makes the driver larger.
 */
#if !defined(STM32_USB_USE_FAST_COPY) || defined(__DOXYGEN__)
#define STM32_USB_USE_FAST_COPY             FALSE
#endif

/**
 * @brief   Host wake-up procedure duration.
 */
#if !defined(STM32_USB_HOST_WAKEUP_DURATION) || defined(__DOXYGEN__)
#define STM32_USB_HOST_WAKEUP_DURATION      2
#endif

/**
 * @brief   Allowed deviation for the 48MHz clock.
 */
#if !defined(STM32_USB_48MHZ_DELTA) || defined(__DOXYGEN__)
#define STM32_USB_48MHZ_DELTA               0
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_USB_USE_USB1 && !STM32_HAS_USB
#error "USB not present in the selected device"
#endif

#if !STM32_USB_USE_USB1
#error "USB driver activated but no USB peripheral assigned"
#endif

#if !defined(STM32_USBCLK)
#error "STM32_USBCLK not defined"
#endif

#if STM32_USB_USE_USB1 &&                                                   \
    (STM32_USB1_HP_NUMBER != STM32_USB1_LP_NUMBER) &&                       \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_USB_USB1_HP_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to USB HP"
#endif

#if STM32_USB_USE_USB1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_USB_USB1_LP_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to USB LP"
#endif

#if !defined(STM32_USB1_HP_HANDLER)
#error "STM32_USB1_HP_HANDLER not defined"
#endif

#if !defined(STM32_USB1_HP_NUMBER)
#error "STM32_USB1_HP_NUMBER not defined"
#endif

#if !defined(STM32_USB1_LP_HANDLER)
#error "STM32_USB1_LP_HANDLER not defined"
#endif

#if !defined(STM32_USB1_LP_NUMBER)
#error "STM32_USB1_LP_NUMBER not defined"
#endif

#if (STM32_USB_HOST_WAKEUP_DURATION < 2) || (STM32_USB_HOST_WAKEUP_DURATION > 15)
#error "invalid STM32_USB_HOST_WAKEUP_DURATION setting, it must be between 2 and 15"
#endif

/* Maximum clock delta, note, clock is not just checked here but also at
   runtime because the source could be dynamic.*/
#if (STM32_USB_48MHZ_DELTA < 0) || (STM32_USB_48MHZ_DELTA > 120000)
#error "invalid STM32_USB_48MHZ_DELTA setting, it must not exceed 120000"
#endif

#if !defined(HAL_LLD_USE_CLOCK_MANAGEMENT)
#if (STM32_USBCLK < (48000000 - STM32_USB_48MHZ_DELTA)) ||                  \
    (STM32_USBCLK > (48000000 + STM32_USB_48MHZ_DELTA))
#error "the USB USBv1 driver requires a 48MHz clock"
#endif
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of an IN endpoint state structure.
 */
typedef struct {
  /**
   * @brief   Requested transmit transfer size.
   */
  size_t                        txsize;
  /**
   * @brief   Transmitted bytes so far.
   */
  size_t                        txcnt;
  /**
   * @brief   Pointer to the transmission linear buffer.
   */
  const uint8_t                 *txbuf;
#if (USB_USE_WAIT == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Waiting thread.
   */
  thread_reference_t            thread;
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief   Size of the last transmitted packet.
   */
  size_t                        txlast;
} USBInEndpointState;

/**
 * @brief   Type of an OUT endpoint state structure.
 */
typedef struct {
  /**
   * @brief   Requested receive transfer size.
   */
  size_t                        rxsize;
  /**
   * @brief   Received bytes so far.
   */
  size_t                        rxcnt;
  /**
   * @brief   Pointer to the receive linear buffer.
   */
  uint8_t                       *rxbuf;
#if (USB_USE_WAIT == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Waiting thread.
   */
  thread_reference_t            thread;
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief   Number of packets to receive.
   */
  uint16_t                      rxpkts;
} USBOutEndpointState;

/**
 * @brief   Type of an USB endpoint configuration structure.
 * @note    Platform specific restrictions may apply to endpoints.
 */
typedef struct {
  /**
   * @brief   Type and mode of the endpoint.
   */
  uint32_t                      ep_mode;
  /**
   * @brief   Setup packet notification callback.
   * @details This callback is invoked when a setup packet has been
   *          received.
   * @post    The application must immediately call @p usbReadPacket() in
   *          order to access the received packet.
   * @note    This field is only valid for @p USB_EP_MODE_TYPE_CTRL
   *          endpoints, it should be set to @p NULL for other endpoint
   *          types.
   */
  usbepcallback_t               setup_cb;
  /**
   * @brief   IN endpoint notification callback.
   * @details This field must be set to @p NULL if callback is not required.
   */
  usbepcallback_t               in_cb;
  /**
   * @brief   OUT endpoint notification callback.
   * @details This field must be set to @p NULL if callback is not required.
   */
  usbepcallback_t               out_cb;
  /**
   * @brief   IN endpoint maximum packet size.
   * @details This field must be set to zero if the IN endpoint is not used.
   */
  uint16_t                      in_maxsize;
  /**
   * @brief   OUT endpoint maximum packet size.
   * @details This field must be set to zero if the OUT endpoint is not used.
   */
  uint16_t                      out_maxsize;
  /**
   * @brief   @p USBEndpointState associated to the IN endpoint.
   * @details This field must be set to @p NULL if the IN endpoint is not
   *          used.
   */
  USBInEndpointState            *in_state;
  /**
   * @brief   @p USBEndpointState associated to the OUT endpoint.
   * @details This field must be set to @p NULL if the OUT endpoint is not
   *          used.
   */
  USBOutEndpointState           *out_state;
  /* End of the mandatory fields.*/
  /**
   * @brief   Reserved field, not currently used.
   * @note    Initialize this field to 1 in order to be forward compatible.
   */
  uint16_t                      ep_buffers;
  /**
   * @brief   Pointer to a buffer for setup packets.
   * @details Setup packets require a dedicated 8-bytes buffer, set this
   *          field to @p NULL for non-control endpoints.
   */
  uint8_t                       *setup_buf;
} USBEndpointConfig;

/**
 * @brief   Type of an USB driver configuration structure.
 */
typedef struct {
  /**
   * @brief   USB events callback.
   * @details This callback is invoked when an USB driver event is registered.
   */
  usbeventcb_t                  event_cb;
  /**
   * @brief   Device GET_DESCRIPTOR request callback.
   * @note    This callback is mandatory and cannot be set to @p NULL.
   */
  usbgetdescriptor_t            get_descriptor_cb;
  /**
   * @brief   Requests hook callback.
   * @details This hook allows to be notified of standard requests or to
   *          handle non standard requests.
   */
  usbreqhandler_t               requests_hook_cb;
  /**
   * @brief   Start Of Frame callback.
   */
  usbcallback_t                 sof_cb;
  /* End of the mandatory fields.*/
} USBConfig;

/**
 * @brief   Structure representing an USB driver.
 */
struct USBDriver {
  /**
   * @brief   Driver state.
   */
  usbstate_t                    state;
  /**
   * @brief   Current configuration data.
   */
  const USBConfig               *config;
  /**
   * @brief   Bit map of the transmitting IN endpoints.
   */
  uint16_t                      transmitting;
  /**
   * @brief   Bit map of the receiving OUT endpoints.
   */
  uint16_t                      receiving;
  /**
   * @brief   Active endpoints configurations.
   */
  const USBEndpointConfig       *epc[USB_MAX_ENDPOINTS + 1];
  /**
   * @brief   Fields available to user, it can be used to associate an
   *          application-defined handler to an IN endpoint.
   * @note    The base index is one, the endpoint zero does not have a
   *          reserved element in this array.
   */
  void                          *in_params[USB_MAX_ENDPOINTS];
  /**
   * @brief   Fields available to user, it can be used to associate an
   *          application-defined handler to an OUT endpoint.
   * @note    The base index is one, the endpoint zero does not have a
   *          reserved element in this array.
   */
  void                          *out_params[USB_MAX_ENDPOINTS];
  /**
   * @brief   Endpoint 0 state.
   */
  usbep0state_t                 ep0state;
  /**
   * @brief   Next position in the buffer to be transferred through endpoint 0.
   */
  uint8_t                       *ep0next;
  /**
   * @brief   Number of bytes yet to be transferred through endpoint 0.
   */
  size_t                        ep0n;
  /**
   * @brief   Endpoint 0 end transaction callback.
   */
  usbcallback_t                 ep0endcb;
  /**
   * @brief   Setup packet buffer.
   */
  uint8_t                       setup[8];
  /**
   * @brief   Current USB device status.
   */
  uint16_t                      status;
  /**
   * @brief   Assigned USB address.
   */
  uint8_t                       address;
  /**
   * @brief   Current USB device configuration.
   */
  uint8_t                       configuration;
  /**
   * @brief   State of the driver when a suspend happened.
   */
  usbstate_t                    saved_state;
#if defined(USB_DRIVER_EXT_FIELDS)
  USB_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief   Pointer to the next address in the packet memory.
   */
  uint32_t                      pmnext;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Returns the current frame number.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The current frame number.
 *
 * @notapi
 */
#define usb_lld_get_frame_number(usbp) (STM32_USB->FNR & FNR_FN_MASK)

/**
 * @brief   Returns the exact size of a receive transaction.
 * @details The received size can be different from the size specified in
 *          @p usbStartReceiveI() because the last packet could have a size
 *          different from the expected one.
 * @pre     The OUT endpoint must have been configured in transaction mode
 *          in order to use this function.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              Received data size.
 *
 * @notapi
 */
#define usb_lld_get_transaction_size(usbp, ep)                              \
  ((usbp)->epc[ep]->out_state->rxcnt)

#if STM32_USB_HAS_BCDR || defined(__DOXYGEN__)
/**
 * @brief   Connects the USB device.
 *
 * @notapi
 */
#if !defined(usb_lld_connect_bus)
#define usb_lld_connect_bus(usbp) (STM32_USB->BCDR |= USB_BCDR_DPPU)
#endif

/**
 * @brief   Disconnect the USB device.
 *
 * @notapi
 */
#if !defined(usb_lld_disconnect_bus)
#define usb_lld_disconnect_bus(usbp) (STM32_USB->BCDR &= ~USB_BCDR_DPPU)
#endif
#endif /* STM32_USB_HAS_BCDR */

#if defined(STM32L1XX)
#if !defined(usb_lld_connect_bus)
#define usb_lld_connect_bus(usbp) (SYSCFG->PMC |= SYSCFG_PMC_USB_PU)
#endif

#if !defined(usb_lld_disconnect_bus)
#define usb_lld_disconnect_bus(usbp) (SYSCFG->PMC &= ~SYSCFG_PMC_USB_PU)
#endif
#endif /* STM32L1XX */

/**
 * @brief   Start of host wake-up procedure.
 *
 * @notapi
 */
#define usb_lld_wakeup_host(usbp)                                           \
  do {                                                                      \
    STM32_USB->CNTR |= USB_CNTR_L2RES;                                      \
    osalThreadSleepMilliseconds(STM32_USB_HOST_WAKEUP_DURATION);            \
    STM32_USB->CNTR &= ~USB_CNTR_L2RES;                                     \
  } while (false)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_USB_USE_USB1 && !defined(__DOXYGEN__)
extern USBDriver USBD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void usb_lld_init(void);
  void usb_lld_start(USBDriver *usbp);
  void usb_lld_stop(USBDriver *usbp);
  void usb_lld_reset(USBDriver *usbp);
  void usb_lld_set_address(USBDriver *usbp);
  void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep);
  void usb_lld_disable_endpoints(USBDriver *usbp);
  usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep);
  usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep);
  void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf);
  void usb_lld_start_out(USBDriver *usbp, usbep_t ep);
  void usb_lld_start_in(USBDriver *usbp, usbep_t ep);
  void usb_lld_stall_out(USBDriver *usbp, usbep_t ep);
  void usb_lld_stall_in(USBDriver *usbp, usbep_t ep);
  void usb_lld_clear_out(USBDriver *usbp, usbep_t ep);
  void usb_lld_clear_in(USBDriver *usbp, usbep_t ep);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_USB */

#endif /* HAL_USB_LLD_H */

/** @} */
