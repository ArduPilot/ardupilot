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
 * @file    hal_usb.c
 * @brief   USB Driver code.
 *
 * @addtogroup USB
 * @{
 */

#include <string.h>

#include "hal.h"

#if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static const uint8_t zero_status[] = {0x00, 0x00};
static const uint8_t active_status[] = {0x00, 0x00};
static const uint8_t halted_status[] = {0x01, 0x00};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static uint16_t get_hword(uint8_t *p) {
  uint16_t hw;

  hw  = (uint16_t)*p++;
  hw |= (uint16_t)*p << 8U;
  return hw;
}

/**
 * @brief  SET ADDRESS transaction callback.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 */
static void set_address(USBDriver *usbp) {

  usbp->address = usbp->setup[2];
  usb_lld_set_address(usbp);
  _usb_isr_invoke_event_cb(usbp, USB_EVENT_ADDRESS);
  usbp->state = USB_SELECTED;
}

/**
 * @brief   Standard requests handler.
 * @details This is the standard requests default handler, most standard
 *          requests are handled here, the user can override the standard
 *          handling using the @p requests_hook_cb hook in the
 *          @p USBConfig structure.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The request handling exit code.
 * @retval false        Request not recognized by the handler or error.
 * @retval true         Request handled.
 */
static bool default_handler(USBDriver *usbp) {
  const USBDescriptor *dp;

  /* Decoding the request.*/
  switch ((((uint32_t)usbp->setup[0] & (USB_RTYPE_RECIPIENT_MASK |
                                        USB_RTYPE_TYPE_MASK)) |
           ((uint32_t)usbp->setup[1] << 8U))) {
  case (uint32_t)USB_RTYPE_RECIPIENT_DEVICE | ((uint32_t)USB_REQ_GET_STATUS << 8):
    /* Just returns the current status word.*/
    usbSetupTransfer(usbp, (uint8_t *)&usbp->status, 2, NULL);
    return true;
  case (uint32_t)USB_RTYPE_RECIPIENT_DEVICE | ((uint32_t)USB_REQ_CLEAR_FEATURE << 8):
    /* Only the DEVICE_REMOTE_WAKEUP is handled here, any other feature
       number is handled as an error.*/
    if (usbp->setup[2] == USB_FEATURE_DEVICE_REMOTE_WAKEUP) {
      usbp->status &= ~2U;
      usbSetupTransfer(usbp, NULL, 0, NULL);
      return true;
    }
    return false;
  case (uint32_t)USB_RTYPE_RECIPIENT_DEVICE | ((uint32_t)USB_REQ_SET_FEATURE << 8):
    /* Only the DEVICE_REMOTE_WAKEUP is handled here, any other feature
       number is handled as an error.*/
    if (usbp->setup[2] == USB_FEATURE_DEVICE_REMOTE_WAKEUP) {
      usbp->status |= 2U;
      usbSetupTransfer(usbp, NULL, 0, NULL);
      return true;
    }
    return false;
  case (uint32_t)USB_RTYPE_RECIPIENT_DEVICE | ((uint32_t)USB_REQ_SET_ADDRESS << 8):
    /* The SET_ADDRESS handling can be performed here or postponed after
       the status packed depending on the USB_SET_ADDRESS_MODE low
       driver setting.*/
#if USB_SET_ADDRESS_MODE == USB_EARLY_SET_ADDRESS
    if ((usbp->setup[0] == USB_RTYPE_RECIPIENT_DEVICE) &&
        (usbp->setup[1] == USB_REQ_SET_ADDRESS)) {
      set_address(usbp);
    }
    usbSetupTransfer(usbp, NULL, 0, NULL);
#else
    usbSetupTransfer(usbp, NULL, 0, set_address);
#endif
    return true;
  case (uint32_t)USB_RTYPE_RECIPIENT_DEVICE | ((uint32_t)USB_REQ_GET_DESCRIPTOR << 8):
  case (uint32_t)USB_RTYPE_RECIPIENT_INTERFACE | ((uint32_t)USB_REQ_GET_DESCRIPTOR << 8):
    /* Handling descriptor requests from the host.*/
    dp = usbp->config->get_descriptor_cb(usbp, usbp->setup[3],
                                         usbp->setup[2],
                                         get_hword(&usbp->setup[4]));
    if (dp == NULL) {
      return false;
    }
    /*lint -save -e9005 [11.8] Removing const is fine.*/
    usbSetupTransfer(usbp, (uint8_t *)dp->ud_string, dp->ud_size, NULL);
    /*lint -restore*/
    return true;
  case (uint32_t)USB_RTYPE_RECIPIENT_DEVICE | ((uint32_t)USB_REQ_GET_CONFIGURATION << 8):
    /* Returning the last selected configuration.*/
    usbSetupTransfer(usbp, &usbp->configuration, 1, NULL);
    return true;
  case (uint32_t)USB_RTYPE_RECIPIENT_DEVICE | ((uint32_t)USB_REQ_SET_CONFIGURATION << 8):
#if defined(USB_SET_CONFIGURATION_OLD_BEHAVIOR)
    /* Handling configuration selection from the host only if it is different
       from the current configuration.*/
    if (usbp->configuration != usbp->setup[2])
#endif
    {
      /* If the USB device is already active then we have to perform the clear
         procedure on the current configuration.*/
      if (usbp->state == USB_ACTIVE) {
        /* Current configuration cleared.*/
        osalSysLockFromISR ();
        usbDisableEndpointsI(usbp);
        osalSysUnlockFromISR ();
        usbp->configuration = 0U;
        usbp->state = USB_SELECTED;
        _usb_isr_invoke_event_cb(usbp, USB_EVENT_UNCONFIGURED);
      }
      if (usbp->setup[2] != 0U) {
        /* New configuration.*/
        usbp->configuration = usbp->setup[2];
        usbp->state = USB_ACTIVE;
        _usb_isr_invoke_event_cb(usbp, USB_EVENT_CONFIGURED);
      }
    }
    usbSetupTransfer(usbp, NULL, 0, NULL);
    return true;
  case (uint32_t)USB_RTYPE_RECIPIENT_INTERFACE | ((uint32_t)USB_REQ_GET_STATUS << 8):
  case (uint32_t)USB_RTYPE_RECIPIENT_ENDPOINT | ((uint32_t)USB_REQ_SYNCH_FRAME << 8):
    /* Just sending two zero bytes, the application can change the behavior
       using a hook.*/
    /*lint -save -e9005 [11.8] Removing const is fine.*/
    usbSetupTransfer(usbp, (uint8_t *)zero_status, 2, NULL);
    /*lint -restore*/
    return true;
  case (uint32_t)USB_RTYPE_RECIPIENT_ENDPOINT | ((uint32_t)USB_REQ_GET_STATUS << 8):
    /* Sending the EP status.*/
    if ((usbp->setup[4] & 0x80U) != 0U) {
      switch (usb_lld_get_status_in(usbp, usbp->setup[4] & 0x0FU)) {
      case EP_STATUS_STALLED:
        /*lint -save -e9005 [11.8] Removing const is fine.*/
        usbSetupTransfer(usbp, (uint8_t *)halted_status, 2, NULL);
        /*lint -restore*/
        return true;
      case EP_STATUS_ACTIVE:
        /*lint -save -e9005 [11.8] Removing const is fine.*/
        usbSetupTransfer(usbp, (uint8_t *)active_status, 2, NULL);
        /*lint -restore*/
        return true;
      case EP_STATUS_DISABLED:
      default:
        return false;
      }
    }
    else {
      switch (usb_lld_get_status_out(usbp, usbp->setup[4] & 0x0FU)) {
      case EP_STATUS_STALLED:
        /*lint -save -e9005 [11.8] Removing const is fine.*/
        usbSetupTransfer(usbp, (uint8_t *)halted_status, 2, NULL);
        /*lint -restore*/
        return true;
      case EP_STATUS_ACTIVE:
        /*lint -save -e9005 [11.8] Removing const is fine.*/
        usbSetupTransfer(usbp, (uint8_t *)active_status, 2, NULL);
        /*lint -restore*/
        return true;
      case EP_STATUS_DISABLED:
      default:
        return false;
      }
    }
  case (uint32_t)USB_RTYPE_RECIPIENT_ENDPOINT | ((uint32_t)USB_REQ_CLEAR_FEATURE << 8):
    /* Only ENDPOINT_HALT is handled as feature.*/
    if (usbp->setup[2] != USB_FEATURE_ENDPOINT_HALT) {
      return false;
    }
    /* Clearing the EP status, not valid for EP0, it is ignored in that case.*/
    if ((usbp->setup[4] & 0x0FU) != 0U) {
      if ((usbp->setup[4] & 0x80U) != 0U) {
        usb_lld_clear_in(usbp, usbp->setup[4] & 0x0FU);
      }
      else {
        usb_lld_clear_out(usbp, usbp->setup[4] & 0x0FU);
      }
    }
    usbSetupTransfer(usbp, NULL, 0, NULL);
    return true;
  case (uint32_t)USB_RTYPE_RECIPIENT_ENDPOINT | ((uint32_t)USB_REQ_SET_FEATURE << 8):
    /* Only ENDPOINT_HALT is handled as feature.*/
    if (usbp->setup[2] != USB_FEATURE_ENDPOINT_HALT) {
      return false;
    }
    /* Stalling the EP, not valid for EP0, it is ignored in that case.*/
    if ((usbp->setup[4] & 0x0FU) != 0U) {
      if ((usbp->setup[4] & 0x80U) != 0U) {
        usb_lld_stall_in(usbp, usbp->setup[4] & 0x0FU);
      }
      else {
        usb_lld_stall_out(usbp, usbp->setup[4] & 0x0FU);
      }
    }
    usbSetupTransfer(usbp, NULL, 0, NULL);
    return true;
  case (uint32_t)USB_RTYPE_RECIPIENT_DEVICE | ((uint32_t)USB_REQ_SET_DESCRIPTOR << 8):
  case (uint32_t)USB_RTYPE_RECIPIENT_INTERFACE | ((uint32_t)USB_REQ_CLEAR_FEATURE << 8):
  case (uint32_t)USB_RTYPE_RECIPIENT_INTERFACE | ((uint32_t)USB_REQ_SET_FEATURE << 8):
  case (uint32_t)USB_RTYPE_RECIPIENT_INTERFACE | ((uint32_t)USB_REQ_GET_INTERFACE << 8):
  case (uint32_t)USB_RTYPE_RECIPIENT_INTERFACE | ((uint32_t)USB_REQ_SET_INTERFACE << 8):
    /* All the above requests are not handled here, if you need them then
       use the hook mechanism and provide handling.*/
  default:
    return false;
  }
}

/**
 * @brief  Reset setup state machine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 */
static void setup_reset(USBDriver *usbp) {

  usbp->receiving &= ~1U;
  usbp->transmitting &= ~1U;
  usbp->ep0n     = 0;
  usbp->ep0state = USB_EP0_STP_WAITING;
}

/**
 * @brief  Set error in setup state machine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 */
static void setup_error(USBDriver *usbp) {

  usb_lld_stall_in(usbp, 0);
  usb_lld_stall_out(usbp, 0);
  _usb_isr_invoke_event_cb(usbp, USB_EVENT_STALLED);
  usbp->receiving &= ~1U;
  usbp->transmitting &= ~1U;
  usbp->ep0n     = 0;
  usbp->ep0state = USB_EP0_ERROR;
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   USB Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void usbInit(void) {

  usb_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p USBDriver structure.
 *
 * @param[out] usbp     pointer to the @p USBDriver object
 *
 * @init
 */
void usbObjectInit(USBDriver *usbp) {
  unsigned i;

  usbp->state        = USB_STOP;
  usbp->config       = NULL;
  for (i = 0; i < (unsigned)USB_MAX_ENDPOINTS; i++) {
    usbp->in_params[i]  = NULL;
    usbp->out_params[i] = NULL;
  }
  usbp->transmitting = 0;
  usbp->receiving    = 0;
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] config    pointer to the @p USBConfig object
 * @return              The operation status.
 *
 * @api
 */
msg_t usbStart(USBDriver *usbp, const USBConfig *config) {
  msg_t msg;
  unsigned i;

  osalDbgCheck((usbp != NULL) && (config != NULL));

  osalSysLock();
  osalDbgAssert((usbp->state == USB_STOP) || (usbp->state == USB_READY),
                "invalid state");

  usbp->config = config;
  for (i = 0; i <= (unsigned)USB_MAX_ENDPOINTS; i++) {
    usbp->epc[i] = NULL;
  }

#if defined(USB_LLD_ENHANCED_API)
  msg = usb_lld_start(usbp);
  if (msg == HAL_RET_SUCCESS) {
    usbp->state = USB_READY;
  }
  else {
    usbp->state = USB_STOP;
  }
#else
  usb_lld_start(usbp);
  usbp->state = USB_READY;
  msg = HAL_RET_SUCCESS;
#endif

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @api
 */
void usbStop(USBDriver *usbp) {
  unsigned i;

  osalDbgCheck(usbp != NULL);

  osalSysLock();

  osalDbgAssert((usbp->state == USB_STOP) || (usbp->state == USB_READY) ||
                (usbp->state == USB_SELECTED) || (usbp->state == USB_ACTIVE) ||
                (usbp->state == USB_SUSPENDED),
                "invalid state");

  usb_lld_stop(usbp);
  usbp->config = NULL;
  usbp->state  = USB_STOP;

  /* Resetting all ongoing synchronous operations.*/
  for (i = 0; i <= (unsigned)USB_MAX_ENDPOINTS; i++) {
#if USB_USE_WAIT == TRUE
    if (usbp->epc[i] != NULL) {
      if (usbp->epc[i]->in_state != NULL) {
        osalThreadResumeI(&usbp->epc[i]->in_state->thread, MSG_RESET);
      }
      if (usbp->epc[i]->out_state != NULL) {
        osalThreadResumeI(&usbp->epc[i]->out_state->thread, MSG_RESET);
      }
    }
#endif
    usbp->epc[i] = NULL;
  }
  osalOsRescheduleS();

  osalSysUnlock();
}

/**
 * @brief   Enables an endpoint.
 * @details This function enables an endpoint, both IN and/or OUT directions
 *          depending on the configuration structure.
 * @note    This function must be invoked in response of a SET_CONFIGURATION
 *          or SET_INTERFACE message.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[in] epcp      the endpoint configuration
 *
 * @iclass
 */
void usbInitEndpointI(USBDriver *usbp, usbep_t ep,
                      const USBEndpointConfig *epcp) {

  osalDbgCheckClassI();
  osalDbgCheck((usbp != NULL) && (epcp != NULL) &&
               (ep <= (usbep_t)USB_MAX_ENDPOINTS));
  osalDbgAssert(usbp->state == USB_ACTIVE,
                "invalid state");
  osalDbgAssert(usbp->epc[ep] == NULL, "already initialized");

  /* Logically enabling the endpoint in the USBDriver structure.*/
  usbp->epc[ep] = epcp;

  /* Clearing the state structures, custom fields as well.*/
  if (epcp->in_state != NULL) {
    memset(epcp->in_state, 0, sizeof(USBInEndpointState));
  }
  if (epcp->out_state != NULL) {
    memset(epcp->out_state, 0, sizeof(USBOutEndpointState));
  }

  /* Low level endpoint activation.*/
  usb_lld_init_endpoint(usbp, ep);
}

/**
 * @brief   Disables all the active endpoints.
 * @details This function disables all the active endpoints except the
 *          endpoint zero.
 * @note    This function must be invoked in response of a SET_CONFIGURATION
 *          message with configuration number zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @iclass
 */
void usbDisableEndpointsI(USBDriver *usbp) {
  unsigned i;

  osalDbgCheckClassI();
  osalDbgCheck(usbp != NULL);
  osalDbgAssert(usbp->state == USB_ACTIVE, "invalid state");

  usbp->transmitting &= 1U;
  usbp->receiving    &= 1U;

  for (i = 1; i <= (unsigned)USB_MAX_ENDPOINTS; i++) {
#if USB_USE_WAIT == TRUE
    /* Signaling the event to threads waiting on endpoints.*/
    if (usbp->epc[i] != NULL) {
      if (usbp->epc[i]->in_state != NULL) {
        osalThreadResumeI(&usbp->epc[i]->in_state->thread, MSG_RESET);
      }
      if (usbp->epc[i]->out_state != NULL) {
        osalThreadResumeI(&usbp->epc[i]->out_state->thread, MSG_RESET);
      }
    }
#endif
    usbp->epc[i] = NULL;
  }

  /* Low level endpoints deactivation.*/
  usb_lld_disable_endpoints(usbp);
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @note    This function can be invoked from ISR context.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @iclass
 */
void usbReadSetupI(USBDriver *usbp, usbep_t ep, uint8_t *buf) {

  osalDbgCheckClassI();
  osalDbgCheck((usbp != NULL) && (buf != NULL) &&
               (ep <= (usbep_t)USB_MAX_ENDPOINTS));

  usb_lld_read_setup(usbp, ep, buf);
}

/**
 * @brief   Starts a receive transaction on an OUT endpoint.
 * @note    This function is meant to be called from ISR context outside
 *          critical zones because there is a potentially slow operation
 *          inside.
 * @note    The transaction terminates when one of the following conditions
 *          has been met:
 *          - The specified amount of data has been received.
 *          - A short packet has been received.
 *          - A zero-length packet has been received.
 *          - The USB has been reset by host or the driver went into
 *            @p USB_SUSPENDED state.
 *          .
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the received data
 * @param[in] n         transaction size. It is recommended a multiple of
 *                      the packet size because the excess is discarded.
 *
 * @iclass
 */
void usbStartReceiveI(USBDriver *usbp, usbep_t ep,
                      uint8_t *buf, size_t n) {
  USBOutEndpointState *osp;

  osalDbgCheckClassI();
  osalDbgCheck((usbp != NULL) && (ep <= (usbep_t)USB_MAX_ENDPOINTS));
  osalDbgAssert(!usbGetReceiveStatusI(usbp, ep), "already receiving");

  /* Marking the endpoint as active.*/
  usbp->receiving |= (uint16_t)((unsigned)1U << (unsigned)ep);

  /* Setting up the transfer.*/
  /*lint -save -e661 [18.1] pclint is confused by the check on ep.*/
  osp = usbp->epc[ep]->out_state;
  /*lint -restore*/
  osp->rxbuf  = buf;
  osp->rxsize = n;
  osp->rxcnt  = 0;
#if USB_USE_WAIT == TRUE
  osp->thread = NULL;
#endif

  /* Starting transfer.*/
  usb_lld_start_out(usbp, ep);
}

/**
 * @brief   Starts a transmit transaction on an IN endpoint.
 * @note    This function is meant to be called from ISR context outside
 *          critical zones because there is a potentially slow operation
 *          inside.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[in] buf       buffer where to fetch the data to be transmitted
 * @param[in] n         transaction size
 *
 * @iclass
 */
void usbStartTransmitI(USBDriver *usbp, usbep_t ep,
                       const uint8_t *buf, size_t n) {
  USBInEndpointState *isp;

  osalDbgCheckClassI();
  osalDbgCheck((usbp != NULL) && (ep <= (usbep_t)USB_MAX_ENDPOINTS));
  osalDbgAssert(!usbGetTransmitStatusI(usbp, ep), "already transmitting");

  /* Marking the endpoint as active.*/
  usbp->transmitting |= (uint16_t)((unsigned)1U << (unsigned)ep);

  /* Setting up the transfer.*/
  /*lint -save -e661 [18.1] pclint is confused by the check on ep.*/
  isp = usbp->epc[ep]->in_state;
  /*lint -restore*/
  isp->txbuf  = buf;
  isp->txsize = n;
  isp->txcnt  = 0;
#if USB_USE_WAIT == TRUE
  isp->thread = NULL;
#endif

  /* Starting transfer.*/
  usb_lld_start_in(usbp, ep);
}

#if (USB_USE_WAIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Performs a receive transaction on an OUT endpoint.
 * @note    The transaction terminates when one of the following conditions
 *          has been met:
 *          - The specified amount of data has been received.
 *          - A short packet has been received.
 *          - A zero-lenght packet has been received.
 *          - The USB has been reset by host or the driver went into
 *            @p USB_SUSPENDED state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the received data
 * @param[in] n         transaction size. It is recommended a multiple of
 *                      the packet size because the excess is discarded.
 *
 * @return              The received effective data size, it can be less than
 *                      the amount specified.
 * @retval MSG_RESET    driver not in @p USB_ACTIVE state or the operation
 *                      has been aborted by an USB reset or a transition to
 *                      the @p USB_SUSPENDED state.
 *
 * @api
 */
msg_t usbReceive(USBDriver *usbp, usbep_t ep, uint8_t *buf, size_t n) {
  msg_t msg;

  osalSysLock();

  if (usbGetDriverStateI(usbp) != USB_ACTIVE) {
    osalSysUnlock();
    return MSG_RESET;
  }

  usbStartReceiveI(usbp, ep, buf, n);
  msg = osalThreadSuspendS(&usbp->epc[ep]->out_state->thread);
  osalSysUnlock();

  return msg;
}

/**
 * @brief   Performs a transmit transaction on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[in] buf       buffer where to fetch the data to be transmitted
 * @param[in] n         transaction size
 *
 * @return              The operation status.
 * @retval MSG_OK       operation performed successfully.
 * @retval MSG_RESET    driver not in @p USB_ACTIVE state or the operation
 *                      has been aborted by an USB reset or a transition to
 *                      the @p USB_SUSPENDED state.
 *
 * @api
 */
msg_t usbTransmit(USBDriver *usbp, usbep_t ep, const uint8_t *buf, size_t n) {
  msg_t msg;

  osalSysLock();

  if (usbGetDriverStateI(usbp) != USB_ACTIVE) {
    osalSysUnlock();
    return MSG_RESET;
  }

  usbStartTransmitI(usbp, ep, buf, n);
  msg = osalThreadSuspendS(&usbp->epc[ep]->in_state->thread);
  osalSysUnlock();

  return msg;
}
#endif /* USB_USE_WAIT == TRUE */

/**
 * @brief   Stalls an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @return              The operation status.
 * @retval false        Endpoint stalled.
 * @retval true         Endpoint busy, not stalled.
 *
 * @iclass
 */
bool usbStallReceiveI(USBDriver *usbp, usbep_t ep) {

  osalDbgCheckClassI();
  osalDbgCheck(usbp != NULL);

  if (usbGetReceiveStatusI(usbp, ep)) {
    return true;
  }

  usb_lld_stall_out(usbp, ep);
  return false;
}

/**
 * @brief   Stalls an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @return              The operation status.
 * @retval false        Endpoint stalled.
 * @retval true         Endpoint busy, not stalled.
 *
 * @iclass
 */
bool usbStallTransmitI(USBDriver *usbp, usbep_t ep) {

  osalDbgCheckClassI();
  osalDbgCheck(usbp != NULL);

  if (usbGetTransmitStatusI(usbp, ep)) {
    return true;
  }

  usb_lld_stall_in(usbp, ep);
  return false;
}

/**
 * @brief   Host wake-up procedure.
 * @note    It is silently ignored if the USB device is not in the
 *          @p USB_SUSPENDED state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @api
 */
void usbWakeupHost(USBDriver *usbp) {

  if (usbp->state == USB_SUSPENDED) {
    /* Starting host wakeup procedure.*/
    usb_lld_wakeup_host(usbp);
  }
}

/**
 * @brief   USB reset routine.
 * @details This function must be invoked when an USB bus reset condition is
 *          detected.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void _usb_reset(USBDriver *usbp) {
  unsigned i;

  /* State transition.*/
  usbp->state         = USB_READY;

  /* Resetting internal state.*/
  usbp->status        = 0;
  usbp->address       = 0;
  usbp->configuration = 0;
  usbp->ep0n          = 0;
  usbp->transmitting  = 0;
  usbp->receiving     = 0;

  /* Invalidates all endpoints into the USBDriver structure.*/
  for (i = 0; i <= (unsigned)USB_MAX_ENDPOINTS; i++) {
#if USB_USE_WAIT == TRUE
    /* Signaling the event to threads waiting on endpoints.*/
    if (usbp->epc[i] != NULL) {
      osalSysLockFromISR();
      if (usbp->epc[i]->in_state != NULL) {
        osalThreadResumeI(&usbp->epc[i]->in_state->thread, MSG_RESET);
      }
      if (usbp->epc[i]->out_state != NULL) {
        osalThreadResumeI(&usbp->epc[i]->out_state->thread, MSG_RESET);
      }
      osalSysUnlockFromISR();
    }
#endif
    usbp->epc[i] = NULL;
  }

  /* EP0 state machine initialization.*/
  usbp->ep0state = USB_EP0_STP_WAITING;

  /* Low level reset.*/
  usb_lld_reset(usbp);

  /* Notification of reset event.*/
  _usb_isr_invoke_event_cb(usbp, USB_EVENT_RESET);
}

/**
 * @brief   USB suspend routine.
 * @details This function must be invoked when an USB bus suspend condition is
 *          detected.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void _usb_suspend(USBDriver *usbp) {

  /* It could happen that multiple suspend events are triggered.*/
  if (usbp->state != USB_SUSPENDED) {

    /* State transition, saving the current state.*/
    usbp->saved_state = usbp->state;
    usbp->state       = USB_SUSPENDED;

    /* Notification of suspend event.*/
    _usb_isr_invoke_event_cb(usbp, USB_EVENT_SUSPEND);

    /* Terminating all pending transactions.*/
    usbp->transmitting  = 0;
    usbp->receiving     = 0;

    /* Signaling the event to threads waiting on endpoints.*/
  #if USB_USE_WAIT == TRUE
    {
      unsigned i;

      for (i = 0; i <= (unsigned)USB_MAX_ENDPOINTS; i++) {
        if (usbp->epc[i] != NULL) {
          osalSysLockFromISR();
          if (usbp->epc[i]->in_state != NULL) {
            osalThreadResumeI(&usbp->epc[i]->in_state->thread, MSG_RESET);
          }
          if (usbp->epc[i]->out_state != NULL) {
            osalThreadResumeI(&usbp->epc[i]->out_state->thread, MSG_RESET);
          }
          osalSysUnlockFromISR();
        }
      }
    }
  #endif
  }
}

/**
 * @brief   USB wake-up routine.
 * @details This function must be invoked when an USB bus wake-up condition is
 *          detected.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void _usb_wakeup(USBDriver *usbp) {

  /* It could happen that multiple wakeup events are triggered.*/
  if (usbp->state == USB_SUSPENDED) {

    /* State transition, returning to the previous state.*/
    usbp->state = usbp->saved_state;

    /* Notification of suspend event.*/
    _usb_isr_invoke_event_cb(usbp, USB_EVENT_WAKEUP);
  }
}

/**
 * @brief   Default EP0 SETUP callback.
 * @details This function is used by the low level driver as default handler
 *          for EP0 SETUP events.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number, always zero
 *
 * @notapi
 */
void _usb_ep0setup(USBDriver *usbp, usbep_t ep) {

  osalDbgAssert(ep == 0, "EP not zero");
  size_t max;

  /* Is the EP0 state machine in the correct state for handling setup
     packets?*/
  if (usbp->ep0state != USB_EP0_STP_WAITING) {
    /* If a new SETUP packet arrives from the host while the device is still
     in a previous control transfer (for example, an OUT data transfer), the
     ongoing transfer is aborted. The EP0 state machine and transmitting and
     receiving bits, must be reset. The count is also reset.*/
    /* EP0 is driven by the control-transfer state machine; no waiters
       are expected on EP0 transfers.*/
    setup_reset(usbp);
  }

  /* Reading the setup data into the driver buffer.*/
  usbReadSetup(usbp, 0, usbp->setup);

  /* First verify if the application has an handler installed for this
     request.*/
  /*lint -save -e9007 [13.5] No side effects, it is intentional.*/
  if ((usbp->config->requests_hook_cb == NULL) ||
      !(usbp->config->requests_hook_cb(usbp))) {
  /*lint -restore*/
    /* Invoking the default handler, if this fails then stalls the
       endpoint zero as error.*/
    /*lint -save -e9007 [13.5] No side effects, it is intentional.*/
    if (((usbp->setup[0] & USB_RTYPE_TYPE_MASK) != USB_RTYPE_TYPE_STD) ||
        !default_handler(usbp)) {
    /*lint -restore*/
      /* Error response, the state machine goes into an error state, the low
         level layer restores setup state based on low level events.*/
      setup_error(usbp);
      return;
    }
  }
#if (USB_SET_ADDRESS_ACK_HANDLING == USB_SET_ADDRESS_ACK_HW)
  if (usbp->setup[1] == USB_REQ_SET_ADDRESS) {
    /* Zero-length packet sent by hardware */
    return;
  }
#endif
  /* Transfer preparation. The request handler must have populated
     correctly the fields ep0next, ep0n and ep0endcb using the macro
     usbSetupTransfer().*/
  max = (size_t)get_hword(&usbp->setup[6]);
  /* The transfer size cannot exceed the specified amount.*/
  if (usbp->ep0n > max) {
    usbp->ep0n = max;
  }
  if ((usbp->setup[0] & USB_RTYPE_DIR_MASK) == USB_RTYPE_DIR_DEV2HOST) {
    /* IN phase.*/
    if (usbp->ep0n != 0U) {
      /* Starts the transmit phase.*/
      usbp->ep0state = USB_EP0_IN_TX;
      osalSysLockFromISR();
      usbStartTransmitI(usbp, 0, usbp->ep0next, usbp->ep0n);
      osalSysUnlockFromISR();
    }
    else {
      /* No transmission phase, directly receiving the zero sized status
         packet.*/
      usbp->ep0state = USB_EP0_OUT_WAITING_STS;
#if (USB_EP0_STATUS_STAGE == USB_EP0_STATUS_STAGE_SW)
      osalSysLockFromISR();
      usbStartReceiveI(usbp, 0, NULL, 0);
      osalSysUnlockFromISR();
#else
      usb_lld_end_setup(usbp, 0);
#endif
    }
  }
  else {
    /* OUT phase.*/
    if (usbp->ep0n != 0U) {
      /* Starts the receive phase.*/
      usbp->ep0state = USB_EP0_OUT_RX;
      osalSysLockFromISR();
      usbStartReceiveI(usbp, 0, (uint8_t *)usbp->ep0next, usbp->ep0n);
      osalSysUnlockFromISR();
    }
    else {
      /* No receive phase, directly sending the zero sized status
         packet.*/
      usbp->ep0state = USB_EP0_IN_SENDING_STS;
#if (USB_EP0_STATUS_STAGE == USB_EP0_STATUS_STAGE_SW)
      osalSysLockFromISR();
      usbStartTransmitI(usbp, 0, NULL, 0);
      osalSysUnlockFromISR();
#else
      usb_lld_end_setup(usbp, 0);
#endif
    }
  }
}

/**
 * @brief   Default EP0 IN callback.
 * @details This function is used by the low level driver as default handler
 *          for EP0 IN events.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number, always zero
 *
 * @notapi
 */
void _usb_ep0in(USBDriver *usbp, usbep_t ep) {

  osalDbgAssert(ep == 0, "EP not zero");

  size_t max;
  switch (usbp->ep0state) {
  case USB_EP0_IN_TX:
    max = (size_t)get_hword(&usbp->setup[6]);
    /* If the transmitted size is less than the requested size and it is a
       multiple of the maximum packet size then a zero size packet must be
       transmitted.*/
    if ((usbp->ep0n < max) &&
        ((usbp->ep0n % usbp->epc[0]->in_maxsize) == 0U)) {
      osalSysLockFromISR();
      usbStartTransmitI(usbp, 0, NULL, 0);
      osalSysUnlockFromISR();
      usbp->ep0state = USB_EP0_IN_WAITING_TX0;
      return;
    }
    /* Falls through.*/
  case USB_EP0_IN_WAITING_TX0:
    /* Transmit phase over, receiving the zero sized status packet.*/
    usbp->ep0state = USB_EP0_OUT_WAITING_STS;
#if (USB_EP0_STATUS_STAGE == USB_EP0_STATUS_STAGE_SW)
    osalSysLockFromISR();
    usbStartReceiveI(usbp, 0, NULL, 0);
    osalSysUnlockFromISR();
#else
    usb_lld_end_setup(usbp, 0);
#endif
    return;
  case USB_EP0_IN_SENDING_STS:
    /* Status packet sent, invoking the callback if defined.*/
    if (usbp->ep0endcb != NULL) {
      usbp->ep0endcb(usbp);
    }

    /* Put setup back in ready state.*/
    setup_reset(usbp);
    return;

  case USB_EP0_OUT_WAITING_STS:
    /* Tolerate out of order setup.*/
    return;
  case USB_EP0_STP_WAITING:
  case USB_EP0_OUT_RX:
    /* All the above are invalid states in the IN phase.*/
    osalDbgAssert(false, "EP0 state machine error");
    /* Falls through.*/
  case USB_EP0_ERROR:
    /* Error response, the state machine goes into an error state, the low
       level layer restores setup state based on low level events.*/
    setup_error(usbp);
    return;
  default:
    osalDbgAssert(false, "EP0 state machine invalid state");
  }
}

/**
 * @brief   Default EP0 OUT callback.
 * @details This function is used by the low level driver as default handler
 *          for EP0 OUT events.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number, always zero
 *
 * @notapi
 */
void _usb_ep0out(USBDriver *usbp, usbep_t ep) {

  osalDbgAssert(ep == 0, "EP not zero");

  switch (usbp->ep0state) {
  case USB_EP0_OUT_RX:
    /* Receive phase over, sending the zero sized status packet.*/
    usbp->ep0state = USB_EP0_IN_SENDING_STS;
#if (USB_EP0_STATUS_STAGE == USB_EP0_STATUS_STAGE_SW)
    osalSysLockFromISR();
    usbStartTransmitI(usbp, 0, NULL, 0);
    osalSysUnlockFromISR();
#else
    usb_lld_end_setup(usbp, 0);
#endif
    return;
  case USB_EP0_OUT_WAITING_STS:
    /* Status packet received, it must be zero sized, invoking the callback
       if defined.*/
#if (USB_EP0_STATUS_STAGE == USB_EP0_STATUS_STAGE_SW)
    if (usbGetReceiveTransactionSizeX(usbp, 0) != 0U) {
      break;
    }
#endif
    if (usbp->ep0endcb != NULL) {
      usbp->ep0endcb(usbp);
    }

    /* Put setup back in ready state.*/
    setup_reset(usbp);
    return;

  case USB_EP0_IN_TX:
    /* Tolerate out of order setup.*/
    return;
  case USB_EP0_STP_WAITING:
  case USB_EP0_IN_WAITING_TX0:
  case USB_EP0_IN_SENDING_STS:
    /* All the above are invalid states in the IN phase.*/
    osalDbgAssert(false, "EP0 state machine error");
    /* Falls through.*/
  case USB_EP0_ERROR:
    /* Error response, the state machine goes into an error state, the low
       level layer restores setup state based on low level events.*/
    setup_error(usbp);
    return;
  default:
    osalDbgAssert(false, "EP0 state machine invalid state");
  }
}

#endif /* HAL_USE_USB == TRUE */

/** @} */
