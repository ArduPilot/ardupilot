/*
    Copyright (C) 2015 Robert Lippert

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
 * @file    USBv1/hal_usb_lld.c
 * @brief   AVR/MEGA USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include "hal.h"

#if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)

#ifndef F_USB
#define F_USB F_CPU
#endif

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

/**
 * @brief   USB1 driver identifier.
 */
#if (AVR_USB_USE_USB1 == TRUE) || defined(__DOXYGEN__)
USBDriver USBD1;
#endif

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/**
 * @brief   EP0 state.
 * @note    It is an union because IN and OUT endpoints are never used at the
 *          same time for EP0.
 */
static union {
  /**
   * @brief   IN EP0 state.
   */
  USBInEndpointState in;
  /**
   * @brief   OUT EP0 state.
   */
  USBOutEndpointState out;
} ep0_state;

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  USB_EP_MODE_TYPE_CTRL,
  _usb_ep0setup,
  _usb_ep0in,
  _usb_ep0out,
  0x40,
  0x40,
  &ep0_state.in,
  &ep0_state.out
};

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

#ifdef AVR_USB_PLL_OFF_IN_SUSPEND
static __attribute__((unused)) void usb_pll_off(void)  {
  PLLCSR = 0;
}
#endif

static void usb_pll_on(void) {
#if (F_USB == 8000000)
  #if (defined(__AVR_AT90USB82__) || defined(__AVR_AT90USB162__) || \
       defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
       defined(__AVR_ATmega32U2__))
    #define PLL_VAL                0
  #elif (defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__))
    #define PLL_VAL                0
  #elif (defined(__AVR_AT90USB646__)  || defined(__AVR_AT90USB1286__))
    #define PLL_VAL                ((0 << PLLP2) | (1 << PLLP1) | (1 << PLLP0))
  #elif (defined(__AVR_AT90USB647__)  || defined(__AVR_AT90USB1287__))
    #define PLL_VAL                ((0 << PLLP2) | (1 << PLLP1) | (1 << PLLP0))
  #endif
#elif (F_USB == 16000000)
  #if (defined(__AVR_AT90USB82__) || defined(__AVR_AT90USB162__) || \
       defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
       defined(__AVR_ATmega32U2__))
    #define PLL_VAL                ((0 << PLLP2) | (0 << PLLP1) | (1 << PLLP0))
  #elif (defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__))
    #define PLL_VAL                (1 << PINDIV)
  #elif (defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB647__))
    #define PLL_VAL                ((1 << PLLP2) | (1 << PLLP1) | (0 << PLLP0))
  #elif (defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1287__))
    #define PLL_VAL                ((1 << PLLP2) | (0 << PLLP1) | (1 << PLLP0))
  #endif
#endif

#ifndef PLL_VAL
#error Could not determine PLL value, unsupported AVR USB model type
#endif

#ifdef PLLFRQ
  /* This initializes PLL on supported devices for USB 48MHz *only*. */
  PLLFRQ = (0 << PDIV3) | (1 << PDIV2) | (0 << PDIV1) | (0 << PDIV0);
#endif

  PLLCSR = PLL_VAL;
  PLLCSR = PLL_VAL | (1 << PLLE);
}

static int usb_pll_is_locked(void) {

  return !!(PLLCSR & (1 << PLOCK));
}

/*==========================================================================*/
/* Driver interrupt handlers and threads.                                   */
/*==========================================================================*/

/**
 * @brief   USB general/OTG/device management event interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USB_GEN_vect) {

  uint8_t usbint, udint;
  USBDriver * const usbp = &USBD1;

  OSAL_IRQ_PROLOGUE();

  usbint = USBINT;
  udint = UDINT;

  if (usbint & (1 << VBUSTI)) {
    /* Connected. */
#ifdef AVR_USB_PLL_OFF_IN_SUSPEND
    usb_pll_on();
    while (!usb_pll_is_locked()) {}
#endif  /* AVR_USB_PLL_OFF_IN_SUSPEND */

    /* Attach to bus. */
    usb_lld_connect_bus(usbp);
    USBINT &= ~(1 << VBUSTI);
  }

  /* USB bus SUSPEND condition handling. */
  if (udint & (1 << SUSPI)) {
    /* Disable suspend interrupt, enable WAKEUP interrupt. */
    UDIEN |= (1 << WAKEUPE);
    UDINT &= ~(1 << WAKEUPI);
    UDIEN &= ~(1 << SUSPE);

    /* Freeze the clock to reduce power consumption. */
    USBCON |= (1 << FRZCLK);
#ifdef AVR_USB_PLL_OFF_IN_SUSPEND
    usb_pll_off();
#endif  /* AVR_USB_PLL_OFF_IN_SUSPEND */

    /* Clear the interrupt. */
    UDINT &= ~(1 << SUSPI);

    _usb_isr_invoke_event_cb(usbp, USB_EVENT_SUSPEND);
  }

  /* USB bus WAKEUP condition handling. */
  if (udint & (1 << WAKEUPI)) {
#ifdef AVR_USB_PLL_OFF_IN_SUSPEND
    usb_pll_on();
    while (!usb_pll_is_locked()) {}
#endif  /* AVR_USB_PLL_OFF_IN_SUSPEND */

    /* Unfreeze the clock. */
    USBCON &= ~(1 << FRZCLK);

    /* Clear & disable wakeup interrupt, enable suspend interrupt. */
    UDINT &= ~(1 << WAKEUPI);
    UDIEN &= ~(1 << WAKEUPE);
    UDIEN |= (1 << SUSPE);

    _usb_isr_invoke_event_cb(usbp, USB_EVENT_WAKEUP);
  }

  /* USB bus RESUME condition handling. */
  if (udint & (1 << EORSMI)) {
    UDINT &= ~(1 << EORSMI);
    UDIEN &= ~(1 << EORSME);
  }

  /* USB bus reset condition handling. */
  if (udint & (1 << EORSTI)) {
    UDINT &= ~(1 << EORSTI);

    /* Clear & disable suspend interrupt, enable WAKEUP interrupt. */
    UDINT &= ~(1 << SUSPI);
    UDIEN &= ~(1 << SUSPE);
    UDIEN |= (1 << WAKEUPE);

    /* Reinitialize EP0.  This is not mentioned in the datasheet but
     * apparently is required. */
    usb_lld_init_endpoint(usbp, 0);

    _usb_isr_invoke_event_cb(usbp, USB_EVENT_RESET);
  }

  /* Start-Of-Frame handling, only if enabled. */
  if ((UDIEN & (1 << SOFE)) && (udint & (1 << SOFI))) {
    _usb_isr_invoke_sof_cb(usbp);
    UDINT &= ~(1 << SOFI);
  }

  OSAL_IRQ_EPILOGUE();
}

static void usb_fifo_write(USBDriver *usbp, usbep_t ep, size_t n) {

  const USBEndpointConfig *epcp = usbp->epc[ep];
  USBInEndpointState *isp = epcp->in_state;
  syssts_t sts;
  if (n == 0) {
    isp->last_tx_size = 0;
    return;
  }

  if (n > epcp->in_maxsize)
    n = epcp->in_maxsize;
  /* I is number of bytes remaining to transmit minus 1 (to handle 256b case). */
  uint8_t i = n - 1;

  /* Must lock for entire operation to ensure nothing changes the ENUM value. */
  sts = osalSysGetStatusAndLockX();
  UENUM = ep & 0xf;
  do {
    UEDATX = *isp->txbuf++;
  } while (i--);
  isp->last_tx_size = n;
  osalSysRestoreStatusX(sts);
}

static void usb_fifo_read(USBDriver *usbp, usbep_t ep, size_t n) {

  const USBEndpointConfig *epcp = usbp->epc[ep];
  USBOutEndpointState *osp = epcp->out_state;
  syssts_t sts;
  if (n == 0)
    return;
  if (n > epcp->out_maxsize)
    n = epcp->out_maxsize;
  /* I is number of bytes remaining to receive minus 1 (to handle 256b case). */
  uint8_t i = n - 1;

  /* Must lock for entire operation to ensure nothing changes the ENUM value. */
  sts = osalSysGetStatusAndLockX();
  UENUM = ep & 0xf;
  do {
    *osp->rxbuf++ = UEDATX;
  } while (i--);
  osalSysRestoreStatusX(sts);
}

static void ep_isr(USBDriver *usbp, usbep_t ep) {

  const USBEndpointConfig *epcp = usbp->epc[ep];
  size_t n;
  UENUM = ep & 0xf;

  /* TODO: if stalling is needed/expected remove this check. */
  osalDbgAssert(!(UEINTX & (1 << STALLEDI)), "Endpoint stalled!");

  if ((UEIENX & (1 << TXINE)) && (UEINTX & (1 << TXINI))) {
    /* Ready to accept more IN data to transmit to host. */
    /* Update transaction counts to reflect newly transmitted bytes. */
    epcp->in_state->txcnt += epcp->in_state->last_tx_size;
    n = epcp->in_state->txsize - epcp->in_state->txcnt;
    if (n > 0) {
      /* Transfer not completed, there are more packets to send. */
      usb_fifo_write(usbp, ep, n);

      /* Clear FIFOCON to send the data in the FIFO and switch bank. */
      UEINTX &= ~((1 << TXINI) | (1 << FIFOCON));
      /* Enable the TX complete interrupt. */
      UEIENX |= (1 << TXINE);
    } else {
      /* Disable TXIN interrupt. */
      UEIENX &= ~(1 << TXINE);
      /* Handshake interrupt status. */
      UEINTX &= ~(1 << TXINI);
      _usb_isr_invoke_in_cb(usbp, ep);
    }
  } else if ((UEIENX & (1 << RXSTPE)) && (UEINTX & (1 << RXSTPI))) {
    /* Received SETUP data. */
    /* Reset transaction state for endpoint. */
    epcp->in_state->txcnt = 0;
    epcp->in_state->txsize = 0;
    epcp->in_state->last_tx_size = 0;
    /* Setup packets handling, setup packets are handled using a
       specific callback. */
    _usb_isr_invoke_setup_cb(usbp, ep);
  } else if ((UEIENX & (1 << RXOUTE)) && (UEINTX & (1 << RXOUTI))) {
    /* Received OUT data from host. */
    if (ep == 0 && usbp->ep0state == USB_EP0_OUT_WAITING_STS) {
      /* SETUP/control transaction complete, invoke the callback. */
      UEIENX &= ~(1 << RXOUTE);
      UEINTX &= ~((1 << RXOUTI) | (1 << FIFOCON));
      _usb_isr_invoke_out_cb(usbp, ep);
    } else {
      /* Check the FIFO byte count to see how many bytes were received. */
      n = UEBCX;

      usb_fifo_read(usbp, ep, n);

      /* Transaction state update. */
      epcp->out_state->rxcnt += n;
      epcp->out_state->rxsize -= n;
      epcp->out_state->rxpkts -= 1;
      if (n < epcp->out_maxsize || epcp->out_state->rxpkts == 0) {
        /* Disable OUT interrupt. */
        UEIENX &= ~(1 << RXOUTE);
        /* Mark OUT FIFO processed to allow more data to be received. */
        UEINTX &= ~((1 << RXOUTI) | (1 << FIFOCON));
        /* Transfer complete, invokes the callback. */
        _usb_isr_invoke_out_cb(usbp, ep);
      } else {
        /* Mark OUT FIFO processed to allow more data to be received. */
        UEINTX &= ~((1 << RXOUTI) | (1 << FIFOCON));
      }
    }
  }
}

/**
 * @brief   USB communication event interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USB_COM_vect) {

  USBDriver *usbp = &USBD1;
  const uint8_t epnum_orig = UENUM;
  uint8_t i;

  OSAL_IRQ_PROLOGUE();

  /* Figure out which endpoint(s) are interrupting. */
  for (i = 0; i < USB_MAX_ENDPOINTS; ++i) {
    if (UEINT & (1 << i)) {
      ep_isr(usbp, i);
    }
  }

  /* Restore endpoint selector to pre-interrupt state. */
  UENUM = epnum_orig;

  OSAL_IRQ_EPILOGUE();
}

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {

#if AVR_USB_USE_USB1 == TRUE
  /* Driver initialization. */
  usbObjectInit(&USBD1);

  /* Start and lock the USB 48MHz PLL (takes ~100ms). */
  usb_pll_on();
  while (!usb_pll_is_locked()) {}
#endif
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {

  if (usbp->state == USB_STOP) {
    /* Enables the peripheral. */
#if AVR_USB_USE_USB1 == TRUE
    if (&USBD1 == usbp) {
      uint8_t i;
      /*
       * Workaround: disable pad drivers as first step in case bootloader left
       * it on.  Otherwise VBUS detection interrupt will not trigger later.
       */
      USBCON &= ~(1 << OTGPADE);

      /* Enable the internal 3.3V pad regulator. */
      UHWCON |= (1 << UVREGE);

      /* Reset and disable all endpoints. */
      UERST = 0x7f;
      UERST = 0;
      for (i = 0; i < USB_MAX_ENDPOINTS; ++i) {
        UENUM   = i;
        UEIENX  = 0;
        UEINTX  = 0;
        UECFG1X = 0;
        UECONX &= ~(1 << EPEN);
      }
    }
#endif
    /* Reset procedure enforced on driver start. */
    _usb_reset(usbp);
  }
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {

  if (usbp->state == USB_READY) {
    /* Disables the peripheral. */
#if AVR_USB_USE_USB1 == TRUE
    if (&USBD1 == usbp) {
      /* Disable and clear transition interrupts. */
#if !defined(__AVR_ATmega32U4__)
      USBCON &= ~((1 << VBUSTE) | (1 << IDTE));
#else
      USBCON &= ~(1 << VBUSTE);
#endif

      USBINT = 0;

      /* Disable and clear device interrupts. */
      UDIEN &= ~((1 << UPRSME) | (1 << EORSME) | (1 << WAKEUPE) | (1 << EORSTE)
          | (1 << SOFE) | (1 << SUSPE));
      UDINT = 0;

      /* Freeze clock. */
      USBCON |= (1 << FRZCLK);

      /* Disable USB logic. */
      USBCON &= ~(1 << USBE);
    }
#endif
  }
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbp) {

  /* Post-reset initialization. */
  /* Reset and enable via toggling the USB macro logic overall enable bit. */
  USBCON &= ~(1 << USBE);
  USBCON |= (1 << USBE);

  /* Unfreeze clock. */
  USBCON &= ~(1 << FRZCLK);

  /* Set Device mode. */
  /* TODO: Support HOST/OTG mode if needed. */

#if !defined(__AVR_ATmega32U4__)
  UHWCON |= (1 << UIMOD);
#endif

  /* Set FULL 12mbps speed. */
  UDCON &= ~(1 << LSM);

  /* Enable device pin interrupt. */
  USBCON |= (1 << VBUSTE);

  /* EP0 initialization. */
  UERST |= (1 << 0);
  UERST &= ~(1 << 0);
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0);

  /* Enable device-level event interrupts. */
  UDINT &= ~(1 << SUSPI);
  UDIEN = (1 << UPRSME) | (1 << EORSME) | (1 << WAKEUPE) | (1 << EORSTE)
      | (1 << SUSPE);
  /* The SOF interrupt is only enabled if a callback is defined for
     this service because it is a high rate source. */
  if (usbp->config->sof_cb != NULL)
    UDIEN |= (1 << SOFE);

  /* Set OTG PAD to on which will trigger VBUS transition if plugged in. */
  USBCON |= (1 << OTGPADE);
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {

  UDADDR = (UDADDR & (1 << ADDEN)) | (usbp->address & 0x7F);

  UDADDR |= (1 << ADDEN);
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep) {

  uint16_t size = 0;
  const USBEndpointConfig *epcp = usbp->epc[ep];

  /* Select this endpoint number for subsequent commands. */
  UENUM = ep & 0xf;

  /* Enable endpoint to take out of reset. */
  UECONX |= (1 << EPEN);

  UECFG1X = 0;
  /* Set the endpoint type. */
  switch (epcp->ep_mode & USB_EP_MODE_TYPE) {
  case USB_EP_MODE_TYPE_ISOC:
    UECFG0X = (0 << EPTYPE1) | (1 << EPTYPE0);
    break;
  case USB_EP_MODE_TYPE_BULK:
    UECFG0X = (1 << EPTYPE1) | (0 << EPTYPE0);
    break;
  case USB_EP_MODE_TYPE_INTR:
    UECFG0X = (1 << EPTYPE1) | (1 << EPTYPE0);
    break;
  default:
    UECFG0X = (0 << EPTYPE1) | (0 << EPTYPE0);
  }
  if ((epcp->ep_mode & USB_EP_MODE_TYPE) == USB_EP_MODE_TYPE_CTRL) {
    /* CTRL endpoint. */
    osalDbgCheck(epcp->in_maxsize == epcp->out_maxsize);
    size = epcp->in_maxsize;
  } else {
    osalDbgAssert(!(epcp->in_cb != NULL && epcp->out_cb != NULL),
                  "On AVR each endpoint can be IN or OUT not both");

    /* IN endpoint? */
    if (epcp->in_cb != NULL) {
      UECFG0X |= (1 << EPDIR);
      size = epcp->in_maxsize;
    }

    /* OUT endpoint? */
    if (epcp->out_cb != NULL) {
      UECFG0X &= ~(1 << EPDIR);
      size = epcp->out_maxsize;
    }
  }

  /* Endpoint size and address initialization. */
  switch (size) {
    case 8: UECFG1X = (0 << EPSIZE0) | (1 << ALLOC); break;
    case 16: UECFG1X = (1 << EPSIZE0) | (1 << ALLOC); break;
    case 32: UECFG1X = (2 << EPSIZE0) | (1 << ALLOC); break;
    case 64: UECFG1X = (3 << EPSIZE0) | (1 << ALLOC); break;
    case 128:
      osalDbgAssert(ep == 1, "Endpoint size of 128 bytes only valid for EP#1");
      UECFG1X = (4 << EPSIZE0) | (1 << ALLOC); break;
    case 256:
      osalDbgAssert(ep == 1, "Endpoint size of 256 bytes only valid for EP#1");
      UECFG1X = (5 << EPSIZE0) | (1 << ALLOC); break;
    default:
      osalDbgAssert(false, "Invalid size for USB endpoint");
  }

  UEIENX |=  (1 << RXSTPE)/* | (1 << RXOUTE) */ | (1 << STALLEDE) ;

  osalDbgAssert((UESTA0X & (1 << CFGOK)),
                "Hardware reports endpoint config is INVALID");
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {

  uint8_t i;
  for (i = 1; i <= USB_MAX_ENDPOINTS; ++i) {
    UENUM = i;
    UECFG1X &= ~(1 << ALLOC);
    UECONX &= ~(1 << EPEN);
  }
}

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep) {

  /* Select this endpoint number for subsequent commands. */
  UENUM = ep & 0xf;

  if (!(UECONX & (1 << EPEN)))
    return EP_STATUS_DISABLED;
  if (UECONX & (1 << STALLRQ))
    return EP_STATUS_STALLED;
  return EP_STATUS_ACTIVE;
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep) {

  return usb_lld_get_status_out(usbp, ep);
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @notapi
 */
void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf) {

  uint8_t i;
  /* Select this endpoint number for subsequent commands. */
  UENUM = ep & 0xf;

  for (i = 0; i < 8; ++i) {
    *buf++ = UEDATX;
  }
  /* Clear FIFOCON and RXSTPI to drain the setup packet data from the FIFO. */
  UEINTX &= ~((1 << FIFOCON) | (1 << RXSTPI));
}

/**
 * @brief   Ends a SETUP transaction
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to finish an entire setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_end_setup(USBDriver *usbp, usbep_t ep) {

  /* Select this endpoint number for subsequent commands. */
  UENUM = ep & 0xf;

  if ((usbp->setup[0] & USB_RTYPE_DIR_MASK) == USB_RTYPE_DIR_DEV2HOST) {
    /* Enable interrupt and wait for OUT packet. */
    usbp->epc[ep]->out_state->rxsize = 0;
    usbp->epc[ep]->out_state->rxpkts = 1;

    UEINTX &= ~((1 << FIFOCON) | (1 << RXOUTI));
    UEIENX |= (1 << RXOUTE);
  } else {
    /* Enable interrupt and wait for IN packet. */
    usbp->epc[ep]->in_state->last_tx_size = 0;
    usbp->epc[ep]->in_state->txcnt = 0;
    usbp->epc[ep]->in_state->txsize = 0;

    UEINTX &= ~((1 << FIFOCON) | (1 << TXINI));
    UEIENX |= (1 << TXINE);
  }
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbp, usbep_t ep) {

  USBOutEndpointState *osp = usbp->epc[ep]->out_state;
  syssts_t sts;

  /* Initialize transfer by recording how many packets we expect to receive. */
  if (osp->rxsize == 0)         /* Special case for zero sized packets. */
    osp->rxpkts = 1;
  else
    osp->rxpkts = (uint8_t)((osp->rxsize + usbp->epc[ep]->out_maxsize - 1) /
                             usbp->epc[ep]->out_maxsize);

  /* Select this endpoint number for subsequent commands. */
  /* Must lock for entire operation to ensure nothing changes the ENUM value. */
  sts = osalSysGetStatusAndLockX();
  UENUM = ep & 0xf;

  UEIENX |= (1 << RXOUTE);
  osalSysRestoreStatusX(sts);
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_in(USBDriver *usbp, usbep_t ep) {

  USBInEndpointState *isp = usbp->epc[ep]->in_state;
  syssts_t sts;

  /* Initialize transfer by filling FIFO with passed data. */
  usb_fifo_write(usbp, ep, isp->txsize);

  /* Select this endpoint number for subsequent commands. */
  /* Must lock for entire operation to ensure nothing changes the ENUM value. */
  sts = osalSysGetStatusAndLockX();
  UENUM = ep & 0xf;

  /* Clear FIFOCON to send the data in the FIFO and switch bank. */
  UEINTX &= ~((1 << TXINI) | (1 << FIFOCON));

  /* Enable the TX complete interrupt. */
  UEIENX |= (1 << TXINE);

  osalSysRestoreStatusX(sts);
}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver *usbp, usbep_t ep) {

  syssts_t sts;
  (void)usbp;

  /* Select this endpoint number for subsequent commands. */
  /* Must lock for entire operation to ensure nothing changes the ENUM value. */
  sts = osalSysGetStatusAndLockX();
  UENUM = ep & 0xf;

  UECONX |= (1 << STALLRQ);
  osalSysRestoreStatusX(sts);
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver *usbp, usbep_t ep) {

  usb_lld_stall_out(usbp, ep);
}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver *usbp, usbep_t ep) {

  syssts_t sts;
  (void)usbp;

  /* Select this endpoint number for subsequent commands. */
  /* Must lock for entire operation to ensure nothing changes the ENUM value. */
  sts = osalSysGetStatusAndLockX();
  UENUM = ep & 0xf;

  UECONX |= (1 << STALLRQC);
  osalSysRestoreStatusX(sts);
}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver *usbp, usbep_t ep) {

  usb_lld_clear_out(usbp, ep);
}

#endif /* HAL_USE_USB == TRUE */

/** @} */
