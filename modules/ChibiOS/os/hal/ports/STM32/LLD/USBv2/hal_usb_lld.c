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
 * @file    USBv2/hal_usb_lld.c
 * @brief   STM32 USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include <string.h>

#include "hal.h"

#if HAL_USE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Returns an endpoint descriptor pointer.
 */
#define USB_GET_DESCRIPTOR(ep)      (&STM32_USB_DRD_PMA_BUFF[ep])

/**
 * @brief   Gets the address of a RX buffer.
 */
#define USB_GET_RX_BUFFER(udp)      (uint32_t *)(USB_DRD_PMAADDR +       \
                                                ((udp)->RXBD0 & 0x0000FFFFU))

/**
 * @brief   Gets the address of a TX buffer.
 */
#define USB_GET_TX_BUFFER(udp)     (uint32_t *)(USB_DRD_PMAADDR +       \
                                               ((udp)->TXBD0 & 0x0000FFFFU))

/**
 * @brief   Gets the counter 0 of a RX buffer.
 */
#define USB_GET_RX_COUNT0(udp)     (size_t)(((udp)->RXBD0 >> 16) & 0x000003FFU)

/**
 * @brief   Gets the counter 0 of a RX buffer.
 */
#define USB_GET_TX_COUNT0(udp)     (size_t)(((udp)->TXBD0 >> 16) & 0x000003FFU)

/**
 * @brief   Gets the counter 1 of a RX buffer.
 */
#define USB_GET_RX_COUNT1(udp)     USB_GET_TX_COUNT0(udp)

/**
 * @brief   Gets the counter 1 of a TX buffer.
 */
#define USB_GET_TX_COUNT1(udp)     USB_GET_RX_COUNT0(udp)

/**
 * @brief   Sets the counter 0 of a RX buffer.
 */
#define USB_SET_RX_COUNT0(udp, n) do {                                      \
  (udp)->RXBD0 = (((udp)->RXBD0 & ~0x03FF0000U) | ((uint32_t)(n) << 16));   \
} while (false)

/**
 * @brief   Sets the counter 0 of a TX buffer.
 */
#define USB_SET_TX_COUNT0(udp, n) do {                                      \
  (udp)->TXBD0 = (((udp)->TXBD0 & ~0x03FF0000U) | ((uint32_t)(n) << 16));   \
} while (false)

/**
 * @brief   Sets the counter 1 of a RX buffer.
 */
#define USB_SET_RX_COUNT1(udp, n)   USB_SET_TX_COUNT0(udp, n)

/**
 * @brief   Sets the counter 1 of a TX buffer.
 */
#define USB_SET_TX_COUNT1(udp, n)   USB_SET_RX_COUNT0(udp, n)

/**
 * @brief   Mask of all the toggling bits in the CHEPR register.
 */
#define CHEPR_TOGGLE_MASK       (USB_CHEP_TX_STTX_Msk |                     \
                                 USB_CHEP_DTOG_TX_Msk |                     \
                                 USB_CHEP_RX_STRX_Msk |                     \
                                 USB_CHEP_DTOG_RX_Msk |                     \
                                 USB_CHEP_SETUP_Msk)

/**
 * @brief   Clears the VTRX bit.
 */
#define CHEPR_CLEAR_VTRX(usbp, ep)                                          \
  (usbp)->usb->CHEPR[ep] = ((usbp)->usb->CHEPR[ep] & ~USB_EP_VTRX & ~CHEPR_TOGGLE_MASK) | USB_EP_VTTX

/**
 * @brief   Clears the VTTX bit.
 */
#define CHEPR_CLEAR_VTTX(usbp, ep)                                          \
  (usbp)->usb->CHEPR[ep] = ((usbp)->usb->CHEPR[ep] & ~USB_EP_VTTX & ~CHEPR_TOGGLE_MASK) | USB_EP_VTRX

/**
 * @brief   Sets the STATRX field.
 */
#define CHEPR_SET_STATRX(usbp, ep, epr)                                     \
  (usbp)->usb->CHEPR[ep] = (((usbp)->usb->CHEPR[ep] &                       \
                            ~(CHEPR_TOGGLE_MASK & ~USB_CHEP_RX_STRX_Msk)) ^ \
                           (epr)) | USB_EP_VTTX | USB_EP_VTRX

/**
 * @brief   Sets the STATTX field.
 */
#define CHEPR_SET_STATTX(usbp, ep, epr)                                     \
  (usbp)->usb->CHEPR[ep] = (((usbp)->usb->CHEPR[ep] &                       \
                            ~(CHEPR_TOGGLE_MASK & ~USB_CHEP_TX_STTX_Msk)) ^ \
                           (epr)) | USB_EP_VTTX | USB_EP_VTRX

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USB1 driver identifier.*/
#if STM32_USB_USE_USB1 || defined(__DOXYGEN__)
USBDriver USBD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

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
 * @brief   Buffer for the EP0 setup packets.
 */
static uint8_t ep0setup_buffer[8];

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  .ep_mode          = USB_EP_MODE_TYPE_CTRL,
  .setup_cb         = _usb_ep0setup,
  .in_cb            = _usb_ep0in,
  .out_cb           = _usb_ep0out,
  .in_maxsize       = 0x40U,
  .out_maxsize      = 0x40U,
  .in_state         = &ep0_state.in,
  .out_state        = &ep0_state.out,
  .ep_buffers       = 1U,
  .setup_buf        = ep0setup_buffer
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Resets the packet memory allocator.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 */
static void usb_pm_reset(USBDriver *usbp) {

  /* The first 64 bytes are reserved for the descriptors table. The effective
     available RAM.*/
  usbp->pmnext = 64U;
}

/**
 * @brief   Resets the packet memory allocator.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] size      size of the packet buffer to allocate
 * @return              The packet buffer address.
 */
static uint32_t usb_pm_alloc(USBDriver *usbp, size_t size) {
  uint32_t next;

  next = usbp->pmnext;
  usbp->pmnext += (size + 3U) & ~3U;

  osalDbgAssert(usbp->pmnext <= STM32_USB_PMA_SIZE, "PMA overflow");

  return next;
}

/**
 * @brief   Reads from a dedicated packet buffer.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 * @return              The size of the receivee packet.
 *
 * @notapi
 */
static size_t usb_packet_read_to_buffer(USBDriver *usbp,
                                        usbep_t ep,
                                        uint8_t *buf) {
  size_t n;
  uint32_t w;
  stm32_usb_pmabufdesc_t *udp = USB_GET_DESCRIPTOR(ep);
  uint32_t *pmap = USB_GET_RX_BUFFER(udp);
  int i;

#if STM32_USB_USE_ISOCHRONOUS
  uint32_t chepr = usbp->usb->CHEPR[ep];

  /* Double buffering is always enabled for isochronous endpoints, and
     although we overlap the two buffers for simplicity, we still need
     to read from the right counter. The DTOG_RX bit indicates the buffer
     that is currently in use by the USB peripheral, that is, the buffer
     in which the next received packet will be stored, so we need to
     read the counter of the OTHER buffer, which is where the last
     received packet was stored.*/
  if (((chepr & USB_CHEP_UTYPE_Msk) == USB_EP_ISOCHRONOUS) &&
      ((chepr & USB_EP_DTOG_RX) != 0U)) {
    n = USB_GET_RX_COUNT1(udp);
  }
  else {
    n = USB_GET_RX_COUNT0(udp);
  }
#else
  (void)usbp;

  n = USB_GET_RX_COUNT0(udp);
#endif

  i = (int)n;

#if STM32_USB_USE_FAST_COPY
  while (i >= 16) {
    uint32_t w;

    w = *(pmap + 0);
    *(buf + 0) = (uint8_t)w;
    *(buf + 1) = (uint8_t)(w >> 8);
    *(buf + 2) = (uint8_t)(w >> 16);
    *(buf + 3) = (uint8_t)(w >> 24);
    w = *(pmap + 1);
    *(buf + 4) = (uint8_t)w;
    *(buf + 5) = (uint8_t)(w >> 8);
    *(buf + 6) = (uint8_t)(w >> 16);
    *(buf + 7) = (uint8_t)(w >> 24);
    w = *(pmap + 2);
    *(buf + 8) = (uint8_t)w;
    *(buf + 9) = (uint8_t)(w >> 8);
    *(buf + 10) = (uint8_t)(w >> 16);
    *(buf + 11) = (uint8_t)(w >> 24);
    w = *(pmap + 3);
    *(buf + 12) = (uint8_t)w;
    *(buf + 13) = (uint8_t)(w >> 8);
    *(buf + 14) = (uint8_t)(w >> 16);
    *(buf + 15) = (uint8_t)(w >> 24);

    i -= 16;
    buf += 16;
    pmap += 4;
  }
#endif /* STM32_USB_USE_FAST_COPY */

  while (i >= 4) {
    w = *pmap++;
    if (i < 4) {
      break;
    }
    *buf++ = (uint8_t)w;
    *buf++ = (uint8_t)(w >> 8);
    *buf++ = (uint8_t)(w >> 16);
    *buf++ = (uint8_t)(w >> 24);
    i -= 4;
  }

  if (i == 3) {
    w = *pmap;
    *(buf + 0) = (uint8_t)w;
    *(buf + 1) = (uint8_t)(w >> 8);
    *(buf + 2) = (uint8_t)(w >> 16);
  }
  else if (i == 2) {
    w = *pmap;
    *(buf + 0) = (uint8_t)w;
    *(buf + 1) = (uint8_t)(w >> 8);
  }
  else if (i == 1) {
    w = *pmap;
    *(buf + 0) = (uint8_t)w;
  }

  return n;
}

/**
 * @brief   Writes to a dedicated packet buffer.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[in] buf       buffer where to fetch the packet data
 * @param[in] n         maximum number of bytes to copy. This value must
 *                      not exceed the maximum packet size for this endpoint.
 *
 * @notapi
 */
static void usb_packet_write_from_buffer(USBDriver *usbp,
                                         usbep_t ep,
                                         const uint8_t *buf,
                                         size_t n) {
  stm32_usb_pmabufdesc_t *udp = USB_GET_DESCRIPTOR(ep);
  uint32_t *pmap = USB_GET_TX_BUFFER(udp);
  int i;

#if STM32_USB_USE_ISOCHRONOUS
  uint32_t chepr = usbp->usb->CHEPR[ep];

  /* Double buffering is always enabled for isochronous endpoints, and
     although we overlap the two buffers for simplicity, we still need
     to write to the right counter. The DTOG_TX bit indicates the buffer
     that is currently in use by the USB peripheral, that is, the buffer
     from which the next packet will be sent, so we need to write the
     counter of that buffer.*/
  if (((chepr & USB_CHEP_UTYPE_Msk) == USB_EP_ISOCHRONOUS) &&
      ((chepr & USB_EP_DTOG_TX) != 0U)) {
    USB_SET_TX_COUNT1(udp, n);
  }
  else {
    USB_SET_TX_COUNT0(udp, n);
  }
#else
  (void)usbp;

  USB_SET_TX_COUNT0(udp, n);
#endif

  i = (int)n;

#if STM32_USB_USE_FAST_COPY
  while (i >= 16) {
    uint32_t w;

    w  = (uint32_t)*(buf + 0);
    w |= (uint32_t)*(buf + 1) << 8;
    w |= (uint32_t)*(buf + 2) << 16;
    w |= (uint32_t)*(buf + 3) << 24;
    *(pmap + 0) = w;
    w  = (uint32_t)*(buf + 4);
    w |= (uint32_t)*(buf + 5) << 8;
    w |= (uint32_t)*(buf + 6) << 16;
    w |= (uint32_t)*(buf + 7) << 24;
    *(pmap + 1) = w;
    w  = (uint32_t)*(buf + 8);
    w |= (uint32_t)*(buf + 9) << 8;
    w |= (uint32_t)*(buf + 10) << 16;
    w |= (uint32_t)*(buf + 11) << 24;
    *(pmap + 2) = w;
    w  = (uint32_t)*(buf + 12);
    w |= (uint32_t)*(buf + 13) << 8;
    w |= (uint32_t)*(buf + 14) << 16;
    w |= (uint32_t)*(buf + 15) << 24;
    *(pmap + 3) = w;

    i -= 16;
    buf += 16;
    pmap += 4;
  }
#endif /* STM32_USB_USE_FAST_COPY */

  while (i >= 4) {
    uint32_t w;

    w  = (uint32_t)(*buf++);
    w |= (uint32_t)(*buf++) << 8;
    w |= (uint32_t)(*buf++) << 16;
    w |= (uint32_t)(*buf++) << 24;
    *pmap++ = w;
    i -= 4U;
  }

  if (i != 0) {
    uint32_t w = 0U;

    switch (i) {
    case 3:
      w |= (uint32_t)buf[2] << 16;
      /* Falls through.*/
    case 2:
      w |= (uint32_t)buf[1] << 8;
      /* Falls through.*/
    case 1:
      w |= (uint32_t)buf[0];
      break;
    default:
      break;
    }
    *pmap++ = w;
  }
}

/**
 * @brief   Common ISR code, serves the EP-related interrupts.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] istr      ISTR register value to consider
 *
 * @notapi
 */
static void usb_serve_endpoints(USBDriver *usbp, uint32_t istr) {
  size_t n;
  uint32_t ep = istr & USB_ISTR_IDN_Msk;
  uint32_t chepr = usbp->usb->CHEPR[ep];
  const USBEndpointConfig *epcp = usbp->epc[ep];

  if ((istr & USB_ISTR_DIR) == 0U) {
    /* IN endpoint, transmission.*/
    USBInEndpointState *isp = epcp->in_state;

    CHEPR_CLEAR_VTTX(usbp, ep);

    isp->txcnt += isp->txlast;
    n = isp->txsize - isp->txcnt;
    if (n > 0U) {
      /* Transfer not completed, there are more packets to send.*/
      if (n > epcp->in_maxsize)
        n = epcp->in_maxsize;

      /* Writes the packet from the defined buffer.*/
      isp->txbuf += isp->txlast;
      isp->txlast = n;
      usb_packet_write_from_buffer(usbp, ep, isp->txbuf, n);

      /* Starting IN operation.*/
      CHEPR_SET_STATTX(usbp, ep, USB_EP_TX_VALID);
    }
    else {
      /* Transfer completed, invokes the callback.*/
      _usb_isr_invoke_in_cb(usbp, ep);
    }
  }
  else {
    /* OUT endpoint, receive.*/

    CHEPR_CLEAR_VTRX(usbp, ep);

    if (chepr & USB_EP_SETUP) {
      /* Setup packets handling, setup packets are handled using a
         specific callback.*/
      _usb_isr_invoke_setup_cb(usbp, ep);
    }
    else {
      USBOutEndpointState *osp = epcp->out_state;

      /* Reads the packet into the defined buffer.*/
      n = usb_packet_read_to_buffer(usbp, ep, osp->rxbuf);
      osp->rxbuf += n;

      /* Transaction data updated.*/
      osp->rxcnt  += n;
      osp->rxsize -= n;
      osp->rxpkts -= 1U;

      /* The transaction is completed if the specified number of packets
         has been received or the current packet is a short packet.*/
      if ((n < epcp->out_maxsize) || (osp->rxpkts == 0)) {
        /* Transfer complete, invokes the callback.*/
        _usb_isr_invoke_out_cb(usbp, ep);
      }
      else {
        /* Transfer not complete, there are more packets to receive.*/
        CHEPR_SET_STATRX(usbp, ep, USB_EP_RX_STRX);
      }
    }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {

  /* Driver initialization.*/
  usbObjectInit(&USBD1);

  USBD1.usb = STM32_USB;
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The operation status.
 *
 * @notapi
 */
msg_t usb_lld_start(USBDriver *usbp) {

  if (usbp->state == USB_STOP) {
    /* Clock activation.*/
#if STM32_USB_USE_USB1
    if (&USBD1 == usbp) {

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT)
      if ((STM32_USBCLK < (48000000U - STM32_USB_48MHZ_DELTA)) ||
          (STM32_USBCLK > (48000000U + STM32_USB_48MHZ_DELTA))) {
        return HAL_RET_CONFIG_ERROR;
      }
#endif

      /* USB clock enabled.*/
      rccEnableUSB(true);
      rccResetUSB();

      /* Powers up the transceiver while holding the USB in reset state.*/
      usbp->usb->CNTR = USB_CNTR_USBRST;

      /* Releases the USB reset.*/
      usbp->usb->CNTR = 0U;
    }
#endif
    /* Reset procedure enforced on driver start.*/
    usb_lld_reset(usbp);
  }

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {

  /* If in ready state then disables the USB clock.*/
  if (usbp->state != USB_STOP) {
#if STM32_USB_USE_USB1
    if (&USBD1 == usbp) {

      usbp->usb->CNTR = USB_CNTR_PDWN | USB_CNTR_L2RES;
      rccDisableUSB();
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
  uint32_t cntr;

  /* Post reset initialization.*/
  usbp->usb->ISTR   = 0U;
  usbp->usb->DADDR  = USB_DADDR_EF;
  cntr              = /* USB_CNTR_ESOFM | */ USB_CNTR_RESETM  | USB_CNTR_SUSPM |
                      USB_CNTR_WKUPM | USB_CNTR_ERRM |/* USB_CNTR_PMAOVRM |*/ USB_CNTR_CTRM;
  /* The SOF interrupt is only enabled if a callback is defined for
     this service because it is an high rate source.*/
  if (usbp->config->sof_cb != NULL)
    cntr |= USB_CNTR_SOFM;
  usbp->usb->CNTR = cntr;

  /* Resets the packet memory allocator.*/
  usb_pm_reset(usbp);

  /* EP0 initialization.*/
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0U);
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {

  usbp->usb->DADDR = (uint32_t)(usbp->address) | USB_DADDR_EF;
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
  uint32_t chepr;
  stm32_usb_pmabufdesc_t *dp;
  const USBEndpointConfig *epcp = usbp->epc[ep];

  /* Setting the endpoint type. Note that isochronous endpoints cannot be
     bidirectional because it uses double buffering and both transmit and
     receive descriptor fields are used for either direction.*/
  switch (epcp->ep_mode & USB_EP_MODE_TYPE) {
  case USB_EP_MODE_TYPE_ISOC:
#if STM32_USB_USE_ISOCHRONOUS
    osalDbgAssert((epcp->in_state == NULL) || (epcp->out_state == NULL),
                  "isochronous EP cannot be IN and OUT");
    chepr = USB_EP_ISOCHRONOUS;
    break;
#else
    osalDbgAssert(false, "isochronous support disabled");
#endif
    /* Falls through.*/
  case USB_EP_MODE_TYPE_BULK:
    chepr = USB_EP_BULK;
    break;
  case USB_EP_MODE_TYPE_INTR:
    chepr = USB_EP_INTERRUPT;
    break;
  default:
    chepr = USB_EP_CONTROL;
  }

  dp = USB_GET_DESCRIPTOR(ep);

  /* IN endpoint handling.*/
  if (epcp->in_state != NULL) {
    dp->TXBD0 = usb_pm_alloc(usbp, epcp->in_maxsize);

#if STM32_USB_USE_ISOCHRONOUS
    if (chepr == USB_EP_ISOCHRONOUS) {
      chepr |= USB_EP_TX_VALID;
      dp->TXBD1 = dp->TXBD0;   /* Both buffers overlapped.*/
    }
    else {
      chepr |= USB_EP_TX_NAK;
    }
#else
    chepr |= USB_EP_TX_NAK;
#endif
  }

  /* OUT endpoint handling.*/
  if (epcp->out_state != NULL) {
    uint32_t nblocks;

    /* Endpoint size and address initialization.*/
    if (epcp->out_maxsize > 62U) {
      nblocks = (((((uint32_t)epcp->out_maxsize - 1U) | 0x1FU) / 32U) << 26) |
                0x80000000U;
    }
    else {
      nblocks = (((((uint32_t)epcp->out_maxsize - 1U) | 1U) + 1U) / 2U) << 26;
    }
    dp->RXBD0 = nblocks | usb_pm_alloc(usbp, epcp->out_maxsize);

#if STM32_USB_USE_ISOCHRONOUS
    if (chepr == USB_EP_ISOCHRONOUS) {
      chepr |= USB_EP_RX_VALID;
      dp->RXBD1 = dp->RXBD0;   /* Both buffers overlapped.*/
    }
    else {
      chepr |= USB_EP_RX_NAK;
    }
#else
    chepr |= USB_EP_RX_NAK;
#endif
  }

  /* Resetting the data toggling bits for this endpoint.*/
  if (usbp->usb->CHEPR[ep] & USB_EP_DTOG_RX) {
    chepr |= USB_EP_DTOG_RX;
  }

  if (usbp->usb->CHEPR[ep] & USB_EP_DTOG_TX) {
    chepr |= USB_EP_DTOG_TX;
  }

  /* CHEPxR register cleared and initialized.*/
  usbp->usb->CHEPR[ep] = usbp->usb->CHEPR[ep];
  usbp->usb->CHEPR[ep] = chepr | ep;
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {
  unsigned i;

  /* Resets the packet memory allocator.*/
  usb_pm_reset(usbp);

  /* Disabling all endpoints.*/
  for (i = 1U; i <= (unsigned)USB_ENDPOINTS_NUMBER; i++) {

    /* Clearing all toggle bits then zeroing the rest.*/
    usbp->usb->CHEPR[i] = usbp->usb->CHEPR[i];
    usbp->usb->CHEPR[i] = 0U;
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

  (void)usbp;
  switch (usbp->usb->CHEPR[ep] & USB_CHEP_RX_STRX_Msk) {
  case USB_EP_RX_DIS:
    return EP_STATUS_DISABLED;
  case USB_EP_RX_STALL:
    return EP_STATUS_STALLED;
  default:
    return EP_STATUS_ACTIVE;
  }
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

  (void)usbp;
  switch (usbp->usb->CHEPR[ep] & USB_CHEP_TX_STTX_Msk) {
  case USB_EP_TX_DIS:
    return EP_STATUS_DISABLED;
  case USB_EP_TX_STALL:
    return EP_STATUS_STALLED;
  default:
    return EP_STATUS_ACTIVE;
  }
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
  uint32_t *pmap;
  stm32_usb_pmabufdesc_t *udp;

  (void)usbp;

  udp = USB_GET_DESCRIPTOR(ep);
  pmap = USB_GET_RX_BUFFER(udp);
  *(uint32_t *)(void *)(buf + 0) = *pmap++;
  *(uint32_t *)(void *)(buf + 4) = *pmap++;
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

  /* Transfer initialization.*/
  if (osp->rxsize == 0U)        /* Special case for zero sized packets.*/
    osp->rxpkts = 1U;
  else
    osp->rxpkts = (uint16_t)((osp->rxsize + usbp->epc[ep]->out_maxsize - 1U)/
                             usbp->epc[ep]->out_maxsize);

  CHEPR_SET_STATRX(usbp, ep, USB_EP_RX_VALID);
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
  size_t n;
  USBInEndpointState *isp = usbp->epc[ep]->in_state;

  /* Transfer initialization.*/
  n = isp->txsize;
  if (n > (size_t)usbp->epc[ep]->in_maxsize)
    n = (size_t)usbp->epc[ep]->in_maxsize;

  isp->txlast = n;
  usb_packet_write_from_buffer(usbp, ep, isp->txbuf, n);

  CHEPR_SET_STATTX(usbp, ep, USB_EP_TX_VALID);
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

  (void)usbp;

  CHEPR_SET_STATRX(usbp, ep, USB_EP_RX_STALL);
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

  (void)usbp;

  CHEPR_SET_STATTX(usbp, ep, USB_EP_TX_STALL);
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

  (void)usbp;

  /* Makes sure to not put to NAK an endpoint that is already
     transferring.*/
  if ((usbp->usb->CHEPR[ep] & USB_CHEP_RX_STRX_Msk) != USB_EP_RX_VALID) {
    CHEPR_SET_STATRX(usbp, ep, USB_EP_RX_NAK);
  }
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

  (void)usbp;

  /* Makes sure to not put to NAK an endpoint that is already
     transferring.*/
  if ((usbp->usb->CHEPR[ep] & USB_CHEP_TX_STTX_Msk) != USB_EP_TX_VALID) {
    CHEPR_SET_STATTX(usbp, ep, USB_EP_TX_NAK);
  }
}

/**
 * @brief   Shared USB service routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_serve_interrupt(USBDriver *usbp) {
  uint32_t istr;

  /* Reading interrupt sources and atomically clearing them.*/
  istr = usbp->usb->ISTR;
  usbp->usb->ISTR = ~istr;

  /* USB bus reset condition handling.*/
  if (istr & USB_ISTR_RESET) {
    _usb_reset(usbp);
  }

  /* USB bus SUSPEND condition handling.*/
  if ((istr & USB_ISTR_SUSP) != 0U) {
    usbp->usb->CNTR |= USB_CNTR_SUSPEN;
    _usb_suspend(usbp);
  }

  /* USB bus WAKEUP condition handling.*/
  if ((istr & USB_ISTR_WKUP) != 0U) {
    uint32_t fnr = usbp->usb->FNR;
    if ((fnr & USB_FNR_RXDP) == 0U) {
      _usb_wakeup(usbp);
    }
  }

  /* SOF handling.*/
  if ((istr & USB_ISTR_SOF) != 0U) {
    _usb_isr_invoke_sof_cb(usbp);
  }

  /* ERR handling.*/
  if ((istr & USB_ISTR_ERR) != 0U) {
    /* CHTODO */
  }

  /* Endpoint events handling.*/
  while ((istr & USB_ISTR_CTR) != 0U) {
    usb_serve_endpoints(usbp, istr);
    istr = usbp->usb->ISTR;
  }
}

#endif /* HAL_USE_USB */

/** @} */
