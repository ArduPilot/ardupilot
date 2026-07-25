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
 * @file    USBv1/stm32_usb.h
 * @brief   STM32 USB registers layout header.
 * @note    This file requires definitions from the ST STM32 header files
 *          stm32f10x.h or stm32l1xx.h.
 *
 * @addtogroup USB
 * @{
 */

#ifndef STM32_USB_H
#define STM32_USB_H

/**
 * @brief   Number of the available endpoints.
 * @details This value does not include the endpoint 0 which is always present.
 */
#define USB_ENDPOINTS_NUMBER            7

/**
 * @brief   Width of USB packet memory accesses.
 */
#if STM32_USB_ACCESS_SCHEME_2x16
typedef uint16_t stm32_usb_pma_t;
#else
typedef uint32_t stm32_usb_pma_t;
#endif

/**
 * @brief   USB registers block.
 */
typedef struct {
  /**
   * @brief   Endpoint registers.
   */
  volatile uint32_t             EPR[USB_ENDPOINTS_NUMBER + 1];
  /*
   * @brief   Reserved space.
   */
  volatile uint32_t             _r20[8];
  /*
   * @brief   Control Register.
   */
  volatile uint32_t             CNTR;
  /*
   * @brief   Interrupt Status Register.
   */
  volatile uint32_t             ISTR;
  /*
   * @brief   Frame Number Register.
   */
  volatile uint32_t             FNR;
  /*
   * @brief   Device Address Register.
   */
  volatile uint32_t             DADDR;
  /*
   * @brief   Buffer Table Address.
   */
  volatile uint32_t             BTABLE;
  /*
   * @brief   LPM Control and Status Register.
   */
  volatile uint32_t             LPMCSR;
#if STM32_USB_HAS_BCDR
  /*
   * @brief   Battery Charging Detector
   */
  volatile uint32_t             BCDR;
#endif
} stm32_usb_t;

/**
 * @brief   USB descriptor registers block.
 */
typedef struct {
  /**
   * @brief   TX buffer offset register.
   */
  volatile stm32_usb_pma_t      TXADDR0;
  /**
   * @brief   TX counter register 0.
   */
  volatile stm32_usb_pma_t      TXCOUNT0;
  /**
   * @brief   RX buffer offset register.
   */
  volatile stm32_usb_pma_t      RXADDR0;
  /**
   * @brief   RX counter register 0.
   */
  volatile stm32_usb_pma_t      RXCOUNT0;
} stm32_usb_descriptor_t;

/**
 * @name    Register aliases
 * @{
 */
#define RXCOUNT1                TXCOUNT0
#define TXCOUNT1                RXCOUNT0
#define RXADDR1                 TXADDR0
#define TXADDR1                 RXADDR0
/** @} */

/**
 * @brief USB registers block numeric address.
 */
#if defined(USB1_BASE) || defined(__DOXYGEN__)
  #define STM32_USB_BASE        USB1_BASE
#elif defined(USB_BASE)
  #define STM32_USB_BASE        USB_BASE
#else
  #define STM32_USB_BASE        (APB1PERIPH_BASE + 0x5C00)
#endif

/**
 * @brief USB RAM numeric address.
 */
#if defined(USB1_PMAADDR) || defined(__DOXYGEN__)
  #define STM32_USBRAM_BASE     USB1_PMAADDR
#elif defined(USB_PMAADDR)
  #define STM32_USBRAM_BASE     USB_PMAADDR
#elif defined(USB_DRD_PMAADDR)
#define STM32_USBRAM_BASE       USB_DRD_PMAADDR
#else
  #define STM32_USBRAM_BASE     (APB1PERIPH_BASE + 0x6000)
#endif

/**
 * @brief Pointer to the USB registers block.
 */
#define STM32_USB               ((stm32_usb_t *)STM32_USB_BASE)

/**
 * @brief   Pointer to the USB RAM.
 */
#define STM32_USBRAM            ((stm32_usb_pma_t *)STM32_USBRAM_BASE)

/**
 * @brief   Mask of all the toggling bits in the EPR register.
 */
#define EPR_TOGGLE_MASK         (EPR_STAT_TX_MASK | EPR_DTOG_TX |           \
                                 EPR_STAT_RX_MASK | EPR_DTOG_RX |           \
                                 EPR_SETUP)

#define EPR_EA_MASK             0x000F
#define EPR_STAT_TX_MASK        0x0030
#define EPR_STAT_TX_DIS         0x0000
#define EPR_STAT_TX_STALL       0x0010
#define EPR_STAT_TX_NAK         0x0020
#define EPR_STAT_TX_VALID       0x0030
#define EPR_DTOG_TX             0x0040
#define EPR_SWBUF_RX            EPR_DTOG_TX
#define EPR_CTR_TX              0x0080
#define EPR_EP_KIND             0x0100
#define EPR_EP_DBL_BUF          EPR_EP_KIND
#define EPR_EP_STATUS_OUT       EPR_EP_KIND
#define EPR_EP_TYPE_MASK        0x0600
#define EPR_EP_TYPE_BULK        0x0000
#define EPR_EP_TYPE_CONTROL     0x0200
#define EPR_EP_TYPE_ISO         0x0400
#define EPR_EP_TYPE_INTERRUPT   0x0600
#define EPR_SETUP               0x0800
#define EPR_STAT_RX_MASK        0x3000
#define EPR_STAT_RX_DIS         0x0000
#define EPR_STAT_RX_STALL       0x1000
#define EPR_STAT_RX_NAK         0x2000
#define EPR_STAT_RX_VALID       0x3000
#define EPR_DTOG_RX             0x4000
#define EPR_SWBUF_TX            EPR_DTOG_RX
#define EPR_CTR_RX              0x8000

#define CNTR_FRES               0x0001
#define CNTR_PDWN               0x0002
#define CNTR_LP_MODE            0x0004
#define CNTR_FSUSP              0x0008
#define CNTR_RESUME             0x0010
#define CNTR_ESOFM              0x0100
#define CNTR_SOFM               0x0200
#define CNTR_RESETM             0x0400
#define CNTR_SUSPM              0x0800
#define CNTR_WKUPM              0x1000
#define CNTR_ERRM               0x2000
#define CNTR_PMAOVRM            0x4000
#define CNTR_CTRM               0x8000

#define ISTR_EP_ID_MASK         0x000F
#define ISTR_DIR                0x0010
#define ISTR_ESOF               0x0100
#define ISTR_SOF                0x0200
#define ISTR_RESET              0x0400
#define ISTR_SUSP               0x0800
#define ISTR_WKUP               0x1000
#define ISTR_ERR                0x2000
#define ISTR_PMAOVR             0x4000
#define ISTR_CTR                0x8000

#define FNR_FN_MASK             0x07FF
#define FNR_LSOF                0x1800
#define FNR_LCK                 0x2000
#define FNR_RXDM                0x4000
#define FNR_RXDP                0x8000

#define DADDR_ADD_MASK          0x007F
#define DADDR_EF                0x0080

#define RXCOUNT_COUNT_MASK      0x03FF
#define TXCOUNT_COUNT_MASK      0x03FF

#define EPR_CTR_MASK            (EPR_CTR_TX | EPR_CTR_RX)

#define EPR_SET_STAT_RX(ep, epr)                                            \
  STM32_USB->EPR[ep] = ((STM32_USB->EPR[ep] &                               \
                        ~(EPR_TOGGLE_MASK & ~EPR_STAT_RX_MASK)) ^           \
                       (epr)) | EPR_CTR_MASK

#define EPR_SET_STAT_TX(ep, epr)                                            \
  STM32_USB->EPR[ep] = ((STM32_USB->EPR[ep] &                               \
                        ~(EPR_TOGGLE_MASK & ~EPR_STAT_TX_MASK)) ^           \
                       (epr)) | EPR_CTR_MASK

#define EPR_CLEAR_CTR_RX(ep)                                                \
  STM32_USB->EPR[ep] = (STM32_USB->EPR[ep] & ~EPR_CTR_RX & ~EPR_TOGGLE_MASK)\
                       | EPR_CTR_TX

#define EPR_CLEAR_CTR_TX(ep)                                                \
  STM32_USB->EPR[ep] = (STM32_USB->EPR[ep] & ~EPR_CTR_TX & ~EPR_TOGGLE_MASK)\
                       | EPR_CTR_RX

/**
 * @brief   Returns an endpoint descriptor pointer.
 */
#define USB_GET_DESCRIPTOR(ep)                                              \
  ((stm32_usb_descriptor_t *)((uint32_t)STM32_USBRAM_BASE +                 \
                              (uint32_t)STM32_USB->BTABLE +                 \
                              (uint32_t)(ep) *                              \
                              sizeof(stm32_usb_descriptor_t)))

/**
 * @brief   Converts from a PMA address to a physical address.
 */
#define USB_ADDR2PTR(addr)                                                  \
  ((stm32_usb_pma_t *)((addr) *                                             \
                       (sizeof(stm32_usb_pma_t) / 2) +                      \
                       STM32_USBRAM_BASE))

#endif /* STM32_USB_H */

/** @} */
