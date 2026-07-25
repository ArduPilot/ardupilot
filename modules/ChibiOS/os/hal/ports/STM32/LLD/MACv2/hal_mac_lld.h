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
 * @file    MACv2/hal_mac_lld.h
 * @brief   STM32 low level MAC driver header.
 *
 * @addtogroup MAC
 * @{
 */

#ifndef HAL_MAC_LLD_H
#define HAL_MAC_LLD_H

#if HAL_USE_MAC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   This implementation does not support the zero-copy mode API.
 */
#define MAC_SUPPORTS_ZERO_COPY      FALSE

/**
 * @name    RDES1 constants
 * @{
 */
#define STM32_RDES1_OPC_MASK        0xFFFF0000
#define STM32_RDES1_TD              0x00008000
#define STM32_RDES1_TSA             0x00004000
#define STM32_RDES1_PV              0x00002000
#define STM32_RDES1_PFT             0x00001000
#define STM32_RDES1_PMT_MASK        0x00000F00
#define STM32_RDES1_IPCE            0x00000080
#define STM32_RDES1_IPCB            0x00000040
#define STM32_RDES1_IPV6            0x00000020
#define STM32_RDES1_IPV4            0x00000010
#define STM32_RDES1_IPHE            0x00000008
#define STM32_RDES1_PT_MASK         0x00000007
/** @} */

/**
 * @name    RDES2 constants
 * @{
 */
#define STM32_RDES2_L3L4FM_MASK     0xE0000000
#define STM32_RDES2_L4FM            0x01000000
#define STM32_RDES2_L3FM            0x00800000
#define STM32_RDES2_MADRM_MASK      0x007F8000
#define STM32_RDES2_HF              0x00004000
#define STM32_RDES2_DAF             0x00002000
#define STM32_RDES2_SAF             0x00001000
#define STM32_RDES2_VF              0x00000800
#define STM32_RDES2_RES1            0x00000780
#define STM32_RDES2_ARPNR           0x00000040
#define STM32_RDES2_RES2            0x0000003F
/** @} */

/**
 * @name    RDES3 constants
 * @{
 */
#define STM32_RDES3_OWN             0x80000000
#define STM32_RDES3_IOC             0x40000000 /* Read */
#define STM32_RDES3_CTXT            0x40000000 /* Write */
#define STM32_RDES3_FD              0x20000000
#define STM32_RDES3_LD              0x10000000
#define STM32_RDES3_RS2V            0x08000000
#define STM32_RDES3_RS1V            0x04000000
#define STM32_RDES3_BUF2V           0x02000000 /* Read */
#define STM32_RDES3_RS0V            0x02000000 /* Write */
#define STM32_RDES3_BUF1V           0x01000000 /* Read */
#define STM32_RDES3_CE              0x01000000 /* Write */
#define STM32_RDES3_GP              0x00800000
#define STM32_RDES3_RWT             0x00400000
#define STM32_RDES3_OE              0x00200000
#define STM32_RDES3_RE              0x00100000
#define STM32_RDES3_DE              0x00080000
#define STM32_RDES3_LT_MASK         0x00070000
#define STM32_RDES3_ES              0x00008000
#define STM32_RDES3_PL_MASK         0x00007FFF
/** @} */

/**
 * @name    TDES1 constants
 * @{
 */
#define STM32_TDES1_LOCKED          0x01000000 /* NOTE: Pseudo flag.        */
/** @} */

/**
 * @name    TDES2 constants
 * @{
 */
#define STM32_TDES2_IOC             0x80000000
#define STM32_TDES2_TTSE            0x40000000
#define STM32_TDES2_B2L_MASK        0x3FFF0000
#define STM32_TDES2_VTIR_MASK       0x0000C000
#define STM32_TDES2_VTIR(n)         ((n) << 14)
#define STM32_TDES2_B1L_MASK        0x00003FFF
/** @} */

/**
 * @name    TDES3 constants
 * @{
 */
#define STM32_TDES3_OWN             0x80000000
#define STM32_TDES3_CTXT            0x40000000
#define STM32_TDES3_FD              0x20000000
#define STM32_TDES3_LD              0x10000000
#define STM32_TDES3_CPC_MASK        0x0C000000
#define STM32_TDES3_CPC(n)          ((n) << 26)
#define STM32_TDES3_SAIC_MASK       0x03800000
#define STM32_TDES3_SAIC(n)         ((n) << 23)
#define STM32_TDES3_THL_MASK        0x00780000
#define STM32_TDES3_THL(n)          ((n) << 19)
#define STM32_TDES3_TSE             0x00040000
#define STM32_TDES3_CIC_MASK        0x00030000
#define STM32_TDES3_CIC(n)          ((n) << 16)
#define STM32_TDES3_TPL             0x00008000
#define STM32_TDES3_FL              0x00007FFF
/** @} */

/**
 * @name    Link types
 * @{
 */
#define MAC_LINK_DYNAMIC            0
#define MAC_LINK_100_FULLDUPLEX     1
#define MAC_LINK_10_FULLDUPLEX      2
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Number of available transmit buffers.
 */
#if !defined(STM32_MAC_TRANSMIT_BUFFERS) || defined(__DOXYGEN__)
#define STM32_MAC_TRANSMIT_BUFFERS          4
#endif

/**
 * @brief   Number of available receive buffers.
 */
#if !defined(STM32_MAC_RECEIVE_BUFFERS) || defined(__DOXYGEN__)
#define STM32_MAC_RECEIVE_BUFFERS           4
#endif

/**
 * @brief   Maximum supported frame size.
 */
#if !defined(STM32_MAC_BUFFERS_SIZE) || defined(__DOXYGEN__)
#define STM32_MAC_BUFFERS_SIZE              1524
#endif

/**
 * @brief   PHY detection timeout.
 * @details Timeout for PHY address detection, the scan for a PHY is performed
 *          the specified number of times before invoking the failure handler.
 *          This setting applies only if the PHY address is not explicitly
 *          set in the board header file using @p BOARD_PHY_ADDRESS. A zero
 *          value disables the timeout and a single search is performed.
 */
#if !defined(STM32_MAC_PHY_TIMEOUT) || defined(__DOXYGEN__)
#define STM32_MAC_PHY_TIMEOUT               100
#endif

/**
 * @brief   Change the PHY power state inside the driver.
 */
#if !defined(STM32_MAC_ETH1_CHANGE_PHY_STATE) || defined(__DOXYGEN__)
#define STM32_MAC_ETH1_CHANGE_PHY_STATE     TRUE
#endif

/**
 * @brief   ETHD1 interrupt priority level setting.
 */
#if !defined(STM32_MAC_ETH1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_MAC_ETH1_IRQ_PRIORITY         13
#endif

/**
 * @brief   IP checksum offload.
 * @details The following modes are available:
 *          - 0 Function disabled.
 *          - 1 Only IP header checksum calculation and insertion are enabled.
 *          - 2 IP header checksum and payload checksum calculation and
 *              insertion are enabled, but pseudo-header checksum is not
 *              calculated in hardware.
 *          - 3 IP Header checksum and payload checksum calculation and
 *              insertion are enabled, and pseudo-header checksum is
 *              calculated in hardware.
 *          .
 */
#if !defined(STM32_MAC_IP_CHECKSUM_OFFLOAD) || defined(__DOXYGEN__)
#define STM32_MAC_IP_CHECKSUM_OFFLOAD       0
#endif

/**
 * @brief   Link connection type.
 */
#if !defined(STM32_MAC_PHY_LINK_TYPE) || defined(__DOXYGEN__)
#define STM32_MAC_PHY_LINK_TYPE             MAC_LINK_DYNAMIC
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of an STM32 Ethernet receive descriptor.
 */
typedef struct {
  volatile uint32_t     rdes0;
  volatile uint32_t     rdes1;
  volatile uint32_t     rdes2;
  volatile uint32_t     rdes3;
} stm32_eth_rx_descriptor_t;

/**
 * @brief   Type of an STM32 Ethernet transmit descriptor.
 */
typedef struct {
  volatile uint32_t     tdes0;
  volatile uint32_t     tdes1;
  volatile uint32_t     tdes2;
  volatile uint32_t     tdes3;
} stm32_eth_tx_descriptor_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the MAC driver structure.
 */
#define mac_lld_driver_fields                                               \
  /* Link status flag.*/                                                    \
  bool                          link_up;                                    \
  /* PHY address (pre shifted).*/                                           \
  uint32_t                      phyaddr;

/**
 * @brief   Low level fields of the MAC configuration structure.
 */
#define mac_lld_config_fields                                               \
  /* MAC address.*/                                                         \
  const uint8_t                 *mac_address;

/**
 * @brief   Low level fields of the MAC transmit descriptor structure.
 */
#define mac_lld_transmit_descriptor_fields                                  \
  /* Pointer to the physical descriptor.*/                                  \
  stm32_eth_tx_descriptor_t     *physdesc;

/**
 * @brief   Low level fields of the MAC receive descriptor structure.
 */
#define mac_lld_receive_descriptor_fields                                   \
  /* Pointer to the physical descriptor.*/                                  \
  stm32_eth_rx_descriptor_t     *physdesc;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern MACDriver ETHD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void mii_write(MACDriver *macp, uint32_t reg, uint32_t value);
  uint32_t mii_read(MACDriver *macp, uint32_t reg);
  bool mac_lld_init(void);
  void mac_lld_start(MACDriver *macp);
  void mac_lld_stop(MACDriver *macp);
  msg_t mac_lld_get_transmit_descriptor(MACDriver *macp,
                                        MACTransmitDescriptor *tdp);
  void mac_lld_release_transmit_descriptor(MACTransmitDescriptor *tdp);
  msg_t mac_lld_get_receive_descriptor(MACDriver *macp,
                                       MACReceiveDescriptor *rdp);
  void mac_lld_release_receive_descriptor(MACReceiveDescriptor *rdp);
  bool mac_lld_poll_link_status(MACDriver *macp);
  size_t mac_lld_write_transmit_descriptor(MACTransmitDescriptor *tdp,
                                           uint8_t *buf,
                                           size_t size);
  size_t mac_lld_read_receive_descriptor(MACReceiveDescriptor *rdp,
                                         uint8_t *buf,
                                         size_t size);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_MAC */

#endif /* HAL_MAC_LLD_H */

/** @} */
