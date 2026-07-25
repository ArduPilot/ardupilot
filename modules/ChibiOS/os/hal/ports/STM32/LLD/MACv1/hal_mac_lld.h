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
 * @file    MACv1/hal_mac_lld.h
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
 * @brief   This implementation supports the zero-copy mode API.
 */
#define MAC_SUPPORTS_ZERO_COPY      TRUE

/**
 * @name    RDES0 constants
 * @{
 */
#define STM32_RDES0_OWN             0x80000000
#define STM32_RDES0_AFM             0x40000000
#define STM32_RDES0_FL_MASK         0x3FFF0000
#define STM32_RDES0_ES              0x00008000
#define STM32_RDES0_DESERR          0x00004000
#define STM32_RDES0_SAF             0x00002000
#define STM32_RDES0_LE              0x00001000
#define STM32_RDES0_OE              0x00000800
#define STM32_RDES0_VLAN            0x00000400
#define STM32_RDES0_FS              0x00000200
#define STM32_RDES0_LS              0x00000100
#define STM32_RDES0_IPHCE           0x00000080
#define STM32_RDES0_LCO             0x00000040
#define STM32_RDES0_FT              0x00000020
#define STM32_RDES0_RWT             0x00000010
#define STM32_RDES0_RE              0x00000008
#define STM32_RDES0_DE              0x00000004
#define STM32_RDES0_CE              0x00000002
#define STM32_RDES0_PCE             0x00000001
/** @} */

/**
 * @name    RDES1 constants
 * @{
 */
#define STM32_RDES1_DIC             0x80000000
#define STM32_RDES1_RBS2_MASK       0x1FFF0000
#define STM32_RDES1_RER             0x00008000
#define STM32_RDES1_RCH             0x00004000
#define STM32_RDES1_RBS1_MASK       0x00001FFF
/** @} */

/**
 * @name    TDES0 constants
 * @{
 */
#define STM32_TDES0_OWN             0x80000000
#define STM32_TDES0_IC              0x40000000
#define STM32_TDES0_LS              0x20000000
#define STM32_TDES0_FS              0x10000000
#define STM32_TDES0_DC              0x08000000
#define STM32_TDES0_DP              0x04000000
#define STM32_TDES0_TTSE            0x02000000
#define STM32_TDES0_LOCKED          0x01000000 /* NOTE: Pseudo flag.        */
#define STM32_TDES0_CIC_MASK        0x00C00000
#define STM32_TDES0_CIC(n)          ((n) << 22)
#define STM32_TDES0_TER             0x00200000
#define STM32_TDES0_TCH             0x00100000
#define STM32_TDES0_TTSS            0x00020000
#define STM32_TDES0_IHE             0x00010000
#define STM32_TDES0_ES              0x00008000
#define STM32_TDES0_JT              0x00004000
#define STM32_TDES0_FF              0x00002000
#define STM32_TDES0_IPE             0x00001000
#define STM32_TDES0_LCA             0x00000800
#define STM32_TDES0_NC              0x00000400
#define STM32_TDES0_LCO             0x00000200
#define STM32_TDES0_EC              0x00000100
#define STM32_TDES0_VF              0x00000080
#define STM32_TDES0_CC_MASK         0x00000078
#define STM32_TDES0_ED              0x00000004
#define STM32_TDES0_UF              0x00000002
#define STM32_TDES0_DB              0x00000001
/** @} */

/**
 * @name    TDES1 constants
 * @{
 */
#define STM32_TDES1_TBS2_MASK       0x1FFF0000
#define STM32_TDES1_TBS1_MASK       0x00001FFF
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
#define STM32_MAC_TRANSMIT_BUFFERS          2
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
#define STM32_MAC_BUFFERS_SIZE              1522
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
  volatile uint32_t             rdes0;
  volatile uint32_t             rdes1;
  volatile uint32_t             rdes2;
  volatile uint32_t             rdes3;
} stm32_eth_rx_descriptor_t;

/**
 * @brief   Type of an STM32 Ethernet transmit descriptor.
 */
typedef struct {
  volatile uint32_t             tdes0;
  volatile uint32_t             tdes1;
  volatile uint32_t             tdes2;
  volatile uint32_t             tdes3;
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
  uint32_t                      phyaddr;                                    \
  /* Receive next frame pointer.*/                                          \
  stm32_eth_rx_descriptor_t     *rxptr;                                     \
  /* Transmit next frame pointer.*/                                         \
  stm32_eth_tx_descriptor_t     *txptr;

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
#if MAC_USE_ZERO_COPY
  uint8_t *mac_lld_get_next_transmit_buffer(MACTransmitDescriptor *tdp,
                                            size_t size,
                                            size_t *sizep);
  const uint8_t *mac_lld_get_next_receive_buffer(MACReceiveDescriptor *rdp,
                                                 size_t *sizep);
#endif /* MAC_USE_ZERO_COPY */
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_MAC */

#endif /* HAL_MAC_LLD_H */

/** @} */
