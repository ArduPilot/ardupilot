#pragma once

#include "AP_Networking_Config.h"

#if (AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET || defined(AP_BOOTLOADER_NETWORK_ENABLED))

/*
  Shared MAC buffer allocation for ChibiOS Ethernet.
  Used by both the SwitchPort Ethernet backend and the bootloader.
*/

#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <hal.h>

#ifndef STM32_ETH_BUFFERS_EXTERN
#error "Must use external ethernet buffers"
#endif

/*
  MAC DMA buffers - referenced as globals by ChibiOS MAC driver.
  These are defined in the .cpp file and referenced by hal_mac_lld.c
*/
extern stm32_eth_rx_descriptor_t *__eth_rd;
extern stm32_eth_tx_descriptor_t *__eth_td;
extern uint32_t *__eth_rb[STM32_MAC_RECEIVE_BUFFERS];
extern uint32_t *__eth_tb[STM32_MAC_TRANSMIT_BUFFERS];

namespace AP_Networking_MAC_Buffers {
    /*
      Allocate buffers for MAC DMA.
      Sets up MPU region and initializes the global buffer pointers.
      Returns true on success, false on failure.
    */
    bool allocate_buffers();
}

#endif // AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET || AP_BOOTLOADER_NETWORK_ENABLED
