// this file is inserted (by chibios_hwdef.py) into hwdef.h when
// configuring for bootloader builds

#define HAL_DSHOT_ALARM_ENABLED 0
#define HAL_LOGGING_ENABLED 0
#define HAL_SCHEDULER_ENABLED 0

// bootloaders *definitely* don't use the FFT library:
#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif

// bootloaders don't talk to the GCS:
#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 0
#endif

// by default bootloaders don't use INS:
#ifndef AP_INERTIALSENSOR_ENABLED
#define AP_INERTIALSENSOR_ENABLED 0
#endif

#define HAL_MAX_CAN_PROTOCOL_DRIVERS 0

// bootloader does not save temperature cals etc:
#ifndef HAL_ENABLE_SAVE_PERSISTENT_PARAMS
#define HAL_ENABLE_SAVE_PERSISTENT_PARAMS 0
#endif

// make diagnosing Faults (e.g. HardFault) harder, but save bytes:
#ifndef AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED
#define AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED 0
#endif

#ifndef AP_WATCHDOG_SAVE_FAULT_ENABLED
#define AP_WATCHDOG_SAVE_FAULT_ENABLED 0
#endif

// less LWIP functionality in the bootloader
#define LWIP_DHCP 0
#define LWIP_UDP 0
#define LWIP_PPP 0
#define LWIP_IGMP 0
#define LWIP_ALTCP 0
#define IP_FORWARD 0
#define LWIP_SINGLE_NETIF 1
#define SO_REUSE 0
#define LWIP_SOCKET_POLL 0
#define LINK_STATS 0
#define ICMP_STATS 0
#define IPFRAG_STATS 0
#define TCP_STATS 0
#define ARP_PROXYARP_SUPPORT 0
#define LWIP_HAVE_LOOPIF 0
#define LWIP_NETIF_LOOPBACK 0

/*
  we need DMA on H7 to allow for ECC error checking
  Note that ChibiOS uses #ifdef for STM32_DMA_REQUIRED not #if
 */
#if !defined(STM32_DMA_REQUIRED) && defined(STM32H7)
#define STM32_DMA_REQUIRED 1
#endif

