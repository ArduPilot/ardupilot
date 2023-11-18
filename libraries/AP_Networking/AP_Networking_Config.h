#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_NETWORKING_ENABLED
#define AP_NETWORKING_ENABLED 0
#endif

#ifndef AP_NETWORKING_BACKEND_DEFAULT_ENABLED
#define AP_NETWORKING_BACKEND_DEFAULT_ENABLED AP_NETWORKING_ENABLED
#endif


// ---------------------------
// Backends
// ---------------------------
#ifndef AP_NETWORKING_BACKEND_CHIBIOS
#define AP_NETWORKING_BACKEND_CHIBIOS AP_NETWORKING_BACKEND_DEFAULT_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#endif

#ifndef AP_NETWORKING_BACKEND_SITL
#define AP_NETWORKING_BACKEND_SITL AP_NETWORKING_BACKEND_DEFAULT_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL
#endif

#define AP_NETWORKING_SOCKETS_ENABLED (HAL_OS_SOCKETS || AP_NETWORKING_BACKEND_CHIBIOS)

// ---------------------------
// IP Features
// ---------------------------
#if AP_NETWORKING_BACKEND_CHIBIOS
#define AP_NETWORKING_DHCP_AVAILABLE    LWIP_DHCP
#else
#define AP_NETWORKING_DHCP_AVAILABLE    1 // for non-ChibiOS, assume it's available
#endif


// ---------------------------
// Below are default params
// ---------------------------

// Default DHCP
#ifndef AP_NETWORKING_DEFAULT_DHCP_ENABLE
#define AP_NETWORKING_DEFAULT_DHCP_ENABLE 1
#endif

// Default Static IP Address: 192.168.13.14
#ifndef AP_NETWORKING_DEFAULT_STATIC_IP_ADDR
#define AP_NETWORKING_DEFAULT_STATIC_IP_ADDR "192.168.13.14"
#endif

// Default Netmask: 24
// Note, the netmask is the number of consecutive bits
#ifndef AP_NETWORKING_DEFAULT_NETMASK
#define AP_NETWORKING_DEFAULT_NETMASK       24 // 255.255.255.0 (for 10.0.xxx.xxx or 172.xxx.xxx.xxx type networks)
// #define AP_NETWORKING_DEFAULT_NETMASK    16 // 255.255.0.0 (for 192.168.xxx.xxxx type networks)
#endif


// Default Static IP Address: 192.168.13.1
#ifndef AP_NETWORKING_DEFAULT_STATIC_GW_ADDR
#define AP_NETWORKING_DEFAULT_STATIC_GW_ADDR "192.168.13.1"
#endif

// Default MAC Address: C2:AF:51:03:CF:46
// Note, lower 3 bytes (ADDR3,4,5) will be replaced with the platform UUID
#ifndef AP_NETWORKING_DEFAULT_MAC_ADDR
#define AP_NETWORKING_DEFAULT_MAC_ADDR "C2:AF:51:03:CF:46"
#endif

#ifndef AP_NETWORKING_TESTS_ENABLED
#define AP_NETWORKING_TESTS_ENABLED 0
#endif

#if AP_NETWORKING_TESTS_ENABLED
#ifndef AP_NETWORKING_TEST_IP
#define AP_NETWORKING_TEST_IP "192.168.13.2"
#endif
#endif

#ifndef AP_NETWORKING_NUM_PORTS
#define AP_NETWORKING_NUM_PORTS 4
#endif
