#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_SerialManager/AP_SerialManager_config.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

#if defined(AP_NETWORKING_BACKEND_PPP) && !defined(AP_NETWORKING_ENABLED)
// allow --enable-PPP to enable networking
#define AP_NETWORKING_ENABLED AP_NETWORKING_BACKEND_PPP
#endif


#ifndef AP_NETWORKING_ENABLED
#if !defined(__APPLE__) && defined(__clang__)
// clang fails on linux
#define AP_NETWORKING_ENABLED 0
#else
#define AP_NETWORKING_ENABLED ((CONFIG_HAL_BOARD == HAL_BOARD_LINUX) || (CONFIG_HAL_BOARD == HAL_BOARD_SITL))
#endif
#endif

#ifndef AP_NETWORKING_BACKEND_DEFAULT_ENABLED
#define AP_NETWORKING_BACKEND_DEFAULT_ENABLED AP_NETWORKING_ENABLED
#endif

// ---------------------------
// Backends
// ---------------------------
#ifndef AP_NETWORKING_BACKEND_CHIBIOS
#ifndef HAL_USE_MAC
#define HAL_USE_MAC 0
#endif
#define AP_NETWORKING_BACKEND_CHIBIOS (AP_NETWORKING_BACKEND_DEFAULT_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS) && HAL_USE_MAC)
#endif

#if !defined(AP_NETWORKING_BACKEND_PPP)
#define AP_NETWORKING_BACKEND_PPP (AP_NETWORKING_BACKEND_DEFAULT_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS) && !HAL_USE_MAC)
#endif

#ifndef AP_NETWORKING_BACKEND_SITL
#define AP_NETWORKING_BACKEND_SITL (AP_NETWORKING_BACKEND_DEFAULT_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL))
#endif

#ifndef AP_NETWORKING_SOCKETS_ENABLED
#define AP_NETWORKING_SOCKETS_ENABLED AP_NETWORKING_ENABLED
#endif

#ifndef AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED
// AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED should only be true if we have the ability to
// change the IP address. If not then the IP, GW, NetMask, MAC and DHCP params are hidden. 
// This does not mean that the system/OS does not have the ability to set the IP, just that
// we have no control from this scope. For example, Linux systems (including SITL) have
// their own DHCP client running but we have no control over it.
#define AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED AP_NETWORKING_BACKEND_CHIBIOS
#endif

#define AP_NETWORKING_NEED_LWIP (AP_NETWORKING_BACKEND_CHIBIOS || AP_NETWORKING_BACKEND_PPP)

// ---------------------------
// IP Features
// ---------------------------
#ifndef AP_NETWORKING_DHCP_AVAILABLE
// AP_NETWORKING_DHCP_AVAILABLE should only be true if, by setting the NET_DHCP parameter,
// we have the ability to turn on/off the DHCP client which effects the assigned IP address.
// Otherwise, param NET_DHCP will be hidden. This does not mean that the system/OS does not
// have DHCP, just that we have no control from this scope. For example, Linux systems
// (including SITL) have their own DHCP client running but we have no control over it.
#define AP_NETWORKING_DHCP_AVAILABLE (AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED || AP_NETWORKING_BACKEND_CHIBIOS)
#endif


// ---------------------------
// Below are default params
// ---------------------------

// Default DHCP
#ifndef AP_NETWORKING_DEFAULT_DHCP_ENABLE
#define AP_NETWORKING_DEFAULT_DHCP_ENABLE AP_NETWORKING_DHCP_AVAILABLE
#endif

// Default Static IP Address: 192.168.144.14
#ifndef AP_NETWORKING_DEFAULT_STATIC_IP_ADDR
#define AP_NETWORKING_DEFAULT_STATIC_IP_ADDR "192.168.144.14"
#endif

// Default Netmask: 24
// Note, the netmask is the number of consecutive bits
#ifndef AP_NETWORKING_DEFAULT_NETMASK
#define AP_NETWORKING_DEFAULT_NETMASK       24 // 255.255.255.0 (for 10.0.xxx.xxx or 172.xxx.xxx.xxx type networks)
// #define AP_NETWORKING_DEFAULT_NETMASK    16 // 255.255.0.0 (for 192.168.xxx.xxxx type networks)
#endif


// Default Static IP Address: 192.168.144.1
#ifndef AP_NETWORKING_DEFAULT_STATIC_GW_ADDR
#define AP_NETWORKING_DEFAULT_STATIC_GW_ADDR "192.168.144.1"
#endif

// Default MAC Address: C2:AF:51:03:CF:46
// Note, lower 3 bytes (ADDR3,4,5) will be replaced with the platform UUID
#ifndef AP_NETWORKING_DEFAULT_MAC_ADDR
#define AP_NETWORKING_DEFAULT_MAC_ADDR "C2:AF:51:03:CF:46"
#endif

#ifndef AP_NETWORKING_TESTS_ENABLED
#define AP_NETWORKING_TESTS_ENABLED 0
#endif

#ifndef AP_NETWORKING_CAN_MCAST_ENABLED
#define AP_NETWORKING_CAN_MCAST_ENABLED 0
#endif

#ifndef AP_NETWORKING_CAN_MCAST_BRIDGING_ENABLED
#define AP_NETWORKING_CAN_MCAST_BRIDGING_ENABLED AP_NETWORKING_CAN_MCAST_ENABLED
#endif

#if AP_NETWORKING_TESTS_ENABLED
#ifndef AP_NETWORKING_TEST_IP
#define AP_NETWORKING_TEST_IP "192.168.144.2"
#endif
#endif

#ifndef AP_NETWORKING_NUM_PORTS
#define AP_NETWORKING_NUM_PORTS 4
#endif

#ifndef AP_NETWORKING_NUM_SENDFILES
#define AP_NETWORKING_NUM_SENDFILES 20
#endif

#ifndef AP_NETWORKING_SENDFILE_BUFSIZE
#define AP_NETWORKING_SENDFILE_BUFSIZE (64*512)
#endif

#ifndef AP_NETWORKING_PPP_GATEWAY_ENABLED
#define AP_NETWORKING_PPP_GATEWAY_ENABLED (AP_NETWORKING_BACKEND_PPP && (CONFIG_HAL_BOARD != HAL_BOARD_CHIBIOS || AP_NETWORKING_BACKEND_CHIBIOS != 0))
#endif

/*
  the IP address given to the remote end of the PPP link when running
  as a PPP<->ethernet gateway. If this is on the same subnet as the
  ethernet interface IP then proxyarp will be used
 */
#ifndef AP_NETWORKING_REMOTE_PPP_IP
#define AP_NETWORKING_REMOTE_PPP_IP "0.0.0.0"
#endif

#ifndef AP_NETWORKING_REGISTER_PORT_ENABLED
#define AP_NETWORKING_REGISTER_PORT_ENABLED AP_NETWORKING_ENABLED && AP_SERIALMANAGER_REGISTER_ENABLED
#endif

// SITL only network capture, use custom build server to enable on real boards
#ifndef AP_NETWORKING_CAPTURE_ENABLED
#define AP_NETWORKING_CAPTURE_ENABLED (AP_NETWORKING_NEED_LWIP && AP_FILESYSTEM_FILE_WRITING_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif
