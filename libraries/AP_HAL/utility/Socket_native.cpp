/*
  variant of SocketAPM using native sockets (not via lwip)
 */
#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "Socket_native.h"

#if AP_SOCKET_NATIVE_ENABLED
#undef AP_NETWORKING_BACKEND_PPP
#define IN_SOCKET_NATIVE_CPP
#define SOCKET_CLASS_NAME SocketAPM_native
#include "Socket.cpp"
#endif

#endif
