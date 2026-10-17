/*
  variant of SocketAPM using native sockets (not via lwip)
 */
#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "Socket_native.h"

#if AP_SOCKET_NATIVE_ENABLED
// this TU wants the native sockets, not the lwIP ones, so the PPP
// backend is turned off for it.  Define it to 0 rather than undefining
// it: with --enable-PPP, AP_NETWORKING_ENABLED is an alias which expands
// to AP_NETWORKING_BACKEND_PPP where it is used, so leaving the name
// undefined makes every later test of it evaluate an identifier which no
// longer exists - fatal under clang's -Wundef.
#undef AP_NETWORKING_BACKEND_PPP
#define AP_NETWORKING_BACKEND_PPP 0
#define IN_SOCKET_NATIVE_CPP
#define SOCKET_CLASS_NAME SocketAPM_native
#include "Socket.cpp"
#endif

#endif
