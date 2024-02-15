/*
  variant of SocketAPM using either lwip or native sockets
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef SOCKET_CLASS_NAME
#define SOCKET_CLASS_NAME SocketAPM
#endif

#include "Socket.hpp"

#undef SOCKET_CLASS_NAME
