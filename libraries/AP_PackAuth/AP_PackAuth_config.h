#pragma once
#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_PACKAUTH_ENABLED
#define AP_PACKAUTH_ENABLED 0          // enabled per board in hwdef.dat
#endif
#ifndef AP_PACKAUTH_PARAMS_ENABLED
#define AP_PACKAUTH_PARAMS_ENABLED 1   // 0 = bus/address fixed at build time, no parameters
#endif
#ifndef AP_PACKAUTH_BUS_DEFAULT
#define AP_PACKAUTH_BUS_DEFAULT -1     // -1 = scan the external buses
#endif
#ifndef AP_PACKAUTH_ADDR_DEFAULT
#define AP_PACKAUTH_ADDR_DEFAULT 0x0B
#endif
#ifndef AP_PACKAUTH_DEBUG
#define AP_PACKAUTH_DEBUG 0            // testing only: status text every 5 s
#endif