#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Networking/AP_Networking_Config.h>

#ifndef AP_DDS_ENABLED
#define AP_DDS_ENABLED 1
#endif

// UDP only on SITL for now
#ifndef AP_DDS_UDP_ENABLED
#define AP_DDS_UDP_ENABLED AP_DDS_ENABLED && AP_NETWORKING_SOCKETS_ENABLED
#endif

#include <AP_VisualOdom/AP_VisualOdom_config.h>
#ifndef AP_DDS_VISUALODOM_ENABLED
#define AP_DDS_VISUALODOM_ENABLED HAL_VISUALODOM_ENABLED && AP_DDS_ENABLED
#endif
