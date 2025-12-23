#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_RCProtocol/AP_RCProtocol_config.h>

#ifndef AP_CRSF_PROTOCOL_ENABLED
#define AP_CRSF_PROTOCOL_ENABLED 1
#endif

#ifndef AP_CRSF_OUT_ENABLED
#define AP_CRSF_OUT_ENABLED AP_CRSF_PROTOCOL_ENABLED && AP_RCPROTOCOL_CRSF_ENABLED
#endif

