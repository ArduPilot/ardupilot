#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Mount/AP_Mount_config.h>

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 1
#endif

// BATTERY2 is slated to be removed:
#ifndef AP_MAVLINK_BATTERY2_ENABLED
#define AP_MAVLINK_BATTERY2_ENABLED 1
#endif

// gremsy is the only user of this feature:
#ifndef AP_MAVLINK_FORWARD_FILTERING_ENABLED
#define AP_MAVLINK_FORWARD_FILTERING_ENABLED HAL_MOUNT_GREMSY_ENABLED
#endif
