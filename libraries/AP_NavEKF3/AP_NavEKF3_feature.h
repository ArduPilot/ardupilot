/*
  feature selection for EKF3
 */

#pragma once

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Beacon/AP_Beacon_config.h>

// define for when to include all features
#define EK3_FEATURE_ALL APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) || APM_BUILD_TYPE(APM_BUILD_Replay)

// body odomotry (which includes wheel encoding) on rover or 2M boards
#ifndef EK3_FEATURE_BODY_ODOM
#define EK3_FEATURE_BODY_ODOM EK3_FEATURE_ALL || APM_BUILD_TYPE(APM_BUILD_Rover) || BOARD_FLASH_SIZE > 1024
#endif

// external navigation on 2M boards
#ifndef EK3_FEATURE_EXTERNAL_NAV
#define EK3_FEATURE_EXTERNAL_NAV EK3_FEATURE_ALL || BOARD_FLASH_SIZE > 1024
#endif

// drag fusion on 2M boards
#ifndef EK3_FEATURE_DRAG_FUSION
#define EK3_FEATURE_DRAG_FUSION EK3_FEATURE_ALL || BOARD_FLASH_SIZE > 1024
#endif

// Beacon Fusion if beacon data available
#ifndef EK3_FEATURE_BEACON_FUSION
#define EK3_FEATURE_BEACON_FUSION AP_BEACON_ENABLED
#endif
