/*
  feature selection for EKF3
 */

#pragma once

#include <AP_Vehicle/AP_Vehicle_Type.h>

// define for when to include all features
#define EK3_FEATURE_ALL APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) || APM_BUILD_TYPE(APM_BUILD_Replay)

// body odomotry (which includes wheel encoding) on rover or 2M boards
#define EK3_FEATURE_BODY_ODOM EK3_FEATURE_ALL || APM_BUILD_TYPE(APM_BUILD_Rover) || BOARD_FLASH_SIZE > 1024
