#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef OSD_ENABLED
#define OSD_ENABLED 1
#endif

#ifndef HAL_WITH_OSD_BITMAP
#define HAL_WITH_OSD_BITMAP OSD_ENABLED && (defined(HAL_WITH_SPI_OSD) || defined(WITH_SITL_OSD))
#endif

#ifndef OSD_PARAM_ENABLED
#define OSD_PARAM_ENABLED 1
#endif

#ifndef HAL_OSD_SIDEBAR_ENABLE
#define HAL_OSD_SIDEBAR_ENABLE 1
#endif
