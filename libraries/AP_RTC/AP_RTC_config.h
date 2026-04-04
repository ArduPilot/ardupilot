#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Logger/AP_Logger_config.h>

#ifndef AP_RTC_ENABLED
#define AP_RTC_ENABLED 1
#endif

#ifndef AP_RTC_LOGGING_ENABLED
#define AP_RTC_LOGGING_ENABLED AP_RTC_ENABLED && HAL_LOGGING_ENABLED
#endif
