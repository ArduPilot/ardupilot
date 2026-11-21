#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Logger/AP_Logger_config.h>
#include <AP_Follow/AP_Follow_config.h>

#ifndef AP_RTC_ENABLED
#define AP_RTC_ENABLED 1
#endif

#ifndef AP_RTC_LOGGING_ENABLED
#define AP_RTC_LOGGING_ENABLED AP_RTC_ENABLED && HAL_LOGGING_ENABLED
#endif

// AP_Follow is the only user of the remote clocks
#ifndef AP_RTC_REMOTE_CLOCKS_ENABLED
#define AP_RTC_REMOTE_CLOCKS_ENABLED (AP_RTC_ENABLED && AP_FOLLOW_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

#ifndef AP_RTC_REMOTE_CLOCKS_MAX
#define AP_RTC_REMOTE_CLOCKS_MAX 4
#endif

// a remote boot time further back than this is a reboot; smaller steps back
// are messages arriving late over a slower link
#ifndef AP_RTC_REMOTE_REBOOT_ALLOWANCE_MS
#define AP_RTC_REMOTE_REBOOT_ALLOWANCE_MS 10000
#endif

// the most a correctly converted remote time can appear to be in the future:
// the GPS lock error of this clock plus millisecond rounding
#ifndef AP_RTC_REMOTE_CLOCK_TOLERANCE_MS
#define AP_RTC_REMOTE_CLOCK_TOLERANCE_MS 20
#endif
