#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_TRUSTED_FLIGHT \
    LOG_TRUSTED_FLIGHT_MSG

#define LOG_STRUCTURE_FROM_TRUSTED_FLIGHT \
    { LOG_TRUSTED_FLIGHT_MSG, sizeof(log_Message), \
      "ATF", "QZ", "TimeUS,Message", "s-", "F-" },
