#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_TRUSTED_FLIGHT \
    LOG_TRUSTED_FLIGHT_MSG

// @LoggerMessage: TFL
// @Description: Trusted Flight messages
// @Field: TimeUS: Time since system startup
// @Field: Message: message text

#define LOG_STRUCTURE_FROM_TRUSTED_FLIGHT \
    { LOG_TRUSTED_FLIGHT_MSG, sizeof(log_Message), \
      "TFL", "QZ", "TimeUS,Message", "s-", "F-" },
