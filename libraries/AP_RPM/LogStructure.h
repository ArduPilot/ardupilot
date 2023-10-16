#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_RPM \
    LOG_RPM_MSG

// @LoggerMessage: RPM
// @Description: Data from RPM sensors
// @Field: TimeUS: Time since system startup
// @Field: rpm1: First sensor's data
// @Field: rpm2: Second sensor's data
struct PACKED log_RPM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float rpm1;
    float rpm2;
    float rpm3;
    float rpm4;
};

#define LOG_STRUCTURE_FROM_RPM        \
    { LOG_RPM_MSG, sizeof(log_RPM), \
      "RPM",  "Qffff", "TimeUS,rpm1,rpm2,rpm3,rpm4", "sqqqq", "F0000" , true },
