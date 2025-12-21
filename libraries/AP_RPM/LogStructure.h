#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_RPM_config.h"

#define LOG_IDS_FROM_RPM \
    LOG_RPM_MSG

// @LoggerMessage: RPM
// @Description: Data from RPM sensors
// @Field: TimeUS: Time since system startup
// @Field: I: Instance
// @Field: RPM: Sensor's rpm measurement
// @Field: Qual: Signal quality
// @Field: H: Sensor Health (Bool)
struct PACKED log_RPM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t inst;
    float rpm;
    float quality;
    uint8_t health;
};

#if AP_RPM_ENABLED
#define LOG_STRUCTURE_FROM_RPM        \
    { LOG_RPM_MSG, sizeof(log_RPM), \
      "RPM",  "QBffB", "TimeUS,I,RPM,Qual,H", "s#q--", "F-000" , true },
#else
#define LOG_STRUCTURE_FROM_RPM
#endif
