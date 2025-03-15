#pragma once

#include <AP_Logger/LogStructure.h>
#include "GCS_config.h"

#define LOG_IDS_FROM_GCS \
    LOG_NVF_MSG

// @LoggerMessage: NVF
// @Description: Named Value Float messages; messages sent to GCS via NAMED_VALUE_FLOAT
// @Field: TimeUS: Time since system startup
// @Field: Name: Name of float
// @Field: Value: Value of float
struct PACKED log_NVF {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    char name[16]; // actually char[10] in the malink packet;
    float value;
};

#if HAL_GCS_ENABLED
#define LOG_STRUCTURE_FROM_GCS        \
    { LOG_NVF_MSG, sizeof(log_NVF), \
      "NVF","QNf","TimeUS,Name,Value", "s#-", "F--" , false },
#else
  #define LOG_STRUCTURE_FROM_GCS
#endif
