#pragma once

#include <AP_Logger/LogStructure.h>
#include "AC_Fence_config.h"

#define LOG_IDS_FROM_FENCE \
    LOG_FENCE_MSG

// @LoggerMessage: FNCE
// @Description: currently loaded Geo Fence points
// @Field: TimeUS: Time since system startup
// @Field: Tot: total number of stored items
// @Field: Seq: index in current sequence
// @Field: Type: point type
// @Field: Lat: point latitude
// @Field: Lng: point longitude
// @Field: Count: vertex cound in polygon if applicable
// @Field: Radius: radius of circle if applicable

struct PACKED log_Fence {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t total;
    uint8_t sequence;
    uint8_t type;
    int32_t latitude;
    int32_t longitude;
    uint8_t vertex_count;
    float radius;
};

#if !AP_FENCE_ENABLED
#define LOG_STRUCTURE_FROM_FENCE
#else
#define LOG_STRUCTURE_FROM_FENCE \
    { LOG_FENCE_MSG, sizeof(log_Fence), \
      "FNCE", "QBBBLLBf", "TimeUS,Tot,Seq,Type,Lat,Lng,Count,Radius", "s---DU-m", "F---GG--" },
#endif
