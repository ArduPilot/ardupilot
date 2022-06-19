#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_BEACON \
    LOG_BEACON_MSG

// @LoggerMessage: BCN
// @Description: Beacon information
// @Field: TimeUS: Time since system startup
// @Field: Health: True if beacon sensor is healthy
// @Field: Cnt: Number of beacons being used
// @Field: D0: Distance to first beacon
// @Field: D1: Distance to second beacon
// @Field: D2: Distance to third beacon
// @Field: D3: Distance to fourth beacon
// @Field: PosX: Calculated beacon position, x-axis
// @Field: PosY: Calculated beacon position, y-axis
// @Field: PosZ: Calculated beacon position, z-axis

struct PACKED log_Beacon {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t health;
    uint8_t count;
    float dist0;
    float dist1;
    float dist2;
    float dist3;
    float posx;
    float posy;
    float posz;
};

#define LOG_STRUCTURE_FROM_BEACON \
    { LOG_BEACON_MSG, sizeof(log_Beacon), \
        "BCN", "QBBfffffff",  "TimeUS,Health,Cnt,D0,D1,D2,D3,PosX,PosY,PosZ", "s--mmmmmmm", "F--0000000", true },
