#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_MISSION \
    LOG_MISE_MSG,            \
    LOG_CMD_MSG

// @LoggerMessage: CMD
// @Description: Uploaded mission command information
// @Field: TimeUS: Time since system startup
// @Field: CTot: Total number of mission commands
// @Field: CNum: This command's offset in mission
// @Field: CId: Command type
// @Field: Prm1: Parameter 1
// @Field: Prm2: Parameter 2
// @Field: Prm3: Parameter 3
// @Field: Prm4: Parameter 4
// @Field: Lat: Command latitude
// @Field: Lng: Command longitude
// @Field: Alt: Command altitude
// @Field: Frame: Frame used for position
struct PACKED log_CMD {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t command_total;
    uint16_t sequence;
    uint16_t command;
    float param1;
    float param2;
    float param3;
    float param4;
    int32_t latitude;
    int32_t longitude;
    float altitude;
    uint8_t frame;
};

// @LoggerMessage: MISE
// @Description: Executed mission command information; emitted when we start to run an item
// @Field: TimeUS: Time since system startup
// @Field: CTot: Total number of mission commands
// @Field: CNum: This command's offset in mission
// @Field: CId: Command type
// @Field: Prm1: Parameter 1
// @Field: Prm2: Parameter 2
// @Field: Prm3: Parameter 3
// @Field: Prm4: Parameter 4
// @Field: Lat: Command latitude
// @Field: Lng: Command longitude
// @Field: Alt: Command altitude
// @Field: Frame: Frame used for position

// note we currently reuse the same structure for CMD and MISE.
#define LOG_STRUCTURE_FROM_MISSION      \
        { LOG_CMD_MSG, sizeof(log_CMD), \
      "CMD", "QHHHffffLLfB","TimeUS,CTot,CNum,CId,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,Frame", "s-------DUm-", "F-------GG0-" }, \
        { LOG_MISE_MSG, sizeof(log_CMD), \
      "MISE", "QHHHffffLLfB","TimeUS,CTot,CNum,CId,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,Frame", "s-------DUm-", "F-------GG0-" },
