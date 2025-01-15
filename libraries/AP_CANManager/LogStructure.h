#pragma once
/*
  log structures for AP_CANManager
 */

#include <AP_Logger/LogStructure.h>
#include "AP_CANManager_config.h"

#define LOG_IDS_FROM_CANMANAGER \
    LOG_CANF_MSG,               \
    LOG_CAFD_MSG

// @LoggerMessage: CANF
// @Description: CAN Frame
// @Field: TimeUS: Time since system startup
// @Field: Bus: bus number
// @Field: Id: frame identifier
// @Field: DLC: data length code
// @Field: B0: byte 0
// @Field: B1: byte 1
// @Field: B2: byte 2
// @Field: B3: byte 3
// @Field: B4: byte 4
// @Field: B5: byte 5
// @Field: B6: byte 6
// @Field: B7: byte 7
struct PACKED log_CANF {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t bus;
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
};

// @LoggerMessage: CAFD
// @Description: CANFD Frame
// @Field: TimeUS: Time since system startup
// @Field: Bus: bus number
// @Field: Id: frame identifier
// @Field: DLC: data length code
// @Field: D0: data 0
// @Field: D1: data 1
// @Field: D2: data 2
// @Field: D3: data 3
// @Field: D4: data 4
// @Field: D5: data 5
// @Field: D6: data 6
// @Field: D7: data 7
struct PACKED log_CAFD {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t bus;
    uint32_t id;
    uint8_t dlc;
    uint64_t data[8];
};

#if AP_CAN_LOGGING_ENABLED
#define LOG_STRUCTURE_FROM_CANMANAGER \
    { LOG_CANF_MSG, sizeof(log_CANF), \
            "CANF", \
            "Q"       "B"    "I"    "B"    "B"    "B"    "B"    "B"    "B"    "B"    "B"    "B", \
            "TimeUS," "Bus," "Id,"  "DLC," "B0,"  "B1,"  "B2,"  "B3,"  "B4,"  "B5,"  "B6,"  "B7", \
            "s"       "#"    "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-", \
            "F"       "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-", \
            false \
            }, \
    { LOG_CAFD_MSG, sizeof(log_CAFD), \
            "CAFD", \
            "Q"       "B"    "I"    "B"    "Q"    "Q"    "Q"    "Q"    "Q"    "Q"    "Q"    "Q", \
            "TimeUS," "Bus," "Id,"  "DLC," "D0,"  "D1,"  "D2,"  "D3,"  "D4,"  "D5,"  "D6,"  "D7", \
            "s"       "#"    "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-", \
            "F"       "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-"    "-", \
            false \
            },
#else
#define LOG_STRUCTURE_FROM_CANMANAGER
#endif // AP_CAN_LOGGING_ENABLED
