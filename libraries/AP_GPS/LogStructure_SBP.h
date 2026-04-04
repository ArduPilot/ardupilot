#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_GPS_config.h"

#define LOG_IDS_FROM_GPS_SBP                    \
    LOG_MSG_SBPHEALTH,                          \
    LOG_MSG_SBPEVENT

// @LoggerMessage: SBPH
// @Description: Swift Health Data
// @Field: TimeUS: Time since system startup
// @Field: CrcError: Number of packet CRC errors on serial connection
// @Field: LastInject: Timestamp of last raw data injection to GPS
// @Field: IARhyp: Current number of integer ambiguity hypotheses

struct PACKED log_SbpHealth {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t crc_error_counter;
    uint32_t last_injected_data_ms;
    uint32_t last_iar_num_hypotheses;
};

// @LoggerMessage: SBRE
// @Description: Swift Time Data
// @Field: TimeUS: Time since system startup
// @Field: GWk: GPS week number
// @Field: GMS: Milliseconds through GPS week
// @Field: ns_residual: residual of milliseconds rounding in ns
// @Field: level: GPIO pin levels
// @Field: quality: time quality

struct PACKED log_SbpEvent {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t wn;
    uint32_t tow;
    int32_t ns_residual;
    uint8_t level;
    uint8_t quality;
};

#if AP_GPS_SBP_ENABLED
#define LOG_STRUCTURE_FROM_GPS_SBP \
    { LOG_MSG_SBPHEALTH, sizeof(log_SbpHealth), \
      "SBPH", "QIII", "TimeUS,CrcError,LastInject,IARhyp", "s---", "F---" , true }, \
    { LOG_MSG_SBPEVENT, sizeof(log_SbpEvent), \
      "SBRE", "QHIiBB", "TimeUS,GWk,GMS,ns_residual,level,quality", "s?????", "F?????" },
#else
#define LOG_STRUCTURE_FROM_GPS_SBP
#endif
