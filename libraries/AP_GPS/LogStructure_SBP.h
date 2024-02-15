#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_GPS_config.h"

#define LOG_IDS_FROM_GPS_SBP                    \
    LOG_MSG_SBPHEALTH,                          \
    LOG_MSG_SBPRAWH,                            \
    LOG_MSG_SBPRAWM,                            \
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

// @LoggerMessage: SBRH
// @Description: Swift Raw Message Data
// @Field: TimeUS: Time since system startup
// @Field: msg_flag: Swift message type
// @Field: 1: Sender ID
// @Field: 2: index; always 1
// @Field: 3: pages; number of pages received
// @Field: 4: msg length; number of bytes received
// @Field: 5: unused; always zero
// @Field: 6: data received from device

struct PACKED log_SbpRAWH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t msg_type;
    uint16_t sender_id;
    uint8_t index;
    uint8_t pages;
    uint8_t msg_len;
    uint8_t res;
    uint8_t data[48];
};

struct PACKED log_SbpRAWM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t msg_type;
    uint16_t sender_id;
    uint8_t index;
    uint8_t pages;
    uint8_t msg_len;
    uint8_t res;
    uint8_t data[104];
};

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
    { LOG_MSG_SBPRAWH, sizeof(log_SbpRAWH), \
      "SBRH", "QQQQQQQQ", "TimeUS,msg_flag,1,2,3,4,5,6", "s-------", "F-------" , true }, \
    { LOG_MSG_SBPRAWM, sizeof(log_SbpRAWM), \
      "SBRM", "QQQQQQQQQQQQQQQ", "TimeUS,msg_flag,1,2,3,4,5,6,7,8,9,10,11,12,13", "s??????????????", "F??????????????" , true }, \
    { LOG_MSG_SBPEVENT, sizeof(log_SbpEvent), \
      "SBRE", "QHIiBB", "TimeUS,GWk,GMS,ns_residual,level,quality", "s?????", "F?????" },
#else
#define LOG_STRUCTURE_FROM_GPS_SBP
#endif
