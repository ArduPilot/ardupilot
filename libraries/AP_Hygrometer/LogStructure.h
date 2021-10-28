#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_HYGROMETER \
    LOG_HYGRO_MSG

// @LoggerMessage: HYGR
// @Description: Hygrometer sensor data
// @Field: TimeUS: Time since system startup
// @Field: I: Hygrometer sensor instance number
// @Field: Temp: Current temperature
// @Field: Humi: Current humidity
// @Field: Pri: True if sensor is the primary sensor
struct PACKED log_HYGRO {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    int16_t temperature;
    uint16_t   humidity;
    uint8_t primary;
};

#define LOG_STRUCTURE_FROM_HYGROMETER \
    { LOG_HYGRO_MSG, sizeof(log_HYGRO), \
      "HYGR", "QBcCB", "TimeUS,I,Temp,Humi,Pri", "s#O%-", "F-BB-", true }
