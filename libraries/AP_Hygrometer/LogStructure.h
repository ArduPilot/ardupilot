#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_HYGROMETER \
    LOG_HYGRO_MSG

// @LoggerMessage: HYGR
// @Description: Hygrometer sensor data
// @Field: TimeUS: Time since system startup
// @Field: I: Hygrometer sensor instance number
// @Field: Temperature: Current temperature
// @Field: Humidity: Current humidity
struct PACKED log_HYGRO {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float temperature;
    float humidity;
};

#define LOG_STRUCTURE_FROM_HYGROMETER \
    { LOG_HYGRO_MSG, sizeof(log_HYGRO), \
      "HYGR", "QBff", "TimeUS,I,Temperature,Humidity", "s#O%", "F-00", true }
