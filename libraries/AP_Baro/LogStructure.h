#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_BARO \
    LOG_BARO_MSG

// @LoggerMessage: BARO
// @Description: Gathered Barometer data
// @Field: TimeUS: Time since system startup
// @Field: I: barometer sensor instance number
// @Field: Alt: calculated altitude
// @Field: Press: measured atmospheric pressure
// @Field: Temp: measured atmospheric temperature
// @Field: CRt: derived climb rate from primary barometer
// @Field: SMS: time last sample was taken
// @Field: Offset: raw adjustment of barometer altitude, zeroed on calibration, possibly set by GCS
// @Field: GndTemp: temperature on ground, specified by parameter or measured while on ground
// @Field: Health: true if barometer is considered healthy
struct PACKED log_BARO {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float   altitude;
    float   pressure;
    int16_t temperature;
    float   climbrate;
    uint32_t sample_time_ms;
    float   drift_offset;
    float   ground_temp;
    uint8_t healthy;
};

#define LOG_STRUCTURE_FROM_BARO \
    { LOG_BARO_MSG, sizeof(log_BARO), \
      "BARO",  "QBffcfIffB", "TimeUS,I,Alt,Press,Temp,CRt,SMS,Offset,GndTemp,Health", "s#mPOnsmO-", "F-00B0C?0-" },
