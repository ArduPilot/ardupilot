#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_BARO \
    LOG_BARO_MSG, \
    LOG_BARD_MSG

// @LoggerMessage: BARO
// @Description: Gathered Barometer data
// @Field: TimeUS: Time since system startup
// @Field: I: barometer sensor instance number
// @Field: Alt: calculated altitude
// @Field: AltAMSL: altitude AMSL
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
    float   altitude_AMSL;
    float   pressure;
    int16_t temperature;
    float   climbrate;
    uint32_t sample_time_ms;
    float   drift_offset;
    float   ground_temp;
    uint8_t healthy;
};

// @LoggerMessage: BARD
// @Description: Barometer dynamic data
// @Field: TimeUS: Time since system startup
// @Field: I: barometer sensor instance number
// @Field: DynPrX: calculated dynamic pressure in the bodyframe X-axis
// @Field: DynPrY: calculated dynamic pressure in the bodyframe Y-axis
// @Field: DynPrZ: calculated dynamic pressure in the bodyframe Z-axis
struct PACKED log_BARD {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float dyn_pressure_x;
    float dyn_pressure_y;
    float dyn_pressure_z;
};

#define LOG_STRUCTURE_FROM_BARO                                         \
    { LOG_BARO_MSG, sizeof(log_BARO),                                   \
            "BARO",                                                     \
            "Q"       "B"  "f"    "f"        "f"      "c"     "f"    "I"    "f"       "f"        "B", \
            "TimeUS," "I," "Alt," "AltAMSL," "Press," "Temp," "CRt," "SMS," "Offset," "GndTemp," "Health", \
            "s"       "#"  "m"    "m"        "P"      "O"     "n"    "s"    "m"       "O"        "-", \
            "F"       "-"  "0"    "0"        "0"      "B"     "0"    "C"    "?"       "0"        "-", \
            true                                                        \
            },                                                          \
    { LOG_BARD_MSG, sizeof(log_BARD),                                   \
            "BARD",                                                     \
            "Q"       "B"  "fff", \
            "TimeUS," "I," "DynPrX,DynPrY,DynPrZ", \
            "s"       "#"  "PPP", \
            "F"       "-"  "000", \
            true                                                        \
            },
