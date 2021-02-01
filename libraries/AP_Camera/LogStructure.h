#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_CAMERA \
    LOG_CAMERA_MSG, \
    LOG_TRIGGER_MSG

// @LoggerMessage: CAM,TRIG
// @Description: Camera shutter information
// @Field: TimeUS: Time since system startup
// @Field: GPSTime: milliseconds since start of GPS week
// @Field: GPSWeek: weeks since 5 Jan 1980
// @Field: Lat: current latitude
// @Field: Lng: current longitude
// @Field: Alt: current altitude
// @Field: RelAlt: current altitude relative to home
// @Field: GPSAlt: altitude as reported by GPS
// @Field: Roll: current vehicle roll
// @Field: Pitch: current vehicle pitch
// @Field: Yaw: current vehicle yaw
struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t gps_time;
    uint16_t gps_week;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    int32_t  altitude_rel;
    int32_t  altitude_gps;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
};

#define LOG_STRUCTURE_FROM_CAMERA \
    { LOG_CAMERA_MSG, sizeof(log_Camera), \
      "CAM", "QIHLLeeeccC","TimeUS,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,Roll,Pitch,Yaw", "s--DUmmmddd", "F--GGBBBBBB" }, \
    { LOG_TRIGGER_MSG, sizeof(log_Camera), \
      "TRIG", "QIHLLeeeccC","TimeUS,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,Roll,Pitch,Yaw", "s--DUmmmddd", "F--GGBBBBBB" },
