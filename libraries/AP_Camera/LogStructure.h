#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_Camera_config.h"

#define LOG_IDS_FROM_CAMERA \
    LOG_CAMERA_MSG, \
    LOG_TRIGGER_MSG

// @LoggerMessage: CAM,TRIG
// @Description: Camera shutter information
// @Field: TimeUS: Time since system startup
// @Field: I: Instance number
// @Field: Img: Image number
// @Field: GPSTime: milliseconds since start of GPS week
// @Field: GPSWeek: weeks since 5 Jan 1980
// @Field: Lat: current latitude
// @Field: Lng: current longitude
// @Field: Alt: current altitude
// @Field: RelAlt: current altitude relative to home
// @Field: GPSAlt: altitude as reported by GPS
// @Field: R: current vehicle roll
// @Field: P: current vehicle pitch
// @Field: Y: current vehicle yaw
struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    uint16_t image_number;
    uint32_t gps_time;
    uint16_t gps_week;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    int32_t  altitude_rel;
    int32_t  altitude_gps;
    float  roll;
    float  pitch;
    float yaw;
};

#if AP_CAMERA_ENABLED
#define LOG_STRUCTURE_FROM_CAMERA \
    { LOG_CAMERA_MSG, sizeof(log_Camera), \
      "CAM", "QBHIHLLeeefff","TimeUS,I,Img,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,R,P,Y", "s#---DUmmmddd", "F----GGBBB000" }, \
    { LOG_TRIGGER_MSG, sizeof(log_Camera), \
      "TRIG", "QBHIHLLeeefff","TimeUS,I,Img,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,R,P,Y", "s#---DUmmmddd", "F----GGBBB000" },
#else
#define LOG_STRUCTURE_FROM_CAMERA
#endif
