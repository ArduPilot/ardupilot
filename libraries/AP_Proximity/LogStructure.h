#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_Proximity_config.h"

#define LOG_IDS_FROM_PROXIMITY \
    LOG_PROXIMITY_MSG, \
    LOG_RAW_PROXIMITY_MSG

// @LoggerMessage: PRX
// @Description: Proximity Filtered sensor data
// @Field: TimeUS: Time since system startup
// @Field: Layer: Pitch(instance) at which the obstacle is at. 0th layer {-75,-45} degrees. 1st layer {-45,-15} degrees. 2nd layer {-15, 15} degrees. 3rd layer {15, 45} degrees. 4th layer {45,75} degrees. Minimum distance in each layer will be logged.
// @Field: He: True if proximity sensor is healthy
// @Field: D0: Nearest object in sector surrounding 0-degrees
// @Field: D45: Nearest object in sector surrounding 45-degrees
// @Field: D90: Nearest object in sector surrounding 90-degrees
// @Field: D135: Nearest object in sector surrounding 135-degrees
// @Field: D180: Nearest object in sector surrounding 180-degrees
// @Field: D225: Nearest object in sector surrounding 225-degrees
// @Field: D270: Nearest object in sector surrounding 270-degrees
// @Field: D315: Nearest object in sector surrounding 315-degrees
// @Field: DUp: Nearest object in upwards direction
// @Field: CAn: Angle to closest object
// @Field: CDis: Distance to closest object

// proximity sensor logging
struct PACKED log_Proximity {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    uint8_t health;
    float dist0;
    float dist45;
    float dist90;
    float dist135;
    float dist180;
    float dist225;
    float dist270;
    float dist315;
    float distup;
    float closest_angle;
    float closest_dist;
};

// @LoggerMessage: PRXR
// @Description: Proximity Raw sensor data
// @Field: TimeUS: Time since system startup
// @Field: Layer: Pitch(instance) at which the obstacle is at. 0th layer {-75,-45} degrees. 1st layer {-45,-15} degrees. 2nd layer {-15, 15} degrees. 3rd layer {15, 45} degrees. 4th layer {45,75} degrees. Minimum distance in each layer will be logged.
// @Field: D0: Nearest object in sector surrounding 0-degrees
// @Field: D45: Nearest object in sector surrounding 45-degrees
// @Field: D90: Nearest object in sector surrounding 90-degrees
// @Field: D135: Nearest object in sector surrounding 135-degrees
// @Field: D180: Nearest object in sector surrounding 180-degrees
// @Field: D225: Nearest object in sector surrounding 225-degrees
// @Field: D270: Nearest object in sector surrounding 270-degrees
// @Field: D315: Nearest object in sector surrounding 315-degrees

struct PACKED log_Proximity_raw {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float raw_dist0;
    float raw_dist45;
    float raw_dist90;
    float raw_dist135;
    float raw_dist180;
    float raw_dist225;
    float raw_dist270;
    float raw_dist315;
};


#if HAL_PROXIMITY_ENABLED
#define LOG_STRUCTURE_FROM_PROXIMITY \
    { LOG_PROXIMITY_MSG, sizeof(log_Proximity), \
      "PRX", "QBBfffffffffff", "TimeUS,Layer,He,D0,D45,D90,D135,D180,D225,D270,D315,DUp,CAn,CDis", "s#-mmmmmmmmmhm", "F--00000000000", true }, \
    { LOG_RAW_PROXIMITY_MSG, sizeof(log_Proximity_raw), \
      "PRXR", "QBffffffff", "TimeUS,Layer,D0,D45,D90,D135,D180,D225,D270,D315", "s#mmmmmmmm", "F-00000000", true },
#else
#define LOG_STRUCTURE_FROM_PROXIMITY
#endif
