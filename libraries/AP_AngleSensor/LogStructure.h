#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_AngleSensor_config.h"

#define LOG_IDS_FROM_ANGLESENSOR \
    LOG_ANGLESENSOR_MSG

// @LoggerMessage: AENC
// @Description: Angle Sensor Status
// @Field: TimeUS: Time since system startup
// @Field: Angle0: Absolute angle of axis 0
// @Field: Qual0: Measurement quality for axis 0
// @Field: Angle1: Absolute angle of axis 1
// @Field: Qual1: Measurement quality for axis 1


struct PACKED log_AngleSensor {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float angle_0;
    uint8_t quality_0;
    float angle_1;
    uint8_t quality_1;
};

#if !AP_ANGLESENSOR_ENABLED
#define LOG_STRUCTURE_FROM_ANGLESENSOR
#else
#define LOG_STRUCTURE_FROM_ANGLESENSOR \
   { LOG_ANGLESENSOR_MSG, sizeof(log_AngleSensor), \
      "AENC",  "Qfbfb", "TimeUS,Angle0,Qual0,Angle1,Qual1", "sr%r%", "F0-0-" , true }, 
#endif
