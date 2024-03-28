#pragma once

#define LOG_IDS_FROM_MAG_PMOT                  \
    LOG_MAG_PMOT_MSG

#include <AP_Logger/LogStructure.h>

// @LoggerMessage: ESC
// @Description: Feedback received from ESCs
// @Field: TimeUS: microseconds since system startup
// @Field: Instance: ESC instance number
// @Field: RPM: reported motor rotation rate
// @Field: RawRPM: reported motor rotation rate without slew adjustment
// @Field: Volt: Perceived input voltage for the ESC
// @Field: Curr: Perceived current through the ESC
// @Field: Temp: ESC temperature in centi-degrees C
// @Field: CTot: current consumed total mAh
// @Field: MotTemp: measured motor temperature in centi-degrees C
// @Field: Err: error rate
struct PACKED log_MAG_PerMotor {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float motor1_offset_x;
    float motor1_offset_y;
    float motor1_offset_z;
    float motor2_offset_x;
    float motor2_offset_y;
    float motor2_offset_z;
    float motor3_offset_x;
    float motor3_offset_y;
    float motor3_offset_z;
    float motor4_offset_x;
    float motor4_offset_y;
    float motor4_offset_z;
    float motor_tot_offset_x;
    float motor_tot_offset_y;
    float motor_tot_offset_z;
};

#define LOG_STRUCTURE_FROM_MAG_PMOT \
    { LOG_MAG_PMOT_MSG, sizeof(log_MAG_PerMotor), \
      "PMOT",  "QBffffffffffff", "TimeUS,Instance,M1X,M1Y,M1Z,M2X,M2Y,M2Z,M3X,M3Y,M3Z,M4X,M4Y,M4Z", "s#------------", "F-------------" , true },
