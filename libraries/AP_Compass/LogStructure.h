#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_COMPASS \
    LOG_MAG_MSG

// @LoggerMessage: MAG
// @Description: Information received from compasses
// @Field: TimeUS: Time since system startup
// @Field: I: magnetometer sensor instance number
// @Field: MagX: magnetic field strength in body frame
// @Field: MagY: magnetic field strength in body frame
// @Field: MagZ: magnetic field strength in body frame
// @Field: OfsX: magnetic field offset in body frame
// @Field: OfsY: magnetic field offset in body frame
// @Field: OfsZ: magnetic field offset in body frame
// @Field: MOX: motor interference magnetic field offset in body frame
// @Field: MOY: motor interference magnetic field offset in body frame
// @Field: MOZ: motor interference magnetic field offset in body frame
// @Field: Health: true if the compass is considered healthy
// @Field: S: time measurement was taken
struct PACKED log_MAG {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    int16_t  offset_x;
    int16_t  offset_y;
    int16_t  offset_z;
    int16_t  motor_offset_x;
    int16_t  motor_offset_y;
    int16_t  motor_offset_z;
    uint8_t  health;
    uint32_t SUS;
};

#define LOG_STRUCTURE_FROM_COMPASS \
    { LOG_MAG_MSG, sizeof(log_MAG), \
      "MAG", "QBhhhhhhhhhBI",    "TimeUS,I,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOX,MOY,MOZ,Health,S", "s#GGGGGGGGG-s", "F-CCCCCCCCC-F", true },
