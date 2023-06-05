#pragma once

#include <AP_Logger/LogStructure.h>

// @LoggerMessage: MNT
// @Description: Mount information
// @Field: TimeUS: Time since system startup
// @Field: I: Instance number
// @Field: TR: target vehicle roll
// @Field: TP: target vehicle pitch
// @Field: TY: target vehicle yaw
// @Field: MR: current vehicle roll
// @Field: MP: current vehicle pitch
// @Field: MY: current vehicle yaw
struct PACKED log_mount {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    //uint16_t mode;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
    int16_t  mount_roll;
    int16_t  mount_pitch;
    uint16_t mount_yaw;
};
 
#define LOG_STRUCTURE_FROM_MOUNT \
    { LOG_MOUNT_MSG, sizeof(log_mount), \
      "MNT", "QBccCccC","TimeUS,I,TR,TP,TY,MR,MP,MY", "s#dddddd", "F-BBBBBB" },

