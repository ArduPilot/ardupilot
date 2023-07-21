#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_MOUNT \
    LOG_MOUNT_MSG

// @LoggerMessage: MNT
// @Description: Mount's actual and Target/Desired RPY information
// @Field: TimeUS: Time since system startup
// @Field: I: Instance number
// @Field: DesRoll: mount's desired roll
// @Field: Roll: mount's actual roll
// @Field: DesPitch: mount's desired pitch
// @Field: Pitch: mount's actual pitch
// @Field: DesYawB: mount's desired yaw in body frame
// @Field: YawB: mount's actual yaw in body frame
// @Field: DesYawE: mount's desired yaw in earth frame
// @Field: YawE: mount's actual yaw in earth frame

struct PACKED log_Mount {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float    desired_roll;
    float    actual_roll;
    float    desired_pitch;
    float    actual_pitch;
    float    desired_yaw_bf;
    float    actual_yaw_bf;
    float    desired_yaw_ef;
    float    actual_yaw_ef;
};

#define LOG_STRUCTURE_FROM_MOUNT \
    { LOG_MOUNT_MSG, sizeof(log_Mount), \
      "MNT", "QBffffffff","TimeUS,I,DesRoll,Roll,DesPitch,Pitch,DesYawB,YawB,DesYawE,YawE", "s#dddddddd", "F---------" },

