#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_Mount_config.h"

#define LOG_IDS_FROM_MOUNT \
    LOG_MOUNT_MSG

// @LoggerMessage: MNT
// @Description: Mount's desired and actual roll, pitch and yaw angles
// @Field: TimeUS: Time since system startup
// @Field: I: Instance number
// @Field: DRoll: Desired roll
// @Field: Roll: Actual roll
// @Field: DPitch: Desired pitch
// @Field: Pitch: Actual pitch
// @Field: DYawB: Desired yaw in body frame
// @Field: YawB: Actual yaw in body frame
// @Field: DYawE: Desired yaw in earth frame
// @Field: YawE: Actual yaw in earth frame
// @Field: Dist: Rangefinder distance

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
    float    rangefinder_dist;
};

#if HAL_MOUNT_ENABLED
#define LOG_STRUCTURE_FROM_MOUNT \
    { LOG_MOUNT_MSG, sizeof(log_Mount), \
      "MNT", "QBfffffffff","TimeUS,I,DRoll,Roll,DPitch,Pitch,DYawB,YawB,DYawE,YawE,Dist", "s#ddddddddm", "F---------0" },
#else
#define LOG_STRUCTURE_FROM_MOUNT
#endif
