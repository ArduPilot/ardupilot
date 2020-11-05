#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_NAVEKF2 \
    LOG_NKT_MSG

struct PACKED log_NKT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    uint32_t timing_count;
    float dtIMUavg_min;
    float dtIMUavg_max;
    float dtEKFavg_min;
    float dtEKFavg_max;
    float delAngDT_min;
    float delAngDT_max;
    float delVelDT_min;
    float delVelDT_max;
};

// @LoggerMessage: NKT
// @Description: EKF2 timing information
// @Field: TimeUS: Time since system startup
// @Field: C: EKF core this message instance applies to
// @Field: Cnt: count of samples used to create this message
// @Field: IMUMin: smallest IMU sample interval
// @Field: IMUMax: largest IMU sample interval
// @Field: EKFMin: low-passed achieved average time step rate for the EKF (minimum)
// @Field: EKFMax: low-passed achieved average time step rate for the EKF (maximum)
// @Field: AngMin: accumulated measurement time interval for the delta angle (minimum)
// @Field: AngMax: accumulated measurement time interval for the delta angle (maximum)
// @Field: VMin: accumulated measurement time interval for the delta velocity (minimum)
// @Field: VMax: accumulated measurement time interval for the delta velocity (maximum)

#define LOG_STRUCTURE_FROM_NAVEKF2        \
    { LOG_NKT_MSG, sizeof(log_NKT),   \
      "NKT", "QBIffffffff", "TimeUS,C,Cnt,IMUMin,IMUMax,EKFMin,EKFMax,AngMin,AngMax,VMin,VMax", "s#sssssssss", "F-000000000"},

