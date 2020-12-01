#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_NAVEKF2 \
    LOG_NKT_MSG,  \
    LOG_NKY0_MSG, \
    LOG_NKY1_MSG


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

// @LoggerMessage: NKY0
// @Description: EKF2 Yaw Estimator States
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: YC: GSF yaw estimate (rad)
// @Field: YCS: GSF yaw estimate 1-Sigma uncertainty (rad)
// @Field: Y0: Yaw estimate from individual EKF filter 0 (rad)
// @Field: Y1: Yaw estimate from individual EKF filter 1 (rad)
// @Field: Y2: Yaw estimate from individual EKF filter 2 (rad)
// @Field: Y3: Yaw estimate from individual EKF filter 3 (rad)
// @Field: Y4: Yaw estimate from individual EKF filter 4 (rad)
// @Field: W0: Weighting applied to yaw estimate from individual EKF filter 0
// @Field: W1: Weighting applied to yaw estimate from individual EKF filter 1
// @Field: W2: Weighting applied to yaw estimate from individual EKF filter 2
// @Field: W3: Weighting applied to yaw estimate from individual EKF filter 3
// @Field: W4: Weighting applied to yaw estimate from individual EKF filter 4
struct PACKED log_NKY0 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float yaw_composite;
    float yaw_composite_variance;
    float yaw0;
    float yaw1;
    float yaw2;
    float yaw3;
    float yaw4;
    float wgt0;
    float wgt1;
    float wgt2;
    float wgt3;
    float wgt4;
};

// @LoggerMessage: NKY1
// @Description: EKF2 Yaw Estimator Innovations
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: IVN0: North velocity innovation from individual EKF filter 0 (m/s)
// @Field: IVN1: North velocity innovation from individual EKF filter 1 (m/s)
// @Field: IVN2: North velocity innovation from individual EKF filter 2 (m/s)
// @Field: IVN3: North velocity innovation from individual EKF filter 3 (m/s)
// @Field: IVN4: North velocity innovation from individual EKF filter 4 (m/s)
// @Field: IVE0: East velocity innovation from individual EKF filter 0 (m/s)
// @Field: IVE1: East velocity innovation from individual EKF filter 1 (m/s)
// @Field: IVE2: East velocity innovation from individual EKF filter 2 (m/s)
// @Field: IVE3: East velocity innovation from individual EKF filter 3 (m/s)
// @Field: IVE4: East velocity innovation from individual EKF filter 4 (m/s)
struct PACKED log_NKY1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float ivn0;
    float ivn1;
    float ivn2;
    float ivn3;
    float ivn4;
    float ive0;
    float ive1;
    float ive2;
    float ive3;
    float ive4;
};

#define LOG_STRUCTURE_FROM_NAVEKF2        \
    { LOG_NKT_MSG, sizeof(log_NKT),   \
      "NKT", "QBIffffffff", "TimeUS,C,Cnt,IMUMin,IMUMax,EKFMin,EKFMax,AngMin,AngMax,VMin,VMax", "s#sssssssss", "F-000000000"}, \
    { LOG_NKY0_MSG, sizeof(log_NKY0),                         \
      "NKY0", "QBffffffffffff", "TimeUS,C,YC,YCS,Y0,Y1,Y2,Y3,Y4,W0,W1,W2,W3,W4", "s#rrrrrrr-----", "F-000000000000"}, \
    { LOG_NKY1_MSG, sizeof(log_NKY1),                         \
      "NKY1", "QBffffffffff", "TimeUS,C,IVN0,IVN1,IVN2,IVN3,IVN4,IVE0,IVE1,IVE2,IVE3,IVE4", "s#nnnnnnnnnn", "F-0000000000"},
