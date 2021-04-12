#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_PRECLAND \
    LOG_PRECLAND_MSG

// @LoggerMessage: PL
// @Description: Precision Landing messages
// @Field: TimeUS: Time since system startup
// @Field: Heal: True if Precision Landing is healthy
// @Field: TAcq: True if landing target is detected
// @Field: pX: Target position relative to vehicle, X-Axis (0 if target not found)
// @Field: pY: Target position relative to vehicle, Y-Axis (0 if target not found)
// @Field: vX: Target velocity relative to vehicle, X-Axis (0 if target not found)
// @Field: vY: Target velocity relative to vehicle, Y-Axis (0 if target not found)
// @Field: mX: Target's relative to origin position as 3-D Vector, X-Axis
// @Field: mY: Target's relative to origin position as 3-D Vector, Y-Axis
// @Field: mZ: Target's relative to origin position as 3-D Vector, Z-Axis
// @Field: LastMeasMS: Time when target was last detected
// @Field: EKFOutl: EKF's outlier count
// @Field: Est: Type of estimator used

// precision landing logging
struct PACKED log_Precland {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t healthy;
    uint8_t target_acquired;
    float pos_x;
    float pos_y;
    float vel_x;
    float vel_y;
    float meas_x;
    float meas_y;
    float meas_z;
    uint32_t last_meas;
    uint32_t ekf_outcount;
    uint8_t estimator;
};

#define LOG_STRUCTURE_FROM_PRECLAND                                     \
    { LOG_PRECLAND_MSG, sizeof(log_Precland),                           \
      "PL",    "QBBfffffffIIB",    "TimeUS,Heal,TAcq,pX,pY,vX,vY,mX,mY,mZ,LastMeasMS,EKFOutl,Est", "s--mmnnmmms--","F--BBBBBBBC--" },
