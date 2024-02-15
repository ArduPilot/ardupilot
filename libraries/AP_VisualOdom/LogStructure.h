#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_VisualOdom_config.h"

#define LOG_IDS_FROM_VISUALODOM \
    LOG_VISUALODOM_MSG, \
    LOG_VISUALPOS_MSG, \
    LOG_VISUALVEL_MSG

// @LoggerMessage: VISO
// @Description: Visual Odometry
// @Field: TimeUS: System time
// @Field: dt: Time period this data covers
// @Field: AngDX: Angular change for body-frame roll axis
// @Field: AngDY: Angular change for body-frame pitch axis
// @Field: AngDZ: Angular change for body-frame z axis
// @Field: PosDX: Position change for body-frame X axis (Forward-Back)
// @Field: PosDY: Position change for body-frame Y axis (Right-Left)
// @Field: PosDZ: Position change for body-frame Z axis (Down-Up)
// @Field: conf: Confidence
struct PACKED log_VisualOdom {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float time_delta;
    float angle_delta_x;
    float angle_delta_y;
    float angle_delta_z;
    float position_delta_x;
    float position_delta_y;
    float position_delta_z;
    float confidence;
};

// @LoggerMessage: VISP
// @Description: Vision Position
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: CTimeMS: Corrected system time
// @Field: PX: Position X-axis (North-South)
// @Field: PY: Position Y-axis (East-West)
// @Field: PZ: Position Z-axis (Down-Up)
// @Field: Roll: Roll lean angle
// @Field: Pitch: Pitch lean angle
// @Field: Yaw: Yaw angle
// @Field: PErr: Position estimate error
// @Field: AErr: Attitude estimate error
// @Field: Rst: Position reset counter
// @Field: Ign: Ignored
struct PACKED log_VisualPosition {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float pos_x;
    float pos_y;
    float pos_z;
    float roll;     // degrees
    float pitch;    // degrees
    float yaw;      // degrees
    float pos_err;  // meters
    float ang_err;  // radians
    uint8_t reset_counter;
    uint8_t ignored;
};

// @LoggerMessage: VISV
// @Description: Vision Velocity
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: CTimeMS: Corrected system time
// @Field: VX: Velocity X-axis (North-South)
// @Field: VY: Velocity Y-axis (East-West)
// @Field: VZ: Velocity Z-axis (Down-Up)
// @Field: VErr: Velocity estimate error
// @Field: Rst: Velocity reset counter
// @Field: Ign: Ignored
struct PACKED log_VisualVelocity {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float vel_x;
    float vel_y;
    float vel_z;
    float vel_err;
    uint8_t reset_counter;
    uint8_t ignored;
};

#if HAL_VISUALODOM_ENABLED
#define LOG_STRUCTURE_FROM_VISUALODOM \
    { LOG_VISUALODOM_MSG, sizeof(log_VisualOdom), \
      "VISO", "Qffffffff", "TimeUS,dt,AngDX,AngDY,AngDZ,PosDX,PosDY,PosDZ,conf", "ssrrrmmm-", "FF000000-" }, \
    { LOG_VISUALPOS_MSG, sizeof(log_VisualPosition), \
      "VISP", "QQIffffffffBB", "TimeUS,RTimeUS,CTimeMS,PX,PY,PZ,Roll,Pitch,Yaw,PErr,AErr,Rst,Ign", "sssmmmddhmd--", "FFC00000000--" }, \
    { LOG_VISUALVEL_MSG, sizeof(log_VisualVelocity), \
      "VISV", "QQIffffBB", "TimeUS,RTimeUS,CTimeMS,VX,VY,VZ,VErr,Rst,Ign", "sssnnnn--", "FFC0000--" },
#else
#define LOG_STRUCTURE_FROM_VISUALODOM
#endif
