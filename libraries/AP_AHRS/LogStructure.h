#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_AHRS \
    LOG_AHR2_MSG, \
    LOG_AOA_SSA_MSG, \
    LOG_ATTITUDE_MSG, \
    LOG_ORGN_MSG, \
    LOG_POS_MSG, \
    LOG_RATE_MSG, \
    LOG_ATSC_MSG

// @LoggerMessage: AHR2
// @Description: Backup AHRS data
// @Field: TimeUS: Time since system startup
// @Field: Roll: Estimated roll
// @Field: Pitch: Estimated pitch
// @Field: Yaw: Estimated yaw
// @Field: Alt: Estimated altitude
// @Field: Lat: Estimated latitude
// @Field: Lng: Estimated longitude
// @Field: Q1: Estimated attitude quaternion component 1
// @Field: Q2: Estimated attitude quaternion component 2
// @Field: Q3: Estimated attitude quaternion component 3
// @Field: Q4: Estimated attitude quaternion component 4
struct PACKED log_AHRS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float alt;
    int32_t lat;
    int32_t lng;
    float q1, q2, q3, q4;
};

// @LoggerMessage: AOA
// @Description: Angle of attack and Side Slip Angle values
// @Field: TimeUS: Time since system startup
// @Field: AOA: Angle of Attack calculated from airspeed, wind vector,velocity vector 
// @Field: SSA: Side Slip Angle calculated from airspeed, wind vector,velocity vector
struct PACKED log_AOA_SSA {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float AOA;
    float SSA;
};

// @LoggerMessage: ATT
// @Description: Canonical vehicle attitude
// @Field: TimeUS: Time since system startup
// @Field: DesRoll: vehicle desired roll
// @Field: Roll: achieved vehicle roll
// @Field: DesPitch: vehicle desired pitch
// @Field: Pitch: achieved vehicle pitch
// @Field: DesYaw: vehicle desired yaw
// @Field: Yaw: achieved vehicle yaw
// @Field: AEKF: active EKF type
struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float control_roll;
    float roll;
    float control_pitch;
    float pitch;
    float control_yaw;
    float yaw;
    uint8_t  active;
};

// @LoggerMessage: ORGN
// @Description: Vehicle navigation origin or other notable position
// @Field: TimeUS: Time since system startup
// @Field: Type: Position type
// @FieldValueEnum: Type: AP_AHRS::LogOriginType
// @Field: Lat: Position latitude
// @Field: Lng: Position longitude
// @Field: Alt: Position altitude
struct PACKED log_ORGN {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t origin_type;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
};

// @LoggerMessage: POS
// @Description: Canonical vehicle position
// @Field: TimeUS: Time since system startup
// @Field: Lat: Canonical vehicle latitude
// @Field: Lng: Canonical vehicle longitude
// @Field: Alt: Canonical vehicle altitude
// @Field: RelHomeAlt: Canonical vehicle altitude relative to home
// @Field: RelOriginAlt: Canonical vehicle altitude relative to navigation origin
struct PACKED log_POS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t lat;
    int32_t lng;
    float alt;
    float rel_home_alt;
    float rel_origin_alt;
};

// @LoggerMessage: VSTB
// @Description: Log message for video stabilisation software such as Gyroflow
// @Field: TimeUS: Time since system startup
// @Field: GyrX: measured rotation rate about X axis
// @Field: GyrY: measured rotation rate about Y axis
// @Field: GyrZ: measured rotation rate about Z axis
// @Field: AccX: acceleration along X axis
// @Field: AccY: acceleration along Y axis
// @Field: AccZ: acceleration along Z axis
// @Field: Q1: Estimated attitude quaternion component 1
// @Field: Q2: Estimated attitude quaternion component 2
// @Field: Q3: Estimated attitude quaternion component 3
// @Field: Q4: Estimated attitude quaternion component 4

struct PACKED log_Video_Stabilisation {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float Q1;
    float Q2;
    float Q3;
    float Q4;
};

// @LoggerMessage: ATSC
// @Description: Scale factors for attitude controller
// @Field: TimeUS: Time since system startup
// @Field: AngPScX: Angle P scale X
// @Field: AngPScY: Angle P scale Y
// @Field: AngPScZ: Angle P scale Z
// @Field: PDScX: PD scale X
// @Field: PDScY: PD scale Y
// @Field: PDScZ: PD scale Z
struct PACKED log_ATSC {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float scaleP_x;
    float scaleP_y;
    float scaleP_z;
    float scalePD_x;
    float scalePD_y;
    float scalePD_z;
};


#define LOG_STRUCTURE_FROM_AHRS \
    { LOG_AHR2_MSG, sizeof(log_AHRS), \
        "AHR2","QccCfLLffff","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng,Q1,Q2,Q3,Q4","sddhmDU----", "FBBB0GG----" , true }, \
    { LOG_AOA_SSA_MSG, sizeof(log_AOA_SSA), \
        "AOA", "Qff", "TimeUS,AOA,SSA", "sdd", "F00" , true }, \
    { LOG_ATTITUDE_MSG, sizeof(log_Attitude),\
        "ATT", "QffffffB", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,AEKF", "sddddhh-", "F000000-" , true }, \
    { LOG_ORGN_MSG, sizeof(log_ORGN), \
        "ORGN","QBLLe","TimeUS,Type,Lat,Lng,Alt", "s#DUm", "F-GGB" }, \
    { LOG_POS_MSG, sizeof(log_POS), \
        "POS","QLLfff","TimeUS,Lat,Lng,Alt,RelHomeAlt,RelOriginAlt", "sDUmmm", "FGG000" , true }, \
    { LOG_ATSC_MSG, sizeof(log_ATSC), \
        "ATSC", "Qffffff",  "TimeUS,AngPScX,AngPScY,AngPScZ,PDScX,PDScY,PDScZ", "s------", "F000000" , true }, \
    { LOG_VIDEO_STABILISATION_MSG, sizeof(log_Video_Stabilisation), \
        "VSTB", "Qffffffffff",  "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,Q1,Q2,Q3,Q4", "sEEEooo----", "F0000000000" },

