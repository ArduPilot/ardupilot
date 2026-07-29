#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_SwarmMesh_config.h"

#define LOG_IDS_FROM_SWARMMESH \
    LOG_SWARMMESH_MSG,         \
    LOG_SWARMMESH_HB_MSG,      \
    LOG_SWARMMESH_SS_MSG,      \
    LOG_SWARMMESH_GP_MSG,      \
    LOG_SWARMMESH_LP_MSG,      \
    LOG_SWARMMESH_PT_MSG,      \
    LOG_SWARMMESH_ES_MSG,      \
    LOG_SWARMMESH_AT_MSG,      \
    LOG_SWARMMESH_EK_MSG,      \
    LOG_SWARMMESH_IM_MSG
    // TODO: Add more log types

// @LoggerMessage: SMST
// @Description: SwarmMesh connection stats
// @Field: TimeUS: Time since system startup
// @Field: CRCFail: RX packets which failed header CRC
// @Field: Stale: RX packets passed deadline threshold
// @Field: TTL: RX packets forwarding expired
// @Field: Dedup: RX packet duplicate count
// @Field: Drop: RX packets dropped
// @Field: TXseq: TX packets sent (original)
// @Field: TXfwd: TX packets sent (forwarded)

struct PACKED log_SwarmMesh {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t crc_fail;
    uint16_t stale;
    uint16_t ttl;
    uint16_t dedup;
    uint16_t drop;
    uint16_t txseq;
    uint16_t txfwd;
    uint16_t txdrop;
};

// @LoggerMessage: SMHB
// @Description: SwarmMesh RX heartbeats
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: VType: Vehicle type
// @Field: Mode: Mode
// @Field: Arm: Armed state

struct PACKED log_SwarmMesh_HB {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    uint8_t  vehicle_type;
    uint8_t  mode;
    uint8_t  armed_state;
};

// @LoggerMessage: SMSS
// @Description: SwarmMesh RX system status
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: BVol: Battery voltage
// @Field: FS: Failsafe flags (unhealthy-sensor bitmask)

struct PACKED log_SwarmMesh_SS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    uint16_t bat_voltage;
    uint32_t failsafe;
};

// @LoggerMessage: SMGP
// @Description: SwarmMesh RX global position
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: Lat: Latitude in degE7
// @Field: Lon: Longitude in degE7
// @Field: Alt: Altitude above MSL in mm
// @Field: VX: Velocity X (NED)
// @Field: VY: Velocity Y (NED)
// @Field: VZ: Velocity Z (NED)

struct PACKED log_SwarmMesh_GP {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    int32_t  lat;
    int32_t  lon;
    int32_t  alt;
    int16_t  vx;
    int16_t  vy;
    int16_t  vz;
};

// @LoggerMessage: SMLP
// @Description: SwarmMesh RX local position
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: x: x distance in m
// @Field: y: y distance in m
// @Field: z: z distance in m
// @Field: VX: Velocity X (local NED)
// @Field: VY: Velocity Y (local NED)
// @Field: VZ: Velocity Z (local NED)

struct PACKED log_SwarmMesh_LP {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    float    x;
    float    y;
    float    z;
    float    vx;
    float    vy;
    float    vz;
};

// @LoggerMessage: SMPT
// @Description: SwarmMesh RX Position Target Global
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: Lat: Latitude in degE7
// @Field: Lon: Longitude in degE7
// @Field: Alt: Altitude AMSL in m
// @Field: VX: Target velocity X (NED)
// @Field: VY: Target velocity Y (NED)
// @Field: VZ: Target velocity Z (NED)
// @Field: AX: Target acceleration X (NED)
// @Field: AY: Target acceleration Y (NED)
// @Field: AZ: Target acceleration Z (NED)

struct PACKED log_SwarmMesh_PT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    int32_t  lat;
    int32_t  lon;
    float    alt;
    float    vx;
    float    vy;
    float    vz;
    float    afx;
    float    afy;
    float    afz;
};

// @LoggerMessage: SMES
// @Description: SwarmMesh RX Extended System State
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: Lat: Latitude in degE7
// @Field: Lon: Longitude in degE7
// @Field: Alt: Altitude above MSL in mm

struct PACKED log_SwarmMesh_ES {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    uint8_t  landed_state;
};

// @LoggerMessage: SMAT
// @Description: SwarmMesh RX Attitude
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: Pitch: Pitch angle in deg
// @Field: Roll: Roll angle deg
// @Field: Yaw: Yaw angle in deg

struct PACKED log_SwarmMesh_AT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    float    pitch;
    float    roll;
    float    yaw;
};

// @LoggerMessage: SMEK
// @Description: SwarmMesh RX EKF Status Report
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: Pos Horiz Variance:
// @Field: Pos Vert Variance:
// @Field: Vel Variance:

struct PACKED log_SwarmMesh_EK {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    float    pos_horiz_var;
    float    pos_vert_var;
    float    vel_var;
};

// @LoggerMessage: SMIM
// @Description: SwarmMesh RX Scaled IMU (actual acceleration)
// @Field: TimeUS: Time since system startup
// @Field: SysID: SysID of origin
// @Field: AX: X acceleration (body frame, mG)
// @Field: AY: Y acceleration (body frame, mG)
// @Field: AZ: Z acceleration (body frame, mG)

struct PACKED log_SwarmMesh_IM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  sysid;
    int16_t  xacc;
    int16_t  yacc;
    int16_t  zacc;
};

#if AP_SWARMMESH_ENABLED
#define LOG_STRUCTURE_FROM_SWARMMESH \
    { LOG_SWARMMESH_MSG, sizeof(log_SwarmMesh), \
        "SMST", "QHHHHHHHH",  "TimeUS,CRCFail,Stale,TTL,Dedup,Drop,TXseq,TXfwd,TXdrop", "s--------", "F--------", true },  \
    { LOG_SWARMMESH_HB_MSG, sizeof(log_SwarmMesh_HB), \
        "SMHB", "QBBBB",  "TimeUS,SysID,VType,Mode,Arm", "s----", "F----", true },  \
    { LOG_SWARMMESH_SS_MSG, sizeof(log_SwarmMesh_SS), \
        "SMSS", "QBHI",  "TimeUS,SysID,BVol,FS", "s---", "F---", true },  \
    { LOG_SWARMMESH_GP_MSG, sizeof(log_SwarmMesh_GP), \
        "SMGP", "QBLLihhh",  "TimeUS,SysID,Lat,Lon,Alt,VX,VY,VZ", "s----nnn", "F----BBB", true },  \
    { LOG_SWARMMESH_LP_MSG, sizeof(log_SwarmMesh_LP), \
        "SMLP", "QBffffff",  "TimeUS,SysID,x,y,z,VX,VY,VZ", "s-mmmnnn", "F-000000", true },  \
    { LOG_SWARMMESH_PT_MSG, sizeof(log_SwarmMesh_PT), \
        "SMPT", "QBLLfffffff",  "TimeUS,SysID,Lat,Lon,Alt,VX,VY,VZ,AX,AY,AZ", "s---mnnnooo", "F---0000000", true },  \
    { LOG_SWARMMESH_ES_MSG, sizeof(log_SwarmMesh_ES), \
        "SMES", "QBB",  "TimeUS,SysID,LS", "s--", "F--", true },  \
    { LOG_SWARMMESH_AT_MSG, sizeof(log_SwarmMesh_AT), \
        "SMAT", "QBfff",  "TimeUS,SysID,Pitch,Roll,Yaw", "s----", "F----", true },  \
    { LOG_SWARMMESH_EK_MSG, sizeof(log_SwarmMesh_EK), \
        "SMEK", "QBfff",  "TimeUS,SysID,PHV,PVV,VV", "s----", "F----", true },  \
    { LOG_SWARMMESH_IM_MSG, sizeof(log_SwarmMesh_IM), \
        "SMIM", "QBhhh",  "TimeUS,SysID,AX,AY,AZ", "s----", "F----", true },
#else
#define LOG_STRUCTURE_FROM_SWARMMESH
#endif