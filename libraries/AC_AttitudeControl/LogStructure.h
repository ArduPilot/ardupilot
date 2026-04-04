#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_AC_ATTITUDECONTROL \
    LOG_PSCN_MSG, \
    LOG_PSCE_MSG, \
    LOG_PSCD_MSG, \
    LOG_PSON_MSG, \
    LOG_PSOE_MSG, \
    LOG_PSOD_MSG, \
    LOG_PSOT_MSG, \
    LOG_ANG_MSG

// @LoggerMessage: PSCN
// @Description: Position Control North
// @Field: TimeUS: Time since system startup
// @Field: DPN: Desired position relative to EKF origin
// @Field: TPN: Target position relative to EKF origin
// @Field: PN: Position relative to EKF origin
// @Field: DVN: Desired velocity North
// @Field: TVN: Target velocity North
// @Field: VN: Velocity North
// @Field: DAN: Desired acceleration North
// @Field: TAN: Target acceleration North
// @Field: AN: Acceleration North

// @LoggerMessage: PSCE
// @Description: Position Control East
// @Field: TimeUS: Time since system startup
// @Field: DPE: Desired position relative to EKF origin + Offsets
// @Field: TPE: Target position relative to EKF origin
// @Field: PE: Position relative to EKF origin
// @Field: DVE: Desired velocity East
// @Field: TVE: Target velocity East
// @Field: VE: Velocity East
// @Field: DAE: Desired acceleration East
// @Field: TAE: Target acceleration East
// @Field: AE: Acceleration East

// @LoggerMessage: PSCD
// @Description: Position Control Down
// @Field: TimeUS: Time since system startup
// @Field: DPD: Desired position relative to EKF origin + Offsets
// @Field: TPD: Target position relative to EKF origin
// @Field: PD: Position relative to EKF origin
// @Field: DVD: Desired velocity Down
// @Field: TVD: Target velocity Down
// @Field: VD: Velocity Down
// @Field: DAD: Desired acceleration Down
// @Field: TAD: Target acceleration Down
// @Field: AD: Acceleration Down


// position controller per-axis logging
struct PACKED log_PSCx {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float pos_desired;
    float pos_target;
    float pos;
    float vel_desired;
    float vel_target;
    float vel;
    float accel_desired;
    float accel_target;
    float accel;
};

// @LoggerMessage: PSON
// @Description: Position Control Offsets North
// @Field: TimeUS: Time since system startup
// @Field: TPON: Target position offset North
// @Field: PON: Position offset North
// @Field: TVON: Target velocity offset North
// @Field: VON: Velocity offset North
// @Field: TAON: Target acceleration offset North
// @Field: AON: Acceleration offset North

// @LoggerMessage: PSOE
// @Description: Position Control Offsets East
// @Field: TimeUS: Time since system startup
// @Field: TPOE: Target position offset East
// @Field: POE: Position offset East
// @Field: TVOE: Target velocity offset East
// @Field: VOE: Velocity offset East
// @Field: TAOE: Target acceleration offset East
// @Field: AOE: Acceleration offset East

// @LoggerMessage: PSOD
// @Description: Position Control Offsets Down
// @Field: TimeUS: Time since system startup
// @Field: TPOD: Target position offset Down
// @Field: POD: Position offset Down
// @Field: TVOD: Target velocity offset Down
// @Field: VOD: Velocity offset Down
// @Field: TAOD: Target acceleration offset Down
// @Field: AOD: Acceleration offset Down

// @LoggerMessage: PSOT
// @Description: Position Control Offsets Terrain (Down)
// @Field: TimeUS: Time since system startup
// @Field: TPOT: Target position offset Terrain
// @Field: POT: Position offset Terrain
// @Field: TVOT: Target velocity offset Terrain
// @Field: VOT: Velocity offset Terrain
// @Field: TAOT: Target acceleration offset Terrain
// @Field: AOT: Acceleration offset Terrain

// position controller per-axis offset logging
struct PACKED log_PSOx {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float pos_target_offset;
    float pos_offset;
    float vel_target_offset;
    float vel_offset;
    float accel_target_offset;
    float accel_offset;
};

// @LoggerMessage: RATE
// @Description: Desired and achieved vehicle attitude rates. Not logged in Fixed Wing Plane modes.
// @Field: TimeUS: Time since system startup
// @Field: RDes: vehicle desired roll rate
// @Field: R: achieved vehicle roll rate
// @Field: ROut: normalized output for Roll
// @Field: PDes: vehicle desired pitch rate
// @Field: P: vehicle pitch rate
// @Field: POut: normalized output for Pitch
// @Field: YDes: vehicle desired yaw rate
// @Field: Y: achieved vehicle yaw rate
// @Field: YOut: normalized output for Yaw
// @Field: ADes: desired vehicle vertical acceleration
// @Field: A: achieved vehicle vertical acceleration
// @Field: AOut: percentage of vertical thrust output current being used
// @Field: AOutSlew: vertical thrust output slew rate
struct PACKED log_Rate {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   control_roll;
    float   roll;
    float   roll_out;
    float   control_pitch;
    float   pitch;
    float   pitch_out;
    float   control_yaw;
    float   yaw;
    float   yaw_out;
    float   control_accel;
    float   accel;
    float   accel_out;
    float   throttle_slew;
};

// @LoggerMessage: ANG
// @Description: Attitude control attitude
// @Field: TimeUS: Timestamp of the current Attitude loop
// @Field: DesRoll: vehicle desired roll
// @Field: Roll: achieved vehicle roll
// @Field: DesPitch: vehicle desired pitch
// @Field: Pitch: achieved vehicle pitch
// @Field: DesYaw: vehicle desired yaw
// @Field: Yaw: achieved vehicle yaw
// @Field: Dt: attitude delta time
struct PACKED log_ANG {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float control_roll;
    float roll;
    float control_pitch;
    float pitch;
    float control_yaw;
    float yaw;
    float sensor_dt;
};

#define PSCx_FMT "Qfffffffff"
#define PSCx_UNITS "smmmnnnooo"
#define PSCx_MULTS "F000000000"

#define PSOx_FMT "Qffffff"
#define PSOx_UNITS "smmnnoo"
#define PSOx_MULTS "F000000"

#define LOG_STRUCTURE_FROM_AC_ATTITUDECONTROL        \
    { LOG_PSCN_MSG, sizeof(log_PSCx), \
      "PSCN", PSCx_FMT, "TimeUS,DPN,TPN,PN,DVN,TVN,VN,DAN,TAN,AN", PSCx_UNITS, PSCx_MULTS }, \
    { LOG_PSCE_MSG, sizeof(log_PSCx), \
      "PSCE", PSCx_FMT, "TimeUS,DPE,TPE,PE,DVE,TVE,VE,DAE,TAE,AE", PSCx_UNITS, PSCx_MULTS }, \
    { LOG_PSCD_MSG, sizeof(log_PSCx), \
      "PSCD", PSCx_FMT, "TimeUS,DPD,TPD,PD,DVD,TVD,VD,DAD,TAD,AD", PSCx_UNITS, PSCx_MULTS }, \
    { LOG_PSON_MSG, sizeof(log_PSOx), \
      "PSON", PSOx_FMT, "TimeUS,TPON,PON,TVON,VON,TAON,AON", PSOx_UNITS, PSOx_MULTS }, \
    { LOG_PSOE_MSG, sizeof(log_PSOx), \
      "PSOE", PSOx_FMT, "TimeUS,TPOE,POE,TVOE,VOE,TAOE,AOE", PSOx_UNITS, PSOx_MULTS }, \
    { LOG_PSOD_MSG, sizeof(log_PSOx), \
      "PSOD", PSOx_FMT, "TimeUS,TPOD,POD,TVOD,VOD,TAOD,AOD", PSOx_UNITS, PSOx_MULTS }, \
    { LOG_PSOT_MSG, sizeof(log_PSOx), \
      "PSOT", PSOx_FMT, "TimeUS,TPOT,POT,TVOT,VOT,TAOT,AOT", PSOx_UNITS, PSOx_MULTS }, \
    { LOG_RATE_MSG, sizeof(log_Rate), \
        "RATE", "Qfffffffffffff",  "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut,AOutSlew", "skk-kk-kk-oo--", "F?????????BB--" , true }, \
    { LOG_ANG_MSG, sizeof(log_ANG),\
        "ANG", "Qfffffff", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,Dt", "sddddhhs", "F0000000" , true }
