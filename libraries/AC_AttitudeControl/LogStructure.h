#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_AC_ATTITUDECONTROL \
    LOG_PSCN_MSG, \
    LOG_PSCE_MSG, \
    LOG_PSCD_MSG

// @LoggerMessage: PSCN
// @Description: Position Control North
// @Field: TimeUS: Time since system startup
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
    float pos_target;
    float pos;
    float vel_desired;
    float vel_target;
    float vel;
    float accel_desired;
    float accel_target;
    float accel;
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
// @Field: Y: achieved vehicle yaw rate
// @Field: YOut: normalized output for Yaw
// @Field: YDes: vehicle desired yaw rate
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

#define PSCx_FMT "Qffffffff"
#define PSCx_UNITS "smmnnnooo"
#define PSCx_MULTS "F00000000"

#define LOG_STRUCTURE_FROM_AC_ATTITUDECONTROL        \
    { LOG_PSCN_MSG, sizeof(log_PSCx), \
      "PSCN", PSCx_FMT, "TimeUS,TPN,PN,DVN,TVN,VN,DAN,TAN,AN", PSCx_UNITS, PSCx_MULTS }, \
    { LOG_PSCE_MSG, sizeof(log_PSCx), \
      "PSCE", PSCx_FMT, "TimeUS,TPE,PE,DVE,TVE,VE,DAE,TAE,AE", PSCx_UNITS, PSCx_MULTS }, \
    { LOG_PSCD_MSG, sizeof(log_PSCx), \
      "PSCD", PSCx_FMT, "TimeUS,TPD,PD,DVD,TVD,VD,DAD,TAD,AD", PSCx_UNITS, PSCx_MULTS }, \
    { LOG_RATE_MSG, sizeof(log_Rate), \
        "RATE", "Qfffffffffffff",  "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut,AOutSlew", "skk-kk-kk-oo--", "F?????????BB--" , true }
