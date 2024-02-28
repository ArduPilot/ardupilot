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

#define PSCx_FMT "Qffffffff"
#define PSCx_UNITS "smmnnnooo"
#define PSCx_MULTS "F00000000"

#define LOG_STRUCTURE_FROM_AC_ATTITUDECONTROL        \
    { LOG_PSCN_MSG, sizeof(log_PSCx), \
      "PSCN", PSCx_FMT, "TimeUS,TPN,PN,DVN,TVN,VN,DAN,TAN,AN", PSCx_UNITS, PSCx_MULTS }, \
    { LOG_PSCE_MSG, sizeof(log_PSCx), \
      "PSCE", PSCx_FMT, "TimeUS,TPE,PE,DVE,TVE,VE,DAE,TAE,AE", PSCx_UNITS, PSCx_MULTS }, \
    { LOG_PSCD_MSG, sizeof(log_PSCx), \
      "PSCD", PSCx_FMT, "TimeUS,TPD,PD,DVD,TVD,VD,DAD,TAD,AD", PSCx_UNITS, PSCx_MULTS }
