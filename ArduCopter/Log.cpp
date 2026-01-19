#include "Copter.h"
#include <AP_InertialSensor/AP_InertialSensor_rate_config.h>

#if HAL_LOGGING_ENABLED

// Code to Write and Read packets from AP_Logger log memory
// Code to interact with the user to dump or erase logs

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    throttle_in;
    float    angle_boost;
    float    throttle_out;
    float    throttle_hover;
    float    desired_alt;
    float    inav_alt;
    float    baro_alt;
    float    desired_rangefinder_alt;
    float    rangefinder_alt;
    float    terr_alt;
    int16_t  target_climb_rate;
    int16_t  climb_rate;
};

// Write a control tuning packet
void Copter::Log_Write_Control_Tuning()
{
    // get terrain altitude
    float terr_alt = 0.0f;
#if AP_TERRAIN_AVAILABLE
    if (!terrain.height_above_terrain(terr_alt, true)) {
        terr_alt = AP_Logger::quiet_nanf();
    }
#endif
    float des_alt_m = 0.0f;
    float target_climb_rate_ms = 0;
    if (!flightmode->has_manual_throttle()) {
        des_alt_m = pos_control->get_pos_target_U_m();
        target_climb_rate_ms = pos_control->get_vel_target_U_ms();
    }

    float desired_rangefinder_alt_m;
#if AP_RANGEFINDER_ENABLED
    if (!surface_tracking.get_target_dist_for_logging(desired_rangefinder_alt_m)) {
        desired_rangefinder_alt_m = AP_Logger::quiet_nanf();
    }
#else
    // get surface tracking alts
    desired_rangefinder_alt_m = AP_Logger::quiet_nanf();
#endif

    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control->get_throttle_in(),
        angle_boost         : attitude_control->angle_boost(),
        throttle_out        : motors->get_throttle(),
        throttle_hover      : motors->get_throttle_hover(),
        desired_alt         : des_alt_m,
        inav_alt            : float(pos_control->get_pos_estimate_U_m()),
        baro_alt            : baro_alt_m,
        desired_rangefinder_alt : desired_rangefinder_alt_m,
#if AP_RANGEFINDER_ENABLED
        rangefinder_alt     : surface_tracking.get_dist_for_logging(),
#else
        rangefinder_alt     : AP_Logger::quiet_nanf(),
#endif
        terr_alt            : terr_alt,
        target_climb_rate   : int16_t(target_climb_rate_ms * 100.0),
        climb_rate          : int16_t(pos_control->get_vel_estimate_U_ms() * 100.0) // float -> int16_t
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void Copter::Log_Write_Attitude()
{
    attitude_control->Write_ANG();
}

void Copter::Log_Write_Rate()
{
    attitude_control->Write_Rate(*pos_control);
}

// Write PIDS packets
void Copter::Log_Write_PIDS()
{
   if (should_log(MASK_LOG_PID)) {
        logger.Write_PID(LOG_PIDR_MSG, attitude_control->get_rate_roll_pid().get_pid_info());
        logger.Write_PID(LOG_PIDP_MSG, attitude_control->get_rate_pitch_pid().get_pid_info());
        logger.Write_PID(LOG_PIDY_MSG, attitude_control->get_rate_yaw_pid().get_pid_info());
        logger.Write_PID(LOG_PIDA_MSG, pos_control->D_get_accel_pid().get_pid_info() );
        if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS())) {
            logger.Write_PID(LOG_PIDN_MSG, pos_control->NE_get_vel_pid().get_pid_info_x());
            logger.Write_PID(LOG_PIDE_MSG, pos_control->NE_get_vel_pid().get_pid_info_y());
        }
    }
}

// Write an EKF and POS packet
void Copter::Log_Write_EKF_POS()
{
    AP::ahrs().Log_Write();
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
UNUSED_FUNCTION
void Copter::Log_Write_Data(LogDataID id, int16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
UNUSED_FUNCTION 
void Copter::Log_Write_Data(LogDataID id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
void Copter::Log_Write_Data(LogDataID id, int32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            time_us  : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
void Copter::Log_Write_Data(LogDataID id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    float data_value;
};

// Write a float data packet
UNUSED_FUNCTION
void Copter::Log_Write_Data(LogDataID id, float value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_PTUN {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  parameter;     // parameter we are tuning, e.g. 39 is CH6_CIRCLE_RATE
    float    tuning_value;  // normalized value used inside tuning() function
    float    tuning_min;    // tuning minimum value
    float    tuning_max;    // tuning maximum value
    float    norm_in;       // normalized control input (-1 to 1)
};

void Copter::Log_Write_PTUN(uint8_t param, float tuning_val, float tune_min, float tune_max, float norm_in)
{
    const struct log_PTUN pkt_tune {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG),
        time_us        : AP_HAL::micros64(),
        parameter      : param,
        tuning_value   : tuning_val,
        tuning_min     : tune_min,
        tuning_max     : tune_max,
        norm_in        : norm_in
    };

    logger.WriteBlock(&pkt_tune, sizeof(pkt_tune));
}

void Copter::Log_Video_Stabilisation()
{
    if (!should_log(MASK_LOG_VIDEO_STABILISATION)) {
        return;
    }
    ahrs.write_video_stabilisation();
}

struct PACKED log_SysIdD {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    waveform_time;
    float    waveform_sample;
    float    waveform_freq;
    float    angle_x;
    float    angle_y;
    float    angle_z;
    float    accel_x;
    float    accel_y;
    float    accel_z;
};

// Write an rate packet
void Copter::Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z)
{
#if MODE_SYSTEMID_ENABLED
    struct log_SysIdD pkt_sidd = {
        LOG_PACKET_HEADER_INIT(LOG_SYSIDD_MSG),
        time_us         : AP_HAL::micros64(),
        waveform_time   : waveform_time,
        waveform_sample : waveform_sample,
        waveform_freq   : waveform_freq,
        angle_x         : angle_x,
        angle_y         : angle_y,
        angle_z         : angle_z,
        accel_x         : accel_x,
        accel_y         : accel_y,
        accel_z         : accel_z
    };
    logger.WriteBlock(&pkt_sidd, sizeof(pkt_sidd));
#endif
}

struct PACKED log_SysIdS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  systemID_axis;
    float    waveform_magnitude;
    float    frequency_start;
    float    frequency_stop;
    float    time_fade_in;
    float    time_const_freq;
    float    time_record;
    float    time_fade_out;
};

// Write an rate packet
void Copter::Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out)
{
#if MODE_SYSTEMID_ENABLED
    struct log_SysIdS pkt_sids = {
        LOG_PACKET_HEADER_INIT(LOG_SYSIDS_MSG),
        time_us             : AP_HAL::micros64(),
        systemID_axis       : systemID_axis,
        waveform_magnitude  : waveform_magnitude,
        frequency_start     : frequency_start,
        frequency_stop      : frequency_stop,
        time_fade_in        : time_fade_in,
        time_const_freq     : time_const_freq,
        time_record         : time_record,
        time_fade_out       : time_fade_out
    };
    logger.WriteBlock(&pkt_sids, sizeof(pkt_sids));
#endif
}

// guided position target logging
struct PACKED log_Guided_Position_Target {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float pos_target_x;
    float pos_target_y;
    float pos_target_z;
    uint8_t terrain;
    float vel_target_x;
    float vel_target_y;
    float vel_target_z;
    float accel_target_x;
    float accel_target_y;
    float accel_target_z;
};

// guided attitude target logging
struct PACKED log_Guided_Attitude_Target {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float thrust;
    float climb_rate;
};

// rate thread dt stats
struct PACKED log_Rate_Thread_Dt {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float dt;
    float dtAvg;
    float dtMax;
    float dtMin;
};

// Write a Guided mode position target
// pos_target_ned_m is lat, lon, alt OR offset from ekf origin in m
// terrain should be 0 if pos_target_ned_m.z is alt-above-ekf-origin, 1 if alt-above-terrain
// vel_target_ms is m/s
void Copter::Log_Write_Guided_Position_Target(ModeGuided::SubMode submode, const Vector3p& pos_target_ned_m, bool is_terrain_alt, const Vector3f& vel_target_ms, const Vector3f& accel_target_mss)
{
    const log_Guided_Position_Target pkt {
        LOG_PACKET_HEADER_INIT(LOG_GUIDED_POSITION_TARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : (uint8_t)submode,
        pos_target_x    : (float)pos_target_ned_m.x,
        pos_target_y    : (float)pos_target_ned_m.y,
        pos_target_z    : (float)pos_target_ned_m.z,
        terrain         : is_terrain_alt,
        vel_target_x    : vel_target_ms.x,
        vel_target_y    : vel_target_ms.y,
        vel_target_z    : vel_target_ms.z,
        accel_target_x  : accel_target_mss.x,
        accel_target_y  : accel_target_mss.y,
        accel_target_z  : accel_target_mss.z
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write a Guided mode attitude target
// roll_rad, pitch_rad and yaw_rad are in radians
// ang_vel_rads: angular velocity, [roll rate, pitch_rate, yaw_rate] in radians/sec
// thrust is between 0 to 1
// climb_rate is in (m/s)
void Copter::Log_Write_Guided_Attitude_Target(ModeGuided::SubMode submode, float roll_rad, float pitch_rad, float yaw_rad, const Vector3f &ang_vel_rads, float thrust, float climb_rate_ms)
{
    const log_Guided_Attitude_Target pkt {
        LOG_PACKET_HEADER_INIT(LOG_GUIDED_ATTITUDE_TARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : (uint8_t)submode,
        roll            : degrees(roll_rad),       // rad to deg
        pitch           : degrees(pitch_rad),      // rad to deg
        yaw             : degrees(yaw_rad),        // rad to deg
        roll_rate       : degrees(ang_vel_rads.x),  // rad/s to deg/s
        pitch_rate      : degrees(ang_vel_rads.y),  // rad/s to deg/s
        yaw_rate        : degrees(ang_vel_rads.z),  // rad/s to deg/s
        thrust          : thrust,
        climb_rate      : climb_rate_ms
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_Write_Rate_Thread_Dt(float dt, float dtAvg, float dtMax, float dtMin)
{
#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    const log_Rate_Thread_Dt pkt {
        LOG_PACKET_HEADER_INIT(LOG_RATE_THREAD_DT_MSG),
        time_us         : AP_HAL::micros64(),
        dt              : dt,
        dtAvg           : dtAvg,
        dtMax           : dtMax,
        dtMin           : dtMin
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
#endif
}

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Copter::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    
// @LoggerMessage: PTUN
// @Description: Parameter Tuning information
// @URL: https://ardupilot.org/copter/docs/tuning.html#in-flight-tuning
// @Field: TimeUS: Time since system startup
// @Field: Param: Parameter being tuned
// @Field: TunVal: Normalized value used inside tuning() function
// @Field: TunMin: Tuning minimum limit
// @Field: TunMax: Tuning maximum limit
// @Field: NIn: normalaised control input (normalised -1 to 1 value)

    { LOG_PARAMTUNE_MSG, sizeof(log_PTUN),
      "PTUN", "QBffff",         "TimeUS,Param,TunVal,TunMin,TunMax,NIn", "s#----", "F-----" },

// @LoggerMessage: CTUN
// @Description: Control Tuning information
// @Field: TimeUS: Time since system startup
// @Field: ThI: throttle input
// @Field: ABst: angle boost
// @Field: ThO: throttle output
// @Field: ThH: calculated hover throttle
// @Field: DAlt: desired altitude
// @Field: Alt: achieved altitude
// @Field: BAlt: barometric altitude
// @Field: DSAlt: desired rangefinder altitude
// @Field: SAlt: achieved rangefinder altitude
// @Field: TAlt: terrain altitude
// @Field: DCRt: desired climb rate
// @Field: CRt: climb rate

// @LoggerMessage: D16
// @Description: Generic 16-bit-signed-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU16
// @Description: Generic 16-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: D32
// @Description: Generic 32-bit-signed-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DFLT
// @Description: Generic float storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU32
// @Description: Generic 32-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qffffffffffhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----000000BB" , true },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "QBh",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "QBH",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "QBi",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "QBI",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "QBf",         "TimeUS,Id,Value", "s--", "F--" },

// @LoggerMessage: SIDD
// @Description: System ID data
// @Field: TimeUS: Time since system startup
// @Field: Time: Time reference for waveform
// @Field: Targ: Current waveform sample
// @Field: F: Instantaneous waveform frequency
// @Field: Gx: Delta angle, X-Axis
// @Field: Gy: Delta angle, Y-Axis
// @Field: Gz: Delta angle, Z-Axis
// @Field: Ax: Delta velocity, X-Axis
// @Field: Ay: Delta velocity, Y-Axis
// @Field: Az: Delta velocity, Z-Axis

    { LOG_SYSIDD_MSG, sizeof(log_SysIdD),
      "SIDD", "Qfffffffff",  "TimeUS,Time,Targ,F,Gx,Gy,Gz,Ax,Ay,Az", "ss-zkkkooo", "F---------" , true },

// @LoggerMessage: SIDS
// @Description: System ID settings
// @Field: TimeUS: Time since system startup
// @Field: Ax: The axis which is being excited
// @Field: Mag: Magnitude of the chirp waveform
// @Field: FSt: Frequency at the start of chirp
// @Field: FSp: Frequency at the end of chirp
// @Field: TFin: Time to reach maximum amplitude of chirp
// @Field: TC: Time at constant frequency before chirp starts
// @Field: TR: Time taken to complete chirp waveform
// @Field: TFout: Time to reach zero amplitude after chirp finishes

    { LOG_SYSIDS_MSG, sizeof(log_SysIdS),
      "SIDS", "QBfffffff",  "TimeUS,Ax,Mag,FSt,FSp,TFin,TC,TR,TFout", "s--ssssss", "F--------" , true },

// @LoggerMessage: GUIP
// @Description: Guided mode position target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: pX: Target position, X-Axis
// @Field: pY: Target position, Y-Axis
// @Field: pZ: Target position, Z-Axis
// @Field: Terrain: Target position, Z-Axis is alt above terrain
// @Field: vX: Target velocity, X-Axis
// @Field: vY: Target velocity, Y-Axis
// @Field: vZ: Target velocity, Z-Axis
// @Field: aX: Target acceleration, X-Axis
// @Field: aY: Target acceleration, Y-Axis
// @Field: aZ: Target acceleration, Z-Axis

    { LOG_GUIDED_POSITION_TARGET_MSG, sizeof(log_Guided_Position_Target),
      "GUIP",  "QBfffbffffff",    "TimeUS,Type,pX,pY,pZ,Terrain,vX,vY,vZ,aX,aY,aZ", "s-mmm-nnnooo", "F-000-000000" , true },

// @LoggerMessage: GUIA
// @Description: Guided mode attitude target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: Roll: Target attitude, Roll
// @Field: Pitch: Target attitude, Pitch
// @Field: Yaw: Target attitude, Yaw
// @Field: RollRt: Roll rate
// @Field: PitchRt: Pitch rate
// @Field: YawRt: Yaw rate
// @Field: Thrust: Thrust 
// @Field: ClimbRt: Climb rate

    { LOG_GUIDED_ATTITUDE_TARGET_MSG, sizeof(log_Guided_Attitude_Target),
      "GUIA",  "QBffffffff",    "TimeUS,Type,Roll,Pitch,Yaw,RollRt,PitchRt,YawRt,Thrust,ClimbRt", "s-dddkkk-n", "F-000000-0" , true },

// @LoggerMessage: RTDT
// @Description: Attitude controller time deltas
// @Field: TimeUS: Time since system startup
// @Field: dt: current time delta
// @Field: dtAvg: current time delta average
// @Field: dtMax: Max time delta since last log output
// @Field: dtMin: Min time delta since last log output

    { LOG_RATE_THREAD_DT_MSG, sizeof(log_Rate_Thread_Dt),
      "RTDT", "Qffff", "TimeUS,dt,dtAvg,dtMax,dtMin", "sssss", "F----" , true },

};

uint8_t Copter::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

void Copter::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    char frame_and_type_string[30];
    copter.motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    logger.Write_MessageF("%s", frame_and_type_string);
    logger.Write_Mode((uint8_t)flightmode->mode_number(), control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

#endif // HAL_LOGGING_ENABLED
