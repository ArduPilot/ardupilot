#include "Copter.h"

#if LOGGING_ENABLED == ENABLED

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
    int32_t  baro_alt;
    float    desired_rangefinder_alt;
    float    rangefinder_alt;
    float    terr_alt;
    int16_t  target_climb_rate;
    int16_t  climb_rate;
    float    dynamic_notch_freq;
};

// Write a control tuning packet
void Copter::Log_Write_Control_Tuning()
{
    // get terrain altitude
    float terr_alt = 0.0f;
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    if (!terrain.height_above_terrain(terr_alt, true)) {
        terr_alt = logger.quiet_nan();
    }
#endif
    float des_alt_m = 0.0f;
    int16_t target_climb_rate_cms = 0;
    if (!flightmode->has_manual_throttle()) {
        des_alt_m = pos_control->get_alt_target() / 100.0f;
        target_climb_rate_cms = pos_control->get_vel_target_z();
    }

    // get surface tracking alts
    float desired_rangefinder_alt;
    if (!surface_tracking.get_target_dist_for_logging(desired_rangefinder_alt)) {
        desired_rangefinder_alt = AP::logger().quiet_nan();
    }

    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control->get_throttle_in(),
        angle_boost         : attitude_control->angle_boost(),
        throttle_out        : motors->get_throttle(),
        throttle_hover      : motors->get_throttle_hover(),
        desired_alt         : des_alt_m,
        inav_alt            : inertial_nav.get_altitude() / 100.0f,
        baro_alt            : baro_alt,
        desired_rangefinder_alt : desired_rangefinder_alt,
        rangefinder_alt     : surface_tracking.get_dist_for_logging(),
        terr_alt            : terr_alt,
        target_climb_rate   : target_climb_rate_cms,
        climb_rate          : int16_t(inertial_nav.get_velocity_z()), // float -> int16_t
        dynamic_notch_freq  : ins.get_gyro_dynamic_notch_center_freq_hz()
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void Copter::Log_Write_Attitude()
{
    Vector3f targets = attitude_control->get_att_target_euler_cd();
    targets.z = wrap_360_cd(targets.z);
    logger.Write_Attitude(targets);
    logger.Write_Rate(ahrs_view, *motors, *attitude_control, *pos_control);
    if (should_log(MASK_LOG_PID)) {
        logger.Write_PID(LOG_PIDR_MSG, attitude_control->get_rate_roll_pid().get_pid_info());
        logger.Write_PID(LOG_PIDP_MSG, attitude_control->get_rate_pitch_pid().get_pid_info());
        logger.Write_PID(LOG_PIDY_MSG, attitude_control->get_rate_yaw_pid().get_pid_info());
        logger.Write_PID(LOG_PIDA_MSG, pos_control->get_accel_z_pid().get_pid_info() );
    }
}

// Write an EKF and POS packet
void Copter::Log_Write_EKF_POS()
{
    AP::ahrs_navekf().Log_Write();
    logger.Write_AHRS2();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE();
#endif
    logger.Write_POS();
}

struct PACKED log_MotBatt {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   lift_max;
    float   bat_volt;
    float   bat_res;
    float   th_limit;
};

// Write an rate packet
void Copter::Log_Write_MotBatt()
{
#if FRAME_CONFIG != HELI_FRAME
    struct log_MotBatt pkt_mot = {
        LOG_PACKET_HEADER_INIT(LOG_MOTBATT_MSG),
        time_us         : AP_HAL::micros64(),
        lift_max        : (float)(motors->get_lift_max()),
        bat_volt        : (float)(motors->get_batt_voltage_filt()),
        bat_res         : (float)(battery.get_resistance()),
        th_limit        : (float)(motors->get_throttle_limit())
    };
    logger.WriteBlock(&pkt_mot, sizeof(pkt_mot));
#endif
}

// Wrote an event packet
void Copter::Log_Write_Event(Log_Event id)
{
    logger.Write_Event(id);
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
UNUSED_FUNCTION
void Copter::Log_Write_Data(uint8_t id, int16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : id,
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
void Copter::Log_Write_Data(uint8_t id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : id,
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
void Copter::Log_Write_Data(uint8_t id, int32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            time_us  : AP_HAL::micros64(),
            id          : id,
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
void Copter::Log_Write_Data(uint8_t id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            time_us     : AP_HAL::micros64(),
            id          : id,
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
void Copter::Log_Write_Data(uint8_t id, float value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            time_us     : AP_HAL::micros64(),
            id          : id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_ParameterTuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  parameter;     // parameter we are tuning, e.g. 39 is CH6_CIRCLE_RATE
    float    tuning_value;  // normalized value used inside tuning() function
    float    tuning_min;    // tuning minimum value
    float    tuning_max;    // tuning maximum value
};

void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max)
{
    struct log_ParameterTuning pkt_tune = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG),
        time_us        : AP_HAL::micros64(),
        parameter      : param,
        tuning_value   : tuning_val,
        tuning_min     : tune_min,
        tuning_max     : tune_max
    };

    logger.WriteBlock(&pkt_tune, sizeof(pkt_tune));
}

// logs when baro or compass becomes unhealthy
void Copter::Log_Sensor_Health()
{
    // check baro
    if (sensor_health.baro != barometer.healthy()) {
        sensor_health.baro = barometer.healthy();
        AP::logger().Write_Error(LogErrorSubsystem::BARO,
                                 (sensor_health.baro ? LogErrorCode::ERROR_RESOLVED : LogErrorCode::UNHEALTHY));
    }

    // check compass
    if (sensor_health.compass != compass.healthy()) {
        sensor_health.compass = compass.healthy();
        AP::logger().Write_Error(LogErrorSubsystem::COMPASS, (sensor_health.compass ? LogErrorCode::ERROR_RESOLVED : LogErrorCode::UNHEALTHY));
    }

    // check primary GPS
    if (sensor_health.primary_gps != gps.primary_sensor()) {
        sensor_health.primary_gps = gps.primary_sensor();
        Log_Write_Event(DATA_GPS_PRIMARY_CHANGED);
    }
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
#if MODE_SYSTEMID_ENABLED == ENABLED
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
#if MODE_SYSTEMID_ENABLED == ENABLED
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

struct PACKED log_Heli {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    desired_rotor_speed;
    float    main_rotor_speed;
    float    governor_output;
    float    control_output;
};

#if FRAME_CONFIG == HELI_FRAME
// Write an helicopter packet
void Copter::Log_Write_Heli()
{
    struct log_Heli pkt_heli = {
        LOG_PACKET_HEADER_INIT(LOG_HELI_MSG),
        time_us                 : AP_HAL::micros64(),
        desired_rotor_speed     : motors->get_desired_rotor_speed(),
        main_rotor_speed        : motors->get_main_rotor_speed(),
        governor_output         : motors->get_governor_output(),
        control_output          : motors->get_control_output(),
    };
    logger.WriteBlock(&pkt_heli, sizeof(pkt_heli));
}
#endif

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

// Write a precision landing entry
void Copter::Log_Write_Precland()
{
 #if PRECISION_LANDING == ENABLED
    // exit immediately if not enabled
    if (!precland.enabled()) {
        return;
    }

    Vector3f target_pos_meas = Vector3f(0.0f,0.0f,0.0f);
    Vector2f target_pos_rel = Vector2f(0.0f,0.0f);
    Vector2f target_vel_rel = Vector2f(0.0f,0.0f);
    precland.get_target_position_relative_cm(target_pos_rel);
    precland.get_target_velocity_relative_cms(target_vel_rel);
    precland.get_target_position_measurement_cm(target_pos_meas);

    struct log_Precland pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PRECLAND_MSG),
        time_us         : AP_HAL::micros64(),
        healthy         : precland.healthy(),
        target_acquired : precland.target_acquired(),
        pos_x           : target_pos_rel.x,
        pos_y           : target_pos_rel.y,
        vel_x           : target_vel_rel.x,
        vel_y           : target_vel_rel.y,
        meas_x          : target_pos_meas.x,
        meas_y          : target_pos_meas.y,
        meas_z          : target_pos_meas.z,
        last_meas       : precland.last_backend_los_meas_ms(),
        ekf_outcount    : precland.ekf_outlier_count(),
        estimator       : precland.estimator_type()
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
 #endif     // PRECISION_LANDING == ENABLED
}

// guided target logging
struct PACKED log_GuidedTarget {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float pos_target_x;
    float pos_target_y;
    float pos_target_z;
    float vel_target_x;
    float vel_target_y;
    float vel_target_z;
};

// Write a Guided mode target
void Copter::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target)
{
    struct log_GuidedTarget pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GUIDEDTARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : target_type,
        pos_target_x    : pos_target.x,
        pos_target_y    : pos_target.y,
        pos_target_z    : pos_target.z,
        vel_target_x    : vel_target.x,
        vel_target_y    : vel_target.y,
        vel_target_z    : vel_target.z
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Copter::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_PARAMTUNE_MSG, sizeof(log_ParameterTuning),
      "PTUN", "QBfff",         "TimeUS,Param,TunVal,TunMin,TunMax", "s----", "F----" },
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qffffffefffhhf", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt,N", "s----mmmmmmnnz", "F----00B000BB-" },
    { LOG_MOTBATT_MSG, sizeof(log_MotBatt),
      "MOTB", "Qffff",  "TimeUS,LiftMax,BatVolt,BatRes,ThLimit", "s-vw-", "F-00-" },
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
#if FRAME_CONFIG == HELI_FRAME
    { LOG_HELI_MSG, sizeof(log_Heli),
      "HELI",  "Qffff",        "TimeUS,DRRPM,ERRPM,Gov,Throt", "s----", "F----" },
#endif
#if PRECISION_LANDING == ENABLED
    { LOG_PRECLAND_MSG, sizeof(log_Precland),
      "PL",    "QBBfffffffIIB",    "TimeUS,Heal,TAcq,pX,pY,vX,vY,mX,mY,mZ,LastMeasMS,EKFOutl,Est", "s--mmnnmmms--","F--BBBBBBBC--" },
#endif
    { LOG_SYSIDD_MSG, sizeof(log_SysIdD),
      "SIDD", "Qfffffffff",  "TimeUS,Time,Targ,F,Gx,Gy,Gz,Ax,Ay,Az", "ss-zkkkooo", "F---------" },
    { LOG_SYSIDS_MSG, sizeof(log_SysIdS),
      "SIDS", "QBfffffff",  "TimeUS,Ax,Mag,FSt,FSp,TFin,TC,TR,TFout", "s--ssssss", "F--------" },
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUID",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ", "s-mmmnnn", "F-000000" },
};

void Copter::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    logger.Write_MessageF("Frame: %s", get_frame_string());
    logger.Write_Mode((uint8_t)control_mode, control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

void Copter::log_init(void)
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else // LOGGING_ENABLED

void Copter::Log_Write_Control_Tuning() {}
void Copter::Log_Write_Performance() {}
void Copter::Log_Write_Attitude(void) {}
void Copter::Log_Write_EKF_POS() {}
void Copter::Log_Write_MotBatt() {}
void Copter::Log_Write_Event(Log_Event id) {}
void Copter::Log_Write_Data(uint8_t id, int32_t value) {}
void Copter::Log_Write_Data(uint8_t id, uint32_t value) {}
void Copter::Log_Write_Data(uint8_t id, int16_t value) {}
void Copter::Log_Write_Data(uint8_t id, uint16_t value) {}
void Copter::Log_Write_Data(uint8_t id, float value) {}
void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max) {}
void Copter::Log_Sensor_Health() {}
void Copter::Log_Write_Precland() {}
void Copter::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target) {}
void Copter::Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out) {}
void Copter::Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z) {}
void Copter::Log_Write_Vehicle_Startup_Messages() {}

#if FRAME_CONFIG == HELI_FRAME
void Copter::Log_Write_Heli() {}
#endif

void Copter::log_init(void) {}

#endif // LOGGING_ENABLED
