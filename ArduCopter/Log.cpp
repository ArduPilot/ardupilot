#include "Copter.h"

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

#if AUTOTUNE_ENABLED == ENABLED
struct PACKED log_AutoTune {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t axis;           // roll or pitch
    uint8_t tune_step;      // tuning PI or D up or down
    float   meas_target;    // target achieved rotation rate
    float   meas_min;       // maximum achieved rotation rate
    float   meas_max;       // maximum achieved rotation rate
    float   new_gain_rp;    // newly calculated gain
    float   new_gain_rd;    // newly calculated gain
    float   new_gain_sp;    // newly calculated gain
    float   new_ddt;        // newly calculated gain
};

// Write an Autotune data packet
void Copter::ModeAutoTune::Log_Write_AutoTune(uint8_t _axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt)
{
    struct log_AutoTune pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNE_MSG),
        time_us     : AP_HAL::micros64(),
        axis        : _axis,
        tune_step   : tune_step,
        meas_target : meas_target,
        meas_min    : meas_min,
        meas_max    : meas_max,
        new_gain_rp : new_gain_rp,
        new_gain_rd : new_gain_rd,
        new_gain_sp : new_gain_sp,
        new_ddt     : new_ddt
    };
    copter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_AutoTuneDetails {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    angle_cd;      // lean angle in centi-degrees
    float    rate_cds;      // current rotation rate in centi-degrees / second
};

// Write an Autotune data packet
void Copter::ModeAutoTune::Log_Write_AutoTuneDetails(float angle_cd, float rate_cds)
{
    struct log_AutoTuneDetails pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNEDETAILS_MSG),
        time_us     : AP_HAL::micros64(),
        angle_cd    : angle_cd,
        rate_cds    : rate_cds
    };
    copter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t surface_quality;
    float flow_x;
    float flow_y;
    float body_x;
    float body_y;
};

// Write an optical flow packet
void Copter::Log_Write_Optflow()
{
 #if OPTFLOW == ENABLED
    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }
    const Vector2f &flowRate = optflow.flowRate();
    const Vector2f &bodyRate = optflow.bodyRate();
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        time_us         : AP_HAL::micros64(),
        surface_quality : optflow.quality(),
        flow_x          : flowRate.x,
        flow_y          : flowRate.y,
        body_x          : bodyRate.x,
        body_y          : bodyRate.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
 #endif     // OPTFLOW == ENABLED
}

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
    int16_t  rangefinder_alt;
    float    terr_alt;
    int16_t  target_climb_rate;
    int16_t  climb_rate;
};

// Write a control tuning packet
void Copter::Log_Write_Control_Tuning()
{
    // get terrain altitude
    float terr_alt = 0.0f;
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    if (!terrain.height_above_terrain(terr_alt, true)) {
        terr_alt = DataFlash.quiet_nan();
    }
#endif

    float _target_rangefinder_alt;
    if (target_rangefinder_alt_used) {
        _target_rangefinder_alt = target_rangefinder_alt * 0.01f; // cm->m
    } else {
        _target_rangefinder_alt = DataFlash.quiet_nan();
    }
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control->get_throttle_in(),
        angle_boost         : attitude_control->angle_boost(),
        throttle_out        : motors->get_throttle(),
        throttle_hover      : motors->get_throttle_hover(),
        desired_alt         : pos_control->get_alt_target() / 100.0f,
        inav_alt            : inertial_nav.get_altitude() / 100.0f,
        baro_alt            : baro_alt,
        desired_rangefinder_alt : _target_rangefinder_alt,
        rangefinder_alt     : rangefinder_state.alt_cm,
        terr_alt            : terr_alt,
        target_climb_rate   : (int16_t)pos_control->get_vel_target_z(),
        climb_rate          : climb_rate
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void Copter::Log_Write_Attitude()
{
    Vector3f targets = attitude_control->get_att_target_euler_cd();
    targets.z = wrap_360_cd(targets.z);
    DataFlash.Log_Write_Attitude(ahrs, targets);
    DataFlash.Log_Write_Rate(ahrs, *motors, *attitude_control, *pos_control);
    if (should_log(MASK_LOG_PID)) {
        DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control->get_rate_roll_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control->get_rate_pitch_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control->get_rate_yaw_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIDA_MSG, pos_control->get_accel_z_pid().get_pid_info() );
    }
}

// Write an EKF and POS packet
void Copter::Log_Write_EKF_POS()
{
    DataFlash.Log_Write_EKF(ahrs);
    DataFlash.Log_Write_AHRS2(ahrs);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE(&DataFlash);
#endif
    DataFlash.Log_Write_POS(ahrs);
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
    DataFlash.WriteBlock(&pkt_mot, sizeof(pkt_mot));
#endif
}

struct PACKED log_Event {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
};

// Wrote an event packet
void Copter::Log_Write_Event(uint8_t id)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Event pkt = {
            LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
            time_us  : AP_HAL::micros64(),
            id       : id
        };
        DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
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
        DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
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
        DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
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
        DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
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
        DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
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
        DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Error {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t sub_system;
    uint8_t error_code;
};

// Write an error packet
void Copter::Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        time_us       : AP_HAL::micros64(),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}

struct PACKED log_ParameterTuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  parameter;     // parameter we are tuning, e.g. 39 is CH6_CIRCLE_RATE
    float    tuning_value;  // normalized value used inside tuning() function
    int16_t  control_in;    // raw tune input value
    int16_t  tuning_low;    // tuning low end value
    int16_t  tuning_high;   // tuning high end value
};

void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, int16_t control_in, int16_t tune_low, int16_t tune_high)
{
    struct log_ParameterTuning pkt_tune = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG),
        time_us        : AP_HAL::micros64(),
        parameter      : param,
        tuning_value   : tuning_val,
        control_in     : control_in,
        tuning_low     : tune_low,
        tuning_high    : tune_high
    };

    DataFlash.WriteBlock(&pkt_tune, sizeof(pkt_tune));
}

// logs when baro or compass becomes unhealthy
void Copter::Log_Sensor_Health()
{
    // check baro
    if (sensor_health.baro != barometer.healthy()) {
        sensor_health.baro = barometer.healthy();
        Log_Write_Error(ERROR_SUBSYSTEM_BARO, (sensor_health.baro ? ERROR_CODE_ERROR_RESOLVED : ERROR_CODE_UNHEALTHY));
    }

    // check compass
    if (sensor_health.compass != compass.healthy()) {
        sensor_health.compass = compass.healthy();
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS, (sensor_health.compass ? ERROR_CODE_ERROR_RESOLVED : ERROR_CODE_UNHEALTHY));
    }

    // check primary GPS
    if (sensor_health.primary_gps != gps.primary_sensor()) {
        sensor_health.primary_gps = gps.primary_sensor();
        Log_Write_Event(DATA_GPS_PRIMARY_CHANGED);
    }
}

struct PACKED log_Heli {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    desired_rotor_speed;
    float    main_rotor_speed;
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
    };
    DataFlash.WriteBlock(&pkt_heli, sizeof(pkt_heli));
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
};

// Write an optical flow packet
void Copter::Log_Write_Precland()
{
 #if PRECISION_LANDING == ENABLED
    // exit immediately if not enabled
    if (!precland.enabled()) {
        return;
    }

    Vector2f target_pos_rel = Vector2f(0.0f,0.0f);
    Vector2f target_vel_rel = Vector2f(0.0f,0.0f);
    precland.get_target_position_relative_cm(target_pos_rel);
    precland.get_target_velocity_relative_cms(target_vel_rel);

    struct log_Precland pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PRECLAND_MSG),
        time_us         : AP_HAL::micros64(),
        healthy         : precland.healthy(),
        target_acquired : precland.target_acquired(),
        pos_x           : target_pos_rel.x,
        pos_y           : target_pos_rel.y,
        vel_x           : target_vel_rel.x,
        vel_y           : target_vel_rel.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
 #endif     // PRECISION_LANDING == ENABLED
}

// precision landing logging
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
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// type and unit information can be found in
// libraries/DataFlash/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Copter::log_structure[] = {
    LOG_COMMON_STRUCTURES,
#if AUTOTUNE_ENABLED == ENABLED
    { LOG_AUTOTUNE_MSG, sizeof(log_AutoTune),
      "ATUN", "QBBfffffff",       "TimeUS,Axis,TuneStep,Targ,Min,Max,RP,RD,SP,ddt", "s--ddd---o", "F--BBB---0" },
    { LOG_AUTOTUNEDETAILS_MSG, sizeof(log_AutoTuneDetails),
      "ATDE", "Qff",          "TimeUS,Angle,Rate", "sdk", "FBB" },
#endif
    { LOG_PARAMTUNE_MSG, sizeof(log_ParameterTuning),
      "PTUN", "QBfHHH",          "TimeUS,Param,TunVal,CtrlIn,TunLo,TunHi", "s-----", "F-----" },
#if OPTFLOW == ENABLED
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),
      "OF",   "QBffff",   "TimeUS,Qual,flowX,flowY,bodyX,bodyY", "s-EEEE", "F-0000" },
#endif
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qffffffefcfhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00B0BBBB" },
    { LOG_MOTBATT_MSG, sizeof(log_MotBatt),
      "MOTB", "Qffff",  "TimeUS,LiftMax,BatVolt,BatRes,ThLimit", "s-vw-", "F-00-" },
    { LOG_EVENT_MSG, sizeof(log_Event),         
      "EV",   "QB",           "TimeUS,Id", "s-", "F-" },
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
    { LOG_ERROR_MSG, sizeof(log_Error),         
      "ERR",   "QBB",         "TimeUS,Subsys,ECode", "s--", "F--" },
#if FRAME_CONFIG == HELI_FRAME
    { LOG_HELI_MSG, sizeof(log_Heli),
      "HELI",  "Qff",         "TimeUS,DRRPM,ERRPM", "s--", "F--" },
#endif
#if PRECISION_LANDING == ENABLED
    { LOG_PRECLAND_MSG, sizeof(log_Precland),
      "PL",    "QBBffff",    "TimeUS,Heal,TAcq,pX,pY,vX,vY", "s--ddmm","F--00BB" },
#endif
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUID",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ", "s-mmmnnn", "F-000000" },
};

void Copter::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by DataFlash
    DataFlash.Log_Write_MessageF("Frame: %s", get_frame_string());
    DataFlash.Log_Write_Mode(control_mode, control_mode_reason);
#if AC_RALLY
    DataFlash.Log_Write_Rally(rally);
#endif
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_DataFlash_Log_Startup_messages();
}


void Copter::log_init(void)
{
    DataFlash.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else // LOGGING_ENABLED

void Copter::Log_Write_Control_Tuning() {}
void Copter::Log_Write_Performance() {}
void Copter::Log_Write_Attitude(void) {}
void Copter::Log_Write_EKF_POS() {}
void Copter::Log_Write_MotBatt() {}
void Copter::Log_Write_Event(uint8_t id) {}
void Copter::Log_Write_Data(uint8_t id, int32_t value) {}
void Copter::Log_Write_Data(uint8_t id, uint32_t value) {}
void Copter::Log_Write_Data(uint8_t id, int16_t value) {}
void Copter::Log_Write_Data(uint8_t id, uint16_t value) {}
void Copter::Log_Write_Data(uint8_t id, float value) {}
void Copter::Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, int16_t control_in, int16_t tune_low, int16_t tune_high) {}
void Copter::Log_Sensor_Health() {}
void Copter::Log_Write_Precland() {}
void Copter::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target) {}
void Copter::Log_Write_Vehicle_Startup_Messages() {}

#if FRAME_CONFIG == HELI_FRAME
void Copter::Log_Write_Heli() {}
#endif

#if OPTFLOW == ENABLED
void Copter::Log_Write_Optflow() {}
#endif

void Copter::log_init(void) {}

#endif // LOGGING_ENABLED
