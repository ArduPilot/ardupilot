#include "Sub.h"
#include "version.h"

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

void Sub::do_erase_logs(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Erasing logs");
    DataFlash.EraseAll();
    gcs().send_text(MAV_SEVERITY_INFO, "Log erase complete");
}

// Write a Current data packet
void Sub::Log_Write_Current()
{
    DataFlash.Log_Write_Current(battery);

    // also write power status
    DataFlash.Log_Write_Power();
}

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
void Sub::Log_Write_Optflow()
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

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    desired_pos_x;
    float    desired_pos_y;
    float    pos_x;
    float    pos_y;
    float    desired_vel_x;
    float    desired_vel_y;
    float    vel_x;
    float    vel_y;
    float    desired_accel_x;
    float    desired_accel_y;
};

// Write an Nav Tuning packet
void Sub::Log_Write_Nav_Tuning()
{
    const Vector3f &pos_target = pos_control.get_pos_target();
    const Vector3f &vel_target = pos_control.get_vel_target();
    const Vector3f &accel_target = pos_control.get_accel_target();
    const Vector3f &position = inertial_nav.get_position();
    const Vector3f &velocity = inertial_nav.get_velocity();

    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        time_us         : AP_HAL::micros64(),
        desired_pos_x   : pos_target.x,
        desired_pos_y   : pos_target.y,
        pos_x           : position.x,
        pos_y           : position.y,
        desired_vel_x   : vel_target.x,
        desired_vel_y   : vel_target.y,
        vel_x           : velocity.x,
        vel_y           : velocity.y,
        desired_accel_x : accel_target.x,
        desired_accel_y : accel_target.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float  throttle_in;
    float  angle_boost;
    float    throttle_out;
    float    throttle_hover;
    float    desired_alt;
    float    inav_alt;
    float    baro_alt;
    int16_t  desired_rangefinder_alt;
    int16_t  rangefinder_alt;
    float    terr_alt;
    int16_t  target_climb_rate;
    int16_t  climb_rate;
};

// Write a control tuning packet
void Sub::Log_Write_Control_Tuning()
{
    // get terrain altitude
    float terr_alt = 0.0f;
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    terrain.height_above_terrain(terr_alt, true);
#endif

    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control.get_throttle_in(),
        angle_boost         : attitude_control.angle_boost(),
        throttle_out        : motors.get_throttle(),
        throttle_hover      : motors.get_throttle_hover(),
        desired_alt         : pos_control.get_alt_target() / 100.0f,
        inav_alt            : inertial_nav.get_altitude() / 100.0f,
        baro_alt            : barometer.get_altitude(),
        desired_rangefinder_alt   : (int16_t)target_rangefinder_alt,
        rangefinder_alt           : rangefinder_state.alt_cm,
        terr_alt            : terr_alt,
        target_climb_rate   : (int16_t)pos_control.get_vel_target_z(),
        climb_rate          : climb_rate
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    int16_t  pm_test;
    uint8_t i2c_lockup_count;
    uint16_t ins_error_count;
    uint32_t log_dropped;
    uint32_t mem_avail;
};

// Write a performance monitoring packet
void Sub::Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        time_us          : AP_HAL::micros64(),
        num_long_running : perf_info_get_num_long_running(),
        num_loops        : perf_info_get_num_loops(),
        max_time         : perf_info_get_max_time(),
        pm_test          : pmTest1,
        i2c_lockup_count : 0,
        ins_error_count  : ins.error_count(),
        log_dropped      : DataFlash.num_dropped() - perf_info_get_num_dropped(),
        hal.util->available_memory()
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void Sub::Log_Write_Attitude()
{
    Vector3f targets = attitude_control.get_att_target_euler_cd();
    targets.z = wrap_360_cd(targets.z);
    DataFlash.Log_Write_Attitude(ahrs, targets);

#if OPTFLOW == ENABLED
    DataFlash.Log_Write_EKF(ahrs,optflow.enabled());
#else
    DataFlash.Log_Write_EKF(ahrs,false);
#endif
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
void Sub::Log_Write_MotBatt()
{
    struct log_MotBatt pkt_mot = {
        LOG_PACKET_HEADER_INIT(LOG_MOTBATT_MSG),
        time_us         : AP_HAL::micros64(),
        lift_max        : (float)(motors.get_lift_max()),
        bat_volt        : (float)(motors.get_batt_voltage_filt()),
        bat_res         : (float)(motors.get_batt_resistance()),
        th_limit        : (float)(motors.get_throttle_limit())
    };
    DataFlash.WriteBlock(&pkt_mot, sizeof(pkt_mot));
}

struct PACKED log_Event {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
};

// Wrote an event packet
void Sub::Log_Write_Event(uint8_t id)
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
void Sub::Log_Write_Data(uint8_t id, int16_t value)
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
void Sub::Log_Write_Data(uint8_t id, uint16_t value)
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
void Sub::Log_Write_Data(uint8_t id, int32_t value)
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
void Sub::Log_Write_Data(uint8_t id, uint32_t value)
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
void Sub::Log_Write_Data(uint8_t id, float value)
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
void Sub::Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        time_us       : AP_HAL::micros64(),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}

void Sub::Log_Write_Baro(void)
{
    if (!ahrs.have_ekf_logging()) {
        DataFlash.Log_Write_Baro(barometer);
    }
}

// log EKF origin and ahrs home to dataflash
void Sub::Log_Write_Home_And_Origin()
{
    // log ekf origin if set
    Location ekf_orig;
    if (ahrs.get_origin(ekf_orig)) {
        DataFlash.Log_Write_Origin(LogOriginType::ekf_origin, ekf_orig);
    }

    // log ahrs home if set
    if (ap.home_state != HOME_UNSET) {
        DataFlash.Log_Write_Origin(LogOriginType::ahrs_home, ahrs.get_home());
    }
}

// logs when baro or compass becomes unhealthy
void Sub::Log_Sensor_Health()
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
}

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
void Sub::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target)
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

const struct LogStructure Sub::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),       
      "OF",   "QBffff",   "TimeUS,Qual,flowX,flowY,bodyX,bodyY" },
    { LOG_NAV_TUNING_MSG, sizeof(log_Nav_Tuning),       
      "NTUN", "Qffffffffff", "TimeUS,DPosX,DPosY,PosX,PosY,DVelX,DVelY,VelX,VelY,DAccX,DAccY" },
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qfffffffccfhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt" },
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "QHHIhBHII",    "TimeUS,NLon,NLoop,MaxT,PMT,I2CErr,INSErr,LogDrop,Mem" },
    { LOG_MOTBATT_MSG, sizeof(log_MotBatt),
      "MOTB", "Qffff",  "TimeUS,LiftMax,BatVolt,BatRes,ThLimit" },
    { LOG_EVENT_MSG, sizeof(log_Event),         
      "EV",   "QB",           "TimeUS,Id" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "QBh",         "TimeUS,Id,Value" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "QBH",         "TimeUS,Id,Value" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "QBi",         "TimeUS,Id,Value" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "QBI",         "TimeUS,Id,Value" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "QBf",         "TimeUS,Id,Value" },
    { LOG_ERROR_MSG, sizeof(log_Error),         
      "ERR",   "QBB",         "TimeUS,Subsys,ECode" },
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUID",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ" },
};

void Sub::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by DataFlash
    DataFlash.Log_Write_Mode(control_mode, control_mode_reason);
    Log_Write_Home_And_Origin();
    gps.Write_DataFlash_Log_Startup_messages();
}


void Sub::start_logging()
{
    if (g.log_bitmask == 0) {
        return;
    }
    if (DataFlash.in_log_download()) {
        return;
    }

    ap.logging_started = true;

    // dataflash may have stopped logging - when we get_log_data,
    // for example.  Always try to restart:
    DataFlash.StartUnstartedLogging();
}

void Sub::log_init(void)
{
    DataFlash.Init(log_structure, ARRAY_SIZE(log_structure));

    gcs().reset_cli_timeout();
}

#else // LOGGING_ENABLED

void Sub::do_erase_logs(void) {}
void Sub::Log_Write_Current() {}
void Sub::Log_Write_Nav_Tuning() {}
void Sub::Log_Write_Control_Tuning() {}
void Sub::Log_Write_Performance() {}
void Sub::Log_Write_Attitude(void) {}
void Sub::Log_Write_MotBatt() {}
void Sub::Log_Write_Startup() {}
void Sub::Log_Write_Event(uint8_t id) {}
void Sub::Log_Write_Data(uint8_t id, int32_t value) {}
void Sub::Log_Write_Data(uint8_t id, uint32_t value) {}
void Sub::Log_Write_Data(uint8_t id, int16_t value) {}
void Sub::Log_Write_Data(uint8_t id, uint16_t value) {}
void Sub::Log_Write_Data(uint8_t id, float value) {}
void Sub::Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
void Sub::Log_Write_Baro(void) {}
void Sub::Log_Write_Home_And_Origin() {}
void Sub::Log_Sensor_Health() {}
void Sub::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target) {}

#if OPTFLOW == ENABLED
void Sub::Log_Write_Optflow() {}
#endif

void Sub::start_logging() {}
void Sub::log_init(void) {}

#endif // LOGGING_ENABLED
