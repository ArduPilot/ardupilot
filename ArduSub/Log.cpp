#include "Sub.h"

#if HAL_LOGGING_ENABLED

// Code to Write and Read packets from AP_Logger log memory
// Code to interact with the user to dump or erase logs

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
#if AP_TERRAIN_AVAILABLE
    if (terrain.enabled()) {
        terrain.height_above_terrain(terr_alt, true);
    } else {
        terr_alt = rangefinder_state.rangefinder_terrain_offset_cm * 0.01f;
    }
#else
    terr_alt = rangefinder_state.rangefinder_terrain_offset_cm * 0.01f;
#endif

    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control.get_throttle_in(),
        angle_boost         : attitude_control.angle_boost(),
        throttle_out        : motors.get_throttle(),
        throttle_hover      : motors.get_throttle_hover(),
        desired_alt         : pos_control.get_pos_target_z_cm() / 100.0f,
        inav_alt            : inertial_nav.get_position_z_up_cm() * 0.01f,
        baro_alt            : barometer.get_altitude(),
        desired_rangefinder_alt   : (int16_t)mode_surftrak.get_rangefinder_target_cm(),
        rangefinder_alt           : rangefinder_state.alt_cm,
        terr_alt            : terr_alt,
        target_climb_rate   : (int16_t)pos_control.get_vel_target_z_cms(),
        climb_rate          : climb_rate
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void Sub::Log_Write_Attitude()
{
    Vector3f targets = attitude_control.get_att_target_euler_cd();
    targets.z = wrap_360_cd(targets.z);
    ahrs.Write_Attitude(targets);

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
void Sub::Log_Write_Data(LogDataID id, int16_t value)
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
void Sub::Log_Write_Data(LogDataID id, uint16_t value)
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
void Sub::Log_Write_Data(LogDataID id, int32_t value)
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
void Sub::Log_Write_Data(LogDataID id, uint32_t value)
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
void Sub::Log_Write_Data(LogDataID id, float value)
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
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_SensorData {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t baro_instance;
    float temperature_c;
    float pressure_pa;
    float depth_m;
    float rangefinder_alt_m;
    float inertial_alt_m;
    int8_t rangefinder_quality_pct;
    uint8_t depth_healthy;
    uint8_t rangefinder_healthy;
    uint8_t depth_sensor_present;
};

void Sub::Log_Write_SensorData()
{
    const uint8_t baro_instance = ap.depth_sensor_present ? depth_sensor_idx : barometer.get_primary();

    int8_t rangefinder_quality_pct = -1;
#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder_state.enabled) {
        rangefinder_quality_pct = rangefinder.signal_quality_pct_orient(ROTATION_PITCH_270);
    }
#endif

    const struct log_SensorData pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SENSOR_DATA_MSG),
        time_us                : AP_HAL::micros64(),
        baro_instance          : baro_instance,
        temperature_c          : barometer.get_temperature(baro_instance),
        pressure_pa            : barometer.get_pressure(baro_instance),
        depth_m                : barometer.get_altitude(baro_instance),
        rangefinder_alt_m      : rangefinder_state.alt_cm * 0.01f,
        inertial_alt_m         : inertial_nav.get_position_z_up_cm() * 0.01f,
        rangefinder_quality_pct: rangefinder_quality_pct,
        depth_healthy          : ap.depth_sensor_present ? (uint8_t)sensor_health.depth : (uint8_t)barometer.healthy(baro_instance),
        rangefinder_healthy    : (uint8_t)rangefinder_state.alt_healthy,
        depth_sensor_present   : (uint8_t)ap.depth_sensor_present
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_ROVData {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float vel_x_m;
    float vel_y_m;
    float vel_z_m;
    float pos_x_m;
    float pos_y_m;
    float pos_z_m;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
};

void Sub::Log_Write_ROVData()
{
    const struct log_ROVData pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ROV_DATA_MSG),
        time_us  : AP_HAL::micros64(),
        vel_x_m  : inertial_nav.get_velocity_neu_cms().x * 0.01f,
        vel_y_m  : inertial_nav.get_velocity_neu_cms().y * 0.01f,
        vel_z_m  : inertial_nav.get_velocity_neu_cms().z * 0.01f,
        pos_x_m  : inertial_nav.get_position_neu_cm().x * 0.01f,
        pos_y_m  : inertial_nav.get_position_neu_cm().y * 0.01f,
        pos_z_m  : inertial_nav.get_position_neu_cm().z * 0.01f,
        roll_deg : ahrs.roll_sensor * 0.01f,
        pitch_deg: ahrs.pitch_sensor * 0.01f,
        yaw_deg  : ahrs.yaw_sensor * 0.01f
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_DVLData {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t dvl_ok;
    float vx_mps;
    float vy_mps;
    float vz_mps;
    float dvl_quality;
    uint8_t dvl_lock;
    uint32_t dvl_time_ms;
    uint8_t ua_ok;
    uint8_t ub_ok;
    uint8_t uc_ok;
    uint8_t ud_ok;
    float ua_m;
    float ub_m;
    float uc_m;
    float ud_m;
};

void Sub::Log_Write_DVLData()
{
    Vector3f vel_body_mps {};
    uint32_t dvl_t_ms = 0;
    float dvl_quality = 0.0f;
    DVL_LockState dvl_lock = DVL_LockState::NO_LOCK;
    const bool dvl_ok = inertial_doppler.get_velocity_body(vel_body_mps, dvl_t_ms, dvl_quality, dvl_lock);

    DVL_U_Msg ua{}, ub{}, uc{}, ud{};
    const bool ua_ok = inertial_doppler.get_ua_msg(ua);
    const bool ub_ok = inertial_doppler.get_ub_msg(ub);
    const bool uc_ok = inertial_doppler.get_uc_msg(uc);
    const bool ud_ok = inertial_doppler.get_ud_msg(ud);

    const struct log_DVLData pkt = {
        LOG_PACKET_HEADER_INIT(LOG_DVL_DATA_MSG),
        time_us     : AP_HAL::micros64(),
        dvl_ok      : (uint8_t)dvl_ok,
        vx_mps      : vel_body_mps.x,
        vy_mps      : vel_body_mps.y,
        vz_mps      : vel_body_mps.z,
        dvl_quality : dvl_quality,
        dvl_lock    : (uint8_t)dvl_lock,
        dvl_time_ms : dvl_t_ms,
        ua_ok       : (uint8_t)ua_ok,
        ub_ok       : (uint8_t)ub_ok,
        uc_ok       : (uint8_t)uc_ok,
        ud_ok       : (uint8_t)ud_ok,
        ua_m        : ua_ok ? ua.distance_m : 0.0f,
        ub_m        : ub_ok ? ub.distance_m : 0.0f,
        uc_m        : uc_ok ? uc.distance_m : 0.0f,
        ud_m        : ud_ok ? ud.distance_m : 0.0f
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

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

// @LoggerMessage: DU16
// @Description: Generic 16-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU32
// @Description: Generic 32-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: GUIP
// @Description: Guided mode target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: pX: Target position, X-Axis
// @Field: pY: Target position, Y-Axis
// @Field: pZ: Target position, Z-Axis
// @Field: vX: Target velocity, X-Axis
// @Field: vY: Target velocity, Y-Axis
// @Field: vZ: Target velocity, Z-Axis

// @LoggerMessage: SENS
// @Description: Depth and range-related sensor values
// @Field: TimeUS: Time since system startup
// @Field: BI: Barometer instance used for logging
// @Field: Temp: Temperature in degrees Celsius
// @Field: Press: Pressure in pascals
// @Field: Depth: Depth sensor altitude/depth estimate in meters
// @Field: RFAlt: Rangefinder altitude in meters
// @Field: IAlt: Inertial altitude in meters
// @Field: RFQ: Rangefinder signal quality in percent, -1 if unavailable
// @Field: DH: Depth sensor healthy flag
// @Field: RH: Rangefinder healthy flag
// @Field: DSP: External depth sensor present flag

// @LoggerMessage: ROVS
// @Description: Fused vehicle position, velocity and attitude values
// @Field: TimeUS: Time since system startup
// @Field: VX: X velocity in meters per second, NEU frame
// @Field: VY: Y velocity in meters per second, NEU frame
// @Field: VZ: Z velocity in meters per second, NEU frame
// @Field: PX: X position in meters, NEU frame
// @Field: PY: Y position in meters, NEU frame
// @Field: PZ: Z position in meters, NEU frame
// @Field: Roll: Roll angle in degrees
// @Field: Pitch: Pitch angle in degrees
// @Field: Yaw: Yaw angle in degrees

// @LoggerMessage: DVL
// @Description: DVL status, body-frame velocity and beam distances
// @Field: TimeUS: Time since system startup
// @Field: OK: True when DVL body velocity is valid
// @Field: VX: Body X velocity in meters per second
// @Field: VY: Body Y velocity in meters per second
// @Field: VZ: Body Z velocity in meters per second
// @Field: Q: DVL quality metric
// @Field: Lock: DVL lock status
// @Field: DVLT: DVL sample time in milliseconds
// @Field: AO: Beam A distance valid flag
// @Field: BO: Beam B distance valid flag
// @Field: CO: Beam C distance valid flag
// @Field: DO: Beam D distance valid flag
// @Field: UA: Beam A distance in meters
// @Field: UB: Beam B distance in meters
// @Field: UC: Beam C distance in meters
// @Field: UD: Beam D distance in meters

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Sub::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qfffffffccfhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00BBBBBB" },
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
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUIP",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ", "s-mmmnnn", "F-000000" },
    { LOG_SENSOR_DATA_MSG, sizeof(log_SensorData),
      "SENS",  "QBfffffbBBB", "TimeUS,BI,Temp,Press,Depth,RFAlt,IAlt,RFQ,DH,RH,DSP", "s#---------", "F----------" },
    { LOG_ROV_DATA_MSG, sizeof(log_ROVData),
      "ROVS",  "Qfffffffff", "TimeUS,VX,VY,VZ,PX,PY,PZ,Roll,Pitch,Yaw", "s---------", "F---------" },
    { LOG_DVL_DATA_MSG, sizeof(log_DVLData),
      "DVL",   "QBffffBIBBBBffff", "TimeUS,OK,VX,VY,VZ,Q,Lock,DVLT,AO,BO,CO,DO,UA,UB,UC,UD", "s---------------", "F---------------" },
};

void Sub::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    logger.Write_Mode((uint8_t)control_mode, control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}


void Sub::log_init()
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}

#endif // HAL_LOGGING_ENABLED