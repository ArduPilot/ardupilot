#include "Copter.h"
#include "version.h"

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

#if CLI_ENABLED == ENABLED

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
static const struct Menu::command log_menu_commands[] = {
    {"dump",        MENU_FUNC(dump_log)},
    {"erase",       MENU_FUNC(erase_logs)},
    {"enable",      MENU_FUNC(select_logs)},
    {"disable",     MENU_FUNC(select_logs)}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, FUNCTOR_BIND(&copter, &Copter::print_log_menu, bool));

bool Copter::print_log_menu(void)
{
    cliSerial->printf("logs enabled: ");

    if (0 == g.log_bitmask) {
        cliSerial->printf("none");
    }else{
        // Macro to make the following code a bit easier on the eye.
        // Pass it the capitalised name of the log option, as defined
        // in defines.h but without the LOG_ prefix.  It will check for
        // the bit being set and print the name of the log option to suit.
#define PLOG(_s) if (g.log_bitmask & MASK_LOG_ ## _s) cliSerial->printf(" %s", # _s)
        PLOG(ATTITUDE_FAST);
        PLOG(ATTITUDE_MED);
        PLOG(GPS);
        PLOG(PM);
        PLOG(CTUN);
        PLOG(NTUN);
        PLOG(RCIN);
        PLOG(IMU);
        PLOG(CMD);
        PLOG(CURRENT);
        PLOG(RCOUT);
        PLOG(OPTFLOW);
        PLOG(COMPASS);
        PLOG(CAMERA);
        PLOG(PID);
#undef PLOG
    }

    cliSerial->printf("\n");

    DataFlash.ListAvailableLogs(cliSerial);

    return(true);
}

int8_t Copter::dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log_num;
    uint16_t dump_log_start;
    uint16_t dump_log_end;

    // check that the requested log number can be read
    dump_log_num = argv[1].i;

    if (dump_log_num == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log_num <= 0) {
        cliSerial->printf("dumping all\n");
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2) || ((uint16_t)dump_log_num > DataFlash.get_num_logs())) {
        cliSerial->printf("bad log number\n");
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log_num, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log_num, dump_log_start, dump_log_end);
    return (0);
}

int8_t Copter::erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}

int8_t Copter::select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint16_t bits;

    if (argc != 2) {
        cliSerial->printf("missing log type\n");
        return(-1);
    }

    bits = 0;

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp(argv[1].str, "all")) {
        bits = ~0;
    } else {
 #define TARG(_s)        if (!strcasecmp(argv[1].str, # _s)) bits |= MASK_LOG_ ## _s
        TARG(ATTITUDE_FAST);
        TARG(ATTITUDE_MED);
        TARG(GPS);
        TARG(PM);
        TARG(CTUN);
        TARG(NTUN);
        TARG(RCIN);
        TARG(IMU);
        TARG(CMD);
        TARG(CURRENT);
        TARG(RCOUT);
        TARG(OPTFLOW);
        TARG(COMPASS);
        TARG(CAMERA);
        TARG(PID);
 #undef TARG
    }

    if (!strcasecmp(argv[0].str, "enable")) {
        g.log_bitmask.set_and_save(g.log_bitmask | bits);
    }else{
        g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
    }

    return(0);
}

int8_t Copter::process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}
#endif // CLI_ENABLED

void Copter::do_erase_logs(void)
{
    gcs_send_text(MAV_SEVERITY_INFO, "Erasing logs");
    DataFlash.EraseAll();
    gcs_send_text(MAV_SEVERITY_INFO, "Log erase complete");
}

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
void Copter::Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt)
{
    struct log_AutoTune pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNE_MSG),
        time_us     : AP_HAL::micros64(),
        axis        : axis,
        tune_step   : tune_step,
        meas_target : meas_target,
        meas_min    : meas_min,
        meas_max    : meas_max,
        new_gain_rp : new_gain_rp,
        new_gain_rd : new_gain_rd,
        new_gain_sp : new_gain_sp,
        new_ddt     : new_ddt
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_AutoTuneDetails {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    angle_cd;      // lean angle in centi-degrees
    float    rate_cds;      // current rotation rate in centi-degrees / second
};

// Write an Autotune data packet
void Copter::Log_Write_AutoTuneDetails(float angle_cd, float rate_cds)
{
    struct log_AutoTuneDetails pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNEDETAILS_MSG),
        time_us     : AP_HAL::micros64(),
        angle_cd    : angle_cd,
        rate_cds    : rate_cds
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

// Write a Current data packet
void Copter::Log_Write_Current()
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
void Copter::Log_Write_Nav_Tuning()
{
    const Vector3f &pos_target = pos_control->get_pos_target();
    const Vector3f &vel_target = pos_control->get_vel_target();
    const Vector3f &accel_target = pos_control->get_accel_target();
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
    float    throttle_in;
    float    angle_boost;
    float    throttle_out;
    float    throttle_hover;
    float    desired_alt;
    float    inav_alt;
    int32_t  baro_alt;
    int16_t  desired_rangefinder_alt;
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
    if (terrain.height_above_terrain(terr_alt, true)) {
        terr_alt = 0.0f;
    }
#endif

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
        desired_rangefinder_alt : (int16_t)target_rangefinder_alt,
        rangefinder_alt     : rangefinder_state.alt_cm,
        terr_alt            : terr_alt,
        target_climb_rate   : (int16_t)pos_control->get_vel_target_z(),
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
void Copter::Log_Write_Performance()
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
void Copter::Log_Write_Attitude()
{
    Vector3f targets = attitude_control->get_att_target_euler_cd();
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
void Copter::Log_Write_MotBatt()
{
#if FRAME_CONFIG != HELI_FRAME
    struct log_MotBatt pkt_mot = {
        LOG_PACKET_HEADER_INIT(LOG_MOTBATT_MSG),
        time_us         : AP_HAL::micros64(),
        lift_max        : (float)(motors->get_lift_max()),
        bat_volt        : (float)(motors->get_batt_voltage_filt()),
        bat_res         : (float)(motors->get_batt_resistance()),
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

void Copter::Log_Write_Baro(void)
{
    if (!ahrs.have_ekf_logging()) {
        DataFlash.Log_Write_Baro(barometer);
    }
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

// log EKF origin and ahrs home to dataflash
void Copter::Log_Write_Home_And_Origin()
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

// precision landing logging
struct PACKED log_Throw {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t stage;
    float velocity;
    float velocity_z;
    float accel;
    float ef_accel_z;
    uint8_t throw_detect;
    uint8_t attitude_ok;
    uint8_t height_ok;
    uint8_t pos_ok;
};

// Write a Throw mode details
void Copter::Log_Write_Throw(ThrowModeStage stage, float velocity, float velocity_z, float accel, float ef_accel_z, bool throw_detect, bool attitude_ok, bool height_ok, bool pos_ok)
{
    struct log_Throw pkt = {
        LOG_PACKET_HEADER_INIT(LOG_THROW_MSG),
        time_us         : AP_HAL::micros64(),
        stage           : (uint8_t)stage,
        velocity        : velocity,
        velocity_z      : velocity_z,
        accel           : accel,
        ef_accel_z      : ef_accel_z,
        throw_detect    : throw_detect,
        attitude_ok     : attitude_ok,
        height_ok       : height_ok,
        pos_ok          : pos_ok
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// proximity sensor logging
struct PACKED log_Proximity {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t health;
    float dist0;
    float dist45;
    float dist90;
    float dist135;
    float dist180;
    float dist225;
    float dist270;
    float dist315;
    float distup;
    float closest_angle;
    float closest_dist;
};

// Write proximity sensor distances
void Copter::Log_Write_Proximity()
{
#if PROXIMITY_ENABLED == ENABLED
    // exit immediately if not enabled
    if (g2.proximity.get_status() == AP_Proximity::Proximity_NotConnected) {
        return;
    }

    float sector_distance[8] = {0,0,0,0,0,0,0,0};
    g2.proximity.get_horizontal_distance(0, sector_distance[0]);
    g2.proximity.get_horizontal_distance(45, sector_distance[1]);
    g2.proximity.get_horizontal_distance(90, sector_distance[2]);
    g2.proximity.get_horizontal_distance(135, sector_distance[3]);
    g2.proximity.get_horizontal_distance(180, sector_distance[4]);
    g2.proximity.get_horizontal_distance(225, sector_distance[5]);
    g2.proximity.get_horizontal_distance(270, sector_distance[6]);
    g2.proximity.get_horizontal_distance(315, sector_distance[7]);

    float dist_up;
    if (!g2.proximity.get_upward_distance(dist_up)) {
        dist_up = 0.0f;
    }

    float close_ang = 0.0f, close_dist = 0.0f;
    g2.proximity.get_closest_object(close_ang, close_dist);

    struct log_Proximity pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PROXIMITY_MSG),
        time_us         : AP_HAL::micros64(),
        health          : (uint8_t)g2.proximity.get_status(),
        dist0           : sector_distance[0],
        dist45          : sector_distance[1],
        dist90          : sector_distance[2],
        dist135         : sector_distance[3],
        dist180         : sector_distance[4],
        dist225         : sector_distance[5],
        dist270         : sector_distance[6],
        dist315         : sector_distance[7],
        distup          : dist_up,
        closest_angle   : close_ang,
        closest_dist    : close_dist
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#endif
}

// beacon sensor logging
struct PACKED log_Beacon {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t health;
    uint8_t count;
    float dist0;
    float dist1;
    float dist2;
    float dist3;
    float posx;
    float posy;
    float posz;
};

// Write beacon position and distances
void Copter::Log_Write_Beacon()
{
    // exit immediately if feature is disabled
    if (!g2.beacon.enabled()) {
        return;
    }

    // position
    Vector3f pos;
    float accuracy = 0.0f;
    g2.beacon.get_vehicle_position_ned(pos, accuracy);

    struct log_Beacon pkt = {
        LOG_PACKET_HEADER_INIT(LOG_BEACON_MSG),
        time_us         : AP_HAL::micros64(),
        health          : (uint8_t)g2.beacon.healthy(),
        count           : (uint8_t)g2.beacon.count(),
        dist0           : g2.beacon.beacon_distance(0),
        dist1           : g2.beacon.beacon_distance(1),
        dist2           : g2.beacon.beacon_distance(2),
        dist3           : g2.beacon.beacon_distance(3),
        posx            : pos.x,
        posy            : pos.y,
        posz            : pos.z
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

const struct LogStructure Copter::log_structure[] = {
    LOG_COMMON_STRUCTURES,
#if AUTOTUNE_ENABLED == ENABLED
    { LOG_AUTOTUNE_MSG, sizeof(log_AutoTune),
      "ATUN", "QBBfffffff",       "TimeUS,Axis,TuneStep,Targ,Min,Max,RP,RD,SP,ddt" },
    { LOG_AUTOTUNEDETAILS_MSG, sizeof(log_AutoTuneDetails),
      "ATDE", "Qff",          "TimeUS,Angle,Rate" },
#endif
    { LOG_PARAMTUNE_MSG, sizeof(log_ParameterTuning),
      "PTUN", "QBfHHH",          "TimeUS,Param,TunVal,CtrlIn,TunLo,TunHi" },  
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),       
      "OF",   "QBffff",   "TimeUS,Qual,flowX,flowY,bodyX,bodyY" },
    { LOG_NAV_TUNING_MSG, sizeof(log_Nav_Tuning),       
      "NTUN", "Qffffffffff", "TimeUS,DPosX,DPosY,PosX,PosY,DVelX,DVelY,VelX,VelY,DAccX,DAccY" },
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qffffffeccfhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt" },
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
    { LOG_HELI_MSG, sizeof(log_Heli),
      "HELI",  "Qff",         "TimeUS,DRRPM,ERRPM" },
    { LOG_PRECLAND_MSG, sizeof(log_Precland),
      "PL",    "QBBffff",    "TimeUS,Heal,TAcq,pX,pY,vX,vY" },
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUID",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ" },
    { LOG_THROW_MSG, sizeof(log_Throw),
      "THRO",  "QBffffbbbb",  "TimeUS,Stage,Vel,VelZ,Acc,AccEfZ,Throw,AttOk,HgtOk,PosOk" },
    { LOG_PROXIMITY_MSG, sizeof(log_Proximity),
      "PRX",   "QBfffffffffff","TimeUS,Health,D0,D45,D90,D135,D180,D225,D270,D315,DUp,CAn,CDis" },
    { LOG_BEACON_MSG, sizeof(log_Beacon),
      "BCN",   "QBBfffffff",  "TimeUS,Health,Cnt,D0,D1,D2,D3,PosX,PosY,PosZ" },
};

#if CLI_ENABLED == ENABLED
// Read the DataFlash log memory
void Copter::Log_Read(uint16_t list_entry, uint16_t start_page, uint16_t end_page)
{
    cliSerial->printf("\n" FIRMWARE_STRING
                        "\nFree RAM: %u\n"
                        "\nFrame: %s\n",
                        (unsigned) hal.util->available_memory(),
                        get_frame_string());

    cliSerial->printf("%s\n", HAL_BOARD_NAME);

    DataFlash.LogReadProcess(list_entry, start_page, end_page,
                             FUNCTOR_BIND_MEMBER(&Copter::print_flight_mode, void, AP_HAL::BetterStream *, uint8_t),
                             cliSerial);
}
#endif // CLI_ENABLED

void Copter::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by DataFlash
    char frame_buf[20];
    snprintf(frame_buf, sizeof(frame_buf), "Frame: %s", get_frame_string());
    DataFlash.Log_Write_Message(frame_buf);
    DataFlash.Log_Write_Mode(control_mode, control_mode_reason);
#if AC_RALLY
    DataFlash.Log_Write_Rally(rally);
#endif
    Log_Write_Home_And_Origin();
}


// start a new log
void Copter::start_logging() 
{
    if (g.log_bitmask != 0 && !in_log_download) {
        if (!ap.logging_started) {
            ap.logging_started = true;
            DataFlash.set_mission(&mission);
            DataFlash.setVehicle_Startup_Log_Writer(FUNCTOR_BIND(&copter, &Copter::Log_Write_Vehicle_Startup_Messages, void));
            DataFlash.StartNewLog();
        } else if (!DataFlash.logging_started()) {
            // dataflash may have stopped logging - when we get_log_data,
            // for example.  Try to restart:
            DataFlash.StartNewLog();
        }
        // enable writes
        DataFlash.EnableWrites(true);
    }
}

void Copter::log_init(void)
{
    DataFlash.Init(log_structure, ARRAY_SIZE(log_structure));
    if (!DataFlash.CardInserted()) {
        gcs_send_text(MAV_SEVERITY_WARNING, "No dataflash card inserted");
    } else if (DataFlash.NeedPrep()) {
        gcs_send_text(MAV_SEVERITY_INFO, "Preparing log system");
        DataFlash.Prep();
        gcs_send_text(MAV_SEVERITY_INFO, "Prepared log system");
        for (uint8_t i=0; i<num_gcs; i++) {
            gcs_chan[i].reset_cli_timeout();
        }
    }
}

#else // LOGGING_ENABLED

#if CLI_ENABLED == ENABLED
bool Copter::print_log_menu(void) { return true; }
int8_t Copter::dump_log(uint8_t argc, const Menu::arg *argv) { return 0; }
int8_t Copter::erase_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
int8_t Copter::select_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
int8_t Copter::process_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
void Copter::Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page) {}
#endif // CLI_ENABLED == ENABLED

void Copter::do_erase_logs(void) {}
void Copter::Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float meas_target, \
                                float meas_min, float meas_max, float new_gain_rp, \
                                float new_gain_rd, float new_gain_sp, float new_ddt) {}
void Copter::Log_Write_AutoTuneDetails(float angle_cd, float rate_cds) {}
void Copter::Log_Write_Current() {}
void Copter::Log_Write_Nav_Tuning() {}
void Copter::Log_Write_Control_Tuning() {}
void Copter::Log_Write_Performance() {}
void Copter::Log_Write_Attitude(void) {}
void Copter::Log_Write_MotBatt() {}
void Copter::Log_Write_Event(uint8_t id) {}
void Copter::Log_Write_Data(uint8_t id, int32_t value) {}
void Copter::Log_Write_Data(uint8_t id, uint32_t value) {}
void Copter::Log_Write_Data(uint8_t id, int16_t value) {}
void Copter::Log_Write_Data(uint8_t id, uint16_t value) {}
void Copter::Log_Write_Data(uint8_t id, float value) {}
void Copter::Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
void Copter::Log_Write_Baro(void) {}
void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, int16_t control_in, int16_t tune_low, int16_t tune_high) {}
void Copter::Log_Write_Home_And_Origin() {}
void Copter::Log_Sensor_Health() {}
void Copter::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target) {}
void Copter::Log_Write_Proximity() {}
void Copter::Log_Write_Beacon() {}
void Copter::Log_Write_Precland() {}
void Copter::Log_Write_Throw(ThrowModeStage stage, float velocity, float velocity_z, float accel, float ef_accel_z, bool throw_detect, bool attitude_ok, bool height_ok, bool pos_ok) {}

#if FRAME_CONFIG == HELI_FRAME
void Copter::Log_Write_Heli() {}
#endif

#if OPTFLOW == ENABLED
void Copter::Log_Write_Optflow() {}
#endif

void Copter::start_logging() {}
void Copter::log_init(void) {}

#endif // LOGGING_ENABLED
