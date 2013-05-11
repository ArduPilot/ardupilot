// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static bool     print_log_menu(void);
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command log_menu_commands[] PROGMEM = {
    {"dump",        dump_log},
    {"erase",       erase_logs},
    {"enable",      select_logs},
    {"disable",     select_logs}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

static bool
print_log_menu(void)
{
    cliSerial->printf_P(PSTR("logs enabled: "));

    if (0 == g.log_bitmask) {
        cliSerial->printf_P(PSTR("none"));
    }else{
        if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST) cliSerial->printf_P(PSTR(" ATTITUDE_FAST"));
        if (g.log_bitmask & MASK_LOG_ATTITUDE_MED) cliSerial->printf_P(PSTR(" ATTITUDE_MED"));
        if (g.log_bitmask & MASK_LOG_GPS) cliSerial->printf_P(PSTR(" GPS"));
        if (g.log_bitmask & MASK_LOG_PM) cliSerial->printf_P(PSTR(" PM"));
        if (g.log_bitmask & MASK_LOG_CTUN) cliSerial->printf_P(PSTR(" CTUN"));
        if (g.log_bitmask & MASK_LOG_NTUN) cliSerial->printf_P(PSTR(" NTUN"));
        if (g.log_bitmask & MASK_LOG_IMU) cliSerial->printf_P(PSTR(" IMU"));
        if (g.log_bitmask & MASK_LOG_CMD) cliSerial->printf_P(PSTR(" CMD"));
        if (g.log_bitmask & MASK_LOG_CURRENT) cliSerial->printf_P(PSTR(" CURRENT"));
        if (g.log_bitmask & MASK_LOG_MOTORS) cliSerial->printf_P(PSTR(" MOTORS"));
        if (g.log_bitmask & MASK_LOG_OPTFLOW) cliSerial->printf_P(PSTR(" OPTFLOW"));
        if (g.log_bitmask & MASK_LOG_PID) cliSerial->printf_P(PSTR(" PID"));
        if (g.log_bitmask & MASK_LOG_COMPASS) cliSerial->printf_P(PSTR(" COMPASS"));
        if (g.log_bitmask & MASK_LOG_INAV) cliSerial->printf_P(PSTR(" INAV"));
        if (g.log_bitmask & MASK_LOG_CAMERA) cliSerial->printf_P(PSTR(" CAMERA"));
    }

    cliSerial->println();

    DataFlash.ListAvailableLogs(cliSerial);

    return(true);
}

static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    uint16_t dump_log_start;
    uint16_t dump_log_end;
    uint16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log <= 0) {
        cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2) || (dump_log <= (last_log_num - DataFlash.get_num_logs())) || (dump_log > last_log_num)) {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return (0);
}

static void do_erase_logs(void)
{
	gcs_send_text_P(SEVERITY_LOW, PSTR("Erasing logs\n"));
    DataFlash.EraseAll();
	gcs_send_text_P(SEVERITY_LOW, PSTR("Log erase complete\n"));
}

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}

static int8_t
select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint16_t bits;

    if (argc != 2) {
        cliSerial->printf_P(PSTR("missing log type\n"));
        return(-1);
    }

    bits = 0;

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp_P(argv[1].str, PSTR("all"))) {
        bits = ~0;
    } else {
 #define TARG(_s)        if (!strcasecmp_P(argv[1].str, PSTR(# _s))) bits |= MASK_LOG_ ## _s
        TARG(ATTITUDE_FAST);
        TARG(ATTITUDE_MED);
        TARG(GPS);
        TARG(PM);
        TARG(CTUN);
        TARG(NTUN);
        TARG(MODE);
        TARG(IMU);
        TARG(CMD);
        TARG(CURRENT);
        TARG(MOTORS);
        TARG(OPTFLOW);
        TARG(PID);
        TARG(COMPASS);
        TARG(INAV);
        TARG(CAMERA);
 #undef TARG
    }

    if (!strcasecmp_P(argv[0].str, PSTR("enable"))) {
        g.log_bitmask.set_and_save(g.log_bitmask | bits);
    }else{
        g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
    }

    return(0);
}

static int8_t
process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    int16_t throttle_in;
    uint32_t throttle_integrator;
    int16_t battery_voltage;
    int16_t current_amps;
    uint16_t board_voltage;
    float current_total;
};

// Write an Current data packet
static void Log_Write_Current()
{
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        throttle_in         : g.rc_3.control_in,
        throttle_integrator : throttle_integrator,
        battery_voltage     : (int16_t) (battery_voltage1 * 100.0f),
        current_amps        : (int16_t) (current_amps1 * 100.0f),
        board_voltage       : board_voltage(),
        current_total       : current_total1
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Motors {
    LOG_PACKET_HEADER;
#if FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME
    int16_t motor_out[8];
#elif FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME
    int16_t motor_out[6];
#elif FRAME_CONFIG == HELI_FRAME
    int16_t motor_out[4];
    int16_t ext_gyro_gain;
#else        // quads & TRI_FRAME
    int16_t motor_out[4];
#endif
};

// Write an Motors packet
static void Log_Write_Motors()
{
    struct log_Motors pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MOTORS_MSG),
#if FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_3],
                         motors.motor_out[AP_MOTORS_MOT_4],
                         motors.motor_out[AP_MOTORS_MOT_5],
                         motors.motor_out[AP_MOTORS_MOT_6],
                         motors.motor_out[AP_MOTORS_MOT_7],
                         motors.motor_out[AP_MOTORS_MOT_8]}
#elif FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_3],
                         motors.motor_out[AP_MOTORS_MOT_4],
                         motors.motor_out[AP_MOTORS_MOT_5],
                         motors.motor_out[AP_MOTORS_MOT_6]}
#elif FRAME_CONFIG == HELI_FRAME
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_3],
                         motors.motor_out[AP_MOTORS_MOT_4]},
        ext_gyro_gain   : motors.ext_gyro_gain
#elif FRAME_CONFIG == TRI_FRAME
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_4],
                         motors.motor_out[g.rc_4.radio_out]}
#else // QUAD frame
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_3],
                         motors.motor_out[AP_MOTORS_MOT_4]}
#endif
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    int16_t dx;
    int16_t dy;
    uint8_t surface_quality;
    int16_t x_cm;
    int16_t y_cm;
    float   latitude;
    float   longitude;
    int32_t roll;
    int32_t pitch;
};

// Write an optical flow packet
static void Log_Write_Optflow()
{
 #if OPTFLOW == ENABLED
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        dx              : optflow.dx,
        dy              : optflow.dx,
        surface_quality : optflow.surface_quality,
        x_cm            : (int16_t) optflow.x_cm,
        y_cm            : (int16_t) optflow.y_cm,
        latitude        : optflow.vlat,
        longitude       : optflow.vlon,
        roll            : of_roll,
        pitch           : of_pitch
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
 #endif     // OPTFLOW == ENABLED
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t wp_distance;
    int16_t  wp_bearing;
    float    lat_error;
    float    lon_error;
    int16_t  nav_pitch;
    int16_t  nav_roll;
    int16_t  lat_speed;
    int16_t  lon_speed;
};

// Write an Nav Tuning packet
static void Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        wp_distance : wp_distance,
        wp_bearing  : (int16_t) (wp_bearing/100),
        lat_error   : lat_error,
        lon_error   : lon_error,
        nav_pitch   : (int16_t) nav_pitch,
        nav_roll    : (int16_t) nav_roll,
        lat_speed   : (int16_t) inertial_nav.get_latitude_velocity(),
        lon_speed   : (int16_t) inertial_nav.get_longitude_velocity()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    int16_t throttle_in;
    int16_t sonar_alt;
    int32_t baro_alt;
    float   next_wp_alt;
    int16_t nav_throttle;
    int16_t angle_boost;
    int16_t climb_rate;
    int16_t throttle_out;
    int16_t desired_climb_rate;
};

// Write a control tuning packet
static void Log_Write_Control_Tuning()
{
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        throttle_in         : g.rc_3.control_in,
        sonar_alt           : sonar_alt,
        baro_alt            : baro_alt,
        next_wp_alt         : get_target_alt_for_reporting() / 100.0f,
        nav_throttle        : nav_throttle,
        angle_boost         : angle_boost,
        climb_rate          : climb_rate,
        throttle_out        : g.rc_3.servo_out,
        desired_climb_rate  : desired_climb_rate
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
    int16_t motor_offset_x;
    int16_t motor_offset_y;
    int16_t motor_offset_z;
};

// Write a Compass packet
static void Log_Write_Compass()
{
    Vector3f mag_offsets = compass.get_offsets();
    Vector3f mag_motor_offsets = compass.get_motor_offsets();
    struct log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
        mag_x           : compass.mag_x,
        mag_y           : compass.mag_y,
        mag_z           : compass.mag_z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint8_t renorm_count;
    uint8_t renorm_blowup;
    uint8_t gps_fix_count;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    int16_t  pm_test;
    uint8_t i2c_lockup_count;
};

// Write a performance monitoring packet
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        renorm_count     : ahrs.renorm_range_count,
        renorm_blowup    : ahrs.renorm_blowup_count,
        gps_fix_count    : gps_fix_count,
        num_long_running : perf_info_get_num_long_running(),
        num_loops        : perf_info_get_num_loops(),
        max_time         : perf_info_get_max_time(),
        pm_test          : pmTest1,
        i2c_lockup_count : hal.i2c->lockup_count()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Cmd {
    LOG_PACKET_HEADER;
    uint8_t command_total;
    uint8_t command_number;
    uint8_t waypoint_id;
    uint8_t waypoint_options;
    uint8_t waypoint_param1;
    int32_t waypoint_altitude;
    int32_t waypoint_latitude;
    int32_t waypoint_longitude;
};

// Write a command processing packet
static void Log_Write_Cmd(uint8_t num, const struct Location *wp)
{
    struct log_Cmd pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CMD_MSG),
        command_total       : g.command_total,
        command_number      : num,
        waypoint_id         : wp->id,
        waypoint_options    : wp->options,
        waypoint_param1     : wp->p1,
        waypoint_altitude   : wp->alt,
        waypoint_latitude   : wp->lat,
        waypoint_longitude  : wp->lng
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    int16_t roll_in;
    int16_t roll;
    int16_t pitch_in;
    int16_t pitch;
    int16_t yaw_in;
    uint16_t yaw;
    uint16_t nav_yaw;
};

// Write an attitude packet
static void Log_Write_Attitude()
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        roll_in     : (int16_t)control_roll,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch_in    : (int16_t)control_pitch,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw_in      : (int16_t)g.rc_4.control_in,
        yaw         : (uint16_t)ahrs.yaw_sensor,
        nav_yaw     : (uint16_t)nav_yaw
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_INAV {
    LOG_PACKET_HEADER;
    int16_t baro_alt;
    int16_t inav_alt;
    int16_t inav_climb_rate;
    float   accel_corr_x;
    float   accel_corr_y;
    float   accel_corr_z;
    int32_t gps_lat_from_home;
    int32_t gps_lon_from_home;
    float   inav_lat_from_home;
    float   inav_lon_from_home;
};

// Write an INAV packet
static void Log_Write_INAV()
{
    Vector3f accel_corr = inertial_nav.accel_correction_ef;

    struct log_INAV pkt = {
        LOG_PACKET_HEADER_INIT(LOG_INAV_MSG),
        baro_alt            : (int16_t)baro_alt,                        // 1 barometer altitude
        inav_alt            : (int16_t)inertial_nav.get_altitude(),     // 2 accel + baro filtered altitude
        inav_climb_rate     : (int16_t)inertial_nav.get_velocity_z(),   // 3 accel + baro based climb rate
        accel_corr_x        : accel_corr.x,                             // 4 accel correction x-axis
        accel_corr_y        : accel_corr.y,                             // 5 accel correction y-axis
        accel_corr_z        : accel_corr.z,                             // 6 accel correction z-axis
        gps_lat_from_home   : g_gps->latitude-home.lat,                 // 7 lat from home
        gps_lon_from_home   : g_gps->longitude-home.lng,                // 8 lon from home
        inav_lat_from_home  : inertial_nav.get_latitude_diff(),         // 9 accel based lat from home
        inav_lon_from_home  : inertial_nav.get_longitude_diff()        // 10 accel based lon from home
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint8_t mode;
    int16_t throttle_cruise;
};

// Write a mode packet
static void Log_Write_Mode(uint8_t mode)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        mode            : mode,
        throttle_cruise : g.throttle_cruise,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
};

// Write Startup packet
static void Log_Write_Startup()
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Event {
    LOG_PACKET_HEADER;
    uint8_t id;
};

// Wrote an event packet
static void Log_Write_Event(uint8_t id)
{
    if (g.log_bitmask != 0) {
        struct log_Event pkt = {
            LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
            id  : id
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
static void Log_Write_Data(uint8_t id, int16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
static void Log_Write_Data(uint8_t id, uint16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
static void Log_Write_Data(uint8_t id, int32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
static void Log_Write_Data(uint8_t id, uint32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint8_t id;
    float data_value;
};

// Write a float data packet
static void Log_Write_Data(uint8_t id, float value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_PID {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t error;
    int32_t p;
    int32_t i;
    int32_t d;
    int32_t output;
    float  gain;
};

// Write an PID packet
static void Log_Write_PID(uint8_t pid_id, int32_t error, int32_t p, int32_t i, int32_t d, int32_t output, float gain)
{
    struct log_PID pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PID_MSG),
        id      : pid_id,
        error   : error,
        p       : p,
        i       : i,
        d       : d,
        output  : output,
        gain    : gain
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_DMP {
    LOG_PACKET_HEADER;
    int16_t  dcm_roll;
    int16_t  dmp_roll;
    int16_t  dcm_pitch;
    int16_t  dmp_pitch;
    uint16_t dcm_yaw;
    uint16_t dmp_yaw;
};

#if SECONDARY_DMP_ENABLED == ENABLED
// Write a DMP attitude packet
static void Log_Write_DMP()
{
    struct log_DMP pkt = {
        LOG_PACKET_HEADER_INIT(LOG_DMP_MSG),
        dcm_roll    : (int16_t)ahrs.roll_sensor,
        dmp_roll    : (int16_t)ahrs2.roll_sensor,
        dcm_pitch   : (int16_t)ahrs.pitch_sensor,
        dmp_pitch   : (int16_t)ahrs2.pitch_sensor,
        dcm_yaw     : (uint16_t)ahrs.yaw_sensor,
        dmp_yaw     : (uint16_t)ahrs2.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint32_t gps_time;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
};

// Write a Camera packet
static void Log_Write_Camera()
{
#if CAMERA == ENABLED
    struct log_Camera pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CAMERA_MSG),
        gps_time    : g_gps->time,
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        altitude    : current_loc.alt,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#endif
}

struct PACKED log_Error {
    LOG_PACKET_HEADER;
    uint8_t sub_system;
    uint8_t error_code;
};

// Write an error packet
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_WPNAV {
    LOG_PACKET_HEADER;
    float   pos_error_x;
    float   pos_error_y;
    float   desired_velocity_x;
    float   desired_velocity_y;
    float   velocity_x;
    float   velocity_y;
    float   desired_accel_x;
    float   desired_accel_y;
    int32_t desired_roll;
    int32_t desired_pitch;
};

// Write an WPNAV packet
static void Log_Write_WPNAV()
{
    Vector3f velocity = inertial_nav.get_velocity();

    struct log_WPNAV pkt = {
        LOG_PACKET_HEADER_INIT(LOG_WPNAV_MSG),
        pos_error_x         : wp_nav.dist_error.x,
        pos_error_y         : wp_nav.dist_error.y,
        desired_velocity_x  : wp_nav.desired_vel.x,
        desired_velocity_y  : wp_nav.desired_vel.y,
        velocity_x          : velocity.x,
        velocity_y          : velocity.y,
        desired_accel_x     : wp_nav.desired_accel.x,
        desired_accel_y     : wp_nav.desired_accel.y,
        desired_roll        : wp_nav.get_desired_roll(),
        desired_pitch       : wp_nav.get_desired_pitch()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
    { LOG_CURRENT_MSG, sizeof(log_Current),             
      "CURR", "hIhhhf",      "Thr,ThrInt,Volt,Curr,Vcc,CurrTot" },

#if FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME
    { LOG_MOTORS_MSG, sizeof(log_Motors),       
      "MOT",  "hhhhhhhh",    "Mot1,Mot2,Mot3,Mot4,Mot5,Mot6,Mot7,Mot8" },
#elif FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME
    { LOG_MOTORS_MSG, sizeof(log_Motors),       
      "MOT",  "hhhhhh",      "Mot1,Mot2,Mot3,Mot4,Mot5,Mot6" },
#elif FRAME_CONFIG == HELI_FRAME
    { LOG_MOTORS_MSG, sizeof(log_Motors),       
      "MOT",  "hhhhh",       "Mot1,Mot2,Mot3,Mot4,GGain" },
#else
    { LOG_MOTORS_MSG, sizeof(log_Motors),       
      "MOT",  "hhhh",        "Mot1,Mot2,Mot3,Mot4" },
#endif

    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),       
      "OF",   "hhBccffee",   "Dx,Dy,SQual,X,Y,Lat,Lng,Roll,Pitch" },
    { LOG_NAV_TUNING_MSG, sizeof(log_Nav_Tuning),       
      "NTUN", "Ecffcccc",    "WPDist,TargBrg,LatErr,LngErr,NavPtch,NavRll,LatSpd,LngSpd" },
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "hcefhhhhh",   "ThrIn,SonAlt,BarAlt,WPAlt,NavThr,AngBst,CRate,ThrOut,DCRate" },
    { LOG_COMPASS_MSG, sizeof(log_Compass),             
      "MAG", "hhhhhhhhh",    "MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "BBBHHIhB",       "RenCnt,RenBlw,FixCnt,NLon,NLoop,MaxT,PMT,I2CErr" },
    { LOG_CMD_MSG, sizeof(log_Cmd),                 
      "CMD", "BBBBBeLL",     "CTot,CNum,CId,COpt,Prm1,Alt,Lat,Lng" },
    { LOG_ATTITUDE_MSG, sizeof(log_Attitude),       
      "ATT", "cccccCC",      "RollIn,Roll,PitchIn,Pitch,YawIn,Yaw,NavYaw" },
    { LOG_INAV_MSG, sizeof(log_INAV),       
      "INAV", "cccfffiiff",  "BAlt,IAlt,IClb,ACorrX,ACorrY,ACorrZ,GLat,GLng,ILat,ILng" },
    { LOG_MODE_MSG, sizeof(log_Mode),
      "MODE", "Mh",          "Mode,ThrCrs" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "",            "" },
    { LOG_EVENT_MSG, sizeof(log_Event),         
      "EV",   "B",           "Id" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "Bh",         "Id,Value" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "BH",         "Id,Value" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "Bi",         "Id,Value" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "BI",         "Id,Value" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "Bf",         "Id,Value" },
    { LOG_PID_MSG, sizeof(log_PID),         
      "PID",   "Biiiiif",    "Id,Error,P,I,D,Out,Gain" },
    { LOG_DMP_MSG, sizeof(log_DMP),         
      "DMP",   "ccccCC",     "DCMRoll,DMPRoll,DCMPtch,DMPPtch,DCMYaw,DMPYaw" },
    { LOG_CAMERA_MSG, sizeof(log_Camera),                 
      "CAM",   "ILLeccC",    "GPSTime,Lat,Lng,Alt,Roll,Pitch,Yaw" },
    { LOG_ERROR_MSG, sizeof(log_Error),         
      "ERR",   "BB",         "Subsys,ECode" },
    { LOG_WPNAV_MSG, sizeof(log_WPNAV),         
      "WNAV",  "ffffffffee", "PErrX,PErrY,DVelX,DVelY,VelX,VelY,DAccX,DAccY,DRoll,DPtch" },
};

// Read the DataFlash log memory
static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page)
{
 #ifdef AIRFRAME_NAME
    cliSerial->printf_P(PSTR((AIRFRAME_NAME)));
 #endif

    cliSerial->printf_P(PSTR("\n" THISFIRMWARE
                             "\nFree RAM: %u\n"),
                        (unsigned) memcheck_available_memory());

    cliSerial->println_P(PSTR(HAL_BOARD_NAME));

	DataFlash.LogReadProcess(log_num, start_page, end_page, 
                             sizeof(log_structure)/sizeof(log_structure[0]),
                             log_structure, 
                             print_flight_mode,
                             cliSerial);
}

// start a new log
static void start_logging() 
{
    if (g.log_bitmask != 0 && !ap.logging_started) {
        ap.logging_started = true;
        DataFlash.StartNewLog(sizeof(log_structure)/sizeof(log_structure[0]), log_structure);
    }
}

#else // LOGGING_ENABLED

static void Log_Write_Startup() {}
static void Log_Write_Cmd(uint8_t num, const struct Location *wp) {}
static void Log_Write_Mode(uint8_t mode) {}
static void Log_Write_IMU() {}
static void Log_Write_GPS() {}
static void Log_Write_Current() {}
static void Log_Write_Compass() {}
static void Log_Write_Attitude() {}
static void Log_Write_INAV() {}
static void Log_Write_Data(uint8_t id, int16_t value){}
static void Log_Write_Data(uint8_t id, uint16_t value){}
static void Log_Write_Data(uint8_t id, int32_t value){}
static void Log_Write_Data(uint8_t id, uint32_t value){}
static void Log_Write_Data(uint8_t id, float value){}
static void Log_Write_Event(uint8_t id){}
static void Log_Write_Optflow() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Motors() {}
static void Log_Write_Performance() {}
static void Log_Write_PID(uint8_t pid_id, int32_t error, int32_t p, int32_t i, int32_t d, int32_t output, float gain) {}
#if SECONDARY_DMP_ENABLED == ENABLED
static void Log_Write_DMP() {}
#endif
static void Log_Write_Camera() {}
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}

#endif // LOGGING_DISABLED
