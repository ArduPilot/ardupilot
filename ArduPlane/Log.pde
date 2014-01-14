// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash.log memory
// Code to interact with the user to dump or erase logs

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
static const struct Menu::command log_menu_commands[] PROGMEM = {
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
    cliSerial->println_P(PSTR("logs enabled: "));

    if (0 == g.log_bitmask) {
        cliSerial->println_P(PSTR("none"));
    }else{
        // Macro to make the following code a bit easier on the eye.
        // Pass it the capitalised name of the log option, as defined
        // in defines.h but without the LOG_ prefix.  It will check for
        // the bit being set and print the name of the log option to suit.
 #define PLOG(_s) if (g.log_bitmask & MASK_LOG_ ## _s) cliSerial->printf_P(PSTR(" %S"), PSTR(# _s))
        PLOG(ATTITUDE_FAST);
        PLOG(ATTITUDE_MED);
        PLOG(GPS);
        PLOG(PM);
        PLOG(CTUN);
        PLOG(NTUN);
        PLOG(MODE);
        PLOG(IMU);
        PLOG(CMD);
        PLOG(CURRENT);
        PLOG(COMPASS);
        PLOG(TECS);
        PLOG(CAMERA);
        PLOG(RC);
        PLOG(SONAR);
 #undef PLOG
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
    } else if ((argc != 2)
               || ((uint16_t)dump_log > last_log_num))
    {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return 0;
}

static void do_erase_logs(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Erasing logs"));
    DataFlash.EraseAll();
    gcs_send_text_P(SEVERITY_LOW, PSTR("Log erase complete"));
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
    uint32_t bits;

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
        bits = 0xFFFFFFFFUL;
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
        TARG(COMPASS);
        TARG(TECS);
        TARG(CAMERA);
        TARG(RC);
        TARG(SONAR);
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

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    uint16_t error_rp;
    uint16_t error_yaw;
};

// Write an attitude packet. Total length : 10 bytes
static void Log_Write_Attitude(void)
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_ms : hal.scheduler->millis(),
        roll  : (int16_t)ahrs.roll_sensor,
        pitch : (int16_t)ahrs.pitch_sensor,
        yaw   : (uint16_t)ahrs.yaw_sensor,
        error_rp  : (uint16_t)(ahrs.get_error_rp() * 100),
        error_yaw : (uint16_t)(ahrs.get_error_yaw() * 100)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint32_t loop_time;
    uint16_t main_loop_count;
    uint32_t g_dt_max;
    uint8_t  renorm_count;
    uint8_t  renorm_blowup;
    int16_t  gyro_drift_x;
    int16_t  gyro_drift_y;
    int16_t  gyro_drift_z;
    uint8_t  i2c_lockup_count;
    uint16_t ins_error_count;
};

// Write a performance monitoring packet. Total length : 19 bytes
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        loop_time       : millis() - perf_mon_timer,
        main_loop_count : mainLoop_count,
        g_dt_max        : G_Dt_max,
        renorm_count    : ahrs.renorm_range_count,
        renorm_blowup   : ahrs.renorm_blowup_count,
        gyro_drift_x    : (int16_t)(ahrs.get_gyro_drift().x * 1000),
        gyro_drift_y    : (int16_t)(ahrs.get_gyro_drift().y * 1000),
        gyro_drift_z    : (int16_t)(ahrs.get_gyro_drift().z * 1000),
        i2c_lockup_count: hal.i2c->lockup_count(),
        ins_error_count  : ins.error_count()
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

// Write a command processing packet. Total length : 19 bytes
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

struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint32_t gps_time;
    uint16_t gps_week;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
};

// Write a Camera packet. Total length : 26 bytes
static void Log_Write_Camera()
{
#if CAMERA == ENABLED
    struct log_Camera pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CAMERA_MSG),
        gps_time    : g_gps->time_week_ms,
        gps_week    : g_gps->time_week,
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

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
    uint8_t startup_type;
    uint8_t command_total;
};

static void Log_Write_Startup(uint8_t type)
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG),
        startup_type    : type,
        command_total   : g.command_total
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

    // write all commands to the dataflash as well
    struct Location cmd;
    for (uint8_t i = 0; i <= g.command_total; i++) {
        cmd = get_cmd_with_index(i);
        Log_Write_Cmd(i, &cmd);
    }
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t nav_roll_cd;
    int16_t roll;
    int16_t nav_pitch_cd;
    int16_t pitch;
    int16_t throttle_out;
    int16_t rudder_out;
    float   accel_y;
};

// Write a control tuning packet. Total length : 22 bytes
static void Log_Write_Control_Tuning()
{
    Vector3f accel = ins.get_accel();
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CTUN_MSG),
        time_ms         : hal.scheduler->millis(),
        nav_roll_cd     : (int16_t)nav_roll_cd,
        roll            : (int16_t)ahrs.roll_sensor,
        nav_pitch_cd    : (int16_t)nav_pitch_cd,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)channel_throttle->servo_out,
        rudder_out      : (int16_t)channel_rudder->servo_out,
        accel_y         : accel.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Write a TECS tuning packet
static void Log_Write_TECS_Tuning(void)
{
    SpdHgt_Controller->log_data(DataFlash, LOG_TECS_MSG);
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint16_t yaw;
    uint32_t wp_distance;
    uint16_t target_bearing_cd;
    uint16_t nav_bearing_cd;
    int16_t altitude_error_cm;
    int16_t airspeed_cm;
    float   altitude;
    uint32_t groundspeed_cm;
};

// Write a navigation tuning packe
static void Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_ms             : hal.scheduler->millis(),
        yaw                 : (uint16_t)ahrs.yaw_sensor,
        wp_distance         : wp_distance,
        target_bearing_cd   : (uint16_t)nav_controller->target_bearing_cd(),
        nav_bearing_cd      : (uint16_t)nav_controller->nav_bearing_cd(),
        altitude_error_cm   : (int16_t)altitude_error_cm,
        airspeed_cm         : (int16_t)airspeed.get_airspeed_cm(),
        altitude            : barometer.get_altitude(),
        groundspeed_cm      : g_gps->ground_speed_cm
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t mode;
    uint8_t mode_num;
};

// Write a mode packet. Total length : 5 bytes
static void Log_Write_Mode(uint8_t mode)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        time_ms  : hal.scheduler->millis(),
        mode     : mode,
        mode_num : mode
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Sonar {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float distance;
    float voltage;
    float baro_alt;
    float groundspeed;
    uint8_t throttle;
};

// Write a sonar packet
static void Log_Write_Sonar()
{
    struct log_Sonar pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SONAR_MSG),
        timestamp   : hal.scheduler->millis(),
        distance    : sonar.distance_cm(),
        voltage     : sonar.voltage(),
        baro_alt    : barometer.get_altitude(),
        groundspeed : (0.01f * g_gps->ground_speed_cm),
        throttle    : (uint8_t)(100 * channel_throttle->norm_output())
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t throttle_in;
    int16_t battery_voltage;
    int16_t current_amps;
    uint16_t board_voltage;
    float   current_total;
};

struct PACKED log_Arm_Disarm {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t  arm_state;
    uint16_t arm_checks;
};

static void Log_Write_Current()
{
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        time_ms                 : hal.scheduler->millis(),
        throttle_in             : channel_throttle->control_in,
        battery_voltage         : (int16_t)(battery.voltage() * 100.0),
        current_amps            : (int16_t)(battery.current_amps() * 100.0),
        board_voltage           : board_voltage(),
        current_total           : battery.current_total_mah()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static void Log_Arm_Disarm() {
    struct log_Arm_Disarm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_ms                 : hal.scheduler->millis(),
        arm_state               : arming.is_armed(),
        arm_checks              : arming.get_enabled_checks()      
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
};

// Write a Compass packet. Total length : 15 bytes
static void Log_Write_Compass()
{
    const Vector3f &mag_offsets = compass.get_offsets();
    const Vector3f &mag = compass.get_field();
    struct log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
        time_ms         : hal.scheduler->millis(),
        mag_x           : (int16_t)mag.x,
        mag_y           : (int16_t)mag.y,
        mag_z           : (int16_t)mag.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#if COMPASS_MAX_INSTANCES > 1
    if (compass.get_count() > 1) {
        const Vector3f &mag2_offsets = compass.get_offsets(1);
        const Vector3f &mag2 = compass.get_field(1);
        struct log_Compass pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_COMPASS2_MSG),
            time_ms         : hal.scheduler->millis(),
            mag_x           : (int16_t)mag2.x,
            mag_y           : (int16_t)mag2.y,
            mag_z           : (int16_t)mag2.z,
            offset_x        : (int16_t)mag2_offsets.x,
            offset_y        : (int16_t)mag2_offsets.y,
            offset_z        : (int16_t)mag2_offsets.z
        };
        DataFlash.WriteBlock(&pkt2, sizeof(pkt2));
    }
#endif
}

static void Log_Write_GPS(void)
{
    DataFlash.Log_Write_GPS(g_gps, current_loc.alt);
}

static void Log_Write_IMU() 
{
    DataFlash.Log_Write_IMU(ins);
}

static void Log_Write_RC(void)
{
    DataFlash.Log_Write_RCIN();
    DataFlash.Log_Write_RCOUT();
}

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
    { LOG_ATTITUDE_MSG, sizeof(log_Attitude),       
      "ATT", "IccCCC",        "TimeMS,Roll,Pitch,Yaw,ErrorRP,ErrorYaw" },
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "IHIBBhhhBH", "LTime,MLC,gDt,RNCnt,RNBl,GDx,GDy,GDz,I2CErr,INSErr" },
    { LOG_CMD_MSG, sizeof(log_Cmd),                 
      "CMD", "BBBBBeLL",   "CTot,CNum,CId,COpt,Prm1,Alt,Lat,Lng" },
    { LOG_CAMERA_MSG, sizeof(log_Camera),                 
      "CAM", "IHLLeccC",   "GPSTime,GPSWeek,Lat,Lng,Alt,Roll,Pitch,Yaw" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "BB",         "SType,CTot" },
    { LOG_CTUN_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "Icccchhf",    "TimeMS,NavRoll,Roll,NavPitch,Pitch,ThrOut,RdrOut,AccY" },
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),         
      "NTUN", "ICICCccfI",   "TimeMS,Yaw,WpDist,TargBrg,NavBrg,AltErr,Arspd,Alt,GSpdCM" },
    { LOG_SONAR_MSG, sizeof(log_Sonar),             
      "SONR", "IffffB",   "TimeMS,DistCM,Volt,BaroAlt,GSpd,Thr" },
    { LOG_MODE_MSG, sizeof(log_Mode),             
      "MODE", "IMB",         "TimeMS,Mode,ModeNum" },
    { LOG_CURRENT_MSG, sizeof(log_Current),             
      "CURR", "IhhhHf",      "TimeMS,Thr,Volt,Curr,Vcc,CurrTot" },
    { LOG_COMPASS_MSG, sizeof(log_Compass),             
      "MAG", "Ihhhhhh",   "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ" },
    { LOG_COMPASS2_MSG, sizeof(log_Compass),             
      "MAG2", "Ihhhhhh",   "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ" },
    { LOG_ARM_DISARM_MSG, sizeof(log_Arm_Disarm),
      "ARM", "IHB", "TimeMS,ArmState,ArmChecks" },
    TECS_LOG_FORMAT(LOG_TECS_MSG),
};

// Read the DataFlash.log memory : Packet Parser
static void Log_Read(uint16_t log_num, int16_t start_page, int16_t end_page)
{
    cliSerial->printf_P(PSTR("\n" FIRMWARE_STRING
                             "\nFree RAM: %u\n"),
                        (unsigned)hal.util->available_memory());

    cliSerial->println_P(PSTR(HAL_BOARD_NAME));

	DataFlash.LogReadProcess(log_num, start_page, end_page, 
                             print_flight_mode,
                             cliSerial);
}

// start a new log
static void start_logging() 
{
    DataFlash.StartNewLog();
    DataFlash.Log_Write_Message_P(PSTR(FIRMWARE_STRING));
#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
    DataFlash.Log_Write_Message_P(PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

    // write system identifier as well if available
    char sysid[40];
    if (hal.util->get_system_id(sysid)) {
        DataFlash.Log_Write_Message(sysid);
    }
}

#else // LOGGING_ENABLED

// dummy functions
static void Log_Write_Startup(uint8_t type) {}
static void Log_Write_Cmd(uint8_t num, const struct Location *wp) {}
static void Log_Write_Current() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_TECS_Tuning() {}
static void Log_Write_Performance() {}
static void Log_Write_Attitude() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Camera() {}
static void Log_Write_Mode(uint8_t mode) {}
static void Log_Write_Compass() {}
static void Log_Write_GPS() {}
static void Log_Write_IMU() {}
static void Log_Write_RC() {}

static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}


#endif // LOGGING_ENABLED
