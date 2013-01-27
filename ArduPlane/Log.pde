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
    int16_t log_start;
    int16_t log_end;
    int16_t temp;
    int16_t last_log_num = DataFlash.find_last_log();

    uint16_t num_logs = DataFlash.get_num_logs();

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
 #undef PLOG
    }

    cliSerial->println();

    if (num_logs == 0) {
        cliSerial->printf_P(PSTR("\nNo logs\n\n"));
    }else{
        cliSerial->printf_P(PSTR("\n%u logs\n"), (unsigned)num_logs);

        for(int16_t i=num_logs; i>=1; i--) {
            int16_t last_log_start = log_start, last_log_end = log_end;
            temp = last_log_num-i+1;
            DataFlash.get_log_boundaries(temp, log_start, log_end);
            cliSerial->printf_P(PSTR("Log %d,    start %d,   end %d\n"), (int)temp, (int)log_start, (int)log_end);
            if (last_log_start == log_start && last_log_end == log_end) {
                // we are printing bogus logs
                break;
            }
        }
        cliSerial->println();
    }
    return(true);
}

static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    int16_t dump_log_start;
    int16_t dump_log_end;
    int16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        for(uint16_t count=1; count<=DataFlash.df_NumPages; count++) {
            DataFlash.StartRead(count);
            cliSerial->printf_P(PSTR("DF page, log file #, log page: %d,\t"), (int)count);
            cliSerial->printf_P(PSTR("%d,\t"), (int)DataFlash.GetFileNumber());
            cliSerial->printf_P(PSTR("%d\n"), (int)DataFlash.GetFilePage());
        }
        return(-1);
    } else if (dump_log <= 0) {
        cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(1, DataFlash.df_NumPages);
        return(-1);
    } else if ((argc != 2)
        || (dump_log <= (last_log_num - DataFlash.get_num_logs()))
        || (dump_log > last_log_num))
    {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    cliSerial->printf_P(PSTR("Dumping Log %d,    start pg %d,   end pg %d\n"),
                    (int)dump_log,
                    (int)dump_log_start,
                    (int)dump_log_end);

    Log_Read(dump_log_start, dump_log_end);
    cliSerial->printf_P(PSTR("Done\n"));
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

// print_latlon - prints an latitude or longitude value held in an int32_t
// probably this should be moved to AP_Common
void print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon)
{
    int32_t dec_portion, frac_portion;
    int32_t abs_lat_or_lon = labs(lat_or_lon);

    // extract decimal portion (special handling of negative numbers to ensure we round towards zero)
    dec_portion = abs_lat_or_lon / T7;

    // extract fractional portion
    frac_portion = abs_lat_or_lon - dec_portion*T7;

    // print output including the minus sign
    if( lat_or_lon < 0 ) {
        s->printf_P(PSTR("-"));
    }
    s->printf_P(PSTR("%ld.%07ld"),(long)dec_portion,(long)frac_portion);
}

struct log_Attitute {
    LOG_PACKET_HEADER;
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
};

// Write an attitude packet. Total length : 10 bytes
static void Log_Write_Attitude(void)
{
    struct log_Attitute pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        roll  : ahrs.roll_sensor,
        pitch : ahrs.pitch_sensor,
        yaw   : ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read an attitude packet
static void Log_Read_Attitude()
{
    struct log_Attitute pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("ATT, %ld, %ld, %ld\n"),
                        (long)pkt.roll, 
                        (long)pkt.pitch,
                        (long)pkt.yaw);
}

struct log_Performance {
    LOG_PACKET_HEADER;
    uint32_t loop_time;
    uint16_t main_loop_count;
    int16_t  g_dt_max;
    uint8_t  renorm_count;
    uint8_t  renorm_blowup;
    uint8_t  gps_fix_count;
    int16_t  gyro_drift_x;
    int16_t  gyro_drift_y;
    int16_t  gyro_drift_z;
    int16_t  pm_test;
};

// Write a performance monitoring packet. Total length : 19 bytes
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        loop_time       : millis()- perf_mon_timer,
        main_loop_count : mainLoop_count,
        g_dt_max        : G_Dt_max,
        renorm_count    : ahrs.renorm_range_count,
        renorm_blowup   : ahrs.renorm_blowup_count,
        gps_fix_count   : gps_fix_count,
        gyro_drift_x    : (int16_t)(ahrs.get_gyro_drift().x * 1000),
        gyro_drift_y    : (int16_t)(ahrs.get_gyro_drift().y * 1000),
        gyro_drift_z    : (int16_t)(ahrs.get_gyro_drift().z * 1000),
        pm_test         : pmTest1
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a performance packet
static void Log_Read_Performance()
{
    struct log_Performance pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    cliSerial->printf_P(PSTR("PM, %lu, %u, %d, %u, %u, %u, %d, %d, %d, %d\n"),
            pkt.loop_time,
            (unsigned)pkt.main_loop_count,
            (int)pkt.g_dt_max,
            (unsigned)pkt.renorm_count,
            (unsigned)pkt.renorm_blowup,
            (unsigned)pkt.gps_fix_count,
            (int)pkt.gyro_drift_x,
            (int)pkt.gyro_drift_y,
            (int)pkt.gyro_drift_z,
            (int)pkt.pm_test);
}

struct log_Cmd {
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
static void Log_Write_Cmd(uint8_t num, struct Location *wp)
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

// Read a command processing packet
static void Log_Read_Cmd()
{
    struct log_Cmd pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    cliSerial->printf_P(PSTR("CMD, %u, %u, %u, %u, %u, %ld, %ld, %ld\n"),
        (unsigned)pkt.command_total,
        (unsigned)pkt.command_number,
        (unsigned)pkt.waypoint_id,
        (unsigned)pkt.waypoint_options,
        (unsigned)pkt.waypoint_param1,
        (long)pkt.waypoint_altitude,
        (long)pkt.waypoint_latitude,
        (long)pkt.waypoint_longitude);
}

struct log_Startup {
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

static void Log_Read_Startup()
{
    struct log_Startup pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    switch( pkt.startup_type ) {
        case TYPE_AIRSTART_MSG:
            cliSerial->printf_P(PSTR("AIR START"));
            break;
        case TYPE_GROUNDSTART_MSG:
            cliSerial->printf_P(PSTR("GROUND START"));
            break;
        default:
            cliSerial->printf_P(PSTR("UNKNOWN STARTUP"));
            break;
    }

    cliSerial->printf_P(PSTR(" - %u commands in memory\n"),(unsigned)pkt.command_total);
}

struct log_Control_Tuning {
    LOG_PACKET_HEADER;
    int16_t roll_out;
    int16_t nav_roll_cd;
    int16_t roll;
    int16_t pitch_out;
    int16_t nav_pitch_cd;
    int16_t pitch;
    int16_t throttle_out;
    int16_t rudder_out;
    int16_t accel_y;
};

// Write a control tuning packet. Total length : 22 bytes
static void Log_Write_Control_Tuning()
{
    Vector3f accel = ins.get_accel();
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        roll_out        : (int16_t)g.channel_roll.servo_out,
        nav_roll_cd     : (int16_t)nav_roll_cd,
        roll            : (int16_t)ahrs.roll_sensor,
        pitch_out       : (int16_t)g.channel_pitch.servo_out,
        nav_pitch_cd    : (int16_t)nav_pitch_cd,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)g.channel_throttle.servo_out,
        rudder_out      : (int16_t)g.channel_rudder.servo_out,
        accel_y         : (int16_t)(accel.y * 10000)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read an control tuning packet
static void Log_Read_Control_Tuning()
{
    struct log_Control_Tuning pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    cliSerial->printf_P(PSTR("CTUN, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.4f\n"),
        (float)pkt.roll_out / 100.f,
        (float)pkt.nav_roll_cd / 100.f,
        (float)pkt.roll / 100.f,
        (float)pkt.pitch_out / 100.f,
        (float)pkt.nav_pitch_cd / 100.f,
        (float)pkt.pitch / 100.f,
        (float)pkt.throttle_out / 100.f,
        (float)pkt.rudder_out / 100.f,
        (float)pkt.accel_y / 10000.f
    );
}

struct log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint16_t yaw;
    uint32_t wp_distance;
    uint16_t target_bearing_cd;
    uint16_t nav_bearing_cd;
    int16_t altitude_error_cm;
    int16_t airspeed_cm;
};

// Write a navigation tuning packet. Total length : 18 bytes
static void Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        yaw                 : (uint16_t)ahrs.yaw_sensor,
        wp_distance         : wp_distance,
        target_bearing_cd   : (uint16_t)target_bearing_cd,
        nav_bearing_cd      : (uint16_t)nav_bearing_cd,
        altitude_error_cm   : (int16_t)altitude_error_cm,
        airspeed_cm         : (int16_t)airspeed.get_airspeed_cm()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a nav tuning packet
static void Log_Read_Nav_Tuning()
{
    struct log_Nav_Tuning pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    cliSerial->printf_P(PSTR("NTUN, %4.4f, %lu, %4.4f, %4.4f, %4.4f, %4.4f\n"),
                    (float)pkt.yaw/100.0f,
                    (unsigned long)pkt.wp_distance,
                    (float)(pkt.target_bearing_cd/100.0f),
                    (float)(pkt.nav_bearing_cd/100.0f),
                    (float)(pkt.altitude_error_cm/100.0f),
                    (float)(pkt.airspeed_cm/100.0f));
}

struct log_Mode {
    LOG_PACKET_HEADER;
    uint8_t mode;
};

// Write a mode packet. Total length : 5 bytes
static void Log_Write_Mode(uint8_t mode)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        mode : mode
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a mode packet
static void Log_Read_Mode()
{
    struct log_Mode pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("MOD,"));
    print_flight_mode(pkt.mode);
}

struct log_GPS {
    LOG_PACKET_HEADER;
    uint32_t gps_time;
    uint8_t  num_sats;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  rel_altitude;
    int32_t  altitude;
    uint32_t ground_speed;
    int32_t  ground_course;
};

// Write an GPS packet. Total length : 30 bytes
static void Log_Write_GPS(void)
{
    struct log_GPS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GPS_MSG),
    	gps_time      : g_gps->time,
        num_sats      : g_gps->num_sats,
        latitude      : g_gps->latitude,
        longitude     : g_gps->longitude,
        rel_altitude  : current_loc.alt,
        altitude      : g_gps->altitude,
        ground_speed  : g_gps->ground_speed,
        ground_course : g_gps->ground_course
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a GPS packet
static void Log_Read_GPS()
{
    struct log_GPS pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("GPS, %ld, %u, "),
                        (long)pkt.gps_time,
                        (unsigned)pkt.num_sats);
    print_latlon(cliSerial, pkt.latitude);
    cliSerial->print_P(PSTR(", "));
    print_latlon(cliSerial, pkt.longitude);
    cliSerial->printf_P(PSTR(", %4.4f, %4.4f, %lu, %ld\n"),
                        (float)pkt.rel_altitude*0.01,
                        (float)pkt.altitude*0.01,
                        (unsigned long)pkt.ground_speed,
                        (long)pkt.ground_course);
}

struct log_IMU {
    LOG_PACKET_HEADER;
    Vector3f gyro;
    Vector3f accel;
};

// Write an raw accel/gyro data packet. Total length : 28 bytes
static void Log_Write_IMU()
{
    struct log_IMU pkt = {
        LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
        gyro  : ins.get_gyro(),
        accel : ins.get_accel()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a raw accel/gyro packet
static void Log_Read_IMU()
{
    struct log_IMU pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("IMU, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f\n"),
                        pkt.gyro.x,
                        pkt.gyro.y,
                        pkt.gyro.z,
                        pkt.accel.x,
                        pkt.accel.y,
                        pkt.accel.z);
}

struct log_Current {
    LOG_PACKET_HEADER;
    int16_t throttle_in;
    int16_t battery_voltage;
    int16_t current_amps;
    int16_t current_total;
};

static void Log_Write_Current()
{
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        throttle_in             : g.channel_throttle.control_in,
        battery_voltage         : (int16_t)(battery_voltage1 * 100.0),
        current_amps            : (int16_t)(current_amps1 * 100.0),
        current_total           : (int16_t)current_total1
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a Current packet
static void Log_Read_Current()
{
    struct log_Current pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("CURRENT, %d, %4.4f, %4.4f, %d\n"),
                    (int)pkt.throttle_in,
                    ((float)pkt.battery_voltage / 100.f),
                    ((float)pkt.current_amps / 100.f),
                    (int)pkt.current_total);
}

// Read the DataFlash.log memory : Packet Parser
static void Log_Read(int16_t start_page, int16_t end_page)
{
    cliSerial->printf_P(PSTR("\n" THISFIRMWARE
                             "\nFree RAM: %u\n"),
                        memcheck_available_memory());

    DataFlash.log_read_process(start_page, end_page, log_callback);
}

// Read the DataFlash.log memory : Packet Parser
static void log_callback(uint8_t msgid)
{
    switch (msgid) {
    case LOG_ATTITUDE_MSG:
        Log_Read_Attitude();
        break;
    case LOG_MODE_MSG:
        Log_Read_Mode();
        break;
    case LOG_CONTROL_TUNING_MSG:
        Log_Read_Control_Tuning();
        break;
    case LOG_NAV_TUNING_MSG:
        Log_Read_Nav_Tuning();
        break;
    case LOG_PERFORMANCE_MSG:
        Log_Read_Performance();
        break;
    case LOG_IMU_MSG:
        Log_Read_IMU();
        break;
    case LOG_CMD_MSG:
        Log_Read_Cmd();
        break;
    case LOG_CURRENT_MSG:
        Log_Read_Current();
        break;
    case LOG_STARTUP_MSG:
        Log_Read_Startup();
        break;
    case LOG_GPS_MSG:
        Log_Read_GPS();
        break;
    }
}

                        
#else // LOGGING_ENABLED

// dummy functions
static void Log_Write_Mode(uint8_t mode) {}
static void Log_Write_Startup(uint8_t type) {}
static void Log_Write_Cmd(uint8_t num, struct Location *wp) {}
static void Log_Write_Current() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_GPS() {}
static void Log_Write_Performance() {}
static void Log_Write_Attitude() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_IMU() {}

static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}


#endif // LOGGING_ENABLED
