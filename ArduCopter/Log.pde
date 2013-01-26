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
    int16_t log_start;
    int16_t log_end;
    int16_t temp;
    int16_t last_log_num = DataFlash.find_last_log();

    uint16_t num_logs = DataFlash.get_num_logs();

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
        if (g.log_bitmask & MASK_LOG_ITERM) cliSerial->printf_P(PSTR(" ITERM"));
        if (g.log_bitmask & MASK_LOG_INAV) cliSerial->printf_P(PSTR(" INAV"));
        if (g.log_bitmask & MASK_LOG_CAMERA) cliSerial->printf_P(PSTR(" CAMERA"));
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
    } else if ((argc != 2) || (dump_log <= (last_log_num - DataFlash.get_num_logs())) || (dump_log > last_log_num)) {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    /*cliSerial->printf_P(PSTR("Dumping Log number %d,    start %d,   end %d\n"),
     *                         dump_log,
     *                         dump_log_start,
     *                         dump_log_end);
     */
    Log_Read(dump_log_start, dump_log_end);
    //cliSerial->printf_P(PSTR("Done\n"));
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
        TARG(ITERM);
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

// Write an GPS packet. Total length : 31 bytes
static void Log_Write_GPS()
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

    // need to fix printf formatting

    cliSerial->printf_P(PSTR("GPS, %ld, %u, "),
                        (long)pkt.gps_time,
                        (unsigned)pkt.num_sats);
    print_latlon(cliSerial, pkt.latitude);
    cliSerial->print_P(PSTR(", "));
    print_latlon(cliSerial, pkt.longitude);
    cliSerial->printf_P(PSTR(", %4.4f, %4.4f, %lu, %ld\n"),
                        pkt.rel_altitude*0.01,
                        pkt.altitude*0.01,
                        (unsigned long)pkt.ground_speed,
                        (long)pkt.ground_course);
}

struct log_IMU {
    LOG_PACKET_HEADER;
    Vector3f gyro;
    Vector3f accel;
};

// Write an imu accel/gyro packet. Total length : 27 bytes
static void Log_Write_IMU()
{
    struct log_IMU pkt = {
        LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
        gyro      : ins.get_gyro(),
        accel     : ins.get_accel()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a raw accel/gyro packet
static void Log_Read_IMU()
{
    struct log_IMU pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                                 1      2      3      4      5      6
    cliSerial->printf_P(PSTR("IMU, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f\n"),
        (float)pkt.gyro.x,
        (float)pkt.gyro.y,
        (float)pkt.gyro.z,
        (float)pkt.accel.x,
        (float)pkt.accel.y,
        (float)pkt.accel.z);
}

struct log_Current {
    LOG_PACKET_HEADER;
    int16_t throttle_in;
    uint32_t throttle_integrator;
    int16_t battery_voltage;
    int16_t current_amps;
    int16_t current_total;
};

// Write an Current data packet. Total length : 16 bytes
static void Log_Write_Current()
{
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        throttle_in         : g.rc_3.control_in,
        throttle_integrator : throttle_integrator,
        battery_voltage     : (int16_t) (battery_voltage1 * 100.0f),
        current_amps        : (int16_t) (current_amps1 * 100.0f),
        current_total       : (int16_t) current_total1
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a Current packet
static void Log_Read_Current()
{
    struct log_Current pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                                  1    2      3      4   5
    cliSerial->printf_P(PSTR("CURRENT, %d, %lu, %4.4f, %4.4f, %d\n"),
                    (int)pkt.throttle_in,
                    (unsigned long)pkt.throttle_integrator,
                    (float)pkt.battery_voltage/100.0f,
                    (float)pkt.current_amps/100.0f,
                    (int)pkt.current_total);
}

struct log_Motors {
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

// Write an Motors packet. Total length : 12 ~ 20 bytes
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

// Read a Motors packet.
static void Log_Read_Motors()
{
    struct log_Motors pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

#if FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME
                                 // 1   2   3   4   5   6   7   8
    cliSerial->printf_P(PSTR("MOT, %d, %d, %d, %d, %d, %d, %d, %d\n"),
                    (int)pkt.motor_out[0],
                    (int)pkt.motor_out[1],
                    (int)pkt.motor_out[2],
                    (int)pkt.motor_out[3],
                    (int)pkt.motor_out[4],
                    (int)pkt.motor_out[5],
                    (int)pkt.motor_out[6],
                    (int)pkt.motor_out[7]);
#elif FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME
                                 // 1   2   3   4   5   6
    cliSerial->printf_P(PSTR("MOT, %d, %d, %d, %d, %d, %d\n"),
                    (int)pkt.motor_out[0],
                    (int)pkt.motor_out[1],
                    (int)pkt.motor_out[2],
                    (int)pkt.motor_out[3],
                    (int)pkt.motor_out[4],
                    (int)pkt.motor_out[5]);
#elif FRAME_CONFIG == HELI_FRAME
                                 // 1   2   3   4   5
    cliSerial->printf_P(PSTR("MOT, %d, %d, %d, %d, %d\n"),
                    (int)pkt.motor_out[0],
                    (int)pkt.motor_out[1],
                    (int)pkt.motor_out[2],
                    (int)pkt.motor_out[3],
                    (int)pkt.ext_gyro_gain);
#else // TRI_FRAME or QUAD_FRAME
                                 // 1   2   3   4
    cliSerial->printf_P(PSTR("MOT, %d, %d, %d, %d\n"),
                    (int)pkt.motor_out[0],
                    (int)pkt.motor_out[1],
                    (int)pkt.motor_out[2],
                    (int)pkt.motor_out[3]);
#endif
}

struct log_Optflow {
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

// Write an optical flow packet. Total length : 30 bytes
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

// Read an optical flow packet.
static void Log_Read_Optflow()
{
    struct log_Optflow pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                             1   2   3   4   5      6      7    8    9
    cliSerial->printf_P(PSTR("OF, %d, %d, %d, %d, %d, %4.7f, %4.7f, %ld, %ld\n"),
                    (int)pkt.dx,
                    (int)pkt.dy,
                    (int)pkt.surface_quality,
                    (int)pkt.x_cm,
                    (int)pkt.y_cm,
                    (float)pkt.latitude,
                    (float)pkt.longitude,
                    (long)pkt.roll,
                    (long)pkt.pitch);
}

struct log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t wp_distance;
    int16_t wp_bearing;
    int16_t lat_error;
    int16_t lon_error;
    int16_t nav_pitch;
    int16_t nav_roll;
    int16_t lat_speed;
    int16_t lon_speed;
};

// Write an Nav Tuning packet. Total length : 24 bytes
static void Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        wp_distance : wp_distance,
        wp_bearing  : (int16_t) (wp_bearing/100),
        lat_error   : (int16_t) lat_error,
        lon_error   : (int16_t) long_error,
        nav_pitch   : (int16_t) nav_pitch,
        nav_roll    : (int16_t) nav_roll,
        lat_speed   : lat_speed,
        lon_speed   : lon_speed
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a Nav Tuning packet.
static void Log_Read_Nav_Tuning()
{
    struct log_Nav_Tuning pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                               1   2   3   4   5   6   7   8
    cliSerial->printf_P(PSTR("NTUN, %lu, %d, %d, %d, %d, %d, %d, %d\n"),
        (unsigned long)pkt.wp_distance,
        (int)pkt.wp_bearing,
        (int)pkt.lat_error,
        (int)pkt.lon_error,
        (int)pkt.nav_pitch,
        (int)pkt.nav_roll,
        (int)pkt.lat_speed,
        (int)pkt.lon_speed
    );
}

struct log_Control_Tuning {
    LOG_PACKET_HEADER;
    int16_t throttle_in;
    int16_t sonar_alt;
    int16_t baro_alt;
    int16_t next_wp_alt;
    int16_t nav_throttle;
    int16_t angle_boost;
    int16_t climb_rate;
    int16_t throttle_out;
    int16_t desired_climb_rate;
};

// Write a control tuning packet. Total length : 26 bytes
static void Log_Write_Control_Tuning()
{
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        throttle_in         : g.rc_3.control_in,
        sonar_alt           : sonar_alt,
        baro_alt            : (int16_t) baro_alt,
        next_wp_alt         : (int16_t) next_WP.alt,
        nav_throttle        : nav_throttle,
        angle_boost         : angle_boost,
        climb_rate          : climb_rate,
        throttle_out        : g.rc_3.servo_out,
        desired_climb_rate  : desired_climb_rate
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read an control tuning packet
static void Log_Read_Control_Tuning()
{
    struct log_Control_Tuning pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                               1   2   3   4   5   6   7   8   9
    cliSerial->printf_P(PSTR("CTUN, %d, %d, %d, %d, %d, %d, %d, %d, %d\n"),
        (int)pkt.throttle_in,
        (int)pkt.sonar_alt,
        (int)pkt.baro_alt,
        (int)pkt.next_wp_alt,
        (int)pkt.nav_throttle,
        (int)pkt.angle_boost,
        (int)pkt.climb_rate,
        (int)pkt.throttle_out,
        (int)pkt.desired_climb_rate
    );
}

struct log_Iterm {
    LOG_PACKET_HEADER;
    int16_t rate_roll;
    int16_t rate_pitch;
    int16_t rate_yaw;
    int16_t accel_throttle;
    int16_t nav_lat;
    int16_t nav_lon;
    int16_t loiter_rate_lat;
    int16_t loiter_rate_lon;
    int16_t throttle_cruise;
};

static void Log_Write_Iterm()
{
    struct log_Iterm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        rate_roll       : (int16_t) g.pid_rate_roll.get_integrator(),
        rate_pitch      : (int16_t) g.pid_rate_pitch.get_integrator(),
        rate_yaw        : (int16_t) g.pid_rate_yaw.get_integrator(),
        accel_throttle  : (int16_t) g.pid_throttle_accel.get_integrator(),
        nav_lat         : (int16_t) g.pid_nav_lat.get_integrator(),
        nav_lon         : (int16_t) g.pid_nav_lon.get_integrator(),
        loiter_rate_lat : (int16_t) g.pid_loiter_rate_lat.get_integrator(),
        loiter_rate_lon : (int16_t) g.pid_loiter_rate_lon.get_integrator(),
        throttle_cruise : (int16_t) g.throttle_cruise
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read an control tuning packet
static void Log_Read_Iterm()
{
    struct log_Iterm pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                                1   2   3   4   5   6   7   8   9
    cliSerial->printf_P(PSTR("ITERM, %d, %d, %d, %d, %d, %d, %d, %d, %d\n"),
        (int)pkt.rate_roll,
        (int)pkt.rate_pitch,
        (int)pkt.rate_yaw,
        (int)pkt.accel_throttle,
        (int)pkt.nav_lat,
        (int)pkt.nav_lon,
        (int)pkt.loiter_rate_lat,
        (int)pkt.loiter_rate_lon,
        (int)pkt.throttle_cruise
    );
    cliSerial->printf_P(PSTR("ITERM, "));
}

struct log_Performance {
    LOG_PACKET_HEADER;
    uint8_t renorm_count;
    uint8_t renorm_blowup;
    uint8_t gps_fix_count;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    uint8_t end;
};

// Write a performance monitoring packet. Total length : 11 bytes
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        renorm_count     : ahrs.renorm_range_count,
        renorm_blowup    : ahrs.renorm_blowup_count,
        gps_fix_count    : gps_fix_count,
        num_long_running : perf_info_get_num_long_running(),
        num_loops        : perf_info_get_num_loops(),
        max_time         : perf_info_get_max_time()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a performance packet
static void Log_Read_Performance()
{
    struct log_Performance pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    //                            1   2   3   4   5    6
    cliSerial->printf_P(PSTR("PM, %u, %u, %u, %u, %u, %lu\n"),
                        (unsigned)pkt.renorm_count,
                        (unsigned)pkt.renorm_blowup,
                        (unsigned)pkt.gps_fix_count,
                        (unsigned)pkt.num_long_running,
                        (unsigned)pkt.num_loops,
                        (unsigned long)pkt.max_time);
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

// Write a command processing packet.  Total length : 21 bytes
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

    //                               1   2   3   4   5    6    7    8
    cliSerial->printf_P(PSTR( "CMD, %u, %u, %u, %u, %u, %ld, %ld, %ld\n"),
                    (unsigned)pkt.command_total,
                    (unsigned)pkt.command_number,
                    (unsigned)pkt.waypoint_id,
                    (unsigned)pkt.waypoint_options,
                    (unsigned)pkt.waypoint_param1,
                    (long)pkt.waypoint_altitude,
                    (long)pkt.waypoint_latitude,
                    (long)pkt.waypoint_longitude);
}

struct log_Attitude {
    LOG_PACKET_HEADER;
    int16_t roll_in;
    int16_t roll;
    int16_t pitch_in;
    int16_t pitch;
    int16_t yaw_in;
    uint16_t yaw;
    uint16_t nav_yaw;
};

// Write an attitude packet. Total length : 16 bytes
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

// Read an attitude packet
static void Log_Read_Attitude()
{
    struct log_Attitude pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                              1   2   3    4   5   6  7
    cliSerial->printf_P(PSTR("ATT, %d, %d, %d, %d, %d, %u, %u\n"),
                    (int)pkt.roll_in,
                    (int)pkt.roll,
                    (int)pkt.pitch_in,
                    (int)pkt.pitch,
                    (int)pkt.yaw_in,
                    (unsigned)pkt.yaw,
                    (unsigned)pkt.nav_yaw);
}

struct log_INAV {
    LOG_PACKET_HEADER;
    int16_t baro_alt;
    int16_t inav_alt;
    int16_t baro_climb_rate;
    int16_t inav_climb_rate;
    float   accel_corr_x;
    float   accel_corr_y;
    float   accel_corr_z;
    float   accel_corr_ef_z;
    int32_t gps_lat_from_home;
    int32_t gps_lon_from_home;
    float   inav_lat_from_home;
    float   inav_lon_from_home;
    float   inav_lat_speed;
    float   inav_lon_speed;
};

// Write an INAV packet. Total length : 52 Bytes
static void Log_Write_INAV()
{
#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
    Vector3f accel_corr = inertial_nav.accel_correction_ef;

    struct log_INAV pkt = {
        LOG_PACKET_HEADER_INIT(LOG_INAV_MSG),
        baro_alt            : (int16_t)baro_alt,                        // 1 barometer altitude
        inav_alt            : (int16_t)inertial_nav.get_altitude(),     // 2 accel + baro filtered altitude
        baro_climb_rate     : baro_rate,                                // 3 barometer based climb rate
        inav_climb_rate     : (int16_t)inertial_nav.get_velocity_z(),   // 4 accel + baro based climb rate
        accel_corr_x        : accel_corr.x,                             // 5 accel correction x-axis
        accel_corr_y        : accel_corr.y,                             // 6 accel correction y-axis
        accel_corr_z        : accel_corr.z,                             // 7 accel correction z-axis
        accel_corr_ef_z     : inertial_nav.accel_correction_ef.z,       // 8 accel correction earth frame
        gps_lat_from_home   : g_gps->latitude-home.lat,                 // 9 lat from home
        gps_lon_from_home   : g_gps->longitude-home.lng,                // 10 lon from home
        inav_lat_from_home  : inertial_nav.get_latitude_diff(),         // 11 accel based lat from home
        inav_lon_from_home  : inertial_nav.get_longitude_diff(),        // 12 accel based lon from home
        inav_lat_speed      : inertial_nav.get_latitude_velocity(),     // 13 accel based lat velocity
        inav_lon_speed      : inertial_nav.get_longitude_velocity()     // 14 accel based lon velocity
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#endif
}

// Read an INAV packet
static void Log_Read_INAV()
{
    struct log_INAV pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

                                  // 1   2   3   4      5      6      7      8    9   10     11     12     13     14
    cliSerial->printf_P(PSTR("INAV, %d, %d, %d, %d, %6.4f, %6.4f, %6.4f, %6.4f, %ld, %ld, %6.4f, %6.4f, %6.4f, %6.4f\n"),
                    (int)pkt.baro_alt,                  // 1 barometer altitude
                    (int)pkt.inav_alt,                  // 2 accel + baro filtered altitude
                    (int)pkt.baro_climb_rate,           // 3 barometer based climb rate
                    (int)pkt.inav_climb_rate,           // 4 accel + baro based climb rate
                    (float)pkt.accel_corr_x,            // 5 accel correction x-axis
                    (float)pkt.accel_corr_y,            // 6 accel correction y-axis
                    (float)pkt.accel_corr_z,            // 7 accel correction z-axis
                    (float)pkt.accel_corr_ef_z,         // 8 accel correction earth frame
                    (long)pkt.gps_lat_from_home,        // 9 lat from home
                    (long)pkt.gps_lon_from_home,        // 10 lon from home
                    (float)pkt.inav_lat_from_home,      // 11 accel based lat from home
                    (float)pkt.inav_lon_from_home,      // 12 accel based lon from home
                    (float)pkt.inav_lat_speed,          // 13 accel based lat velocity
                    (float)pkt.inav_lon_speed);         // 14 accel based lon velocity
}

struct log_Mode {
    LOG_PACKET_HEADER;
    uint8_t mode;
    int16_t throttle_cruise;
};

// Write a mode packet. Total length : 7 bytes
static void Log_Write_Mode(uint8_t mode)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        mode            : mode,
        throttle_cruise : g.throttle_cruise,
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
    cliSerial->printf_P(PSTR(", %d\n"),(int)pkt.throttle_cruise);
}

struct log_Startup {
    LOG_PACKET_HEADER;
};

// Write Startup packet. Total length : 4 bytes
static void Log_Write_Startup()
{
    DataFlash.WriteByte(LOG_STARTUP_MSG);
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a startup packet
static void Log_Read_Startup()
{
    struct log_Startup pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("START UP\n"));
}

struct log_Event {
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

// Read an event packet
static void Log_Read_Event()
{
    struct log_Event pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("EV, %u\n"), (unsigned)pkt.id);
}

struct log_Data_Int16t {
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

// Read an int16_t data packet
static void Log_Read_Int16t()
{
    struct log_Data_Int16t pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("DATA, %u, %d\n"), (unsigned)pkt.id, (int)pkt.data_value);
}

struct log_Data_UInt16t {
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

// Read an uint16_t data packet
static void Log_Read_UInt16t()
{
    struct log_Data_UInt16t pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("DATA, %u, %u\n"), (unsigned)pkt.id, (unsigned)pkt.data_value);
}

struct log_Data_Int32t {
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

// Read an int32_t data packet
static void Log_Read_Int32t()
{
    struct log_Data_Int32t pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("DATA, %u, %ld\n"), (unsigned)pkt.id, (long)pkt.data_value);
}

struct log_Data_UInt32t {
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

// Read a uint32_t data packet
static void Log_Read_UInt32t()
{
    struct log_Data_UInt32t pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("DATA, %u, %lu\n"), (unsigned)pkt.id, (unsigned long)pkt.data_value);
}

struct log_Data_Float {
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

// Read a float data packet
static void Log_Read_Float()
{
    struct log_Data_Float pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("DATA, %u, %1.6f\n"), (unsigned)pkt.id, (float)pkt.data_value);
}

struct log_PID {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t error;
    int32_t p;
    int32_t i;
    int32_t d;
    int32_t output;
    float  gain;
};

// Write an PID packet. Total length : 28 bytes
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

// Read a PID packet
static void Log_Read_PID()
{
    struct log_PID pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    //                             1    2    3    4    5    6      7
    cliSerial->printf_P(PSTR("PID-%u, %ld, %ld, %ld, %ld, %ld, %4.4f\n"),
                    (unsigned)pkt.id,
                    (long)pkt.error,
                    (long)pkt.p,
                    (long)pkt.i,
                    (long)pkt.d,
                    (long)pkt.output,
                    (float)pkt.gain);
}

struct log_DMP {
    LOG_PACKET_HEADER;
    int16_t  dcm_roll;
    int16_t  dmp_roll;
    int16_t  dcm_pitch;
    int16_t  dmp_pitch;
    uint16_t dcm_yaw;
    uint16_t dmp_yaw;
};

// Write a DMP attitude packet. Total length : 16 bytes
static void Log_Write_DMP()
{
#if SECONDARY_DMP_ENABLED == ENABLED
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
#endif
}

// Read a DMP attitude packet
static void Log_Read_DMP()
{
    struct log_DMP pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

                                 // 1   2   3   4   5   6
    cliSerial->printf_P(PSTR("DMP, %d, %d, %d, %d, %u, %u\n"),
                    (int)pkt.dcm_roll,
                    (int)pkt.dmp_roll,
                    (int)pkt.dcm_pitch,
                    (int)pkt.dmp_pitch,
                    (unsigned)pkt.dcm_yaw,
                    (unsigned)pkt.dmp_yaw);
}

struct log_Camera {
    LOG_PACKET_HEADER;
    uint32_t gps_time;
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

// Read a camera packet
static void Log_Read_Camera()
{
    struct log_Camera pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
                                     // 1
    cliSerial->printf_P(PSTR("CAMERA, %lu, "),(unsigned long)pkt.gps_time); // 1 time
    print_latlon(cliSerial, pkt.latitude);              // 2 lat
    cliSerial->print_P(PSTR(", "));
    print_latlon(cliSerial, pkt.longitude);             // 3 lon
                               // 4   5   6   7
    cliSerial->printf_P(PSTR(", %ld, %d, %d, %u\n"),
                    (long)pkt.altitude,                 // 4 altitude
                    (int)pkt.roll,                      // 5 roll in centidegrees
                    (int)pkt.pitch,                     // 6 pitch in centidegrees
                    (unsigned)pkt.yaw);                 // 7 yaw in centidegrees
}

struct log_Error {
    LOG_PACKET_HEADER;
    uint8_t sub_system;
    uint8_t error_code;
};

// Write an error packet. Total length : 5 bytes
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read an error packet
static void Log_Read_Error()
{
    struct log_Error pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));

    cliSerial->print_P(PSTR("ERR, "));

    // print subsystem
    switch(pkt.sub_system) {
        case ERROR_SUBSYSTEM_MAIN:
            cliSerial->print_P(PSTR("MAIN"));
            break;
        case ERROR_SUBSYSTEM_RADIO:
            cliSerial->print_P(PSTR("RADIO"));
            break;
        case ERROR_SUBSYSTEM_COMPASS:
            cliSerial->print_P(PSTR("COM"));
            break;
        case ERROR_SUBSYSTEM_OPTFLOW:
            cliSerial->print_P(PSTR("OF"));
            break;
        case ERROR_SUBSYSTEM_FAILSAFE:
            cliSerial->print_P(PSTR("FS"));
            break;
        default:
            // if undefined print subsytem as a number
            cliSerial->printf_P(PSTR("%u"),(unsigned)pkt.sub_system);
            break;
    }

    // print error code
    cliSerial->printf_P(PSTR(", %u\n"),(unsigned)pkt.error_code);
}


// Read the DataFlash log memory
static void Log_Read(int16_t start_page, int16_t end_page)
{
 #ifdef AIRFRAME_NAME
    cliSerial->printf_P(PSTR((AIRFRAME_NAME)));
 #endif

    cliSerial->printf_P(PSTR("\n" THISFIRMWARE
                             "\nFree RAM: %u\n"),
                        (unsigned) memcheck_available_memory());

 #if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    cliSerial->printf_P(PSTR("APM 1\n"));
 #elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
    cliSerial->printf_P(PSTR("APM 2\n"));
 #endif

#if CLI_ENABLED == ENABLED
	setup_show(0, NULL);
#endif

    DataFlash.log_read_process(start_page, end_page, log_callback);
}

// read one packet from the dataflash
static void log_callback(uint8_t msgid)
{
    switch(msgid) {
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
        
    case LOG_MOTORS_MSG:
        Log_Read_Motors();
        break;
        
    case LOG_OPTFLOW_MSG:
        Log_Read_Optflow();
        break;
        
    case LOG_GPS_MSG:
        Log_Read_GPS();
        break;
        
    case LOG_EVENT_MSG:
        Log_Read_Event();
        break;
        
    case LOG_PID_MSG:
        Log_Read_PID();
        break;
        
    case LOG_ITERM_MSG:
        Log_Read_Iterm();
        break;
        
    case LOG_DMP_MSG:
        Log_Read_DMP();
        break;
        
    case LOG_INAV_MSG:
        Log_Read_INAV();
        break;
        
    case LOG_CAMERA_MSG:
        Log_Read_Camera();
        break;
        
    case LOG_ERROR_MSG:
        Log_Read_Error();
        break;
        
    case LOG_DATA_INT16_MSG:
        Log_Read_Int16t();
        break;
        
    case LOG_DATA_UINT16_MSG:
        Log_Read_UInt16t();
        break;

    case LOG_DATA_INT32_MSG:
        Log_Read_Int32t();
        break;
        
    case LOG_DATA_UINT32_MSG:
        Log_Read_UInt32t();
        break;
        
    case LOG_DATA_FLOAT_MSG:
        Log_Read_Float();
        break;
    }
}


#else // LOGGING_ENABLED

void print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon) {}
static void Log_Write_Startup() {}
static void Log_Write_Cmd(uint8_t num, struct Location *wp) {}
static void Log_Write_Mode(uint8_t mode) {}
static void Log_Write_IMU() {}
static void Log_Write_GPS() {}
static void Log_Write_Current() {}
static void Log_Write_Iterm() {}
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
static void Log_Write_DMP() {}
static void Log_Write_Camera() {}
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}

#endif // LOGGING_DISABLED
