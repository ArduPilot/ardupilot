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
        PLOG(RAW);
        PLOG(CMD);
        PLOG(CUR);
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
        TARG(RAW);
        TARG(CMD);
        TARG(CUR);
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
    cliSerial->printf_P(PSTR("ATT: %ld, %ld, %ld\n"),
                        (long)pkt.roll, 
                        (long)pkt.pitch,
                        (long)pkt.yaw);
}


// Write a performance monitoring packet. Total length : 19 bytes
static void Log_Write_Performance()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_PERFORMANCE_MSG);
    DataFlash.WriteLong(millis()- perf_mon_timer);
    DataFlash.WriteInt((int16_t)mainLoop_count);
    DataFlash.WriteInt(G_Dt_max);
    DataFlash.WriteByte(0);
    DataFlash.WriteByte(0);
    DataFlash.WriteByte(ahrs.renorm_range_count);
    DataFlash.WriteByte(ahrs.renorm_blowup_count);
    DataFlash.WriteByte(gps_fix_count);
    DataFlash.WriteInt(1);     // AHRS health
    DataFlash.WriteInt((int)(ahrs.get_gyro_drift().x * 1000));
    DataFlash.WriteInt((int)(ahrs.get_gyro_drift().y * 1000));
    DataFlash.WriteInt((int)(ahrs.get_gyro_drift().z * 1000));
    DataFlash.WriteInt(pmTest1);
}

// Write a command processing packet. Total length : 19 bytes
//void Log_Write_Cmd(byte num, byte id, byte p1, int32_t alt, int32_t lat, int32_t lng)
static void Log_Write_Cmd(uint8_t num, struct Location *wp)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_CMD_MSG);
    DataFlash.WriteByte(num);
    DataFlash.WriteByte(wp->id);
    DataFlash.WriteByte(wp->p1);
    DataFlash.WriteLong(wp->alt);
    DataFlash.WriteLong(wp->lat);
    DataFlash.WriteLong(wp->lng);
}

static void Log_Write_Startup(uint8_t type)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_STARTUP_MSG);
    DataFlash.WriteByte(type);
    DataFlash.WriteByte(g.command_total);

    // create a location struct to hold the temp Waypoints for printing
    struct Location cmd = get_cmd_with_index(0);
    Log_Write_Cmd(0, &cmd);

    for (int16_t i = 1; i <= g.command_total; i++) {
        cmd = get_cmd_with_index(i);
        Log_Write_Cmd(i, &cmd);
    }
}


// Write a control tuning packet. Total length : 22 bytes
static void Log_Write_Control_Tuning()
{
    Vector3f accel = ins.get_accel();

    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_CONTROL_TUNING_MSG);
    DataFlash.WriteInt(g.channel_roll.servo_out);
    DataFlash.WriteInt(nav_roll_cd);
    DataFlash.WriteInt((int)ahrs.roll_sensor);
    DataFlash.WriteInt((int)(g.channel_pitch.servo_out));
    DataFlash.WriteInt((int)nav_pitch_cd);
    DataFlash.WriteInt((int)ahrs.pitch_sensor);
    DataFlash.WriteInt((int)(g.channel_throttle.servo_out));
    DataFlash.WriteInt((int)(g.channel_rudder.servo_out));
    DataFlash.WriteInt((int)(accel.y * 10000));
}

// Write a navigation tuning packet. Total length : 18 bytes
static void Log_Write_Nav_Tuning()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_NAV_TUNING_MSG);
    DataFlash.WriteInt((uint16_t)ahrs.yaw_sensor);
    DataFlash.WriteInt((int16_t)wp_distance);
    DataFlash.WriteInt(target_bearing_cd);
    DataFlash.WriteInt(nav_bearing_cd);
    DataFlash.WriteInt(altitude_error_cm);
    DataFlash.WriteInt((int16_t)airspeed.get_airspeed_cm());
    DataFlash.WriteInt(0);     // was nav_gain_scaler
}

// Write a mode packet. Total length : 5 bytes
static void Log_Write_Mode(uint8_t mode)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_MODE_MSG);
    DataFlash.WriteByte(mode);
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
    cliSerial->printf_P(PSTR("GPS, %ld, %u, %.7f, %.7f, %4.4f, %4.4f, %d, %ld\n"),
                        (long)pkt.gps_time,
                        (unsigned)pkt.num_sats,
                        pkt.latitude*1.0e-7,
                        pkt.longitude*1.0e-7,
                        pkt.rel_altitude*0.01,
                        pkt.altitude*0.01,
                        (unsigned long)pkt.ground_speed,
                        (long)pkt.ground_course);
}

struct log_Raw {
    LOG_PACKET_HEADER;
    Vector3f gyro;
    Vector3f accel;
};

// Write an raw accel/gyro data packet. Total length : 28 bytes
static void Log_Write_Raw()
{
    struct log_Raw pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RAW_MSG),
        gyro  : ins.get_gyro(),
        accel : ins.get_accel()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Read a raw accel/gyro packet
static void Log_Read_Raw()
{
    struct log_Raw pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    cliSerial->printf_P(PSTR("RAW: %f, %f, %f, %f, %f, %f\n"),
                        pkt.gyro.x,
                        pkt.gyro.y,
                        pkt.gyro.z,
                        pkt.accel.x,
                        pkt.accel.y,
                        pkt.accel.z);
}


static void Log_Write_Current()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_CURRENT_MSG);
    DataFlash.WriteInt(g.channel_throttle.control_in);
    DataFlash.WriteInt((int)(battery_voltage1       * 100.0));
    DataFlash.WriteInt((int)(current_amps1          * 100.0));
    DataFlash.WriteInt((int)current_total1);
}

// Read a Current packet
static void Log_Read_Current()
{
    cliSerial->printf_P(PSTR("CURR: %d, %4.4f, %4.4f, %d\n"),
                    (int)DataFlash.ReadInt(),
                    ((float)DataFlash.ReadInt() / 100.f),
                    ((float)DataFlash.ReadInt() / 100.f),
                    (int)DataFlash.ReadInt());
}

// Read an control tuning packet
static void Log_Read_Control_Tuning()
{
    float logvar;

    cliSerial->printf_P(PSTR("CTUN:"));
    for (int16_t y = 1; y < 10; y++) {
        logvar = DataFlash.ReadInt();
        if(y < 8) logvar        = logvar/100.f;
        if(y == 9) logvar       = logvar/10000.f;
        cliSerial->print(logvar);
        print_comma();
    }
    cliSerial->println();
}

// Read a nav tuning packet
static void Log_Read_Nav_Tuning()
{
    int16_t d[7];
    for (int8_t i=0; i<7; i++) {
        d[i] = DataFlash.ReadInt();
    }
    cliSerial->printf_P(PSTR("NTUN: %4.4f, %d, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f,\n"),              // \n
                    d[0]/100.0,
                    (int)d[1],
                    ((uint16_t)d[2])/100.0,
                    ((uint16_t)d[3])/100.0,
                    d[4]/100.0,
                    d[5]/100.0,
                    d[5]/1000.0);
}

// Read a performance packet
static void Log_Read_Performance()
{
    int32_t pm_time;
    int16_t logvar;

    cliSerial->printf_P(PSTR("PM:"));
    pm_time = DataFlash.ReadLong();
    cliSerial->print(pm_time);
    print_comma();
    for (int16_t y = 1; y <= 12; y++) {
        if(y < 3 || y > 7) {
            logvar = DataFlash.ReadInt();
        }else{
            logvar = DataFlash.ReadByte();
        }
        cliSerial->print(logvar);
        print_comma();
    }
    cliSerial->println();
}

// Read a command processing packet
static void Log_Read_Cmd()
{
    uint8_t logvarb;
    int32_t logvarl;

    cliSerial->printf_P(PSTR("CMD:"));
    for(int16_t i = 1; i < 4; i++) {
        logvarb = DataFlash.ReadByte();
        cliSerial->print(logvarb, DEC);
        print_comma();
    }
    for(int16_t i = 1; i < 4; i++) {
        logvarl = DataFlash.ReadLong();
        cliSerial->print(logvarl, DEC);
        print_comma();
    }
    cliSerial->println();
}

static void Log_Read_Startup()
{
    uint8_t logbyte = DataFlash.ReadByte();

    if (logbyte == TYPE_AIRSTART_MSG)
        cliSerial->printf_P(PSTR("AIR START - "));
    else if (logbyte == TYPE_GROUNDSTART_MSG)
        cliSerial->printf_P(PSTR("GROUND START - "));
    else
        cliSerial->printf_P(PSTR("UNKNOWN STARTUP - "));

    cliSerial->printf_P(PSTR(" %d commands in memory\n"),(int)DataFlash.ReadByte());
}

// Read a mode packet
static void Log_Read_Mode()
{
    cliSerial->printf_P(PSTR("MOD:"));
    print_flight_mode(DataFlash.ReadByte());
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
    case LOG_RAW_MSG:
        Log_Read_Raw();
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
static void Log_Write_Raw() {}

static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}


#endif // LOGGING_ENABLED
