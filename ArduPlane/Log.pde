// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

 #define HEAD_BYTE1      0xA3   // Decimal 163
 #define HEAD_BYTE2      0x95   // Decimal 149
 #define END_BYTE        0xBA   // Decimal 186


// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
//static int8_t	help_log(uint8_t argc,          const Menu::arg *argv)
/*{
 *       Serial.printf_P(PSTR("\n"
 *                                                "Commands:\n"
 *                                                "  dump <n>"
 *                                                "  erase (all logs)\n"
 *                                                "  enable <name> | all\n"
 *                                                "  disable <name> | all\n"
 *                                                "\n"));
 *   return 0;
 *  }*/

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

    Serial.printf_P(PSTR("logs enabled: "));

    if (0 == g.log_bitmask) {
        Serial.printf_P(PSTR("none"));
    }else{
        // Macro to make the following code a bit easier on the eye.
        // Pass it the capitalised name of the log option, as defined
        // in defines.h but without the LOG_ prefix.  It will check for
        // the bit being set and print the name of the log option to suit.
 #define PLOG(_s)        if (g.log_bitmask & MASK_LOG_ ## _s) Serial.printf_P(PSTR(" %S"), PSTR(# _s))
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

    Serial.println();

    if (num_logs == 0) {
        Serial.printf_P(PSTR("\nNo logs\n\n"));
    }else{
        Serial.printf_P(PSTR("\n%d logs\n"), (int)num_logs);

        for(int16_t i=num_logs; i>=1; i--) {
            int16_t last_log_start = log_start, last_log_end = log_end;
            temp = last_log_num-i+1;
            DataFlash.get_log_boundaries(temp, log_start, log_end);
            Serial.printf_P(PSTR("Log %d,    start %d,   end %d\n"), (int)temp, (int)log_start, (int)log_end);
            if (last_log_start == log_start && last_log_end == log_end) {
                // we are printing bogus logs
                break;
            }
        }
        Serial.println();
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
            Serial.printf_P(PSTR("DF page, log file #, log page: %d,\t"), (int)count);
            Serial.printf_P(PSTR("%d,\t"), (int)DataFlash.GetFileNumber());
            Serial.printf_P(PSTR("%d\n"), (int)DataFlash.GetFilePage());
        }
        return(-1);
    } else if (dump_log <= 0) {
        Serial.printf_P(PSTR("dumping all\n"));
        Log_Read(1, DataFlash.df_NumPages);
        return(-1);
    } else if ((argc != 2) || (dump_log <= (last_log_num - DataFlash.get_num_logs())) || (dump_log > last_log_num)) {
        Serial.printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Serial.printf_P(PSTR("Dumping Log %d,    start pg %d,   end pg %d\n"),
                    (int)dump_log,
                    (int)dump_log_start,
                    (int)dump_log_end);

    Log_Read(dump_log_start, dump_log_end);
    Serial.printf_P(PSTR("Done\n"));
    return 0;
}

static void do_erase_logs(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Erasing logs"));
    DataFlash.EraseAll(mavlink_delay);
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
        Serial.printf_P(PSTR("missing log type\n"));
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




// Write an attitude packet. Total length : 10 bytes
static void Log_Write_Attitude(int16_t log_roll, int16_t log_pitch, uint16_t log_yaw)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_ATTITUDE_MSG);
    DataFlash.WriteInt(log_roll);
    DataFlash.WriteInt(log_pitch);
    DataFlash.WriteInt(log_yaw);
    DataFlash.WriteByte(END_BYTE);
}

// Write a performance monitoring packet. Total length : 19 bytes
 #if HIL_MODE != HIL_MODE_ATTITUDE
static void Log_Write_Performance()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_PERFORMANCE_MSG);
    DataFlash.WriteLong(millis()- perf_mon_timer);
    DataFlash.WriteInt((int16_t)mainLoop_count);
    DataFlash.WriteInt(G_Dt_max);
    DataFlash.WriteByte(0);
    DataFlash.WriteByte(imu.adc_constraints);
    DataFlash.WriteByte(ahrs.renorm_range_count);
    DataFlash.WriteByte(ahrs.renorm_blowup_count);
    DataFlash.WriteByte(gps_fix_count);
    DataFlash.WriteInt(1);     // AHRS health
    DataFlash.WriteInt((int)(ahrs.get_gyro_drift().x * 1000));
    DataFlash.WriteInt((int)(ahrs.get_gyro_drift().y * 1000));
    DataFlash.WriteInt((int)(ahrs.get_gyro_drift().z * 1000));
    DataFlash.WriteInt(pmTest1);
    DataFlash.WriteByte(END_BYTE);
}
 #endif

// Write a command processing packet. Total length : 19 bytes
//void Log_Write_Cmd(byte num, byte id, byte p1, int32_t alt, int32_t lat, int32_t lng)
static void Log_Write_Cmd(byte num, struct Location *wp)
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
    DataFlash.WriteByte(END_BYTE);
}

static void Log_Write_Startup(byte type)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_STARTUP_MSG);
    DataFlash.WriteByte(type);
    DataFlash.WriteByte(g.command_total);
    DataFlash.WriteByte(END_BYTE);

    // create a location struct to hold the temp Waypoints for printing
    struct Location cmd = get_cmd_with_index(0);
    Log_Write_Cmd(0, &cmd);

    for (int16_t i = 1; i <= g.command_total; i++) {
        cmd = get_cmd_with_index(i);
        Log_Write_Cmd(i, &cmd);
    }
}


// Write a control tuning packet. Total length : 22 bytes
 #if HIL_MODE != HIL_MODE_ATTITUDE
static void Log_Write_Control_Tuning()
{
    Vector3f accel = imu.get_accel();

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
    DataFlash.WriteByte(END_BYTE);
}
 #endif

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
    DataFlash.WriteByte(END_BYTE);
}

// Write a mode packet. Total length : 5 bytes
static void Log_Write_Mode(byte mode)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_MODE_MSG);
    DataFlash.WriteByte(mode);
    DataFlash.WriteByte(END_BYTE);
}

// Write an GPS packet. Total length : 30 bytes
static void Log_Write_GPS(      int32_t log_Time, int32_t log_Lattitude, int32_t log_Longitude, int32_t log_gps_alt, int32_t log_mix_alt,
                                int32_t log_Ground_Speed, int32_t log_Ground_Course, byte log_Fix, byte log_NumSats)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_GPS_MSG);
    DataFlash.WriteLong(log_Time);
    DataFlash.WriteByte(log_Fix);
    DataFlash.WriteByte(log_NumSats);
    DataFlash.WriteLong(log_Lattitude);
    DataFlash.WriteLong(log_Longitude);
    DataFlash.WriteInt(sonar_alt);                              // This one is just temporary for testing out sonar in fixed wing
    DataFlash.WriteLong(log_mix_alt);
    DataFlash.WriteLong(log_gps_alt);
    DataFlash.WriteLong(log_Ground_Speed);
    DataFlash.WriteLong(log_Ground_Course);
    DataFlash.WriteByte(END_BYTE);
}

// Write an raw accel/gyro data packet. Total length : 28 bytes
 #if HIL_MODE != HIL_MODE_ATTITUDE
static void Log_Write_Raw()
{
    Vector3f gyro = imu.get_gyro();
    Vector3f accel = imu.get_accel();
    gyro *= t7;                                                                 // Scale up for storage as long integers
    accel *= t7;
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_RAW_MSG);

    DataFlash.WriteLong((long)gyro.x);
    DataFlash.WriteLong((long)gyro.y);
    DataFlash.WriteLong((long)gyro.z);
    DataFlash.WriteLong((long)accel.x);
    DataFlash.WriteLong((long)accel.y);
    DataFlash.WriteLong((long)accel.z);

    DataFlash.WriteByte(END_BYTE);
}
 #endif

static void Log_Write_Current()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_CURRENT_MSG);
    DataFlash.WriteInt(g.channel_throttle.control_in);
    DataFlash.WriteInt((int)(battery_voltage1       * 100.0));
    DataFlash.WriteInt((int)(current_amps1          * 100.0));
    DataFlash.WriteInt((int)current_total1);
    DataFlash.WriteByte(END_BYTE);
}

// Read a Current packet
static void Log_Read_Current()
{
    Serial.printf_P(PSTR("CURR: %d, %4.4f, %4.4f, %d\n"),
                    (int)DataFlash.ReadInt(),
                    ((float)DataFlash.ReadInt() / 100.f),
                    ((float)DataFlash.ReadInt() / 100.f),
                    (int)DataFlash.ReadInt());
}

// Read an control tuning packet
static void Log_Read_Control_Tuning()
{
    float logvar;

    Serial.printf_P(PSTR("CTUN:"));
    for (int16_t y = 1; y < 10; y++) {
        logvar = DataFlash.ReadInt();
        if(y < 8) logvar        = logvar/100.f;
        if(y == 9) logvar       = logvar/10000.f;
        Serial.print(logvar);
        print_comma();
    }
    Serial.println();
}

// Read a nav tuning packet
static void Log_Read_Nav_Tuning()
{
    int16_t d[7];
    for (int8_t i=0; i<7; i++) {
        d[i] = DataFlash.ReadInt();
    }
    Serial.printf_P(PSTR("NTUN: %4.4f, %d, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f,\n"),              // \n
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

    Serial.printf_P(PSTR("PM:"));
    pm_time = DataFlash.ReadLong();
    Serial.print(pm_time);
    print_comma();
    for (int16_t y = 1; y <= 12; y++) {
        if(y < 3 || y > 7) {
            logvar = DataFlash.ReadInt();
        }else{
            logvar = DataFlash.ReadByte();
        }
        Serial.print(logvar);
        print_comma();
    }
    Serial.println();
}

// Read a command processing packet
static void Log_Read_Cmd()
{
    byte logvarb;
    int32_t logvarl;

    Serial.printf_P(PSTR("CMD:"));
    for(int16_t i = 1; i < 4; i++) {
        logvarb = DataFlash.ReadByte();
        Serial.print(logvarb, DEC);
        print_comma();
    }
    for(int16_t i = 1; i < 4; i++) {
        logvarl = DataFlash.ReadLong();
        Serial.print(logvarl, DEC);
        print_comma();
    }
    Serial.println();
}

static void Log_Read_Startup()
{
    byte logbyte = DataFlash.ReadByte();

    if (logbyte == TYPE_AIRSTART_MSG)
        Serial.printf_P(PSTR("AIR START - "));
    else if (logbyte == TYPE_GROUNDSTART_MSG)
        Serial.printf_P(PSTR("GROUND START - "));
    else
        Serial.printf_P(PSTR("UNKNOWN STARTUP - "));

    Serial.printf_P(PSTR(" %d commands in memory\n"),(int)DataFlash.ReadByte());
}

// Read an attitude packet
static void Log_Read_Attitude()
{
    int16_t d[3];
    d[0] = DataFlash.ReadInt();
    d[1] = DataFlash.ReadInt();
    d[2] = DataFlash.ReadInt();
    Serial.printf_P(PSTR("ATT: %d, %d, %u\n"),
                    (int)d[0], (int)d[1],
                    (unsigned)d[2]);
}

// Read a mode packet
static void Log_Read_Mode()
{
    Serial.printf_P(PSTR("MOD:"));
    print_flight_mode(DataFlash.ReadByte());
}

// Read a GPS packet
static void Log_Read_GPS()
{
    int32_t l[7];
    byte b[2];
    int16_t i;
    l[0] = DataFlash.ReadLong();
    b[0] = DataFlash.ReadByte();
    b[1] = DataFlash.ReadByte();
    l[1] = DataFlash.ReadLong();
    l[2] = DataFlash.ReadLong();
    i = DataFlash.ReadInt();
    l[3] = DataFlash.ReadLong();
    l[4] = DataFlash.ReadLong();
    l[5] = DataFlash.ReadLong();
    l[6] = DataFlash.ReadLong();
    Serial.printf_P(PSTR("GPS: %ld, %d, %d, %4.7f, %4.7f, %d, %4.4f, %4.4f, %4.4f, %4.4f\n"),
                    (long)l[0], (int)b[0], (int)b[1],
                    l[1]/t7, l[2]/t7,
                    (int)i,
                    l[3]/100.0, l[4]/100.0, l[5]/100.0, l[6]/100.0);
}

// Read a raw accel/gyro packet
static void Log_Read_Raw()
{
    float logvar;
    Serial.printf_P(PSTR("RAW:"));
    for (int16_t y = 0; y < 6; y++) {
        logvar = (float)DataFlash.ReadLong() / t7;
        Serial.print(logvar);
        print_comma();
    }
    Serial.println();
}

// Read the DataFlash log memory : Packet Parser
static void Log_Read(int16_t start_page, int16_t end_page)
{
    int16_t packet_count = 0;

 #ifdef AIRFRAME_NAME
    Serial.printf_P(PSTR((AIRFRAME_NAME)
 #endif
    Serial.printf_P(PSTR("\n" THISFIRMWARE
                         "\nFree RAM: %u\n"),
                    memcheck_available_memory());

    if(start_page > end_page)
    {
        packet_count = Log_Read_Process(start_page, DataFlash.df_NumPages);
        packet_count += Log_Read_Process(1, end_page);
    } else {
        packet_count = Log_Read_Process(start_page, end_page);
    }

    Serial.printf_P(PSTR("Number of packets read: %d\n"), (int) packet_count);
}

// Read the DataFlash log memory : Packet Parser
static int16_t Log_Read_Process(int16_t start_page, int16_t end_page)
{
    byte data;
    byte log_step = 0;
    int16_t page = start_page;
    int16_t packet_count = 0;

    DataFlash.StartRead(start_page);
                        while (page < end_page && page != -1) {
                            data = DataFlash.ReadByte();

                            switch(log_step)     // This is a state machine to read the packets
                            {
                            case 0:
                                if(data == HEAD_BYTE1)  // Head byte 1
                                    log_step++;
                                break;
                            case 1:
                                if(data == HEAD_BYTE2)  // Head byte 2
                                    log_step++;
                                else
                                    log_step = 0;
                                break;
                            case 2:
                                if(data == LOG_ATTITUDE_MSG) {
                                    Log_Read_Attitude();
                                    log_step++;

                                }else if(data == LOG_MODE_MSG) {
                                    Log_Read_Mode();
                                    log_step++;

                                }else if(data == LOG_CONTROL_TUNING_MSG) {
                                    Log_Read_Control_Tuning();
                                    log_step++;

                                }else if(data == LOG_NAV_TUNING_MSG) {
                                    Log_Read_Nav_Tuning();
                                    log_step++;

                                }else if(data == LOG_PERFORMANCE_MSG) {
                                    Log_Read_Performance();
                                    log_step++;

                                }else if(data == LOG_RAW_MSG) {
                                    Log_Read_Raw();
                                    log_step++;

                                }else if(data == LOG_CMD_MSG) {
                                    Log_Read_Cmd();
                                    log_step++;

                                }else if(data == LOG_CURRENT_MSG) {
                                    Log_Read_Current();
                                    log_step++;

                                }else if(data == LOG_STARTUP_MSG) {
                                    Log_Read_Startup();
                                    log_step++;
                                }else {
                                    if(data == LOG_GPS_MSG) {
                                        Log_Read_GPS();
                                        log_step++;
                                    }else{
                                        Serial.printf_P(PSTR("Error Reading Packet: %d\n"),packet_count);
                                        log_step = 0;            // Restart, we have a problem...
                                    }
                                }
                                break;
                            case 3:
                                if(data == END_BYTE) {
                                    packet_count++;
                                }else{
                                    Serial.printf_P(PSTR("Error Reading END_BYTE: %d\n"),(int)data);
                                }
                                log_step = 0;                   // Restart sequence: new packet...
                                break;
                            }
                            page = DataFlash.GetPage();
                        }
                        return packet_count;
}

#else // LOGGING_ENABLED

// dummy functions
static void Log_Write_Mode(byte mode) {
}
static void Log_Write_Startup(byte type) {
}
static void Log_Write_Cmd(byte num, struct Location *wp) {
}
static void Log_Write_Current() {
}
static void Log_Write_Nav_Tuning() {
}
static void Log_Write_GPS(      int32_t log_Time, int32_t log_Lattitude, int32_t log_Longitude, int32_t log_gps_alt, int32_t log_mix_alt,
                                int32_t log_Ground_Speed, int32_t log_Ground_Course, byte log_Fix, byte log_NumSats) {
}
static void Log_Write_Performance() {
}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}
static void Log_Write_Attitude(int16_t log_roll, int16_t log_pitch, uint16_t log_yaw) {
}
static void Log_Write_Control_Tuning() {
}
static void Log_Write_Raw() {
}


#endif // LOGGING_ENABLED
