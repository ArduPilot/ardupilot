// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149
#define END_BYTE    0xBA    // Decimal 186


// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static bool     print_log_menu(void);
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
//static int8_t	help_log(uint8_t argc,          const Menu::arg *argv)
/*{
 *       cliSerial->printf_P(PSTR("\n"
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
const struct Menu::command log_menu_commands[] PROGMEM = {
    {"dump",        dump_log},
    {"erase",       erase_logs},
    {"enable",      select_logs},
    {"disable",     select_logs}
};

static int32_t get_int(float f)
{
    float_int.float_value = f;
    return float_int.int_value;
}

static float get_float(int32_t i)
{
    float_int.int_value = i;
    return float_int.float_value;
}


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
        if (g.log_bitmask & MASK_LOG_RAW) cliSerial->printf_P(PSTR(" RAW"));
        if (g.log_bitmask & MASK_LOG_CMD) cliSerial->printf_P(PSTR(" CMD"));
        if (g.log_bitmask & MASK_LOG_CUR) cliSerial->printf_P(PSTR(" CURRENT"));
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
        TARG(RAW);
        TARG(CMD);
        TARG(CUR);
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

// Write an GPS packet. Total length : 31 bytes
static void Log_Write_GPS()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_GPS_MSG);

    DataFlash.WriteLong(g_gps->time);           		 // 1
    DataFlash.WriteByte(g_gps->num_sats);       		 // 2

    DataFlash.WriteLong(g_gps->latitude);       		 // 3
    DataFlash.WriteLong(g_gps->longitude);       		 // 4
    DataFlash.WriteLong(current_loc.alt);       		 // 5
    DataFlash.WriteLong(g_gps->altitude);       		 // 6

    DataFlash.WriteInt(g_gps->ground_speed);    		 // 7
    DataFlash.WriteLong(g_gps->ground_course);  		 // 8

    DataFlash.WriteByte(END_BYTE);
}

// Read a GPS packet
static void Log_Read_GPS()
{
    int32_t temp1   = DataFlash.ReadLong();           // 1 time
    int8_t temp2    = DataFlash.ReadByte();           // 2 sats
    int32_t temp3   = DataFlash.ReadLong();           // 3 lat
    int32_t temp4   = DataFlash.ReadLong();           // 4 lon
    float temp5     = DataFlash.ReadLong() / 100.0;   // 5 sensor alt
    float temp6     = DataFlash.ReadLong() / 100.0;   // 6 gps alt
    int16_t temp7   = DataFlash.ReadInt();            // 7 ground speed
    int32_t temp8   = DataFlash.ReadLong();           // 8 ground course

    //  1   2    3      4     5      6      7    8
    cliSerial->printf_P(PSTR("GPS, %ld, %d, "),
                    (long)temp1,                          // 1 time
                    (int)temp2);                          // 2 sats
    print_latlon(cliSerial, temp3);
    cliSerial->print_P(PSTR(", "));
    print_latlon(cliSerial, temp4);
    cliSerial->printf_P(PSTR(", %4.4f, %4.4f, %d, %ld\n"),
                    temp5,                                // 5 gps alt
                    temp6,                                // 6 sensor alt
                    (int)temp7,                           // 7 ground speed
                    (long)temp8);                         // 8 ground course
}

static void Log_Write_Raw()
{
    Vector3f gyro = ins.get_gyro();
    Vector3f accel = ins.get_accel();

    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_RAW_MSG);

    DataFlash.WriteLong(get_int(gyro.x));
    DataFlash.WriteLong(get_int(gyro.y));
    DataFlash.WriteLong(get_int(gyro.z));

    DataFlash.WriteLong(get_int(accel.x));
    DataFlash.WriteLong(get_int(accel.y));
    DataFlash.WriteLong(get_int(accel.z));

    DataFlash.WriteByte(END_BYTE);

	/*
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_RAW_MSG);
	DataFlash.WriteLong(get_int(ahrs._omega_I.x));
	DataFlash.WriteLong(get_int(ahrs._omega_I.y));

    DataFlash.WriteByte(END_BYTE);
	*/
}

// Read a raw accel/gyro packet
static void Log_Read_Raw()
{
    float logvar;
    cliSerial->printf_P(PSTR("RAW,"));
    for (int16_t y = 0; y < 6; y++) {
        logvar = get_float(DataFlash.ReadLong());
        cliSerial->print(logvar);
        cliSerial->print_P(PSTR(", "));
    }
    cliSerial->println_P(PSTR(" "));

	/*
	float temp1 = get_float(DataFlash.ReadLong());
	float temp2 = get_float(DataFlash.ReadLong());

	cliSerial->printf_P(PSTR("RAW, %4.4f, %4.4f\n"),
			temp1,
			temp2);
	*/
}


// Write an Current data packet. Total length : 16 bytes
static void Log_Write_Current()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_CURRENT_MSG);

    DataFlash.WriteInt(g.rc_3.control_in);                      // 1
    DataFlash.WriteLong(throttle_integrator);                   // 2
    DataFlash.WriteInt(battery_voltage1     * 100.0);           // 3
    DataFlash.WriteInt(current_amps1        * 100.0);           // 4
    DataFlash.WriteInt(current_total1);                                 // 5

    DataFlash.WriteByte(END_BYTE);
}

// Read a Current packet
static void Log_Read_Current()
{
    int16_t temp1 = DataFlash.ReadInt();                        // 1
    int32_t temp2 = DataFlash.ReadLong();                       // 2
    float temp3 = DataFlash.ReadInt() / 100.f;          // 3
    float temp4 = DataFlash.ReadInt() / 100.f;          // 4
    int16_t temp5 = DataFlash.ReadInt();                        // 5

    //  1    2    3      4      5
    cliSerial->printf_P(PSTR("CURR, %d, %ld, %4.4f, %4.4f, %d\n"),
                    (int)temp1,
                    (long)temp2,
                    temp3,
                    temp4,
                    (int)temp5);
}

// Write an Motors packet. Total length : 12 ~ 20 bytes
static void Log_Write_Motors()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_MOTORS_MSG);

 #if FRAME_CONFIG ==     TRI_FRAME
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_1]);    //1
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_2]);    //2
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_4]);    //3
    DataFlash.WriteInt(g.rc_4.radio_out);    //4

 #elif FRAME_CONFIG == HEXA_FRAME
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_1]);    //1
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_2]);    //2
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_3]);    //3
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_4]);    //4
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_5]);    //5
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_6]);    //6

 #elif FRAME_CONFIG == Y6_FRAME
    //left
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_2]);    //1
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_3]);    //2
    //right
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_5]);    //3
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_1]);    //4
    //back
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_6]);    //5
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_4]);    //6

 #elif FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_1]);    //1
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_2]);    //2
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_3]);    //3
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_4]);    //4
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_5]);    //5
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_6]);     //6
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_7]);    //7
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_8]);    //8

 #elif FRAME_CONFIG == HELI_FRAME
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_1]);    //1
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_2]);    //2
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_3]);    //3
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_4]);    //4
    DataFlash.WriteInt(motors.ext_gyro_gain);    //5

 #else        // quads
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_1]);    //1
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_2]);    //2
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_3]);    //3
    DataFlash.WriteInt(motors.motor_out[AP_MOTORS_MOT_4]);    //4
 #endif

    DataFlash.WriteByte(END_BYTE);
}

// Read a Motors packet.
static void Log_Read_Motors()
{
 #if FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME
    int16_t temp1 = DataFlash.ReadInt();                        // 1
    int16_t temp2 = DataFlash.ReadInt();                        // 2
    int16_t temp3 = DataFlash.ReadInt();                        // 3
    int16_t temp4 = DataFlash.ReadInt();                        // 4
    int16_t temp5 = DataFlash.ReadInt();                        // 5
    int16_t temp6 = DataFlash.ReadInt();                        // 6
    // 1  2   3   4   5   6
    cliSerial->printf_P(PSTR("MOT, %d, %d, %d, %d, %d, %d\n"),
                    (int)temp1,         //1
                    (int)temp2,         //2
                    (int)temp3,         //3
                    (int)temp4,         //4
                    (int)temp5,         //5
                    (int)temp6);        //6

 #elif FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME
    int16_t temp1 = DataFlash.ReadInt();                        // 1
    int16_t temp2 = DataFlash.ReadInt();                        // 2
    int16_t temp3 = DataFlash.ReadInt();                        // 3
    int16_t temp4 = DataFlash.ReadInt();                        // 4
    int16_t temp5 = DataFlash.ReadInt();                        // 5
    int16_t temp6 = DataFlash.ReadInt();                        // 6
    int16_t temp7 = DataFlash.ReadInt();                        // 7
    int16_t temp8 = DataFlash.ReadInt();                        // 8
    // 1   2   3   4   5   6   7   8
    cliSerial->printf_P(PSTR("MOT, %d, %d, %d, %d, %d, %d, %d, %d\n"),
                    (int)temp1,         //1
                    (int)temp2,         //2
                    (int)temp3,         //3
                    (int)temp4,         //4
                    (int)temp5,         //5
                    (int)temp6,         //6
                    (int)temp7,         //7
                    (int)temp8);        //8

 #elif FRAME_CONFIG == HELI_FRAME
    int16_t temp1 = DataFlash.ReadInt();                        // 1
    int16_t temp2 = DataFlash.ReadInt();                        // 2
    int16_t temp3 = DataFlash.ReadInt();                        // 3
    int16_t temp4 = DataFlash.ReadInt();                        // 4
    int16_t temp5 = DataFlash.ReadInt();                        // 5
    // 1   2   3   4   5
    cliSerial->printf_P(PSTR("MOT, %d, %d, %d, %d, %d\n"),
                    (int)temp1,         //1
                    (int)temp2,         //2
                    (int)temp3,         //3
                    (int)temp4,         //4
                    (int)temp5);        //5

 #else        // quads, TRIs
    int16_t temp1 = DataFlash.ReadInt();                        // 1
    int16_t temp2 = DataFlash.ReadInt();                        // 2
    int16_t temp3 = DataFlash.ReadInt();                        // 3
    int16_t temp4 = DataFlash.ReadInt();                        // 4

    // 1   2   3   4
    cliSerial->printf_P(PSTR("MOT, %d, %d, %d, %d\n"),
                    (int)temp1,         //1
                    (int)temp2,         //2
                    (int)temp3,         //3
                    (int)temp4);     //4;
 #endif
}

// Write an optical flow packet. Total length : 30 bytes
static void Log_Write_Optflow()
{
 #if OPTFLOW == ENABLED
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_OPTFLOW_MSG);
    DataFlash.WriteInt((int)optflow.dx);
    DataFlash.WriteInt((int)optflow.dy);
    DataFlash.WriteInt((int)optflow.surface_quality);
    DataFlash.WriteInt((int)optflow.x_cm);
    DataFlash.WriteInt((int)optflow.y_cm);
    DataFlash.WriteLong(optflow.vlat);    //optflow_offset.lat + optflow.lat);
    DataFlash.WriteLong(optflow.vlon);    //optflow_offset.lng + optflow.lng);
    DataFlash.WriteLong(of_roll);
    DataFlash.WriteLong(of_pitch);
    DataFlash.WriteByte(END_BYTE);
 #endif     // OPTFLOW == ENABLED
}

// Read an optical flow packet.
static void Log_Read_Optflow()
{
    int16_t temp1   = DataFlash.ReadInt();                      // 1
    int16_t temp2   = DataFlash.ReadInt();                      // 2
    int16_t temp3   = DataFlash.ReadInt();                      // 3
    int16_t temp4   = DataFlash.ReadInt();                      // 4
    int16_t temp5   = DataFlash.ReadInt();                      // 5
    float temp6     = DataFlash.ReadLong();                     // 6
    float temp7     = DataFlash.ReadLong();                     // 7
    int32_t temp8   = DataFlash.ReadLong();                     // 8
    int32_t temp9   = DataFlash.ReadLong();                     // 9

    cliSerial->printf_P(PSTR("OF, %d, %d, %d, %d, %d, %4.7f, %4.7f, %ld, %ld\n"),
                    (int)temp1,
                    (int)temp2,
                    (int)temp3,
                    (int)temp4,
                    (int)temp5,
                    temp6,
                    temp7,
                    (long)temp8,
                    (long)temp9);
}

// Write an Nav Tuning packet. Total length : 24 bytes
static void Log_Write_Nav_Tuning()
{
    //Matrix3f tempmat = dcm.get_dcm_matrix();

    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_NAV_TUNING_MSG);

	DataFlash.WriteInt(wp_distance);                        // 1
	DataFlash.WriteInt(wp_bearing/100);                 // 2
	DataFlash.WriteInt(long_error);                         // 3
	DataFlash.WriteInt(lat_error);                          // 4

	DataFlash.WriteInt(nav_pitch);                          // 5
	DataFlash.WriteInt(nav_roll);                           // 6
	DataFlash.WriteInt(lon_speed);                          // 7
	DataFlash.WriteInt(lat_speed);                          // 8

    DataFlash.WriteByte(END_BYTE);
}

// Read a Nav Tuning packet.
static void Log_Read_Nav_Tuning()
{
    int16_t temp;

    cliSerial->printf_P(PSTR("NTUN, "));

    for(int8_t i = 1; i < 8; i++ ) {
        temp = DataFlash.ReadInt();
        cliSerial->printf_P(PSTR("%d, "), (int)temp);
    }
    // read 8
    temp = DataFlash.ReadInt();
    cliSerial->printf_P(PSTR("%d\n"), (int)temp);
}


// Write a control tuning packet. Total length : 26 bytes
static void Log_Write_Control_Tuning()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_CONTROL_TUNING_MSG);

    DataFlash.WriteInt(g.rc_3.control_in);                 	// 1
    DataFlash.WriteInt(sonar_alt);                          // 2
    DataFlash.WriteInt(baro_alt);                           // 3
    DataFlash.WriteInt(next_WP.alt);                        // 4
    DataFlash.WriteInt(nav_throttle);                       // 5
    DataFlash.WriteInt(angle_boost);                        // 6
    DataFlash.WriteInt(climb_rate);                 	    // 7
    DataFlash.WriteInt(g.rc_3.servo_out);                  	// 8
    DataFlash.WriteInt(desired_climb_rate);                  // 9


    DataFlash.WriteByte(END_BYTE);
}

// Read an control tuning packet
static void Log_Read_Control_Tuning()
{
    int16_t temp;

    cliSerial->printf_P(PSTR("CTUN, "));

    for(uint8_t i = 1; i < 9; i++ ) {
        temp = DataFlash.ReadInt();
        cliSerial->printf_P(PSTR("%d, "), (int)temp);
    }
    // read 9
    temp = DataFlash.ReadInt();
    cliSerial->printf_P(PSTR("%d\n"), (int)temp);
}

static void Log_Write_Iterm()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_ITERM_MSG);

    DataFlash.WriteInt((int16_t)g.pi_stabilize_roll.get_integrator());   	// 1
    DataFlash.WriteInt((int16_t)g.pi_stabilize_pitch.get_integrator());  	// 2
    DataFlash.WriteInt((int16_t)g.pi_stabilize_yaw.get_integrator());    	// 3
    DataFlash.WriteInt((int16_t)g.pid_rate_roll.get_integrator());     		// 4
    DataFlash.WriteInt((int16_t)g.pid_rate_pitch.get_integrator());    	 	// 5
    DataFlash.WriteInt((int16_t)g.pid_rate_yaw.get_integrator());    	 	// 6
	DataFlash.WriteInt((int16_t)g.pid_nav_lat.get_integrator());         	// 7
	DataFlash.WriteInt((int16_t)g.pid_nav_lon.get_integrator());         	// 8
	DataFlash.WriteInt((int16_t)g.pid_loiter_rate_lat.get_integrator()); 	// 9
	DataFlash.WriteInt((int16_t)g.pid_loiter_rate_lon.get_integrator()); 	// 10
	DataFlash.WriteInt((int16_t)g.pid_throttle.get_integrator()); 			// 11
	DataFlash.WriteInt(g.throttle_cruise); 									// 12

    DataFlash.WriteByte(END_BYTE);
}

// Read an control tuning packet
static void Log_Read_Iterm()
{
    int16_t temp;

    cliSerial->printf_P(PSTR("ITERM, "));

    for(uint8_t i = 1; i < 12; i++ ) {
        temp = DataFlash.ReadInt();
        cliSerial->printf_P(PSTR("%d, "), (int)temp);
    }
    // read 12
    temp = DataFlash.ReadInt();
    cliSerial->println((int)temp);
}


// Write a performance monitoring packet. Total length : 11 bytes
static void Log_Write_Performance()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_PERFORMANCE_MSG);
    DataFlash.WriteByte(ahrs.renorm_range_count);           //1
    DataFlash.WriteByte(ahrs.renorm_blowup_count);          //2
    DataFlash.WriteByte(gps_fix_count);                     //3
    DataFlash.WriteInt(perf_info_get_num_long_running());   //4  - number of long running loops
    DataFlash.WriteInt(perf_info_get_num_loops());          //5  - total number of loops
    DataFlash.WriteLong(perf_info_get_max_time());          //6  - time of longest running loop
    DataFlash.WriteByte(END_BYTE);
}

// Read a performance packet
static void Log_Read_Performance()
{
    int8_t temp1    = DataFlash.ReadByte();
    int8_t temp2    = DataFlash.ReadByte();
    int8_t temp3    = DataFlash.ReadByte();
    uint16_t temp4  = DataFlash.ReadInt();
    uint16_t temp5  = DataFlash.ReadInt();
    uint32_t temp6  = DataFlash.ReadLong();

    //                         1   2   3   4   5    6
    cliSerial->printf_P(PSTR("PM, %d, %d, %d, %u, %u, %lu\n"),
                    (int)temp1,
                    (int)temp2,
                    (int)temp3,
                    (unsigned int)temp4,
                    (unsigned int)temp5,
                    (unsigned long)temp6);
}

// Write a command processing packet.  Total length : 21 bytes
static void Log_Write_Cmd(uint8_t num, struct Location *wp)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_CMD_MSG);

    DataFlash.WriteByte(g.command_total);       // 1
    DataFlash.WriteByte(num);                                   // 2
    DataFlash.WriteByte(wp->id);                        // 3
    DataFlash.WriteByte(wp->options);                   // 4
    DataFlash.WriteByte(wp->p1);                        // 5
    DataFlash.WriteLong(wp->alt);                       // 6
    DataFlash.WriteLong(wp->lat);                       // 7
    DataFlash.WriteLong(wp->lng);                       // 8

    DataFlash.WriteByte(END_BYTE);
}
//CMD, 3, 0, 16, 8, 1, 800, 340440192, -1180692736


// Read a command processing packet
static void Log_Read_Cmd()
{
    int8_t temp1    = DataFlash.ReadByte();
    int8_t temp2    = DataFlash.ReadByte();
    int8_t temp3    = DataFlash.ReadByte();
    int8_t temp4    = DataFlash.ReadByte();
    int8_t temp5    = DataFlash.ReadByte();
    int32_t temp6   = DataFlash.ReadLong();
    int32_t temp7   = DataFlash.ReadLong();
    int32_t temp8   = DataFlash.ReadLong();

    //  1   2    3   4   5   6   7    8
    cliSerial->printf_P(PSTR( "CMD, %d, %d, %d, %d, %d, %ld, %ld, %ld\n"),
                    (int)temp1,
                    (int)temp2,
                    (int)temp3,
                    (int)temp4,
                    (int)temp5,
                    (long)temp6,
                    (long)temp7,
                    (long)temp8);
}

// Write an attitude packet. Total length : 16 bytes
static void Log_Write_Attitude()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_ATTITUDE_MSG);

    DataFlash.WriteInt(control_roll);										// 1
    DataFlash.WriteInt((int16_t)ahrs.roll_sensor);                          // 2
    DataFlash.WriteInt(control_pitch);                                      // 3
    DataFlash.WriteInt((int16_t)ahrs.pitch_sensor);                         // 4
    DataFlash.WriteInt(g.rc_4.control_in);                                  // 5
    DataFlash.WriteInt((uint16_t)ahrs.yaw_sensor);                          // 6
    DataFlash.WriteInt((uint16_t)nav_yaw);                                  // 7 (this used to be compass.heading)
    DataFlash.WriteByte(END_BYTE);
}

// Read an attitude packet
static void Log_Read_Attitude()
{
    int16_t temp1   = DataFlash.ReadInt();
    int16_t temp2   = DataFlash.ReadInt();
    int16_t temp3   = DataFlash.ReadInt();
    int16_t temp4   = DataFlash.ReadInt();
    int16_t temp5   = DataFlash.ReadInt();
    uint16_t temp6  = DataFlash.ReadInt();
    uint16_t temp7  = DataFlash.ReadInt();

    // 1   2   3    4   5   6  7    8   9
    cliSerial->printf_P(PSTR("ATT, %d, %d, %d, %d, %d, %u, %u\n"),
                    (int)temp1,
                    (int)temp2,
                    (int)temp3,
                    (int)temp4,
                    (int)temp5,
                    (unsigned)temp6,
                    (unsigned)temp7);
}

// Write an INAV packet. Total length : 52 Bytes
static void Log_Write_INAV()
{
#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
    Vector3f accel_corr = inertial_nav.accel_correction.get();

    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_INAV_MSG);

    DataFlash.WriteInt((int16_t)baro_alt);                                  // 1 barometer altitude
    DataFlash.WriteInt((int16_t)inertial_nav.get_altitude());               // 2 accel + baro filtered altitude
    DataFlash.WriteInt((int16_t)baro_rate);                                 // 3 barometer based climb rate
    DataFlash.WriteInt((int16_t)inertial_nav.get_velocity_z());             // 4 accel + baro based climb rate
    DataFlash.WriteLong(get_int(accel_corr.x));                             // 5 accel correction x-axis
    DataFlash.WriteLong(get_int(accel_corr.y));                             // 6 accel correction y-axis
    DataFlash.WriteLong(get_int(accel_corr.z));                             // 7 accel correction z-axis
    DataFlash.WriteLong(get_int(inertial_nav.accel_correction_ef.z));       // 8 accel correction earth frame
    DataFlash.WriteLong(g_gps->latitude-home.lat);                          // 9 lat from home
    DataFlash.WriteLong(g_gps->longitude-home.lng);                         // 10 lon from home
    DataFlash.WriteLong(get_int(inertial_nav.get_latitude_diff()));         // 11 accel based lat from home
    DataFlash.WriteLong(get_int(inertial_nav.get_longitude_diff()));        // 12 accel based lon from home
    DataFlash.WriteLong(get_int(inertial_nav.get_latitude_velocity()));     // 13 accel based lat velocity
    DataFlash.WriteLong(get_int(inertial_nav.get_longitude_velocity()));    // 14 accel based lon velocity

    DataFlash.WriteByte(END_BYTE);
#endif
}

// Read an INAV packet
static void Log_Read_INAV()
{
    int16_t temp1   = DataFlash.ReadInt();              // 1 barometer altitude
    int16_t temp2   = DataFlash.ReadInt();              // 2 accel + baro filtered altitude
    int16_t temp3   = DataFlash.ReadInt();              // 3 barometer based climb rate
    int16_t temp4   = DataFlash.ReadInt();              // 4 accel + baro based climb rate
    float temp5     = get_float(DataFlash.ReadLong());  // 5 accel correction x-axis
    float temp6     = get_float(DataFlash.ReadLong());  // 6 accel correction y-axis
    float temp7     = get_float(DataFlash.ReadLong());  // 7 accel correction z-axis
    float temp8     = get_float(DataFlash.ReadLong());  // 8 accel correction earth frame
    int32_t temp9   = DataFlash.ReadLong();             // 9 lat from home
    int32_t temp10  = DataFlash.ReadLong();             // 10 lon from home
    float temp11    = get_float(DataFlash.ReadLong());  // 11 accel based lat from home
    float temp12    = get_float(DataFlash.ReadLong());  // 12 accel based lon from home
    float temp13    = get_float(DataFlash.ReadLong());  // 13 accel based lat velocity
    float temp14    = get_float(DataFlash.ReadLong());  // 14 accel based lon velocity
                                  // 1   2   3   4      5      6      7      8    9   10   11     12     13     14
    cliSerial->printf_P(PSTR("INAV, %d, %d, %d, %d, %6.4f, %6.4f, %6.4f, %6.4f, %ld, %ld, %6.4f, %6.4f, %6.4f, %6.4f\n"),
                    (int)temp1,
                    (int)temp2,
                    (int)temp3,
                    (int)temp4,
                    temp5,
                    temp6,
                    temp7,
                    temp8,
                    temp9,
                    temp10,
                    temp11,
                    temp12,
                    temp13,
                    temp14);
}

// Write a mode packet. Total length : 7 bytes
static void Log_Write_Mode(uint8_t mode)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_MODE_MSG);
    DataFlash.WriteByte(mode);
    DataFlash.WriteInt(g.throttle_cruise);
    DataFlash.WriteByte(END_BYTE);
}

// Read a mode packet
static void Log_Read_Mode()
{
    cliSerial->printf_P(PSTR("MOD:"));
    print_flight_mode(DataFlash.ReadByte());
    cliSerial->printf_P(PSTR(", %d\n"),(int)DataFlash.ReadInt());
}

// Write Startup packet. Total length : 4 bytes
static void Log_Write_Startup()
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_STARTUP_MSG);
    DataFlash.WriteByte(END_BYTE);
}

// Read a startup packet
static void Log_Read_Startup()
{
    cliSerial->printf_P(PSTR("START UP\n"));
}

#define DATA_INT32 0
#define DATA_FLOAT 1
#define DATA_INT16 2
#define DATA_UINT16 3
#define DATA_EVENT 4

static void Log_Write_Data(uint8_t _index, int32_t _data)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_DATA_MSG);
    DataFlash.WriteByte(_index);
    DataFlash.WriteByte(DATA_INT32);
    DataFlash.WriteLong(_data);
    DataFlash.WriteByte(END_BYTE);
}

static void Log_Write_Data(uint8_t _index, float _data)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_DATA_MSG);
    DataFlash.WriteByte(_index);
    DataFlash.WriteByte(DATA_FLOAT);
    DataFlash.WriteLong(get_int(_data));
    DataFlash.WriteByte(END_BYTE);
}

static void Log_Write_Data(uint8_t _index, int16_t _data)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_DATA_MSG);
    DataFlash.WriteByte(_index);
    DataFlash.WriteByte(DATA_INT16);
    DataFlash.WriteInt(_data);
    DataFlash.WriteByte(END_BYTE);
}

static void Log_Write_Data(uint8_t _index, uint16_t _data)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_DATA_MSG);
    DataFlash.WriteByte(_index);
    DataFlash.WriteByte(DATA_UINT16);
    DataFlash.WriteInt(_data);
    DataFlash.WriteByte(END_BYTE);
}


static void Log_Write_Event(uint8_t _index)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_DATA_MSG);
    DataFlash.WriteByte(_index);
    DataFlash.WriteByte(DATA_EVENT);
    DataFlash.WriteByte(END_BYTE);
}

// Read a mode packet
static void Log_Read_Data()
{
    int8_t _index = DataFlash.ReadByte();
    int8_t _type = DataFlash.ReadByte();

    if(_type == DATA_EVENT) {
        cliSerial->printf_P(PSTR("EV: %u\n"), _index);

    }else if(_type == DATA_FLOAT) {
        float _value = get_float(DataFlash.ReadLong());
        cliSerial->printf_P(PSTR("DATA: %u, %1.6f\n"), _index, _value);

    }else if(_type == DATA_INT16) {
        int16_t _value = DataFlash.ReadInt();
        cliSerial->printf_P(PSTR("DATA: %u, %d\n"), _index, _value);

    }else if(_type == DATA_UINT16) {
        uint16_t _value = DataFlash.ReadInt();
        cliSerial->printf_P(PSTR("DATA: %u, %u\n"), _index, _value);

    }else if(_type == DATA_INT32) {
        int32_t _value = DataFlash.ReadLong();
        cliSerial->printf_P(PSTR("DATA: %u, %ld\n"), _index, _value);
    }
}

// Write an PID packet. Total length : 28 bytes
static void Log_Write_PID(int8_t pid_id, int32_t error, int32_t p, int32_t i, int32_t d, int32_t output, float gain)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_PID_MSG);

    DataFlash.WriteByte(pid_id);                        // 1
    DataFlash.WriteLong(error);                                 // 2
    DataFlash.WriteLong(p);                                     // 3
    DataFlash.WriteLong(i);                                     // 4
    DataFlash.WriteLong(d);                                     // 5
    DataFlash.WriteLong(output);                        // 6
    DataFlash.WriteLong(gain * 1000);                   // 7

    DataFlash.WriteByte(END_BYTE);
}

// Read a PID packet
static void Log_Read_PID()
{
    int8_t temp1    = DataFlash.ReadByte();             // pid id
    int32_t temp2   = DataFlash.ReadLong();             // error
    int32_t temp3   = DataFlash.ReadLong();             // p
    int32_t temp4   = DataFlash.ReadLong();             // i
    int32_t temp5   = DataFlash.ReadLong();             // d
    int32_t temp6   = DataFlash.ReadLong();             // output
    float temp7     = DataFlash.ReadLong() / 1000.f;                    // gain

    //  1    2    3    4    5    6      7
    cliSerial->printf_P(PSTR("PID-%d, %ld, %ld, %ld, %ld, %ld, %4.4f\n"),
                    (int)temp1,         // pid id
                    (long)temp2,                // error
                    (long)temp3,                // p
                    (long)temp4,                // i
                    (long)temp5,                // d
                    (long)temp6,                // output
                    temp7);             // gain
}

// Write a DMP attitude packet. Total length : 16 bytes
static void Log_Write_DMP()
{
#if SECONDARY_DMP_ENABLED == ENABLED
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_DMP_MSG);

    DataFlash.WriteInt((int16_t)ahrs.roll_sensor);                          // 1
    DataFlash.WriteInt((int16_t)ahrs2.roll_sensor);                         // 2
    DataFlash.WriteInt((int16_t)ahrs.pitch_sensor);                         // 3
    DataFlash.WriteInt((int16_t)ahrs2.pitch_sensor);                        // 4
    DataFlash.WriteInt((uint16_t)ahrs.yaw_sensor);                          // 5
    DataFlash.WriteInt((uint16_t)ahrs2.yaw_sensor);                         // 6
    DataFlash.WriteByte(END_BYTE);
#endif
}

// Read a DMP attitude packet
static void Log_Read_DMP()
{
    int16_t temp1   = DataFlash.ReadInt();
    int16_t temp2   = DataFlash.ReadInt();
    int16_t temp3   = DataFlash.ReadInt();
    int16_t temp4   = DataFlash.ReadInt();
    uint16_t temp5   = DataFlash.ReadInt();
    uint16_t temp6  = DataFlash.ReadInt();

                             // 1   2   3   4   5   6
    cliSerial->printf_P(PSTR("DMP, %d, %d, %d, %d, %u, %u\n"),
                    (int)temp1,
                    (int)temp2,
                    (int)temp3,
                    (int)temp4,
                    (unsigned)temp5,
                    (unsigned)temp6);
}

// Write a Camera packet. Total length : 26 bytes
static void Log_Write_Camera()
{
#if CAMERA == ENABLED
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_CAMERA_MSG);

    DataFlash.WriteLong(g_gps->time);           		 // 1
    DataFlash.WriteLong(current_loc.lat);       		 // 2
    DataFlash.WriteLong(current_loc.lng);       		 // 3
    DataFlash.WriteLong(current_loc.alt);       		 // 4
    DataFlash.WriteInt((int16_t)ahrs.roll_sensor);       // 5
    DataFlash.WriteInt((int16_t)ahrs.pitch_sensor);      // 6
    DataFlash.WriteInt((uint16_t)ahrs.yaw_sensor);       // 7
    DataFlash.WriteByte(END_BYTE);
#endif
}

// Read a camera packet
static void Log_Read_Camera()
{
    int32_t temp1   = DataFlash.ReadLong();             // 1 time
    int32_t temp2   = DataFlash.ReadLong();             // 2 lat
    int32_t temp3   = DataFlash.ReadLong();             // 3 lon
    float temp4     = DataFlash.ReadLong() / 100.0;     // 4 altitude
    int16_t temp5   = DataFlash.ReadInt();              // 5 roll in centidegrees
    int16_t temp6   = DataFlash.ReadInt();              // 6 pitch in centidegrees
    uint16_t temp7  = DataFlash.ReadInt();              // 7 yaw in centidegrees

                                     // 1
    cliSerial->printf_P(PSTR("CAMERA, %ld, "),(long)temp1);  // 1 time
    print_latlon(cliSerial, temp2);                         // 2 lat
    cliSerial->print_P(PSTR(", "));
    print_latlon(cliSerial, temp3);                         // 3 lon
                                 // 4   5   6   7
    cliSerial->printf_P(PSTR(", %4.4f, %d, %d, %u\n"),
                    temp4,                                  // 4 altitude
                    (int)temp5,                             // 5 roll in centidegrees
                    (int)temp6,                             // 6 pitch in centidegrees
                    (unsigned int)temp7);                   // 7 yaw in centidegrees
}

// Write an error packet. Total length : 6 bytes
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteByte(LOG_ERROR_MSG);

    DataFlash.WriteByte(sub_system);                // 1 sub system
    DataFlash.WriteByte(error_code);                // 2 error code
    DataFlash.WriteByte(END_BYTE);
}

// Read an error packet
static void Log_Read_Error()
{
    uint8_t sub_system   = DataFlash.ReadByte();    // 1 sub system
    uint8_t error_code   = DataFlash.ReadByte();    // 2 error code

    cliSerial->print_P(PSTR("ERR, "));

    // print subsystem
    switch(sub_system) {
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
            cliSerial->printf_P(PSTR("%d"),(int)sub_system);    // 1 sub system
            break;
    }

    // print error code
    cliSerial->printf_P(PSTR(", %d\n"),(int)error_code);    // 2 error code
}


// Read the DataFlash log memory
static void Log_Read(int16_t start_page, int16_t end_page)
{
    int16_t packet_count = 0;

 #ifdef AIRFRAME_NAME
    cliSerial->printf_P(PSTR((AIRFRAME_NAME)
 #endif

    cliSerial->printf_P(PSTR("\n" THISFIRMWARE
                         "\nFree RAM: %u\n"),
                    (unsigned) memcheck_available_memory());

 #if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
    cliSerial->printf_P(PSTR("APM 2\n"));
 #elif  CONFIG_APM_HARDWARE == APM2_BETA_HARDWARE
    cliSerial->printf_P(PSTR("APM 2Beta\n"));
 #else
    cliSerial->printf_P(PSTR("APM 1\n"));
 #endif

#if CLI_ENABLED == ENABLED
	setup_show(0, NULL);
#endif

    if(start_page > end_page) {
        packet_count = Log_Read_Process(start_page, DataFlash.df_NumPages);
        packet_count += Log_Read_Process(1, end_page);
    } else {
        packet_count = Log_Read_Process(start_page, end_page);
    }

    //cliSerial->printf_P(PSTR("Number of packets read: %d\n"), (int)packet_count);
}

// Read the DataFlash log memory : Packet Parser
static int16_t Log_Read_Process(int16_t start_page, int16_t end_page)
{
    uint8_t data;
    uint8_t log_step           = 0;
    int16_t page                    = start_page;
    int16_t packet_count = 0;

    DataFlash.StartRead(start_page);

	while(page < end_page && page != -1){

		data = DataFlash.ReadByte();

		// This is a state machine to read the packets
		switch(log_step) {
			case 0:
				if(data == HEAD_BYTE1)  // Head byte 1
					log_step++;
				break;

			case 1:
				if(data == HEAD_BYTE2)  // Head byte 2
					log_step++;
				else{
					log_step = 0;
					cliSerial->println_P(PSTR("."));
				}
				break;

			case 2:
				log_step = 0;
				switch(data) {
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

					case LOG_MOTORS_MSG:
						Log_Read_Motors();
						break;

					case LOG_OPTFLOW_MSG:
						Log_Read_Optflow();
						break;

					case LOG_GPS_MSG:
						Log_Read_GPS();
						break;

					case LOG_DATA_MSG:
						Log_Read_Data();
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
				}
				break;
		case 3:
			if(data == END_BYTE){
				packet_count++;
			}else{
				cliSerial->printf_P(PSTR("Error Reading END_BYTE: %d\n"),data);
			}
			log_step = 0;                   // Restart sequence: new packet...
			break;
		}
		page = DataFlash.GetPage();
	}
	return packet_count;
}


#else // LOGGING_ENABLED

static void Log_Write_Startup() {
}
static void Log_Read_Startup() {
}
static void Log_Read(int16_t start_page, int16_t end_page) {
}
static void Log_Write_Cmd(uint8_t num, struct Location *wp) {
}
static void Log_Write_Mode(uint8_t mode) {
}
static void Log_Write_Raw() {
}
void print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon) {
}
static void Log_Write_GPS() {
}
static void Log_Write_Current() {
}
static void Log_Write_Iterm() {
}
static void Log_Write_Attitude() {
}
static void Log_Write_INAV() {
}
static void Log_Write_Data(uint8_t _index, float _data){
}
static void Log_Write_Data(uint8_t _index, int32_t _data){
}
static void Log_Write_Data(uint8_t _index, int16_t _data){
}
static void Log_Write_Data(uint8_t _index, uint16_t _data){
}
static void Log_Write_Event(uint8_t _index){
}
static void Log_Write_Optflow() {
}
static void Log_Write_Nav_Tuning() {
}
static void Log_Write_Control_Tuning() {
}
static void Log_Write_Motors() {
}
static void Log_Write_Performance() {
}
static void Log_Write_PID(int8_t pid_id, int32_t error, int32_t p, int32_t i, int32_t d, int32_t output, float gain) {
}
static void Log_Write_DMP() {
}
static void Log_Write_Camera() {
}
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) {
}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}

#endif // LOGGING_DISABLED
