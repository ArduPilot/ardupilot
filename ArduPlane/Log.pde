// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

#define HEAD_BYTE1 	0xA3	// Decimal 163
#define HEAD_BYTE2 	0x95	// Decimal 149
#define END_BYTE	0xBA	// Decimal 186


// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	dump_log(uint8_t argc, 			const Menu::arg *argv);
static int8_t	erase_logs(uint8_t argc, 		const Menu::arg *argv);
static int8_t	select_logs(uint8_t argc, 		const Menu::arg *argv);

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
//static int8_t	help_log(uint8_t argc, 			const Menu::arg *argv)
/*{
	Serial.printf_P(PSTR("\n"
						 "Commands:\n"
						 "  dump <n>"
						 "  erase (all logs)\n"
						 "  enable <name> | all\n"
						 "  disable <name> | all\n"
						 "\n"));
    return 0;
}*/

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
static const struct Menu::command log_menu_commands[] PROGMEM = {
	{"dump",	dump_log},
	{"erase",	erase_logs},
	{"enable",	select_logs},
	{"disable",	select_logs}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

static void get_log_boundaries(byte log_num, int & start_page, int & end_page);

static bool
print_log_menu(void)
{
	int log_start;
	int log_end;
	int temp;	
	int last_log_num = find_last_log();
	
	uint16_t num_logs = get_num_logs();

	Serial.printf_P(PSTR("logs enabled: "));

	if (0 == g.log_bitmask) {
		Serial.printf_P(PSTR("none"));
	}else{
		// Macro to make the following code a bit easier on the eye.
		// Pass it the capitalised name of the log option, as defined
		// in defines.h but without the LOG_ prefix.  It will check for
		// the bit being set and print the name of the log option to suit.
		#define PLOG(_s)	if (g.log_bitmask & MASK_LOG_ ## _s) Serial.printf_P(PSTR(" %S"), PSTR(#_s))
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
		Serial.printf_P(PSTR("\n%d logs\n"), num_logs);

		for(int i=num_logs;i>=1;i--) {
            int last_log_start = log_start, last_log_end = log_end;
			temp = last_log_num-i+1;
			get_log_boundaries(temp, log_start, log_end);
			Serial.printf_P(PSTR("Log %d,    start %d,   end %d\n"), temp, log_start, log_end);
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
	int dump_log;
	int dump_log_start;
	int dump_log_end;
	byte last_log_num;

	// check that the requested log number can be read
	dump_log = argv[1].i;
	last_log_num = find_last_log();
	
	if (dump_log == -2) {
		for(uint16_t count=1; count<=DataFlash.df_NumPages; count++) {
			DataFlash.StartRead(count);
			Serial.printf_P(PSTR("DF page, log file #, log page: %d,\t"), count);
			Serial.printf_P(PSTR("%d,\t"), DataFlash.GetFileNumber());
			Serial.printf_P(PSTR("%d\n"), DataFlash.GetFilePage());
		}
		return(-1);
	} else if (dump_log <= 0) {
		Serial.printf_P(PSTR("dumping all\n"));
		Log_Read(1, DataFlash.df_NumPages);
		return(-1);
	} else if ((argc != 2) || (dump_log <= (last_log_num - get_num_logs())) || (dump_log > last_log_num)) {
		Serial.printf_P(PSTR("bad log number\n"));
		return(-1);
	}

	get_log_boundaries(dump_log, dump_log_start, dump_log_end);
	Serial.printf_P(PSTR("Dumping Log %d,    start pg %d,   end pg %d\n"),
				  dump_log,
				  dump_log_start,
				  dump_log_end);

	Log_Read(dump_log_start, dump_log_end);
	Serial.printf_P(PSTR("Done\n"));
    return 0;
}

static void
do_erase_logs(void (*delay_cb)(unsigned long))
{
	Serial.printf_P(PSTR("\nErasing log...\n"));
	DataFlash.SetFileNumber(0xFFFF);
	for(uint16_t j = 1; j <= DataFlash.df_NumPages; j++) {
		DataFlash.PageErase(j);
		DataFlash.StartWrite(j);		// We need this step to clean FileNumbers
		if(j%128 == 0) Serial.printf_P(PSTR("+"));
        delay_cb(1);
	}

	Serial.printf_P(PSTR("\nLog erased.\n"));
	DataFlash.FinishWrite();
}

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    do_erase_logs(delay);
    return 0;
}

static int8_t
select_logs(uint8_t argc, const Menu::arg *argv)
{
	uint16_t	bits;

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
		#define TARG(_s)	if (!strcasecmp_P(argv[1].str, PSTR(#_s))) bits |= MASK_LOG_ ## _s
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


// This function determines the number of whole or partial log files in the DataFlash
// Wholly overwritten files are (of course) lost.
static byte get_num_logs(void)
{
	uint16_t lastpage;
	uint16_t last;
	uint16_t first;

	if(find_last_page() == 1) return 0;

	DataFlash.StartRead(1);
	
	if(DataFlash.GetFileNumber() == 0XFFFF) return 0;

	lastpage = find_last_page();
	DataFlash.StartRead(lastpage);
	last = DataFlash.GetFileNumber();
	DataFlash.StartRead(lastpage + 2);
	first = DataFlash.GetFileNumber();
	if(first > last) {
		DataFlash.StartRead(1);
		first = DataFlash.GetFileNumber();
	}
	if(last == first)
	{
		return 1;
	} else {
		return (last - first + 1);
	}
}

// This function starts a new log file in the DataFlash
static void start_new_log()
{
	uint16_t	last_page = find_last_page();
	
	DataFlash.StartRead(last_page);
	//Serial.print("last page: ");	Serial.println(last_page);
	//Serial.print("file #: ");	Serial.println(DataFlash.GetFileNumber());
	//Serial.print("file page: ");	Serial.println(DataFlash.GetFilePage());
	
	if(find_last_log() == 0 || DataFlash.GetFileNumber() == 0xFFFF) {
		DataFlash.SetFileNumber(1);
		DataFlash.StartWrite(1);
		//Serial.println("start log from 0");
		return;
	}
	
	// Check for log of length 1 page and suppress
	if(DataFlash.GetFilePage() <= 1) {
		DataFlash.SetFileNumber(DataFlash.GetFileNumber());		// Last log too short, reuse its number
		DataFlash.StartWrite(last_page);					// and overwrite it
		//Serial.println("start log from short");
	} else {
		if(last_page == 0xFFFF) last_page=0;
		DataFlash.SetFileNumber(DataFlash.GetFileNumber()+1);
		DataFlash.StartWrite(last_page + 1);
		//Serial.println("start log normal");
	}
}

// This function finds the first and last pages of a log file
// The first page may be greater than the last page if the DataFlash has been filled and partially overwritten.
static void get_log_boundaries(byte log_num, int & start_page, int & end_page)
{
	int num = get_num_logs();
	int look;
	
	if(num == 1)
	{
		DataFlash.StartRead(DataFlash.df_NumPages);
		if(DataFlash.GetFileNumber() == 0xFFFF)
		{
			start_page = 1;
			end_page = find_last_page_of_log((uint16_t)log_num);
		} else {
			end_page = find_last_page_of_log((uint16_t)log_num);
			start_page = end_page + 1;
		}

	} else {
		if(log_num==1) {
			DataFlash.StartRead(DataFlash.df_NumPages);
			if(DataFlash.GetFileNumber() == 0xFFFF) {
				start_page = 1;
			} else {
				start_page = find_last_page() + 1;
			}
		 } else {
			if(log_num == find_last_log() - num + 1) {
				start_page = find_last_page() + 1;
			} else {
				look = log_num-1;
				do {
				start_page = find_last_page_of_log(look) + 1;
				look--;
				} while (start_page <= 0 && look >=1);
			}
		}
	}
	if(start_page == (int)DataFlash.df_NumPages+1 || start_page == 0) start_page=1;
	end_page = find_last_page_of_log((uint16_t)log_num);
	if(end_page <= 0) end_page = start_page;
}

static bool check_wrapped(void)
{
	DataFlash.StartRead(DataFlash.df_NumPages);
	if(DataFlash.GetFileNumber() == 0xFFFF)
		return 0;
	else
		return 1;
}


// This funciton finds the last log number
static int find_last_log(void)
{
	int last_page = find_last_page();
	DataFlash.StartRead(last_page);
	return DataFlash.GetFileNumber();
}
	
// This function finds the last page of the last file
static int find_last_page(void)
{
uint16_t look;
uint16_t bottom = 1;
uint16_t top = DataFlash.df_NumPages;
uint32_t look_hash;
uint32_t bottom_hash;
uint32_t top_hash;

		DataFlash.StartRead(bottom);
		bottom_hash = ((long)DataFlash.GetFileNumber()<<16) | DataFlash.GetFilePage();
		
		while(top-bottom > 1)
		{
			look = (top+bottom)/2;
			DataFlash.StartRead(look);
			look_hash = (long)DataFlash.GetFileNumber()<<16 | DataFlash.GetFilePage();
			if (look_hash >= 0xFFFF0000) look_hash = 0;
			
			if(look_hash < bottom_hash) {
				// move down
				top = look;
			} else {
				// move up
				bottom = look;
				bottom_hash = look_hash;
			}
		}
		
		DataFlash.StartRead(top);
		top_hash = ((long)DataFlash.GetFileNumber()<<16) | DataFlash.GetFilePage();
		if (top_hash >= 0xFFFF0000) top_hash = 0;
		if (top_hash > bottom_hash)
		{
			return top;
		} else {
			return bottom;
		}
}

// This function finds the last page of a particular log file
static int find_last_page_of_log(uint16_t log_number)
{

uint16_t look;
uint16_t bottom;
uint16_t top;
uint32_t look_hash;
uint32_t check_hash;

	if(check_wrapped())
	{
		DataFlash.StartRead(1);
		bottom = DataFlash.GetFileNumber();
		if (bottom > log_number)
		{
			bottom = find_last_page();
			top = DataFlash.df_NumPages;
		} else {
			bottom = 1;
			top = find_last_page();
		}
	} else {
		bottom = 1;
		top = find_last_page();
	}

	check_hash = (long)log_number<<16 | 0xFFFF;
		
	while(top-bottom > 1)
	{
		look = (top+bottom)/2;
		DataFlash.StartRead(look);
		look_hash = (long)DataFlash.GetFileNumber()<<16 | DataFlash.GetFilePage();
		if (look_hash >= 0xFFFF0000) look_hash = 0;
		
		if(look_hash > check_hash) {
			// move down
			top = look;
		} else {
			// move up
			bottom = look;
		}
	}
		
	DataFlash.StartRead(top);
	if (DataFlash.GetFileNumber() == log_number) return top;
	
	DataFlash.StartRead(bottom);
	if (DataFlash.GetFileNumber() == log_number) return bottom;
	
	return -1;
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
	DataFlash.WriteByte(dcm.gyro_sat_count);
	DataFlash.WriteByte(imu.adc_constraints);
	DataFlash.WriteByte(dcm.renorm_sqrt_count);
	DataFlash.WriteByte(dcm.renorm_blowup_count);
	DataFlash.WriteByte(gps_fix_count);
	DataFlash.WriteInt((int)(dcm.get_health() * 1000));
	DataFlash.WriteInt((int)(dcm.get_integrator().x * 1000));
	DataFlash.WriteInt((int)(dcm.get_integrator().y * 1000));
	DataFlash.WriteInt((int)(dcm.get_integrator().z * 1000));
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

	for (int i = 1; i <= g.command_total; i++){
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
	DataFlash.WriteInt((int)(g.channel_roll.servo_out));
	DataFlash.WriteInt((int)nav_roll);
	DataFlash.WriteInt((int)dcm.roll_sensor);
	DataFlash.WriteInt((int)(g.channel_pitch.servo_out));
	DataFlash.WriteInt((int)nav_pitch);
	DataFlash.WriteInt((int)dcm.pitch_sensor);
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
	DataFlash.WriteInt((uint16_t)dcm.yaw_sensor);
	DataFlash.WriteInt((int)wp_distance);
	DataFlash.WriteInt((uint16_t)target_bearing);
	DataFlash.WriteInt((uint16_t)nav_bearing);
	DataFlash.WriteInt(altitude_error);
	DataFlash.WriteInt((int)airspeed);
	DataFlash.WriteInt((int)(nav_gain_scaler*1000));
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
static void Log_Write_GPS(	int32_t log_Time, int32_t log_Lattitude, int32_t log_Longitude, int32_t log_gps_alt, int32_t log_mix_alt,
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
	DataFlash.WriteInt(sonar_alt);				// This one is just temporary for testing out sonar in fixed wing
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
	gyro *= t7;								// Scale up for storage as long integers
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
	DataFlash.WriteInt((int)(battery_voltage 	* 100.0));
	DataFlash.WriteInt((int)(current_amps 		* 100.0));
	DataFlash.WriteInt((int)current_total);
	DataFlash.WriteByte(END_BYTE);
}

// Read a Current packet
static void Log_Read_Current()
{
	Serial.printf_P(PSTR("CURR: %d, %4.4f, %4.4f, %d\n"),
			DataFlash.ReadInt(),
			((float)DataFlash.ReadInt() / 100.f),
			((float)DataFlash.ReadInt() / 100.f),
			DataFlash.ReadInt());
}

// Read an control tuning packet
static void Log_Read_Control_Tuning()
{
	float logvar;

	Serial.printf_P(PSTR("CTUN:"));
	for (int y = 1; y < 10; y++) {
		logvar = DataFlash.ReadInt();
		if(y < 8) logvar 	= logvar/100.f;
		if(y == 9) logvar 	= logvar/10000.f;
		Serial.print(logvar);
		Serial.print(comma);
	}
	Serial.println(" ");
}

// Read a nav tuning packet
static void Log_Read_Nav_Tuning()
{
    int16_t d[7];
    for (int8_t i=0; i<7; i++) {
        d[i] = DataFlash.ReadInt();
    }
	Serial.printf_P(PSTR("NTUN: %4.4f, %d, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f,\n"),		// \n
                    d[0]/100.0,
                    d[1],
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
	int logvar;

	Serial.printf_P(PSTR("PM:"));
	pm_time = DataFlash.ReadLong();
	Serial.print(pm_time);
	Serial.print(comma);
	for (int y = 1; y <= 12; y++) {
		if(y < 3 || y > 7){
			logvar = DataFlash.ReadInt();
		}else{
			logvar = DataFlash.ReadByte();
		}
		Serial.print(logvar);
		Serial.print(comma);
	}
	Serial.println(" ");
}

// Read a command processing packet
static void Log_Read_Cmd()
{
	byte logvarb;
	int32_t logvarl;

	Serial.printf_P(PSTR("CMD:"));
	for(int i = 1; i < 4; i++) {
		logvarb = DataFlash.ReadByte();
		Serial.print(logvarb, DEC);
		Serial.print(comma);
	}
	for(int i = 1; i < 4; i++) {
		logvarl = DataFlash.ReadLong();
		Serial.print(logvarl, DEC);
		Serial.print(comma);
	}
	Serial.println(" ");
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
                    d[0], d[1],
                    (uint16_t)d[2]);
}

// Read a mode packet
static void Log_Read_Mode()
{
	Serial.printf_P(PSTR("MOD:"));
	Serial.println(flight_mode_strings[DataFlash.ReadByte()]);
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
	for (int y = 0; y < 6; y++) {
		logvar = (float)DataFlash.ReadLong() / t7;
		Serial.print(logvar);
		Serial.print(comma);
	}
	Serial.println(" ");
}

// Read the DataFlash log memory : Packet Parser
static void Log_Read(int16_t start_page, int16_t end_page)
{
	int packet_count = 0;

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

	Serial.printf_P(PSTR("Number of packets read: %d\n"), packet_count);
}

// Read the DataFlash log memory : Packet Parser
static int Log_Read_Process(int16_t start_page, int16_t end_page)
{
	byte data;
	byte log_step = 0;
	int page = start_page;
	int packet_count = 0;

	DataFlash.StartRead(start_page);
	while (page < end_page && page != -1){
		data = DataFlash.ReadByte();

		switch(log_step)		 // This is a state machine to read the packets
			{
			case 0:
				if(data == HEAD_BYTE1)	// Head byte 1
					log_step++;
				break;
			case 1:
				if(data == HEAD_BYTE2)	// Head byte 2
					log_step++;
				else
					log_step = 0;
				break;
			case 2:
				if(data == LOG_ATTITUDE_MSG){
					Log_Read_Attitude();
					log_step++;

				}else if(data == LOG_MODE_MSG){
					Log_Read_Mode();
					log_step++;

				}else if(data == LOG_CONTROL_TUNING_MSG){
					Log_Read_Control_Tuning();
					log_step++;

				}else if(data == LOG_NAV_TUNING_MSG){
					Log_Read_Nav_Tuning();
					log_step++;

				}else if(data == LOG_PERFORMANCE_MSG){
					Log_Read_Performance();
					log_step++;

				}else if(data == LOG_RAW_MSG){
					Log_Read_Raw();
					log_step++;

				}else if(data == LOG_CMD_MSG){
					Log_Read_Cmd();
					log_step++;

				}else if(data == LOG_CURRENT_MSG){
					Log_Read_Current();
					log_step++;

				}else if(data == LOG_STARTUP_MSG){
					Log_Read_Startup();
					log_step++;
				}else {
					if(data == LOG_GPS_MSG){
						Log_Read_GPS();
						log_step++;
					}else{
						Serial.printf_P(PSTR("Error Reading Packet: %d\n"),packet_count);
						log_step = 0;	 // Restart, we have a problem...
					}
				}
				break;
			case 3:
				if(data == END_BYTE){
					 packet_count++;
				}else{
					Serial.printf_P(PSTR("Error Reading END_BYTE: %d\n"),data);
				}
				log_step = 0;			// Restart sequence: new packet...
				break;
			}
		page = DataFlash.GetPage();
		}
	return packet_count;
}

#else // LOGGING_ENABLED

// dummy functions
static void Log_Write_Mode(byte mode) {}
static void Log_Write_Startup(byte type) {}
static void Log_Write_Cmd(byte num, struct Location *wp) {}
static void Log_Write_Current() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_GPS(	int32_t log_Time, int32_t log_Lattitude, int32_t log_Longitude, int32_t log_gps_alt, int32_t log_mix_alt,
                            int32_t log_Ground_Speed, int32_t log_Ground_Course, byte log_Fix, byte log_NumSats) {}
static void Log_Write_Performance() {}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
static byte get_num_logs(void) { return 0; }
static void start_new_log() {}
static void Log_Write_Attitude(int16_t log_roll, int16_t log_pitch, uint16_t log_yaw) {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Raw() {}


#endif // LOGGING_ENABLED
