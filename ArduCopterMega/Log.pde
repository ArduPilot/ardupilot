// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

#define HEAD_BYTE1 	0xA3	// Decimal 163
#define HEAD_BYTE2 	0x95	// Decimal 149
#define END_BYTE	0xBA	// Decimal 186


// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static bool     print_log_menu(void);
static int8_t	dump_log(uint8_t argc, 			const Menu::arg *argv);
static int8_t	erase_logs(uint8_t argc, 		const Menu::arg *argv);
static int8_t	select_logs(uint8_t argc, 		const Menu::arg *argv);

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
static int8_t	help_log(uint8_t argc, 			const Menu::arg *argv)
{
	Serial.printf_P(PSTR("\n"
						 "Commands:\n"
						 "  dump <n>"
						 "  erase (all logs)\n"
						 "  enable <name> | all\n"
						 "  disable <name> | all\n"
						 "\n"));
    return 0;
}

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command log_menu_commands[] PROGMEM = {
	{"dump",	dump_log},
	{"erase",	erase_logs},
	{"enable",	select_logs},
	{"disable",	select_logs},
	{"help",	help_log}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

static void get_log_boundaries(byte log_num, int & start_page, int & end_page);

static bool
print_log_menu(void)
{
	int log_start;
	int log_end;
	byte last_log_num = get_num_logs();

	Serial.printf_P(PSTR("logs enabled: "));

	if (0 == g.log_bitmask) {
		Serial.printf_P(PSTR("none"));
	}else{
		if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST)	Serial.printf_P(PSTR(" ATTITUDE_FAST"));
		if (g.log_bitmask & MASK_LOG_ATTITUDE_MED)	Serial.printf_P(PSTR(" ATTITUDE_MED"));
		if (g.log_bitmask & MASK_LOG_GPS)			Serial.printf_P(PSTR(" GPS"));
		if (g.log_bitmask & MASK_LOG_PM)			Serial.printf_P(PSTR(" PM"));
		if (g.log_bitmask & MASK_LOG_CTUN)			Serial.printf_P(PSTR(" CTUN"));
		if (g.log_bitmask & MASK_LOG_NTUN)			Serial.printf_P(PSTR(" NTUN"));
		if (g.log_bitmask & MASK_LOG_RAW)			Serial.printf_P(PSTR(" RAW"));
		if (g.log_bitmask & MASK_LOG_CMD)			Serial.printf_P(PSTR(" CMD"));
		if (g.log_bitmask & MASK_LOG_CUR)			Serial.printf_P(PSTR(" CURRENT"));
		if (g.log_bitmask & MASK_LOG_MOTORS)		Serial.printf_P(PSTR(" MOTORS"));
		if (g.log_bitmask & MASK_LOG_OPTFLOW)		Serial.printf_P(PSTR(" OPTFLOW"));
	}

	Serial.println();

	if (last_log_num == 0) {
		Serial.printf_P(PSTR("\nNo logs\nType 'dump 0'.\n\n"));
	}else{
		Serial.printf_P(PSTR("\n%d logs\n"), last_log_num);

		for(int i = 1; i < last_log_num + 1; i++) {
			get_log_boundaries(i, log_start, log_end);
			//Serial.printf_P(PSTR("last_num %d "), last_log_num);
			Serial.printf_P(PSTR("Log # %d,    start %d,   end %d\n"), i, log_start, log_end);
		}
		Serial.println();
	}
	return(true);
}

static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
	byte dump_log;
	int dump_log_start;
	int dump_log_end;

	byte last_log_num = get_num_logs();
	//Serial.printf_P(PSTR("\n%d logs\n"), last_log_num);

	// check that the requested log number can be read
	dump_log = argv[1].i;

	if(dump_log == 99){
		Log_Read(1, 4096);
	}

	if (/*(argc != 2) || */ (dump_log < 1) || (dump_log > last_log_num)) {
		Serial.printf_P(PSTR("bad log # %d\n"), dump_log);
		Log_Read(1, 4095);
		erase_logs(NULL, NULL);
		return(-1);
	}

	get_log_boundaries(dump_log, dump_log_start, dump_log_end);
	/*Serial.printf_P(PSTR("Dumping Log number %d,    start %d,   end %d\n"),
				  dump_log,
				  dump_log_start,
				  dump_log_end);
	*/
	Log_Read(dump_log_start, dump_log_end);
	//Serial.printf_P(PSTR("Done\n"));
	return (0);
}

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
	//for(int i = 10 ; i > 0; i--) {
	//	Serial.printf_P(PSTR("ATTENTION - Erasing log in %d seconds.\n"), i);
	//	delay(1000);
	//}

	// lay down a bunch of "log end" messages.
	Serial.printf_P(PSTR("\nErasing log...\n"));
	for(int j = 1; j < 4096; j++)
		DataFlash.PageErase(j);

	DataFlash.StartWrite(1);
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_INDEX_MSG);
	DataFlash.WriteByte(0);
	DataFlash.WriteByte(END_BYTE);
	DataFlash.FinishWrite();
	Serial.printf_P(PSTR("\nLog erased.\n"));
	return (0);
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
		TARG(MOTORS);
		TARG(OPTFLOW);
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


// finds out how many logs are available
static byte get_num_logs(void)
{
	int page = 1;
	byte data;
	byte log_step = 0;

	DataFlash.StartRead(1);

	while (page == 1) {
		data = DataFlash.ReadByte();

		switch(log_step){		 //This is a state machine to read the packets
			case 0:
				if(data==HEAD_BYTE1)	// Head byte 1
					log_step++;
				break;

			case 1:
				if(data==HEAD_BYTE2)	// Head byte 2
					log_step++;
				else
					log_step = 0;
				break;

			case 2:
				if(data == LOG_INDEX_MSG){
					byte num_logs = DataFlash.ReadByte();
					//Serial.printf("num_logs, %d\n", num_logs);

					return num_logs;
				}else{
					//Serial.printf("* %d\n", data);
					log_step = 0;	 // Restart, we have a problem...
				}
				break;
			}
		page = DataFlash.GetPage();
	}
	return 0;
}

// send the number of the last log?
static void start_new_log()
{
	byte num_existing_logs = get_num_logs();

	int start_pages[50] = {0,0,0};
	int end_pages[50]	= {0,0,0};

	if(num_existing_logs > 0){
		for(int i = 0; i < num_existing_logs; i++) {
			get_log_boundaries(i + 1, start_pages[i], end_pages[i]);
		}
		end_pages[num_existing_logs - 1] = find_last_log_page(start_pages[num_existing_logs - 1]);
	}

	if((end_pages[num_existing_logs - 1] < 4095) && (num_existing_logs < MAX_NUM_LOGS /*50*/)) {

		if(num_existing_logs > 0)
			start_pages[num_existing_logs] = end_pages[num_existing_logs - 1] + 1;
		else
			start_pages[0] = 2;

		num_existing_logs++;

		DataFlash.StartWrite(1);
		DataFlash.WriteByte(HEAD_BYTE1);
		DataFlash.WriteByte(HEAD_BYTE2);
		DataFlash.WriteByte(LOG_INDEX_MSG);
		DataFlash.WriteByte(num_existing_logs);

		for(int i = 0; i < MAX_NUM_LOGS; i++) {
			DataFlash.WriteInt(start_pages[i]);
			DataFlash.WriteInt(end_pages[i]);
		}

		DataFlash.WriteByte(END_BYTE);
		DataFlash.FinishWrite();
		DataFlash.StartWrite(start_pages[num_existing_logs - 1]);

	}else{
		gcs.send_text_P(SEVERITY_LOW,PSTR("Logs full"));
	}
}

// All log data is stored in page 1?
static void get_log_boundaries(byte log_num, int & start_page, int & end_page)
{
	int page 		= 1;
	byte data;
	byte log_step	= 0;

	DataFlash.StartRead(1);
	while (page == 1) {
		data = DataFlash.ReadByte();
		switch(log_step)		 //This is a state machine to read the packets
			{
			case 0:
				if(data==HEAD_BYTE1)	// Head byte 1
					log_step++;
				break;
			case 1:
				if(data==HEAD_BYTE2)	// Head byte 2
					log_step++;
				else
					log_step = 0;
				break;
			case 2:
				if(data==LOG_INDEX_MSG){
					byte num_logs = DataFlash.ReadByte();
					for(int i=0;i<log_num;i++) {
						start_page = DataFlash.ReadInt();
						end_page = DataFlash.ReadInt();
					}
					if(log_num==num_logs)
						end_page = find_last_log_page(start_page);

					return;		// This is the normal exit point
				}else{
						log_step=0;	 // Restart, we have a problem...
				}
				break;
			}
		page = DataFlash.GetPage();
	}
	//  Error condition if we reach here with page = 2   TO DO - report condition
}

//
static int find_last_log_page(int bottom_page)
{
	int top_page = 4096;
	int look_page;
	long check;

	while((top_page - bottom_page) > 1) {
		look_page = (top_page + bottom_page) / 2;
		DataFlash.StartRead(look_page);
		check = DataFlash.ReadLong();

		//Serial.printf("look page:%d, check:%d\n", look_page, check);

		if(check == (long)0xFFFFFFFF)
			top_page = look_page;
		else
			bottom_page = look_page;
	}
	return top_page;
}

// Write an GPS packet. Total length : 30 bytes
static void Log_Write_GPS()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_GPS_MSG);

	DataFlash.WriteLong(g_gps->time);						// 1
	DataFlash.WriteByte(g_gps->fix);						// 2
	DataFlash.WriteByte(g_gps->num_sats);					// 3

	DataFlash.WriteLong(current_loc.lat);					// 4
	DataFlash.WriteLong(current_loc.lng);					// 5
	DataFlash.WriteLong(g_gps->altitude);					// 6
	DataFlash.WriteLong(current_loc.alt);					// 7

	DataFlash.WriteInt(g_gps->ground_speed);				// 8
	DataFlash.WriteInt((uint16_t)g_gps->ground_course);				// 9

	DataFlash.WriteByte(END_BYTE);
}

// Read a GPS packet
static void Log_Read_GPS()
{
	Serial.printf_P(PSTR("GPS, %ld, %d, %d, "
					  	"%4.7f, %4.7f, %4.4f, %4.4f, "
					  	"%d, %u\n"),

						DataFlash.ReadLong(),					// 1 time
						(int)DataFlash.ReadByte(),				// 2 fix
						(int)DataFlash.ReadByte(),				// 3 sats

						(float)DataFlash.ReadLong() / t7,		// 4 lat
						(float)DataFlash.ReadLong() / t7,		// 5 lon
						(float)DataFlash.ReadLong() / 100.0,	// 6 gps alt
						(float)DataFlash.ReadLong() / 100.0,	// 7 sensor alt

						DataFlash.ReadInt(),					// 8 ground speed
						(uint16_t)DataFlash.ReadInt());			// 9 ground course
}


// Write an raw accel/gyro data packet. Total length : 28 bytes
#if HIL_MODE != HIL_MODE_ATTITUDE
static void Log_Write_Raw()
{
	Vector3f gyro = imu.get_gyro();
	Vector3f accel = imu.get_accel();
	//Vector3f accel_filt	= imu.get_accel_filtered();

	gyro *= t7;								// Scale up for storage as long integers
	accel *= t7;
	//accel_filt *= t7;

	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_RAW_MSG);

	DataFlash.WriteLong((long)gyro.x);
	DataFlash.WriteLong((long)gyro.y);
	DataFlash.WriteLong((long)gyro.z);


	//DataFlash.WriteLong((long)(accels_rot.x * t7));
	//DataFlash.WriteLong((long)(accels_rot.y * t7));
	//DataFlash.WriteLong((long)(accels_rot.z * t7));

	DataFlash.WriteLong((long)accel.x);
	DataFlash.WriteLong((long)accel.y);
	DataFlash.WriteLong((long)accel.z);

	DataFlash.WriteByte(END_BYTE);
}
#endif

// Read a raw accel/gyro packet
static void Log_Read_Raw()
{
	float logvar;
	Serial.printf_P(PSTR("RAW,"));
	for (int y = 0; y < 6; y++) {
		logvar = (float)DataFlash.ReadLong() / t7;
		Serial.print(logvar);
		Serial.print(comma);
	}
	Serial.println(" ");
}

static void Log_Write_Current()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_CURRENT_MSG);

	DataFlash.WriteInt(g.rc_3.control_in);
	DataFlash.WriteLong(throttle_integrator);

	DataFlash.WriteInt((int)(battery_voltage 	* 100.0));
	DataFlash.WriteInt((int)(current_amps 		* 100.0));
	DataFlash.WriteInt((int)current_total);

	DataFlash.WriteByte(END_BYTE);
}

// Read a Current packet
static void Log_Read_Current()
{
	Serial.printf_P(PSTR("CURR: %d, %ld, %4.4f, %4.4f, %d\n"),
			DataFlash.ReadInt(),
			DataFlash.ReadLong(),

			((float)DataFlash.ReadInt() / 100.f),
			((float)DataFlash.ReadInt() / 100.f),
			DataFlash.ReadInt());
}

static void Log_Write_Motors()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_MOTORS_MSG);

	DataFlash.WriteInt(motor_out[CH_1]);
	DataFlash.WriteInt(motor_out[CH_2]);
	DataFlash.WriteInt(motor_out[CH_3]);
	DataFlash.WriteInt(motor_out[CH_4]);

	DataFlash.WriteByte(END_BYTE);
}

// Read a Current packet
static void Log_Read_Motors()
{
	Serial.printf_P(PSTR("MOT: %d, %d, %d, %d\n"),
			DataFlash.ReadInt(),
			DataFlash.ReadInt(),
			DataFlash.ReadInt(),
			DataFlash.ReadInt());
}

#ifdef OPTFLOW_ENABLED
// Write an optical flow packet. Total length : 18 bytes
static void Log_Write_Optflow()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_OPTFLOW_MSG);
	DataFlash.WriteInt((int)optflow.dx);
	DataFlash.WriteInt((int)optflow.dy);
	DataFlash.WriteInt((int)optflow.surface_quality);
	DataFlash.WriteLong(optflow.lat);//optflow_offset.lat + optflow.lat);
	DataFlash.WriteLong(optflow.lng);//optflow_offset.lng + optflow.lng);
	DataFlash.WriteByte(END_BYTE);
}
#endif

// Read an attitude packet
static void Log_Read_Optflow()
{
	Serial.printf_P(PSTR("OF, %d, %d, %d, %4.7f, %4.7f\n"),
			DataFlash.ReadInt(),
			DataFlash.ReadInt(),
			DataFlash.ReadInt(),
			(float)DataFlash.ReadLong(),// / t7,
			(float)DataFlash.ReadLong() // / t7
			);
}

static void Log_Write_Nav_Tuning()
{
	Matrix3f tempmat = dcm.get_dcm_matrix();

	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_NAV_TUNING_MSG);

	DataFlash.WriteInt((int)wp_distance);				// 1
	DataFlash.WriteByte(wp_verify_byte);				// 2
	DataFlash.WriteInt((int)(target_bearing/100));		// 3
	DataFlash.WriteInt((int)(nav_bearing/100));			// 4

	DataFlash.WriteInt((int)long_error);				// 5
	DataFlash.WriteInt((int)lat_error);					// 6
	DataFlash.WriteInt((int)nav_lon);					// 7
	DataFlash.WriteInt((int)nav_lat);					// 8

	DataFlash.WriteByte(END_BYTE);
}


static void Log_Read_Nav_Tuning()
{
							 //	 1	 2	 3	 4
	Serial.printf_P(PSTR( "NTUN, %d, %d, %d, %d, "
							    "%d, %d, %d, %d\n"),

				DataFlash.ReadInt(),	//distance
				DataFlash.ReadByte(),	//bitmask
				DataFlash.ReadInt(),	//target bearing
				DataFlash.ReadInt(),

				DataFlash.ReadInt(),
				DataFlash.ReadInt(),
				DataFlash.ReadInt(),
				DataFlash.ReadInt());	//nav bearing
}

// Write a control tuning packet. Total length : 22 bytes
#if HIL_MODE != HIL_MODE_ATTITUDE
static void Log_Write_Control_Tuning()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_CONTROL_TUNING_MSG);

	// Control
	DataFlash.WriteInt((int)(g.rc_4.control_in/100));
	DataFlash.WriteInt((int)(g.rc_4.servo_out/100));

	// yaw
	DataFlash.WriteByte(yaw_debug);
	DataFlash.WriteInt((int)(dcm.yaw_sensor/100));
	DataFlash.WriteInt((int)(nav_yaw/100));
	DataFlash.WriteInt((int)yaw_error/100);
	DataFlash.WriteInt((int)(omega.z * 100));

	// Alt hold
	DataFlash.WriteInt(g.rc_3.servo_out);
	DataFlash.WriteInt(sonar_alt);					//
	DataFlash.WriteInt(baro_alt);					//

	DataFlash.WriteInt((int)next_WP.alt);				//
	DataFlash.WriteInt((int)altitude_error);			//
	DataFlash.WriteInt((int)g.pid_throttle.get_integrator());

	DataFlash.WriteByte(END_BYTE);
}
#endif

// Read an control tuning packet
static void Log_Read_Control_Tuning()
{
	Serial.printf_P(PSTR(   "CTUN, %d, %d, "
				"%d, %d, %d, %d, %1.4f, "
				"%d, %d, %d, %d, %d, %d\n"),

				// Control
				DataFlash.ReadInt(),
				DataFlash.ReadInt(),

				// yaw
				(int)DataFlash.ReadByte(),
				DataFlash.ReadInt(),
				DataFlash.ReadInt(),
				DataFlash.ReadInt(),
				(float)DataFlash.ReadInt(),// Gyro Rate

				// Alt Hold
				DataFlash.ReadInt(),
				DataFlash.ReadInt(),
				DataFlash.ReadInt(),

				DataFlash.ReadInt(),
				DataFlash.ReadInt(),
				DataFlash.ReadInt());
}

// Write a performance monitoring packet. Total length : 19 bytes
static void Log_Write_Performance()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_PERFORMANCE_MSG);

	//DataFlash.WriteByte(	delta_ms_fast_loop);
	//DataFlash.WriteByte(	loop_step);


	//*
	DataFlash.WriteLong(	millis()- perf_mon_timer);
	DataFlash.WriteInt (	mainLoop_count);
	DataFlash.WriteInt (	G_Dt_max);
	DataFlash.WriteByte(	dcm.gyro_sat_count);
	DataFlash.WriteByte(	imu.adc_constraints);
	DataFlash.WriteByte(	dcm.renorm_sqrt_count);
	DataFlash.WriteByte(	dcm.renorm_blowup_count);
	DataFlash.WriteByte(	gps_fix_count);
	DataFlash.WriteInt (	(int)(dcm.get_health() * 1000));
	//*/
	//PM,	20005,	3742, 10,0,0,0,0,89,1000,

	DataFlash.WriteByte(END_BYTE);
}

// Read a performance packet
static void Log_Read_Performance()
{
	//*
	long pm_time;
	int logvar;

	Serial.printf_P(PSTR("PM,"));
	pm_time = DataFlash.ReadLong();
	Serial.print(pm_time);
	Serial.print(comma);

	for (int y = 1; y < 9; y++) {
		if(y < 3 || y > 7){
			logvar = DataFlash.ReadInt();
		}else{
			logvar = DataFlash.ReadByte();
		}
		Serial.print(logvar);
		Serial.print(comma);
	}
	Serial.println(" ");
	//*/

	/*
	Serial.printf_P(PSTR("PM, %d, %d\n"),
			DataFlash.ReadByte(),
			DataFlash.ReadByte());
	//*/
}

// Write a command processing packet.
//void Log_Write_Cmd(byte num, byte id, byte p1, long alt, long lat, long lng)
static void Log_Write_Cmd(byte num, struct Location *wp)
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_CMD_MSG);

	DataFlash.WriteByte(g.waypoint_total);

	DataFlash.WriteByte(num);
	DataFlash.WriteByte(wp->id);
	DataFlash.WriteByte(wp->options);
	DataFlash.WriteByte(wp->p1);
	DataFlash.WriteLong(wp->alt);
	DataFlash.WriteLong(wp->lat);
	DataFlash.WriteLong(wp->lng);

	DataFlash.WriteByte(END_BYTE);
}
//CMD, 3, 0, 16, 8, 1, 800, 340440192, -1180692736


// Read a command processing packet
static void Log_Read_Cmd()
{
	Serial.printf_P(PSTR( "CMD, %d, %d, %d, %d, %d, %ld, %ld, %ld\n"),

				// WP total
				DataFlash.ReadByte(),

				// num, id, p1, options
				DataFlash.ReadByte(),
				DataFlash.ReadByte(),
				DataFlash.ReadByte(),
				DataFlash.ReadByte(),

				// Alt, lat long
				DataFlash.ReadLong(),
				DataFlash.ReadLong(),
				DataFlash.ReadLong());
}

// Write an attitude packet. Total length : 10 bytes
static void Log_Write_Attitude()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_ATTITUDE_MSG);
	DataFlash.WriteInt((int)dcm.roll_sensor);
	DataFlash.WriteInt((int)dcm.pitch_sensor);
	DataFlash.WriteInt((uint16_t)dcm.yaw_sensor);
	DataFlash.WriteByte(END_BYTE);
}

// Read an attitude packet
static void Log_Read_Attitude()
{
	Serial.printf_P(PSTR("ATT, %d, %d, %u\n"),
			DataFlash.ReadInt(),
			DataFlash.ReadInt(),
			(uint16_t)DataFlash.ReadInt());
}


// Write a mode packet. Total length : 5 bytes
static void Log_Write_Mode(byte mode)
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
	Serial.printf_P(PSTR("MOD:"));
	Serial.print(flight_mode_strings[DataFlash.ReadByte()]);
	Serial.printf_P(PSTR(", %d\n"),DataFlash.ReadInt());
}

static void Log_Write_Startup()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_STARTUP_MSG);
	DataFlash.WriteByte(END_BYTE);
}

// Read a mode packet
static void Log_Read_Startup()
{
	Serial.printf_P(PSTR("START UP\n"));
}


// Read the DataFlash log memory : Packet Parser
static void Log_Read(int start_page, int end_page)
{
	byte data;
	byte log_step 		= 0;
	int page 			= start_page;

	DataFlash.StartRead(start_page);

	while (page < end_page && page != -1){

		data = DataFlash.ReadByte();

		// This is a state machine to read the packets
		switch(log_step){
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
				log_step = 0;
				switch(data){
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
				}
				break;
		}
		page = DataFlash.GetPage();
	}
}

#else // LOGGING_ENABLED

static void Log_Write_Startup() {}
static void Log_Read_Startup() {}
static void Log_Read(int start_page, int end_page) {}
static void Log_Write_Cmd(byte num, struct Location *wp) {}
static void Log_Write_Mode(byte mode) {}
static void start_new_log() {}
static void Log_Write_Raw() {}
static void Log_Write_GPS() {}
static void Log_Write_Current() {}
static void Log_Write_Attitude() {}
#ifdef OPTFLOW_ENABLED
static void Log_Write_Optflow() {}
#endif
static void Log_Write_Nav_Tuning() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Motors() {}
static void Log_Write_Performance() {}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) { return 0; }

#endif // LOGGING_ENABLED
