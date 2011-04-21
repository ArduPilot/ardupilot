// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

#define HEAD_BYTE1 0xA3    // Decimal 163
#define HEAD_BYTE2 0x95    // Decimal 149
#define END_BYTE 0xBA      // Decimal 186


// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	print_log_menu(uint8_t argc, 	const Menu::arg *argv);
static int8_t	dump_log(uint8_t argc, 			const Menu::arg *argv);
static int8_t	erase_logs(uint8_t argc, 		const Menu::arg *argv);
static int8_t	select_logs(uint8_t argc, 		const Menu::arg *argv);

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
static int8_t	help_log(uint8_t argc, 			const Menu::arg *argv)
{
	// log hack
	//Serial.begin(115200, 128, 128);

	Serial.printf_P(PSTR("\n"
						 "Commands:\n"
						 "  dump <n>"
						 "  erase (all logs)\n"
						 "  enable <name> | all\n"
						 "  disable <name> | all\n"
						 "\n"));
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
		PLOG(CURRENT);
		#undef PLOG
	}
	Serial.println();

	if (last_log_num == 0) {
		Serial.printf_P(PSTR("\nNo logs\n"));
	}else{
		Serial.printf_P(PSTR("\n%d logs\n"), last_log_num);
		for(int i = 1; i < last_log_num + 1; i++) {
			get_log_boundaries(last_log_num, i, log_start, log_end);
			Serial.printf_P(PSTR("Log # %d,    start %d,   end %d\n"),
							i, log_start, log_end);
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
	byte last_log_num;

	// check that the requested log number can be read
	dump_log = argv[1].i;
	last_log_num = get_num_logs();
	if ((argc != 2) || (dump_log < 1) || (dump_log > last_log_num)) {
		Serial.printf_P(PSTR("bad log number\n"));
		return(-1);
	}

	get_log_boundaries(last_log_num, dump_log, dump_log_start, dump_log_end);
	Serial.printf_P(PSTR("Dumping Log number %d,    start %d,   end %d\n"),
				  dump_log,
				  dump_log_start,
				  dump_log_end);

	Log_Read(dump_log_start, dump_log_end);
	Serial.printf_P(PSTR("Done\n"));
}

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
	for(int i = 10 ; i > 0; i--) {
		Serial.printf_P(PSTR("ATTENTION - Erasing log in %d seconds.\n"), i);
		delay(1000);
	}
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
		bits = ~(bits = 0);
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

int8_t
process_logs(uint8_t argc, const Menu::arg *argv)
{
	log_menu.run();
}


byte get_num_logs(void)
{
	int page = 1;
	byte data;
	byte log_step = 0;

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
					return num_logs;
				}else{
						log_step=0;	 // Restart, we have a problem...
				}
				break;
			}
		page = DataFlash.GetPage();
	}
	return 0;
}

void start_new_log(byte num_existing_logs)
{
	int page;
	int start_pages[50] = {0,0,0};
	int end_pages[50]	= {0,0,0};
	byte data;

	if (num_existing_logs > 0) {
		for(int i=0;i<num_existing_logs;i++) {
			get_log_boundaries(num_existing_logs, i+1,start_pages[i],end_pages[i]);
		}
		end_pages[num_existing_logs - 1] = find_last_log_page(start_pages[num_existing_logs - 1]);
	}

	if (end_pages[num_existing_logs - 1] < 4095 && num_existing_logs < MAX_NUM_LOGS) {
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

		for(int i=0;i<MAX_NUM_LOGS;i++) {
			DataFlash.WriteInt(start_pages[i]);
			DataFlash.WriteInt(end_pages[i]);
		}

		DataFlash.WriteByte(END_BYTE);
		DataFlash.FinishWrite();
		DataFlash.StartWrite(start_pages[num_existing_logs-1]);

	}else{
		gcs.send_text_P(SEVERITY_LOW,PSTR("<start_new_log> Logs full"));
	}
}

void get_log_boundaries(byte num_logs, byte log_num, int & start_page, int & end_page)
{
	int page 		= 1;
	byte data;
	byte log_step	= 0;

	DataFlash.StartRead(1);
	while (page = 1) {
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

int find_last_log_page(int bottom_page)
{
	int top_page = 4096;
	int look_page;
	long check;

	while((top_page - bottom_page) > 1) {
		look_page = (top_page + bottom_page) / 2;
		DataFlash.StartRead(look_page);
		check = DataFlash.ReadLong();
		if(check == 0xFFFFFFFF)
			top_page = look_page;
		else
			bottom_page = look_page;
	}
	return top_page;
}


// Write an attitude packet. Total length : 10 bytes
void Log_Write_Attitude(int log_roll, int log_pitch, uint16_t log_yaw)
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
void Log_Write_Performance()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_PERFORMANCE_MSG);
	DataFlash.WriteLong(millis()- perf_mon_timer);
	DataFlash.WriteInt(mainLoop_count);
	DataFlash.WriteInt(G_Dt_max);
	DataFlash.WriteByte(dcm.gyro_sat_count);
	DataFlash.WriteByte(imu.adc_constraints);
	DataFlash.WriteByte(dcm.renorm_sqrt_count);
	DataFlash.WriteByte(dcm.renorm_blowup_count);
	DataFlash.WriteByte(gps_fix_count);
	DataFlash.WriteInt((int)(dcm.get_health() * 1000));
	DataFlash.WriteByte(END_BYTE);
}

// Write a command processing packet. Total length : 19 bytes
//void Log_Write_Cmd(byte num, byte id, byte p1, long alt, long lat, long lng)
void Log_Write_Cmd(byte num, struct Location *wp)
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_CMD_MSG);
	DataFlash.WriteByte(num);
	DataFlash.WriteByte(wp->id);
	DataFlash.WriteByte(wp->p1);
	DataFlash.WriteByte(wp->options);
	DataFlash.WriteLong(wp->alt);
	DataFlash.WriteLong(wp->lat);
	DataFlash.WriteLong(wp->lng);
	DataFlash.WriteByte(END_BYTE);
}

void Log_Write_Nav_Tuning()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_NAV_TUNING_MSG);

	DataFlash.WriteInt((int)(dcm.yaw_sensor/100));		// 1
	DataFlash.WriteInt((int)wp_distance);				// 2
	DataFlash.WriteInt((int)(target_bearing/100));		// 3
	DataFlash.WriteInt((int)(nav_bearing/100));			// 4

	DataFlash.WriteInt((int)(g.rc_3.servo_out));		// 5
	DataFlash.WriteByte(altitude_sensor);				// 6
	DataFlash.WriteInt((int)sonar_alt);					// 7
	DataFlash.WriteInt((int)baro_alt);					// 8

	DataFlash.WriteInt(home.alt);						// 9
	DataFlash.WriteInt((int)next_WP.alt);				// 11
	DataFlash.WriteInt((int)altitude_error);			// 11

	DataFlash.WriteByte(END_BYTE);
}

void Log_Read_Nav_Tuning()
{
							 //	1	2	3	4	5	6	7	8	9	10  11
	Serial.printf_P(PSTR("NTUN, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n"),
				DataFlash.ReadInt(),	//yaw sensor
				DataFlash.ReadInt(),	//distance
				DataFlash.ReadInt(),	//target bearing
				DataFlash.ReadInt(),	//nav bearing

				DataFlash.ReadInt(),	//throttle
				DataFlash.ReadByte(),	//Alt sensor
				DataFlash.ReadInt(),	//Sonar
				DataFlash.ReadInt(),	//Baro

				DataFlash.ReadInt(),	//home.alt
				DataFlash.ReadInt(),	//Next_alt
				DataFlash.ReadInt());	//Alt Error
}


// Write a mode packet. Total length : 5 bytes
void Log_Write_Mode(byte mode)
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_MODE_MSG);
	DataFlash.WriteByte(mode);
	DataFlash.WriteByte(END_BYTE);
}

// Write an GPS packet. Total length : 30 bytes
void Log_Write_GPS()
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
	DataFlash.WriteInt((int)(g_gps->ground_course/100));	// 9

	DataFlash.WriteByte(END_BYTE);
}

// Read a GPS packet
void Log_Read_GPS()
{						//	1	2	3 4				5			6		 7			8        9
					//	GPS, t, 1, 8, 37.7659070, -122.4329400, 57.0500, 58.1400, 658.8400, -11636846.0000
							//	1   2   3   4      5		6	 7		 8		9
	Serial.printf_P(PSTR("GPS, %ld, %d, %d, %4.7f, %4.7f, %4.4f, %4.4f, %4.4f, %4.4f\n"),

			DataFlash.ReadLong(),					// 1 time
			(int)DataFlash.ReadByte(),				// 2 fix
			(int)DataFlash.ReadByte(),				// 3 sats

			(float)DataFlash.ReadLong() / t7,		// 4 lat
			(float)DataFlash.ReadLong() / t7,		// 5 lon
			(float)DataFlash.ReadLong() / 100.0,	// 6 gps alt
			(float)DataFlash.ReadLong() / 100.0,	// 7 sensor alt

			(float)DataFlash.ReadInt() / 100.0,		// 8 ground speed
			(float)DataFlash.ReadInt());			// 9 ground course
}


// Write an raw accel/gyro data packet. Total length : 28 bytes
#if HIL_MODE != HIL_MODE_ATTITUDE
void Log_Write_Raw()
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

void Log_Write_Current()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_CURRENT_MSG);
	DataFlash.WriteInt(g.rc_3.control_in);
	DataFlash.WriteInt((int)(current_voltage 	* 100.0));
	DataFlash.WriteInt((int)(current_amps 		* 100.0));
	DataFlash.WriteInt((int)current_total);
	DataFlash.WriteByte(END_BYTE);
}

// Read a Current packet
void Log_Read_Current()
{
	Serial.printf_P(PSTR("CURR: %d, %4.4f, %4.4f, %d\n"),
			DataFlash.ReadInt(),
			((float)DataFlash.ReadInt() / 100.f),
			((float)DataFlash.ReadInt() / 100.f),
			DataFlash.ReadInt());
}


// Write a control tuning packet. Total length : 22 bytes
#if HIL_MODE != HIL_MODE_ATTITUDE
void Log_Write_Control_Tuning()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_CONTROL_TUNING_MSG);

	// Control
	DataFlash.WriteInt((int)(g.rc_3.control_in));
	DataFlash.WriteInt((int)(g.rc_3.servo_out));
	DataFlash.WriteInt((int)(g.rc_4.control_in));
	DataFlash.WriteInt((int)(g.rc_4.servo_out));
	DataFlash.WriteInt((int)yaw_error);

	// Yaw mode
	DataFlash.WriteByte(yaw_debug);

	// Gyro Rates
	DataFlash.WriteInt((int)(omega.x * 1000));
	DataFlash.WriteInt((int)(omega.y * 1000));
	DataFlash.WriteInt((int)(omega.z * 1000));

	DataFlash.WriteInt((int)g.throttle_cruise);
	DataFlash.WriteInt((int)g.pid_baro_throttle.get_integrator());
	DataFlash.WriteInt((int)g.pid_sonar_throttle.get_integrator());

	// Position
	//DataFlash.WriteInt((int)dcm.pitch_sensor);
	//DataFlash.WriteInt((int)dcm.roll_sensor);
	//DataFlash.WriteInt((int)(dcm.yaw_sensor/10));

	DataFlash.WriteByte(END_BYTE);
}
#endif

// Read an control tuning packet
void Log_Read_Control_Tuning()
{
	Serial.printf_P(PSTR("CTUN, %d, %d, %d, %d, %d, %d, %1.4f, %1.4f, %1.4f, %d, %d, %d\n"),
				// Control
				DataFlash.ReadInt(),
				DataFlash.ReadInt(),
				DataFlash.ReadInt(),
				DataFlash.ReadInt(),

				DataFlash.ReadInt(),

				// Yaw Mode
				(int)DataFlash.ReadByte(),

				// Gyro Rates
				(float)DataFlash.ReadInt() / 1000.0,
				(float)DataFlash.ReadInt() / 1000.0,
				(float)DataFlash.ReadInt() / 1000.0,

				// Position
				//DataFlash.ReadInt(),
				//DataFlash.ReadInt(),
				//(long)DataFlash.ReadInt() * 10);

				// Alt Hold
				DataFlash.ReadInt(),
				DataFlash.ReadInt(),
				DataFlash.ReadInt());

}


// Read a performance packet
void Log_Read_Performance()
{
	long pm_time;
	int logvar;

	Serial.printf_P(PSTR("PM:"));
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
}

// Read a command processing packet
void Log_Read_Cmd()
{
	byte logvarb;
	long logvarl;

	Serial.printf_P(PSTR("CMD:"));
	for(int i = 1; i <= 4; i++) {
		logvarb = DataFlash.ReadByte();
		Serial.print(logvarb, DEC);
		Serial.print(comma);
	}
	for(int i = 1; i <= 3; i++) {
		logvarl = DataFlash.ReadLong();
		Serial.print(logvarl, DEC);
		Serial.print(comma);
	}
	Serial.println(" ");
}

// Read an attitude packet
void Log_Read_Attitude()
{
	Serial.printf_P(PSTR("ATT: %d, %d, %u\n"),
			DataFlash.ReadInt(),
			DataFlash.ReadInt(),
			(uint16_t)DataFlash.ReadInt());
}

// Read a mode packet
void Log_Read_Mode()
{
	Serial.printf_P(PSTR("MOD:"));
	Serial.println(flight_mode_strings[DataFlash.ReadByte()]);
}

// Read a raw accel/gyro packet
void Log_Read_Raw()
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
void Log_Read(int start_page, int end_page)
{
	byte data;
	byte log_step 		= 0;
	int packet_count 	= 0;
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
					// not implemented
					log_step++;

				}else if(data == LOG_GPS_MSG){
					Log_Read_GPS();
					log_step++;

				}else{
					Serial.printf_P(PSTR("Error P: %d\n"),packet_count);
					log_step = 0;	 // Restart, we have a problem...
				}
				break;

			case 3:
				if(data == END_BYTE){
					 packet_count++;
				}else{
					Serial.printf_P(PSTR("Error EB: %d\n"),data);
				}
				log_step = 0;			// Restart sequence: new packet...
				break;
		}

		page = DataFlash.GetPage();
	}
	//Serial.printf_P(PSTR("# of packets read: %d\n"), packet_count);
}


