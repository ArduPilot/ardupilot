// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

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
	Serial.printf_P(PSTR("\n"
						 "Commands:\n"
						 "  dump <n>             dump log <n>\n"
						 "  erase                erase all logs\n"
						 "  enable <name>|all    enable logging <name> or everything\n"
						 "  disable <name>|all   disable logging <name> or everything\n"
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
	byte last_log_num = eeprom_read_byte((uint8_t *) EE_LAST_LOG_NUM);

	Serial.printf_P(PSTR("logs enabled: "));
	if (0 == log_bitmask) {
		Serial.printf_P(PSTR("none"));
	} else {
		// Macro to make the following code a bit easier on the eye.
		// Pass it the capitalised name of the log option, as defined
		// in defines.h but without the LOG_ prefix.  It will check for
		// the bit being set and print the name of the log option to suit.
#define PLOG(_s)	if (log_bitmask & LOGBIT_ ## _s) Serial.printf_P(PSTR(" %S"), PSTR(#_s))
		PLOG(ATTITUDE_FAST);
		PLOG(ATTITUDE_MED);
		PLOG(GPS);
		PLOG(PM);
		PLOG(CTUN);
		PLOG(NTUN);
		PLOG(MODE);
		PLOG(RAW);
		PLOG(CMD);
#undef PLOG
	}
	Serial.println();

	if (last_log_num == 0) {
		Serial.printf_P(PSTR("\nNo logs available for download\n"));
	} else {

		Serial.printf_P(PSTR("\n%d logs available for download\n"), last_log_num);
		for(int i=1;i<last_log_num+1;i++) {
			log_start = eeprom_read_word((uint16_t *) (EE_LOG_1_START+(i-1)*0x02));
			log_end = eeprom_read_word((uint16_t *) (EE_LOG_1_START+(i)*0x02))-1;
			if (i == last_log_num) {	
				log_end = eeprom_read_word((uint16_t *) EE_LAST_LOG_PAGE);
			}	
			Serial.printf_P(PSTR("Log number %d,    start page %d,   end page %d\n"),
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
	last_log_num = eeprom_read_byte((uint8_t *) EE_LAST_LOG_NUM);
	if ((argc != 2) || (dump_log < 1) || (dump_log > last_log_num)) {
		Serial.printf_P(PSTR("bad log number\n"));
		return(-1);
	}

	dump_log_start = eeprom_read_word((uint16_t *) (EE_LOG_1_START+(dump_log-1)*0x02));
	dump_log_end = eeprom_read_word((uint16_t *) (EE_LOG_1_START+(dump_log)*0x02))-1;
	if (dump_log == last_log_num) {	
		dump_log_end = eeprom_read_word((uint16_t *) EE_LAST_LOG_PAGE);
	}	
	Serial.printf_P(PSTR("Dumping Log number %d,    start page %d,   end page %d\n"),
				  dump_log, dump_log_start, dump_log_end);
	Log_Read(dump_log_start, dump_log_end);
	Serial.printf_P(PSTR("Log read complete\n"));
}

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
	for(int i = 10 ; i > 0; i--) {
		Serial.printf_P(PSTR("ATTENTION - Erasing log in %d seconds.  Power off now to save log! \n"), i);
		delay(1000);
	}
	Serial.printf_P(PSTR("\nErasing log...\n"));
	for(int j = 1; j < 4001; j++)
		DataFlash.PageErase(j);
	eeprom_write_byte((uint8_t *)EE_LAST_LOG_NUM, 0);
	eeprom_write_byte((uint8_t *)EE_LAST_LOG_PAGE, 1);
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
#define TARG(_s)	if (!strcasecmp_P(argv[1].str, PSTR(#_s))) bits |= LOGBIT_ ## _s
		TARG(ATTITUDE_FAST);
		TARG(ATTITUDE_MED);
		TARG(GPS);
		TARG(PM);
		TARG(CTUN);
		TARG(NTUN);
		TARG(MODE);
		TARG(RAW);
		TARG(CMD);
#undef TARG
	}

	if (!strcasecmp_P(argv[0].str, PSTR("enable"))) {
		log_bitmask |= bits;
	} else {
		log_bitmask &= ~bits;
	}
	save_user_configs();		// XXX this is a bit heavyweight...

	return(0);
}

int8_t
process_logs(uint8_t argc, const Menu::arg *argv)
{
	log_menu.run();
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
	DataFlash.WriteByte(gyro_sat_count);
	DataFlash.WriteByte(adc_constraints);
	DataFlash.WriteByte(renorm_sqrt_count);
	DataFlash.WriteByte(renorm_blowup_count);
	DataFlash.WriteByte(gps_fix_count);
	DataFlash.WriteInt((int)(imu_health*1000));
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
	DataFlash.WriteLong(wp->alt);
	DataFlash.WriteLong(wp->lat);
	DataFlash.WriteLong(wp->lng);
	DataFlash.WriteByte(END_BYTE);
}

void Log_Write_Startup(byte type)
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_STARTUP_MSG);
	DataFlash.WriteByte(type);
	DataFlash.WriteByte(wp_total);
	DataFlash.WriteByte(END_BYTE);

	// create a location struct to hold the temp Waypoints for printing
	struct Location cmd = get_wp_with_index(0);
		Log_Write_Cmd(0, &cmd);
	
	for (int i=1; i<wp_total; i++){
		cmd = get_wp_with_index(i);
		Log_Write_Cmd(i, &cmd);
	}
}


// Write a control tuning packet. Total length : 22 bytes
void Log_Write_Control_Tuning()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_CONTROL_TUNING_MSG);
	DataFlash.WriteInt((int)(servo_out[CH_ROLL]));
	DataFlash.WriteInt((int)nav_roll);
	DataFlash.WriteInt((int)roll_sensor);
	DataFlash.WriteInt((int)(servo_out[CH_PITCH]));
	DataFlash.WriteInt((int)nav_pitch);
	DataFlash.WriteInt((int)pitch_sensor);
	DataFlash.WriteInt((int)(servo_out[CH_THROTTLE]));
	DataFlash.WriteInt((int)(servo_out[CH_RUDDER]));
	DataFlash.WriteInt((int)AN[4]);
	DataFlash.WriteByte(END_BYTE);
}

// Write a navigation tuning packet. Total length : 18 bytes
void Log_Write_Nav_Tuning()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_NAV_TUNING_MSG);
	DataFlash.WriteInt((uint16_t)yaw_sensor);
	DataFlash.WriteInt((int)wp_distance);
	DataFlash.WriteInt((uint16_t)target_bearing);
	DataFlash.WriteInt((uint16_t)nav_bearing);
	DataFlash.WriteInt(altitude_error);
	DataFlash.WriteInt((int)airspeed);
	DataFlash.WriteInt((int)(nav_gain_scaler*1000));
	DataFlash.WriteByte(END_BYTE);
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
void Log_Write_GPS(	long log_Time, long log_Lattitude, long log_Longitude, long log_mix_alt, long log_gps_alt, 
					long log_Ground_Speed, long log_Ground_Course, byte log_Fix, byte log_NumSats)
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_GPS_MSG);
	DataFlash.WriteLong(log_Time);
	DataFlash.WriteByte(log_Fix);
	DataFlash.WriteByte(log_NumSats);
	DataFlash.WriteLong(log_Lattitude);
	DataFlash.WriteLong(log_Longitude);
	DataFlash.WriteLong(log_mix_alt);
	DataFlash.WriteLong(log_gps_alt);
	DataFlash.WriteLong(log_Ground_Speed);
	DataFlash.WriteLong(log_Ground_Course);
	DataFlash.WriteByte(END_BYTE);
	DataFlash.WriteByte(END_BYTE);
}

// Write an raw accel/gyro data packet. Total length : 28 bytes
void Log_Write_Raw()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_RAW_MSG);
	for(int i=0;i<6;i++) 
		DataFlash.WriteLong((long)(AN[i]*1000.0));
	DataFlash.WriteByte(END_BYTE);
}

// Read an control tuning packet
void Log_Read_Control_Tuning()
{	
	float logvar;

	Serial.print("CTUN:");
	for (int y=1;y<10;y++) {
		logvar = DataFlash.ReadInt();
		if(y < 9) logvar = logvar/100.f;
		Serial.print(logvar);
		Serial.print(comma);
	}
	Serial.println(" ");
}

// Read a nav tuning packet
void Log_Read_Nav_Tuning()
{
	Serial.print("NTUN:");
	Serial.print((float)((uint16_t)DataFlash.ReadInt())/100.0);			// Yaw from IMU
	Serial.print(comma);
	Serial.print(DataFlash.ReadInt());									// wp_distance
	Serial.print(comma);
	Serial.print((float)((uint16_t)DataFlash.ReadInt())/100.0);			// target_bearing - Bearing to Target
	Serial.print(comma);
	Serial.print((float)((uint16_t)DataFlash.ReadInt())/100.0);			// nav_bearing - Bearing to steer
	Serial.print(comma);
	Serial.print((float)DataFlash.ReadInt()/100.0);						// Altitude error
	Serial.print(comma);
	Serial.print((float)DataFlash.ReadInt()/100.0);						// Airspeed 
	Serial.print(comma);
	Serial.print((float)DataFlash.ReadInt()/1000.0);					// nav_gain_scaler
	Serial.println(comma);
}

// Read a performance packet
void Log_Read_Performance()
{	
	long pm_time;
	int logvar;

	Serial.print("PM:");
	pm_time = DataFlash.ReadLong();
	Serial.print(pm_time);
	Serial.print(comma);
	for (int y=1;y<9;y++) {
		if(y<3 || y>7) logvar = DataFlash.ReadInt();
		else logvar = DataFlash.ReadByte();
		//if(y > 7) logvar = logvar/1000.f;
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

	Serial.print("CMD:");
	for(int i=1;i<4;i++) {
		logvarb = DataFlash.ReadByte();
		Serial.print(logvarb,DEC);
		Serial.print(comma);
	}
	for(int i=1;i<4;i++) {
		logvarl = DataFlash.ReadLong();
		Serial.print(logvarl,DEC);
		Serial.print(comma);
	}
	Serial.println(" ");
}

void Log_Read_Startup()
{
	byte logbyte = DataFlash.ReadByte();
	if (logbyte == TYPE_AIRSTART_MSG) 
		Serial.print("AIR START - ");
	else if (logbyte == TYPE_GROUNDSTART_MSG)
		Serial.print("GROUND START - ");
	else
		Serial.print("UNKNOWN STARTUP TYPE -");
	Serial.print(DataFlash.ReadByte(), DEC);
	Serial.println(" commands in memory");
}

// Read an attitude packet
void Log_Read_Attitude()
{	
	int log_roll;
	int log_pitch;
	uint16_t log_yaw;

	log_roll 	= DataFlash.ReadInt();
	log_pitch 	= DataFlash.ReadInt();
	log_yaw 	= (uint16_t)DataFlash.ReadInt();
	
	Serial.print("ATT:");
	Serial.print(log_roll);
	Serial.print(comma);
	Serial.print(log_pitch);
	Serial.print(comma);
	Serial.print(log_yaw);
	Serial.println();
}

// Read a mode packet
void Log_Read_Mode()
{	
	byte mode;

	mode = DataFlash.ReadByte();
	Serial.print("MOD:");
	switch (mode) {
		case 0:
			Serial.println("Manual");
			break;
		case 1:
			Serial.println("Stab");
			break;
		case 5:
			Serial.println("FBW_A");
			break;
		case 6:
			Serial.println("FBW_B");
			break;
		case 10:
			Serial.println("AUTO");
			break;
		case 11:
			Serial.println("RTL");
			break;
		case 12:
			Serial.println("Loiter");
			break;
		case 98:
			Serial.println("AS_COM");
			break;
		case 99:
			Serial.println("GS_COM");
			break;
		}
}

// Read a GPS packet
void Log_Read_GPS()
{

	Serial.print("GPS:");
	Serial.print(DataFlash.ReadLong());						// Time
	Serial.print(comma);
	Serial.print((int)DataFlash.ReadByte());				// Fix
	Serial.print(comma);
	Serial.print((int)DataFlash.ReadByte());				// Num of Sats
	Serial.print(comma);
	Serial.print((float)DataFlash.ReadLong()/t7, 7);			// Lattitude
	Serial.print(comma);
	Serial.print((float)DataFlash.ReadLong()/t7, 7);			// Longitude
	Serial.print(comma);
	Serial.print((float)DataFlash.ReadLong()/100.0);		// Baro/gps altitude mix
	Serial.print(comma);
	Serial.print((float)DataFlash.ReadLong()/100.0);		// GPS altitude
	Serial.print(comma);
	Serial.print((float)DataFlash.ReadLong()/100.0);		// Ground Speed
	Serial.print(comma);
	Serial.println((float)DataFlash.ReadLong()/100.0);		// Ground Course

}

// Read a raw accel/gyro packet
void Log_Read_Raw()
{	
	float logvar;
	Serial.print("RAW:");
	for (int y=0;y<6;y++) {
		logvar = DataFlash.ReadLong()/1000.f;
		Serial.print(logvar);
		Serial.print(comma);
	}
	Serial.println(" ");
}

// Read the DataFlash log memory : Packet Parser
void Log_Read(int start_page, int end_page)
{
	byte data;
	byte log_step=0;
	int packet_count=0; 
	int page = start_page;


	DataFlash.StartRead(start_page);
	while (page < end_page && page != -1)
		{
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
				if(data==LOG_ATTITUDE_MSG){
					Log_Read_Attitude();
					log_step++;
				
				}else if(data==LOG_MODE_MSG){
					Log_Read_Mode();
					log_step++;

				}else if(data==LOG_CONTROL_TUNING_MSG){
					Log_Read_Control_Tuning();
					log_step++;

				}else if(data==LOG_NAV_TUNING_MSG){
					Log_Read_Nav_Tuning();
					log_step++;
				
				}else if(data==LOG_PERFORMANCE_MSG){
					Log_Read_Performance();
					log_step++;
					
				}else if(data==LOG_RAW_MSG){
					Log_Read_Raw();
					log_step++;
					
				}else if(data==LOG_CMD_MSG){
					Log_Read_Cmd();
					log_step++;
					
				}else if(data==LOG_STARTUP_MSG){
					Log_Read_Startup();
					log_step++;
				}else {
					if(data==LOG_GPS_MSG){
						Log_Read_GPS();
						log_step++;
					} else {
						Serial.print("Error Reading Packet: ");
						Serial.print(packet_count); 
						log_step=0;	 // Restart, we have a problem...
					}
				}
				break;
			case 3:
				if(data==END_BYTE){
					 packet_count++;
				}else{
					Serial.print("Error Reading END_BYTE  ");
					Serial.println(data,DEC);
				}
				log_step=0;			// Restart sequence: new packet...
				break;
			}
		page = DataFlash.GetPage();
		}
	Serial.print("Number of packets read: ");
	Serial.println(packet_count);
}


