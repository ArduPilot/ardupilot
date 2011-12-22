// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED && CONFIG_LOGGING == LOGGING_SIMPLE

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
const struct Menu::command log_menu_commands[] PROGMEM = {
	{"dump",	dump_log},
	{"erase",	erase_logs},
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
		for(int count=1; count<=DF_LAST_PAGE; count++) {
			DataFlash.StartRead(count);
			Serial.printf_P(PSTR("DF page, log file #, log page: %d,\t"), count);
			Serial.printf_P(PSTR("%d,\t"), DataFlash.GetFileNumber());
			Serial.printf_P(PSTR("%d\n"), DataFlash.GetFilePage());
		}
		return(-1);
	} else if (dump_log <= 0) {
		Serial.printf_P(PSTR("dumping all\n"));
		Log_Read(1, DF_LAST_PAGE);
		return(-1);
	} else if ((argc != 2) || (dump_log <= (last_log_num - get_num_logs())) || (dump_log > last_log_num)) {
		Serial.printf_P(PSTR("bad log number\n"));
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
	Serial.printf_P(PSTR("\nErasing log...\n"));
	DataFlash.SetFileNumber(0xFFFF);
	for(int j = 1; j <= DF_LAST_PAGE; j++) {
		DataFlash.PageErase(j);
		DataFlash.StartWrite(j);		// We need this step to clean FileNumbers
		if(j%128 == 0) Serial.printf_P(PSTR("+"));
	}

	Serial.printf_P(PSTR("\nLog erased.\n"));
	DataFlash.FinishWrite();
    return 0;
}

static void
do_erase_logs(void (*delay_cb)(unsigned long))
{
	Serial.printf_P(PSTR("\nErasing log...\n"));
	DataFlash.SetFileNumber(0xFFFF);
	for(int j = 1; j <= DF_LAST_PAGE; j++) {
		DataFlash.PageErase(j);
		DataFlash.StartWrite(j);		// We need this step to clean FileNumbers
		if(j%128 == 0) Serial.printf_P(PSTR("+"));
        delay_cb(1);
	}

	Serial.printf_P(PSTR("\nLog erased.\n"));
	DataFlash.FinishWrite();
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
		DataFlash.StartRead(DF_LAST_PAGE);
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
			DataFlash.StartRead(DF_LAST_PAGE);
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
	if(start_page == DF_LAST_PAGE+1 || start_page == 0) start_page=1;
	end_page = find_last_page_of_log((uint16_t)log_num);
	if(end_page <= 0) end_page = start_page;
}

static bool check_wrapped(void)
{
	DataFlash.StartRead(DF_LAST_PAGE);
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
uint16_t top = DF_LAST_PAGE;
uint32_t look_hash;
uint32_t bottom_hash;
uint32_t top_hash;

		DataFlash.StartRead(bottom);
		bottom_hash = (long)DataFlash.GetFileNumber()<<16 | DataFlash.GetFilePage();

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
		top_hash = (long)DataFlash.GetFileNumber()<<16 | DataFlash.GetFilePage();
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
			top = DF_LAST_PAGE;
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



// Write an GPS packet. Total length : 30 bytes
static void Log_Write_GPS()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_GPS_MSG);

	DataFlash.WriteLong(g_gps->time);						// 1
	DataFlash.WriteByte(g_gps->num_sats);					// 2

	DataFlash.WriteLong(current_loc.lat);					// 3
	DataFlash.WriteLong(current_loc.lng);					// 4
	DataFlash.WriteLong(current_loc.alt);					// 5
	DataFlash.WriteLong(g_gps->altitude);					// 6

	DataFlash.WriteInt(g_gps->ground_speed);				// 7
	DataFlash.WriteLong(g_gps->ground_course);		// 8

	DataFlash.WriteByte(END_BYTE);
}

// Read a GPS packet
static void Log_Read_GPS()
{
	int32_t temp1 	= DataFlash.ReadLong();			// 1 time
	int8_t temp2 	= DataFlash.ReadByte();			// 2 sats
	float temp3 	= DataFlash.ReadLong() / t7;	// 3 lat
	float temp4 	= DataFlash.ReadLong() / t7;	// 4 lon
	float temp5 	= DataFlash.ReadLong() / 100.0;	// 5 gps alt
	float temp6 	= DataFlash.ReadLong() / 100.0;	// 6 sensor alt
	int16_t temp7 	= DataFlash.ReadInt();			// 7 ground speed
	int32_t temp8 	= DataFlash.ReadLong();// 8 ground course

							//  1   2    3      4     5      6      7    8
	Serial.printf_P(PSTR("GPS, %ld, %d, %4.7f, %4.7f, %4.4f, %4.4f, %d, %ld\n"),
							temp1,		// 1 time
							temp2,		// 2 sats
							temp3,		// 3 lat
							temp4,		// 4 lon
							temp5,		// 5 gps alt
							temp6,		// 6 sensor alt
							temp7,		// 7 ground speed
							temp8);		// 8 ground course
}


// Read the DataFlash log memory
static void Log_Read(int start_page, int end_page)
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
    	packet_count = Log_Read_Process(start_page, DF_LAST_PAGE);
    	packet_count += Log_Read_Process(1, end_page);
    } else {
    	packet_count = Log_Read_Process(start_page, end_page);
    }

	//Serial.printf_P(PSTR("Number of packets read: %d\n"), packet_count);
}

// Read the DataFlash log memory : Packet Parser
static int Log_Read_Process(int start_page, int end_page)
{
	byte data;
	byte log_step 		= 0;
	int page 			= start_page;
	int packet_count = 0;

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
				else{
					log_step = 0;
					Serial.println(".");
				}
				break;

			case 2:
				log_step = 0;
				Log_Read_GPS();
				break;
		}
		page = DataFlash.GetPage();
	}
	return packet_count;
}


static void Log_Write_Startup() {}
static void Log_Read_Startup() {}
static void Log_Write_Cmd(byte num, struct Location *wp) {}
static void Log_Write_Raw() {}
static void Log_Write_Mode(byte mode) {}
static void Log_Write_Current() {}
static void Log_Write_Attitude() {}
static void Log_Write_Data(int8_t _type, float _data){}
static void Log_Write_Data(int8_t _type, int32_t _data){}
#ifdef OPTFLOW_ENABLED
static void Log_Write_Optflow() {}
#endif
static void Log_Write_Nav_Tuning() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Motors() {}
static void Log_Write_Performance() {}

#endif
